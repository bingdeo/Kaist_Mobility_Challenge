#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel, AccelStamped
from ament_index_python.packages import get_package_share_directory
from smyd.Collision_Avoidance import CollisionAvoidance
import os
import numpy as np


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation

    # Simulator Euler-packed format:
    # orientation.x=roll, y=pitch, z=yaw (rad), w=1.0(dummy)
    if abs(q.w - 1.0) < 1e-3 and (abs(q.x) > 1.0 or abs(q.y) > 1.0 or abs(q.z) > 1.0):
        return float(q.z)

    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)

class SteeringPID:
    def __init__(self, Kp, Ki, Kd, i_limit=0.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.i_limit = i_limit
        self.e_prev = 0.0
        self.e_int = 0.0

    def reset(self):
        self.e_prev = 0.0
        self.e_int = 0.0

    def update(self, e, dt):
        if dt <= 0.0:
            return 0.0

        # Integral
        self.e_int += e * dt
        self.e_int = max(-self.i_limit, min(self.i_limit, self.e_int))

        # Derivative
        de = (e - self.e_prev) / dt
        self.e_prev = e

        # PID output (steering angle [rad])
        delta_fb = (
            self.Kp * e +
            self.Ki * self.e_int +
            self.Kd * de
        )
        return delta_fb

class P12Follower(Node):
    def __init__(self):
        super().__init__("p1_2_cav1")

        self.declare_parameter("waypoints_json", "")
        self.declare_parameter("v_ref", 2.0)
        self.declare_parameter("w_max", 5.0)

        # V2V (state only)
        self.declare_parameter("my_share_topic", "/cav1/v2v_state")
        self.declare_parameter("peer_share_topic", "/peer/cav2/v2v_state")

        # zones_json도 남겨서 호환되게 처리
        self.declare_parameter("conflict_map_json", "")
        self.declare_parameter("zones_json", "")  # fallback

        # sampling + CROSS_POINT danger length
        self.declare_parameter("ds_per_point", 0.01)          # 0.01m
        self.declare_parameter("point_danger_points", 30)     # 0.3m 점유(필요시 조절)

        self.declare_parameter("search_window", 600)
        self.declare_parameter("peer_timeout", 0.8)  # seconds

        self.declare_parameter("tie_eta_sec", 1.0)     # |Δeta| <= tie => cav1 우선
        self.declare_parameter("eta_gate_sec", 5.0)     # 너무 멀리서 감속 방지
        self.declare_parameter("yield_ratio", 0.5)     # 감속 비율 (v_cmd *= yield_ratio)
        self.declare_parameter("v_min", 0.20)    
        # 정지는 안 됨 (최저 속도)

        wp = self.get_parameter("waypoints_json").value
        if not wp:
            raise RuntimeError("waypoints_json is empty")

        self.v_ref = float(self.get_parameter("v_ref").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.win = int(self.get_parameter("search_window").value)
        self.peer_timeout = float(self.get_parameter("peer_timeout").value)

        self.ds_per_point = float(self.get_parameter("ds_per_point").value)
        self.point_danger_points = int(self.get_parameter("point_danger_points").value)

        self.tie_eta_sec = float(self.get_parameter("tie_eta_sec").value)
        self.eta_gate_sec = float(self.get_parameter("eta_gate_sec").value)
        self.yield_ratio = float(self.get_parameter("yield_ratio").value)
        self.v_min = float(self.get_parameter("v_min").value)

        self.is_cav1 = True  
        self.lap = 0          # 내 바퀴수   

        self.pts = self.load_json(wp)
        if len(self.pts) < 5:
            raise RuntimeError("waypoints too short")
        self.N = len(self.pts)

        # unwrapped progress index (monotonic)
        self.idx_u = 0
        self.allow_back = 0

        # lap timing
        self.start_time = None
        self.prev_lap = 0

        # ---- PID params ----

        # PID gains
        self.declare_parameter("Kp", 0.2)
        self.declare_parameter("Ki", 0.0)
        self.declare_parameter("Kd", 0.0)


        self.pid = SteeringPID(
            Kp=float(self.get_parameter("Kp").value),
            Ki=float(self.get_parameter("Ki").value),
            Kd=float(self.get_parameter("Kd").value),
            i_limit=0.3
        )
      
        # PID state
        self.e_prev = 0.0
        self.e_int = 0.0

        log_path = "/tmp/pid_log_p1_2_cav1.csv"
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        # pkg = get_package_share_directory("cav")
        # log_path = os.path.join(pkg, "log", "pid_log.csv")
        
        self.log_f = open("/tmp/pid_log_p1_2_cav1.csv", "w")
        self.log_f.write("t,y_r,w,w_pp,w_pid,P,I,D\n")
        self.t0 = self.get_clock().now()

        # control I/O (simulator interface)
        self.sub = self.create_subscription(PoseStamped, "/Ego_pose", self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Accel, "/Accel", qos_profile_sensor_data)

        # V2V state pub/sub
        my_topic = self.get_parameter("my_share_topic").value
        peer_topic = self.get_parameter("peer_share_topic").value
        self.share_pub = self.create_publisher(AccelStamped, my_topic, qos_profile_sensor_data)
        self.peer_sub = self.create_subscription(AccelStamped, peer_topic, self.cb_peer, qos_profile_sensor_data)
        self.peer_state = None

        # load conflict map (cav1 = path1)
        self.path_key = "path1"
        cm_path = self.get_parameter("conflict_map_json").value
        if not cm_path:
            cm_path = self.get_parameter("zones_json").value  # fallback
        if not cm_path:
            raise RuntimeError("conflict_map_json (or zones_json) is empty")

        self.zones = self.load_conflict_map(cm_path, self.path_key)

        self.get_logger().info(f"Loaded pts={len(self.pts)} file={wp}")
        self.get_logger().info(f"Loaded conflict zones={len(self.zones)} file={cm_path}")
        
        # Collision Avoidance module
        self.ca = CollisionAvoidance(self)
        
        # Zone tracking for logging
        self.prev_zone_id = -1
        self.prev_in_danger = 0
        self.prev_in_conflict = False
        self.zone_log_lap = {}  # Track which lap we logged for each zone

    def load_json(self, path):
        with open(path, "r") as f:
            data = json.load(f)

        X = data.get("X", data.get("x"))
        Y = data.get("Y", data.get("y"))
        if X is None or Y is None:
            raise RuntimeError("JSON must contain X,Y (or x,y)")
        if len(X) != len(Y):
            raise RuntimeError("lengths mismatch")

        return [(float(x), float(y)) for x, y in zip(X, Y)]

    def load_conflict_map(self, path, path_key):
        with open(path, "r") as f:
            data = json.load(f)

        zones = []
        for z in data.get("zones", []):
            zid = int(z["zone_id"])
            kind = z.get("kind", "")

            p = z[path_key]
            ms = int(p["monitor_start_idx"])
            cs = int(p["conflict_start_idx"])
            ce = p.get("conflict_end_idx", None)
            ce = int(ce) if ce is not None else None

            # CROSS_POINT는 end가 없으니, 점유(danger) 길이를 짧게라도 부여
            if ce is None:
                ce = (cs + self.point_danger_points) % self.N
            else:
                ce = ce % self.N

            zones.append({
                "zone_id": zid,
                "kind": kind,
                "monitor_start": ms % self.N,
                "conflict_start": cs % self.N,
                "conflict_end": ce,
            })
        return zones

    def nearest_index(self, x, y):
        back = 50
        fwd = self.win

        best_j = self.idx_u
        best_d2 = 1e18

        for j in range(self.idx_u - back, self.idx_u + fwd + 1):
            px, py = self.pts[j % self.N]
            dx = px - x
            dy = py - y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_j = j

        if best_j < self.idx_u - self.allow_back:
            best_j = self.idx_u

        self.idx_u = best_j
        return best_j

    def publish(self, v, w):
        if w > self.w_max:
            w = self.w_max
        elif w < -self.w_max:
            w = -self.w_max

        msg = Accel()
        msg.linear.x = float(v)
        msg.angular.x = float(w)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    # ---------- V2V ----------
    def cb_peer(self, msg: AccelStamped):
        a = msg.accel
        self.peer_state = {
            "zone": int(round(a.linear.x)),
            "in_danger": int(round(a.linear.y)),  # 0/1
            "eta": float(a.linear.z),
            "lap": int(round(a.angular.x)),
            "x": float(a.angular.y),  # x 좌표
            "y": float(a.angular.z),  # y 좌표
            "t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

    def peer_is_fresh(self):
        if self.peer_state is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self.peer_state["t"]) < self.peer_timeout

    def publish_v2v(self, zone_id, in_danger, eta, lap, x, y):
        m = AccelStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.accel.linear.x = float(zone_id)
        m.accel.linear.y = float(in_danger)  # 0/1
        m.accel.linear.z = float(eta)
        m.accel.angular.x = float(lap)       # lap 공유
        m.accel.angular.y = float(x)         # x 좌표
        m.accel.angular.z = float(y)         # y 좌표
        self.share_pub.publish(m)

    # ---------- zone logic ----------
    def _in_range(self, idx, a, b):
        # inclusive range, wrap-around supported
        if a <= b:
            return a <= idx <= b
        return (idx >= a) or (idx <= b)

    def compute_zone_state(self, cur_idx_mod, v_used):
        """
        현재 위치에서 가장 우선순위 높은 zone의 상태 반환
        in_danger는 여기서 설정하지 않음 (호출한 곳에서 peer와 비교하여 설정)
        """
        best = None  # (priority, eta, zone_id, in_conflict)
        v = max(float(v_used), 0.05)

        for z in self.zones:
            zid = z["zone_id"]
            ms = z["monitor_start"]
            cs = z["conflict_start"]
            ce = z["conflict_end"]

            # conflict zone 내부 (점유 중) - 우선순위 0
            if self._in_range(cur_idx_mod, cs, ce):
                cand = (0, 0.0, zid, 1)  # in_conflict=1
                if best is None or cand < best:
                    best = cand
                continue

            # monitoring zone (접근 중): monitor_start ~ conflict_start 직전
            end_monitor = (cs - 1) % self.N
            if self._in_range(cur_idx_mod, ms, end_monitor):
                remain_pts = (cs - cur_idx_mod) % self.N
                dist = remain_pts * self.ds_per_point
                eta = dist / v
                cand = (1, eta, zid, 0)  # in_conflict=0
                if best is None or cand < best:
                    best = cand

        if best is None:
            return -1, 0, 1e9

        _, eta, zid, in_conflict = best
        return zid, in_conflict, float(eta)

    def log_zone_status(self, zone_id, in_danger, in_conflict, lap_now):
        """Log zone entry/exit/danger status with colors"""
        YELLOW = '\033[93m'
        RED = '\033[91m'
        BLUE = '\033[94m'
        RESET = '\033[0m'
        
        # Zone entry (monitoring zone)
        if zone_id >= 0 and zone_id != self.prev_zone_id:
            log_key = (zone_id, lap_now, 'entry')
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{YELLOW}zone_id:{zone_id}에 접근 중{RESET}")
                self.zone_log_lap[log_key] = True
        
        # Collision danger (both in same zone)
        if in_danger == 1 and self.prev_in_danger == 0 and zone_id >= 0:
            log_key = (zone_id, lap_now, 'danger')
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{RED}zone_id:{zone_id}에서 충돌위험(in_danger) 발생{RESET}")
                self.zone_log_lap[log_key] = True
        
        # Exit from conflict zone
        if self.prev_in_conflict and not in_conflict and self.prev_zone_id >= 0:
            log_key = (self.prev_zone_id, lap_now, 'exit_conflict')
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{YELLOW}zone_id:{self.prev_zone_id}에서 나옴{RESET}")
                self.zone_log_lap[log_key] = True
        
        # Complete pass through zone (zone changed)
        if zone_id != self.prev_zone_id and self.prev_zone_id >= 0:
            log_key = (self.prev_zone_id, lap_now, 'pass')
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{BLUE}zone_id:{self.prev_zone_id}에서 통과함{RESET}")
                self.zone_log_lap[log_key] = True
        
        self.prev_zone_id = zone_id
        self.prev_in_danger = in_danger
        self.prev_in_conflict = in_conflict
    
    # ====== (3) 우선순위 감속 로직: "하나의 함수"로 묶어서 추가 ======
    def priority_speed(self, v_cmd, my_zone, my_in_danger, my_eta, my_lap):
        # peer 없거나 zone 없으면 그대로
        if my_zone < 0 or (not self.peer_is_fresh()) or self.peer_state is None:
            return v_cmd

        pz = int(self.peer_state["zone"])
        p_in = int(self.peer_state["in_danger"])
        p_eta = float(self.peer_state["eta"])
        p_lap = int(self.peer_state.get("lap", -999))

        # 같은 zone + 같은 lap일 때만 비교(랩 반복으로 잘못 양보 방지)
        if pz != my_zone or p_lap != my_lap:
            return v_cmd

        # 너무 멀리서 감속 방지 (둘 다 monitor일 때만 적용)
        if (my_in_danger == 0 and p_in == 0 and min(my_eta, p_eta) > self.eta_gate_sec):
            return v_cmd

        d = my_eta - p_eta
        ad = abs(d)

        # tie-breaker: |Δeta| 이내면 cav1 우선
        if ad <= self.tie_eta_sec:
            need_yield = (not self.is_cav1)  # cav2만 감속
        else:
            # eta가 더 큰(늦는) 쪽이 감속
            need_yield = (d > 0.0)

        if need_yield:
            v_cmd = max(self.v_min, v_cmd * self.yield_ratio)
        return v_cmd
    # ---------- main callback ----------
    def cb(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_pose(msg)

        i = self.nearest_index(x, y)
        idx_mod = i % self.N

        # ===== Lap timing =====
        now_time = self.get_clock().now()

        # 주행 시작 시점 기록
        if self.start_time is None:
            self.start_time = now_time
            self.prev_lap = i // self.N
        else:
            current_lap = i // self.N
            if current_lap > self.prev_lap:
                lap_time = (now_time - self.start_time).nanoseconds * 1e-9
                self.get_logger().info(
                    f"[Lap {current_lap}] {lap_time:.2f} s"
                )
                self.start_time = now_time
                self.prev_lap = current_lap

        # lap counter (idx_u is unwrapped)
        lap_now = int(i // self.N)
        if lap_now != self.lap:
            self.lap = lap_now
            self.get_logger().info(f"Lap = {self.lap}")

        # publish my V2V state (zone_id, in_danger, eta)
        # compute_zone_state는 in_conflict 반환 (conflict zone 내부 여부)
        zone_id, in_conflict, eta = self.compute_zone_state(idx_mod, self.v_ref)
        
        # in_danger는 peer와 같은 zone에 있을 때만 1
        in_danger = 0
        if zone_id >= 0 and self.peer_is_fresh() and self.peer_state:
            p_zone = int(self.peer_state.get("zone", -1))
            if p_zone == zone_id:
                in_danger = 1
        
        self.publish_v2v(zone_id, in_danger, eta, self.lap, x, y)
        
        # Log zone status
        self.log_zone_status(zone_id, in_danger, in_conflict, self.lap)

        # v_cmd: priority logic로 감속(정지 없음)
        v_cmd = self.v_ref
        
        # Collision avoidance when in danger
        if in_danger == 1 and self.peer_is_fresh() and self.peer_state:
            peer_eta = float(self.peer_state.get("eta", 1e9))
            peer_lap = int(self.peer_state.get("lap", 0))
            peer_x = float(self.peer_state.get("x", 0.0))
            peer_y = float(self.peer_state.get("y", 0.0))
            v_cmd = self.ca.avoid_collision(zone_id, v_cmd, eta, peer_eta, self.lap, peer_lap, x, y, peer_x, peer_y, self.is_cav1)
        else:
            v_cmd = self.priority_speed(v_cmd, zone_id, in_danger, eta, self.lap)

        # lookahead
        tgt = i + 60

        for _ in range(10):
            tx, ty = self.pts[tgt % self.N]
            dx = tx - x
            dy = ty - y
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            x_r = cos_y*dx + sin_y*dy
            if x_r > 0.05:
                break
            tgt += 1
        else:
            tx, ty = self.pts[tgt % self.N]
            dx = tx - x
            dy = ty - y
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            x_r = cos_y*dx + sin_y*dy

        now = self.get_clock().now()
        dt = (now - self.t_prev).nanoseconds * 1e-9
        self.t_prev = now
        # Pure Pursuit: w = v * kappa, kappa = 2*y_r / L^2
        y_r = -sin_y*dx + cos_y*dy
        x_r =  cos_y*dx + sin_y*dy

        Ld = math.hypot(x_r, y_r)
        if Ld < 1e-3:
            self.publish(0.0, 0.0)
            return
        
        # ===== Pure Pursuit =====
        alpha = math.atan2(y_r, x_r)
        kappa = 2.0 * math.sin(alpha) / Ld

        L_wb = 0.30   # wheelbase [m]
        delta_ff = math.atan(L_wb * kappa)

        # ===== PID feedback (steering angle) =====
        delta_fb = self.pid.update(y_r, dt)

        # ===== Steering angle synthesis =====
        delta = delta_ff + delta_fb

        # ===== Saturation (조향각 제한) =====
        delta_max = math.radians(30.0)
        delta = max(-delta_max, min(delta_max, delta))

        # ===== Steering → yaw rate =====
        w = self.v_ref / L_wb * math.tan(delta)

        # PID csv params
        t = (now_time - self.t0).nanoseconds * 1e-9
        w_pp = self.v_ref / L_wb * math.tan(delta_ff)
        w_pid = self.v_ref / L_wb * math.tan(delta_fb)
        e = y_r

        P = self.pid.Kp * e
        I = self.pid.Ki * self.e_int
        D = self.pid.Kd * ((e - self.pid.e_prev) / dt if dt > 0 else 0.0)

        self.log_f.write(
            f"{t},{y_r},{w},{w_pp},{w_pid},{P},{I},{D}\n"
        )

        self.publish(self.v_ref, w)


def main():
    rclpy.init()
    node = P12Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
