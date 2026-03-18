#!/usr/bin/env python3
import math
import json
import rclpy
import os

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from pkg_p3.msg import V2VState
from ament_index_python.packages import get_package_share_directory
from pkg_p3.Collision_Avoidance3 import CollisionAvoidance


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation

    # Simulator Euler-packed format:
    # If w?1 and x/y/z are out-of-range, z is used as yaw directly.
    if abs(q.w - 1.0) < 1e-3 and (abs(q.x) > 1.0 or abs(q.y) > 1.0 or abs(q.z) > 1.0):
        return float(q.z)

    # Normal quaternion -> yaw
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
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

        self.e_int += e * dt
        self.e_int = max(-self.i_limit, min(self.i_limit, self.e_int))

        de = (e - self.e_prev) / dt
        self.e_prev = e

        # PID output (steering angle [rad])
        delta_fb = (
            self.Kp * e +
            self.Ki * self.e_int +
            self.Kd * de
        )
        return delta_fb


class P3Follower(Node):
    def __init__(self):
        domain = int(os.environ.get("ROS_DOMAIN_ID", "1")) # if this is other CAV, then might should be fixed
        self.vid = domain

        super().__init__(f"p3_cav{self.vid}")
        self.get_logger().info(f"Started P3Follower for CAV {self.vid} (Domain {domain})")

        pkg_p3_share = get_package_share_directory("pkg_p3")

        # Waypoints: 3_cav1.json, 3_cav2.json, ...
        default_wp = os.path.join(pkg_p3_share, "waypoints", f"3_cav{self.vid}.json")
        default_cm = os.path.join(pkg_p3_share, "config", "3_zone_database.json")

        self.declare_parameter("waypoints_json", default_wp)
        self.declare_parameter("conflict_map_json", default_cm)

        # V2V state topics
        self.my_share_topic = f"/p3_v2v/cav{self.vid}/state"
        self.peer_share_topic = "/p3_v2v/peer/state"

        # Control / logic parameters
        self.declare_parameter("v_ref", 1.0)
        self.declare_parameter("w_max", 5.0)
        self.declare_parameter("ds_per_point", 0.01)          # meters per waypoint index
        self.declare_parameter("point_danger_points", 30)     # conflict end extension for cross-point zones
        self.declare_parameter("lookahead_idx", 80)
        self.declare_parameter("search_window", 600)
        self.declare_parameter("peer_timeout", 0.8)           # seconds
        self.declare_parameter("tie_eta_sec", 1.0)            # |eta diff| <= tie => cav1 priority (legacy)
        self.declare_parameter("eta_gate_sec", 5.0)           # gate window (legacy)
        self.declare_parameter("yield_ratio", 0.5)            # yielding ratio (legacy)
        self.declare_parameter("v_min", 0.20)                 # minimum moving speed

        wp = self.get_parameter("waypoints_json").value
        if not wp:
            raise RuntimeError("waypoints_json is empty")

        self.pts = self.load_json(wp)
        if len(self.pts) < 5:
            raise RuntimeError("waypoints too short")
        self.N = len(self.pts)

        self.v_ref = float(self.get_parameter("v_ref").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.lookahead_idx = int(self.get_parameter("lookahead_idx").value)
        self.win = int(self.get_parameter("search_window").value)
        self.peer_timeout = float(self.get_parameter("peer_timeout").value)
        self.ds_per_point = float(self.get_parameter("ds_per_point").value)
        self.point_danger_points = int(self.get_parameter("point_danger_points").value)
        self.tie_eta_sec = float(self.get_parameter("tie_eta_sec").value)
        self.eta_gate_sec = float(self.get_parameter("eta_gate_sec").value)
        self.yield_ratio = float(self.get_parameter("yield_ratio").value)
        self.v_min = float(self.get_parameter("v_min").value)

        # Progress / lap tracking
        self.lap = 0
        self.idx_u = 0  # unwrapped progress index (monotonic)
        self.allow_back = 0

        # Zone-flag logic
        self.my_flag = 0
        self.flag_threshold = 0.2

        # Lap timing
        self.start_time = None
        self.prev_lap = 0

        # Flag targets (zone 7 / zone 8) by vehicle id
        self.flag_target = None
        if self.vid == 2:
            self.flag_target = (1.55833333333333, 1.499460188019)
        elif self.vid == 3:
            self.flag_target = (1.30833333333333, 1.499460188019)
        elif self.vid == 1:
            self.flag_target = (0.217, -0.108333)
        elif self.vid == 4:
            self.flag_target = (0.217, -0.358333333333333)

        # PID params
        self.declare_parameter("Kp", 0.25)
        self.declare_parameter("Ki", 0.0)
        self.declare_parameter("Kd", 0.02)

        self.pid = SteeringPID(
            Kp=float(self.get_parameter("Kp").value),
            Ki=float(self.get_parameter("Ki").value),
            Kd=float(self.get_parameter("Kd").value),
            i_limit=0.3,
        )

        # Simulator I/O
        self.sub = self.create_subscription(PoseStamped, "/Ego_pose", self.cb, qos_profile_sensor_data)

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, "/cmd_vel", cmd_qos)

        # V2V pub/sub
        self.share_pub = self.create_publisher(V2VState, self.my_share_topic, 10)
        self.peer_sub = self.create_subscription(V2VState, self.peer_share_topic, self.cb_peer, 10)
        self.peer_states = {}

        # Load conflict map for this path
        self.path_key = f"path{self.vid}"
        cm_path = self.get_parameter("conflict_map_json").value
        if not cm_path:
            raise RuntimeError("conflict_map_json is empty")

        self.zones = self.load_conflict_map(cm_path, self.path_key)

        self.get_logger().info(f"Loaded pts={len(self.pts)} file={wp}")
        self.get_logger().info(f"Loaded conflict zones={len(self.zones)} file={cm_path}")

        # Collision avoidance module (expects hv_data entries as 4-tuples: (x, y, yaw, v))
        self.ca = CollisionAvoidance(self)

        # Zone logging state
        self.prev_zone_id = -1
        self.prev_in_danger = 0
        self.prev_in_conflict = False
        self.zone_log_lap = {}  # (zone_id, lap, tag) -> True

        self.t_prev = self.get_clock().now()

        # HV tracking:
        # hv_data: hv_id -> (x, y, yaw, v_front)  (v_front may be None)
        # hv_state: hv_id -> {"x","y","t","x_prev","y_prev","t_prev"}
        self.hv_data = {}
        self.hv_state = {}

        for hv_id in range(19, 21):
            topic = f"/HV_{hv_id:02d}"  # e.g., /HV_19
            self.create_subscription(PoseStamped, topic, self.make_hv_cb(hv_id), qos_profile_sensor_data)

    # ----------------------------
    # HV callbacks / speed estimate
    # ----------------------------
    def make_hv_cb(self, hv_id):
        def hv_cb(msg: PoseStamped):
            # Update raw HV state for speed estimation
            self.cb_hv(msg, hv_id)

            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            yaw = yaw_from_pose(msg)

            # Preserve previous speed if we have it
            v_prev = None
            if hv_id in self.hv_data and len(self.hv_data[hv_id]) >= 4:
                v_prev = self.hv_data[hv_id][3]

            # Store as a 4-tuple (x, y, yaw, v_front)
            self.hv_data[hv_id] = (x, y, yaw, v_prev)

        return hv_cb

    def cb_hv(self, msg: PoseStamped, hv_id: int):
        t = float(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if t <= 0.0:
            # Fallback if stamp is not valid
            t = self.get_clock().now().nanoseconds * 1e-9

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)

        if hv_id in self.hv_state:
            prev = self.hv_state[hv_id]
            self.hv_state[hv_id] = {
                "x": x, "y": y, "t": t,
                "x_prev": prev["x"], "y_prev": prev["y"], "t_prev": prev["t"],
            }
        else:
            self.hv_state[hv_id] = {"x": x, "y": y, "t": t}

    def _compute_front_hv_speed(self, st, hv_yaw):
        # Compute HV speed projected onto HV heading (front direction).
        if "x_prev" not in st or "y_prev" not in st or "t_prev" not in st:
            st["x_prev"] = st["x"]
            st["y_prev"] = st["y"]
            st["t_prev"] = st["t"]
            return None

        dt = st["t"] - st["t_prev"]
        if dt <= 1e-3:
            return None

        dx = st["x"] - st["x_prev"]
        dy = st["y"] - st["y_prev"]

        cos_y = math.cos(hv_yaw)
        sin_y = math.sin(hv_yaw)

        v_hv_front = (cos_y * dx + sin_y * dy) / dt

        st["x_prev"] = st["x"]
        st["y_prev"] = st["y"]
        st["t_prev"] = st["t"]

        return v_hv_front

    # ----------------------------
    # Data loading
    # ----------------------------
    def load_json(self, path):
        with open(path, "r") as f:
            data = json.load(f)

        X = data.get("X", data.get("x"))
        Y = data.get("Y", data.get("y"))
        if X is None or Y is None:
            raise RuntimeError("JSON must contain X,Y (or x,y)")
        if len(X) != len(Y):
            raise RuntimeError("length mismatch between X and Y")

        return [(float(x), float(y)) for x, y in zip(X, Y)]

    def load_conflict_map(self, path, path_key):
        with open(path, "r") as f:
            data = json.load(f)

        zones = []
        for z in data:
            if path_key not in z:
                # Skip zones that do not define this path
                continue

            zid = int(z["zone_id"])
            kind = z.get("kind", "")

            p = z[path_key]
            ms = int(p["monitor_start_idx"])
            cs = int(p["conflict_start_idx"])
            ce = p.get("conflict_end_idx", None)
            ce = int(ce) if ce is not None else None

            # If end is missing (cross-point), extend a short segment as conflict.
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

    # ----------------------------
    # Path tracking
    # ----------------------------
    def nearest_index(self, x, y):
        back = 50
        fwd = self.win

        best_j = self.idx_u
        best_d2 = 1e18

        for j in range(self.idx_u - back, self.idx_u + fwd + 1):
            px, py = self.pts[j % self.N]
            dx = px - x
            dy = py - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_j = j

        if best_j < self.idx_u - self.allow_back:
            best_j = self.idx_u

        self.idx_u = best_j
        return best_j

    def publish(self, v, w):
        w = max(-self.w_max, min(self.w_max, float(w)))

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    # ----------------------------
    # V2V
    # ----------------------------
    def cb_peer(self, msg: V2VState):
        pid = msg.vehicle_id
        if pid == self.vid:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        self.peer_states[pid] = {
            "zone": int(msg.zone_id),
            "in_danger": int(msg.in_danger),
            "eta": float(msg.eta),
            "velocity": float(msg.velocity),
            "lap": int(msg.lap),
            "x": float(msg.x),
            "y": float(msg.y),
            "flag": int(msg.flag),
            "t": now_sec,
        }

    def publish_v2v(self, zone_id, in_danger, eta, lap, x, y, flag, velocity):
        m = V2VState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.vehicle_id = int(self.vid)
        m.zone_id = int(zone_id)
        m.in_danger = int(in_danger)
        m.eta = float(eta)
        m.lap = int(lap)
        m.x = float(x)
        m.y = float(y)
        m.flag = int(flag)
        m.velocity = float(velocity)
        self.share_pub.publish(m)

    # ----------------------------
    # Zone logic
    # ----------------------------
    def _in_range(self, idx, a, b):
        # Inclusive range with wrap-around support
        if a <= b:
            return a <= idx <= b
        return (idx >= a) or (idx <= b)

    def compute_zone_state(self, cur_idx_mod, v_used):
        # Choose the highest priority zone:
        # - in_conflict zones win first
        # - then smallest ETA among monitored zones
        best = None  # (priority, eta, zone_id, in_conflict)
        v = max(float(v_used), 0.05)

        for z in self.zones:
            zid = z["zone_id"]
            ms = z["monitor_start"]
            cs = z["conflict_start"]
            ce = z["conflict_end"]

            if self._in_range(cur_idx_mod, cs, ce):
                cand = (0, 0.0, zid, 1)
                if best is None or cand < best:
                    best = cand
                continue

            end_monitor = (cs - 1) % self.N
            if self._in_range(cur_idx_mod, ms, end_monitor):
                remain_pts = (cs - cur_idx_mod) % self.N
                dist = remain_pts * self.ds_per_point
                eta = dist / v
                cand = (1, eta, zid, 0)
                if best is None or cand < best:
                    best = cand

        if best is None:
            return -1, 0, 1e9

        _, eta, zid, in_conflict = best
        return zid, in_conflict, float(eta)

    def log_zone_status(self, zone_id, in_danger, in_conflict, lap_now):
        # ANSI colors (may or may not show depending on logger)
        YELLOW = "\033[93m"
        RED = "\033[91m"
        BLUE = "\033[94m"
        GREEN = "\033[92m"
        RESET = "\033[0m"

        # Zone entry
        if zone_id >= 0 and zone_id != self.prev_zone_id:
            log_key = (zone_id, lap_now, "entry")
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{YELLOW}zone_id:{zone_id} entered monitoring range{RESET}")
                self.zone_log_lap[log_key] = True

        # Danger onset
        if in_danger == 1 and self.prev_in_danger == 0 and zone_id >= 0:
            log_key = (zone_id, lap_now, "danger")
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{RED}zone_id:{zone_id} danger (peer in same zone){RESET}")
                self.zone_log_lap[log_key] = True

        # Exit conflict
        if self.prev_in_conflict and not in_conflict and self.prev_zone_id >= 0:
            log_key = (self.prev_zone_id, lap_now, "exit_conflict")
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{YELLOW}zone_id:{self.prev_zone_id} exited conflict segment{RESET}")
                self.zone_log_lap[log_key] = True

        # Passed zone
        if zone_id != self.prev_zone_id and self.prev_zone_id >= 0:
            log_key = (self.prev_zone_id, lap_now, "pass")
            if log_key not in self.zone_log_lap:
                self.get_logger().info(f"{BLUE}zone_id:{self.prev_zone_id} passed{RESET}")
                self.zone_log_lap[log_key] = True

        # Reset flag when leaving zone 7 / 8 (per vehicle group)
        if self.vid in (2, 3):
            if self.prev_zone_id == 7 and zone_id != 7:
                self.my_flag = 0
                self.get_logger().info(f"{GREEN}Flag reset (left zone 7){RESET}")
        if self.vid in (1, 4):
            if self.prev_zone_id == 8 and zone_id != 8:
                self.my_flag = 0
                self.get_logger().info(f"{GREEN}Flag reset (left zone 8){RESET}")

        self.prev_zone_id = zone_id
        self.prev_in_danger = in_danger
        self.prev_in_conflict = in_conflict

    # ----------------------------
    # Main control callback
    # ----------------------------
    def cb(self, msg: PoseStamped):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        dt = (now - self.t_prev).nanoseconds * 1e-9
        self.t_prev = now

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_pose(msg)

        # Update HV speed estimates using HV yaw (not Ego yaw)
        for hv_id in (19, 20):
            if hv_id not in self.hv_state:
                continue
            if hv_id not in self.hv_data or len(self.hv_data[hv_id]) < 3:
                continue

            hx, hy, hv_yaw, v_old = self.hv_data[hv_id]
            v_hv = self._compute_front_hv_speed(self.hv_state[hv_id], hv_yaw)
            if v_hv is not None:
                self.hv_data[hv_id] = (hx, hy, hv_yaw, v_hv)
            else:
                self.hv_data[hv_id] = (hx, hy, hv_yaw, v_old)

        # Find closest waypoint index
        i = self.nearest_index(x, y)
        idx_mod = i % self.N

        # Lap tracking
        current_lap = int(i // self.N)
        if self.start_time is None:
            self.start_time = now
            self.prev_lap = current_lap
        else:
            if current_lap > self.prev_lap:
                lap_time = (now - self.start_time).nanoseconds * 1e-9
                self.get_logger().info(f"[Lap {current_lap}] {lap_time:.2f} s")
                self.start_time = now
                self.prev_lap = current_lap

        # Keep lap value consistent for V2V / avoidance
        self.lap = current_lap

        # Flag detection (only if configured)
        if self.flag_target is not None:
            dist_to_flag = math.sqrt((x - self.flag_target[0]) ** 2 + (y - self.flag_target[1]) ** 2)
            if self.vid in (2, 3):
                if dist_to_flag < self.flag_threshold and (y - self.flag_target[1]) > 0:
                    self.my_flag = 1
            if self.vid in (1, 4):
                if dist_to_flag < self.flag_threshold and (x - self.flag_target[0]) < 0:
                    self.my_flag = 1

        # Compute current zone state (use nominal speed for ETA estimate)
        zone_id, in_conflict, eta = self.compute_zone_state(idx_mod, self.v_ref)
        in_danger = 0
        swapped = False

        def get_peer_zone(pid):
            p_s = self.peer_states.get(pid, None)
            if p_s is None:
                return -999
            if (now_sec - p_s["t"]) < self.peer_timeout:
                return int(p_s.get("zone", -999))
            return -999

        # Zone remapping rules (legacy behavior)
        if self.vid == 2 and zone_id == 4:
            if get_peer_zone(1) == 1:
                zone_id = 1
                swapped = True

        if self.vid == 2 and zone_id == 2:
            if get_peer_zone(1) == 3:
                zone_id = 3
                swapped = True

        if self.vid == 3 and zone_id == 5:
            if get_peer_zone(1) != 5 and get_peer_zone(2) == 6:
                zone_id = 6
                swapped = True

        if self.vid == 4 and zone_id == 9:
            if get_peer_zone(1) != 9 and get_peer_zone(2) == 10:
                zone_id = 10
                swapped = True

        if swapped:
            in_danger = 1

        # Start with nominal command speed
        final_v_cmd = self.v_ref

        # Collect peers in the same zone (fresh only)
        nearby_peers = {}
        for pid, p_state in self.peer_states.items():
            if (now_sec - p_state["t"]) > self.peer_timeout:
                continue
            p_zone = int(p_state.get("zone", -1))
            if zone_id >= 0 and p_zone == zone_id:
                nearby_peers[pid] = p_state

        if nearby_peers:
            in_danger = 1

        # Apply peer-based avoidance
        if nearby_peers or in_danger == 1:
            for pid, p_state in nearby_peers.items():
                peer_eta = float(p_state.get("eta", 1e9))
                peer_lap = int(p_state.get("lap", 0))
                peer_x = float(p_state.get("x", 0.0))
                peer_y = float(p_state.get("y", 0.0))
                peer_flag = int(p_state.get("flag", 0))

                temp_v_cmd = self.ca.avoid_collision(
                    zone_id, final_v_cmd, eta, peer_eta,
                    self.lap, peer_lap,
                    x, y, peer_x, peer_y,
                    self.vid, pid,
                    self.my_flag, peer_flag,
                    self.peer_states, self.hv_data
                )

                override = False
                if isinstance(temp_v_cmd, tuple):
                    temp_v_cmd, override = temp_v_cmd

                if override:
                    final_v_cmd = max(final_v_cmd, float(temp_v_cmd))
                else:
                    final_v_cmd = min(final_v_cmd, float(temp_v_cmd))

        # Apply HV-only avoidance only when no peers are in the same zone
        if zone_id in (7, 8) and not nearby_peers:
            temp_v_cmd = self.ca.avoid_hv(
                zone_id, final_v_cmd, eta,
                self.lap, x, y,
                self.vid, self.my_flag,
                self.peer_states, self.hv_data
            )

            override = False
            if isinstance(temp_v_cmd, tuple):
                temp_v_cmd, override = temp_v_cmd

            if override:
                final_v_cmd = max(final_v_cmd, float(temp_v_cmd))
            else:
                final_v_cmd = min(final_v_cmd, float(temp_v_cmd))

        # Publish V2V state
        self.publish_v2v(zone_id, in_danger, eta, self.lap, x, y, self.my_flag, final_v_cmd)

        # Log zone transitions
        self.log_zone_status(zone_id, in_danger, in_conflict, self.lap)

        v_cmd = float(final_v_cmd)

        # Lookahead target index
        tgt = i + self.lookahead_idx

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Ensure target is in front of the vehicle (in local x > 0)
        for _ in range(10):
            tx, ty = self.pts[tgt % self.N]
            dx = tx - x
            dy = ty - y
            x_r = cos_y * dx + sin_y * dy
            if x_r > 0.05:
                break
            tgt += 1

        # Pure pursuit in vehicle frame
        x_r = cos_y * dx + sin_y * dy
        y_r = -sin_y * dx + cos_y * dy
        Ld = math.hypot(x_r, y_r)

        if Ld < 1e-3:
            self.publish(0.0, 0.0)
            return

        alpha = math.atan2(y_r, x_r)
        kappa = 2.0 * math.sin(alpha) / Ld

        L_wb = 0.30  # wheelbase [m]
        delta_ff = math.atan(L_wb * kappa)

        # PID feedback on lateral error (y in vehicle frame)
        e_y = y_r
        delta_fb = self.pid.update(e_y, dt)

        # Final steering
        delta = delta_ff + delta_fb

        delta_max = math.radians(30.0)
        delta = max(-delta_max, min(delta_max, delta))

        # Steering angle -> yaw rate
        w = (v_cmd / L_wb) * math.tan(delta)

        self.publish(v_cmd, w)


def main():
    rclpy.init()
    node = P3Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
