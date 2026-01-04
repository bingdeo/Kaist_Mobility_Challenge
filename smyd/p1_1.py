#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation

    # Simulator Euler-packed format:
    if abs(q.w - 1.0) < 1e-3 and (abs(q.x) > 1.0 or abs(q.y) > 1.0 or abs(q.z) > 1.0):
        return float(q.z)

    # Normal quaternion -> yaw
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

class P11Follower(Node):
    def __init__(self):
        super().__init__("p1_1")

        self.declare_parameter("waypoints_json", "")
        self.declare_parameter("v_ref", 2.0)
        self.declare_parameter("w_max", 5.0)
        self.v_ref = float(self.get_parameter("v_ref").value)
        self.w_max = float(self.get_parameter("w_max").value)

        # (deprecated) kept for CLI compatibility, but not used anymore
        # self.declare_parameter("lookahead_idx", 30)
        self.declare_parameter("search_window", 600)
        self.win = int(self.get_parameter("search_window").value)

        # wp
        # waypoints path
        wp_param = self.get_parameter("waypoints_json").value

        if wp_param:
            # 1) launch / CLI에서 경로를 줬으면 그걸 사용
            wp = wp_param
        else:
            # 2) 기본값: cav 패키지의 waypoints/path.json
            pkg_share = get_package_share_directory("cav")
            wp = os.path.join(pkg_share, "waypoints", "cav1_path.json")

        self.get_logger().info(f"Using waypoints file: {wp}")

        # waypoints(wp)
        self.pts = self.load_json(wp)
        if len(self.pts) < 5:
            raise RuntimeError("waypoints too short")

        self.N = len(self.pts)

        # 조향 안정화 param
        self.declare_parameter("alpha_w", 0.8)   # 0~1, 높을수록 더 부드러움(0.75~0.9)
        self.declare_parameter("dw_max", 10.0)   # rad/s^2, w 변화율 제한(8~20)
        self.alpha_w = float(self.get_parameter("alpha_w").value)
        self.dw_max = float(self.get_parameter("dw_max").value)

        self.w_prev = 0.0
        self.t_prev = self.get_clock().now()

        # unwrapped progress index (monotonic)
        self.idx_u = 0
        self.allow_back = 0  # 0 = never go backward

        # lap timing
        self.start_time = None
        self.prev_lap = 0

        # sub/pub
        self.sub = self.create_subscription(PoseStamped, "/Ego_pose", self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Accel, "/Accel", qos_profile_sensor_data)

        self.get_logger().info(f"Loaded pts={len(self.pts)} file={wp}")
        self.get_logger().info("Lookahead is fixed to the next waypoint (i+60).")

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

        log_path = "/tmp/pid_log_p1_1.csv"
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        # pkg = get_package_share_directory("cav")
        # log_path = os.path.join(pkg, "log", "pid_log.csv")
        
        self.log_f = open("/tmp/pid_log_p1_1.csv", "w")
        self.log_f.write("t,y_r,w,w_pp,w_pid,P,I,D\n")
        self.t0 = self.get_clock().now()

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

        # enforce monotonic progress
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
        # documentation can be inconsistent; set both
        msg.angular.x = float(w)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def cb(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_pose(msg)

        i = self.nearest_index(x, y)   # unwrapped

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

        # ✅ lookahead = "next waypoint"
        tgt = i + 60                    # unwrapped target (always next)

        # make sure the target is in front of the car
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
    node = P11Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
