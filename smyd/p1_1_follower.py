#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation

    # Simulator Euler-packed format:
    # orientation.x=roll, y=pitch, z=yaw (rad), w=1.0(dummy)
    # Heuristic: any component magnitude > 1.0 => not a quaternion
    if abs(q.w - 1.0) < 1e-3 and (abs(q.x) > 1.0 or abs(q.y) > 1.0 or abs(q.z) > 1.0):
        return float(q.z)

    # Normal quaternion -> yaw
    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)



class P11Follower(Node):
    def __init__(self):
        super().__init__("p1_1_follower")

        self.declare_parameter("waypoints_json", "")
        self.declare_parameter("v_ref", 0.6)
        self.declare_parameter("w_max", 5.0)

        # (deprecated) kept for CLI compatibility, but not used anymore
        self.declare_parameter("lookahead_idx", 30)

        self.declare_parameter("search_window", 600)

        wp = self.get_parameter("waypoints_json").value
        if not wp:
            raise RuntimeError("waypoints_json is empty")

        self.declare_parameter("alpha_w", 0.8)   # 0~1, 높을수록 더 부드러움(0.75~0.9)
        self.declare_parameter("dw_max", 10.0)   # rad/s^2, w 변화율 제한(8~20)
        self.alpha_w = float(self.get_parameter("alpha_w").value)
        self.dw_max = float(self.get_parameter("dw_max").value)

        self.w_prev = 0.0
        self.t_prev = self.get_clock().now()

        self.v_ref = float(self.get_parameter("v_ref").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.win = int(self.get_parameter("search_window").value)

        self.pts = self.load_json(wp)
        if len(self.pts) < 5:
            raise RuntimeError("waypoints too short")

        self.N = len(self.pts)

        # unwrapped progress index (monotonic)
        self.idx_u = 0
        self.allow_back = 0  # 0 = never go backward

        self.sub = self.create_subscription(PoseStamped, "/Ego_pose", self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Accel, "/Accel", qos_profile_sensor_data)

        self.get_logger().info(f"Loaded pts={len(self.pts)} file={wp}")
        self.get_logger().info("Lookahead is fixed to the next waypoint (i+10).")

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

        # ✅ lookahead = "next waypoint"
        tgt = i + 10                    # unwrapped target (always next)

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

        # Pure Pursuit: w = v * kappa, kappa = 2*y_r / L^2
        y_r = -sin_y*dx + cos_y*dy
        L = math.hypot(x_r, y_r)
        if L < 1e-3:
            self.publish(0.0, 0.0)
            return

        kappa = 2.0 * y_r / (L * L)
        w = self.v_ref * kappa

        self.publish(self.v_ref, w)


def main():
    rclpy.init()
    node = P11Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
