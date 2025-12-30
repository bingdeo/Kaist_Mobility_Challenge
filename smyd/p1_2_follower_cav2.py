#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel, AccelStamped


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation

    # Simulator Euler-packed format:
    # orientation.x=roll, y=pitch, z=yaw (rad), w=1.0(dummy)
    if abs(q.w - 1.0) < 1e-3 and (abs(q.x) > 1.0 or abs(q.y) > 1.0 or abs(q.z) > 1.0):
        return float(q.z)

    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)


class P12Follower(Node):
    def __init__(self):
        super().__init__("p1_2_follower_cav2")

        self.declare_parameter("waypoints_json", "")
        self.declare_parameter("v_ref", 0.6)
        self.declare_parameter("w_max", 5.0)

        # V2V (state only)
        self.declare_parameter("my_share_topic", "/cav2/v2v_state")
        self.declare_parameter("peer_share_topic", "/peer/cav1/v2v_state")

        # conflict map (your conflict_map_p1_2.json)
        self.declare_parameter("conflict_map_json", "")
        self.declare_parameter("zones_json", "")  # fallback

        # sampling + CROSS_POINT danger length
        self.declare_parameter("ds_per_point", 0.01)          # 0.01m
        self.declare_parameter("point_danger_points", 30)     # 0.3m 점유(필요시 조절)

        self.declare_parameter("search_window", 600)
        self.declare_parameter("peer_timeout", 0.8)  # seconds

        wp = self.get_parameter("waypoints_json").value
        if not wp:
            raise RuntimeError("waypoints_json is empty")

        self.v_ref = float(self.get_parameter("v_ref").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.win = int(self.get_parameter("search_window").value)
        self.peer_timeout = float(self.get_parameter("peer_timeout").value)

        self.ds_per_point = float(self.get_parameter("ds_per_point").value)
        self.point_danger_points = int(self.get_parameter("point_danger_points").value)

        self.pts = self.load_json(wp)
        if len(self.pts) < 5:
            raise RuntimeError("waypoints too short")
        self.N = len(self.pts)

        # unwrapped progress index (monotonic)
        self.idx_u = 0
        self.allow_back = 0

        # control I/O (simulator interface)
        self.sub = self.create_subscription(PoseStamped, "/Ego_pose", self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Accel, "/Accel", qos_profile_sensor_data)

        # V2V state pub/sub
        my_topic = self.get_parameter("my_share_topic").value
        peer_topic = self.get_parameter("peer_share_topic").value
        self.share_pub = self.create_publisher(AccelStamped, my_topic, qos_profile_sensor_data)
        self.peer_sub = self.create_subscription(AccelStamped, peer_topic, self.cb_peer, qos_profile_sensor_data)
        self.peer_state = None

        # load conflict map (cav2 = path2)
        self.path_key = "path2"
        cm_path = self.get_parameter("conflict_map_json").value
        if not cm_path:
            cm_path = self.get_parameter("zones_json").value  # fallback
        if not cm_path:
            raise RuntimeError("conflict_map_json (or zones_json) is empty")

        self.zones = self.load_conflict_map(cm_path, self.path_key)

        self.get_logger().info(f"Loaded pts={len(self.pts)} file={wp}")
        self.get_logger().info(f"Loaded conflict zones={len(self.zones)} file={cm_path}")
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
            "t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

    def peer_is_fresh(self):
        if self.peer_state is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self.peer_state["t"]) < self.peer_timeout

    def publish_v2v(self, zone_id, in_danger, eta):
        m = AccelStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.accel.linear.x = float(zone_id)
        m.accel.linear.y = float(in_danger)  # 0/1
        m.accel.linear.z = float(eta)
        self.share_pub.publish(m)

    # ---------- zone logic ----------
    def _in_range(self, idx, a, b):
        # inclusive range, wrap-around supported
        if a <= b:
            return a <= idx <= b
        return (idx >= a) or (idx <= b)

    def compute_zone_state(self, cur_idx_mod, v_used):
        best = None  # (priority, eta, zone_id, in_danger)
        v = max(float(v_used), 0.05)

        for z in self.zones:
            zid = z["zone_id"]
            ms = z["monitor_start"]
            cs = z["conflict_start"]
            ce = z["conflict_end"]

            # danger(점유) 우선
            if self._in_range(cur_idx_mod, cs, ce):
                cand = (0, 0.0, zid, 1)
                if best is None or cand < best:
                    best = cand
                continue

            # monitor(접근): monitor_start ~ conflict_start 직전
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

        _, eta, zid, in_danger = best
        return zid, in_danger, float(eta)

    # ---------- main callback ----------
    def cb(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_pose(msg)

        i = self.nearest_index(x, y)
        idx_mod = i % self.N

        # publish my V2V state (zone_id, in_danger, eta)
        zone_id, in_danger, eta = self.compute_zone_state(idx_mod, self.v_ref)
        self.publish_v2v(zone_id, in_danger, eta)

        # (yield 로직은 다음 단계에서 v_cmd만 제한하면 됨)
        v_cmd = self.v_ref

        # lookahead
        tgt = i + 10
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

        y_r = -sin_y*dx + cos_y*dy
        L = math.hypot(x_r, y_r)
        if L < 1e-3:
            self.publish(0.0, 0.0)
            return

        kappa = 2.0 * y_r / (L * L)
        w = v_cmd * kappa

        self.publish(v_cmd, w)


def main():
    rclpy.init()
    node = P12Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
