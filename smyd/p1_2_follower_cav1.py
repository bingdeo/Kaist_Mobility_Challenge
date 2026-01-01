#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel
from scipy.spatial import KDTree  # 충돌 감지용 KDTree 추가
from smyd.Collision_Avoidance import CollisionAvoidance # 회피 모듈 추가


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



class P12Follower(Node):
    def __init__(self):
        super().__init__("p1_2_follower_cav1")

        self.declare_parameter("waypoints_json", "")
        self.declare_parameter("v_ref", 0.6)
        self.declare_parameter("w_max", 5.0)

        # for V2V communication
        self.declare_parameter("my_share_topic", "/cav1/share")
        self.declare_parameter("peer_share_topic", "/peer/cav2/share")
        
        # (추가) 충돌 감지를 위한 파라미터
        self.declare_parameter("peer_waypoints_json", "")
        self.declare_parameter("monitoring_zones_json", "")

        my_topic = self.get_parameter("my_share_topic").value
        peer_topic = self.get_parameter("peer_share_topic").value

        # (deprecated) kept for CLI compatibility, but not used anymore
        self.declare_parameter("lookahead_idx", 30)

        self.declare_parameter("search_window", 600)

        wp = self.get_parameter("waypoints_json").value
        if not wp:
            raise RuntimeError("waypoints_json is empty")
            
        peer_wp = self.get_parameter("peer_waypoints_json").value
        zones_file = self.get_parameter("monitoring_zones_json").value

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

        # (추가) V2V pub/sub
        self.share_pub = self.create_publisher(PoseStamped, my_topic, qos_profile_sensor_data)
        self.peer_sub  = self.create_subscription(PoseStamped, peer_topic, self.cb_peer, qos_profile_sensor_data)

        self.peer_pose = None  # (추가) 상대 최신 PoseStamped 저장
        self.collision_flag = False # 충돌 위험 플래그
        
        # (추가) 회피 모듈 초기화
        self.avoider = CollisionAvoidance(self)

        # (추가) 충돌 감지용 데이터 로드 및 KDTree 생성
        self.tree_my = KDTree(self.pts)
        self.tree_peer = None
        self.zones = []

        if peer_wp:
            try:
                self.peer_pts = self.load_json(peer_wp)
                self.tree_peer = KDTree(self.peer_pts)
                self.get_logger().info(f"Loaded peer pts={len(self.peer_pts)}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load peer waypoints: {e}")

        if zones_file:
            try:
                with open(zones_file, 'r') as f:
                    self.zones = json.load(f)
                self.get_logger().info(f"Loaded {len(self.zones)} monitoring zones")
            except Exception as e:
                self.get_logger().warn(f"Failed to load monitoring zones: {e}")

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

        # (추가) Reset Detection: 로컬 검색 결과가 신뢰도가 낮으면(1m 이상) KDTree로 전역 검색 확인
        # 기존 3m(9.0) -> 1m(1.0)로 기준 강화
        if best_d2 > 1.0:
            d, idx = self.tree_my.query([x, y])
            
            # 전역 검색 결과가 로컬 검색보다 훨씬 더 가까우면(예: 1m 이상 차이) 리셋
            # 또는 전역 검색이 매우 가까우면(0.5m 이내) 리셋
            if d < math.sqrt(best_d2) - 1.0 or d < 0.5:
                self.get_logger().warn(f"Track lost (local={math.sqrt(best_d2):.1f}m, global={d:.1f}m). Resetting index to {idx}")
                self.idx_u = int(idx)
                return self.idx_u

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

    def cb_peer(self, msg: PoseStamped):
        # self.get_logger().info(f"Received peer pose: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}", throttle_duration_sec=2.0)
        self.peer_pose = msg  # 상대 최신 포즈 업데이트

    def check_collision_risk(self, my_x, my_y, my_idx):
        if not self.peer_pose or not self.tree_peer or not self.zones:
            return

        peer_x = self.peer_pose.pose.position.x
        peer_y = self.peer_pose.pose.position.y

        # 1. 상대 위치의 인덱스 찾기 (KDTree 사용)
        # _, my_idx = self.tree_my.query((my_x, my_y))  <-- 인자로 받음
        _, peer_idx = self.tree_peer.query((peer_x, peer_y))
        
        # my_idx = int(my_idx)
        peer_idx = int(peer_idx)

        risk_detected = False
        target_zone_id = None
        
        # 2. Zone 검사
        for zone in self.zones:
            z_id = zone['id']
            
            # CAV1(나) -> cav1_waypoint_index_range
            # CAV2(상대) -> cav2_waypoint_index_range
            my_zone_range = zone['cav1_waypoint_index_range']
            peer_zone_range = zone['cav2_waypoint_index_range']

            my_start, my_end = my_zone_range['start'], my_zone_range['end']
            peer_start, peer_end = peer_zone_range['start'], peer_zone_range['end']

            # (수정) 누적 인덱스(my_idx)를 현재 바퀴의 인덱스로 변환하여 비교
            curr_my_idx = my_idx % self.N

            # (추가) 이미 지난 Zone은 계산하지 않음
            if curr_my_idx > my_end:
                continue
            
            # 남은 인덱스 (음수면 이미 진입했거나 지남)
            diff_my = my_start - curr_my_idx
            diff_peer = peer_start - peer_idx

            # (추가) Zone 시작점과 50cm 이내로 가까워지면 Zone ID 출력
            sx, sy = self.pts[my_start]
            dist_to_start = math.hypot(sx - my_x, sy - my_y)
            
            if dist_to_start < 0.5:
                self.get_logger().info(f"[ZONE INFO] Approaching Zone {z_id} (dist={dist_to_start:.2f}m)")
            
            # 두 차량 모두 Zone을 향해 가고 있고(양수), 50 인덱스 이내로 근접
            # 또는 이미 진입한 경우(음수)도 고려해야 함 -> my_idx <= my_end
            THRESHOLD_IDX = 50
            
            is_my_risk = (curr_my_idx <= my_end) and (diff_my < THRESHOLD_IDX)
            is_peer_risk = (peer_idx <= peer_end) and (diff_peer < THRESHOLD_IDX)

            if is_my_risk and is_peer_risk:
                risk_detected = True
                target_zone_id = z_id
                self.get_logger().warn(
                    f"[COLLISION WARNING] Zone {z_id} Risk! "
                    f"Me(idx={curr_my_idx})->ZoneStart({my_start}) remain={diff_my}, "
                    f"Peer(idx={peer_idx})->ZoneStart({peer_start}) remain={diff_peer}"
                )
                
                # (추가) 회피 기동 실행
                self.avoider.execute_avoidance(z_id)
                
                break
        
        # (Optional) Flag logic if needed elsewhere
        self.collision_flag = risk_detected

    def cb(self, msg: PoseStamped):

        # (추가) 내 포즈를 V2V로 송신 (그대로 relay)
        self.share_pub.publish(msg)
        
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_pose(msg)

        i = self.nearest_index(x, y)   # unwrapped

        # (추가) 충돌 감지 로직 실행 (이미 계산된 인덱스 i 사용)
        self.check_collision_risk(x, y, i)

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
    node = P12Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
