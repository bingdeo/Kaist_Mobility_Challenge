#!/usr/bin/env python3
import math
import json
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel
from ament_index_python.packages import get_package_share_directory
import os

def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation

    # packed euler
    if abs(q.w - 1.0) < 1e-6:
        return float(q.z)

    # quaternion -> yaw
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

    def update(self, e, dt):
        if dt <= 0.0:
            return 0.0

        self.e_int += e * dt
        self.e_int = max(-self.i_limit, min(self.i_limit, self.e_int))

        de = (e - self.e_prev) / dt
        self.e_prev = e

        return (self.Kp * e) + (self.Ki * self.e_int) + (self.Kd * de)


class P2Follower(Node):
    """
    Problem 2:
      - 3-lane waypoint following
      - subscribe HV poses (domain 1) bridged from domain 100
      - car-following (slow down) + lane change when safe
      - merge zone handling
    """

    def __init__(self):
        super().__init__("p2")

        # ---- params ----
        self.declare_parameter("lane_waypoints_jsons", [])   # [lane1.json, lane2.json, lane3.json]
        self.declare_parameter("v_ref", 2.0)
        self.declare_parameter("v_min", 0.10)
        self.declare_parameter("follow_ratio", 0.20)         # v_ref * 0.2
        self.declare_parameter("follow_dist", 1.00)          # 전방 차량과의 간격 (m)
        self.declare_parameter("front_check_range", 3.00)    # 전방 차량 탐색 범위 (m)

        self.declare_parameter("merge_world_tol", 0.10)     # lane center가 이 거리 이내로 모이면 merge로 판단
        self.declare_parameter("merge_lookahead_m", 3.5)    # 몇 m 앞을 보고 merge_soon 판단할지
        self.declare_parameter("merge_follow_dist", 1.5)    # merge(soon 포함)에서는 조금 더 멀리서 전방차 감속
        self.declare_parameter("merge_follow_ratio", 0.2)  # v_ref * 0.2   # merge 구간 내 전방차 감속 비율


        self.declare_parameter("w_max", 5.0)
        self.declare_parameter("lookahead_points", 60)
        self.declare_parameter("search_back", 80)
        self.declare_parameter("search_fwd", 500)

        # 차선 판별/안전 체크
        self.declare_parameter("lane_band", 0.14)            # "차선 위" 판정 (y_r - lane_offset) 허용폭
        self.declare_parameter("gap_front", 1.0)            # 차선변경 시 target lane 전방 안전거리
        self.declare_parameter("gap_rear", 1.0)             # 차선변경 시 target lane 후방 안전거리
        self.declare_parameter("lane1_trigger_dist", 0.5)   # [m] 이보다 앞차가 가까우면 lane1로 변경(가능 시)
        self.declare_parameter("merge_gap_rear", 1.0)  # merge 구간 rear 안전거리(큰 값)
        # "수평 구간에서만" 차선 변경
        self.declare_parameter("straight_check_ahead_pts", 25)
        # 토픽
        self.declare_parameter("ego_topics", ["/Ego_pose", "/CAV_01"])  # 둘 다 구독(브릿지/환경 따라 다름)
        self.declare_parameter("hv_topics", [])             # 비우면 /HV_19 ~ /HV_36 자동

        self.declare_parameter("hv_timeout", 0.8)
        self.declare_parameter("merge_lat_band", 0.3)  # merge 구간에서 전방차로 인정할 lateral 허용폭
        self.declare_parameter("lane_classify_band", 0.20)  # 코너에서 HV lane 판정용(조금 넓게)

        # ---- load params ----
        lane_files = list(self.get_parameter("lane_waypoints_jsons").value)

        if not lane_files:
            try:
                pkg_share = get_package_share_directory("pkg_p2")
                lane_files = [
                    os.path.join(pkg_share, "waypoints", "p2_lane1.json"),
                    os.path.join(pkg_share, "waypoints", "p2_lane2.json"),
                    os.path.join(pkg_share, "waypoints", "p2_lane3.json"),
                ]
            except Exception as e:
                raise RuntimeError(
                    "lane_waypoints_jsons not set and failed to locate pkg_p2 package"
                ) from e
            
        if len(lane_files) != 3:
            raise RuntimeError("lane_waypoints_jsons must be a list of 3 json paths (3 lanes).")

        self.v_ref = float(self.get_parameter("v_ref").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.follow_ratio = float(self.get_parameter("follow_ratio").value)
        self.follow_dist = float(self.get_parameter("follow_dist").value)
        self.front_check_range = float(self.get_parameter("front_check_range").value)
        self.lane_classify_band = float(self.get_parameter("lane_classify_band").value)

        self.w_max = float(self.get_parameter("w_max").value)
        self.lookahead_points = int(self.get_parameter("lookahead_points").value)
        self.search_back = int(self.get_parameter("search_back").value)
        self.search_fwd = int(self.get_parameter("search_fwd").value)

        self.lane_band = float(self.get_parameter("lane_band").value)
        self.gap_front = float(self.get_parameter("gap_front").value)
        self.gap_rear = float(self.get_parameter("gap_rear").value)
        self.merge_gap_rear = float(self.get_parameter("merge_gap_rear").value)


        self.straight_check_ahead_pts = int(self.get_parameter("straight_check_ahead_pts").value)
        self.hv_timeout = float(self.get_parameter("hv_timeout").value)

        self.merge_world_tol = float(self.get_parameter("merge_world_tol").value)
        self.merge_lookahead_m = float(self.get_parameter("merge_lookahead_m").value)
        self.merge_follow_dist = float(self.get_parameter("merge_follow_dist").value)
        self.merge_follow_ratio = float(self.get_parameter("merge_follow_ratio").value)
        self.merge_lat_band = float(self.get_parameter("merge_lat_band").value)
        self.lane1_trigger_dist = float(self.get_parameter("lane1_trigger_dist").value)


        # ---- load lanes ----
        self.lanes = [self._load_json(p) for p in lane_files]
        self.N = [len(pts) for pts in self.lanes]
        if min(self.N) < 10:
            raise RuntimeError("lane waypoints too short")
        
        # ---- precompute per-lane arc-length (s) ----
        self.lane_s = [self._compute_lane_s(pts) for pts in self.lanes]
        self.lane_len = [s[-1] for s in self.lane_s]


        # per-lane unwrapped index
        self.idx_u = [None, None, None]   # init on first ego pose

        self.current_lane = None          # 0/1/2
        self.lane_change_active = False

        # PID
        self.declare_parameter("Kp", 0.2)
        self.declare_parameter("Ki", 0.0)
        self.declare_parameter("Kd", 0.0)
        self.pid = SteeringPID(
            float(self.get_parameter("Kp").value),
            float(self.get_parameter("Ki").value),
            float(self.get_parameter("Kd").value),
            i_limit=0.3
        )
        self.t_prev = self.get_clock().now()

        # ---- I/O ----
        self.pub = self.create_publisher(Accel, "/Accel", qos_profile_sensor_data)

        # ego subscriptions (robust)
        self._last_ego_stamp_ns = None
        for t in list(self.get_parameter("ego_topics").value):
            self.create_subscription(PoseStamped, t, self.cb_ego, qos_profile_sensor_data)

        # HV subscriptions
        hv_topics = list(self.get_parameter("hv_topics").value)
        if not hv_topics:
            hv_topics = [f"/HV_{i}" for i in range(19, 37)]  # 19~36

        self.hv_state = {}  # key(topic) -> dict(x,y,t)
        for t in hv_topics:
            self.create_subscription(PoseStamped, t, partial(self.cb_hv, key=t), qos_profile_sensor_data)
        self._dbg_t_prev = self.get_clock().now()

    def _load_json(self, path):
        with open(path, "r") as f:
            data = json.load(f)

        X = data.get("X", data.get("x"))
        Y = data.get("Y", data.get("y"))
        if X is None or Y is None or len(X) != len(Y):
            raise RuntimeError(f"{path}: JSON must contain X,Y (or x,y) and lengths must match")

        return [(float(x), float(y)) for x, y in zip(X, Y)]

    def cb_hv(self, msg: PoseStamped, key: str):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.hv_state[key] = {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "t_rx": float(now),
        }

    def _hv_is_fresh(self, st):
        now = self.get_clock().now().nanoseconds * 1e-9
        t0 = st.get("t_rx", 0.0)
        return (now - t0) < self.hv_timeout

    def _nearest_index_full(self, lane_i, x, y):
        pts = self.lanes[lane_i]
        best_j = 0
        best_d2 = 1e18
        for j, (px, py) in enumerate(pts):
            dx = px - x
            dy = py - y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_j = j
        return best_j, best_d2

    def _nearest_index_window(self, lane_i, x, y, idx_u):
        pts = self.lanes[lane_i]
        n = self.N[lane_i]
        back = self.search_back
        fwd = self.search_fwd

        best_j = idx_u if idx_u is not None else 0
        best_d2 = 1e18

        if idx_u is None:
            j0 = 0
            j1 = n - 1
        else:
            j0 = idx_u - back
            j1 = idx_u + fwd

        for j in range(j0, j1 + 1):
            px, py = pts[j % n]
            dx = px - x
            dy = py - y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_j = j

        return best_j, best_d2

    def _publish(self, v, w):
        if w > self.w_max:
            w = self.w_max
        elif w < -self.w_max:
            w = -self.w_max

        msg = Accel()
        msg.linear.x = float(v)
        msg.angular.x = float(w)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def _is_straight_section(self, lane_i, idx_u):
        """
        Decide straightness by checking heading change along the lane.
        If heading variation ahead is small, treat as straight.
        """
        pts = self.lanes[lane_i]
        n = self.N[lane_i]

        i0 = idx_u % n
        k = max(5, self.straight_check_ahead_pts)

        def heading(i):
            x0, y0 = pts[i % n]
            x1, y1 = pts[(i + 1) % n]
            return math.atan2(y1 - y0, x1 - x0)

        h0 = heading(i0)
        max_dh = 0.0

        for d in (1, 3, k):
            hi = heading(i0 + d)
            dh = abs(math.atan2(math.sin(hi - h0), math.cos(hi - h0)))
            max_dh = max(max_dh, dh)

        # 기준: 3도 이내면 직선으로 간주
        return max_dh < math.radians(3.0)

    def _classify_lane_by_offset(self, y_r, offsets, band=None):
        band = self.lane_band if band is None else band

        best_li = None
        best = 1e18
        for li in range(3):
            d = abs(y_r - offsets[li])
            if d < best:
                best = d
                best_li = li

        if best_li is None or best > band:
            return None
        return best_li


    def _find_merge_side_vehicle(self, offsets, x, y, yaw, my_lane):
        """
        merge 구간에서 '내 차선이 아닌' 차량이 내 옆/앞/뒤에 붙어있는지 탐색.
        반환: (x_r, y_r, li, st) or None
        """
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        best = None  # choose smallest |x_r|
        for st in self.hv_state.values():
            if not self._hv_is_fresh(st):
                continue

            dx = st["x"] - x
            dy = st["y"] - y
            x_r = cos_y*dx + sin_y*dy
            y_r = -sin_y*dx + cos_y*dy

            # 내 주변(앞/뒤)만 본다: 기존 gap_front/gap_rear 재사용
            if x_r < -self.merge_gap_rear or x_r > self.gap_front:
                continue

            # 어느 lane 쪽인지(merge에서도 쓰려고 band는 넓게: lane_classify_band 사용)
            li = self._classify_lane_by_offset(y_r, offsets, self.lane_classify_band)
            if li is None:
                continue
            if li == my_lane:
                continue  # 내 차선 차량은 여기서 제외(급감속으로 뒤차 박을 위험)

            if best is None or abs(x_r) < abs(best[0]):
                best = (x_r, y_r, li, st)

        return best

    def _find_rear_vehicle_same_lane(self, s_ego, my_lane, rear_range):
        """
        내 차선에서 '바로 뒤' 차량이 너무 가까우면 merge-yield 급감속을 피하기 위한 체크.
        반환: ds_back (m) or None
        """
        L = self.lane_len[my_lane]
        best = None

        for st in self.hv_state.values():
            if not self._hv_is_fresh(st):
                continue

            s_hv, d_hv = self._project_to_lane_sd(my_lane, st["x"], st["y"], idx_hint_u=self.idx_u[my_lane])
            if s_hv is None:
                continue
            if abs(d_hv) > self.lane_classify_band:
                continue

            ds_back = (s_ego - s_hv) % L  # hv가 뒤면 작은 양수
            if 0.0 < ds_back <= rear_range:
                if best is None or ds_back < best:
                    best = ds_back

        return best

    def _find_front_vehicle_merged(self, offsets, x, y, yaw):
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        best = None  # (x_r, st)
        for st in self.hv_state.values():
            if not self._hv_is_fresh(st):
                continue

            dx = st["x"] - x
            dy = st["y"] - y
            x_r = cos_y*dx + sin_y*dy
            if x_r <= 0.0 or x_r > self.front_check_range:
                continue

            y_r = -sin_y*dx + cos_y*dy

            # merge 통로: 3개 lane center offsets 중 가장 가까운 center와의 lateral 거리로 판단
            dlat = min(abs(y_r - o) for o in offsets)
            if dlat > self.merge_lat_band:
                continue

            if best is None or x_r < best[0]:
                best = (x_r, st)

        return best

    
    def _wrap_s(self, lane_i, s):
        L = self.lane_len[lane_i]
        if L <= 1e-6:
            return s
        s2 = s % L
        return s2

    def _lane_point_at_s(self, lane_i, s_ref):
        s_ref = self._wrap_s(lane_i, s_ref)
        j = self._idx_at_s(lane_i, s_ref)
        return self.lanes[lane_i][j]

    def _is_merge_at_s(self, s_ref, base_lane=1):
        rx, ry = self._lane_point_at_s(base_lane, s_ref)
        tol2 = self.merge_world_tol * self.merge_world_tol

        # ref점 근처에서 각 lane의 nearest를 비교(=s 정렬 불필요)
        best = []
        for li in range(3):
            j, d2 = self._nearest_index_window(li, rx, ry, self.idx_u[li])
            best.append((li, j, d2))

        # 어떤 두 lane이라도 ref 근처에서 서로 가까워지면 merge
        for i in range(3):
            for j in range(i+1, 3):
                # 각 lane의 nearest 점 좌표
                li, ji, _ = best[i]
                lj, jj, _ = best[j]
                xi, yi = self.lanes[li][ji % self.N[li]]
                xj, yj = self.lanes[lj][jj % self.N[lj]]
                if (xi-xj)**2 + (yi-yj)**2 < tol2:
                    return True

        return False

    def _adjacent_lanes(self, offsets, current_lane):
        # offsets(ego y축 기준) 정렬로 좌/우 인접 차선 결정
        order = sorted(range(3), key=lambda i: offsets[i])  # 작은 y: 우측, 큰 y: 좌측
        k = order.index(current_lane)
        left_lane = order[k+1] if k < 2 else None
        right_lane = order[k-1] if k > 0 else None
        return left_lane, right_lane

    def _target_lane_is_free(self, offsets, target_lane, x, y, yaw):
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        for st in self.hv_state.values():
            if not self._hv_is_fresh(st):
                continue
            dx = st["x"] - x
            dy = st["y"] - y
            x_r = cos_y*dx + sin_y*dy
            y_r = -sin_y*dx + cos_y*dy

            li = self._classify_lane_by_offset(y_r, offsets)
            if li != target_lane:
                continue

            # 타겟 차선에서 내 주변(앞/뒤) 일정 범위 내에 차량이 있으면 불가
            if (-self.gap_rear) <= x_r <= self.gap_front:
                return False

        return True

    def _find_front_vehicle_same_lane(self, s_ego, my_lane):
        L = self.lane_len[my_lane]
        best = None  # (ds, st)

        for st in self.hv_state.values():
            if not self._hv_is_fresh(st):
                continue

            s_hv, d_hv = self._project_to_lane_sd(
                my_lane, st["x"], st["y"], idx_hint_u=self.idx_u[my_lane]
            )
            if s_hv is None:
                continue

            # 같은 차선 판정: |d|가 충분히 작아야 함
            if abs(d_hv) > self.lane_classify_band:
                continue

            ds = (s_hv - s_ego) % L  # [0, L)
            if ds <= 0.0 or ds > self.front_check_range:
                continue

            if best is None or ds < best[0]:
                best = (ds, st)

        return best


    def cb_ego(self, msg: PoseStamped):
        # duplicate 방지 (Ego_pose와 CAV_01 둘 다 받을 수 있음)
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self._last_ego_stamp_ns == stamp_ns:
            return
        self._last_ego_stamp_ns = stamp_ns

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_pose(msg)

        # init lane selection (첫 ego pose에서 3개 lane 중 가장 가까운 lane을 current_lane으로)
        if self.current_lane is None:
            best_lane = 0
            best_d2 = 1e18
            for li in range(3):
                j, d2 = self._nearest_index_full(li, x, y)
                self.idx_u[li] = j
                if d2 < best_d2:
                    best_d2 = d2
                    best_lane = li
            self.current_lane = best_lane
        
        # --- s_ego 계산 ---
        s_ego, _ = self._project_to_lane_sd(self.current_lane, x, y, self.idx_u[self.current_lane])
        if s_ego is None:
            j, _ = self._nearest_index_window(self.current_lane, x, y, self.idx_u[self.current_lane])
            self.idx_u[self.current_lane] = j
            s_ego = self.lane_s[self.current_lane][j % self.N[self.current_lane]]
            s_ego = self._wrap_s(self.current_lane, s_ego)

        # --- offsets는 항상 계산 (중요) ---
        offsets, _ = self._lane_offsets_at_s_in_ego_frame(
            s_ego, x, y, yaw, base_lane=self.current_lane
        )

        # --- merge 판정은 로그보다 먼저 (중요) ---
        merged_now  = self._is_merge_at_s(s_ego, base_lane=self.current_lane)
        merged_soon = self._is_merge_at_s(s_ego + self.merge_lookahead_m, base_lane=self.current_lane)
        in_merge_logic = (merged_now or merged_soon)
        merge_approach_lock = (merged_soon and (not merged_now))


        # --- 디버그 로그(시간 타입 유지) ---
        now_t = self.get_clock().now()
        if (now_t - self._dbg_t_prev).nanoseconds > 1_000_000_000:
            self._dbg_t_prev = now_t

            rx, ry = self._lane_point_at_s(self.current_lane, s_ego)

            anchors = []
            for li in range(3):
                j, d2 = self._nearest_index_window(li, rx, ry, self.idx_u[li])
                px, py = self.lanes[li][j % self.N[li]]
                anchors.append((px, py, math.sqrt(d2)))

        straight_ok = self._is_straight_section(self.current_lane, self.idx_u[self.current_lane])

        # 전방(같은 차선) 차량 탐색
        if in_merge_logic:
            front = self._find_front_vehicle_merged(offsets, x, y, yaw)
        else:
            front = self._find_front_vehicle_same_lane(s_ego, self.current_lane)
        merge_side = None
        rear_same = None
        if in_merge_logic:
            merge_side = self._find_merge_side_vehicle(offsets, x, y, yaw, self.current_lane)
            rear_same = self._find_rear_vehicle_same_lane(s_ego, self.current_lane, rear_range=self.merge_gap_rear)



        # lane change decision (conservative)
        # - Prefer lane2(1), lane3(2)
        # - Consider lane1(0) only if neither lane2 nor lane3 is feasible now
        # - Even then: choose lane1 vs wait (slow down & keep lane) by lane1_trigger_dist
         # [ADD HERE] 예외: merge soon(approach lock)인데 내가 lane1(0)이면 lane2(1)로만 변경 허용/강제
        if (not self.lane_change_active) and merge_approach_lock and straight_ok and (self.current_lane == 0):
            target = 1  # lane2
            if self._target_lane_is_free(offsets, target, x, y, yaw):
                self.current_lane = target
                self.lane_change_active = True
                self.idx_u[target] = self._idx_at_s(target, s_ego)

        if (not self.lane_change_active) and (not merge_approach_lock) and straight_ok and (front is not None):
            front_dist, _st = front
            if front_dist < self.follow_dist:
                left_lane, right_lane = self._adjacent_lanes(offsets, self.current_lane)

                # adjacent candidates (ignore None / current)
                adjacent = [li for li in (right_lane, left_lane) if li is not None and li != self.current_lane]

                def _lane_feasible(li):
                    """Return (feasible_now, front_dist_in_lane or None)."""
                    if not self._target_lane_is_free(offsets, li, x, y, yaw):
                        return (False, None)

                    f = self._find_front_vehicle_same_lane(s_ego, li)
                    fd = None if f is None else f[0]

                    # "막힘" 정의: 바꿔도 바로 앞차가 follow_dist 이내면 추월/진행 이득이 거의 없으므로 비가용 처리
                    if fd is not None and fd < self.follow_dist:
                        return (False, fd)

                    return (True, fd)

                feasible = {}
                for li in adjacent:
                    feasible[li] = _lane_feasible(li)  # (ok, fd)

                # 1) lane3(2) 우선, 2) lane2(1) 다음
                target = None
                if 2 in feasible and feasible[2][0]:
                    target = 2
                elif 1 in feasible and feasible[1][0]:
                    target = 1
                else:
                    # lane2/3이 "지금 당장 변경 가능한 대안"으로 없을 때만 lane1(0) 고려
                    if 0 in feasible and feasible[0][0]:
                        # lane1로 갈지(wait할지) 선택:
                        # - 앞차가 너무 가까우면(lane1_trigger_dist 미만) lane1로 변경
                        # - 아니면 감속하며 현 차선 유지(기회 생기면 lane2/3로)
                        if front_dist < self.lane1_trigger_dist:
                            target = 0
                        else:
                            target = None  # wait

                if target is not None:
                    self.current_lane = target
                    self.lane_change_active = True
                    j = self._idx_at_s(target, s_ego)
                    self.idx_u[target] = j

        # speed decision
        v_cmd = self.v_ref

        if in_merge_logic:
            # 1) 합류 side-swipe 방지: 옆 차선 차량이 내 주변(앞/뒤)에 붙어있으면 먼저 양보(감속)
            if merge_side is not None:
                v_yield = max(self.v_min, self.v_ref * self.merge_follow_ratio)

                # 내 차선 뒤차가 너무 붙어있으면 급감속은 피한다(최소한의 보호)
                if rear_same is not None and rear_same < 0.6 * self.merge_gap_rear:
                    v_yield = max(v_yield, 0.6 * self.v_ref)

                v_cmd = min(v_cmd, v_yield)

            # 2) 기존 merge 전방 감속(더 멀리서)
            if front is not None:
                front_dist, _ = front
                if front_dist < self.merge_follow_dist:
                    v_cmd = min(v_cmd, max(self.v_min, self.v_ref * self.merge_follow_ratio))

        else:
            # 기존 일반구간 전방차 추종
            if front is not None:
                front_dist, _ = front
                if front_dist < self.follow_dist:
                    v_cmd = max(self.v_min, self.v_ref * self.follow_ratio)



        # lane change 종료 조건(대충 중앙에 가까워지면 해제)
        if self.lane_change_active:
            if abs(offsets[self.current_lane]) < (self.lane_band * 0.5):
                self.lane_change_active = False

        # ---- path following on current lane ----
        li = self.current_lane
        pts = self.lanes[li]
        n = self.N[li]

        # update nearest index for current lane with window
        iu, _ = self._nearest_index_window(li, x, y, self.idx_u[li])
        self.idx_u[li] = iu

        tgt = iu + self.lookahead_points

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        

        # forward target selection (ego frame x_r > 0)
        for _ in range(10):
            tx, ty = pts[tgt % n]
            dx = tx - x
            dy = ty - y
            x_r = cos_y*dx + sin_y*dy
            if x_r > 0.05:
                break
            tgt += 1

        tx, ty = pts[tgt % n]
        dx = tx - x
        dy = ty - y

        now = self.get_clock().now()
        dt = (now - self.t_prev).nanoseconds * 1e-9
        self.t_prev = now

        # ego lateral error to target
        y_r = -sin_y*dx + cos_y*dy
        x_r =  cos_y*dx + sin_y*dy
        Ld = math.hypot(x_r, y_r)
        if Ld < 1e-3:
            self._publish(self.v_min, 0.0)
            return

        # pure pursuit curvature -> steering angle
        alpha = math.atan2(y_r, x_r)
        kappa = 2.0 * math.sin(alpha) / Ld
        L_wb = 0.30
        delta_ff = math.atan(L_wb * kappa)

        # PID feedback on lateral error
        delta_fb = self.pid.update(y_r, dt)

        delta = delta_ff + delta_fb
        delta_max = math.radians(30.0)
        delta = max(-delta_max, min(delta_max, delta))

        w = v_cmd / L_wb * math.tan(delta)
        self._publish(v_cmd, w)
        
    # 0116 유진 수정
    def _compute_lane_s(self, pts):
        # cumulative arc-length along waypoint polyline
        s = [0.0]
        for i in range(1, len(pts)):
            x0, y0 = pts[i-1]
            x1, y1 = pts[i]
            s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
        return s

    # 0116 유진 수정
    def _idx_at_s(self, lane_i, s_ref):
        # find waypoint index whose lane_s is closest to s_ref
        s_arr = self.lane_s[lane_i]
        # O(N) linear search is acceptable for typical waypoint sizes.
        best_i = 0
        best = float("inf")
        for i, sv in enumerate(s_arr):
            d = abs(sv - s_ref)
            if d < best:
                best = d
                best_i = i
        return best_i

    # 0116 유진 수정
    def _project_to_lane_sd(self, lane_i, x, y, idx_hint_u=None):
        pts = self.lanes[lane_i]
        s_arr = self.lane_s[lane_i]
        n = self.N[lane_i]

        if idx_hint_u is None:
            j0, _ = self._nearest_index_full(lane_i, x, y)
            idx_hint_u = j0

        j_start = idx_hint_u - self.search_back
        j_end   = idx_hint_u + self.search_fwd

        best_s = None
        best_d = None
        best_d2 = float("inf")

        for ju in range(j_start, j_end):
            i0 = ju % n
            i1 = (ju + 1) % n

            x0, y0 = pts[i0]
            x1, y1 = pts[i1]
            vx = x1 - x0
            vy = y1 - y0
            seg_len2 = vx*vx + vy*vy
            if seg_len2 < 1e-9:
                continue

            wx = x - x0
            wy = y - y0
            t = (wx*vx + wy*vy) / seg_len2
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

            px = x0 + t * vx
            py = y0 + t * vy

            dx = x - px
            dy = y - py
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                seg_len = math.sqrt(seg_len2)
                s = s_arr[i0] + t * seg_len

                # signed lateral: segment left normal n = (-vy, vx)
                nx = -vy / seg_len
                ny =  vx / seg_len
                d = dx * nx + dy * ny

                best_s = s
                best_d = d
        if best_s is not None:
            best_s = best_s % self.lane_len[lane_i]
        return best_s, best_d


    def _lane_offsets_at_s_in_ego_frame(self, s_ref, x, y, yaw, base_lane=1):
        """
        base_lane의 s_ref 위치(ref point)를 잡고,
        그 ref point에 대해 각 lane에서 가장 가까운 점을 찾아 ego frame y-offset을 계산.
        => lane 길이/진행률이 달라도 '같은 물리 위치 기준' offsets가 됨.
        """
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        rx, ry = self._lane_point_at_s(base_lane, s_ref)

        offsets = [0.0, 0.0, 0.0]
        anchors = [(0.0, 0.0)] * 3

        for li in range(3):
            j, _d2 = self._nearest_index_window(li, rx, ry, self.idx_u[li])
            px, py = self.lanes[li][j % self.N[li]]

            dx = px - x
            dy = py - y
            y_r = -sin_y*dx + cos_y*dy

            offsets[li] = float(y_r)
            anchors[li] = (px, py)

            self.idx_u[li] = j  # window hint 업데이트

        return offsets, anchors




def main(args=None):
    rclpy.init(args=args)
    node = P2Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
