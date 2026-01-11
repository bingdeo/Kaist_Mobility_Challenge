"""
Collision Avoidance Module
Zone-specific collision avoidance algorithms
"""

class CollisionAvoidance:
    def __init__(self, node):
        self.node = node  # 차량 제어 노드 (P12Follower) 인스턴스
        self.logger = node.get_logger()

    def avoid_collision(self, zone_id, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """
        Zone-specific collision avoidance logic
        
        Args:
            zone_id: Current conflict zone ID
            v_cmd: Current commanded velocity
            my_eta: My ETA to conflict zone
            peer_eta: Peer's ETA to conflict zone
            my_lap: My lap count
            peer_lap: Peer's lap count
            my_x, my_y: My position
            peer_x, peer_y: Peer's position
            is_cav1: True if CAV1, False if CAV2
            
        Returns:
            Modified velocity command
        """
        if zone_id < 0:
            return v_cmd
            
        # Zone-specific algorithms
        if zone_id == 1:
            return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        elif zone_id == 2:
            return self._zone_2_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        elif zone_id == 3:
            return self._zone_3_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        elif zone_id == 4:
            return self._zone_4_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        elif zone_id == 5:
            return self._zone_5_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        elif zone_id == 6:
            return self._zone_6_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        elif zone_id == 7:
            return self._zone_7_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
        else:
            return self._default_avoidance(v_cmd, my_eta, peer_eta, is_cav1)
    
    #사실상 실행될 일 없음...
    def _default_avoidance(self, v_cmd, my_eta, peer_eta, is_cav1):
        """Default collision avoidance: ETA-based priority"""
        eta_diff = abs(my_eta - peer_eta)

        # Tie-breaker: CAV1 has priority
        if eta_diff <= self.node.tie_eta_sec:
            if not is_cav1:  # CAV2 yields
                return max(self.node.v_min, v_cmd * self.node.yield_ratio)
        else:
            # Later arrival yields
            if my_eta > peer_eta:
                return max(self.node.v_min, v_cmd * self.node.yield_ratio)
        return v_cmd
    
    def _zone_1_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 1,2,6,7: 충돌지점 알고리즘
        1. Diff < 0.4초: 후순 차량이 잠깐 정지
        2. 정지했던 차량은 우선 차량의 eta가 0이 될때까지 정지
        3. Diff >= 0.4: 우선 차량의 ETA에 1초를 더해 그 ETA에 맞는 속도(v_real) 계산 후 입력
        """
        eta_diff = abs(my_eta - peer_eta)
        if eta_diff < 0.4:
            if my_eta < peer_eta:
                return v_cmd
            else:
                return 0
        else:
            if v_cmd == 0:
                return 0
            else:
                if my_eta < peer_eta:
                    return v_cmd
                elif my_eta >= peer_eta:
                    margin_time = peer_eta + 1.0
                    v_real = (my_eta * v_cmd) / margin_time
                    v_cmd = v_real
                    return v_cmd
        
    
    def _zone_2_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 2: CROSS_POINT"""
        return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
    
    def _zone_3_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 3: 회전교차로 알고리즘
        1. 내 flag가 먼저 올라간 경우: 내가 우선 차량
        2. 상대 flag가 먼저 올라간 경우 : 상대 방이 우선 차량 (v_real 계산)
        3. 둘 다 안올라갔거나 둘 다 올라간 경우 : 모두 속도 유지
        """
        eta_diff = abs(my_eta - peer_eta)
        if my_flag == 1 and peer_flag == 0:
            return v_cmd
        elif my_flag == 0 and peer_flag == 1:
            margin_time = peer_eta + 1.0
            v_real = (my_eta * v_cmd) / margin_time
            v_cmd = v_real
            return v_cmd
        else:
            return v_cmd
    
    def _zone_4_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """
        Zone 4: 합류구간 알고리즘
        1. 거리 >= 1.5m: 원래 속도
        2. 상대와 내 ETA 비교 후 margin_time이 내 eta보다 작은 경우 속도 유지
        3. 내 eta보다 margin_time이 큰 경우 그 시간에 맞춰 v_real 계산
        """
        import math
        
        # 유클리드 거리 계산
        distance = math.sqrt((my_x - peer_x)**2 + (my_y - peer_y)**2)
        
        # 거리 >= 1.5m이면 원래 속도로
        if distance >= 1.5:
            return v_cmd
        
        eta_diff = abs(my_eta - peer_eta)

        if my_eta < peer_eta:
            return v_cmd
        elif my_eta >= peer_eta:
            margin_time = peer_eta + 1.0
            if margin_time < my_eta:
                return v_cmd
            elif margin_time >= my_eta:
                v_real = (my_eta * v_cmd) / margin_time
                v_cmd = v_real
                return v_cmd

    def _zone_5_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 5: MERGE_POINT"""
        return self._zone_4_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
    
    def _zone_6_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 6: CROSS_POINT"""
        return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)
    
    def _zone_7_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 7: CROSS_POINT"""
        return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag)