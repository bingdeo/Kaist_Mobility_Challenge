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
        1. Diff >= 3초: 둘 다 속도 유지
        2. 1초 <= Diff < 3초: ETA 작은 차량 우선
        3. Diff < 1.0 : lap count 및 DOMAiN_ID로 판단
        """
        eta_diff = abs(my_eta - peer_eta)
        
        # Case 1: ETA >= 4초 -> 둘 다 속도 유지
        if eta_diff >= 4.0:
            return v_cmd
        elif eta_diff < 4.0 and eta_diff >= 2.0:
            # Case 2: 2초 <= ETA < 4초 -> lap count tie-breaker
            if my_eta < peer_eta:
                return v_cmd
            elif my_eta > peer_eta:
                return max(self.node.v_min, v_cmd * 0.6)
        else:
            if my_lap == peer_lap:  # 같은 lap -> ETA 기준
                if is_cav1:  # CAV1이 우선 차량 
                    return v_cmd
                else:
                    return max(self.node.v_min, v_cmd * 0.6)
            elif my_lap > peer_lap: # 내가 후순 차량 -> 40% 감속
                return max(self.node.v_min, v_cmd * 0.6)
            else: # 내가 우선 차량 -> 속도 유지
                return v_cmd
        
    
    def _zone_2_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 2: CROSS_POINT"""
        return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1)
    
    def _zone_3_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 3: 회전교차로 알고리즘
        1. ETA >= 5초: 둘 다 속도 유지
        2. ETA < 5초: CAV1 우선, CAV2가 40% 감속
        3. ETA < 1초: lap count 작은 차량 우선, 다른 차량 70% 감속
        """
        eta_diff = abs(my_eta - peer_eta)
        
        # Case 1: ETA >= 3초 -> 둘 다 속도 유지
        if eta_diff >= 3.0:
            return v_cmd
        # Case 2: ETA < 3초 -> flag 고려
        elif eta_diff < 3.0:
            if my_flag == 1 and peer_flag == 0:
                return v_cmd
            elif my_flag == 0 and peer_flag == 1:
                return max(self.node.v_min, v_cmd * 0.2)
            elif my_flag == 1 and peer_flag == 1:
                if not is_cav1:
                    return v_cmd
                else:
                    return max(self.node.v_min, v_cmd * 0.6)
    
    def _zone_4_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """
        Zone 4: 합류구간 알고리즘
        1. 거리 >= 1.5m: 원래 속도
        2. ETA >= 5초: 둘 다 속도 유지
        3. 1초 < ETA < 5초: 후순차량 40% 감속
        4. ETA <= 1초: 후순차량 70% 감속
        """
        import math
        
        # 유클리드 거리 계산
        distance = math.sqrt((my_x - peer_x)**2 + (my_y - peer_y)**2)
        
        # 거리 >= 1.5m이면 원래 속도로
        if distance >= 1.5:
            return v_cmd
        
        eta_diff = abs(my_eta - peer_eta)
        
        # Case 1: ETA >= 5초 -> 둘 다 속도 유지
        if eta_diff >= 3.0:
            return v_cmd
        # Case 2: 1초 < ETA < 5초
        if eta_diff < 3.0 and eta_diff >= 1.0:
            if my_eta > peer_eta:  # 내가 늦게 도착 -> 40% 감속
                return max(self.node.v_min, v_cmd * 0.6)
            else:
                return v_cmd 

        # Case 3: ETA <= 1초 -> lap count tie-breaker
        if eta_diff < 1.0:
            if my_lap > peer_lap:  # 내가 후순 차량 -> 70% 감속
                return max(self.node.v_min, v_cmd * 0.3)
            elif my_lap < peer_lap:
                return v_cmd
            else:  # 같은 lap -> ETA 기준
                if not is_cav1:  # 내가 늦게 도착 -> 70% 감속
                    return max(self.node.v_min, v_cmd * 0.3)
                else:
                    return v_cmd

    def _zone_5_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 5: MERGE_POINT"""
        return self._zone_4_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1)
    
    def _zone_6_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 6: CROSS_POINT"""
        return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1)
    
    def _zone_7_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1, my_flag, peer_flag):
        """Zone 7: CROSS_POINT"""
        return self._zone_1_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, my_x, my_y, peer_x, peer_y, is_cav1)