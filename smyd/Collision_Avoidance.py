class CollisionAvoidance:
    def __init__(self, node):
        self.node = node  # 차량 제어 노드 (P12Follower) 인스턴스

    def execute_avoidance(self, zone_id):
        """
        zone_id에 따라 다른 회피 기동 알고리즘을 수행합니다.
        self.node.v_ref 등을 조절하여 차량을 제어할 수 있습니다.
        """
        self.node.get_logger().info(f"[CollisionAvoidance] Executing logic for Zone {zone_id}")

        if zone_id == 1:
            # Zone 1에 대한 회피 로직 (예시: 정지)
            # self.node.v_ref = 0.0
            pass
        elif zone_id == 2:
            # Zone 2에 대한 회피 로직
            pass
        elif zone_id == 3:
            # Zone 3에 대한 회피 로직
            pass
        elif zone_id == 4:
            # Zone 4에 대한 회피 로직
            pass
        elif zone_id == 5:
            # Zone 5에 대한 회피 로직
            pass
        elif zone_id == 6:
            # Zone 6에 대한 회피 로직
            pass
        elif zone_id == 7:
            # Zone 7에 대한 회피 로직
            pass
        else:
            self.node.get_logger().warn(f"[CollisionAvoidance] Unknown zone_id: {zone_id}")
