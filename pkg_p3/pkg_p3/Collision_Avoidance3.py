"""
Collision Avoidance Module (Problem 3)
- Keeps your existing logic/branching.
- Fixes hv_data unpack crash (hv_data is (x, y, yaw, v) on your node side).
- Replaces every `return 2.0` with a dynamic "go-speed" computed from HV, returned as (v, True)
  so your caller's override(max) path actually works.
"""

class CollisionAvoidance:
    def __init__(self, node):
        self.node = node  # Â÷·® Á¦¾î ³ëµå ÀÎ½ºÅÏ½º
        self.logger = node.get_logger()

    # ----------------------------
    # HV "go" speed helper (replaces constant 2.0)
    # ----------------------------
    def _hv_go_speed(self, v_cmd, my_x, my_y, hv_data, margin=0.15, v_cap=2.0):
        import math

        v_base = max(self.node.v_min, float(v_cmd))

        best_d = None
        best_v = None

        for hid in (19, 20):
            if hid not in hv_data:
                continue
            tup = hv_data[hid]
            # hv_data[hid] expected: (x, y, yaw, v) but tolerate shorter
            if len(tup) < 2:
                continue

            hx = float(tup[0])
            hy = float(tup[1])

            hv_v = None
            if len(tup) >= 4:
                hv_v = tup[3]

            if hv_v is None:
                continue

            d = math.hypot(my_x - hx, my_y - hy)
            if best_d is None or d < best_d:
                best_d = d
                best_v = abs(float(hv_v))

        if best_v is None:
            return min(v_cap, v_base)

        return min(v_cap, max(v_base, best_v + margin))

    # ----------------------------
    # External entrypoints
    # ----------------------------
    def avoid_hv(self, zone_id, v_cmd, my_eta, my_lap, my_x, my_y, self_id, my_flag, all_peers, hv_data):
        if zone_id in [7, 8]:
            return self.avoid_only_hv(zone_id, v_cmd, my_eta, my_lap, my_x, my_y, self_id, my_flag, all_peers, hv_data)
        return v_cmd

    def avoid_collision(
        self,
        zone_id, v_cmd, my_eta, peer_eta,
        my_lap, peer_lap,
        my_x, my_y, peer_x, peer_y,
        self_id, peer_id,
        my_flag, peer_flag,
        all_peers, hv_data
    ):
        if zone_id < 0:
            return v_cmd

        if zone_id in [1, 3]:
            return self._case_1_avoidance(v_cmd, my_eta, peer_eta)
        elif zone_id == 2:
            return self._case_2_avoidance(v_cmd, my_eta, peer_eta, self_id, peer_id, all_peers)
        elif zone_id == 4:
            return self._case_3_avoidance(v_cmd, my_eta, peer_eta, self_id, peer_id, all_peers)
        elif zone_id in [5, 6, 9, 10]:
            return self._case_4_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, self_id, peer_id, my_x, my_y, peer_x, peer_y)
        elif zone_id == 7:
            return self._case_5_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, self_id, peer_id, my_x, my_y, peer_x, peer_y, my_flag, peer_flag, hv_data)
        elif zone_id == 8:
            return self._case_6_avoidance(v_cmd, my_eta, peer_eta, my_lap, peer_lap, self_id, peer_id, my_x, my_y, peer_x, peer_y, my_flag, peer_flag, hv_data)
        else:
            return self._default_avoidance(v_cmd, my_eta, peer_eta)

    # ----------------------------
    # Default / Cases 1~4 (unchanged logic)
    # ----------------------------
    def _default_avoidance(self, v_cmd, my_eta, peer_eta):
        if my_eta > peer_eta:
            return max(self.node.v_min, v_cmd * 0.5)
        return v_cmd

    def _case_1_avoidance(self, v_cmd, my_eta, peer_eta):
        """ Zone 1,3 """
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

    def _case_2_avoidance(self, v_cmd, my_eta, peer_eta, self_id, peer_id, all_peers):
        """ Zone 2 """
        cav3_in_zone1 = False
        cav3_eta = 999
        # CAV3 Á¤º¸ È®ÀÎ (Zone 1¿¡ ÀÖ´ÂÁö)
        if 3 in all_peers:
            p3 = all_peers[3]
            if p3['zone'] == 1:
                cav3_in_zone1 = True
                cav3_eta = float(p3['eta'])

        #³»°¡ CAV1ÀÏ ¶§
        if self_id == 1:
            if cav3_in_zone1:
                if my_eta < cav3_eta:
                    return v_cmd
                else:
                    return 0
            else:
                if peer_id == 2 and (my_eta > peer_eta):
                    return 0
                return v_cmd

        #³»°¡ CAV2ÀÏ ¶§
        if self_id == 2:
            if peer_id == 1:
                if cav3_in_zone1:
                    cav1_eta = peer_eta
                    if cav1_eta < cav3_eta:
                        return 0
                elif my_eta > peer_eta:
                    return 0
            return v_cmd

        # ³»°¡ CAV3ÀÎ °æ¿ì
        if self_id == 3:
            if peer_id == 1:
                if my_eta < peer_eta:
                    return v_cmd
                else:
                    return 0
            return v_cmd

        return v_cmd

    def _case_3_avoidance(self, v_cmd, my_eta, peer_eta, self_id, peer_id, all_peers):
        """ Zone 4 """
        cav4_in_zone3 = False
        cav4_eta = 999.0
        if 4 in all_peers:
            p4 = all_peers[4]
            if p4['zone'] == 3:
                cav4_in_zone3 = True
                cav4_eta = float(p4['eta'])

        if self_id == 1:
            if cav4_in_zone3:
                if my_eta < cav4_eta:
                    return v_cmd
                else:
                    return 0
            else:
                if peer_id == 2 and my_eta > peer_eta:
                    return 0
                return v_cmd

        if self_id == 2:
            if peer_id == 1:
                if cav4_in_zone3:
                    if peer_eta < cav4_eta:
                        return 0
                elif my_eta > peer_eta:
                    return 0
            return v_cmd

        if self_id == 4:
            if peer_id == 1:
                if my_eta < peer_eta:
                    return v_cmd
                else:
                    return 0
            return v_cmd

        return v_cmd

    def _case_4_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, self_id, peer_id, my_x, my_y, peer_x, peer_y):
        """ Zone 5,6,9,10 """
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
        return v_cmd

    # ----------------------------
    # Cases 5~6 (your logic, only fixed hv_data + replaced 2.0)
    # ----------------------------
    def _case_5_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, self_id, peer_id, my_x, my_y, peer_x, peer_y, my_flag, peer_flag, hv_data):
        """ Zone 7 """
        import math

        if 19 not in hv_data or 20 not in hv_data:
            return v_cmd

        x19 = float(hv_data[19][0]); y19 = float(hv_data[19][1])
        x20 = float(hv_data[20][0]); y20 = float(hv_data[20][1])
        dx = x19 - x20
        dy = y19 - y20

        dist_19 = math.sqrt((my_x - x19)**2 + (my_y - y19)**2)
        dist_20 = math.sqrt((my_x - x20)**2 + (my_y - y20)**2)

        if abs(dx) < 1e-6:
            hv_slope = float('inf')
        else:
            hv_slope = dy / dx

        if abs(hv_slope) > 1.0:
            if my_flag == 0 and peer_flag == 0:
                if my_eta < peer_eta:
                    return v_cmd
                else:
                    margin_time = peer_eta + 1.0
                    v_real = (my_eta * v_cmd) / margin_time
                    v_cmd = v_real
                    return v_cmd

            elif my_flag == 0 and peer_flag == 1:
                margin_time = peer_eta + 1.0
                v_real = (my_eta * v_cmd) / margin_time
                v_cmd = v_real
                return v_cmd

            elif my_flag == 1 and peer_flag == 0:
                if my_y < 1.5:
                    if abs(dist_19 - dist_20) < 0.2:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if dist_19 < dist_20:
                            target_hv_x = x19
                            nearest_dist = dist_19
                        else:
                            target_hv_x = x20
                            nearest_dist = dist_20
                        x_diff = my_x - target_hv_x
                        if x_diff < 0:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                        else:
                            if nearest_dist < 0.6:
                                return 0
                            else:
                                return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    return 0

            else:
                if my_y < 1.5:
                    if abs(dist_19 - dist_20) < 0.2:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if dist_19 < dist_20:
                            target_hv_x = x19
                            nearest_dist = dist_19
                        else:
                            target_hv_x = x20
                            nearest_dist = dist_20
                    x_diff = my_x - target_hv_x
                    if x_diff < 0:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if nearest_dist < 0.6:
                            return 0
                        else:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    return 0

        else:
            if my_flag == 0 and peer_flag == 0:
                if my_eta < peer_eta:
                    return v_cmd
                else:
                    margin_time = peer_eta + 1.0
                    v_real = (my_eta * v_cmd) / margin_time
                    v_cmd = v_real
                    return v_cmd

            elif my_flag == 0 and peer_flag == 1:
                margin_time = peer_eta + 1.0
                v_real = (my_eta * v_cmd) / margin_time
                v_cmd = v_real
                return v_cmd

            elif my_flag == 1 and peer_flag == 0:
                if abs(dist_19 - dist_20) < 0.2:
                    return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    if dist_19 < dist_20:
                        target_hv_x = x19
                        nearest_dist = dist_19
                    else:
                        target_hv_x = x20
                        nearest_dist = dist_20
                x_diff = my_x - target_hv_x
                if x_diff < 0:
                    return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    if nearest_dist < 0.6:
                        return 0
                    else:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)

            else:
                if my_y > 1.5 and peer_y > 1.5:
                    if self_id == 2:
                        if abs(dist_19 - dist_20) < 0.2:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                        else:
                            if dist_19 < dist_20:
                                target_hv_y = y19
                                nearest_dist = dist_19
                            else:
                                target_hv_y = y20
                                nearest_dist = dist_20
                        y_diff = my_y - target_hv_y
                        if y_diff < 0:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                        else:
                            if nearest_dist < 0.6:
                                return 0
                            else:
                                return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    elif self_id == 3:
                        return 0

                if my_y <= 1.5 and peer_y > 1.5:
                    if abs(dist_19 - dist_20) < 0.2:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if dist_19 < dist_20:
                            target_hv_y = y19
                            nearest_dist = dist_19
                        else:
                            target_hv_y = y20
                            nearest_dist = dist_20
                    y_diff = my_y - target_hv_y
                    if y_diff < 0:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if nearest_dist < 0.6:
                            return 0
                        else:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)

                if my_y > 1.5 and peer_y <= 1.5:
                    return 0

        return v_cmd

    def _case_6_avoidance(self, v_cmd, my_eta, peer_eta, my_lap, peer_lap, self_id, peer_id, my_x, my_y, peer_x, peer_y, my_flag, peer_flag, hv_data):
        """ Zone 8 """
        import math

        if 19 not in hv_data or 20 not in hv_data:
            return v_cmd

        x19 = float(hv_data[19][0]); y19 = float(hv_data[19][1])
        x20 = float(hv_data[20][0]); y20 = float(hv_data[20][1])
        dx = x19 - x20
        dy = y19 - y20

        dist_19 = math.sqrt((my_x - x19)**2 + (my_y - y19)**2)
        dist_20 = math.sqrt((my_x - x20)**2 + (my_y - y20)**2)

        if abs(dx) < 1e-6:
            hv_slope = float('inf')
        else:
            hv_slope = dy / dx

        if abs(hv_slope) < 1.0:
            if my_flag == 0 and peer_flag == 0:
                if my_eta < peer_eta:
                    return v_cmd
                else:
                    margin_time = peer_eta + 1.0
                    v_real = (my_eta * v_cmd) / margin_time
                    v_cmd = v_real
                    return v_cmd

            elif my_flag == 0 and peer_flag == 1:
                margin_time = peer_eta + 1.0
                v_real = (my_eta * v_cmd) / margin_time
                v_cmd = v_real
                return v_cmd

            elif my_flag == 1 and peer_flag == 0:
                if my_x > 0.3:
                    if abs(dist_19 - dist_20) < 0.2:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if dist_19 < dist_20:
                            target_hv_x = x19
                            nearest_dist = dist_19
                        else:
                            target_hv_x = x20
                            nearest_dist = dist_20
                        x_diff = my_x - target_hv_x
                        if x_diff < 0:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                        else:
                            if nearest_dist < 0.6:
                                return 0
                            else:
                                return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    return 0

            else:
                if my_x > 0.3:
                    if abs(dist_19 - dist_20) < 0.2:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if dist_19 < dist_20:
                            target_hv_y = y19
                            nearest_dist = dist_19
                        else:
                            target_hv_y = y20
                            nearest_dist = dist_20
                    y_diff = my_y - target_hv_y
                    if y_diff < 0:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if nearest_dist < 0.6:
                            return 0
                        else:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    return 0

        else:
            if my_flag == 0 and peer_flag == 0:
                if my_eta < peer_eta:
                    return v_cmd
                else:
                    margin_time = peer_eta + 1.0
                    v_real = (my_eta * v_cmd) / margin_time
                    v_cmd = v_real
                    return v_cmd

            elif my_flag == 0 and peer_flag == 1:
                margin_time = peer_eta + 1.0
                v_real = (my_eta * v_cmd) / margin_time
                v_cmd = v_real
                return v_cmd

            elif my_flag == 1 and peer_flag == 0:
                if abs(dist_19 - dist_20) < 0.2:
                    return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    if dist_19 < dist_20:
                        target_hv_x = x19
                        nearest_dist = dist_19
                    else:
                        target_hv_x = x20
                        nearest_dist = dist_20
                x_diff = my_x - target_hv_x
                if x_diff < 0:
                    return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    if nearest_dist < 0.6:
                        return 0
                    else:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)

            else:
                if my_x < 0.3 and peer_x < 0.3:
                    if self_id == 1:
                        if abs(dist_19 - dist_20) < 0.2:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                        else:
                            if dist_19 < dist_20:
                                target_hv_y = y19
                                nearest_dist = dist_19
                            else:
                                target_hv_y = y20
                                nearest_dist = dist_20
                        y_diff = my_y - target_hv_y
                        if y_diff < 0:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                        else:
                            if nearest_dist < 0.6:
                                return 0
                            else:
                                return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    elif self_id == 4:
                        return 0

                if my_x >= 0.3 and peer_x < 0.3:
                    if abs(dist_19 - dist_20) < 0.2:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if dist_19 < dist_20:
                            target_hv_y = y19
                            nearest_dist = dist_19
                        else:
                            target_hv_y = y20
                            nearest_dist = dist_20
                    y_diff = my_y - target_hv_y
                    if y_diff < 0:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                    else:
                        if nearest_dist < 0.6:
                            return 0
                        else:
                            return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)

                if my_x < 0.3 and peer_x >= 0.3:
                    return 0

        return v_cmd

    # ----------------------------
    # HV-only handling (your logic, only fixed hv_data + replaced 2.0)
    # ----------------------------
    def avoid_only_hv(self, zone_id, v_cmd, my_eta, my_lap, my_x, my_y, self_id, my_flag, all_peers, hv_data):
        import math

        if 19 not in hv_data or 20 not in hv_data:
            return v_cmd

        x19 = float(hv_data[19][0]); y19 = float(hv_data[19][1])
        x20 = float(hv_data[20][0]); y20 = float(hv_data[20][1])
        dx = x19 - x20
        dy = y19 - y20

        dist_19 = math.sqrt((my_x - x19)**2 + (my_y - y19)**2)
        dist_20 = math.sqrt((my_x - x20)**2 + (my_y - y20)**2)

        if abs(dx) < 1e-6:
            hv_slope = float('inf')
        else:
            hv_slope = dy / dx

        if zone_id == 7:
            if abs(hv_slope) > 1.0 and my_y > 1.5:
                if my_flag == 1:
                    return 0
                else:
                    return v_cmd

            if abs(dist_19 - dist_20) < 0.2:
                return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
            else:
                if dist_19 < dist_20:
                    target_hv_x = x19
                    nearest_dist = dist_19
                else:
                    target_hv_x = x20
                    nearest_dist = dist_20

                x_diff = my_x - target_hv_x
                if x_diff < 0:
                    return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    if nearest_dist < 0.6:
                        return 0.0
                    else:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)

        if zone_id == 8:
            if abs(hv_slope) < 1.0 and my_x < 0.3:
                if my_flag == 1:
                    return 0
                else:
                    return v_cmd

            if abs(dist_19 - dist_20) < 0.2:
                return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
            else:
                if dist_19 < dist_20:
                    target_hv_y = y19
                    nearest_dist = dist_19
                else:
                    target_hv_y = y20
                    nearest_dist = dist_20

                y_diff = my_y - target_hv_y
                if y_diff < 0:
                    return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)
                else:
                    if nearest_dist < 0.6:
                        return 0.0
                    else:
                        return (self._hv_go_speed(v_cmd, my_x, my_y, hv_data), True)

        return v_cmd
