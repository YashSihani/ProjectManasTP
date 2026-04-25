#!/usr/bin/env python3
import rclpy
import math
import sys
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D

# ── Tuning ────────────────────────────────────────────────────────────────────
FORWARD_SPEED = 1.2
TURN_SPEED = 0.5
TURN_TOL = 0.05
STOP_DIST = 0.85
DANGER_DIST = 0.70
SIDE_DETECT_DIST = 0.75

# ── Dual Transmission ─────────────────────────────────────────────────────────
LINEAR_SPEED_IN = 1.2
MAX_ANGULAR_IN = 1.5
TARGET_DIST_IN = 0.35

LINEAR_SPEED_OUT = 0.8
MAX_ANGULAR_OUT = 2.5
TARGET_DIST_OUT = 0.70

KP_INWARD_START = 0.15
KP_INWARD_END = 0.7
KP_ESCAPE_START = 0.3
KP_ESCAPE_END = 1.8

TOTAL_RINGS = 6
KD = 5.0
KP_ALIGN = 0.15
ALIGN_GATE = 0.10
DT = 0.05

# ── Gap / Dive / U-Turn ───────────────────────────────────────────────────────
PASSAGE_DIST = 1.3
GAP_CONFIRM_TICKS = 1
GAP_COOLDOWN  = 40
GAP_OVERSHOOT_M = 0.35
DIVE_PAUSE_TICKS = 10
DIVE_CROSS_PAUSE = 10
ABOUT_PAUSE_TICKS = 10
UTURN_SPEED = 1.0
UTURN_SPIN_TICKS = int(math.pi / UTURN_SPEED / DT)


class MazeExplorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pose_sub = self.create_subscription(Pose2D, '/estimated_pose', self.pose_cb, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.STATE_FOLLOW = 0
        self.STATE_ABOUT_TURN = 1
        self.STATE_DIVE_STOP = 2
        self.STATE_DIVE_TURN1 = 3
        self.STATE_DIVE_CROSS = 4
        self.STATE_DIVE_TURN2 = 5
        self.STATE_DIVE_ALIGN = 6
        self.STATE_ESCAPE_PAUSE = 7
        self.STATE_ESCAPE_TURN_RIGHT = 8

        self.current_state = self.STATE_DIVE_CROSS

        self.right_side_hug = -1.0
        self.target_yaw = 0.0
        self.prev_dist_err = 0.0
        self.current_ring = 1

        self.follow_ticks = 0
        self.gap_ticks = 0
        self.dive_ticks = 0
        self.about_ticks = 0
        self.startup_ticks = 0
        self.escape_pause_ticks = 0
        self.STARTUP_COOLDOWN = 20

        self.gap_detected = False
        self.gap_origin_x = 0.0
        self.gap_origin_y = 0.0
        self.dive_cross_pausing = False

        self.memory = []
        self.follow_rotated = 0.0
        self.follow_prev_yaw = 0.0
        self.is_escaping = False

        self.scan_received = False
        self.dist_front = float('inf')
        self.dist_side = float('inf')
        self.dist_side_f  = float('inf')
        self.dist_side_b = float('inf')

        # Separate escape-side sensors (outer wall)
        self.dist_side_esc = float('inf')
        self.dist_side_f_esc = float('inf')
        self.dist_side_b_esc = float('inf')

        self.create_timer(DT, self.control_loop)
        self.get_logger().info('Maze Explorer started in DIVE_CROSS.')

    def pose_cb(self, msg: Pose2D):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta

    def scan_cb(self, msg: LaserScan):
        self.scan_received = True

        if self.current_state in [self.STATE_ABOUT_TURN, self.STATE_ESCAPE_TURN_RIGHT]:
            return

        # Inner wall sensors (always computed — original working sign)
        inner_sign = -1.0 * self.right_side_hug
        self.dist_side = self._sector_min(msg,90.0 * inner_sign, 15)
        self.dist_side_f = self._sector_min(msg,60.0 * inner_sign, 15)
        self.dist_side_b = self._sector_min(msg,120.0 * inner_sign, 15)

        # Outer wall sensors (for escape — flipped sign relative to inner)
        outer_sign = -1.0 * self.right_side_hug
        self.dist_side_esc = self._sector_min(msg, 90.0 * outer_sign, 15)
        self.dist_side_f_esc = self._sector_min(msg, 60.0 * outer_sign, 15)
        self.dist_side_b_esc = self._sector_min(msg, 120.0 * outer_sign, 15)

        self.dist_front = self._sector_min(msg, 0.0, 15)

    def _sector_min(self, msg, center_deg, half_width_deg):
        half = math.radians(half_width_deg)
        cen = math.radians(center_deg)
        inc = msg.angle_increment
        amin = msg.angle_min
        n = len(msg.ranges)
        i0 = max(0,     int((cen - half - amin) / inc))
        i1 = min(n - 1, int((cen + half - amin) / inc))
        valid = [msg.ranges[i] for i in range(i0, i1 + 1)
                 if math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.10]
        return min(valid) if valid else float('inf')

    def _angle_err(self, target):
        err = target - self.current_yaw
        return math.atan2(math.sin(err), math.cos(err))

    def _travelled(self):
        dx = self.current_x - self.gap_origin_x
        dy = self.current_y - self.gap_origin_y
        return math.hypot(dx, dy)

    def _get_dynamic_kp(self):
        ring = min(max(self.current_ring, 1), TOTAL_RINGS)
        if self.is_escaping:
            return KP_ESCAPE_START + ((ring - 1) / (TOTAL_RINGS - 1.0)) * (KP_ESCAPE_END - KP_ESCAPE_START)
        else:
            return KP_INWARD_START + ((ring - 1) / (TOTAL_RINGS - 1.0)) * (KP_INWARD_END - KP_INWARD_START)

    def _set_turn_toward_inner(self):
        turn_rad = ((math.pi + 0.2) / 2.0) * (-self.right_side_hug)
        raw = self.current_yaw + turn_rad
        self.target_yaw = math.atan2(math.sin(raw), math.cos(raw))

    def _set_turn_away_from_inner(self):
        if self.is_escaping and len(self.memory) > 0:
            mem_val = self.memory.pop()
            self.get_logger().info(
                f'[ESCAPE] Popped {mem_val}. '
                f'Turn {"normal" if mem_val == 1 else "flipped"}.'
            )
            turn_rad = ((math.pi - 0.2) / 2.0) * self.right_side_hug * mem_val
        else:
            turn_rad = ((math.pi - 0.2) / 2.0) * self.right_side_hug
        raw = self.current_yaw + turn_rad
        self.target_yaw = math.atan2(math.sin(raw), math.cos(raw))

    def control_loop(self):
        if not self.scan_received:
            return

        msg = Twist()
        self.startup_ticks += 1

        # ==== RING 7 ESCAPE TRIGGER ===========================================
        if self.current_ring >= 7 and not self.is_escaping:
            self.get_logger().info('MAZE CENTER REACHED!')
            self.get_logger().info(f'MEMORY: {self.memory}')
            self.is_escaping = True
            self.current_state = self.STATE_ESCAPE_PAUSE
            self.escape_pause_ticks = 0
            self.cmd_pub.publish(Twist())
            return

        # ==== GLOBAL FRONT WALL GUARD =========================================
        if (self.current_state == self.STATE_FOLLOW and
                self.dist_front != float('inf') and
                self.dist_front < DANGER_DIST):
            self.get_logger().info(f'Dead end → ABOUT_TURN')
            self.about_ticks = 0
            self.gap_detected = False
            self.gap_ticks = 0
            self.prev_dist_err = 0.0
            self.current_state = self.STATE_ABOUT_TURN
            self.cmd_pub.publish(Twist())
            return

        # ==== FOLLOW ==========================================================
        if self.current_state == self.STATE_FOLLOW:
            self.follow_ticks += 1

            raw_delta = self.current_yaw - self.follow_prev_yaw
            delta = math.atan2(math.sin(raw_delta), math.cos(raw_delta))
            self.follow_rotated  += abs(delta)
            self.follow_prev_yaw  = self.current_yaw

            current_linear = LINEAR_SPEED_OUT if self.is_escaping else LINEAR_SPEED_IN
            current_max_ang = MAX_ANGULAR_OUT  if self.is_escaping else MAX_ANGULAR_IN
            current_target_dist = TARGET_DIST_OUT  if self.is_escaping else TARGET_DIST_IN

            # Pick the right sensor set depending on direction
            # Inward: use inner wall sensors (dist_side/f/b)
            # Escape: use outer wall sensors (dist_side_esc/f/b)
            if self.is_escaping:
                sense = self.dist_side_esc
                sense_f = self.dist_side_f_esc
                sense_b = self.dist_side_b_esc
            else:
                sense = self.dist_side
                sense_f = self.dist_side_f
                sense_b = self.dist_side_b

            # ── Gap detection ──────────────────────────────────────────────────
            # Inward: active after cooldown | Escape: always active
            gap_active = True if self.is_escaping else (self.follow_ticks > GAP_COOLDOWN)

            if gap_active:
                if not self.gap_detected:
                    if sense > PASSAGE_DIST:
                        self.gap_ticks += 1
                    else:
                        self.gap_ticks = 0
                    if self.gap_ticks >= GAP_CONFIRM_TICKS:
                        self.gap_detected = True
                        self.gap_origin_x = self.current_x
                        self.gap_origin_y = self.current_y
                        self.get_logger().info(
                            f'GAP confirmed ({"outer" if self.is_escaping else "inner"} '
                            f'dist={sense:.2f}m). Driving {GAP_OVERSHOOT_M}m.'
                        )
                else:
                    if self._travelled() >= GAP_OVERSHOOT_M:
                        if not self.is_escaping:
                            if self.follow_rotated <= math.pi:
                                self.memory.append(1)
                                self.get_logger().info(
                                    f'Short path ({math.degrees(self.follow_rotated):.0f}°) → [1]'
                                )
                            else:
                                self.memory.append(-1)
                                self.get_logger().info(
                                    f'Long path ({math.degrees(self.follow_rotated):.0f}°) → [-1]'
                                )
                        self.gap_detected = False
                        self.gap_ticks = 0
                        self.dive_ticks = 0
                        self.current_state = self.STATE_DIVE_STOP
                        self.cmd_pub.publish(Twist())
                        return
                    else:
                        msg.linear.x  = current_linear * 0.6
                        msg.angular.z = 0.0
                        self.cmd_pub.publish(msg)
                        return

            # ── PD Control ────────────────────────────────────────────────────
            # Inward: steer away from inner wall → -steering * right_side_hug
            # Escape: steer away from outer wall → +steering * right_side_hug
            if sense == float('inf'):
                msg.linear.x = current_linear * 0.5
                self.prev_dist_err = 0.0
            else:
                dist_err = sense - current_target_dist
                d_err    = (dist_err - self.prev_dist_err) / DT
                self.prev_dist_err = dist_err
                steering = (self._get_dynamic_kp() * dist_err) + (KD * d_err)
                if sense_b != float('inf') and sense_f != float('inf'):
                    align_err = sense_f - sense_b
                    if abs(align_err) < ALIGN_GATE:
                        steering += KP_ALIGN * align_err
                raw_angular = -steering * self.right_side_hug
                turn_ratio = abs(raw_angular) / current_max_ang
                msg.linear.x = current_linear * (1.0 - 0.5 * turn_ratio)
                msg.angular.z = max(min(raw_angular, current_max_ang), -current_max_ang)

        # ==== ABOUT_TURN ======================================================
        elif self.current_state == self.STATE_ABOUT_TURN:
            self.about_ticks += 1
            if self.about_ticks <= ABOUT_PAUSE_TICKS:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            elif self.about_ticks <= ABOUT_PAUSE_TICKS + UTURN_SPIN_TICKS:
                msg.linear.x = 0.0
                msg.angular.z = UTURN_SPEED * self.right_side_hug
            else:
                self.right_side_hug *= -1.0
                self.get_logger().info(
                    f'ABOUT_TURN done → hugging '
                    f'{"LEFT" if self.right_side_hug == -1.0 else "RIGHT"} → FOLLOW'
                )
                self.prev_dist_err = 0.0
                self.follow_ticks = 0
                self.gap_ticks = 0
                self.gap_detected = False
                self.follow_rotated = 0.0
                self.follow_prev_yaw = self.current_yaw
                self.current_state = self.STATE_FOLLOW

        # ==== ESCAPE_PAUSE ====================================================
        elif self.current_state == self.STATE_ESCAPE_PAUSE:
            self.escape_pause_ticks += 1
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.escape_pause_ticks >= int(2.0 / DT):
                self.get_logger().info('2s pause done > 90° right > DIVE_CROSS.')
                self.target_yaw = math.atan2(
                    math.sin(self.current_yaw - math.pi / 2.0),
                    math.cos(self.current_yaw - math.pi / 2.0)
                )
                self.current_state = self.STATE_ESCAPE_TURN_RIGHT

        # ==== ESCAPE_TURN_RIGHT ===============================================
        elif self.current_state == self.STATE_ESCAPE_TURN_RIGHT:
            err = self._angle_err(self.target_yaw)
            if abs(err) < TURN_TOL:
                self.get_logger().info('90° right done > DIVE_CROSS.')
                self.dive_cross_pausing = False
                self.dive_ticks = 0
                self.startup_ticks = 0
                self.current_state = self.STATE_DIVE_CROSS
            else:
                msg.linear.x = 0.0
                msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ==== DIVE_STOP =======================================================
        elif self.current_state == self.STATE_DIVE_STOP:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.dive_ticks += 1
            if self.dive_ticks >= DIVE_PAUSE_TICKS:
                self.dive_ticks = 0
                self._set_turn_toward_inner()
                self.current_state = self.STATE_DIVE_TURN1

        # ==== DIVE_TURN1 ======================================================
        elif self.current_state == self.STATE_DIVE_TURN1:
            err = self._angle_err(self.target_yaw)
            if abs(err) < TURN_TOL:
                self.dive_cross_pausing = False
                self.dive_ticks = 0
                self.startup_ticks = 0
                self.current_state = self.STATE_DIVE_CROSS
            else:
                msg.linear.x = 0.0
                msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ==== DIVE_CROSS ======================================================
        elif self.current_state == self.STATE_DIVE_CROSS:
            if self.dive_cross_pausing:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.dive_ticks += 1
                if self.dive_ticks >= DIVE_CROSS_PAUSE:
                    self.dive_ticks = 0
                    self._set_turn_away_from_inner()
                    self.current_state = self.STATE_DIVE_TURN2
            else:
                if (self.startup_ticks > self.STARTUP_COOLDOWN and
                        self.dist_front != float('inf') and
                        self.dist_front < STOP_DIST):
                    self.get_logger().info('DIVE_CROSS: wall reached → pausing.')
                    self.dive_cross_pausing = True
                    self.dive_ticks         = 0
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.linear.x = FORWARD_SPEED * 0.6
                    msg.angular.z = 0.0

        # ==== DIVE_TURN2 ======================================================
        elif self.current_state == self.STATE_DIVE_TURN2:
            err = self._angle_err(self.target_yaw)
            if abs(err) < TURN_TOL:
                self.prev_dist_err = 0.0
                self.gap_ticks     = 0
                self.gap_detected  = False
                self.current_state = self.STATE_DIVE_ALIGN
            else:
                msg.linear.x = 0.0
                msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ==== DIVE_ALIGN ======================================================
        elif self.current_state == self.STATE_DIVE_ALIGN:
            # Use correct sensor set for alignment
            if self.is_escaping:
                align_err = self.dist_side_f_esc - self.dist_side_b_esc
                sf = self.dist_side_f_esc
                sb = self.dist_side_b_esc
            else:
                align_err = self.dist_side_f - self.dist_side_b
                sf = self.dist_side_f
                sb = self.dist_side_b

            if abs(align_err) < 0.03 or sf == float('inf') or sb == float('inf'):
                if self.is_escaping:
                    self.current_ring -= 1
                    if self.current_ring <= 0:
                        self.get_logger().info('MAZE ESCAPED SUCCESSFULLY!')
                        self.cmd_pub.publish(Twist())
                        rclpy.shutdown()
                        sys.exit()
                    self.get_logger().info(f'Aligned! → Ring {self.current_ring} → FOLLOW')
                else:
                    self.current_ring += 1
                    self.get_logger().info(f'Aligned! → Ring {self.current_ring} → FOLLOW')

                self.prev_dist_err = 0.0
                self.follow_ticks = 0
                self.gap_ticks = 0
                self.gap_detected = False
                self.follow_rotated = 0.0
                self.follow_prev_yaw = self.current_yaw
                self.current_state = self.STATE_FOLLOW
            else:
                msg.linear.x  = 0.0
                current_max_ang = MAX_ANGULAR_OUT if self.is_escaping else MAX_ANGULAR_IN
                raw_angular = -(align_err * 1.5) * self.right_side_hug
                msg.angular.z = max(min(raw_angular, 0.4), -current_max_ang)

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()