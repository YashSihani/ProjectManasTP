#!/usr/bin/env python3
"""
Circular Maze Explorer — 0.7m Guard & Formula U-Turn
=========================================================
States:
  FOLLOW        — PD wall hugging along inner ring
  ABOUT_TURN    — 0.5s pause -> tick-formula 180° spin -> FOLLOW
  DIVE_STOP     — gap confirmed, travels GAP_OVERSHOOT_M then stops 0.5s
  DIVE_TURN1    — turn 90° toward inner side
  DIVE_CROSS    — drive straight until front wall < STOP_DIST (STARTING STATE)
  DIVE_TURN2    — turn 90° back to realign
  DIVE_ALIGN    — rotate until parallel to curved wall -> FOLLOW

Startup:
  Bot spawns in STATE_DIVE_CROSS. Drives straight into the maze passage
  until it reads the inner wall at STOP_DIST (0.85m), then realigns.
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D

# ── Tuning ────────────────────────────────────────────────────────────────────
LINEAR_SPEED = 1.2
FORWARD_SPEED = 1.2
TURN_SPEED = 0.5
TURN_TOL = 0.05
STOP_DIST = 0.85  # Stops DIVE_CROSS safely for 90° turns
DANGER_DIST = 0.70  # Triggers ABOUT_TURN emergency 180° spin at 0.7m
SIDE_DETECT_DIST = 0.75
TARGET_DIST = 0.35
DANGER_ANGULAR = 0.15
MAX_ANGULAR = 1.5

# ── Dynamic KP Tuning ─────────────────────────────────────────────────────────
KP_START = 0.15
KP_END = 0.7
TOTAL_RINGS = 6

KD = 5.0
KP_ALIGN = 0.15
ALIGN_GATE = 0.10
DT = 0.05

# ── Gap / Dive / U-Turn Tuning ────────────────────────────────────────────────
PASSAGE_DIST = 1.3
GAP_CONFIRM_TICKS = 1
GAP_COOLDOWN = 40
GAP_OVERSHOOT_M = 0.35
DIVE_PAUSE_TICKS = 10  # 0.5s pause
DIVE_CROSS_PAUSE = 10  # 0.5s pause
ABOUT_PAUSE_TICKS = 10  # 0.5s pause inside the ABOUT_TURN state before spinning

# ── The Formula ───────────────────────────────────────────────────────────────
UTURN_SPEED = 1.0    # rad/s commanded
UTURN_SPIN_TICKS = int((2.6*math.pi) / UTURN_SPEED / DT)


class MazeExplorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pose_sub = self.create_subscription(Pose2D, '/estimated_pose', self.pose_cb, 10)

        # ── Pose ──────────────────────────────────────────────────────────────
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # ── States ────────────────────────────────────────────────────────────
        self.STATE_FOLLOW = 0
        self.STATE_ABOUT_TURN = 1
        self.STATE_DIVE_STOP = 2
        self.STATE_DIVE_TURN1 = 3
        self.STATE_DIVE_CROSS = 4
        self.STATE_DIVE_TURN2 = 5
        self.STATE_DIVE_ALIGN = 6

        # Spawn directly into DIVE_CROSS to enter the maze
        self.current_state = self.STATE_DIVE_CROSS

        # ── Navigation Locks & Targets ────────────────────────────────────────
        self.right_side_hug = -1.0
        self.target_yaw = 0.0

        # ── PD state & Ring Tracking ──────────────────────────────────────────
        self.prev_dist_err = 0.0
        self.current_ring = 1

        # ── Counters ──────────────────────────────────────────────────────────
        self.follow_ticks = 0
        self.gap_ticks = 0
        self.dive_ticks = 0
        self.about_ticks = 0
        self.startup_ticks = 0
        self.STARTUP_COOLDOWN = 20  # Prevent premature wall detection on spawn

        # ── Gap overshoot tracking ─────────────────────────────────────────────
        self.gap_detected = False
        self.gap_origin_x = 0.0
        self.gap_origin_y = 0.0
        self.dive_cross_pausing = False

        # ── Sensors ───────────────────────────────────────────────────────────
        self.scan_received = False
        self.dist_front = float('inf')
        self.dist_side = float('inf')
        self.dist_side_f = float('inf')
        self.dist_side_b = float('inf')

        self.create_timer(DT, self.control_loop)
        self.get_logger().info('Maze Explorer started directly in DIVE_CROSS.')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def pose_cb(self, msg: Pose2D):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta

    def scan_cb(self, msg: LaserScan):
        self.scan_received = True

        # Freeze side sensors during ABOUT_TURN to prevent bad readings mid-spin
        if self.current_state == self.STATE_ABOUT_TURN:
            return

        sign = -1.0 * self.right_side_hug
        self.dist_side = self._sector_min(msg, 90.0 * sign, 15)
        self.dist_side_f = self._sector_min(msg, 60.0 * sign, 15)
        self.dist_side_b = self._sector_min(msg, 120.0 * sign, 15)
        self.dist_front = self._sector_min(msg, 0.0, 15)

    def _sector_min(self, msg: LaserScan, center_deg: float, half_width_deg: float) -> float:
        half = math.radians(half_width_deg)
        cen = math.radians(center_deg)
        inc = msg.angle_increment
        amin = msg.angle_min
        n = len(msg.ranges)
        i0 = max(0, int((cen - half - amin) / inc))
        i1 = min(n - 1, int((cen + half - amin) / inc))
        valid = [msg.ranges[i] for i in range(i0, i1 + 1)
                 if math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.10]
        return min(valid) if valid else float('inf')

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _angle_err(self, target: float) -> float:
        err = target - self.current_yaw
        return math.atan2(math.sin(err), math.cos(err))

    def _travelled(self) -> float:
        dx = self.current_x - self.gap_origin_x
        dy = self.current_y - self.gap_origin_y
        return math.hypot(dx, dy)

    def _get_dynamic_kp(self) -> float:
        ring = min(max(self.current_ring, 1), TOTAL_RINGS)
        return KP_START + ((ring - 1) / (TOTAL_RINGS - 1.0)) * (KP_END - KP_START)

    def _set_turn_toward_inner(self):
        turn_rad = ((math.pi + 0.2) / 2.0) * (-self.right_side_hug)
        raw = self.current_yaw + turn_rad
        self.target_yaw = math.atan2(math.sin(raw), math.cos(raw))

    def _set_turn_away_from_inner(self):
        turn_rad = ((math.pi - 0.2) / 2.0) * self.right_side_hug
        raw = self.current_yaw + turn_rad
        self.target_yaw = math.atan2(math.sin(raw), math.cos(raw))

    # ── Control loop ──────────────────────────────────────────────────────────
    def control_loop(self):
        if not self.scan_received:
            return

        msg = Twist()
        self.startup_ticks += 1

        # ==== GLOBAL FRONT WALL GUARD (Highest Priority in FOLLOW) ============
        if (self.current_state == self.STATE_FOLLOW and
                self.dist_front != float('inf') and
                self.dist_front < DANGER_DIST):
            self.get_logger().info('Obstacle < 0.70m! HARD BRAKE -> ABOUT_TURN')
            self.about_ticks = 0
            self.gap_detected = False
            self.gap_ticks = 0
            self.prev_dist_err = 0.0
            self.current_state = self.STATE_ABOUT_TURN
            self.cmd_pub.publish(Twist())  # Immediate hardware stop
            return

        # ==== FOLLOW ==========================================================
        if self.current_state == self.STATE_FOLLOW:
            self.follow_ticks += 1

            # ── 1. Gap detection ───────────────────────────────────────────────
            if self.follow_ticks > GAP_COOLDOWN:
                if not self.gap_detected:
                    if self.dist_side > PASSAGE_DIST:
                        self.gap_ticks += 1
                    else:
                        self.gap_ticks = 0

                    if self.gap_ticks >= GAP_CONFIRM_TICKS:
                        self.gap_detected = True
                        self.gap_origin_x = self.current_x
                        self.gap_origin_y = self.current_y
                        self.get_logger().info(f'GAP confirmed. Driving {GAP_OVERSHOOT_M}m to centre.')
                else:
                    if self._travelled() >= GAP_OVERSHOOT_M:
                        self.gap_detected = False
                        self.gap_ticks = 0
                        self.dive_ticks = 0
                        self.current_state = self.STATE_DIVE_STOP
                        self.cmd_pub.publish(Twist())
                        return
                    else:
                        msg.linear.x = FORWARD_SPEED * 0.6
                        msg.angular.z = 0.0
                        self.cmd_pub.publish(msg)
                        return

            # ── 2. PD Control ──────────────────────────────────────────────────
            if self.dist_side == float('inf'):
                msg.linear.x = LINEAR_SPEED * 0.5
                self.prev_dist_err = 0.0
            else:
                dist_err = self.dist_side - TARGET_DIST
                d_err = (dist_err - self.prev_dist_err) / DT
                self.prev_dist_err = dist_err

                dynamic_kp = self._get_dynamic_kp()
                steering = (dynamic_kp * dist_err) + (KD * d_err)

                if (self.dist_side_b != float('inf') and self.dist_side_f != float('inf')):
                    align_err = self.dist_side_f - self.dist_side_b
                    if abs(align_err) < ALIGN_GATE:
                        steering += KP_ALIGN * align_err

                raw_angular = -steering * self.right_side_hug
                turn_ratio = abs(raw_angular) / MAX_ANGULAR
                msg.linear.x = LINEAR_SPEED * (1.0 - 0.5 * turn_ratio)
                msg.angular.z = max(min(raw_angular, MAX_ANGULAR), -MAX_ANGULAR)

        # ==== ABOUT_TURN (0.5s Pause -> Formula Tick Spin) ====================
        elif self.current_state == self.STATE_ABOUT_TURN:
            self.about_ticks += 1

            if self.about_ticks <= ABOUT_PAUSE_TICKS:
                # PAUSE PHASE: Hold perfectly still for 0.5 seconds
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            elif self.about_ticks <= ABOUT_PAUSE_TICKS + UTURN_SPIN_TICKS:
                # SPIN PHASE: Spins safely away from the wall using the calculated ticks
                msg.linear.x = 0.0
                msg.angular.z = UTURN_SPEED * self.right_side_hug
            else:
                self.get_logger().info('180° complete via timer. Flipping side -> FOLLOW')
                self.right_side_hug *= -1.0
                self.prev_dist_err = 0.0
                self.follow_ticks = 0
                self.gap_ticks = 0
                self.gap_detected = False
                self.current_state = self.STATE_FOLLOW

        # ==== DIVE_STOP (pause 0.5s) ==========================================
        elif self.current_state == self.STATE_DIVE_STOP:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.dive_ticks += 1
            if self.dive_ticks >= DIVE_PAUSE_TICKS:
                self.dive_ticks = 0
                self._set_turn_toward_inner()
                self.current_state = self.STATE_DIVE_TURN1

        # ==== DIVE_TURN1 (90° toward inner) ===================================
        elif self.current_state == self.STATE_DIVE_TURN1:
            err = self._angle_err(self.target_yaw)
            if abs(err) < TURN_TOL:
                self.dive_cross_pausing = False
                self.dive_ticks = 0
                self.current_state = self.STATE_DIVE_CROSS
            else:
                msg.linear.x = 0.0
                msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ==== DIVE_CROSS (Start State) ========================================
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
                    self.get_logger().info('DIVE_CROSS: inner wall reached -> pausing.')
                    self.dive_cross_pausing = True
                    self.dive_ticks = 0
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.linear.x = FORWARD_SPEED * 0.6
                    msg.angular.z = 0.0

        # ==== DIVE_TURN2 (90° away from inner wall) ===========================
        elif self.current_state == self.STATE_DIVE_TURN2:
            err = self._angle_err(self.target_yaw)
            if abs(err) < TURN_TOL:
                self.prev_dist_err = 0.0
                self.gap_ticks = 0
                self.gap_detected = False
                self.current_state = self.STATE_DIVE_ALIGN
            else:
                msg.linear.x = 0.0
                msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ==== DIVE_ALIGN (rotate until parallel to curved wall) ===============
        elif self.current_state == self.STATE_DIVE_ALIGN:
            err = self.dist_side_f - self.dist_side_b
            if (abs(err) < 0.03 or
                    self.dist_side_f == float('inf') or
                    self.dist_side_b == float('inf')):
                self.current_ring += 1
                self.get_logger().info(f'Aligned! Ring {self.current_ring} → FOLLOW')
                self.prev_dist_err = 0.0
                self.follow_ticks = 0
                self.gap_ticks = 0
                self.gap_detected = False
                self.current_state = self.STATE_FOLLOW
            else:
                msg.linear.x = 0.0
                raw_angular = -(err * 1.5) * self.right_side_hug
                msg.angular.z = max(min(raw_angular, 0.4), -MAX_ANGULAR)

        self.cmd_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────
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
