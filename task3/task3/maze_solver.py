#!/usr/bin/env python3
"""
Circular Maze Explorer — Final Version
=======================================
States:
  FORWARD  — drive straight until wall or side wall detected
  ENTRY    — stabilise after detecting a side wall
  FOLLOW   — PD wall hugging along outer ring
  TURNING  — yaw-based 90 degree right turn on front wall
  DIVE     — 3-phase passage entry into inner ring

Key design decisions:
  - right_side_hug = 1.0  → hugging RIGHT (outer) wall at start
  - inner_side     = -1.0 → inner wall is always on LEFT at start
  - After each dive: inner_side = -right_side_hug (always opposite)
  - Dive turn angle = 75 degrees (shallower than 90, avoids wall collision)
  - All turns use yaw feedback from /estimated_pose — no tick counting
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D


# ── Tuning ────────────────────────────────────────────────────────────────────
LINEAR_SPEED     = 0.99    # m/s — FOLLOW speed
FORWARD_SPEED    = 0.99    # m/s — FORWARD speed
TURN_SPEED       = 0.5    # rad/s — all turns (slow to avoid overshoot)
TURN_TOL         = 0.03    # rad (~2°) — yaw error to consider turn done
STOP_DIST        = 0.85    # m — front wall triggers TURNING
CLEAR_DIST       = 0.80    # m — front must be clear before leaving TURNING
SIDE_DETECT_DIST = 0.75    # m — side wall detected → ENTRY
TARGET_DIST      = 0.4    # m — desired distance from wall in FOLLOW
PASSAGE_DIST     = 1.0     # m — inner side gap triggers DIVE
DANGER_DIST      = 0.50    # m — emergency push away from wall
DANGER_ANGULAR   = 0.15    # rad/s — emergency angular correction
MAX_ANGULAR      = 0.15    # rad/s — clamp on steering output
KP               = 0.19     # proportional gain
KD               = 5     # derivative gain
KP_ALIGN         = 0.15    # alignment gain
ALIGN_GATE       = 0.10    # m — align correction only inside this band
ENTRY_TICKS_MAX  = 20      # ticks before forcing FOLLOW from ENTRY
DT               = 0.05    # s — control loop period
STOP_TICKS       = 10      # ticks to pause in DIVE Phase 0 (= 0.5s)
DIVE_ANGLE_DEG   = 90.0    # degrees — shallower than 90 to avoid wall hit
DIVE_SPEED       = 0.4     # fraction of LINEAR_SPEED during DIVE Phase 2



class MazeExplorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.pose_sub = self.create_subscription(
            Pose2D, '/estimated_pose', self.pose_cb, 10)

        # ── Pose ──────────────────────────────────────────────────────────────
        self.current_yaw = 0.0   # radians from Pose2D.theta

        # ── States ────────────────────────────────────────────────────────────
        self.STATE_FORWARD = 0
        self.STATE_FOLLOW  = 1
        self.STATE_TURNING = 2
        self.STATE_ENTRY   = 3
        self.STATE_DIVE    = 4
        self.current_state = self.STATE_FORWARD

        # ── Side tracking ─────────────────────────────────────────────────────
        self.right_side_hug = -1.0    # 1.0 = right wall  -1.0 = left wall
        self.inner_side     = -1.0   # -1.0 = inner wall on LEFT at start

        # ── Shared turn target ────────────────────────────────────────────────
        self.target_yaw = 0.0

        # ── PD state ──────────────────────────────────────────────────────────
        self.prev_dist_err = 0.0

        # ── Entry stabilisation ───────────────────────────────────────────────
        self.entry_ticks = 0

        # ── Dive ──────────────────────────────────────────────────────────────
        self.dive_phase = 0
        self.dive_ticks = 0

        # ── Sensor cache ──────────────────────────────────────────────────────
        self.dist_front  = float('inf')
        self.dist_inner  = float('inf')
        self.dist_side   = float('inf')
        self.dist_side_f = float('inf')
        self.dist_side_b = float('inf')

        self.create_timer(DT, self.control_loop)
        self.get_logger().info('Maze Explorer started.')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def pose_cb(self, msg: Pose2D):
        self.current_yaw = msg.theta

    def scan_cb(self, msg: LaserScan):
        self.dist_front = self._sector_min(msg, 0.0, 15)

        # Follow-side sensors
        sign = -1.0 * self.right_side_hug
        self.dist_side   = self._sector_min(msg,  90.0 * sign, 15)
        self.dist_side_f = self._sector_min(msg,  60.0 * sign, 15)
        self.dist_side_b = self._sector_min(msg, 120.0 * sign, 15)

        # Inner-side sensor — always opposite of hugging side
        inner_sign = -1.0 * self.inner_side
        self.dist_inner = self._sector_min(msg, 90.0 * inner_sign, 15)

    def _sector_min(self, msg: LaserScan, center_deg: float, half_width_deg: float) -> float:
        half  = math.radians(half_width_deg)
        cen   = math.radians(center_deg)
        inc   = msg.angle_increment
        amin  = msg.angle_min
        n     = len(msg.ranges)
        i0    = max(0,     int((cen - half - amin) / inc))
        i1    = min(n - 1, int((cen + half - amin) / inc))
        valid = [msg.ranges[i] for i in range(i0, i1 + 1)
                 if math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.10]
        return min(valid) if valid else float('inf')

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _angle_err(self, target: float) -> float:
        """Signed shortest difference target - current, wrapped to [-pi, pi]."""
        err = target - self.current_yaw
        return math.atan2(math.sin(err), math.cos(err))

    def _set_turn_right_90(self):
        """Target yaw = 90 degrees to the RIGHT of current heading."""
        raw = self.current_yaw - (2.5 / 2.0)
        self.target_yaw = math.atan2(math.sin(raw), math.cos(raw))
        self.get_logger().info(
            f'TURNING right 90° → target {math.degrees(self.target_yaw):.1f}°'
        )

    def _set_dive_turn(self):
        """Target yaw = DIVE_ANGLE_DEG toward inner_side."""
        raw = self.current_yaw - math.radians(DIVE_ANGLE_DEG) * self.inner_side
        self.target_yaw = math.atan2(math.sin(raw), math.cos(raw))
        self.get_logger().info(
            f'DIVE turn {DIVE_ANGLE_DEG}° → target {math.degrees(self.target_yaw):.1f}°  '
            f'inner_side={self.inner_side}'
        )

    # ── Control loop ──────────────────────────────────────────────────────────
    def control_loop(self):
        if self.dist_front == float('inf'):
            return

        msg = Twist()

        # ==== FORWARD =========================================================
        if self.current_state == self.STATE_FORWARD:
            if self.dist_front < STOP_DIST:
                self._set_turn_right_90()
                self.prev_dist_err = 0.0
                self.current_state = self.STATE_TURNING

            elif self.dist_side < SIDE_DETECT_DIST:
                self.entry_ticks   = 0
                self.prev_dist_err = 0.0
                self.current_state = self.STATE_ENTRY

            else:
                msg.linear.x = FORWARD_SPEED

        # ==== ENTRY ===========================================================
        elif self.current_state == self.STATE_ENTRY:
            if self.dist_front < STOP_DIST:
                self._set_turn_right_90()
                self.prev_dist_err = 0.0
                self.current_state = self.STATE_TURNING
            else:
                self.entry_ticks += 1
                msg.linear.x = FORWARD_SPEED * 0.6

                both_walls = (self.dist_side_f < SIDE_DETECT_DIST and
                              self.dist_side_b < SIDE_DETECT_DIST)
                if both_walls or self.entry_ticks >= ENTRY_TICKS_MAX:
                    self.prev_dist_err = 0.0
                    self.current_state = self.STATE_FOLLOW

        # ==== FOLLOW ==========================================================
        elif self.current_state == self.STATE_FOLLOW:

            if self.dist_front < STOP_DIST:
                self._set_turn_right_90()
                self.prev_dist_err = 0.0
                self.current_state = self.STATE_TURNING

            # Passage detection — guard ensures we are already wall-following
            elif (self.dist_inner > PASSAGE_DIST and
                  self.dist_side  < SIDE_DETECT_DIST):
                self.get_logger().info(
                    f'Passage detected! dist_inner={self.dist_inner:.2f} — starting DIVE.'
                )
                self.dive_phase    = 0
                self.dive_ticks    = 0
                self.prev_dist_err = 0.0
                self.current_state = self.STATE_DIVE

            elif self.dist_side > SIDE_DETECT_DIST + 0.15:
                self.prev_dist_err = 0.0
                self.current_state = self.STATE_FORWARD

            elif self.dist_side == float('inf'):
                msg.linear.x       = LINEAR_SPEED * 0.5
                self.prev_dist_err = 0.0

            else:
                dist_err = self.dist_side - TARGET_DIST
                d_err    = (dist_err - self.prev_dist_err) / DT
                self.prev_dist_err = dist_err

                steering = (KP * dist_err) + (KD * d_err)

                if (self.dist_side_b != float('inf') and
                        self.dist_side_f != float('inf')):
                    align_err = self.dist_side_b - self.dist_side_f
                    if abs(align_err) < ALIGN_GATE:
                        steering += KP_ALIGN * align_err

                raw_angular = -steering * self.right_side_hug

                if self.dist_side < DANGER_DIST:
                    msg.linear.x       = 0.05
                    msg.angular.z      = DANGER_ANGULAR * self.right_side_hug
                    self.prev_dist_err = 0.0
                else:
                    turn_ratio    = abs(raw_angular) / MAX_ANGULAR
                    msg.linear.x  = LINEAR_SPEED * (1.0 - 0.5 * turn_ratio)
                    msg.angular.z = max(min(raw_angular, MAX_ANGULAR), -MAX_ANGULAR)

        # ==== DIVE ============================================================
        elif self.current_state == self.STATE_DIVE:
            self.dive_ticks += 1

            # PHASE 0 — stop in place for STOP_TICKS × DT seconds
            if self.dive_phase == 0:
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                if self.dive_ticks >= STOP_TICKS:
                    self._set_dive_turn()   # record target yaw after fully stopped
                    self.dive_phase = 1
                    self.dive_ticks = 0

            # PHASE 1 — rotate toward inner wall using yaw feedback
            elif self.dive_phase == 1:
                msg.linear.x = 0.0
                err = self._angle_err(self.target_yaw)
                if abs(err) < TURN_TOL:
                    msg.angular.z   = 0.0
                    self.dive_phase = 2
                    self.dive_ticks = 0
                    self.get_logger().info('DIVE turn complete — driving inward.')
                else:
                    msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

            # PHASE 2 — drive straight into the passage at reduced speed
            elif self.dive_phase == 2:
                if self.dist_front < STOP_DIST:
                    # Reached inner ring wall
                    # inner_side is always opposite of the wall we now hug
                    self.inner_side    = -self.right_side_hug
                    self.entry_ticks   = 0
                    self.prev_dist_err = 0.0
                    self.current_state = self.STATE_ENTRY
                    self.get_logger().info(
                        f'DIVE complete — new ring. inner_side={self.inner_side}'
                    )
                else:
                    msg.linear.x  = LINEAR_SPEED * DIVE_SPEED
                    msg.angular.z = 0.0

        # ==== TURNING =========================================================
        elif self.current_state == self.STATE_TURNING:
            err = self._angle_err(self.target_yaw)
            if abs(err) < TURN_TOL:
                if self.dist_front > CLEAR_DIST:
                    self.get_logger().info('Turn complete — resuming FORWARD.')
                    self.prev_dist_err = 0.0
                    self.current_state = self.STATE_FORWARD
                # else stay put until front clears
            else:
                msg.linear.x  = 0.0
                msg.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        self.cmd_pub.publish(msg)


# ── Entry point ──────────────────────────────────────────────────────────────
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