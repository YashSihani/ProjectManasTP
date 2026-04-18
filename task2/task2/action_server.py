#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from my_custom_interfaces.action import MoveRobot
import math
import time

LINEAR_SPEED = 0.8    # m/s  — forward speed
TURN_SPEED = 0.8    # rad/s — turning speed
FRONT_BRAKE_DIST = 0.6   # m    — obstacle trigger distance (front)
SIDE_CLEAR_DIST = 0.62    # m    — shoulder distance to consider obstacle cleared
SIDE_WALL_DIST = 0.62  # m    — wall still present threshold
GOAL_TOL = 0.05   # m    — per-axis goal tolerance
ALIGN_TOL = 0.05   # rad  — heading tolerance (must be > 1 tick = 0.02 rad)
DRIFT_TOL = 0.05   # rad  — heading drift allowed during straight movement
SETTLE = 0.3    # s    — pause after stopping before next command
FEEDBACK_PERIOD = 0.5    # s    — feedback publish rate
POSE_STALE_LIMIT = 0.06   # s    — skip control tick if pose older than this


class ManhattanServer(Node):
    def __init__(self):
        super().__init__('manhattan_server')

        # Robot state
        self.pose = Pose2D()
        self.goal = Pose2D()
        self.state = 'IDLE'

        # Avoidance
        self.avoidance_active = False   # blocks front-brake check while avoiding
        self.avoid_direction = None
        self.avoid_target_angle = 0.0
        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0

        # Timing
        self.settle_end = 0.0
        self.last_feedback_time = 0.0
        self.last_pose_time = 0.0    # set in pose_cb, used for staleness check

        # Action
        self.active_goal_handle = None

        # LiDAR
        self.last_scan = None
        self.dist_front = float('inf')
        self.dist_left = float('inf')
        self.dist_right = float('inf')
        self.dist_shoulder_left = float('inf')
        self.dist_shoulder_right = float('inf')

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.create_subscription(Pose2D,'/estimated_pose', self.pose_cb,10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb,10)

        # Action server in its own callback group → runs in separate thread
        acbg = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self, MoveRobot, 'move_robot',
            self.execute_callback,
            callback_group=acbg
        )

        self.create_timer(0.02, self.control_loop)   # 50 Hz
        self.get_logger().info('Manhattan Server ready.')

    # Callbacks

    def pose_cb(self, msg: Pose2D):
        self.pose = msg
        self.last_pose_time = time.time()

    def lidar_cb(self, msg: LaserScan):
        self.last_scan = msg
        self.dist_front = self._sector_min(msg,0,30)
        self.dist_left = self._sector_min(msg, 90,40)
        self.dist_right = self._sector_min(msg,-90,40)
        self.dist_shoulder_left = self._sector_min(msg,110,30)
        self.dist_shoulder_right = self._sector_min(msg,-110,30)

    def _sector_min(self, msg: LaserScan, center_deg: float, width_deg: float) -> float:
        half = math.radians(width_deg / 2.0)
        cen = math.radians(center_deg)
        inc = msg.angle_increment
        amin = msg.angle_min
        n = len(msg.ranges)
        i0 = max(0,     int((cen - half - amin) / inc))
        i1 = min(n - 1, int((cen + half - amin) / inc))
        valid = [msg.ranges[i] for i in range(i0, i1 + 1)
                 if math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.1]
        return min(valid) if valid else float('inf')

    # Helpers

    def stop(self):
        self.cmd_pub.publish(Twist())

    def angle_error(self, target: float) -> float:
        return math.atan2(math.sin(target - self.pose.theta),
                          math.cos(target - self.pose.theta))

    def x_done(self) -> bool:
        return abs(self.goal.x - self.pose.x) < GOAL_TOL

    def y_done(self) -> bool:
        return abs(self.goal.y - self.pose.y) < GOAL_TOL

    def resume_state(self) -> str:
        if self.avoidance_axis == 'Y':
            if not self.y_done():
                return 'ALIGN_Y'  # just keep going in Y, don't touch X
            if not self.x_done():
                return 'ALIGN_X'
            return 'SUCCESS'
        else:
            if not self.x_done():
                return 'ALIGN_X'
            if not self.y_done():
                return 'ALIGN_Y'
            return 'SUCCESS'

    def trigger_avoidance(self):
        self.stop()
        time.sleep(0.15)
        """Choose swerve direction and start avoidance turn."""
        # Prefer the side with more room; use goal Y as tiebreaker
        if self.dist_right < SIDE_WALL_DIST:
            direction = 'left'
        elif self.dist_left < SIDE_WALL_DIST:
            direction = 'right'
        elif (self.goal.y - self.pose.y) >= 0:
            direction = 'left'
        else:
            direction = 'right'


        self.avoidance_axis = 'Y' if self.state == 'MOVE_Y' else 'X'
        self.avoid_direction = direction
        self.avoid_start_x = self.pose.x
        self.avoid_start_y = self.pose.y
        self.avoidance_active = True          # suppress further front-brake checks

        shift = (math.pi / 2.0)
        raw = self.pose.theta + shift if direction == 'left' else self.pose.theta - shift
        self.avoid_target_angle = math.atan2(math.sin(raw), math.cos(raw))

        self.state = 'AVOID_TURN'
        self.get_logger().info(f'Avoidance → swerve {direction}')

    def finish_avoidance(self):
        """Clear avoidance flag and resume normal navigation."""
        self.avoidance_active = False
        next_st = self.resume_state()
        self.get_logger().info(f'Obstacle cleared → {next_st}')
        self.stop()
        self.state = next_st
        self.settle_end = time.time() + SETTLE

    def publish_feedback(self):
        if self.active_goal_handle is None:
            return
        now = time.time()
        if now - self.last_feedback_time < FEEDBACK_PERIOD:
            return
        self.last_feedback_time = now
        fb = MoveRobot.Feedback()
        fb.current_x = self.pose.x
        fb.current_y = self.pose.y
        fb.distance_to_goal = math.hypot(self.goal.x - self.pose.x,
                                         self.goal.y - self.pose.y)
        self.active_goal_handle.publish_feedback(fb)

    # -----------------------------------------------------------------------
    # Control loop — 50 Hz
    # -----------------------------------------------------------------------

    def control_loop(self):
        if self.state == 'IDLE':
            return

        # Skip tick if pose is stale (pose tracker hasn't published yet)
        if time.time() - self.last_pose_time > POSE_STALE_LIMIT:
            return

        # Wait out settle pause
        if time.time() < self.settle_end:
            return

        self.publish_feedback()

        twist = Twist()

        # Front obstacle check — only when moving forward AND not already avoiding
        if not self.avoidance_active and self.state in ('MOVE_X', 'MOVE_Y'):
            if self.dist_front < FRONT_BRAKE_DIST:
                self.stop()
                self.trigger_avoidance()
                return

        # ---- ALIGN_X ----
        if self.state == 'ALIGN_X':
            target = 0.0 if (self.goal.x - self.pose.x) > 0 else math.pi
            err    = self.angle_error(target)
            if abs(err) < ALIGN_TOL:
                self.stop()
                self.state = 'MOVE_X'
                self.settle_end = time.time() + SETTLE
            else:
                twist.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ---- MOVE_X ----
        elif self.state == 'MOVE_X':
            if self.x_done():
                self.stop()
                self.state = 'ALIGN_Y'
                self.settle_end = time.time() + SETTLE
            else:
                twist.linear.x = LINEAR_SPEED
                # Heading correction — keep robot from drifting off axis
                drift = self.angle_error(0.0 if (self.goal.x - self.pose.x) > 0 else math.pi)
                if abs(drift) > DRIFT_TOL:
                    twist.angular.z = TURN_SPEED * 0.4 if drift > 0 else -TURN_SPEED * 0.4

        # ---- ALIGN_Y ----
        elif self.state == 'ALIGN_Y':
            target = math.pi / 2 if (self.goal.y - self.pose.y) > 0 else -math.pi / 2
            err    = self.angle_error(target)
            if abs(err) < ALIGN_TOL:
                self.stop()
                self.state = 'MOVE_Y'
                self.settle_end = time.time() + SETTLE
            else:
                twist.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ---- MOVE_Y ----
        elif self.state == 'MOVE_Y':
            if self.y_done():
                self.stop()
                # Re-check X — avoidance may have displaced it
                if not self.x_done():
                    self.get_logger().info('Y done but X displaced — correcting.')
                    self.state = 'ALIGN_X'
                    self.settle_end = time.time() + SETTLE
                else:
                    self.state = 'SUCCESS'
            else:
                twist.linear.x = LINEAR_SPEED
                # Heading correction
                drift = self.angle_error(math.pi / 2 if (self.goal.y - self.pose.y) > 0 else -math.pi / 2)
                if abs(drift) > DRIFT_TOL:
                    twist.angular.z = TURN_SPEED * 0.4 if drift > 0 else -TURN_SPEED * 0.4

        # ---- AVOID_TURN ----
        elif self.state == 'AVOID_TURN':
            err = self.angle_error(self.avoid_target_angle)
            if abs(err) < ALIGN_TOL:
                self.stop()
                self.state = 'AVOID_MOVE'
                self.settle_end = time.time() + SETTLE
            else:
                twist.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ---- AVOID_MOVE ----
        elif self.state == 'AVOID_MOVE':
            if self.dist_front < FRONT_BRAKE_DIST:
                self.stop()
                self.trigger_avoidance()
                return
            # Check shoulder on the obstacle side to detect when we're past it
            shoulder_clear = (
                self.dist_shoulder_right > SIDE_CLEAR_DIST
                if self.avoid_direction == 'left'
                else self.dist_shoulder_left > SIDE_CLEAR_DIST
            )
            moved = math.hypot(self.pose.x - self.avoid_start_x,
                               self.pose.y - self.avoid_start_y)

            if shoulder_clear:
                self.finish_avoidance()
            elif moved > 0.7:

                wall_still_there = (
                    self.dist_right < SIDE_WALL_DIST
                    if self.avoid_direction == 'left'
                    else self.dist_left < SIDE_WALL_DIST
                )
                if wall_still_there:
                    # Reset counter and keep going
                    self.avoid_start_x = self.pose.x
                    self.avoid_start_y = self.pose.y
                else:
                    self.finish_avoidance()
            else:
                twist.linear.x = LINEAR_SPEED

        self.cmd_pub.publish(twist)

    # -----------------------------------------------------------------------
    # Action callback — runs in its own thread
    # -----------------------------------------------------------------------

    def execute_callback(self, goal_handle):
        self.active_goal_handle = goal_handle
        self.goal.x = goal_handle.request.x
        self.goal.y = goal_handle.request.y
        self.last_feedback_time = 0.0
        self.state = 'ALIGN_X'

        self.get_logger().info(f'Goal → x={self.goal.x:.2f}  y={self.goal.y:.2f}')

        while rclpy.ok():
            if self.state == 'SUCCESS':
                break
            time.sleep(0.05)

        self.state              = 'IDLE'
        self.active_goal_handle = None
        self.stop()

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.success = True
        return result



def main():
    rclpy.init()
    node = ManhattanServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()