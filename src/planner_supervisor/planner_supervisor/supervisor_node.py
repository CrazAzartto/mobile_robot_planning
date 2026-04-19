#!/usr/bin/env python3
"""
supervisor_node.py
==================
Planner Supervisor — Orchestrates switching between APF, MPC, and RL planners.

Architecture:
  - APF is the DEFAULT mode (reactive, fast, handles most scenarios)
  - When the robot gets stuck (local minimum), the supervisor switches to
    MPC or RL (configured via 'augmentation_mode' parameter)
  - Once the augmented planner resolves the situation, control returns to APF

Stuck Detection (dual-criteria):
  1. Low velocity: speed < stuck_vel_thresh for stuck_time_thresh seconds
  2. No progress: goal distance hasn't decreased by progress_rate_thresh
     over progress_window seconds (catches oscillating/orbiting behavior)

Subscribes:
  /odom_ground_truth  (nav_msgs/Odometry)
  /scan_filtered      (sensor_msgs/LaserScan)
  /goal_pose          (geometry_msgs/PoseStamped)
  /apf_cmd_vel        (geometry_msgs/Twist) — from APF planner
  /mpc_cmd_vel        (geometry_msgs/Twist) — from MPC controller
  /rl_cmd_vel         (geometry_msgs/Twist) — from RL planner

Publishes:
  /cmd_vel            (geometry_msgs/Twist) — final velocity command
  /planner_mode       (std_msgs/String)     — current active planner
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
import time
from collections import deque
from enum import Enum, auto

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import tf_transformations


class PlannerMode(Enum):
    APF = 'apf'
    MPC = 'mpc'
    RL  = 'rl'


class SupervisorState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    STUCK      = auto()
    AUGMENTING = auto()
    REACHED    = auto()


class SupervisorNode(Node):
    """
    Planner supervisor that monitors robot state and switches between
    APF, MPC, and RL planners as needed.
    """

    def __init__(self):
        super().__init__('planner_supervisor')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        self.declare_parameter('augmentation_mode', 'mpc')  # 'mpc', 'rl', or 'none'
        self.declare_parameter('control_rate',      30.0)   # Hz
        self.declare_parameter('goal_tolerance',    0.25)   # m

        # Stuck detection — criterion 1: low velocity
        self.declare_parameter('stuck_vel_thresh',  0.05)   # m/s
        self.declare_parameter('stuck_time_thresh', 2.0)    # seconds (reduced from 3.0)
        self.declare_parameter('stuck_progress_thresh', 0.1) # m — must make this progress

        # Stuck detection — criterion 2: no goal progress (catches oscillation)
        self.declare_parameter('progress_window',     5.0)  # seconds to check progress
        self.declare_parameter('progress_rate_thresh', 0.2) # m — must reduce goal dist by this

        # Augmentation timeout
        self.declare_parameter('augment_timeout',   15.0)   # seconds — max time in MPC/RL
        self.declare_parameter('augment_progress',  0.5)    # m — progress to return to APF

        # Safety
        self.declare_parameter('safety_dist',       0.15)   # m — emergency stop distance

        aug_mode = self.get_parameter('augmentation_mode').value
        self._aug_mode   = PlannerMode(aug_mode) if aug_mode != 'none' else None
        self._ctrl_rate  = self.get_parameter('control_rate').value
        self._goal_tol   = self.get_parameter('goal_tolerance').value
        self._stuck_v    = self.get_parameter('stuck_vel_thresh').value
        self._stuck_t    = self.get_parameter('stuck_time_thresh').value
        self._stuck_prog = self.get_parameter('stuck_progress_thresh').value
        self._prog_window = self.get_parameter('progress_window').value
        self._prog_rate  = self.get_parameter('progress_rate_thresh').value
        self._aug_timeout = self.get_parameter('augment_timeout').value
        self._aug_progress = self.get_parameter('augment_progress').value
        self._safety_dist = self.get_parameter('safety_dist').value

        # ---------------------------------------------------------------- #
        # State                                                             #
        # ---------------------------------------------------------------- #
        self._state         = SupervisorState.IDLE
        self._current_mode  = PlannerMode.APF
        self._robot_x       = 0.0
        self._robot_y       = 0.0
        self._robot_yaw     = 0.0
        self._robot_vx      = 0.0
        self._robot_vy      = 0.0
        self._goal_x        = 0.0
        self._goal_y        = 0.0
        self._has_goal      = False
        self._scan          = None

        self._stuck_start   = None   # time when low velocity first detected
        self._stuck_pos     = None   # (x, y) when stuck was first detected
        self._aug_start     = None   # time when augmentation started
        self._aug_start_pos = None   # (x, y) when augmentation started

        # Goal progress tracking (sliding window)
        self._goal_dist_history = deque(maxlen=300)  # (time, d_goal) pairs
        self._safety_stop_count = 0

        # Latest commands from each planner
        self._apf_cmd       = Twist()
        self._mpc_cmd       = Twist()
        self._rl_cmd        = Twist()

        # Statistics
        self._mode_switches  = 0
        self._total_aug_time = 0.0
        self._log_counter    = 0

        # ---------------------------------------------------------------- #
        # QoS                                                               #
        # ---------------------------------------------------------------- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---------------------------------------------------------------- #
        # Subscribers                                                       #
        # ---------------------------------------------------------------- #
        self.create_subscription(
            Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.create_subscription(
            LaserScan, '/scan_filtered', self._scan_cb, sensor_qos)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10)

        # Sub-planner velocity commands
        self.create_subscription(
            Twist, '/apf_cmd_vel', self._apf_cmd_cb, 10)
        self.create_subscription(
            Twist, '/mpc_cmd_vel', self._mpc_cmd_cb, 10)
        self.create_subscription(
            Twist, '/rl_cmd_vel', self._rl_cmd_cb, 10)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_cmd  = self.create_publisher(Twist,  '/cmd_vel',      10)
        self.pub_mode = self.create_publisher(String, '/planner_mode', 10)

        # ---------------------------------------------------------------- #
        # Timer                                                             #
        # ---------------------------------------------------------------- #
        self._timer = self.create_timer(1.0 / self._ctrl_rate, self._control_loop)

        aug_str = self._aug_mode.value if self._aug_mode else 'none (pure APF)'
        self.get_logger().info(
            f'Supervisor ready. Augmentation: {aug_str}, '
            f'stuck_thresh={self._stuck_v}m/s for {self._stuck_t}s, '
            f'progress_window={self._prog_window}s')

    # -------------------------------------------------------------------- #
    # Callbacks                                                              #
    # -------------------------------------------------------------------- #
    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist
        self._robot_x  = pos.x
        self._robot_y  = pos.y
        self._robot_vx = vel.linear.x
        self._robot_vy = vel.linear.y
        (_, _, self._robot_yaw) = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w])

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _goal_cb(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._has_goal = True
        self._state = SupervisorState.NAVIGATING
        self._current_mode = PlannerMode.APF
        self._stuck_start = None
        self._aug_start = None
        self._goal_dist_history.clear()
        self._safety_stop_count = 0
        self.get_logger().info(
            f'Goal received: ({self._goal_x:.2f}, {self._goal_y:.2f})')

    def _apf_cmd_cb(self, msg: Twist):
        self._apf_cmd = msg

    def _mpc_cmd_cb(self, msg: Twist):
        self._mpc_cmd = msg

    def _rl_cmd_cb(self, msg: Twist):
        self._rl_cmd = msg

    # -------------------------------------------------------------------- #
    # Main control loop                                                     #
    # -------------------------------------------------------------------- #
    def _control_loop(self):
        # Always publish current mode
        mode_msg = String()
        mode_msg.data = self._current_mode.value
        self.pub_mode.publish(mode_msg)

        if self._state == SupervisorState.IDLE or not self._has_goal:
            return

        if self._state == SupervisorState.REACHED:
            self.pub_cmd.publish(Twist())
            return

        # Check goal reached
        d_goal = math.hypot(
            self._goal_x - self._robot_x,
            self._goal_y - self._robot_y)

        if d_goal < self._goal_tol:
            self._state = SupervisorState.REACHED
            self._current_mode = PlannerMode.APF
            self.pub_cmd.publish(Twist())
            self.get_logger().info(
                f'Goal reached! mode_switches={self._mode_switches}, '
                f'total_aug_time={self._total_aug_time:.1f}s')
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        speed = math.hypot(self._robot_vx, self._robot_vy)

        # Track goal distance over time for progress monitoring
        self._goal_dist_history.append((now, d_goal))

        # Safety stop check — but DON'T return; let stuck detection run
        safety_triggered = False
        if self._scan is not None:
            min_dist = self._get_min_obstacle_distance()
            if min_dist < self._safety_dist:
                safety_triggered = True
                self._safety_stop_count += 1

        # Periodic logging
        self._log_counter += 1
        if self._log_counter % (int(self._ctrl_rate) * 5) == 0:
            self.get_logger().info(
                f'[Supervisor] mode={self._current_mode.value}, '
                f'd_goal={d_goal:.2f}m, speed={speed:.3f}m/s, '
                f'pos=({self._robot_x:.2f},{self._robot_y:.2f}), '
                f'safety_stops={self._safety_stop_count}')

        # ---- State machine ----
        if self._current_mode == PlannerMode.APF:
            if safety_triggered:
                self.pub_cmd.publish(Twist())
            else:
                self.pub_cmd.publish(self._apf_cmd)

            # ---- Stuck detection: Criterion 1 — Low velocity ----
            is_stuck_velocity = False
            if speed < self._stuck_v and d_goal > self._goal_tol * 2:
                if self._stuck_start is None:
                    self._stuck_start = now
                    self._stuck_pos = (self._robot_x, self._robot_y)
                elif (now - self._stuck_start) > self._stuck_t:
                    if self._stuck_pos is not None:
                        dist_moved = math.hypot(
                            self._robot_x - self._stuck_pos[0],
                            self._robot_y - self._stuck_pos[1])
                        if dist_moved < self._stuck_prog:
                            is_stuck_velocity = True
            else:
                self._stuck_start = None
                self._stuck_pos = None

            # ---- Stuck detection: Criterion 2 — No goal progress ----
            is_stuck_progress = False
            if (len(self._goal_dist_history) > 1 and
                    d_goal > self._goal_tol * 2):
                oldest_time, oldest_d = self._goal_dist_history[0]
                window_duration = now - oldest_time
                if window_duration >= self._prog_window:
                    progress_made = oldest_d - d_goal
                    if progress_made < self._prog_rate:
                        is_stuck_progress = True

            # ---- Trigger augmentation if either criterion met ----
            if is_stuck_velocity or is_stuck_progress:
                reason = 'low velocity' if is_stuck_velocity else 'no goal progress'
                self.get_logger().warn(
                    f'Stuck detected ({reason})! speed={speed:.3f}, '
                    f'd_goal={d_goal:.2f}')
                self._switch_to_augmented(now)

        elif self._current_mode == PlannerMode.MPC:
            if safety_triggered:
                # In MPC mode, still forward MPC (it's obstacle-aware)
                self.pub_cmd.publish(self._mpc_cmd)
            else:
                self.pub_cmd.publish(self._mpc_cmd)
            self._check_augmentation_exit(now)

        elif self._current_mode == PlannerMode.RL:
            self.pub_cmd.publish(self._rl_cmd)
            self._check_augmentation_exit(now)

    # -------------------------------------------------------------------- #
    # Mode switching logic                                                   #
    # -------------------------------------------------------------------- #
    def _switch_to_augmented(self, now: float):
        """Switch from APF to MPC or RL."""
        if self._aug_mode is None:
            # No augmentation configured — just log and reset
            self.get_logger().warn('Stuck detected but no augmentation configured')
            self._stuck_start = None
            self._goal_dist_history.clear()
            return

        self._current_mode = self._aug_mode
        self._state = SupervisorState.AUGMENTING
        self._aug_start = now
        self._aug_start_pos = (self._robot_x, self._robot_y)
        self._stuck_start = None
        self._goal_dist_history.clear()
        self._mode_switches += 1

        self.get_logger().warn(
            f'Local minimum detected! Switching to {self._aug_mode.value.upper()} '
            f'(switch #{self._mode_switches}, '
            f'pos=({self._robot_x:.2f},{self._robot_y:.2f}))')

    def _check_augmentation_exit(self, now: float):
        """Check if we should return to APF from augmented mode."""
        if self._aug_start is None:
            return

        elapsed = now - self._aug_start
        self._total_aug_time = getattr(self, '_total_aug_time', 0.0) + (1.0 / self._ctrl_rate)

        d_goal = math.hypot(
            self._goal_x - self._robot_x,
            self._goal_y - self._robot_y)

        # Exit conditions:
        # 1. Made enough progress (escaped the local minimum)
        if self._aug_start_pos is not None:
            dist_from_stuck = math.hypot(
                self._robot_x - self._aug_start_pos[0],
                self._robot_y - self._aug_start_pos[1])
            if dist_from_stuck > self._aug_progress:
                self._return_to_apf('Made sufficient progress')
                return

        # 2. Timeout
        if elapsed > self._aug_timeout:
            self._return_to_apf(f'Timeout ({self._aug_timeout}s)')
            return

        # 3. Goal is very close (APF can handle it)
        if d_goal < 1.0:
            self._return_to_apf('Close to goal')
            return

    def _return_to_apf(self, reason: str):
        """Switch back to APF mode."""
        elapsed = (self.get_clock().now().nanoseconds * 1e-9 - self._aug_start
                   if self._aug_start else 0.0)
        self.get_logger().info(
            f'Returning to APF: {reason} '
            f'(was in {self._current_mode.value.upper()} for {elapsed:.1f}s)')
        self._current_mode = PlannerMode.APF
        self._state = SupervisorState.NAVIGATING
        self._aug_start = None
        self._aug_start_pos = None
        self._stuck_start = None
        self._goal_dist_history.clear()

    # -------------------------------------------------------------------- #
    # Helpers                                                               #
    # -------------------------------------------------------------------- #
    def _get_min_obstacle_distance(self) -> float:
        if self._scan is None:
            return float('inf')
        ranges = np.array(self._scan.ranges)
        valid = np.isfinite(ranges) & (ranges > 0.01)
        return float(np.min(ranges[valid])) if np.any(valid) else float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
