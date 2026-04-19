#!/usr/bin/env python3
"""
eval_node.py
============
Evaluation & Benchmarking Node

Monitors the robot's navigation and logs four performance metrics:
  1. Total path length (integrated from odometry)
  2. Replanning frequency (mode switches per unit distance)
  3. Collision count (obstacle distance < safety threshold)
  4. Time-to-goal (wall clock from goal receipt to arrival)

Subscribes:
  /odom_ground_truth  (nav_msgs/Odometry)    — robot pose & velocity
  /scan_filtered      (sensor_msgs/LaserScan) — obstacle distances
  /goal_pose          (geometry_msgs/PoseStamped) — navigation goal
  /planner_mode       (std_msgs/String)       — current planner mode

Publishes:
  /eval_metrics       (std_msgs/String)       — JSON metrics string (live)

Saves results to CSV files in the specified output directory.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
import json
import time
import os
from datetime import datetime

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf_transformations


class EvalNode(Node):
    """Evaluation metrics logger for path planning benchmarking."""

    def __init__(self):
        super().__init__('eval_node')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        self.declare_parameter('output_dir', 'eval_results')
        self.declare_parameter('collision_threshold', 0.25)   # m
        self.declare_parameter('near_miss_threshold', 0.5)    # m
        self.declare_parameter('goal_tolerance', 0.25)        # m
        self.declare_parameter('publish_rate', 2.0)           # Hz

        self._output_dir  = self.get_parameter('output_dir').value
        self._col_thresh  = self.get_parameter('collision_threshold').value
        self._near_thresh = self.get_parameter('near_miss_threshold').value
        self._goal_tol    = self.get_parameter('goal_tolerance').value
        self._pub_rate    = self.get_parameter('publish_rate').value

        os.makedirs(self._output_dir, exist_ok=True)

        # ---------------------------------------------------------------- #
        # State                                                             #
        # ---------------------------------------------------------------- #
        self._active = False
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._prev_x = None
        self._prev_y = None
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._scan = None
        self._current_mode = 'apf'
        self._goal_reached = False

        # Metrics
        self._total_path_length = 0.0
        self._collision_count = 0
        self._near_miss_count = 0
        self._mode_switch_count = 0
        self._prev_mode = 'apf'
        self._start_time = None
        self._end_time = None
        self._min_obstacle_dist = float('inf')

        # Per-timestep log
        self._trajectory = []  # list of (t, x, y, mode, min_obs_dist)

        # Episode tracking
        self._episode_count = 0
        self._episode_results = []

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
        self.create_subscription(
            String, '/planner_mode', self._mode_cb, 10)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_metrics = self.create_publisher(String, '/eval_metrics', 10)

        # ---------------------------------------------------------------- #
        # Timers                                                            #
        # ---------------------------------------------------------------- #
        self.create_timer(1.0 / self._pub_rate, self._publish_metrics)

        self.get_logger().info(
            f'EvalNode ready. Output: {self._output_dir}/, '
            f'collision_threshold={self._col_thresh}m')

    # -------------------------------------------------------------------- #
    # Callbacks                                                              #
    # -------------------------------------------------------------------- #
    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self._robot_x = pos.x
        self._robot_y = pos.y

        if not self._active:
            return

        # Integrate path length
        if self._prev_x is not None:
            dx = self._robot_x - self._prev_x
            dy = self._robot_y - self._prev_y
            dist = math.hypot(dx, dy)
            if dist < 1.0:  # skip teleports
                self._total_path_length += dist

        self._prev_x = self._robot_x
        self._prev_y = self._robot_y

        # Check for collisions / near misses
        if self._scan is not None:
            min_d = self._get_min_obstacle_distance()
            self._min_obstacle_dist = min(self._min_obstacle_dist, min_d)

            if min_d < self._col_thresh:
                self._collision_count += 1
            elif min_d < self._near_thresh:
                self._near_miss_count += 1

        # Log trajectory point
        now = self.get_clock().now().nanoseconds * 1e-9
        min_d = self._get_min_obstacle_distance() if self._scan else float('inf')
        self._trajectory.append(
            (now, self._robot_x, self._robot_y, self._current_mode, min_d))

        # Check goal reached
        d_goal = math.hypot(
            self._goal_x - self._robot_x,
            self._goal_y - self._robot_y)
        if d_goal < self._goal_tol and not self._goal_reached:
            self._goal_reached = True
            self._end_time = time.time()
            self._save_episode()

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _goal_cb(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._active = True
        self._goal_reached = False

        # Reset metrics for new episode
        self._total_path_length = 0.0
        self._collision_count = 0
        self._near_miss_count = 0
        self._mode_switch_count = 0
        self._min_obstacle_dist = float('inf')
        self._start_time = time.time()
        self._end_time = None
        self._trajectory = []
        self._prev_x = self._robot_x
        self._prev_y = self._robot_y
        self._prev_mode = 'apf'
        self._episode_count += 1

        self.get_logger().info(
            f'[Eval] Episode {self._episode_count} started: '
            f'goal=({self._goal_x:.1f}, {self._goal_y:.1f})')

    def _mode_cb(self, msg: String):
        new_mode = msg.data
        if new_mode != self._prev_mode:
            self._mode_switch_count += 1
            self._prev_mode = new_mode
        self._current_mode = new_mode

    # -------------------------------------------------------------------- #
    # Metrics                                                                #
    # -------------------------------------------------------------------- #
    def _get_metrics(self) -> dict:
        """Return current metrics as a dictionary."""
        elapsed = (time.time() - self._start_time) if self._start_time else 0.0
        if self._end_time and self._start_time:
            time_to_goal = self._end_time - self._start_time
        else:
            time_to_goal = elapsed

        replan_freq = (self._mode_switch_count / max(self._total_path_length, 0.1))

        return {
            'episode': self._episode_count,
            'goal_reached': self._goal_reached,
            'path_length_m': round(self._total_path_length, 3),
            'time_to_goal_s': round(time_to_goal, 2),
            'collision_count': self._collision_count,
            'near_miss_count': self._near_miss_count,
            'mode_switches': self._mode_switch_count,
            'replan_freq_per_m': round(replan_freq, 4),
            'min_obstacle_dist_m': round(self._min_obstacle_dist, 3),
            'current_mode': self._current_mode,
            'straight_line_dist_m': round(math.hypot(
                self._goal_x, self._goal_y), 3),  # from origin
            'path_efficiency': round(
                math.hypot(self._goal_x, self._goal_y) /
                max(self._total_path_length, 0.1), 3),
        }

    def _publish_metrics(self):
        if not self._active:
            return
        metrics = self._get_metrics()
        msg = String()
        msg.data = json.dumps(metrics)
        self.pub_metrics.publish(msg)

    # -------------------------------------------------------------------- #
    # Save results                                                           #
    # -------------------------------------------------------------------- #
    def _save_episode(self):
        """Save episode results to CSV."""
        metrics = self._get_metrics()
        self._episode_results.append(metrics)

        # Save summary CSV
        summary_file = os.path.join(self._output_dir, 'summary.csv')
        header_needed = not os.path.exists(summary_file)

        with open(summary_file, 'a') as f:
            if header_needed:
                f.write(','.join(metrics.keys()) + '\n')
            f.write(','.join(str(v) for v in metrics.values()) + '\n')

        # Save trajectory CSV for this episode
        traj_file = os.path.join(
            self._output_dir,
            f'trajectory_ep{self._episode_count:03d}.csv')
        with open(traj_file, 'w') as f:
            f.write('timestamp,x,y,mode,min_obs_dist\n')
            for row in self._trajectory:
                f.write(','.join(str(v) for v in row) + '\n')

        self.get_logger().info(
            f'[Eval] Episode {self._episode_count} complete — '
            f'path={metrics["path_length_m"]:.2f}m, '
            f'time={metrics["time_to_goal_s"]:.1f}s, '
            f'collisions={metrics["collision_count"]}, '
            f'switches={metrics["mode_switches"]}, '
            f'efficiency={metrics["path_efficiency"]:.2f}')

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
    node = EvalNode()
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
