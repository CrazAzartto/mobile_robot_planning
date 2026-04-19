#!/usr/bin/env python3
"""
rl_node.py
==========
RL-based Local Planner — Inference Node

Loads a pre-trained Stable Baselines3 PPO/SAC policy and produces velocity
commands for the robot. The node constructs the same observation vector as
the training environment (rl_env.py) from live ROS topics.

Subscribes:
  /scan_filtered      (sensor_msgs/LaserScan)       — obstacle distances
  /odom_ground_truth  (nav_msgs/Odometry)           — robot pose & velocity
  /goal_pose          (geometry_msgs/PoseStamped)    — navigation goal
  /planner_mode       (std_msgs/String)              — activation signal

Publishes:
  /rl_cmd_vel         (geometry_msgs/Twist)          — velocity commands
  /rl_obs_debug       (std_msgs/Float32MultiArray)   — observation vector (debug)

If no trained model is found, the node falls back to a hand-crafted reactive
policy (goal-seeking with obstacle avoidance) as a demonstration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
import os

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32MultiArray
import tf_transformations


class RLPlannerNode(Node):
    """
    RL inference node for local planning.
    Loads a pre-trained model or uses a fallback reactive policy.
    """

    def __init__(self):
        super().__init__('rl_planner')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        self.declare_parameter('model_path', '')
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('omega_max', 2.5)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('n_lidar_sectors', 8)
        self.declare_parameter('lidar_range', 5.0)

        self._model_path = self.get_parameter('model_path').value
        self._v_max = self.get_parameter('v_max').value
        self._om_max = self.get_parameter('omega_max').value
        self._ctrl_rate = self.get_parameter('control_rate').value
        self._goal_tol = self.get_parameter('goal_tolerance').value
        self._n_sectors = self.get_parameter('n_lidar_sectors').value
        self._lidar_range = self.get_parameter('lidar_range').value

        # ---------------------------------------------------------------- #
        # Load model                                                        #
        # ---------------------------------------------------------------- #
        self._model = None
        self._using_fallback = True

        if self._model_path and os.path.exists(self._model_path):
            try:
                from stable_baselines3 import PPO
                self._model = PPO.load(self._model_path)
                self._using_fallback = False
                self.get_logger().info(
                    f'RL model loaded from: {self._model_path}')
            except Exception as e:
                self.get_logger().warn(
                    f'Failed to load RL model: {e}. Using fallback policy.')
        else:
            self.get_logger().info(
                'No RL model path provided. Using fallback reactive policy. '
                'Train a model using: python3 train_rl.py')

        # ---------------------------------------------------------------- #
        # State                                                             #
        # ---------------------------------------------------------------- #
        self._active = False
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._robot_v = 0.0
        self._robot_w = 0.0
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._has_goal = False
        self._scan = None

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
            LaserScan, '/scan_filtered', self._scan_cb, sensor_qos)
        self.create_subscription(
            Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.create_subscription(
            String, '/planner_mode', self._mode_cb, 10)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_cmd = self.create_publisher(Twist, '/rl_cmd_vel', 10)
        self.pub_obs = self.create_publisher(
            Float32MultiArray, '/rl_obs_debug', 10)

        # ---------------------------------------------------------------- #
        # Timer                                                             #
        # ---------------------------------------------------------------- #
        self._timer = self.create_timer(1.0 / self._ctrl_rate, self._control_loop)
        self._step_count = 0

        mode_str = 'fallback reactive' if self._using_fallback else 'trained PPO'
        self.get_logger().info(
            f'RL Planner ready (mode: {mode_str}). '
            f'v_max={self._v_max}, ω_max={self._om_max}')

    # -------------------------------------------------------------------- #
    # Callbacks                                                              #
    # -------------------------------------------------------------------- #
    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist
        self._robot_x = pos.x
        self._robot_y = pos.y
        self._robot_v = vel.linear.x
        self._robot_w = vel.angular.z
        (_, _, self._robot_yaw) = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w])

    def _goal_cb(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._has_goal = True

    def _mode_cb(self, msg: String):
        was_active = self._active
        self._active = (msg.data == 'rl')
        if self._active and not was_active:
            self.get_logger().info('RL planner activated by supervisor')
        elif not self._active and was_active:
            self.get_logger().info('RL planner deactivated')

    # -------------------------------------------------------------------- #
    # Build observation                                                      #
    # -------------------------------------------------------------------- #
    def _build_observation(self) -> np.ndarray:
        """
        Construct observation vector matching rl_env.py format:
        [dx, dy, d_goal, theta_goal, v, w, sector_0..sector_N]
        """
        max_d = 18.0  # arena_size * 1.5

        dx = self._goal_x - self._robot_x
        dy = self._goal_y - self._robot_y
        d_goal = math.hypot(dx, dy)
        theta_goal = math.atan2(dy, dx) - self._robot_yaw
        while theta_goal > math.pi: theta_goal -= 2 * math.pi
        while theta_goal < -math.pi: theta_goal += 2 * math.pi

        obs = [
            np.clip(dx / max_d, -1, 1),
            np.clip(dy / max_d, -1, 1),
            np.clip(d_goal / max_d, 0, 1),
            theta_goal / math.pi,
            self._robot_v / self._v_max,
            self._robot_w / self._om_max,
        ]

        # LiDAR sectors
        sector_angle = 2 * math.pi / self._n_sectors
        sectors = [self._lidar_range] * self._n_sectors

        if self._scan is not None:
            scan = self._scan
            ranges = np.array(scan.ranges, dtype=np.float64)
            n = len(ranges)
            angles = np.arange(n) * scan.angle_increment + scan.angle_min

            for i in range(n):
                r = ranges[i]
                if not (np.isfinite(r) and 0.05 <= r <= self._lidar_range):
                    continue
                angle = angles[i]
                # Normalise angle to [-pi, pi]
                while angle > math.pi: angle -= 2 * math.pi
                while angle < -math.pi: angle += 2 * math.pi
                sector_idx = int((angle + math.pi) / sector_angle) % self._n_sectors
                sectors[sector_idx] = min(sectors[sector_idx], r)

        for s in sectors:
            obs.append(s / self._lidar_range)

        return np.array(obs, dtype=np.float32)

    # -------------------------------------------------------------------- #
    # Fallback reactive policy                                               #
    # -------------------------------------------------------------------- #
    def _fallback_policy(self, obs: np.ndarray) -> np.ndarray:
        """
        Simple hand-crafted goal-seeking + obstacle avoidance policy.
        Used when no trained RL model is available.
        """
        theta_goal = obs[3] * math.pi  # denormalise
        d_goal = obs[2] * 18.0

        # LiDAR sectors
        sectors = obs[6:] * self._lidar_range

        # Find closest obstacle sector
        min_sector_dist = np.min(sectors)
        min_sector_idx = np.argmin(sectors)

        # Goal-seeking component
        v_goal = 0.6 * np.clip(d_goal, 0, 1)
        w_goal = 2.0 * theta_goal / math.pi

        # Obstacle avoidance component
        if min_sector_dist < 1.0:
            # Determine avoidance direction
            n_sec = len(sectors)
            # If obstacle is in front sectors (around index n_sec//2 ± 1)
            front_range = range(n_sec // 2 - 1, n_sec // 2 + 2)
            if min_sector_idx in front_range:
                v_goal *= 0.3  # slow down
                # Turn away from closest obstacle
                if min_sector_idx <= n_sec // 2:
                    w_goal -= 1.5  # turn right
                else:
                    w_goal += 1.5  # turn left
            else:
                v_goal *= 0.5

        # Normalise to [-1, 1]
        v_norm = np.clip(v_goal / self._v_max, -1, 1)
        w_norm = np.clip(w_goal / self._om_max, -1, 1)

        return np.array([v_norm, w_norm], dtype=np.float32)

    # -------------------------------------------------------------------- #
    # Control loop                                                           #
    # -------------------------------------------------------------------- #
    def _control_loop(self):
        if not self._active or not self._has_goal:
            return

        # Check goal reached
        d_goal = math.hypot(self._goal_x - self._robot_x,
                            self._goal_y - self._robot_y)
        if d_goal < self._goal_tol:
            self.pub_cmd.publish(Twist())
            return

        if self._scan is None:
            return

        # Build observation
        obs = self._build_observation()

        # Publish observation for debugging
        obs_msg = Float32MultiArray()
        obs_msg.data = obs.tolist()
        self.pub_obs.publish(obs_msg)

        # Get action
        if self._model is not None:
            action, _ = self._model.predict(obs, deterministic=True)
        else:
            action = self._fallback_policy(obs)

        # Scale and publish
        cmd = Twist()
        cmd.linear.x = float(np.clip(action[0] * self._v_max,
                                      -self._v_max, self._v_max))
        cmd.angular.z = float(np.clip(action[1] * self._om_max,
                                       -self._om_max, self._om_max))
        self.pub_cmd.publish(cmd)

        self._step_count += 1
        if self._step_count % 50 == 0:
            mode = 'PPO' if self._model else 'fallback'
            self.get_logger().info(
                f'[RL-{mode}] step={self._step_count}, '
                f'd_goal={d_goal:.2f}m, v={cmd.linear.x:.2f}, '
                f'ω={cmd.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = RLPlannerNode()
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
