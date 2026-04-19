#!/usr/bin/env python3
"""
rl_env.py
=========
Custom Gymnasium environment for training RL policies in the mobile robot
navigation task. This environment wraps the robot's observation and action
spaces for use with Stable Baselines3.

Can be used in two modes:
  1. Standalone (offline training): Simulates simplified kinematics internally
     for fast training without Gazebo.
  2. ROS-connected: Bridges to the live Gazebo simulation for fine-tuning.

Observation Space (continuous, 22-dim):
  [0-1]   : relative goal (dx, dy) normalised
  [2]     : distance to goal (normalised)
  [3]     : angle to goal (normalised to [-1, 1])
  [4-5]   : robot velocity (v, ω) normalised
  [6-21]  : 8 LiDAR sectors (min range in each 45° sector), normalised

Action Space (continuous, 2-dim):
  [0]     : linear velocity v ∈ [-1, 1] → scaled to [-v_max, v_max]
  [1]     : angular velocity ω ∈ [-1, 1] → scaled to [-ω_max, ω_max]

Reward:
  + progress toward goal (change in distance)
  + large bonus for reaching goal
  - collision penalty
  - time penalty (small, per step)
  - proximity penalty (getting close to obstacles)
"""

import numpy as np
import math

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces


class RobotNavEnv(gym.Env):
    """
    Simplified 2D navigation environment for RL training.
    Uses internal unicycle kinematics for fast episode rollouts.
    """

    metadata = {'render_modes': ['human']}

    def __init__(self,
                 v_max: float = 1.5,
                 omega_max: float = 2.5,
                 dt: float = 0.1,
                 max_steps: int = 500,
                 arena_size: float = 12.0,
                 n_obstacles: int = 8,
                 obstacle_radius: float = 0.4,
                 goal_tolerance: float = 0.25,
                 collision_radius: float = 0.3,
                 n_lidar_sectors: int = 8,
                 lidar_range: float = 5.0,
                 render_mode=None):
        super().__init__()

        self.v_max = v_max
        self.omega_max = omega_max
        self.dt = dt
        self.max_steps = max_steps
        self.arena_size = arena_size
        self.n_obstacles = n_obstacles
        self.obstacle_radius = obstacle_radius
        self.goal_tolerance = goal_tolerance
        self.collision_radius = collision_radius
        self.n_sectors = n_lidar_sectors
        self.lidar_range = lidar_range
        self.render_mode = render_mode

        # Observation: [dx, dy, d_goal, theta_goal, v, w, sector_0..sector_N]
        obs_dim = 6 + n_lidar_sectors
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32)

        # Action: [v_normalised, omega_normalised]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_v = 0.0
        self.robot_w = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.obstacles = []  # list of (x, y, radius)
        self.step_count = 0
        self.prev_dist_to_goal = 0.0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Random start position near origin
        self.robot_x = self.np_random.uniform(-1.0, 1.0)
        self.robot_y = self.np_random.uniform(-1.0, 1.0)
        self.robot_yaw = self.np_random.uniform(-math.pi, math.pi)
        self.robot_v = 0.0
        self.robot_w = 0.0

        # Random goal (far from start)
        while True:
            self.goal_x = self.np_random.uniform(3.0, self.arena_size - 1.0)
            self.goal_y = self.np_random.uniform(
                -self.arena_size / 3, self.arena_size / 3)
            d = math.hypot(self.goal_x - self.robot_x,
                           self.goal_y - self.robot_y)
            if d > 3.0:
                break

        # Random obstacles (avoid start and goal positions)
        self.obstacles = []
        for _ in range(self.n_obstacles):
            for _attempt in range(50):
                ox = self.np_random.uniform(0.0, self.arena_size - 1.0)
                oy = self.np_random.uniform(
                    -self.arena_size / 3, self.arena_size / 3)
                r = self.np_random.uniform(0.2, 0.6)
                # Check not too close to start or goal
                d_start = math.hypot(ox - self.robot_x, oy - self.robot_y)
                d_goal = math.hypot(ox - self.goal_x, oy - self.goal_y)
                if d_start > 1.5 and d_goal > 1.0:
                    self.obstacles.append((ox, oy, r))
                    break

        self.step_count = 0
        self.prev_dist_to_goal = math.hypot(
            self.goal_x - self.robot_x, self.goal_y - self.robot_y)

        obs = self._get_observation()
        return obs, {}

    def step(self, action):
        self.step_count += 1

        # Scale action
        v = float(action[0]) * self.v_max
        w = float(action[1]) * self.omega_max

        # Unicycle kinematics
        self.robot_x += v * math.cos(self.robot_yaw) * self.dt
        self.robot_y += v * math.sin(self.robot_yaw) * self.dt
        self.robot_yaw += w * self.dt
        # Normalise yaw
        while self.robot_yaw > math.pi: self.robot_yaw -= 2 * math.pi
        while self.robot_yaw < -math.pi: self.robot_yaw += 2 * math.pi
        self.robot_v = v
        self.robot_w = w

        # --- Compute reward ---
        dist_to_goal = math.hypot(
            self.goal_x - self.robot_x, self.goal_y - self.robot_y)

        reward = 0.0

        # Progress reward
        progress = self.prev_dist_to_goal - dist_to_goal
        reward += 10.0 * progress

        # Time penalty
        reward -= 0.01

        # Goal bonus
        reached = dist_to_goal < self.goal_tolerance
        if reached:
            reward += 100.0

        # Collision check
        collision = False
        min_obs_dist = float('inf')
        for (ox, oy, r) in self.obstacles:
            d = math.hypot(self.robot_x - ox, self.robot_y - oy) - r
            min_obs_dist = min(min_obs_dist, d)
            if d < self.collision_radius:
                collision = True
                reward -= 50.0
                break

        # Proximity penalty (proportional to closeness)
        if min_obs_dist < 1.0 and not collision:
            reward -= 2.0 * (1.0 - min_obs_dist)

        # Out of bounds
        out_of_bounds = (abs(self.robot_x) > self.arena_size or
                         abs(self.robot_y) > self.arena_size)
        if out_of_bounds:
            reward -= 30.0

        # Terminal conditions
        terminated = reached or collision or out_of_bounds
        truncated = self.step_count >= self.max_steps

        self.prev_dist_to_goal = dist_to_goal
        obs = self._get_observation()

        return obs, reward, terminated, truncated, {
            'reached': reached,
            'collision': collision,
            'dist_to_goal': dist_to_goal,
        }

    def _get_observation(self) -> np.ndarray:
        """Build the observation vector."""
        # Relative goal
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        d_goal = math.hypot(dx, dy)
        theta_goal = math.atan2(dy, dx) - self.robot_yaw
        while theta_goal > math.pi: theta_goal -= 2 * math.pi
        while theta_goal < -math.pi: theta_goal += 2 * math.pi

        max_d = self.arena_size * 1.5
        obs = [
            np.clip(dx / max_d, -1, 1),
            np.clip(dy / max_d, -1, 1),
            np.clip(d_goal / max_d, 0, 1),
            theta_goal / math.pi,
            self.robot_v / self.v_max,
            self.robot_w / self.omega_max,
        ]

        # Simulated LiDAR sectors
        sector_angle = 2 * math.pi / self.n_sectors
        sectors = [self.lidar_range] * self.n_sectors

        for (ox, oy, r) in self.obstacles:
            dx_o = ox - self.robot_x
            dy_o = oy - self.robot_y
            d_o = math.hypot(dx_o, dy_o) - r
            if d_o > self.lidar_range:
                continue
            angle = math.atan2(dy_o, dx_o) - self.robot_yaw
            while angle > math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi
            sector_idx = int((angle + math.pi) / sector_angle) % self.n_sectors
            sectors[sector_idx] = min(sectors[sector_idx], max(d_o, 0.0))

        # Normalise sectors to [0, 1]
        for i in range(self.n_sectors):
            obs.append(sectors[i] / self.lidar_range)

        return np.array(obs, dtype=np.float32)

    def render(self):
        pass  # Implement matplotlib rendering if needed

    def close(self):
        pass
