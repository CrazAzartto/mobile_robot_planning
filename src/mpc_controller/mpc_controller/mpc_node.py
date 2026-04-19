#!/usr/bin/env python3
"""
mpc_node.py
===========
Model Predictive Control (MPC) Local Planner

Implements a receding-horizon MPC controller for the differential drive robot.
Uses a linearised unicycle kinematic model with obstacle avoidance constraints
solved via scipy.optimize (sequential quadratic programming).

The MPC is designed to be activated by the planner supervisor when the APF
planner encounters a local minimum. It computes a short-horizon optimal
trajectory that escapes the local minimum while respecting kinematic constraints
and obstacle clearance requirements.

Subscribes:
  /scan_filtered      (sensor_msgs/LaserScan)       — obstacle distances
  /odom_ground_truth  (nav_msgs/Odometry)           — robot pose & velocity
  /goal_pose          (geometry_msgs/PoseStamped)    — navigation goal
  /fused_obstacles    (obstacle_msgs/ObstacleArray)  — fused obstacle positions
  /planner_mode       (std_msgs/String)              — activation signal

Publishes:
  /mpc_cmd_vel        (geometry_msgs/Twist)          — velocity commands
  /mpc_path_markers   (visualization_msgs/MarkerArray) — predicted path in RViz

==========================================================================
KINEMATIC MODEL (Unicycle)
==========================================================================
  State:   q = [x, y, θ]
  Control: u = [v, ω]

  Discrete-time (Euler integration, dt = 0.1 s):
    x(k+1) = x(k) + v(k) · cos(θ(k)) · dt
    y(k+1) = y(k) + v(k) · sin(θ(k)) · dt
    θ(k+1) = θ(k) + ω(k) · dt

==========================================================================
COST FUNCTION
==========================================================================
  J = Σ_{k=0}^{N-1} [ w_goal · ||q(k) - q_goal||²
                     + w_ctrl · ||u(k)||²
                     + w_smooth · ||u(k) - u(k-1)||² ]
    + w_terminal · ||q(N) - q_goal||²

==========================================================================
CONSTRAINTS
==========================================================================
  |v(k)| ≤ v_max       (linear velocity bound)
  |ω(k)| ≤ ω_max       (angular velocity bound)
  ||q(k) - q_obs||² ≥ d_safe²  (obstacle clearance, soft penalty)
==========================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
from scipy.optimize import minimize

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from obstacle_msgs.msg import ObstacleArray
import tf_transformations


class MPCNode(Node):
    """
    MPC-based local planner for escaping local minima and producing
    dynamically feasible trajectories.
    """

    def __init__(self):
        super().__init__('mpc_controller')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        # MPC horizon
        self.declare_parameter('horizon_steps',     10)
        self.declare_parameter('dt',                0.1)    # seconds per step

        # Cost weights
        self.declare_parameter('w_goal',            5.0)    # goal tracking
        self.declare_parameter('w_ctrl',            0.1)    # control effort
        self.declare_parameter('w_smooth',          1.0)    # control smoothness
        self.declare_parameter('w_terminal',        20.0)   # terminal cost
        self.declare_parameter('w_obstacle',        50.0)   # obstacle penalty

        # Constraints
        self.declare_parameter('v_max',             1.5)    # m/s
        self.declare_parameter('omega_max',         2.5)    # rad/s
        self.declare_parameter('d_safe',            0.4)    # metres obstacle clearance

        # Obstacle extraction from LiDAR
        self.declare_parameter('obstacle_range',    3.0)    # consider obstacles within this range
        self.declare_parameter('max_obstacles',     15)     # max obstacles for optimisation

        # Control
        self.declare_parameter('control_rate',      10.0)   # Hz
        self.declare_parameter('goal_tolerance',    0.25)   # m

        # Read parameters
        self._N         = self.get_parameter('horizon_steps').value
        self._dt        = self.get_parameter('dt').value
        self._w_goal    = self.get_parameter('w_goal').value
        self._w_ctrl    = self.get_parameter('w_ctrl').value
        self._w_smooth  = self.get_parameter('w_smooth').value
        self._w_term    = self.get_parameter('w_terminal').value
        self._w_obs     = self.get_parameter('w_obstacle').value
        self._v_max     = self.get_parameter('v_max').value
        self._om_max    = self.get_parameter('omega_max').value
        self._d_safe    = self.get_parameter('d_safe').value
        self._obs_range = self.get_parameter('obstacle_range').value
        self._max_obs   = self.get_parameter('max_obstacles').value
        self._ctrl_rate = self.get_parameter('control_rate').value
        self._goal_tol  = self.get_parameter('goal_tolerance').value

        # ---------------------------------------------------------------- #
        # State                                                             #
        # ---------------------------------------------------------------- #
        self._active    = False
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._goal_x    = 0.0
        self._goal_y    = 0.0
        self._has_goal  = False

        self._scan      = None
        self._obstacles_world = []   # list of (x, y) in odom frame

        # Warm-start: previous solution
        self._prev_u    = np.zeros(2 * self._N)

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
            ObstacleArray, '/fused_obstacles', self._fused_cb, 10)
        self.create_subscription(
            String, '/planner_mode', self._mode_cb, 10)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_cmd     = self.create_publisher(Twist,       '/mpc_cmd_vel',     10)
        self.pub_markers = self.create_publisher(MarkerArray, '/mpc_path_markers', 10)

        # ---------------------------------------------------------------- #
        # Control timer                                                     #
        # ---------------------------------------------------------------- #
        self._timer = self.create_timer(1.0 / self._ctrl_rate, self._control_loop)
        self._solve_count = 0

        self.get_logger().info(
            f'MPC Controller ready. N={self._N}, dt={self._dt}s, '
            f'v_max={self._v_max}, ω_max={self._om_max}')

    # -------------------------------------------------------------------- #
    # Callbacks                                                              #
    # -------------------------------------------------------------------- #
    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._robot_x = pos.x
        self._robot_y = pos.y
        (_, _, self._robot_yaw) = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w])

    def _goal_cb(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._has_goal = True

    def _fused_cb(self, msg: ObstacleArray):
        """Extract world-frame obstacle positions from fused detections."""
        obs_list = []
        for obs in msg.obstacles:
            if obs.x != 0.0 or obs.y != 0.0:
                # Transform from robot frame to world frame
                c = math.cos(self._robot_yaw)
                s = math.sin(self._robot_yaw)
                wx = self._robot_x + c * obs.x - s * obs.y
                wy = self._robot_y + s * obs.x + c * obs.y
                obs_list.append((wx, wy))
        self._obstacles_world = obs_list

    def _mode_cb(self, msg: String):
        was_active = self._active
        self._active = (msg.data == 'mpc')
        if self._active and not was_active:
            self.get_logger().info('MPC activated by supervisor')
            self._prev_u = np.zeros(2 * self._N)  # reset warm start
        elif not self._active and was_active:
            self.get_logger().info('MPC deactivated')

    # -------------------------------------------------------------------- #
    # Extract obstacles from LiDAR scan (world frame)                       #
    # -------------------------------------------------------------------- #
    def _get_lidar_obstacles(self) -> list:
        """Extract nearby obstacle positions from LiDAR scan in world frame."""
        if self._scan is None:
            return []

        scan = self._scan
        ranges = np.array(scan.ranges, dtype=np.float64)
        n = len(ranges)
        angles = np.arange(n) * scan.angle_increment + scan.angle_min

        # Filter valid, close obstacles
        valid = (np.isfinite(ranges) &
                 (ranges >= 0.05) &
                 (ranges <= self._obs_range))
        if not np.any(valid):
            return []

        r_v = ranges[valid]
        th_v = angles[valid]

        # Robot frame -> world frame
        ox_robot = r_v * np.cos(th_v)
        oy_robot = r_v * np.sin(th_v)

        c = math.cos(self._robot_yaw)
        s = math.sin(self._robot_yaw)
        ox_world = self._robot_x + c * ox_robot - s * oy_robot
        oy_world = self._robot_y + s * ox_robot + c * oy_robot

        # Subsample to max_obstacles (take closest)
        dists = r_v
        if len(dists) > self._max_obs:
            idx = np.argsort(dists)[:self._max_obs]
            ox_world = ox_world[idx]
            oy_world = oy_world[idx]

        return list(zip(ox_world.tolist(), oy_world.tolist()))

    # -------------------------------------------------------------------- #
    # MPC optimisation                                                       #
    # -------------------------------------------------------------------- #
    def _solve_mpc(self, obstacles: list) -> np.ndarray:
        """
        Solve the MPC problem using scipy.optimize.minimize (SLSQP).

        Decision variables: u = [v0, ω0, v1, ω1, ..., v_{N-1}, ω_{N-1}]
        Total: 2*N variables.

        Returns optimal control sequence as (N, 2) array.
        """
        N = self._N
        dt = self._dt
        x0 = self._robot_x
        y0 = self._robot_y
        th0 = self._robot_yaw
        gx = self._goal_x
        gy = self._goal_y

        obs_arr = np.array(obstacles) if len(obstacles) > 0 else np.zeros((0, 2))

        def simulate(u_flat):
            """Forward simulate the kinematic model."""
            traj = np.zeros((N + 1, 3))
            traj[0] = [x0, y0, th0]
            for k in range(N):
                v = u_flat[2 * k]
                w = u_flat[2 * k + 1]
                x_k, y_k, th_k = traj[k]
                traj[k + 1, 0] = x_k + v * math.cos(th_k) * dt
                traj[k + 1, 1] = y_k + v * math.sin(th_k) * dt
                traj[k + 1, 2] = th_k + w * dt
            return traj

        def cost(u_flat):
            """Compute the total MPC cost."""
            traj = simulate(u_flat)
            J = 0.0

            # Stage cost
            for k in range(1, N + 1):
                dx = traj[k, 0] - gx
                dy = traj[k, 1] - gy
                # Goal tracking
                J += self._w_goal * (dx**2 + dy**2)

                # Obstacle penalty (soft constraint — barrier function)
                if len(obs_arr) > 0:
                    diffs = obs_arr - traj[k, :2]
                    dists_sq = np.sum(diffs**2, axis=1)
                    for d_sq in dists_sq:
                        d = math.sqrt(d_sq) + 1e-6
                        if d < self._d_safe * 2.0:
                            # Exponential barrier penalty
                            J += self._w_obs * math.exp(-3.0 * (d - self._d_safe))

            # Control effort
            for k in range(N):
                v = u_flat[2 * k]
                w = u_flat[2 * k + 1]
                J += self._w_ctrl * (v**2 + w**2)

                # Smoothness (penalise control changes)
                if k > 0:
                    dv = v - u_flat[2 * (k - 1)]
                    dw = w - u_flat[2 * (k - 1) + 1]
                    J += self._w_smooth * (dv**2 + dw**2)

            # Terminal cost (extra weight on final state)
            dx_t = traj[N, 0] - gx
            dy_t = traj[N, 1] - gy
            J += self._w_term * (dx_t**2 + dy_t**2)

            return J

        # Bounds: v in [-v_max, v_max], ω in [-ω_max, ω_max]
        bounds = []
        for k in range(N):
            bounds.append((-self._v_max, self._v_max))   # v
            bounds.append((-self._om_max, self._om_max))  # ω

        # Initial guess: warm start from previous solution (shifted by 1)
        u0 = np.zeros(2 * N)
        if len(self._prev_u) == 2 * N:
            # Shift: drop first control, repeat last
            u0[:-2] = self._prev_u[2:]
            u0[-2:] = self._prev_u[-2:]
        else:
            # Simple heuristic: point toward goal
            dx = gx - x0
            dy = gy - y0
            desired_yaw = math.atan2(dy, dx)
            yaw_err = self._normalise_angle(desired_yaw - th0)
            for k in range(N):
                u0[2 * k] = 0.3       # moderate forward speed
                u0[2 * k + 1] = np.clip(yaw_err, -1.0, 1.0)

        # Solve
        result = minimize(
            cost, u0,
            method='SLSQP',
            bounds=bounds,
            options={
                'maxiter': 50,
                'ftol': 1e-4,
                'disp': False
            }
        )

        u_opt = result.x
        self._prev_u = u_opt.copy()

        return u_opt.reshape(N, 2)

    # -------------------------------------------------------------------- #
    # Main control loop                                                     #
    # -------------------------------------------------------------------- #
    def _control_loop(self):
        """Execute one MPC cycle."""
        if not self._active or not self._has_goal:
            return

        # Check goal reached
        d_goal = math.hypot(self._goal_x - self._robot_x,
                            self._goal_y - self._robot_y)
        if d_goal < self._goal_tol:
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return

        if self._scan is None:
            return

        # Gather obstacles
        obstacles = self._get_lidar_obstacles()

        # Solve MPC
        u_opt = self._solve_mpc(obstacles)
        self._solve_count += 1

        # Apply first control
        cmd = Twist()
        cmd.linear.x = float(np.clip(u_opt[0, 0], -self._v_max, self._v_max))
        cmd.angular.z = float(np.clip(u_opt[0, 1], -self._om_max, self._om_max))
        self.pub_cmd.publish(cmd)

        # Publish predicted trajectory for RViz
        self._publish_predicted_path(u_opt)

        if self._solve_count % 5 == 0 or self._solve_count <= 3:
            self.get_logger().info(
                f'[MPC] solve #{self._solve_count}, d_goal={d_goal:.2f}m, '
                f'v={cmd.linear.x:.2f}, ω={cmd.angular.z:.2f}, '
                f'pos=({self._robot_x:.2f},{self._robot_y:.2f}), '
                f'obstacles={len(obstacles)}')

    # -------------------------------------------------------------------- #
    # Visualisation                                                          #
    # -------------------------------------------------------------------- #
    def _publish_predicted_path(self, u_opt: np.ndarray):
        """Publish the MPC predicted trajectory as RViz markers."""
        array = MarkerArray()

        # Simulate forward
        N = self._N
        dt = self._dt
        traj = np.zeros((N + 1, 3))
        traj[0] = [self._robot_x, self._robot_y, self._robot_yaw]
        for k in range(N):
            v, w = u_opt[k]
            x_k, y_k, th_k = traj[k]
            traj[k + 1, 0] = x_k + v * math.cos(th_k) * dt
            traj[k + 1, 1] = y_k + v * math.sin(th_k) * dt
            traj[k + 1, 2] = th_k + w * dt

        # Line strip for predicted path
        path_m = Marker()
        path_m.header.frame_id = 'odom'
        path_m.header.stamp = self.get_clock().now().to_msg()
        path_m.ns = 'mpc_predicted'
        path_m.id = 0
        path_m.type = Marker.LINE_STRIP
        path_m.action = Marker.ADD
        path_m.scale.x = 0.05
        path_m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)
        path_m.lifetime.nanosec = int(1e9 / self._ctrl_rate * 2)

        for k in range(N + 1):
            p = Point()
            p.x = float(traj[k, 0])
            p.y = float(traj[k, 1])
            p.z = 0.15
            path_m.points.append(p)

        array.markers.append(path_m)

        # Sphere markers at each predicted waypoint
        for k in range(1, N + 1):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'mpc_waypoints'
            m.id = k
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(traj[k, 0])
            m.pose.position.y = float(traj[k, 1])
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            # Colour gradient: green (near) → blue (far)
            frac = k / N
            m.color = ColorRGBA(
                r=0.0, g=float(1.0 - frac), b=float(frac), a=0.8)
            m.lifetime.nanosec = int(1e9 / self._ctrl_rate * 2)
            array.markers.append(m)

        self.pub_markers.publish(array)

    # -------------------------------------------------------------------- #
    # Helpers                                                               #
    # -------------------------------------------------------------------- #
    @staticmethod
    def _normalise_angle(angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
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
