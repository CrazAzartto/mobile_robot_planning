#!/usr/bin/env python3
"""
apf_node.py
===========
Task E — Artificial Potential Field (APF) Path Planner

Subscribes to:
  /scan_filtered    (sensor_msgs/LaserScan)    — obstacle distances
  /odom             (nav_msgs/Odometry)        — robot pose & velocity
  /goal_pose        (geometry_msgs/PoseStamped) — navigation goal

Publishes:
  /cmd_vel          (geometry_msgs/Twist)       — velocity commands
  /apf_force        (geometry_msgs/Vector3)     — total force (debug)
  /apf_path_markers (visualization_msgs/MarkerArray) — trajectory in RViz

==========================================================================
POTENTIAL FUNCTIONS
==========================================================================

Attractive Potential (Conic-well model):
  d_goal = distance to goal

  U_att(q) = { 0.5 · k_att · d_goal²           if d_goal ≤ d_star
             { d_star · k_att · d_goal           if d_goal > d_star
             {   − 0.5 · k_att · d_star²

  Gradient (force):
  F_att =  { k_att · (q_goal − q)               if d_goal ≤ d_star   [quadratic]
           { d_star · k_att · (q_goal − q)/d_goal if d_goal > d_star  [conic]

  The conic model avoids unbounded forces far from the goal.

Repulsive Potential (Inverse-square law):
  d_obs  = minimum obstacle distance (from LiDAR)
  d0     = influence radius = 1.5 m

  U_rep(q) = { 0.5 · k_rep · (1/d_obs − 1/d0)²  if d_obs ≤ d0
             { 0                                   if d_obs > d0

  Gradient (force):
  F_rep = { k_rep · (1/d_obs − 1/d0) · (1/d_obs²) · ∇d_obs   if d_obs ≤ d0
          { 0

  The direction of F_rep is away from the nearest obstacle point.

==========================================================================
LOCAL MINIMA ESCAPE
==========================================================================
  Detected when |v_robot| < v_stuck_thresh AND d_goal > goal_tolerance
  for stuck_time_thresh consecutive seconds.

  Escape strategy: random-walk perturbation — rotate in place by a random
  angle ±(90°–180°), then move forward briefly, then resume APF.
==========================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
import time
from enum import Enum, auto

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf_transformations


class PlannerState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    STUCK      = auto()
    ESCAPING   = auto()
    REACHED    = auto()


class APFPlannerNode(Node):
    """
    APF-based path planner for the differential drive robot.
    Implements conic-well attractive + inverse-square repulsive fields
    with automatic local-minima detection and escape behaviour.
    """

    def __init__(self):
        super().__init__('apf_planner')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        # Attractive field
        self.declare_parameter('k_att',           1.2)   # attractive gain
        self.declare_parameter('d_star',          1.5)   # conic/quad switch (m)

        # Repulsive field
        self.declare_parameter('k_rep',           1.5)   # repulsive gain
        self.declare_parameter('d_influence',     1.0)   # influence radius (m)
        self.declare_parameter('d_safe',          0.25)  # safety stop distance (m)

        # Robot velocity limits
        self.declare_parameter('max_linear_vel',  1.5)   # m/s
        self.declare_parameter('max_angular_vel', 2.5)   # rad/s
        self.declare_parameter('force_to_vel_scale', 1.0)  # scaling factor

        # Goal tolerance
        self.declare_parameter('goal_tolerance',  0.20)  # m

        # Local minima detection
        self.declare_parameter('stuck_vel_thresh',   0.03)  # m/s
        self.declare_parameter('stuck_time_thresh',  3.0)   # s
        self.declare_parameter('escape_rotate_time', 1.5)   # s
        self.declare_parameter('escape_move_time',   3.0)   # s
        self.declare_parameter('escape_angular_vel', 1.5)   # rad/s
        self.declare_parameter('escape_linear_vel',  0.5)   # m/s

        # Control loop
        self.declare_parameter('control_rate',    30.0)  # Hz

        self._k_att        = self.get_parameter('k_att').value
        self._d_star       = self.get_parameter('d_star').value
        self._k_rep        = self.get_parameter('k_rep').value
        self._d_inf        = self.get_parameter('d_influence').value
        self._d_safe       = self.get_parameter('d_safe').value
        self._max_lin      = self.get_parameter('max_linear_vel').value
        self._max_ang      = self.get_parameter('max_angular_vel').value
        self._f2v          = self.get_parameter('force_to_vel_scale').value
        self._goal_tol     = self.get_parameter('goal_tolerance').value
        self._stuck_v      = self.get_parameter('stuck_vel_thresh').value
        self._stuck_t      = self.get_parameter('stuck_time_thresh').value
        self._esc_rot_t    = self.get_parameter('escape_rotate_time').value
        self._esc_mov_t    = self.get_parameter('escape_move_time').value
        self._esc_ang_v    = self.get_parameter('escape_angular_vel').value
        self._esc_lin_v    = self.get_parameter('escape_linear_vel').value
        self._ctrl_rate    = self.get_parameter('control_rate').value

        # ---------------------------------------------------------------- #
        # State                                                             #
        # ---------------------------------------------------------------- #
        self._state     = PlannerState.IDLE
        self._goal_x    = 0.0
        self._goal_y    = 0.0
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._robot_vx  = 0.0
        self._robot_vy  = 0.0

        self._scan         = None   # latest filtered scan
        self._stuck_start  = None   # time robot was first detected as stuck
        self._escape_start = None   # time escape manoeuvre started
        self._escape_dir   = 1.0    # +1 or -1 for rotation direction
        self._path_history = []     # list of (x, y) for visualisation

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
            LaserScan, '/scan_filtered', self._scan_callback, sensor_qos)
        self.create_subscription(
            Odometry,  '/odom',          self._odom_callback,  10)
        self.create_subscription(
            PoseStamped, '/goal_pose',   self._goal_callback,  10)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_cmd     = self.create_publisher(Twist,       '/cmd_vel',          10)
        self.pub_force   = self.create_publisher(Vector3,     '/apf_force',        10)
        self.pub_markers = self.create_publisher(MarkerArray, '/apf_path_markers', 10)

        # ---------------------------------------------------------------- #
        # Control loop timer                                                #
        # ---------------------------------------------------------------- #
        self._ctrl_timer = self.create_timer(
            1.0 / self._ctrl_rate, self._control_loop)

        self.get_logger().info(
            f'APFPlanner ready.\n'
            f'  k_att={self._k_att}, d_star={self._d_star} m\n'
            f'  k_rep={self._k_rep}, d_influence={self._d_inf} m\n'
            f'  max_vel=[{self._max_lin} m/s, {self._max_ang} rad/s]')

    # -------------------------------------------------------------------- #
    # Subscriber callbacks                                                  #
    # -------------------------------------------------------------------- #
    def _scan_callback(self, msg: LaserScan):
        self._scan = msg

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist

        self._robot_x  = pos.x
        self._robot_y  = pos.y
        self._robot_vx = vel.linear.x
        self._robot_vy = vel.linear.y

        # Extract yaw from quaternion
        (_, _, self._robot_yaw) = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w])

    def _goal_callback(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._state  = PlannerState.NAVIGATING
        self._path_history.clear()
        self.get_logger().info(
            f'New goal received: ({self._goal_x:.2f}, {self._goal_y:.2f})')

    # -------------------------------------------------------------------- #
    # Main control loop                                                     #
    # -------------------------------------------------------------------- #
    def _control_loop(self):
        """Execute one planning cycle at the control rate."""

        if self._state == PlannerState.IDLE:
            return

        if self._state == PlannerState.REACHED:
            self._publish_stop()
            return

        # ---- Check if goal is reached -----------------------------------
        d_goal = math.hypot(
            self._goal_x - self._robot_x,
            self._goal_y - self._robot_y)

        if d_goal < self._goal_tol:
            self._state = PlannerState.REACHED
            self._publish_stop()
            self.get_logger().info(
                f'Goal reached! Final distance: {d_goal:.3f} m')
            return

        # ---- Escape manoeuvre state machine -----------------------------
        if self._state == PlannerState.ESCAPING:
            self._execute_escape()
            return

        # ---- Compute APF forces -----------------------------------------
        if self._scan is None:
            return

        F_att = self._attractive_force()
        F_rep = self._repulsive_force()
        F_tot = F_att + F_rep

        # ---- Publish force debug vector ---------------------------------
        fv          = Vector3()
        fv.x, fv.y  = float(F_tot[0]), float(F_tot[1])
        self.pub_force.publish(fv)

        # ---- Local minima detection ------------------------------------
        speed = math.hypot(self._robot_vx, self._robot_vy)
        if speed < self._stuck_v:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._stuck_start is None:
                self._stuck_start = now
            elif (now - self._stuck_start) > self._stuck_t:
                self.get_logger().warn(
                    f'Local minimum detected! Speed={speed:.3f} m/s for '
                    f'{now - self._stuck_start:.1f} s. Initiating escape.')
                self._state        = PlannerState.ESCAPING
                self._escape_start = now
                self._escape_dir   = float(np.random.choice([-1.0, 1.0]))
                self._stuck_start  = None
                return
        else:
            self._stuck_start = None

        # ---- Safety stop ------------------------------------------------
        min_dist = self._get_min_obstacle_distance()
        if min_dist < self._d_safe:
            self._publish_stop()
            self.get_logger().warn(
                f'Safety stop! Obstacle at {min_dist:.2f} m. Initiating INSTANT escape!')
            self._state        = PlannerState.ESCAPING
            self._escape_start = self.get_clock().now().nanoseconds * 1e-9
            self._escape_dir   = float(np.random.choice([-1.0, 1.0]))
            self._stuck_start  = None
            return

        # ---- Convert force to velocity command --------------------------
        cmd = self._force_to_cmd(F_tot)
        self.pub_cmd.publish(cmd)

        # ---- Record path history ----------------------------------------
        self._path_history.append((self._robot_x, self._robot_y))
        if len(self._path_history) > 500:
            self._path_history.pop(0)

        self._publish_path_markers(F_att, F_rep)

    # -------------------------------------------------------------------- #
    # Attractive force (conic-well model)                                   #
    # -------------------------------------------------------------------- #
    def _attractive_force(self) -> np.ndarray:
        """
        Compute 2-D attractive force toward the goal using the conic-well model.
        """
        dx = self._goal_x - self._robot_x
        dy = self._goal_y - self._robot_y
        d  = math.hypot(dx, dy)

        if d < 1e-6:
            return np.zeros(2)

        unit = np.array([dx / d, dy / d])

        if d <= self._d_star:
            # Quadratic region (close to goal) — linear force
            F = self._k_att * d * unit
        else:
            # Conic region (far from goal) — constant magnitude
            F = self._d_star * self._k_att * unit

        return F

    # -------------------------------------------------------------------- #
    # Repulsive force (inverse-square law)                                  #
    # -------------------------------------------------------------------- #
    def _repulsive_force(self) -> np.ndarray:
        """
        Compute 2-D repulsive force using the gradient of the inverse-square
        potential, summed over all LiDAR rays within the influence radius.
        Uses a vectorised approach for efficiency.
        """
        if self._scan is None:
            return np.zeros(2)

        scan   = self._scan
        ranges = np.array(scan.ranges, dtype=np.float64)
        n      = len(ranges)
        angles = np.arange(n) * scan.angle_increment + scan.angle_min

        # Filter: finite, within [min, influence_radius]
        valid = np.isfinite(ranges) & (ranges >= 0.05) & (ranges <= self._d_inf)
        if not np.any(valid):
            return np.zeros(2)

        r_v   = ranges[valid]
        th_v  = angles[valid]

        # Obstacle directions in robot frame
        ox = r_v * np.cos(th_v)
        oy = r_v * np.sin(th_v)

        # Gradient coefficient: k_rep * (1/d - 1/d0) / d^2
        coeff = self._k_rep * (1.0 / r_v - 1.0 / self._d_inf) / (r_v ** 2)

        # ∇d_obs points toward the obstacle; force is −∇U_rep so away
        # F_rep_i = coeff_i * (−unit_toward_obs) = coeff_i * (unit_away)
        fx = -coeff * (ox / r_v)
        fy = -coeff * (oy / r_v)

        # Sum contributions (weighted by influence — closer obstacles dominate)
        F_robot = np.array([np.sum(fx), np.sum(fy)])

        # Rotate to world frame
        yaw = self._robot_yaw
        c, s = np.cos(yaw), np.sin(yaw)
        rot_mat = np.array([[c, -s],
                            [s,  c]])
        F_total = rot_mat.dot(F_robot)

        # Clip to prevent numerical explosion near obstacles
        mag = np.linalg.norm(F_total)
        clip_mag = 8.0
        if mag > clip_mag:
            F_total = F_total / mag * clip_mag

        return F_total

    # -------------------------------------------------------------------- #
    # Force → Twist conversion                                              #
    # -------------------------------------------------------------------- #
    def _force_to_cmd(self, F: np.ndarray) -> Twist:
        """
        Convert the 2-D force vector (in robot's world frame)
        to a Twist command (linear.x + angular.z) for a differential drive.
        """
        cmd = Twist()

        if np.linalg.norm(F) < 1e-6:
            return cmd

        # Desired heading from force direction
        desired_yaw = math.atan2(F[1], F[0])
        yaw_error   = self._normalise_angle(desired_yaw - self._robot_yaw)

        # Linear velocity: proportional to force magnitude, scaled down when
        # robot must turn sharply (cosine envelope)
        force_mag = np.linalg.norm(F)
        turn_penalty = max(0.0, math.cos(yaw_error))  # 0 for 90° turn
        v_lin = self._f2v * force_mag * turn_penalty

        # Angular velocity: proportional to yaw error
        v_ang = 2.0 * yaw_error

        # Clamp
        cmd.linear.x  = float(np.clip(v_lin,  0.0,           self._max_lin))
        cmd.angular.z = float(np.clip(v_ang, -self._max_ang, self._max_ang))

        return cmd

    # -------------------------------------------------------------------- #
    # Escape manoeuvre (local minima recovery)                              #
    # -------------------------------------------------------------------- #
    def _execute_escape(self):
        """
        Two-phase escape:
          Phase 1 (0 → escape_rotate_time):   rotate in place
          Phase 2 (escape_rotate_time → end):  drive forward
          End: return to NAVIGATING
        """
        now     = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._escape_start

        cmd = Twist()

        phase_1_end = self._esc_rot_t
        phase_2_end = self._esc_rot_t + self._esc_mov_t

        if elapsed < phase_1_end:
            # Phase 1: rotate
            cmd.angular.z = self._escape_dir * self._esc_ang_v
        elif elapsed < phase_2_end:
            # Phase 2: move forward
            cmd.linear.x  = self._esc_lin_v
        else:
            # Escape complete — resume normal APF
            self._state = PlannerState.NAVIGATING
            self.get_logger().info('Escape manoeuvre complete. Resuming APF navigation.')
            return

        self.pub_cmd.publish(cmd)

    # -------------------------------------------------------------------- #
    # Helpers                                                               #
    # -------------------------------------------------------------------- #
    def _get_min_obstacle_distance(self) -> float:
        if self._scan is None:
            return float('inf')
        ranges = np.array(self._scan.ranges)
        valid  = np.isfinite(ranges) & (ranges > 0.01)
        return float(np.min(ranges[valid])) if np.any(valid) else float('inf')

    def _publish_stop(self):
        self.pub_cmd.publish(Twist())

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        """Wrap angle to (−π, π]."""
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # -------------------------------------------------------------------- #
    # RViz path + force markers                                             #
    # -------------------------------------------------------------------- #
    def _publish_path_markers(self, F_att: np.ndarray, F_rep: np.ndarray):
        array = MarkerArray()

        # ---- Breadcrumb path trail ---------------------------------------
        if len(self._path_history) > 1:
            path_m            = Marker()
            path_m.header.frame_id = 'odom'
            path_m.header.stamp    = self.get_clock().now().to_msg()
            path_m.ns         = 'apf_trail'
            path_m.id         = 0
            path_m.type       = Marker.LINE_STRIP
            path_m.action     = Marker.ADD
            path_m.scale.x    = 0.03
            path_m.color      = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.7)
            for (px, py) in self._path_history:
                p   = Point()
                p.x = px; p.y = py; p.z = 0.05
                path_m.points.append(p)
            array.markers.append(path_m)

        header_msg = rclpy.node.node.get_clock(self).now().to_msg() \
            if False else self.get_clock().now().to_msg()

        # ---- Attractive force arrow ------------------------------------
        att_m = self._force_arrow(
            'apf_attractive', 1, F_att,
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9))
        array.markers.append(att_m)

        # ---- Repulsive force arrow ------------------------------------
        rep_m = self._force_arrow(
            'apf_repulsive', 2, F_rep,
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9))
        array.markers.append(rep_m)

        # ---- Total force arrow -----------------------------------------
        tot_m = self._force_arrow(
            'apf_total', 3, F_att + F_rep,
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9))
        array.markers.append(tot_m)

        # ---- Goal marker -----------------------------------------------
        goal_m                     = Marker()
        goal_m.header.frame_id     = 'odom'
        goal_m.header.stamp        = self.get_clock().now().to_msg()
        goal_m.ns                  = 'apf_goal'
        goal_m.id                  = 4
        goal_m.type                = Marker.CYLINDER
        goal_m.action              = Marker.ADD
        goal_m.pose.position.x     = self._goal_x
        goal_m.pose.position.y     = self._goal_y
        goal_m.pose.position.z     = 0.1
        goal_m.pose.orientation.w  = 1.0
        goal_m.scale.x = goal_m.scale.y = self._goal_tol * 2.0
        goal_m.scale.z = 0.2
        goal_m.color  = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        array.markers.append(goal_m)

        self.pub_markers.publish(array)

    def _force_arrow(self, ns: str, mid: int,
                     F: np.ndarray, colour: ColorRGBA) -> Marker:
        """Build an arrow Marker representing a 2-D force vector."""
        m               = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp  = self.get_clock().now().to_msg()
        m.ns            = ns
        m.id            = mid
        m.type          = Marker.ARROW
        m.action        = Marker.ADD
        # Arrow from origin to force direction
        start           = Point(); start.x = self._robot_x; start.y = self._robot_y; start.z = 0.2
        scale           = 0.3
        end             = Point()
        end.x = self._robot_x + float(F[0]) * scale
        end.y = self._robot_y + float(F[1]) * scale
        end.z = 0.2
        m.points        = [start, end]
        m.scale.x       = 0.05   # shaft diameter
        m.scale.y       = 0.10   # head diameter
        m.scale.z       = 0.10   # head length
        m.color         = colour
        m.lifetime.nanosec = int(1e9 / self._ctrl_rate * 2)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = APFPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
