#!/usr/bin/env python3
"""
pedestrian_mover.py
===================
Moves pedestrian models (dynamic obstacles) along predefined paths in
Gazebo Harmonic (gz sim) by calling the /world/<world>/set_pose service.

Each pedestrian follows a simple back-and-forth linear trajectory at a
configurable speed, simulating walking humans in the environment.

Uses blocking subprocess calls with timeout to prevent process pile-up.
"""

import rclpy
from rclpy.node import Node
import subprocess
import math
import time
import threading


class PedestrianPath:
    """Defines a linear back-and-forth path for one pedestrian."""

    def __init__(self, name: str,
                 start_x: float, start_y: float,
                 end_x: float, end_y: float,
                 speed: float = 0.4):
        self.name = name
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.speed = speed  # m/s

        dx = end_x - start_x
        dy = end_y - start_y
        self.length = math.hypot(dx, dy)
        self.dir_x = dx / self.length if self.length > 1e-6 else 0.0
        self.dir_y = dy / self.length if self.length > 1e-6 else 0.0

        # Yaw angle for the pedestrian to face direction of travel
        self.yaw_fwd = math.atan2(dy, dx)
        self.yaw_rev = self.yaw_fwd + math.pi

    def get_pose(self, elapsed: float):
        """Compute (x, y, yaw) at the given elapsed time."""
        one_way_time = self.length / self.speed if self.speed > 0 else 1e9
        cycle_time = 2.0 * one_way_time
        t = elapsed % cycle_time

        if t < one_way_time:
            frac = t / one_way_time
            x = self.start_x + frac * (self.end_x - self.start_x)
            y = self.start_y + frac * (self.end_y - self.start_y)
            yaw = self.yaw_fwd
        else:
            frac = (t - one_way_time) / one_way_time
            x = self.end_x + frac * (self.start_x - self.end_x)
            y = self.end_y + frac * (self.start_y - self.end_y)
            yaw = self.yaw_rev

        return x, y, yaw


def euler_to_quat(roll, pitch, yaw):
    """Convert Euler angles to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PedestrianMoverNode(Node):
    """Moves pedestrian models in Gazebo along predefined paths."""

    def __init__(self):
        super().__init__('pedestrian_mover')

        self.declare_parameter('world_name', 'obstacle_world')
        self.declare_parameter('update_rate', 4.0)  # Hz (kept low to avoid overloading gz transport)

        self._world = self.get_parameter('world_name').value
        rate = self.get_parameter('update_rate').value

        # Define pedestrian paths
        self._paths = [
            PedestrianPath(
                name='pedestrian_1',
                start_x=4.0, start_y=1.0,
                end_x=4.0,   end_y=-1.5,
                speed=0.35
            ),
            PedestrianPath(
                name='pedestrian_2',
                start_x=5.0, start_y=-1.0,
                end_x=6.5,   end_y=-1.0,
                speed=0.3
            ),
        ]

        self._start_time = time.time()
        self._busy = False  # guard against overlapping updates

        self._timer = self.create_timer(1.0 / rate, self._update)

        ped_names = [p.name for p in self._paths]
        self.get_logger().info(
            f'PedestrianMover ready. Moving {ped_names} at {rate} Hz')

    def _update(self):
        """Update all pedestrian positions."""
        if self._busy:
            return  # skip if previous update is still running
        self._busy = True

        # Run in a thread so the ROS spin isn't blocked by subprocess calls
        thread = threading.Thread(target=self._move_all, daemon=True)
        thread.start()

    def _move_all(self):
        """Move all pedestrians (runs in background thread)."""
        try:
            elapsed = time.time() - self._start_time
            for path in self._paths:
                x, y, yaw = path.get_pose(elapsed)
                self._set_model_pose(path.name, x, y, 0.0, 0.0, 0.0, yaw)
        finally:
            self._busy = False

    def _set_model_pose(self, model_name: str,
                        x: float, y: float, z: float,
                        roll: float, pitch: float, yaw: float):
        """Set the pose of a model in gz sim via blocking service call."""
        qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
        service = f'/world/{self._world}/set_pose'
        req = (
            f'gz service -s {service} '
            f'--reqtype gz.msgs.Pose '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 200 '
            f'--req \'name: "{model_name}", '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, '
            f'z: {qz:.6f}, w: {qw:.6f}}}\''
        )
        try:
            subprocess.run(
                req, shell=True,
                timeout=1.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianMoverNode()
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
