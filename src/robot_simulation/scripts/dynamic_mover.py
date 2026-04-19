#!/usr/bin/env python3
"""
dynamic_mover.py
================
Moves pedestrian AND vehicle models along predefined paths in
Gazebo Harmonic (gz sim) by calling the /world/<world>/set_pose service.

Pedestrians: walk along sidewalks and crosswalks.
Moving cars: drive slowly through the central driving aisle.

Introduces random pauses to prevent repetitive/predictable obstacle motion.
Uses blocking subprocess calls with timeout to prevent process pile-up.
"""

import rclpy
from rclpy.node import Node
import subprocess
import math
import time
import threading
import random


class DynamicPath:
    """Defines a linear path for one dynamic object with random pausing."""

    def __init__(self, name: str,
                 start_x: float, start_y: float,
                 end_x: float, end_y: float,
                 speed: float = 0.4,
                 z_offset: float = 0.0,
                 loop_type: str = 'pingpong'):
        self.name = name
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.speed = speed
        self.z_offset = z_offset
        self.loop_type = loop_type

        dx = end_x - start_x
        dy = end_y - start_y
        self.length = math.hypot(dx, dy)
        self.dir_x = dx / self.length if self.length > 1e-6 else 0.0
        self.dir_y = dy / self.length if self.length > 1e-6 else 0.0

        self.yaw_fwd = math.atan2(dy, dx)
        self.yaw_rev = self.yaw_fwd + math.pi
        
        # State variables for pausing
        self.virtual_time = 0.0
        self.last_real_time = 0.0
        self.is_paused = False
        self.pause_until = 0.0
        self.next_pause_check = 5.0 # Check for pause soon

    def update_virtual_time(self, real_time: float):
        dt = real_time - self.last_real_time
        self.last_real_time = real_time
        
        if self.is_paused:
            if real_time > self.pause_until:
                self.is_paused = False
                # Schedule next pause check
                self.next_pause_check = real_time + random.uniform(5.0, 15.0)
        else:
            self.virtual_time += dt
            # Check if we should pause
            if real_time > self.next_pause_check:
                if random.random() < 0.6:  # 60% chance to pause
                    self.is_paused = True
                    self.pause_until = real_time + random.uniform(2.0, 5.0)
                else:
                    self.next_pause_check = real_time + random.uniform(5.0, 15.0)

    def get_pose(self, real_time: float):
        """Compute (x, y, z, yaw) at the given time."""
        if self.last_real_time == 0.0:
            self.last_real_time = real_time
            
        self.update_virtual_time(real_time)
            
        one_way_time = self.length / self.speed if self.speed > 0 else 1e9

        if self.loop_type == 'stop':
            if self.virtual_time >= one_way_time:
                x = self.end_x
                y = self.end_y
                yaw = self.yaw_fwd
            else:
                frac = self.virtual_time / one_way_time
                x = self.start_x + frac * (self.end_x - self.start_x)
                y = self.start_y + frac * (self.end_y - self.start_y)
                yaw = self.yaw_fwd
            return x, y, self.z_offset, yaw

        cycle_time = 2.0 * one_way_time
        t = self.virtual_time % cycle_time

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

        return x, y, self.z_offset, yaw


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


class DynamicMoverNode(Node):
    """Moves pedestrian and vehicle models in Gazebo along predefined paths."""

    def __init__(self):
        super().__init__('dynamic_mover')

        self.declare_parameter('world_name', 'obstacle_world')
        self.declare_parameter('update_rate', 4.0)

        self._world = self.get_parameter('world_name').value
        rate = self.get_parameter('update_rate').value
        
        # New paths matching the new perpendicular world layout
        self._paths = [
            # Pedestrian 1: North sidewalk back and forth (y=8.25)
            DynamicPath('pedestrian_1', 2.0, 8.25, 18.0, 8.25, speed=0.4),
            # Pedestrian 2: South sidewalk back and forth (y=-8.25)
            DynamicPath('pedestrian_2', 18.0, -8.25, 2.0, -8.25, speed=0.35),
            # Pedestrian 3: Crosswalk at x=10 (gap between parked cars)
            DynamicPath('pedestrian_3', 10.0, -8.0, 10.0, 8.0, speed=0.45),
            # Pedestrian 4: North sidewalk, shorter range (y=8.25)
            DynamicPath('pedestrian_4', 12.0, 8.25, 5.0, 8.25, speed=0.3),
            
            # Moving car 1: Central aisle (Eastbound, y=1.8 to stay out of chicane)
            DynamicPath('moving_car_1', 0.0, 1.8, 25.0, 1.8, speed=1.2, z_offset=0.65, loop_type='stop'),
        ]

        self._start_time = time.time()
        self._busy = False

        self._timer = self.create_timer(1.0 / rate, self._update)

        obj_names = [p.name for p in self._paths]
        self.get_logger().info(
            f'DynamicMover ready. Moving {obj_names} at {rate} Hz (with random pausing)')

    def _update(self):
        """Update all dynamic object positions."""
        if self._busy:
            return
        self._busy = True
        thread = threading.Thread(target=self._move_all, daemon=True)
        thread.start()

    def _move_all(self):
        """Move all objects (runs in background thread)."""
        try:
            elapsed = time.time() - self._start_time
            for path in self._paths:
                x, y, z, yaw = path.get_pose(elapsed)
                self._set_model_pose(path.name, x, y, z, 0.0, 0.0, yaw)
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
    node = DynamicMoverNode()
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
