#!/usr/bin/env python3
"""
lidar_filter_node.py
====================
Task B — LiDAR Integration

Subscribes to:
  /scan                  (sensor_msgs/LaserScan)  — raw Hokuyo UTM-30LX data

Publishes:
  /scan_filtered         (sensor_msgs/LaserScan)  — cleaned scan at 10 Hz
  /obstacles_cloud       (sensor_msgs/PointCloud2) — obstacle Cartesian coords
  /obstacle_positions    (visualization_msgs/MarkerArray) — RViz markers

Processing pipeline:
  1. Remove invalid returns (NaN, Inf, out-of-range)
  2. Replace invalid readings with range_max (open space)
  3. Apply median filter (kernel = 5) to reduce impulse noise
  4. Extract obstacle positions (x, y) in robot frame
  5. Publish filtered scan + PointCloud2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from scipy.signal import medfilt

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import struct


class LidarFilterNode(Node):
    """Filters and processes raw LiDAR scans from the Hokuyo UTM-30LX."""

    def __init__(self):
        super().__init__('lidar_filter_node')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        self.declare_parameter('min_range',          0.10)   # metres
        self.declare_parameter('max_range',         30.0)    # metres
        self.declare_parameter('median_kernel_size',    5)   # must be odd
        self.declare_parameter('obstacle_threshold', 0.95)   # fraction of max
        self.declare_parameter('cluster_eps',         0.20)  # DBSCAN ε (m)
        self.declare_parameter('cluster_min_pts',        3)  # DBSCAN min pts

        self._min_range  = self.get_parameter('min_range').value
        self._max_range  = self.get_parameter('max_range').value
        self._kernel     = self.get_parameter('median_kernel_size').value
        self._obs_thresh = self.get_parameter('obstacle_threshold').value

        # ---------------------------------------------------------------- #
        # QoS — match Gazebo sensor publishing profile                     #
        # ---------------------------------------------------------------- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---------------------------------------------------------------- #
        # Subscribers                                                       #
        # ---------------------------------------------------------------- #
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_filtered  = self.create_publisher(LaserScan,    '/scan_filtered',      10)
        self.pub_cloud     = self.create_publisher(PointCloud2,  '/obstacles_cloud',    10)
        self.pub_markers   = self.create_publisher(MarkerArray,  '/obstacle_positions', 10)

        # ---------------------------------------------------------------- #
        # Statistics                                                        #
        # ---------------------------------------------------------------- #
        self._msg_count   = 0
        self._total_valid = 0

        self.get_logger().info(
            f'LidarFilterNode ready. '
            f'range=[{self._min_range}, {self._max_range}] m, '
            f'median_kernel={self._kernel}')

    # -------------------------------------------------------------------- #
    # Main callback                                                         #
    # -------------------------------------------------------------------- #
    def scan_callback(self, msg: LaserScan):
        """Process one LaserScan message through the full filtering pipeline."""

        self._msg_count += 1
        ranges = np.array(msg.ranges, dtype=np.float32)
        n_rays  = len(ranges)

        # ---- Step 1: Remove invalid returns (NaN / Inf / out of range) ----
        valid_mask = (
            np.isfinite(ranges) &
            (ranges >= self._min_range) &
            (ranges <= self._max_range)
        )
        # Replace invalid readings with max_range (treated as "no obstacle")
        ranges_clean = np.where(valid_mask, ranges, self._max_range)

        n_valid = int(np.sum(valid_mask))
        self._total_valid += n_valid

        # ---- Step 2: Median filter ----------------------------------------
        kernel = self._kernel if self._kernel % 2 == 1 else self._kernel + 1
        ranges_filtered = medfilt(ranges_clean, kernel_size=kernel).astype(np.float32)

        # ---- Step 3: Build filtered LaserScan message ---------------------
        filtered_msg = LaserScan()
        filtered_msg.header            = msg.header
        filtered_msg.angle_min         = msg.angle_min
        filtered_msg.angle_max         = msg.angle_max
        filtered_msg.angle_increment   = msg.angle_increment
        filtered_msg.time_increment    = msg.time_increment
        filtered_msg.scan_time         = msg.scan_time
        filtered_msg.range_min         = self._min_range
        filtered_msg.range_max         = self._max_range
        filtered_msg.ranges            = ranges_filtered.tolist()
        filtered_msg.intensities       = (
            msg.intensities if len(msg.intensities) == n_rays else []
        )
        self.pub_filtered.publish(filtered_msg)

        # ---- Step 4: Extract obstacle Cartesian positions -----------------
        angles = (
            np.arange(n_rays, dtype=np.float32) * msg.angle_increment
            + msg.angle_min
        )
        obstacle_mask = (
            valid_mask &
            (ranges_filtered < self._max_range * self._obs_thresh)
        )
        obs_ranges = ranges_filtered[obstacle_mask]
        obs_angles = angles[obstacle_mask]

        x_coords = obs_ranges * np.cos(obs_angles)
        y_coords = obs_ranges * np.sin(obs_angles)
        z_coords = np.zeros_like(x_coords)

        # ---- Step 5: Publish PointCloud2 ----------------------------------
        cloud_msg = self._build_point_cloud2(
            msg.header, x_coords, y_coords, z_coords)
        self.pub_cloud.publish(cloud_msg)

        # ---- Step 6: Publish RViz markers ---------------------------------
        marker_array = self._build_markers(
            msg.header, x_coords, y_coords)
        self.pub_markers.publish(marker_array)

        # ---- Periodic statistics ------------------------------------------
        if self._msg_count % 100 == 0:
            avg_valid = self._total_valid / self._msg_count
            self.get_logger().info(
                f'[LiDAR stats] msgs={self._msg_count}, '
                f'avg_valid_rays={avg_valid:.1f}/{n_rays}, '
                f'obstacles_this_scan={len(x_coords)}')

    # -------------------------------------------------------------------- #
    # PointCloud2 builder                                                   #
    # -------------------------------------------------------------------- #
    @staticmethod
    def _build_point_cloud2(header: Header,
                            x: np.ndarray,
                            y: np.ndarray,
                            z: np.ndarray) -> PointCloud2:
        """Pack (x, y, z) float32 arrays into a PointCloud2 message."""

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12   # 3 × float32
        n_pts      = len(x)
        data       = bytearray(n_pts * point_step)

        for i in range(n_pts):
            offset = i * point_step
            struct.pack_into('fff', data, offset, float(x[i]), float(y[i]), float(z[i]))

        cloud = PointCloud2()
        cloud.header     = header
        cloud.height     = 1
        cloud.width      = n_pts
        cloud.is_dense   = True
        cloud.is_bigendian = False
        cloud.fields     = fields
        cloud.point_step = point_step
        cloud.row_step   = n_pts * point_step
        cloud.data       = bytes(data)
        return cloud

    # -------------------------------------------------------------------- #
    # RViz Marker builder                                                   #
    # -------------------------------------------------------------------- #
    @staticmethod
    def _build_markers(header: Header,
                       x: np.ndarray,
                       y: np.ndarray) -> MarkerArray:
        """Build a sphere marker for each obstacle point (for RViz2 debugging)."""

        array = MarkerArray()

        # Delete all previous markers first
        del_marker         = Marker()
        del_marker.header  = header
        del_marker.action  = Marker.DELETEALL
        array.markers.append(del_marker)

        for i, (xi, yi) in enumerate(zip(x, y)):
            m              = Marker()
            m.header       = header
            m.ns           = 'lidar_obstacles'
            m.id           = i
            m.type         = Marker.SPHERE
            m.action       = Marker.ADD
            m.pose.position.x = float(xi)
            m.pose.position.y = float(yi)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.8)
            # Lifetime: auto-expire after 0.5 s so stale markers disappear
            m.lifetime.sec     = 0
            m.lifetime.nanosec = 500_000_000
            array.markers.append(m)

        return array


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
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
