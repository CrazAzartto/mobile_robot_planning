#!/usr/bin/env python3
"""
fusion_node.py
==============
Task D — Preliminary Sensor Fusion Pipeline

Subscribes (time-synchronised):
  /scan_filtered       (sensor_msgs/LaserScan)       — filtered LiDAR @ 10 Hz
  /camera/detections   (obstacle_msgs/ObstacleArray) — camera bounding boxes

Also subscribes (non-synced):
  /camera/camera_info  (sensor_msgs/CameraInfo)       — camera intrinsics

Publishes:
  /fused_obstacles     (obstacle_msgs/ObstacleArray)  — fused obstacle map
  /fused_markers       (visualization_msgs/MarkerArray) — RViz visualisation

Algorithm:
  1. Time-synchronise LiDAR and camera messages (ApproximateTime, 0.1 s slack)
  2. For each LiDAR ray within the camera's horizontal FOV:
       a. Compute 3-D point P_lidar = (r·cos θ, r·sin θ, 0) in laser_link frame
       b. Transform to camera_link frame via precomputed T_cam_lidar
       c. Project onto image plane:  [u, v]ᵀ = K · [X/Z, Y/Z, 1]ᵀ
  3. For each camera bounding box, collect LiDAR projections that land inside it
  4. Estimate obstacle 3-D position as median of valid LiDAR depths
  5. Build fused ObstacleArray and publish

Extrinsic calibration T_cam_lidar is loaded from parameter or computed from TF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import message_filters

import numpy as np
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf_transformations          # from tf_transformations package

from sensor_msgs.msg import LaserScan, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from obstacle_msgs.msg import Obstacle, ObstacleArray


class FusionNode(Node):
    """Fuses filtered LiDAR scans with camera-based bounding box detections."""

    def __init__(self):
        super().__init__('fusion_node')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        self.declare_parameter('sync_slop',         0.10)  # seconds
        self.declare_parameter('lidar_frame',  'laser_link')
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('camera_hfov_deg',   80.0)
        self.declare_parameter('lidar_min_range',    0.10)
        self.declare_parameter('lidar_max_range',   30.0)
        self.declare_parameter('bbox_depth_margin',  0.05)  # pixel margin for projection match

        self._lidar_frame   = self.get_parameter('lidar_frame').value
        self._camera_frame  = self.get_parameter('camera_frame').value
        self._hfov_rad      = np.deg2rad(self.get_parameter('camera_hfov_deg').value)
        self._sync_slop     = self.get_parameter('sync_slop').value
        self._lidar_min     = self.get_parameter('lidar_min_range').value
        self._lidar_max     = self.get_parameter('lidar_max_range').value
        self._margin        = self.get_parameter('bbox_depth_margin').value

        # ---------------------------------------------------------------- #
        # Camera intrinsics (filled from /camera/camera_info)              #
        # ---------------------------------------------------------------- #
        self._K    = None       # 3×3
        self._img_w = 640
        self._img_h = 480

        # ---------------------------------------------------------------- #
        # Extrinsic transform: T_cam_lidar (4×4 homogeneous)               #
        # Loaded from TF tree; if TF lookup fails, use identity            #
        # ---------------------------------------------------------------- #
        self._T_cam_lidar = None

        # ---------------------------------------------------------------- #
        # TF buffer                                                         #
        # ---------------------------------------------------------------- #
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------------------------------------------------------- #
        # QoS                                                               #
        # ---------------------------------------------------------------- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---------------------------------------------------------------- #
        # Camera info subscriber (non-synchronised)                        #
        # ---------------------------------------------------------------- #
        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self._camera_info_cb, sensor_qos)

        # ---------------------------------------------------------------- #
        # ApproximateTime synchroniser for LiDAR + camera detections       #
        # ---------------------------------------------------------------- #
        self._sub_scan = message_filters.Subscriber(
            self, LaserScan, '/scan_filtered',
            qos_profile=sensor_qos)
        self._sub_det  = message_filters.Subscriber(
            self, ObstacleArray, '/camera/detections',
            qos_profile=sensor_qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_scan, self._sub_det],
            queue_size=10,
            slop=self._sync_slop
        )
        self._sync.registerCallback(self._fusion_callback)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_fused   = self.create_publisher(
            ObstacleArray,  '/fused_obstacles', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray,    '/fused_markers',   10)

        # ---------------------------------------------------------------- #
        # Attempt initial TF lookup (may fail until TF tree is ready)      #
        # ---------------------------------------------------------------- #
        self.create_timer(1.0, self._try_load_extrinsic)

        self._fusion_count = 0
        self.get_logger().info('FusionNode initialised. Waiting for sensor data...')

    # -------------------------------------------------------------------- #
    # Camera info callback                                                  #
    # -------------------------------------------------------------------- #
    def _camera_info_cb(self, msg: CameraInfo):
        if self._K is not None:
            return
        self._K    = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._img_w = msg.width
        self._img_h = msg.height
        self.get_logger().info(f'Fusion: camera intrinsics loaded K={self._K[0,0]:.1f}')

    # -------------------------------------------------------------------- #
    # Periodic TF lookup for extrinsic calibration                         #
    # -------------------------------------------------------------------- #
    def _try_load_extrinsic(self):
        if self._T_cam_lidar is not None:
            return  # already loaded
        try:
            tf_stamped: TransformStamped = self._tf_buffer.lookup_transform(
                self._camera_frame, self._lidar_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))

            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            self._T_cam_lidar = tf_transformations.concatenate_matrices(
                tf_transformations.translation_matrix([t.x, t.y, t.z]),
                tf_transformations.quaternion_matrix([r.x, r.y, r.z, r.w])
            )
            self.get_logger().info(
                f'Extrinsic T_cam_lidar loaded from TF: '
                f'trans=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')

        except TransformException:
            # Fall back to fixed offset from URDF: camera is ~0.03 m forward,
            # 0.08 m below LiDAR, rotated −90° around Z then −90° around X
            # (matches the camera_optical_joint in robot.urdf.xacro)
            self._T_cam_lidar = self._default_extrinsic()
            self.get_logger().warn(
                'TF lookup failed — using default extrinsic from URDF geometry')

    @staticmethod
    def _default_extrinsic() -> np.ndarray:
        """
        Fixed transform from laser_link to camera_optical_link
        derived from the URDF joint offsets.
        """
        # Translation: camera is 0.02 m forward, -0.08 m below laser_link
        t = np.array([0.02, 0.0, -0.08])
        # Rotation: camera_optical frame is rotated -pi/2 around X, then -pi/2 around Z
        Rx = tf_transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
        Rz = tf_transformations.rotation_matrix(-np.pi / 2, [0, 0, 1])
        R  = tf_transformations.concatenate_matrices(Rz, Rx)
        T  = tf_transformations.translation_matrix(t)
        return tf_transformations.concatenate_matrices(T, R)

    # -------------------------------------------------------------------- #
    # Main fusion callback                                                  #
    # -------------------------------------------------------------------- #
    def _fusion_callback(self, scan_msg: LaserScan, det_msg: ObstacleArray):
        """Fuse synchronised LiDAR scan with camera detections."""

        if self._K is None:
            return
        if self._T_cam_lidar is None:
            self._try_load_extrinsic()
            return

        self._fusion_count += 1

        # ---- Project LiDAR points into camera image space ----------------
        projected = self._project_lidar_to_image(scan_msg)
        # projected: list of (u, v, range_m, x_robot, y_robot) for each valid ray

        # ---- Associate projections with bounding boxes -------------------
        fused_obstacles = []

        for obs in det_msg.obstacles:
            bb_x  = obs.bbox_x
            bb_y  = obs.bbox_y
            bb_w  = obs.bbox_w
            bb_h  = obs.bbox_h

            # Expand bounding box slightly to catch edge projections
            pad = 5
            in_bbox = [
                (u, v, r, xr, yr) for (u, v, r, xr, yr) in projected
                if (bb_x - pad <= u <= bb_x + bb_w + pad and
                    bb_y - pad <= v <= bb_y + bb_h + pad)
            ]

            if len(in_bbox) == 0:
                # No LiDAR depth available; still include from camera
                fused_obs = Obstacle()
                fused_obs.header     = scan_msg.header
                fused_obs.bbox_x     = obs.bbox_x
                fused_obs.bbox_y     = obs.bbox_y
                fused_obs.bbox_w     = obs.bbox_w
                fused_obs.bbox_h     = obs.bbox_h
                fused_obs.label      = obs.label
                fused_obs.confidence = obs.confidence * 0.5  # reduced without depth
                fused_obs.is_dynamic = obs.is_dynamic
                fused_obs.x = fused_obs.y = fused_obs.z = 0.0
                fused_obstacles.append(fused_obs)
                continue

            # Median depth estimate (robust to outliers)
            ranges    = np.array([r  for (_, _, r, _, _)  in in_bbox])
            x_vals    = np.array([xr for (_, _, _, xr, _) in in_bbox])
            y_vals    = np.array([yr for (_, _, _, _, yr) in in_bbox])

            med_range = float(np.median(ranges))
            med_x     = float(np.median(x_vals))
            med_y     = float(np.median(y_vals))

            # Width estimate from horizontal spread of LiDAR points in bbox
            spread_y  = float(np.percentile(y_vals, 90) - np.percentile(y_vals, 10))

            fused_obs            = Obstacle()
            fused_obs.header     = scan_msg.header
            fused_obs.x          = med_x
            fused_obs.y          = med_y
            fused_obs.z          = 0.0
            fused_obs.depth      = 0.5    # approximate (LiDAR is 2-D scan)
            fused_obs.width      = max(spread_y, 0.2)
            fused_obs.height     = obs.bbox_h / (self._K[1, 1] / max(med_range, 0.1))
            fused_obs.bbox_x     = obs.bbox_x
            fused_obs.bbox_y     = obs.bbox_y
            fused_obs.bbox_w     = obs.bbox_w
            fused_obs.bbox_h     = obs.bbox_h
            fused_obs.label      = obs.label
            fused_obs.confidence = min(obs.confidence + 0.2, 1.0)  # boosted
            fused_obs.is_dynamic = obs.is_dynamic
            fused_obstacles.append(fused_obs)

        # ---- Also include LiDAR-only detections (not matched to camera) --
        # (Useful for static obstacles outside camera FOV)
        matched_pts = set()
        for obs in fused_obstacles:
            if obs.x != 0.0 or obs.y != 0.0:
                matched_pts.add((round(obs.x, 2), round(obs.y, 2)))

        # Cluster unmatched LiDAR points into additional obstacles
        unmatched_pts = [
            (xr, yr, r) for (u, v, r, xr, yr) in projected
            if (round(xr, 2), round(yr, 2)) not in matched_pts
        ]
        lidar_only_obs = self._cluster_lidar_points(unmatched_pts, scan_msg.header)
        fused_obstacles.extend(lidar_only_obs)

        # ---- Publish -------------------------------------------------------
        out_array           = ObstacleArray()
        out_array.header    = scan_msg.header
        out_array.obstacles = fused_obstacles
        self.pub_fused.publish(out_array)

        markers = self._build_fused_markers(fused_obstacles, scan_msg.header)
        self.pub_markers.publish(markers)

        if self._fusion_count % 50 == 0:
            self.get_logger().info(
                f'[Fusion] synced_pairs={self._fusion_count}, '
                f'fused_obstacles={len(fused_obstacles)} '
                f'(camera={len(det_msg.obstacles)}, '
                f'lidar_only={len(lidar_only_obs)})')

    # -------------------------------------------------------------------- #
    # Project LiDAR rays into image coordinates                            #
    # -------------------------------------------------------------------- #
    def _project_lidar_to_image(self, scan: LaserScan):
        """
        For each valid LiDAR ray, compute:
          - 3-D point in laser_link frame
          - Project to camera_optical_link frame via T_cam_lidar
          - Project to image pixel (u, v) via K
        Returns list of (u, v, range_m, x_robot, y_robot).
        """
        fx = self._K[0, 0]
        cx = self._K[0, 2]
        fy = self._K[1, 1]
        cy = self._K[1, 2]

        results = []
        n_rays  = len(scan.ranges)
        angles  = (np.arange(n_rays) * scan.angle_increment + scan.angle_min)
        ranges  = np.array(scan.ranges)

        # Filter rays within camera horizontal FOV
        hfov_half = self._hfov_rad / 2.0

        for i in range(n_rays):
            r = ranges[i]
            if not (self._lidar_min <= r <= self._lidar_max * 0.99):
                continue

            theta = angles[i]

            # Only rays inside camera FOV ± small margin
            if abs(theta) > hfov_half + 0.05:
                continue

            # 3-D point in laser_link frame (z=0, flat scan)
            P_lidar = np.array([r * np.cos(theta), r * np.sin(theta), 0.0, 1.0])

            # Transform to camera frame
            P_cam = self._T_cam_lidar @ P_lidar

            # Depth in camera frame must be positive
            if P_cam[2] <= 0.01:
                continue

            # Project to pixel
            u = fx * (P_cam[0] / P_cam[2]) + cx
            v = fy * (P_cam[1] / P_cam[2]) + cy

            # Check inside image bounds
            if not (0 <= u < self._img_w and 0 <= v < self._img_h):
                continue

            x_robot = float(r * np.cos(theta))
            y_robot = float(r * np.sin(theta))
            results.append((float(u), float(v), float(r), x_robot, y_robot))

        return results

    # -------------------------------------------------------------------- #
    # Simple LiDAR-only clustering (proximity-based)                       #
    # -------------------------------------------------------------------- #
    @staticmethod
    def _cluster_lidar_points(pts, header) -> list:
        """
        Naive proximity clustering of unmatched LiDAR points.
        Returns a list of Obstacle messages (lidar-only, no colour label).
        """
        if not pts:
            return []

        cluster_eps = 0.3  # metres
        clusters    = []
        used        = [False] * len(pts)

        for i, (xi, yi, ri) in enumerate(pts):
            if used[i]:
                continue
            cluster = [(xi, yi, ri)]
            used[i] = True
            for j, (xj, yj, rj) in enumerate(pts):
                if used[j]:
                    continue
                if np.hypot(xi - xj, yi - yj) < cluster_eps:
                    cluster.append((xj, yj, rj))
                    used[j] = True
            if len(cluster) >= 3:   # minimum points to form obstacle
                clusters.append(cluster)

        obstacles = []
        for cl in clusters:
            xs = [p[0] for p in cl]
            ys = [p[1] for p in cl]
            obs              = Obstacle()
            obs.header       = header
            obs.x            = float(np.mean(xs))
            obs.y            = float(np.mean(ys))
            obs.z            = 0.0
            obs.width        = float(max(ys) - min(ys) + 0.1)
            obs.depth        = float(max(xs) - min(xs) + 0.1)
            obs.height       = 1.0
            obs.label        = 'lidar_only'
            obs.confidence   = 0.6
            obs.is_dynamic   = False
            obstacles.append(obs)
        return obstacles

    # -------------------------------------------------------------------- #
    # RViz marker builder                                                   #
    # -------------------------------------------------------------------- #
    @staticmethod
    def _build_fused_markers(obstacles: list, header) -> MarkerArray:
        array = MarkerArray()
        del_m = Marker(); del_m.header = header; del_m.action = Marker.DELETEALL
        array.markers.append(del_m)

        colour_map = {
            'red':        ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9),
            'green':      ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9),
            'blue':       ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.9),
            'lidar_only': ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.7),
        }

        for idx, obs in enumerate(obstacles):
            m              = Marker()
            m.header       = header
            m.ns           = 'fused_obstacles'
            m.id           = idx
            m.type         = Marker.CUBE
            m.action       = Marker.ADD
            m.pose.position.x     = obs.x
            m.pose.position.y     = obs.y
            m.pose.position.z     = obs.height / 2.0
            m.pose.orientation.w  = 1.0
            m.scale.x = max(obs.depth,  0.2)
            m.scale.y = max(obs.width,  0.2)
            m.scale.z = max(obs.height, 0.5)
            m.color = colour_map.get(obs.label, ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8))
            m.lifetime.nanosec = 500_000_000
            array.markers.append(m)

            # Text label marker
            t              = Marker()
            t.header       = header
            t.ns           = 'fused_labels'
            t.id           = 10000 + idx
            t.type         = Marker.TEXT_VIEW_FACING
            t.action       = Marker.ADD
            t.pose.position.x = obs.x
            t.pose.position.y = obs.y
            t.pose.position.z = obs.height + 0.3
            t.pose.orientation.w = 1.0
            t.text         = f'{obs.label}\n{obs.confidence:.2f}'
            t.scale.z      = 0.25
            t.color        = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.lifetime.nanosec = 500_000_000
            array.markers.append(t)

        return array


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
