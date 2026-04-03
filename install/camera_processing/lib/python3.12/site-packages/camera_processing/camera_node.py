#!/usr/bin/env python3
"""
camera_node.py
==============
Task C — Camera Integration

Subscribes to:
  /camera/image_raw    (sensor_msgs/Image)     — raw RGB frames at 30 fps
  /camera/camera_info  (sensor_msgs/CameraInfo) — intrinsic parameters

Publishes:
  /camera/image_rectified  (sensor_msgs/Image)        — undistorted frames
  /camera/image_segmented  (sensor_msgs/Image)        — HSV mask debug view
  /camera/detections       (obstacle_msgs/ObstacleArray) — bounding boxes

Processing pipeline:
  1. Convert ROS Image → OpenCV BGR
  2. Rectification (undistort) using camera_info K and D
  3. Gaussian blur (kernel 5×5, σ=1.5) to reduce sensor noise
  4. HSV colour segmentation for dynamic obstacle markers:
       Red   (hue 0-10 + 170-180) — pedestrian_1 hat
       Green (hue 40-80)          — pedestrian_2 hat
  5. Morphological clean-up (open + close) on binary mask
  6. Connected-component bounding boxes with area filter
  7. Publish ObstacleArray + annotated debug image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

# Custom messages (built from obstacle_msgs package)
from obstacle_msgs.msg import Obstacle, ObstacleArray


# ─── Colour profile definition ────────────────────────────────────────────── #
COLOUR_PROFILES = {
    'red': {
        'ranges': [
            (np.array([  0,  80, 80]), np.array([ 10, 255, 255])),
            (np.array([170,  80, 80]), np.array([180, 255, 255])),
        ],
        'is_dynamic': True,
    },
    'green': {
        'ranges': [
            (np.array([ 40,  60, 60]), np.array([ 80, 255, 255])),
        ],
        'is_dynamic': True,
    },
    'blue': {
        'ranges': [
            (np.array([100,  80, 80]), np.array([130, 255, 255])),
        ],
        'is_dynamic': False,   # static obstacle marker (optional)
    },
}
# ──────────────────────────────────────────────────────────────────────────── #


class CameraNode(Node):
    """Camera preprocessing and HSV-based dynamic obstacle detection node."""

    def __init__(self):
        super().__init__('camera_node')

        # ---------------------------------------------------------------- #
        # Parameters                                                        #
        # ---------------------------------------------------------------- #
        self.declare_parameter('gaussian_kernel', 5)
        self.declare_parameter('gaussian_sigma',  1.5)
        self.declare_parameter('min_bbox_area',   300)   # pixels²
        self.declare_parameter('max_bbox_area',   50000) # pixels²
        self.declare_parameter('publish_debug',   True)

        self._gauss_k      = self.get_parameter('gaussian_kernel').value
        self._gauss_sigma  = self.get_parameter('gaussian_sigma').value
        self._min_area     = self.get_parameter('min_bbox_area').value
        self._max_area     = self.get_parameter('max_bbox_area').value
        self._pub_debug    = self.get_parameter('publish_debug').value

        # ---------------------------------------------------------------- #
        # Camera intrinsics (will be populated from /camera/camera_info)   #
        # ---------------------------------------------------------------- #
        self._K    = None   # 3×3 intrinsic matrix
        self._D    = None   # distortion coefficients
        self._img_w = 640
        self._img_h = 480
        self._map1  = None  # precomputed undistort maps
        self._map2  = None

        # ---------------------------------------------------------------- #
        # Morphological kernels                                             #
        # ---------------------------------------------------------------- #
        self._morph_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._morph_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        # ---------------------------------------------------------------- #
        # CV bridge                                                         #
        # ---------------------------------------------------------------- #
        self._bridge = CvBridge()

        # ---------------------------------------------------------------- #
        # QoS                                                               #
        # ---------------------------------------------------------------- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ---------------------------------------------------------------- #
        # Subscribers                                                       #
        # ---------------------------------------------------------------- #
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.camera_info_callback, sensor_qos)

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, sensor_qos)

        # ---------------------------------------------------------------- #
        # Publishers                                                        #
        # ---------------------------------------------------------------- #
        self.pub_rect     = self.create_publisher(Image,         '/camera/image_rectified',  10)
        self.pub_seg      = self.create_publisher(Image,         '/camera/image_segmented',  10)
        self.pub_detect   = self.create_publisher(ObstacleArray, '/camera/detections',       10)

        # ---------------------------------------------------------------- #
        # Statistics                                                        #
        # ---------------------------------------------------------------- #
        self._frame_count = 0
        self._total_detections = 0

        self.get_logger().info('CameraNode ready. Waiting for camera_info...')

    # -------------------------------------------------------------------- #
    # Camera Info callback — build undistort maps once                     #
    # -------------------------------------------------------------------- #
    def camera_info_callback(self, msg: CameraInfo):
        if self._K is not None:
            return   # already initialised

        self._img_w = msg.width
        self._img_h = msg.height

        # Intrinsic matrix K  (row-major 3×3 from 9-element array)
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        # Distortion coefficients (up to 5 for plumb_bob model)
        self._D = np.array(msg.d, dtype=np.float64)

        # Precompute undistort maps
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self._K, self._D, None, self._K,
            (self._img_w, self._img_h), cv2.CV_32FC1)

        self.get_logger().info(
            f'Camera calibration loaded: '
            f'{self._img_w}×{self._img_h}, '
            f'K={self._K.diagonal()[:2].tolist()}, '
            f'D={self._D.tolist()}'
        )

    # -------------------------------------------------------------------- #
    # Image callback — main processing pipeline                            #
    # -------------------------------------------------------------------- #
    def image_callback(self, msg: Image):
        # Gate on camera info
        if self._K is None:
            return

        self._frame_count += 1

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # ---- Step 1: Rectification (undistort) ---------------------------
        if self._map1 is not None:
            rect = cv2.remap(bgr, self._map1, self._map2, cv2.INTER_LINEAR)
        else:
            rect = bgr.copy()

        # Publish rectified image
        rect_msg = self._bridge.cv2_to_imgmsg(rect, encoding='bgr8')
        rect_msg.header = msg.header
        self.pub_rect.publish(rect_msg)

        # ---- Step 2: Gaussian blur ----------------------------------------
        blurred = cv2.GaussianBlur(
            rect,
            (self._gauss_k, self._gauss_k),
            self._gauss_sigma
        )

        # ---- Step 3: HSV conversion ---------------------------------------
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ---- Step 4 & 5: Colour segmentation + morphological cleanup -----
        obstacle_list = []
        debug_overlay = rect.copy() if self._pub_debug else None

        for colour_name, profile in COLOUR_PROFILES.items():
            combined_mask = np.zeros(
                (self._img_h, self._img_w), dtype=np.uint8)

            for (lo, hi) in profile['ranges']:
                combined_mask = cv2.bitwise_or(
                    combined_mask, cv2.inRange(hsv, lo, hi))

            # Open (remove small noise) then close (fill gaps)
            mask = cv2.morphologyEx(
                combined_mask, cv2.MORPH_OPEN,  self._morph_open)
            mask = cv2.morphologyEx(
                mask,           cv2.MORPH_CLOSE, self._morph_close)

            # ---- Step 6: Bounding boxes ----------------------------------
            bboxes = self._extract_bboxes(mask)

            for (bx, by, bw, bh, conf) in bboxes:
                obs              = Obstacle()
                obs.header       = msg.header
                obs.bbox_x       = float(bx)
                obs.bbox_y       = float(by)
                obs.bbox_w       = float(bw)
                obs.bbox_h       = float(bh)
                obs.label        = colour_name
                obs.confidence   = conf
                obs.is_dynamic   = profile['is_dynamic']
                # 3-D position unknown at camera stage (filled by fusion node)
                obs.x = obs.y = obs.z = 0.0
                obs.width = obs.height = obs.depth = 0.0
                obstacle_list.append(obs)

                if debug_overlay is not None:
                    colour_bgr = {
                        'red':   (0,   0, 255),
                        'green': (0, 255,   0),
                        'blue':  (255,  0,   0),
                    }.get(colour_name, (255, 255, 0))
                    cv2.rectangle(debug_overlay,
                                  (bx, by), (bx + bw, by + bh),
                                  colour_bgr, 2)
                    cv2.putText(debug_overlay,
                                f'{colour_name} {conf:.2f}',
                                (bx, by - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                colour_bgr, 1)

        # ---- Step 7: Publish detections ----------------------------------
        det_array         = ObstacleArray()
        det_array.header  = msg.header
        det_array.obstacles = obstacle_list
        self.pub_detect.publish(det_array)

        self._total_detections += len(obstacle_list)

        # ---- Debug segmentation image ------------------------------------
        if self._pub_debug and debug_overlay is not None:
            seg_msg = self._bridge.cv2_to_imgmsg(debug_overlay, encoding='bgr8')
            seg_msg.header = msg.header
            self.pub_seg.publish(seg_msg)

        # ---- Periodic log -------------------------------------------------
        if self._frame_count % 150 == 0:
            avg_det = self._total_detections / self._frame_count
            self.get_logger().info(
                f'[Camera stats] frames={self._frame_count}, '
                f'avg_detections/frame={avg_det:.2f}')

    # -------------------------------------------------------------------- #
    # Bounding box extraction helper                                        #
    # -------------------------------------------------------------------- #
    def _extract_bboxes(self, mask: np.ndarray):
        """
        Find connected components in binary mask, filter by area,
        return list of (x, y, w, h, confidence) tuples.
        Confidence is approximated from the solidity of the contour.
        """
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)

        bboxes = []
        for label in range(1, num_labels):   # skip background (label 0)
            area = stats[label, cv2.CC_STAT_AREA]
            if not (self._min_area <= area <= self._max_area):
                continue

            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            w = stats[label, cv2.CC_STAT_WIDTH]
            h = stats[label, cv2.CC_STAT_HEIGHT]

            # Confidence: ratio of filled pixels to bounding box area
            bbox_area = max(w * h, 1)
            conf = float(min(area / bbox_area, 1.0))

            bboxes.append((x, y, w, h, conf))

        return bboxes


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
