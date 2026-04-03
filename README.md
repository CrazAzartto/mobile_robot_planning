# Path Planning of a Mobile Robot with Real-Time Feedback from Mounted Camera and LiDAR Sensors

## Project Overview

| Task | Description | Owner | Status |
|------|-------------|-------|--------|
| A | Gazebo Simulation Environment | Sunil Bishnoi | ✅ Complete |
| B | LiDAR Integration & Filtering | Sunil Bishnoi | ✅ Complete |
| C | Camera Integration & HSV Segmentation | Sunny Kumar | ✅ Complete |
| D | Sensor Fusion Pipeline | Sunny Kumar | 🔄 Partial |
| E | APF Path Planner | Sunil Bishnoi | ✅ Complete |

---

## Directory Structure

```
mobile_robot_ws/
├── src/
│   ├── obstacle_msgs/                  # Custom ROS 2 message definitions
│   │   ├── msg/
│   │   │   ├── Obstacle.msg            # Single fused obstacle
│   │   │   └── ObstacleArray.msg       # Array of obstacles
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── robot_description/              # Robot URDF + RViz config
│   │   ├── urdf/
│   │   │   └── robot.urdf.xacro        # Differential drive + Hokuyo + Camera
│   │   ├── rviz/
│   │   │   └── robot.rviz              # Full RViz2 display configuration
│   │   ├── launch/
│   │   │   └── display.launch.py       # URDF-only visualisation (no Gazebo)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── robot_simulation/               # Gazebo world + master launch files
│   │   ├── worlds/
│   │   │   └── obstacle_world.world    # World with static boxes + dynamic actors
│   │   ├── scripts/
│   │   │   └── goal_publisher.py       # One-shot goal sender
│   │   ├── launch/
│   │   │   ├── simulation.launch.py    # Gazebo + robot spawn + RViz2
│   │   │   └── full_system.launch.py   # ★ Master launch (all nodes)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── lidar_processing/               # Task B — LiDAR node
│   │   ├── lidar_processing/
│   │   │   ├── __init__.py
│   │   │   └── lidar_filter_node.py    # Filter + median + PointCloud2
│   │   ├── launch/
│   │   │   └── lidar.launch.py
│   │   ├── resource/lidar_processing
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   ├── camera_processing/              # Task C — Camera node
│   │   ├── camera_processing/
│   │   │   ├── __init__.py
│   │   │   └── camera_node.py          # Rectify + blur + HSV + ObstacleArray
│   │   ├── launch/
│   │   │   └── camera.launch.py
│   │   ├── resource/camera_processing
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   ├── sensor_fusion/                  # Task D — Fusion node
│   │   ├── sensor_fusion/
│   │   │   ├── __init__.py
│   │   │   └── fusion_node.py          # ApproxTimeSync + projection + map
│   │   ├── launch/
│   │   │   └── fusion.launch.py
│   │   ├── resource/sensor_fusion
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   └── apf_planner/                    # Task E — APF planner
│       ├── apf_planner/
│       │   ├── __init__.py
│       │   └── apf_node.py             # APF + local minima escape
│       ├── launch/
│       │   └── planner.launch.py
│       ├── resource/apf_planner
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
```

---

## Dependencies

### ROS 2 Distribution
- **ROS 2 Jazzy** (Ubuntu 24.04 or Noble) — current system
- **Gazebo Harmonic** (via `gz-sim`)

### ROS 2 Packages
```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf-transformations \
  ros-jazzy-message-filters \
  ros-jazzy-cv-bridge-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-rviz2 \
  python3-opencv \
  python3-scipy \
  python3-numpy \
  python3-transforms3d
```

---

## Build

```bash
cd ~/mobile_robot_ws
colcon build --symlink-install
source install/setup.bash
```

Build a single package (faster during development):
```bash
colcon build --packages-select obstacle_msgs
colcon build --packages-select robot_description robot_simulation
colcon build --packages-select lidar_processing camera_processing sensor_fusion apf_planner
```

---

## Running

### Option 1 — Full System (Simulation + Fused Perception + APF)
This command handles Gazebo, sensor processing (LiDAR + Camera), fusion, and the autonomous planner.

```bash
# Sourcing (Ensure both ROS and the workspace are sourced)
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the full simulation
ros2 launch robot_simulation full_system.launch.py
```

With a custom goal (optional):
```bash
ros2 launch robot_simulation full_system.launch.py goal_x:=12.0 goal_y:=0.0
```

### Option 2 — Individual Nodes (for debugging)

**Terminal 1 — Simulation only:**
```bash
ros2 launch robot_simulation simulation.launch.py
```

**Terminal 2 — LiDAR filter:**
```bash
ros2 launch lidar_processing lidar.launch.py
```

**Terminal 3 — Camera processing:**
```bash
ros2 launch camera_processing camera.launch.py
```

**Terminal 4 — Sensor fusion:**
```bash
ros2 launch sensor_fusion fusion.launch.py
```

**Terminal 5 — APF planner:**
```bash
ros2 launch apf_planner planner.launch.py
```

**Terminal 6 — Send a goal manually:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{ header: { frame_id: "odom" },
     pose: { position: { x: 12.0, y: 0.0, z: 0.0 },
             orientation: { w: 1.0 } } }'
```

---

## Topic Map

```
/scan                   ← Gazebo Hokuyo plugin (10 Hz)
        │
        ▼
/scan_filtered          ← lidar_filter_node  (invalid removal + median)
/obstacles_cloud        ← lidar_filter_node  (PointCloud2)
/obstacle_positions     ← lidar_filter_node  (MarkerArray for RViz)

/camera/image_raw       ← Gazebo camera plugin (30 fps)
/camera/camera_info     ← Gazebo camera plugin
        │
        ▼
/camera/image_rectified ← camera_node  (undistorted)
/camera/image_segmented ← camera_node  (HSV debug overlay)
/camera/detections      ← camera_node  (ObstacleArray with bboxes)

/scan_filtered  ─┐
                 ├─ ApproximateTimeSynchronizer ─▶ fusion_node
/camera/detections ─┘
        │
        ▼
/fused_obstacles        ← fusion_node  (position-labelled ObstacleArray)
/fused_markers          ← fusion_node  (MarkerArray for RViz)

/odom                   ← diff_drive_controller
/scan_filtered          ┐
/goal_pose              ├─▶ apf_planner
/fused_obstacles (opt.) ┘
        │
        ▼
/cmd_vel                ← apf_planner  (Twist to robot)
/apf_force              ← apf_planner  (Vector3 debug)
/apf_path_markers       ← apf_planner  (MarkerArray path trail)
```

---

## Sensor Specifications

| Sensor | Parameter | Value |
|--------|-----------|-------|
| Hokuyo UTM-30LX | FOV | 270° |
| | Angular resolution | 0.25° (1080 rays) |
| | Max range | 30 m |
| | Update rate | 10 Hz |
| RGB Camera | Resolution | 640 × 480 |
| | Frame rate | 30 fps |
| | H-FOV | 80° |

---

## APF Planner Parameters

| Parameter | Symbol | Value | Notes |
|-----------|--------|-------|-------|
| Attractive gain | k_att | 1.2 | Quadratic → conic switch |
| Conic switch distance | d_star | 1.5 m | |
| Repulsive gain | k_rep | 1.5 | Inverse-square |
| Influence radius | d0 | 1.5 m | Zero force beyond this |
| Safety stop | d_safe | 0.25 m | Emergency stop |
| Stuck velocity | v_stuck | 0.03 m/s | Minima detection threshold |
| Stuck timeout | t_stuck | 3.0 s | Before escape triggers |

---

## Known Issues / Future Work

- **Task D**: Fusion node needs calibration verification with real extrinsics
- **Local Minima**: APF observed in narrow-corridor scenarios — MPC/RL augmentation planned
- **Dynamic Obstacles**: APF repulsive field reacts to LiDAR returns only; fused velocity
  estimation of dynamic obstacles can further improve avoidance
- **Map Integration**: Currently uses `odom` frame; migration to `map` frame with SLAM planned
