# Path Planning of a Mobile Robot with Real-Time Feedback from Mounted Camera and LiDAR Sensors

## Directory Structure

```
mobile_robot_ws/
├── src/
│   ├── obstacle_msgs/                  # Custom ROS 2 message definitions
│   │   ├── msg/
│   │   │   ├── Obstacle.msg            # Single fused obstacle (+ velocity, track_id)
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
│   │   │   ├── goal_publisher.py       # One-shot goal sender
│   │   │   └── pedestrian_mover.py     # Dynamic obstacle controller
│   │   ├── launch/
│   │   │   ├── simulation.launch.py    # Gazebo + robot spawn + RViz2
│   │   │   └── full_system.launch.py   # ★ Master launch (all nodes)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── lidar_processing/               # LiDAR filter + obstacle extraction
│   │   ├── lidar_processing/
│   │   │   ├── __init__.py
│   │   │   └── lidar_filter_node.py    # Filter + median + PointCloud2
│   │   ├── launch/
│   │   │   └── lidar.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/lidar_processing
│   │
│   ├── camera_processing/              # Camera HSV detection
│   │   ├── camera_processing/
│   │   │   ├── __init__.py
│   │   │   └── camera_node.py          # Rectify + blur + HSV + ObstacleArray
│   │   ├── launch/
│   │   │   └── camera.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/camera_processing
│   │
│   ├── sensor_fusion/                  # LiDAR–Camera fusion + Kalman tracking
│   │   ├── sensor_fusion/
│   │   │   ├── __init__.py
│   │   │   └── fusion_node.py          # ApproxTimeSync + projection + Kalman tracker
│   │   ├── launch/
│   │   │   └── fusion.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/sensor_fusion
│   │
│   ├── apf_planner/                    # APF planner (velocity-aware)
│   │   ├── apf_planner/
│   │   │   ├── __init__.py
│   │   │   └── apf_node.py             # APF + predictive repulsion + supervisor mode
│   │   ├── launch/
│   │   │   └── planner.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/apf_planner
│   │
│   ├── mpc_controller/                 # MPC local planner (NEW)
│   │   ├── mpc_controller/
│   │   │   ├── __init__.py
│   │   │   └── mpc_node.py             # Receding-horizon MPC with SLSQP solver
│   │   ├── launch/
│   │   │   └── mpc.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/mpc_controller
│   │
│   ├── rl_planner/                     # RL planner (NEW)
│   │   ├── rl_planner/
│   │   │   ├── __init__.py
│   │   │   ├── rl_node.py              # RL inference node (PPO/SAC)
│   │   │   └── rl_env.py               # Gymnasium environment for training
│   │   ├── scripts/
│   │   │   └── train_rl.py             # Standalone training script
│   │   ├── launch/
│   │   │   └── rl.launch.py
│   │   ├── models/                     # Trained model checkpoints
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/rl_planner
│   │
│   ├── planner_supervisor/             # Planner orchestrator (NEW)
│   │   ├── planner_supervisor/
│   │   │   ├── __init__.py
│   │   │   └── supervisor_node.py      # APF/MPC/RL switching state machine
│   │   ├── launch/
│   │   │   └── supervisor.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │   └── resource/planner_supervisor
│   │
│   └── evaluation/                     # Benchmarking (NEW)
│       ├── evaluation/
│       │   ├── __init__.py
│       │   └── eval_node.py            # Metrics logger (CSV output)
│       ├── scripts/
│       │   └── plot_results.py         # Matplotlib comparison plots
│       ├── launch/
│       │   └── eval.launch.py
│       ├── setup.py, setup.cfg, package.xml
│       └── resource/evaluation
```

---

## Dependencies

### ROS 2 Distribution
- **ROS 2 Jazzy** (Ubuntu 24.04 or Noble)
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

### Optional (for RL training)
```bash
pip install stable-baselines3 gymnasium matplotlib
```

---

## Build

```bash
cd ~/mobile_robot_planning
colcon build --symlink-install
source install/setup.bash
```

---

## Running

### Option 1 — Full System (APF + MPC augmentation, default)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_simulation full_system.launch.py
```

### Planner Mode Options

| Mode | Command | Description |
|------|---------|-------------|
| **APF + MPC** (default) | `ros2 launch robot_simulation full_system.launch.py planner_mode:=mpc` | APF with MPC for local minimum escape |
| **APF + RL** | `ros2 launch robot_simulation full_system.launch.py planner_mode:=rl` | APF with RL policy for local minimum escape |
| **Pure APF** | `ros2 launch robot_simulation full_system.launch.py planner_mode:=none` | APF only (built-in random-walk escape) |

Custom goal:
```bash
ros2 launch robot_simulation full_system.launch.py goal_x:=12.0 goal_y:=0.0 planner_mode:=mpc
```

### Option 2 — Individual Nodes (for debugging)

| Node | Launch Command | Topic Output |
|------|----------------|--------------| 
| **Simulation** | `ros2 launch robot_simulation simulation.launch.py` | `/scan`, `/camera/image_raw` |
| **LiDAR Filter** | `ros2 launch lidar_processing lidar.launch.py` | `/scan_filtered` |
| **Camera Proc** | `ros2 launch camera_processing camera.launch.py` | `/camera/detections` |
| **Sensor Fusion** | `ros2 launch sensor_fusion fusion.launch.py` | `/fused_obstacles` |
| **APF Planner** | `ros2 launch apf_planner planner.launch.py` | `/apf_cmd_vel` |
| **MPC Controller** | `ros2 launch mpc_controller mpc.launch.py` | `/mpc_cmd_vel` |
| **RL Planner** | `ros2 launch rl_planner rl.launch.py` | `/rl_cmd_vel` |
| **Supervisor** | `ros2 launch planner_supervisor supervisor.launch.py` | `/cmd_vel`, `/planner_mode` |
| **Evaluation** | `ros2 launch evaluation eval.launch.py` | `/eval_metrics` |

---

### RL Training (Optional)

```bash
cd src/rl_planner
python3 scripts/train_rl.py --algorithm ppo --total-timesteps 500000 --output models/nav_policy
```

Use trained model:
```bash
ros2 launch robot_simulation full_system.launch.py planner_mode:=rl
# In a separate terminal, set the model path:
ros2 launch rl_planner rl.launch.py model_path:=/path/to/models/nav_policy.zip
```

---

## System Architecture

```mermaid
graph TD
    LiDAR[/scan/] --> LF[lidar_filter_node]
    LF --> SF[/scan_filtered/]
    LF --> OC[/obstacles_cloud/]

    Cam[/camera/image_raw/] --> CN[camera_node]
    CN --> CD[/camera/detections/]
    CN --> SR[/camera/image_rectified/]

    SF --> FN[fusion_node<br>+ Kalman Tracker]
    CD --> FN
    FN --> FO[/fused_obstacles/<br>with velocity]

    SF --> APF[apf_planner]
    FO --> APF
    GP[goal_publisher] --> APF
    Odom[/odom/] --> APF
    APF --> ACMD[/apf_cmd_vel/]

    SF --> MPC[mpc_controller]
    FO --> MPC
    Odom --> MPC
    GP --> MPC
    MPC --> MCMD[/mpc_cmd_vel/]

    SF --> RL[rl_planner]
    Odom --> RL
    GP --> RL
    RL --> RCMD[/rl_cmd_vel/]

    ACMD --> SUP[planner_supervisor]
    MCMD --> SUP
    RCMD --> SUP
    Odom --> SUP
    SUP --> CV[/cmd_vel/]
    SUP --> PM[/planner_mode/]
    PM --> APF
    PM --> MPC
    PM --> RL

    Odom --> EVAL[eval_node]
    SF --> EVAL
    PM --> EVAL
    EVAL --> EM[/eval_metrics/]
```

---

## Sensor Specifications

| Sensor | Parameter | Value | Notes |
|--------|-----------|-------|-------|
| **Hokuyo UTM-30LX** | FOV | 270° | Centred on robot front |
| | Angular Res | 0.25° | 1080 rays per scan |
| | Range | 0.1 - 30.0 m | Filtered at 10 Hz |
| **RGB Camera** | Resolution | 640 × 480 | 30 FPS |
| | H-FOV | 80° | Tilted 5° down |
| | Distortion | [0, 0, 0, 0, 0] | Pinhole model (Ideal) |

---

## APF Path Planner

Implements an **Artificial Potential Field** (APF) with predictive obstacle avoidance.

### Features
- **Conic-Well Attractive Field**: Quadratic near goal, conic far from goal
- **Inverse-Square Repulsive Field**: Force ∝ 1/d², strong avoidance near obstacles
- **Predictive Repulsion**: Uses Kalman filter velocity estimates to repel from predicted obstacle positions
- **Supervisor Integration**: Publishes to `/apf_cmd_vel`, responds to `/planner_mode`
- **Local Minima Recovery**: Managed by the planner supervisor (delegates to MPC or RL)

### Parameters

| Parameter | Key | Value | Symbol |
|-----------|-----|-------|--------|
| **Attractive Gain** | `k_att` | **1.2** | $\xi$ |
| **Switch Distance** | `d_star` | **1.5 m** | $d^*$ |
| **Repulsive Gain** | `k_rep` | **1.5** | $\eta$ |
| **Influence Radius** | `d_influence` | **1.5 m** | $\rho_0$ |
| **Safety Radius** | `d_safe` | **0.25 m** | $d_{safe}$ |
| **Max Linear Vel** | `max_linear_vel` | **3.5 m/s** | $v_{max}$ |
| **Max Angular Vel** | `max_angular_vel` | **4.0 rad/s** | $\omega_{max}$ |
| **Prediction Horizon** | `prediction_horizon` | **1.0 s** | $T_{pred}$ |

---

## MPC Controller

**Model Predictive Control** with receding-horizon optimization for local minimum escape.

### Features
- **Unicycle Kinematic Model**: x' = v·cos(θ), y' = v·sin(θ), θ' = ω
- **10-step Horizon** (1.0s lookahead) with warm-start from previous solution
- **SLSQP Solver**: Sequential quadratic programming via scipy.optimize
- **Obstacle Barrier Penalty**: Exponential penalty near obstacles
- **Control Smoothness**: Penalises abrupt control changes

### Parameters

| Parameter | Value |
|-----------|-------|
| Horizon Steps | 10 |
| Timestep | 0.1 s |
| v_max | 1.5 m/s |
| ω_max | 2.5 rad/s |
| Obstacle clearance | 0.4 m |

---

## RL Planner

**Reinforcement Learning** policy for learned obstacle avoidance (PPO/SAC).

### Features
- **PPO Policy** (MlpPolicy, 256×256 hidden layers)
- **Observation**: Goal direction + distance + LiDAR sectors (8 sectors) + robot velocity
- **Action**: Continuous [v, ω] normalised to [-1, 1]
- **Fallback**: Hand-crafted reactive policy when no trained model available
- **Training**: Separate Gymnasium environment with simplified kinematics (no Gazebo needed)

---

## Planner Supervisor

Orchestrates switching between APF, MPC, and RL planners.

### State Machine
1. **APF** (default) → monitor for stuck state
2. If stuck for 3.0s with < 0.1m progress → switch to MPC/RL
3. If augmented planner achieves 0.5m progress → return to APF
4. If timeout (15s) → return to APF
5. If goal within 1.0m → return to APF

---

## Evaluation

Logs four performance metrics per episode:

| Metric | Description |
|--------|-------------|
| **Path Length** | Total distance travelled (m) |
| **Time to Goal** | Wall-clock time from goal to arrival (s) |
| **Collision Count** | Obstacles closer than 0.25m |
| **Mode Switches** | Number of APF↔MPC/RL transitions |

Results saved to `eval_results/summary.csv`. Plot comparisons:
```bash
python3 src/evaluation/scripts/plot_results.py eval_results/summary.csv
```

---

## Performance & Known Issues

- **Sensor Fusion**: Functional with Kalman filter tracking. Velocity estimates converge after ~3 updates.
- **Dynamic Obstacles**: APF uses predictive repulsive field from tracked obstacle velocities.
- **MPC**: SLSQP solver runs at ~10 Hz on standard hardware. Falls back gracefully if solver fails.
- **RL**: Requires separate training phase. Fallback reactive policy provides basic functionality.
- **Map Integration**: Operates in `odom` frame. Migration to `map` frame with SLAM planned for future.
