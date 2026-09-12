# Trajectory Collector

Automated trajectory execution, multi-modal sensor recording (MCAP), and synchronized playback for Clearpath Husky A200 in ROS 2 Jazzy and Gazebo.

---

## Quick Start (30 Seconds)

### 1. Build & Source

```bash
cd ~/husky_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select trajectory_collector
source install/setup.bash
```

### 2. Collect a Trajectory

Run full Gazebo simulation, Nav2 stack, autonomous navigation, and MCAP rosbag recording with a single command:

```bash
ros2 launch trajectory_collector collect_trajectory.launch.py goal_x:=2.0 goal_y:=2.0
```

The robot spawns, localizes with AMCL, updates its costmap, drives to the target pose, and saves an MCAP bag in `~/husky_ws/data/trajectories/`.

### 3. Watch the Trajectory Playback

Replay the recorded run with preconfigured RViz visualization (chassis, driven path, LiDAR, and RGB camera):

```bash
ros2 launch trajectory_collector view_bag.launch.py bag:=~/husky_ws/data/trajectories/<bag_name>
```

---

## Core Launch Tools

### 1. Automated Trajectory Collection (`collect_trajectory.launch.py`)

Sequences the entire stack:
1. Launches Gazebo simulation (`warehouse` world) and Clearpath robot platform.
2. Gates execution via `sim_gate` until the robot is spawned and odometry is active.
3. Starts Nav2 navigation, AMCL localization, RViz, and LiDAR footprint self-filtering.
4. Auto-publishes initial pose and waits for AMCL localization confirmation.
5. `navigate_to_goal` waits for `bt_navigator` and local costmap sensor readiness, then initiates MCAP bag recording and dispatches the navigation goal.
6. Automatically finalizes and flushes the MCAP bag on arrival or timeout.

```bash
ros2 launch trajectory_collector collect_trajectory.launch.py [arguments]
```

#### Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `goal_x` | `2.0` | Target X coordinate (meters) |
| `goal_y` | `2.0` | Target Y coordinate (meters) |
| `goal_yaw` | `0.0` | Target yaw orientation (radians) |
| `record_bag` | `true` | Record trajectory and sensor streams (`true` / `false`) |
| `bag_directory`| `~/husky_ws/data/trajectories` | Output directory for rosbags |
| `bag_name` | `""` | Bag folder name (defaults to `traj_<YYYYMMDD_HHMMSS>`) |
| `timeout` | `120.0` | Maximum seconds allowed to reach the goal |
| `auto_shutdown`| `false` | Automatically shutdown simulation stack upon goal completion |
| `world` | `warehouse` | Gazebo world name |

---

### 2. Playback & Visualization (`view_bag.launch.py`)

Opens RViz loaded with `bag_playback.rviz` under `/a200_0000` namespace and synchronizes with simulation clock.

```bash
# Play and view a specific bag
ros2 launch trajectory_collector view_bag.launch.py bag:=~/husky_ws/data/trajectories/<bag_name>

# Playback at 2x speed in a continuous loop
ros2 launch trajectory_collector view_bag.launch.py bag:=<path> rate:=2.0 loop:=true

# Open viewer only (awaits manual 'ros2 bag play' in another terminal)
ros2 launch trajectory_collector view_bag.launch.py
```

---

### 3. Simulation Bringup Only (`bringup.launch.py`)

Brings up Gazebo simulation, Nav2 stack, AMCL, and RViz without sending an automated goal (useful for interactive manual navigation in RViz):

```bash
ros2 launch trajectory_collector bringup.launch.py
```

---

## Recorded Data (MCAP Format)

Trajectories are recorded in the corruption-resilient **MCAP** storage format. The recorded topics capture the minimal set required for 6-DOF spatial path reconstruction plus perception streams:

| Category | Topic | Message Type | Purpose |
| :--- | :--- | :--- | :--- |
| **Transforms** | `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms (`odom -> base_link`, `map -> odom`) |
| | `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms (sensors, chassis) |
| **Odometry & Control**| `/a200_0000/platform/odom` | `nav_msgs/msg/Odometry` | Wheel odometry velocity & pose |
| | `/a200_0000/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL localization history |
| | `/a200_0000/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Commanded robot velocity |
| **LiDAR** | `/a200_0000/sensors/lidar2d_0/scan` | `sensor_msgs/msg/LaserScan` | Raw 2D laser scan |
| | `/a200_0000/sensors/lidar2d_0/scan_filtered` | `sensor_msgs/msg/LaserScan` | Self-filtered laser scan (arch excluded) |
| **Camera** | `/a200_0000/sensors/camera_0/color/image` | `sensor_msgs/msg/Image` | Color RGB camera stream |
| | `/a200_0000/sensors/camera_0/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration parameters |

### Inspecting Bag Metadata

```bash
ros2 bag info ~/husky_ws/data/trajectories/<bag_name>
```

### Viewing in Foxglove Studio (No-ROS GUI)

Because files are stored in native `.mcap` format, you can also drag and drop the recorded `.mcap` file directly into **Foxglove Studio** ([studio.foxglove.dev](https://studio.foxglove.dev)) to scrub the trajectory timeline, view 3D transforms, camera feeds, and LiDAR point clouds.

---

## Package Components

* [`collect_trajectory.launch.py`](launch/collect_trajectory.launch.py): Top-level orchestrator for simulation, navigation, and bag capture.
* [`view_bag.launch.py`](launch/view_bag.launch.py): Single-command bag playback and RViz visualizer.
* [`bringup.launch.py`](launch/bringup.launch.py): Simulation and navigation bringup with gated synchronization.
* [`navigate_to_goal.py`](trajectory_collector/navigate_to_goal.py): Event-driven node verifying bt_navigator/costmap readiness, dispatching goal, and managing `rosbag2_py` lifecycle.
* [`scan_self_filter.py`](trajectory_collector/scan_self_filter.py): Geometric filter masking out Husky sensor arch reflections.
* [`set_initial_pose.py`](trajectory_collector/set_initial_pose.py): Publishes initial pose to AMCL on stack startup.
* [`sim_gate.py`](trajectory_collector/sim_gate.py): Synchronization gate ensuring simulator publishes clock/odometry before starting Nav2.

---

## Testing & Linting

```bash
# Run flake8 linter
ament_flake8 src/trajectory_collector

# Run unit tests
pytest-3 src/trajectory_collector/test/test_navigate_to_goal.py -v

# Run full package test suite
colcon test --packages-select trajectory_collector && colcon test-result --verbose
```
