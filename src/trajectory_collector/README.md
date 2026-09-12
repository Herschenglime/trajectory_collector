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

---

### 3. Simulation Bringup Only (`bringup.launch.py`)

Brings up Gazebo simulation, Nav2 stack, AMCL, and RViz without sending an automated goal (useful for interactive manual navigation in RViz):

```bash
ros2 launch trajectory_collector bringup.launch.py
```

---

### 4. Offline Waypoint Generation (`generate_waypoints`)

Generate topologically reachable, obstacle-cleared `(start, goal)` waypoint pairs with configurable outputs:

```bash
# Generate 10 waypoints on the default warehouse map with a visual preview
ros2 run trajectory_collector generate_waypoints \
  --map warehouse \
  -n 10 \
  -o ~/husky_ws/data/waypoints.csv \
  --preview ~/husky_ws/data/waypoints_preview.png
```

#### CLI Options

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--map`, `-m` | `warehouse` | Map preset name (`warehouse`) or path to custom `map.yaml` |
| `-n`, `--num-samples` | `10` | Number of distinct waypoint pairs to generate |
| `-o`, `--output` | `data/waypoints.csv` | Output CSV file path |
| `--preview` | `""` | Filepath to save PNG map overlay preview |
| `--clearance` | `0.65` | Robot safety clearance radius in meters (Husky A200) |
| `--min-dist` / `--max-dist` | `4.0` / `25.0` | Minimum and maximum straight-line route distances (meters) |
| `--similarity-thresh` | `2.0` | Minimum separation distance between distinct routes |
| `--seed` | `None` | Random seed for deterministic reproducibility |
| `--random-yaw` | `false` | Randomize headings instead of aligning directly toward goal |

---

### 5. Automated Batch Trajectory Sweep (`run_sweep`)

Execute multiple trajectory runs sequentially in cold-restart isolation, using native ROS 2 `LaunchService` process supervision:

```bash
# Run all waypoints in data/waypoints.csv
ros2 run trajectory_collector run_sweep -w data/waypoints.csv -o data/trajectories

# Run a subset of 3 trajectories starting at trajectory ID 2
ros2 run trajectory_collector run_sweep -w data/waypoints.csv -o data/trajectories -n 3 --start-id 2

# Resume into an existing non-empty directory
ros2 run trajectory_collector run_sweep -w data/waypoints.csv -o data/trajectories --overwrite --start-id 5
```

#### Key Characteristics
* **Zero State Leakage**: Each trajectory spins up a fresh simulation and terminates cleanly via `auto_shutdown:=true`.
* **Supervised Lifecycle**: Process lifecycle managed by `launch.LaunchService` with clean child process termination and signal handling.
* **Append-Only Summary**: Flushes run metrics immediately to `sweep_summary.csv` (`id,status,exit_code,duration_s,distance,start_x,start_y,goal_x,goal_y,bag_path`).
* **Pre-flight Safety**: Halts immediately if destination directory already exists and contains data, unless `--overwrite` is specified.
* **Fixed 5-Minute Safety Ceiling**: Default 300-second timeout halts stubborn planning loops while allowing nominal runs to finish and exit immediately.

#### CLI Options

| Flag | Default | Description |
| :--- | :--- | :--- |
| `-w`, `--waypoints` | `data/waypoints.csv` | Path to waypoints CSV file |
| `-o`, `--output-dir`| `data/trajectories` | Directory for output rosbags and `sweep_summary.csv` |
| `--overwrite` | `false` | Allow writing into an existing non-empty destination directory |
| `-n`, `--count` | `None` (all) | Maximum number of trajectories to run |
| `--start-id` | `0` | Trajectory ID to start from (useful for resuming) |
| `--timeout` | `300.0` | Maximum seconds allowed per trajectory (5 minutes) |
| `--cooldown` | `3.0` | Pause seconds between consecutive runs |

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
* [`generate_waypoints.py`](trajectory_collector/generate_waypoints.py): Offline planner sampling reachable, clearance-verified waypoint pairs.
* [`run_sweep.py`](trajectory_collector/run_sweep.py): Batch orchestrator executing sequential cold restarts supervised by `launch.LaunchService`.
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
