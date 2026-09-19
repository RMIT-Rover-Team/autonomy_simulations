# Autonomy Simulation Foundation

A ROS 2 (Jazzy) workspace that simulates a differential-drive robot in Gazebo and builds up the
core autonomy stack.

This is **simulation #1**: the foundation and minimum viable functionality,
with a basic sensor suite (Wheel encoders + IMU + 2D LiDAR).

---

## Demo

### SLAM
![media/slam-mapping.png](https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/slam.png)

https://github.com/user-attachments/assets/e8d3cc5e-d6db-4d4d-83b4-49e1f233919f


### AMCL

![media/amcl.png](https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/amcl.png)

https://github.com/user-attachments/assets/f0261bf2-e251-41dd-a8fe-07eb80e6cc39


### SLAM_NAV

![media/slam_nav.png](https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/slam_nav.png)

https://github.com/user-attachments/assets/f8eb0987-f70f-4091-a2c8-d2b7a98af1b1


### AMCL_NAV

![media/amcl_nav.png](https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/amcl_nav.png)

https://github.com/user-attachments/assets/104773a8-cdc1-4b0e-bd16-49a4653313d7


### SPEED AND SEPARATION MONITORING

<table>
  <tr>
    <td><img src="https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/safety_node1.png" width="300"/></td>
    <td><img src="https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/safety_node2.png" width="300"/></td>
    <td><img src="https://github.com/RMIT-Rover-Team/autonomy_simulations/blob/main/00_sim/media/safety_node3.png" width="300"/></td>
  </tr>
</table>

---

## The robot

Defined in `my_robot_description/urdf/robot.urdf.xacro` (split into `robot_gazebo.xacro` for
sim/sensor plugins and `robot_ros2_control.xacro` for the hardware interface):

- **Base**: a differential-drive chassis (`base_footprint` → `base_link`) with two driven wheels
  (continuous joints) and a passive caster.
- **Sensors** (the "basic sensors" of this first simulation):
  - **2D LiDAR** (`base_scan` link) - Gazebo `gpu_lidar` sensor, 360 samples over a full circle,
    0.22–12 m range, published on `/scan` as `sensor_msgs/LaserScan`.
  - **IMU** (`imu_link`) - Gazebo `imu` sensor with Gaussian noise on angular velocity and linear
    acceleration, bridged to ROS on `/imu/out` as `sensor_msgs/Imu`.
- **Actuation**: `ros2_control` `GazeboSimSystem` hardware interface exposes velocity command /
  position+velocity state interfaces on the two wheel joints, driven by a
  `diff_drive_controller/DiffDriveController`.

---

## Packages overview

| Package | Role |
|---|---|
| [`my_robot_description`](#my_robot_description) | Robot model (URDF/xacro), Gazebo worlds, spawns the robot in sim |
| [`my_robot_controller`](#my_robot_controller) | Diff-drive control, joystick teleop, command-velocity arbitration |
| [`my_robot_localization`](#my_robot_localization) | Sensor fusion (EKF) + map-based localization (AMCL) |
| [`my_robot_mapping`](#my_robot_mapping) | SLAM (build a map) and saved maps |
| [`my_robot_navigation`](#my_robot_navigation) | Nav2 stack: costmaps, planner/controller/BT servers |
| [`my_robot_planning`](#my_robot_planning) | Custom Nav2 global-planner plugins (Dijkstra, A*) |
| [`my_robot_motion`](#my_robot_motion) | Custom Nav2 controller plugins (PD controller, Pure Pursuit) |
| [`my_robot_utils`](#my_robot_utils) | Safety-stop node (LiDAR-based e-stop / slow-down) |
| [`my_robot_bringup`](#my_robot_bringup) | Top-level launch file that wires everything above together |

---

## Package details

### `my_robot_description`
Robot model, simulation worlds, and static visualization.
- `urdf/robot.urdf.xacro` - robot geometry, inertials, joints (includes the two files below).
- `urdf/robot_gazebo.xacro` - Gazebo material/friction tags, the `gz_ros2_control` plugin, and the
  LiDAR + IMU sensor definitions.
- `urdf/robot_ros2_control.xacro` - `ros2_control` hardware interface (velocity command,
  position/velocity state, on both wheel joints).
- `worlds/` - `empty.world`, `room1.world`, `test_zone.world`, `mars_world.world`.
- `models/` - Gazebo models used by those worlds (`cafe_table`, `demo_cube`, `test_zone` building,
  `mars_terrain`).
- `launch/gazebo.launch.py` - starts Gazebo with a chosen `world_name`, spawns the robot from
  `robot_description`, and bridges `/clock`, `/imu`, `/scan` via `ros_gz_bridge`.
- `launch/display.launch.py` - RViz-only visualization with `joint_state_publisher_gui` (no
  Gazebo), useful for checking the model in isolation.

### `my_robot_controller`
Turns velocity commands into wheel motion, and turns joystick input into velocity commands.
- `launch/controller.launch.py` - spawns `joint_state_broadcaster` and the diff-drive
  `robot_controller` via `controller_manager`.
- `launch/joystick_teleop.launch.py` - `joy_node` (raw joystick) → `joy_teleop` (axis mapping,
  deadman button) → `twist_relay.py` (bridges stamped/unstamped `Twist` messages) →
  `twist_mux` (priority arbitration between joystick/keyboard/navigation `cmd_vel` sources, with
  a `/safety_stop`-driven lock).
- `config/` - `robot_controllers.yaml` (diff-drive controller limits/covariances),
  `joy_config.yaml` / `joy_teleop.yaml` (joystick mapping), `twist_mux_*.yaml` (topic priorities,
  the safety lock, and joystick turbo speed steps).
- `my_robot_controller/twist_relay.py` - small relay node converting between `Twist` and
  `TwistStamped` so `joy_teleop` and `twist_mux`/`diff_drive_controller` can talk to each other.

### `my_robot_localization`
Where the robot fuses sensors to know where it is.
- `launch/local_localization.launch.py` - always-on EKF (`robot_localization`'s `ekf_node`)
  fusing `/robot_controller/odom` (wheel odometry) and `/imu/out` (yaw + angular velocity) into
  the `odom → base_footprint` transform (`config/ekf.yaml`).
- `launch/global_localization.launch.py` - loads a saved map (`map_name` arg, default `room1`)
  via `nav2_map_server` and runs `nav2_amcl` against it (`config/amcl.yaml`), publishing
  `map → odom`; opens `rviz/nav2_amcl.rviz` to watch the particle cloud converge.

### `my_robot_mapping`
Where the robot builds a map when one doesn't exist yet.
- `launch/slam.launch.py` - `slam_toolbox` (sync SLAM) + `nav2_map_server`'s `map_saver_server`,
  managed by a Nav2 lifecycle manager; opens `rviz/slam.rviz` to watch the occupancy grid fill in
  live as you teleop the robot around.
- `config/slam_toolbox.yaml` - SLAM Toolbox tuning.
- `maps/` - saved maps (`room1`, `test_zone`), each a `map.yaml` (metadata: resolution, origin,
  occupancy thresholds) + `map.pgm` (the occupancy grid image), produced by a prior SLAM session.

### `my_robot_navigation`
The Nav2 "brain": turns a goal pose into wheel motion.
- `launch/navigation.launch.py` - brings up `controller_server`, `planner_server`,
  `smoother_server`, `bt_navigator`, `behavior_server`, all under one lifecycle manager.
- `config/costmap.yaml` - local costmap (rolling, `odom` frame, obstacle + inflation layers from
  `/scan`) and global costmap (`map` frame, static + obstacle + inflation layers).
- `config/planner_server.yaml` - registers three interchangeable global planners: Nav2's stock
  `SmacPlanner2D` (`GridBased`) plus the workspace's own `DijkstraPlannerPlugin` and
  `AStarPlannerPlugin` (`Custom1`/`Custom2`, from `my_robot_planning`).
- `config/controller_server.yaml` - registers three interchangeable local controllers: Nav2's
  stock `RegulatedPurePursuitController` (`FollowPath`) plus the workspace's own PD controller
  and Pure Pursuit implementation (`Custom1`/`Custom2`, from `my_robot_motion`).
- `config/smoother_server.yaml`, `config/behavior_server.yaml` - path smoothing and recovery
  behaviors (spin, back up, wait).
- `behavior_tree/` - three BehaviorTree.CPP XML trees of increasing sophistication (basic →
  with replanning → with replanning + recovery). Which one `bt_navigator` uses is set at launch
  time by the `default_bt_xml_filename` argument, defaulting to the fullest one.

### `my_robot_planning`
Custom **global planner** plugins, built as a `nav2_core::GlobalPlanner` plugin library
(`global_planner_plugins.xml`) and loaded by `planner_server` - not run as standalone nodes:
- `DijkstraPlannerPlugin` - uniform-cost graph search over the costmap.
- `AStarPlannerPlugin` - heuristic-guided search over the costmap.

### `my_robot_motion`
Custom **local controller** plugins, built as a `nav2_core::Controller` plugin library
(`motion_planner_plugins.xml`) and loaded by `controller_server` - not run as standalone nodes:
- `PDMotionPlannerPlugin` - proportional-derivative path-tracking controller.
- `PurePursuitPlugin` - geometric pure-pursuit path-tracking controller.

### `my_robot_utils`
Cross-cutting safety layer, independent of the navigation stack.
- `my_robot_utils/safety_stop.py` - subscribes to `/scan`; if any return falls inside a
  configurable *danger radius* it asserts `/safety_stop` (a hard lock picked up by `twist_mux`,
  overriding every other `cmd_vel` source), and inside a wider *warning radius* it calls
  `twist_mux`'s joystick-turbo action to slow the robot down. Also publishes `MarkerArray` zone
  visualizations (`rviz/safety_stop_config.rviz`) for RViz.

### `my_robot_bringup`
No nodes of its own - just the top-level launch file that ties the packages above together for a
day-to-day sim session.
- `launch/simulated_robot.launch.py` - starts Gazebo + the robot, the controller stack, joystick
  teleop, and the EKF unconditionally; a single `mode` launch argument (`slam` | `amcl` |
  `slam_nav` | `amcl_nav`, default `slam`) then picks exactly one localization source (SLAM XOR
  AMCL - they can't both own the `map` frame) and whether navigation + a second RViz window
  (`nav2_default_view.rviz`) are layered on top. A `default_bt_xml_filename` argument is forwarded through to
  `navigation.launch.py` for picking which behavior tree `bt_navigator` runs. The safety-stop
  node is present in the file but currently commented out.

---

## How the packages connect

```
                          ┌─────────────────────┐
                          │  my_robot_bringup    │  (top-level orchestrator)
                          │ simulated_robot.     │
                          │ launch.py            │
                          │  mode:=slam|amcl|    │
                          │  slam_nav|amcl_nav   │
                          └─────────┬────────────┘
                                    │ includes
        ┌────────────┬─────────────┼─────────────┬───────────────────┐
        ▼            ▼             ▼             ▼                   ▼
 my_robot_        my_robot_    my_robot_     my_robot_           my_robot_
 description      controller   controller    localization        mapping or
 gazebo.launch    controller.  joystick_     local_localization  localization
 .py              launch.py    teleop.       .launch.py (EKF,    (SLAM XOR AMCL,
 (spawns robot                 launch.py     always on)          picked by mode)
 + world in Gz,
 bridges /scan,                                                        │
 /imu, /clock)                                                         ▼
                                                                 my_robot_navigation
                                                                 navigation.launch.py
                                                                 (only when mode is
                                                                 *_nav) + a second
                                                                 rviz2 (nav2_default_
                                                                 view.rviz)
```

---

**Data flow once everything is running:**

1. **`my_robot_description`** spawns the robot into a Gazebo world (`room1` by default) and
   bridges `/clock`, `/imu`, `/scan` from Gazebo into ROS 2 topics; `robot_state_publisher`
   publishes the static/URDF TF tree (`base_footprint → base_link → wheels/lidar/imu`).
2. **`my_robot_controller`** starts `ros2_control`'s `joint_state_broadcaster` and
   `diff_drive_controller` (publishes wheel odometry on `/robot_controller/odom` and the
   `odom → base_footprint` TF *only if* `enable_odom_tf` were true - here it's left to the EKF).
   Joystick input flows: `joy_node` → `joy_teleop` → `twist_relay` (stamps/un-stamps `Twist` ↔
   `TwistStamped`) → `twist_mux` (arbitrates joystick vs. nav vs. keyboard `cmd_vel`, and can be
   locked by `/safety_stop`) → `diff_drive_controller`.
3. **`my_robot_localization`** always runs an EKF (`local_localization.launch.py`) fusing wheel
   odometry (`/robot_controller/odom`) with IMU yaw/angular-velocity to produce a smoothed
   `odom → base_footprint` transform. On top of that, `simulated_robot.launch.py`'s single `mode`
   argument picks exactly **one** localization source - SLAM and AMCL both want to own the
   `map → odom` transform, so only one is ever active:
   - `mode:=slam` (or `slam_nav`) → **`my_robot_mapping`**'s SLAM stack (`slam_toolbox` +
     `map_saver_server`), which owns `map → odom` while you drive around and build a map live.
   - `mode:=amcl` (or `amcl_nav`) → **`my_robot_localization`**'s global localization
     (`nav2_map_server` loading a saved map + `nav2_amcl`), which owns `map → odom` by
     localizing against that saved map.
4. **`my_robot_navigation`** is brought up in the same launch whenever `mode` is `slam_nav` or
   `amcl_nav` - the Nav2 stack (`controller_server`, `planner_server`, `smoother_server`,
   `bt_navigator`, `behavior_server`) consumes the `map → odom → base_footprint` TF chain and
   `/scan`-fed costmaps from whichever localization source is active, and drives the robot toward
   goals by publishing `cmd_vel`, which feeds back into `twist_mux` from step 2. A second RViz
   window (`nav2_default_view.rviz`) opens alongside it for sending goals and watching costmaps.
5. **`my_robot_utils`**' `safety_stop` node watches `/scan` directly and independently pulls the
   `twist_mux` e-stop lock (`/safety_stop`) and joystick-turbo speed up/down whenever an obstacle
   enters a warning/danger radius - a safety layer that sits outside the planning stack. (Currently
   wired into `simulated_robot.launch.py` but commented out)
6. **`my_robot_planning`** and **`my_robot_motion`** are Nav2 plugin *libraries*, not standalone
   nodes: they're `pluginlib`-loaded directly into `planner_server` and `controller_server`
   (see `my_robot_navigation/config/*.yaml`) as selectable alternatives alongside Nav2's stock
   Smac/RegulatedPurePursuit algorithms.

---

## Running it: the mode argument

Everything - description, control, localization, and (optionally) navigation - comes up from one
launch file, gated by a single `mode` argument:

| `mode` | Runs | Use when |
|---|---|---|
| `slam` (default) | SLAM (`slam_toolbox` + `map_saver_server`) | No map of the space exists yet, or the space changed enough that the old map no longer matches reality. |
| `amcl` | Global localization (`nav2_map_server` + `nav2_amcl`) against a saved map | A map already exists and you just want the robot to know where it is on it. |
| `slam_nav` | SLAM + Nav2 + `nav2_default_view.rviz` | You want to navigate goals while simultaneously building the map (exploration-style; the map - and therefore the costmap - can still change under you, including sudden jumps on loop closure). |
| `amcl_nav` | AMCL + Nav2 + `nav2_default_view.rviz` | The normal "drive the finished robot" case: a map exists, localization is solid, and you want to send goals. |

```bash
# No map yet - build one
ros2 launch my_robot_bringup simulated_robot.launch.py
# (same as mode:=slam)

# Map exists - just localize (against the default map, "room1")
ros2 launch my_robot_bringup simulated_robot.launch.py mode:=amcl

# Map exists - localize AND navigate
ros2 launch my_robot_bringup simulated_robot.launch.py mode:=amcl_nav

# No map yet, but you want to navigate while exploring
ros2 launch my_robot_bringup simulated_robot.launch.py mode:=slam_nav
```

> `simulated_robot.launch.py` doesn't currently expose the world/map choice as its own argument -
> it always uses `gazebo.launch.py`'s default world (`room1`) and `global_localization.launch.py`'s
> default map (also `room1`, so the two line up out of the box). To use a different world/map pair
> (e.g. `test_zone`), edit those two defaults.

In the `_nav` modes, which behavior tree `bt_navigator` runs is controlled by
`default_bt_xml_filename`, - swap it for one of the other two trees in `behavior_tree/` without editing any
file:
```bash
ros2 launch my_robot_bringup simulated_robot.launch.py mode:=amcl_nav \
  default_bt_xml_filename:=$(ros2 pkg prefix my_robot_navigation)/share/my_robot_navigation/behavior_tree/simple_navigation.xml
```

`mode:=slam`/`slam_nav` drive the robot around with the joystick while `slam_toolbox` builds the
occupancy grid live (watch it fill in on `slam.rviz`'s Map/LaserScan/TF displays). Once the map
looks clean, save it:
```bash
ros2 run nav2_map_server map_saver_cli -f <path>/my_robot_mapping/maps/<name>/map
```
(or trigger `map_saver_server`'s save action directly). `room1` and `test_zone` under
`my_robot_mapping/maps/` were both produced this way.

`mode:=amcl`/`amcl_nav` load the saved map and localize against it - watch the particle cloud
converge in `nav2_amcl.rviz`. In the `_nav` modes, `bt_navigator`/costmaps then have the stable
`map → odom → base_footprint` TF chain they need, `nav2_default_view.rviz` opens for sending goal
poses, and `bt_navigator` plans (via `GridBased`/`Custom1`/`Custom2`) and drives (via
`FollowPath`/`Custom1`/`Custom2`) toward them.

```
New space?             → mode:=slam       → drive around → save the map → done with SLAM
Have a map, no nav?    → mode:=amcl       → confirm AMCL converges (nav2_amcl.rviz)
Have a map, want to go?→ mode:=amcl_nav   → send goals in nav2_default_view.rviz
Explore while mapping? → mode:=slam_nav   → send goals into the still-growing map (advanced/riskier)
```

---

## Building the workspace

```bash
cd autonomy_sim_environment
cd 00_sim # being in this directory is important for the commands below
source /opt/ros/jazzy/setup.bash
rosdep update && sudo rosdep install -i --from-path src --rosdistro jazzy -y && colcon build
source ./install/setup.bash
ros2 launch my_robot_bringup simulated_robot.launch.py mode:=slam   # or amcl / slam_nav / amcl_nav
```

