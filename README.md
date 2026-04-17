# Project-Groupe5 - Create 3 Autonomous Explorer

ROS 2 package for the **iRobot Create 3** providing autonomous exploration, reactive obstacle avoidance, manual teleoperation override, and a timed mission return-to-dock behavior.

## Behavior

1. **Initial Boot & Undocking**: Upon startup, if the robot detects it is on the charging station, it automatically invokes the `Undock` action and moves away from the base.
2. **Autonomous Exploration**: Once undocked, the robot enters the `EXPLORING` state. It wanders forward while monitoring its 7 Infrared (IR) intensity sensors.
3. **Reactive Avoidance**:
   - If an obstacle is detected in the **center**, the robot rotates away from the side with higher intensity.
   - If an obstacle is detected on the **left/right sides**, it veers in the opposite direction to maintain a safe distance.
4. **Hazard Recovery**: If a physical collision is detected by the bumpers, the robot immediately executes a safety routine: it backs up for 1 second and performs an evasive rotation before resuming exploration.
5. **Mission Timer & Return**: After **60 seconds** of autonomous activity, the robot stops exploration and invokes the `Dock` action to return to its charger.
6. **Human Interrupt**: At any moment, publishing `'TELEOP'` to the `/mode` topic pauses autonomous motion decisions and gives manual control through the teleop node. Publishing `'AUTO'` returns control to the Explorer node.

## Requirements - Implementation:
The project follows a **modular two-node architecture**:
1. `explorer`: autonomous state machine (dock/undock, obstacle avoidance, mission flow).
2. `keyboard_handler`: terminal teleop interface and mode switching publisher.

*   **Framework**: Python 3 / ROS 2 Jazzy.
*   **Middleware**: Optimized for Hardware via `BEST_EFFORT` QoS on IR and Hazard topics.
*   **Persistence**: Utilizes a `saved_state` buffer to ensure tasks resume correctly after a manual pause.

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-irobot-create-msgs
```

## Build & Install

**1. Clone the repository:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/IM2AG-IntroRob2026/Project-Groupe5.git
```

**2. Build the package:**
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select obstacle_avoidance
source install/setup.bash
```

## Run

The normal workflow is:
1. Launch the system in `AUTO`.
2. Let the `keyboard_handler` decide when to switch to `TELEOP`.
3. Press `SPACE` again to return to `AUTO`.

### Option 1: Launch everything with one command

This starts the autonomous Explorer node and opens the keyboard controller in a separate terminal window:

```bash
ros2 launch obstacle_avoidance explorer.launch.py namespace:=Robot5
```

The launch file passes the runtime parameters to Explorer:

```bash
ros2 launch obstacle_avoidance explorer.launch.py namespace:=Robot5 linear_speed:=0.25 angular_speed:=0.45 exploration_time:=120.0
```

### Option 2: Run the nodes manually

Terminal 1 - Explorer in AUTO mode:

```bash
ros2 run obstacle_avoidance explorer --ros-args -r __ns:=/Robot5
```

Terminal 2 - Keyboard handler:

```bash
ros2 run obstacle_avoidance keyboard_handler --ros-args -r __ns:=/Robot5
```

If your branch uses the `teleop` alias, this is also valid:

```bash
ros2 run obstacle_avoidance teleop --ros-args -r __ns:=/Robot5
```

### How control is managed

- Explorer subscribes to `/Robot5/mode`.
- `keyboard_handler` publishes `TELEOP` when the user presses `SPACE`.
- In `TELEOP`, Explorer stops driving the robot and the keyboard node publishes `cmd_vel` directly.
- Press `SPACE` again to publish `AUTO`; Explorer resumes autonomous navigation and obstacle avoidance.

## Launch File

The launch file [launch/explorer.launch.py](obstacle_avoidance/launch/explorer.launch.py) starts the `explorer` node and accepts an optional `namespace` argument.

Example:

```bash
ros2 launch obstacle_avoidance explorer.launch.py namespace:=Robot5
```

## Architecture

```
                 /Robot5/ir_intensity ──► [Explorer Node]
               /Robot5/hazard_detection ──►      │
                      /Robot5/mode ──►           │
                                                 ▼
                                        /Robot5/cmd_vel
                                       /Robot5/dock (Action)
                                      /Robot5/undock (Action)
```

### Node: `explorer` (Python)
A state-machine based controller that manages the robot's mission lifecycle through five operational states: `DOCKED`, `UNDOCKING`, `EXPLORING`, `RETURNING`, and `TELEOP` (manual override mode).

### Node: `keyboard_handler` (Python)
A non-blocking terminal controller that:
- Publishes mode changes on `mode` (`TELEOP` or `AUTO`).
- Publishes manual velocity commands to `cmd_vel` while teleop is active.
- Allows rapid validation of manual override behavior without extra tooling.

### Topics & Actions

| Name                  | Type                           | Purpose                                     |
|-----------------------|--------------------------------|---------------------------------------------|
| `ir_intensity`        | `IrIntensityVector`           | Input for wall-following/avoidance         |
| `hazard_detection`    | `HazardDetectionVector`       | Detection of bumpers or cliffs              |
| `dock_status`         | `DockStatus`                  | Tracks if robot is charging or undocked     |
| `mode`                | `std_msgs/String`              | Manual (`TELEOP`) or Autonomous (`AUTO`)    |
| `cmd_vel`             | `geometry_msgs/Twist`         | Movement commands (Linear/Angular)          |
| `undock` (Action)     | `irobot_create_msgs/Undock`   | Logic to back away from charging base       |
| `dock` (Action)       | `irobot_create_msgs/Dock`     | Autonomous logic to seek and engage charger |

## Manual Override Verification

You can verify the mode switching in two ways:

1. **Raw ROS topic test**
```bash
ros2 topic pub -1 /Robot5/mode std_msgs/msg/String "{data: 'TELEOP'}"
ros2 topic pub -1 /Robot5/mode std_msgs/msg/String "{data: 'AUTO'}"
```

2. **Keyboard handler test**
Run the keyboard node and use `SPACE` to toggle between `AUTO` and `TELEOP`.

## Safety Parameters

| Parameter              | Value   | Description                                           |
|------------------------|---------|-------------------------------------------------------|
| `EXPLORATION_TIME_SEC` | `60.0`  | Duration of the mission before returning to dock      |
| `IR_THRESHOLD`         | `150`   | Detection sensitivity (Higher = closer to walls)      |
| `LINEAR_SPD`           | `0.15`  | m/s - Maximum forward exploration speed               |
| `ANGULAR_SPD`          | `0.45`  | rad/s - Rotational speed for obstacle avoidance turns |

### Keyboard window requirement

The launch file opens the keyboard controller with `xterm`, so install it if needed:

```bash
sudo apt install xterm
```