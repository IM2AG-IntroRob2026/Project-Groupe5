# Project-Groupe5 - Create 3 Autonomous Explorer

ROS 2 package for the **iRobot Create 3** providing autonomous exploration, reactive obstacle avoidance, and a timed mission return-to-dock behavior.

## Behavior

1. **Initial Boot & Undocking**: Upon startup, if the robot detects it is on the charging station, it automatically invokes the `Undock` action and moves away from the base.
2. **Autonomous Exploration**: Once undocked, the robot enters the `EXPLORING` state. It wanders forward while monitoring its 7 Infrared (IR) intensity sensors.
3. **Reactive Avoidance**:
   - If an obstacle is detected in the **center**, the robot rotates away from the side with higher intensity.
   - If an obstacle is detected on the **left/right sides**, it veers in the opposite direction to maintain a safe distance.
4. **Hazard Recovery**: If a physical collision is detected by the bumpers, the robot immediately executes a safety routine: it backs up for 1 second and performs an evasive rotation before resuming exploration.
5. **Mission Timer & Return**: After **60 seconds** of autonomous activity, the robot stops exploration and invokes the `Dock` action to return to its charger.
6. **Human Interrupt**: At any moment, publishing a `'MANUAL'` string to the `/mode` topic pauses autonomy and grants manual control. Transitioning back to `'AUTO'` resumes the mission timer and behavioral logic.

## Requirements - Implementation:
The project follows a **Modular State Machine** architecture within a single ROS 2 Python node (`explorer`).
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

To launch the autonomous behavior using a specific robot namespace (e.g., `Robot5`):

```bash
ros2 launch obstacle_avoidance explorer.launch.py namespace:=Robot5
```

If you prefer to run the executable directly, you can still use:

```bash
ros2 run obstacle_avoidance explorer --ros-args -r __ns:=/Robot5
```

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
A state-machine based controller that manages the robot's mission lifecycle through five operational states: `DOCKED`, `UNDOCKING`, `EXPLORING`, `RETURNING`, and `MANUAL`.

### Topics & Actions

| Name                  | Type                           | Purpose                                     |
|-----------------------|--------------------------------|---------------------------------------------|
| `ir_intensity`        | `IrIntensityVector`           | Input for wall-following/avoidance         |
| `hazard_detection`    | `HazardDetectionVector`       | Detection of bumpers or cliffs              |
| `dock_status`         | `DockStatus`                  | Tracks if robot is charging or undocked     |
| `mode`                | `std_msgs/String`              | Manual (`MANUAL`) or Autonomous (`AUTO`)    |
| `cmd_vel`             | `geometry_msgs/Twist`         | Movement commands (Linear/Angular)          |
| `undock` (Action)     | `irobot_create_msgs/Undock`   | Logic to back away from charging base       |
| `dock` (Action)       | `irobot_create_msgs/Dock`     | Autonomous logic to seek and engage charger |

## Safety Parameters

| Parameter              | Value   | Description                                           |
|------------------------|---------|-------------------------------------------------------|
| `EXPLORATION_TIME_SEC` | `60.0`  | Duration of the mission before returning to dock      |
| `IR_THRESHOLD`         | `150`   | Detection sensitivity (Higher = closer to walls)      |
| `LINEAR_SPD`           | `0.15`  | m/s - Maximum forward exploration speed               |
| `ANGULAR_SPD`          | `0.45`  | rad/s - Rotational speed for obstacle avoidance turns |

# with keyboard handler 

sudo apt install xterm

to run robot with parameters 
```bash
ros2 launch obstacle_avoidance explorer.launch.py linear_speed:=0.25 exploration_time:=120.0
```