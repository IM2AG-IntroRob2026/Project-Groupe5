# Project-Groupe5 - Create 3 Autonomous Explorer

ROS 2 package for the **iRobot Create 3** providing autonomous exploration, reactive obstacle avoidance, and a timed mission return-to-dock behavior.

## Behavior

1. **Initial Boot & Undocking**: On startup, if the robot is docked, it automatically calls `Undock`.
2. **Autonomous Exploration**: In `EXPLORING`, the robot uses 7 IR sensors and computes:
   - `max_front = max(readings[2], readings[3], readings[4])`
   - averaged sides (`left_side`, `right_side`) to choose turn direction.
3. **Proportional Obstacle Avoidance**:
   - `AWARE` zone: early veer and light slowdown.
   - `WARNING` zone: proportional braking based on front intensity.
   - `CLOSE` zone: stop forward movement and turn aggressively.
4. **Hazard Recovery (Non-blocking)**: On bumper contact, robot switches to `ESCAPING` for a short timed window (reverse + turn), then resumes its previous state.
5. **Timed Return-to-Dock**: After mission timeout, robot enters `RETURNING` and keeps obstacle avoidance active while trying to dock.
6. **Manual Return Command**: Publishing `DOCK` (or `RETURN`) on `mode` also triggers `RETURNING` with avoidance enabled.
7. **Human Override**: `TELEOP` pauses autonomy and gives manual control; `AUTO` resumes previous autonomous state.

## LED Colors (State Feedback)

| Color | Meaning |
|-------|---------|
| Green | Path clear / nominal autonomous motion |
| Yellow | Early obstacle awareness / gentle veer |
| Orange | Warning zone / braking + stronger turn |
| Red | Critical close obstacle / no forward + aggressive turn |
| Purple | Bumper recovery (`ESCAPING`) |
| Blue | Manual control (`TELEOP`) |

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

### Keyboard Controls

The keyboard node supports:

- `SPACE`: Toggle `AUTO` / `TELEOP`
- `D`: Request return to dock (`DOCK` command)
- `Arrow keys`: Manual driving in `TELEOP`
- `Q`: Quit keyboard handler

## Manual Commands

Request dock/return using the `mode` topic:

```bash
ros2 topic pub /Robot5/mode std_msgs/msg/String "{data: 'DOCK'}" -1
```

Resume autonomous mode:

```bash
ros2 topic pub /Robot5/mode std_msgs/msg/String "{data: 'AUTO'}" -1
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
A state-machine controller with six operational states:

- `DOCKED`
- `UNDOCKING`
- `EXPLORING`
- `ESCAPING`
- `RETURNING`
- `MANUAL`

### Topics & Actions

| Name                  | Type                           | Purpose                                     |
|-----------------------|--------------------------------|---------------------------------------------|
| `ir_intensity`        | `IrIntensityVector`           | Input for wall-following/avoidance         |
| `hazard_detection`    | `HazardDetectionVector`       | Detection of bumpers or cliffs              |
| `dock_status`         | `DockStatus`                  | Tracks if robot is charging or undocked     |
| `mode`                | `std_msgs/String`             | `TELEOP`, `AUTO`, `DOCK`/`RETURN`           |
| `cmd_vel`             | `geometry_msgs/Twist`         | Movement commands (Linear/Angular)          |
| `cmd_lightring`       | `LightringLeds`               | Ring LED state feedback                     |
| `undock` (Action)     | `irobot_create_msgs/Undock`   | Logic to back away from charging base       |
| `dock` (Action)       | `irobot_create_msgs/Dock`     | Autonomous logic to seek and engage charger |

## Key Parameters

| Parameter              | Value   | Description                                           |
|------------------------|---------|-------------------------------------------------------|
| `exploration_time`     | `60.0`  | Duration before automatic return mode                 |
| `linear_speed`         | `0.15`  | m/s - max linear speed                                |
| `angular_speed`        | `0.50`  | rad/s - base turn speed                               |
| `ir_very_early_threshold` | `100` | Early awareness threshold                             |
| `ir_slow_threshold`    | `250`   | Proportional braking start                            |
| `ir_stop_threshold`    | `500`   | Critical close threshold                              |

## Return-to-Dock Safety Logic

- During `RETURNING`, obstacle avoidance remains active (it does not freeze navigation).
- Dock goal is retried periodically (non-blocking) when path is sufficiently clear.
- If repeated red-zone detections happen near corners, a short turn-lock window is applied:
   - no forward speed
   - forced turn for a short duration
   - helps avoid oscillation/stalling behavior near obstacles

# with keyboard handler

sudo apt install xterm

to run robot with parameters
```bash
ros2 launch obstacle_avoidance explorer.launch.py linear_speed:=0.25 exploration_time:=120.0
```

Example with IR thresholds:

```bash
ros2 launch obstacle_avoidance explorer.launch.py namespace:=Robot5 \
   linear_speed:=0.12 angular_speed:=0.65 exploration_time:=120.0 \
   ir_very_early_threshold:=100 ir_early_threshold:=150 \
   ir_slow_threshold:=250 ir_stop_threshold:=500
```
