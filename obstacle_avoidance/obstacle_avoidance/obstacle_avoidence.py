#!/usr/bin/env python3
"""
create3_controller.py
Autonomous exploration node for the iRobot Create3.

Behavior:
  - Listens for 'mode' from keyboard_handler to start/interrupt missions.
  - State Machine: IDLE -> UNDOCKING -> EXPLORING -> RETURNING -> DOCKED.
  - Obstacle Avoidance: Uses 7 IR sensors to proactively steer away from walls.
  - Hazard Recovery: Backs up immediately if a bumper ('bump') is triggered.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector, IrIntensityVector, LightringLeds, LedColor
from irobot_create_msgs.action import Undock, DockServicing
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RobotState:
    """Enumeration of robot states for the mission lifecycle."""
    IDLE = "IDLE"           # Initial state, waiting on dock
    UNDOCKING = "UNDOCKING" # Executing undock action
    EXPLORING = "EXPLORING" # Autonomous roaming + avoidance
    RETURNING = "RETURNING" # Time expired, executing docking action
    MANUAL = "MANUAL"       # Human override via teleop

IR_STOP_THRESHOLD = 800    # Stop forward motion and rotate (danger zone)
IR_SLOW_THRESHOLD = 150    # Start slowing down (warning zone)

class Explorer(Node):
    def __init__(self):
        super().__init__('create3_controller')

        # --- 1. ROS 2 Parameters ---
        # Allows tuning speeds via 'ros2 launch' or 'ros2 run'
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('exploration_time', 60.0)
        self.declare_parameter('ir_threshold', 50) # IR sensitivity (higher = closer)

        self.LINEAR_SPD = self.get_parameter('linear_speed').value
        self.ANGULAR_SPD = self.get_parameter('angular_speed').value
        self.EXPLORATION_TIME_SEC = self.get_parameter('exploration_time').value
        self.IR_THRESHOLD = self.get_parameter('ir_threshold').value

        # --- 2. State Management ---
        self.state = RobotState.IDLE
        self.saved_state = RobotState.EXPLORING # To resume after Teleop
        self.mission_start_time = None

        # --- 3. Communication Setup ---
        # Use BEST_EFFORT for real-time sensor data over Wi-Fi
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.current_led_color = None  # Track color to avoid redundant publishing
        self.light_pub = self.create_publisher(LightringLeds, 'cmd_light_ring', 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.mode_sub = self.create_subscription(
            String, 'mode', self.mode_callback, 10)
        
        self.ir_sub = self.create_subscription(
            IrIntensityVector, 'ir_intensity', self.ir_sensor_logic, sensor_qos)
        
        self.hazard_sub = self.create_subscription(
            HazardDetectionVector, 'hazard_detection', self.hazard_callback, sensor_qos)
        
        self.dock_sub = self.create_subscription(
            DockStatus, 'dock_status', self.dock_status_callback, sensor_qos)

        # Action Clients for complex behaviors
        self.undock_action = ActionClient(self, Undock, 'undock')
        self.dock_action = ActionClient(self, DockServicing, 'dock')

        self.get_logger().info(f">> Initialized with Linear: {self.LINEAR_SPD}, Angular: {self.ANGULAR_SPD}")

    def mode_callback(self, msg):
        """Toggles between Manual and Auto modes based on Keyboard Handler."""
        if msg.data == 'TELEOP' and self.state != RobotState.MANUAL:
            self.get_logger().warn(">> MANUAL OVERRIDE: Human taking control")
            self.saved_state = self.state
            self.state = RobotState.MANUAL
            self.stop_robot()
        elif msg.data == 'AUTO':
            if self.state == RobotState.IDLE:
                self.start_undocking()
            elif self.state == RobotState.MANUAL:
                self.get_logger().info(">> RESUMING AUTO MODE")
                self.state = self.saved_state

    def hazard_callback(self, msg):
        """Reactive safety: Back up if a bumper is triggered."""
        if self.state != RobotState.EXPLORING:
            return
        
        # Check if any detection is a bumper bump
        if any(d.header.frame_id.startswith('bump') for d in msg.detections):
            self.get_logger().warn("Bumper triggered! Backing up...")
            back_cmd = Twist()
            back_cmd.linear.x = -0.1
            self.cmd_pub.publish(back_cmd)

    def ir_sensor_logic(self, msg):
        if self.state != RobotState.EXPLORING:
            return

        # Start timer check
        if self.mission_start_time is None:
            self.mission_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        if elapsed > self.EXPLORATION_TIME:
            self.start_docking_mission()
            return

        # Map all 7 sensors
        # 0: Far Left, 1: Left, 2: Front-Left, 3: Center, 4: Front-Right, 5: Right, 6: Far Right
        readings = [r.value for r in msg.readings]
        
        far_left   = readings[0]
        left       = readings[1]
        front_left = readings[2]
        center     = readings[3]
        front_right= readings[4]
        right      = readings[5]
        far_right  = readings[6]

        # Combine front-facing sensors for the "Stop" logic
        # If any of the three front sensors see a wall, we need to stop
        front_intensity = max(front_left, center, front_right)
        
        # Side intensities for veering
        left_side_avg = (far_left + left + front_left) / 3
        right_side_avg = (far_right + right + front_right) / 3

        twist = Twist()

        # 1. EMERGENCY STOP & TURN (Object is very close)
        if front_intensity > IR_STOP_THRESHOLD:
            self.set_leds(255, 0, 0) # Orange
            self.get_logger().info(">> Object Detected! Stopping to turn...")
            twist.linear.x = 0.0
            # Turn away from the highest intensity
            twist.angular.z = self.ANGULAR_SPD if left_side_avg < right_side_avg else -self.ANGULAR_SPD

        # 2. PROPORTIONAL SLOWDOWN (Object is visible but at a distance)
        elif front_intensity > IR_SLOW_THRESHOLD:
            self.set_leds(255, 80, 0) # Orange
            # Calculate a speed multiplier (1.0 at SLOW_THRESHOLD, 0.0 at STOP_THRESHOLD)
            # This makes the robot crawl as it gets closer
            range_span = IR_STOP_THRESHOLD - IR_SLOW_THRESHOLD
            proximity_factor = (front_intensity - IR_SLOW_THRESHOLD) / range_span
            speed_multiplier = max(0.0, 1.0 - proximity_factor)
            
            twist.linear.x = self.LINEAR_SPD * speed_multiplier
            
            # Gentle veering while approaching
            if left_side_avg > right_side_avg:
                twist.angular.z = -0.3 # Veer right
            else:
                twist.angular.z = 0.3  # Veer left
                
        # 3. SIDE AVOIDANCE (Wall to the side, path ahead is clear)
        elif left_side_avg > IR_SLOW_THRESHOLD:
            self.set_leds(255, 80, 0) # Orange
            twist.linear.x = self.LINEAR_SPD
            twist.angular.z = -0.4 # Veer right away from left wall
        elif right_side_avg > IR_SLOW_THRESHOLD:
            self.set_leds(255, 80, 0) # Orange
            twist.linear.x = self.LINEAR_SPD
            twist.angular.z = 0.4  # Veer left away from right wall

        # 4. CLEAR PATH
        else:
            self.set_leds(0, 255, 0) # Green
            twist.linear.x = self.LINEAR_SPD
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def ir_logic(self, msg):
        """
        Proactive avoidance: 
        Uses 7 IR sensors to detect walls before hitting them.
        Left sensors: readings[0-2], Center: [3], Right: [4-6].
        """
        if self.state != RobotState.EXPLORING:
            return

        # Initialize mission clock on first exploration tick
        if self.mission_start_time is None:
            self.mission_start_time = self.get_clock().now()

        # Check mission duration
        elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        if elapsed > self.EXPLORATION_TIME_SEC:
            self.start_docking_sequence()
            return

        # Extract max values from side zones for sensitivity
        readings = [r.value for r in msg.readings]
        left_val = max(readings[0:3])
        center_val = readings[3]
        right_val = max(readings[4:7])

        twist = Twist()
        
        # Avoidance Logic: Prioritize turning if path is blocked
        if center_val > self.IR_THRESHOLD or left_val > self.IR_THRESHOLD or right_val > self.IR_THRESHOLD:
            twist.linear.x = 0.02 # Slow crawl during turn
            if left_val > right_val:
                twist.angular.z = -self.ANGULAR_SPD # Turn Right
            else:
                twist.angular.z = self.ANGULAR_SPD  # Turn Left
        else:
            # Path clear: Go forward straight
            twist.linear.x = self.LINEAR_SPD
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def start_undocking(self):
        """Initiates the Undock Action."""
        self.state = RobotState.UNDOCKING
        self.undock_action.wait_for_server()
        self.undock_action.send_goal_async(Undock.Goal()).add_done_callback(self.undock_done)

    def undock_done(self, future):
        """Callback when Undock Action finishes."""
        self.get_logger().info(">> Undock Successful. Starting Exploration.")
        self.state = RobotState.EXPLORING
        self.mission_start_time = self.get_clock().now()

    def start_docking_sequence(self):
        """Initiates the Return to Dock Action."""
        self.get_logger().info(">> Mission complete. Returning to dock...")
        self.state = RobotState.RETURNING
        self.stop_robot()
        self.dock_action.wait_for_server()
        self.dock_action.send_goal_async(DockServicing.Goal())

    def dock_status_callback(self, msg):
        """Monitors if robot is physically docked."""
        if msg.is_docked and self.state == RobotState.RETURNING:
            self.get_logger().info(">> Robot is Safely Docked. State -> IDLE.")
            self.state = RobotState.IDLE

    def stop_robot(self):
        """Helper to send zero velocity."""
        self.cmd_pub.publish(Twist())

    def set_leds(self, r, g, b):
        # Prevent redundant publishing to save bandwidth
        if self.current_led_color == (r, g, b):
            return
            
        msg = LightringLeds()
        msg.override_system = True
        
        for _ in range(6): # The Create 3 has 6 LEDs in its ring
            color = LedColor()
            color.red, color.green, color.blue = r, g, b
            msg.leds.append(color)
            
        self.light_pub.publish(msg)
        self.current_led_color = (r, g, b)
def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()