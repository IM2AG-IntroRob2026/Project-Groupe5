#!/usr/bin/env python3
"""
This module implements an autonomous exploration node for iRobot Create 3 robot.
The robot explores its environment using infrared sensors for obstacle detection,
avoids collisions, and autonomously returns to dock after a specified duration.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector, IrIntensityVector, LightringLeds, LedColor
from irobot_create_msgs.action import Undock, Dock 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

# Mission and movement parameters
EXPLORATION_TIME = 60.0  # Time to wander before returning home
LINEAR_SPD = 0.15  # Forward movement speed m/s
ANGULAR_SPD = 0.45  # Rotation speed rad/s

# These values CONTROL when the robot starts avoiding obstacles
# Lower values = earlier detection = safer but slower
IR_STOP_THRESHOLD = 800      # DANGER: Very close (< 5 cm), hard stop
IR_SLOW_THRESHOLD = 200      # WARNING: Medium distance (~ 10 cm), start slowing
IR_EARLY_THRESHOLD = 100     # CAUTION: Far distance (~ 15 cm), gentle slowdown
IR_VERY_EARLY_THRESHOLD = 60 # DETECTION: Very far (~ 20 cm), start avoiding
"""
    Enumeration of robot operational states during the autonomous exploration mission

    Attributes:
        DOCKED: Robot is on the charging dock (initial state)
        UNDOCKING: Robot is in process of leaving the dock
        EXPLORING: Robot is actively wandering and avoiding obstacles
        RETURNING: Robot has completed mission and is returning to dock
        MANUAL: Human has taken manual control via override
"""
class RobotState:
    DOCKED = "DOCKED"
    UNDOCKING = "UNDOCKING"
    EXPLORING = "EXPLORING"
    RETURNING = "RETURNING"
    MANUAL = "TELEOP"

"""
    Autonomous exploration node for the robot
    
    This node manages the lifecycle of autonomous robot exploration:
    - Monitors dock status and initiates undocking on startup
    - Performs autonomous exploration using IR-based obstacle avoidance
    - Detects and reacts to hazards (bumper collisions)
    - Supports manual override for human control
    - Autonomously returns to dock after mission timer expires
    
    Subscriptions:
        - ir_intensity: IR sensor readings (7 sensors) for obstacle detection
        - hazard_detection: Bumper/cliff detection events
        - dock_status: Docking state information
        - mode: Manual/auto mode override commands
    
    Publishers:
        - cmd_vel: Velocity commands (linear/angular)
    
    Action Clients:
        - undock: Action service for leaving the dock
        - dock: Action service for autonomous docking
"""
class Explorer(Node):
    
    def __init__(self):
        super().__init__('autonomous_explorer')

        # Tunable parameters with defaults that can be overridden from launch 
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('exploration_time', 60.0)
    
        self.declare_parameter('ir_very_early_threshold', 60)
        self.declare_parameter('ir_early_threshold', 100)
        self.declare_parameter('ir_slow_threshold', 200)
        self.declare_parameter('ir_stop_threshold', 800)
 
        self.LINEAR_SPD = self.get_parameter('linear_speed').value
        self.ANGULAR_SPD = self.get_parameter('angular_speed').value
        self.EXPLORATION_TIME = self.get_parameter('exploration_time').value
        
        # Get IR thresholds from parameters
        self.IR_VERY_EARLY_THRESHOLD = self.get_parameter('ir_very_early_threshold').value
        self.IR_EARLY_THRESHOLD = self.get_parameter('ir_early_threshold').value
        self.IR_SLOW_THRESHOLD = self.get_parameter('ir_slow_threshold').value
        self.IR_STOP_THRESHOLD = self.get_parameter('ir_stop_threshold').value

        # QoS Profile for real hardware sensors
        # allowing for best effort and some message loss without blocking the system
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Robot state variables
        self.state = RobotState.DOCKED           # Current operational state
        self.saved_state = RobotState.EXPLORING  # State to resume after manual override
        self.is_docked = None                    # Docking status from dock_status topic
        self.mission_start_time = None           # Timestamp when exploration began

        self.current_led_color = None  # Track color to avoid redundant publishing
        self.light_pub = self.create_publisher(LightringLeds, 'cmd_light_ring', 10)
        self.set_leds(0, 255, 0)
        # Publishers & Subscribers setup
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.ir_sub = self.create_subscription(
            IrIntensityVector, 'ir_intensity', self.ir_logic, sensor_qos)
        self.hazard_sub = self.create_subscription(
            HazardDetectionVector, 'hazard_detection', self.hazard_check, sensor_qos)
        self.dock_sub = self.create_subscription(
            DockStatus, 'dock_status', self.update_dock_status, sensor_qos)
        self.mode_sub = self.create_subscription(
            String, 'mode', self.manual_override_callback, 10)

        # Action Clients for dock/undock operations
        self.undock_action = ActionClient(self, Undock, 'undock')
        self.dock_action = ActionClient(self, Dock, 'dock')

        self.get_logger().info(
            f">> Explorer Node Initialized. Linear speed: {self.LINEAR_SPD}, "
            f"Angular speed: {self.ANGULAR_SPD}, Exploration time: {self.EXPLORATION_TIME}")

    # Callback Methods
    """
        Handle dock status updates and initiate undocking on startup if robot is docked.
        
        Args:
            msg (DockStatus): Status message containing docking information.
                              msg.is_docked: Boolean indicating if robot is on dock.
    """
    def update_dock_status(self, msg):
        self.is_docked = msg.is_docked

        # Initial Boot logic: If on base, start mission
        if self.is_docked and self.state == RobotState.DOCKED:
            self.start_undocking()

    """        
        Args:
            msg (String): Control command. Supports:
                         - 'TELEOP': Switch to teleoperation mode
                         - 'AUTO': Resume autonomous exploration
    """
    def manual_override_callback(self, msg):
        if msg.data == 'TELEOP' and self.state != RobotState.MANUAL:
            self.get_logger().warn(">> TELEOP OVERRIDE: Human is taking control")
            self.saved_state = self.state
            self.state = RobotState.MANUAL
            self.send_velocity(0.0, 0.0)
        elif msg.data == 'AUTO' and self.state == RobotState.MANUAL:
            self.get_logger().info(">> RESUMING AUTO MODE")
            self.state = self.saved_state

    """ 
        When a bump is detected during exploration, the robot immediately backs up
        and performs an evasive rotation to escape the obstacle.
        
        Args:
            msg (HazardDetectionVector): Contains list of detected hazards.
                                         Each hazard has frame_id indicating type
                                         (e.g., 'bump_front', 'cliff_left').
    """
    def hazard_check(self, msg):
        if self.state != RobotState.EXPLORING:
            return

        if any(detection.header.frame_id.startswith('bump') for detection in msg.detections):
            self.get_logger().warn("Bumper triggered! Evading obstacle...")
            # Simple escape: Back up and rotate
            self.send_velocity(-0.1, 0.0) # Back up for 1 second
            time.sleep(1.0)
            self.send_velocity(0.0, 0.8) # Rotate in place for 1 second
            time.sleep(0.5)

    """        
        Processes 7 IR sensors to determine obstacle positions and adjust
        movement accordingly:
        - Front obstacles: Turn away from the side with stronger signal
        - Left obstacles: Veer right
        - Right obstacles: Veer left
        - Clear path: Move forward straight
        
        Also monitors mission elapsed time and initiates docking when
        EXPLORATION_TIME is reached.
        
        Args:
            msg (IrIntensityVector): IR intensity readings from 7 sensors.
                                    readings[0-2]: left side
                                    readings[3]: center
                                    readings[4-6]: right side
    """
    def ir_logic(self, msg):
        """
        IMPROVED OBSTACLE AVOIDANCE using 4-LEVEL DETECTION:
        
        Level 1 (IR > STOP):      Emergency crawl + hard turn
        Level 2 (IR > SLOW):      Proportional slowdown + turn
        Level 3 (IR > EARLY):     Gentle slowdown + prepare turn
        Level 4 (IR > V_EARLY):   First detection, subtle adjustment
        
        This prevents bumper hits by detecting obstacles EARLY and reacting
        SMOOTHLY before they're too close.
        """
        if self.state != RobotState.EXPLORING:
            return
 
        # Start exploration timer
        if self.mission_start_time is None:
            self.mission_start_time = self.get_clock().now()
 
        # Check if mission time expired
        elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        if elapsed > self.EXPLORATION_TIME:
            self.start_docking_mission()
            return
 
        # ============================================
        # EXTRACT IR SENSOR READINGS
        # ============================================
        readings = [r.value for r in msg.readings]
        
        far_left    = readings[0]
        left        = readings[1]
        front_left  = readings[2]
        center      = readings[3]
        front_right = readings[4]
        right       = readings[5]
        far_right   = readings[6]
 
        # Front obstacle detection (most important)
        front_intensity = max(front_left, center, front_right)
        
        # Side obstacle detection
        left_side_avg = (far_left + left + front_left) / 3
        right_side_avg = (far_right + right + front_right) / 3
 
        twist = Twist()
 
        # ============================================
        # PRIORITY 1: FRONT OBSTACLE AVOIDANCE
        # ============================================
        if front_intensity > self.IR_VERY_EARLY_THRESHOLD:
            
            # Determine which side to turn (turn away from stronger signal)
            turn_right = left_side_avg > right_side_avg
            
            if front_intensity > self.IR_STOP_THRESHOLD:
                # Very close obstacle (< 5cm)
                # Hard stop and aggressive turn
                self.set_leds(255, 0, 0)  # Red
                twist.linear.x = 0.02  # Minimal forward motion
                twist.angular.z = 1.0 if turn_right else -1.0  # Max turn
                self.get_logger().warn(
                    f"EMERGENCY STOP: intensity={front_intensity:.0f}, "
                    f"turn={'R' if turn_right else 'L'}")
                
            elif front_intensity > self.IR_SLOW_THRESHOLD:
                # Medium distance (~ 10cm)
                # Proportional slowdown
                self.set_leds(255, 0, 0)  # Red - critical
                
                # Smooth deceleration from SLOW to STOP threshold
                range_span = self.IR_STOP_THRESHOLD - self.IR_SLOW_THRESHOLD
                proximity_factor = (front_intensity - self.IR_SLOW_THRESHOLD) / range_span
                # Clamp: minimum 3% speed, maximum 50% speed (aggressive slowdown)
                speed_multiplier = max(0.03, min(0.5, 1.0 - proximity_factor))
                
                twist.linear.x = self.LINEAR_SPD * speed_multiplier
                twist.angular.z = 0.8 if turn_right else -0.8  # Hard turn
                self.get_logger().info(
                    f"APPROACHING: intensity={front_intensity:.0f}, "
                    f"speed={speed_multiplier:.1%}, turn={'R' if turn_right else 'L'}")
                
            elif front_intensity > self.IR_EARLY_THRESHOLD:
                # Far distance (~ 15cm)
                # Gentle slowdown
                self.set_leds(255, 165, 0)  # Orange
                
                range_span = self.IR_SLOW_THRESHOLD - self.IR_EARLY_THRESHOLD
                proximity_factor = (front_intensity - self.IR_EARLY_THRESHOLD) / range_span
                # Clamp: minimum 50% speed, maximum 100% speed
                speed_multiplier = max(0.5, min(1.0, 1.0 - proximity_factor))
                
                twist.linear.x = self.LINEAR_SPD * speed_multiplier
                twist.angular.z = 0.5 if turn_right else -0.5  # Moderate turn
                
            else:
                # Very far (~ 20cm)
                # First detection, subtle adjustment
                self.set_leds(255, 200, 0)  # Yellow
                
                twist.linear.x = self.LINEAR_SPD * 0.9  # Slight slowdown
                twist.angular.z = 0.3 if turn_right else -0.3  # Gentle turn
 
        # ============================================
        # PRIORITY 2: SIDE WALL DETECTION
        # ============================================
        elif left_side_avg > self.IR_EARLY_THRESHOLD:
            # Left wall detected
            self.set_leds(255, 165, 0)  # Orange
            
            # Gentle slowdown proportional to wall proximity
            if left_side_avg > self.IR_SLOW_THRESHOLD:
                range_span = self.IR_STOP_THRESHOLD - self.IR_SLOW_THRESHOLD
                proximity_factor = (left_side_avg - self.IR_SLOW_THRESHOLD) / range_span
                speed_mult = max(0.6, 1.0 - proximity_factor * 0.5)
            else:
                speed_mult = 0.85
            
            twist.linear.x = self.LINEAR_SPD * speed_mult
            twist.angular.z = -0.4  # Veer right
            
        elif right_side_avg > self.IR_EARLY_THRESHOLD:
            # Right wall detected
            self.set_leds(255, 165, 0)  # Orange
            
            # Gentle slowdown proportional to wall proximity
            if right_side_avg > self.IR_SLOW_THRESHOLD:
                range_span = self.IR_STOP_THRESHOLD - self.IR_SLOW_THRESHOLD
                proximity_factor = (right_side_avg - self.IR_SLOW_THRESHOLD) / range_span
                speed_mult = max(0.6, 1.0 - proximity_factor * 0.5)
            else:
                speed_mult = 0.85
            
            twist.linear.x = self.LINEAR_SPD * speed_mult
            twist.angular.z = 0.4  # Veer left
 
        # ============================================
        # PRIORITY 3: CLEAR PATH
        # ============================================
        else:
            self.set_leds(0, 255, 0)  # Green - all clear
            twist.linear.x = self.LINEAR_SPD
            twist.angular.z = 0.0
 
        self.cmd_pub.publish(twist)

    # Movement Helpers
    """
        Send velocity command to the robot.
                
        Args:
            x (float): Linear velocity in m/s (positive = forward)
            z (float): Angular velocity in rad/s (positive = counterclockwise)
    """
    def send_velocity(self, x, z):

        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.cmd_pub.publish(msg)

    # Action Client Methods (Undock & Dock)
    """        
        Sends an undock goal to the dock action server. Upon completion,
        transitions to EXPLORING state to begin autonomous exploration.
    """
    def start_undocking(self):
        self.state = RobotState.UNDOCKING
        self.undock_action.wait_for_server()
        goal = Undock.Goal()
        self.get_logger().info(">> Initiating Undock Sequence...")
        self.undock_action.send_goal_async(
            goal).add_done_callback(self.undock_result)

    """        
        Callback invoked when undocking is complete. Transitions robot
        to EXPLORING state and begins autonomous navigation.
        
        Args:
            future: Async result future from undock action.
    """
    def undock_result(self, future):
        self.state = RobotState.EXPLORING
        self.get_logger().info(">> Autonomous Exploration started.")

    """        
        Called when mission timer expires. Stops movement and sends dock goal
        to the dock action server for autonomous return to charger.
    """
    def start_docking_mission(self):
        self.get_logger().info(">> Mission time expires! Navigating to charger...")
        self.state = RobotState.RETURNING
        self.send_velocity(0.0, 0.0)  # Stop exploring

        self.dock_action.wait_for_server()
        goal = Dock.Goal()
        self.dock_action.send_goal_async(goal)

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