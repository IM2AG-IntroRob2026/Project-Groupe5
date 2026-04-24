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
IR_CLOSE_STOP = 500   # Critical proximity
IR_WARNING_SLOW = 250 # Start braking
IR_AWARE_VEER = 100   # Early detection
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
    ESCAPING = "ESCAPING"
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
        - mode: Manual/auto/dock override commands
    
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
    
        self.declare_parameter('ir_very_early_threshold', 100)
        self.declare_parameter('ir_early_threshold', 150)
        self.declare_parameter('ir_slow_threshold', 250)
        self.declare_parameter('ir_stop_threshold', 500)
 
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        exploration_time = self.get_parameter('exploration_time').value
        self.LINEAR_SPD = float(linear_speed if linear_speed is not None else 0.15)
        self.ANGULAR_SPD = float(angular_speed if angular_speed is not None else 0.5)
        self.EXPLORATION_TIME = float(exploration_time if exploration_time is not None else 60.0)
        
        # Get IR thresholds from parameters
        ir_very_early = self.get_parameter('ir_very_early_threshold').value
        ir_early = self.get_parameter('ir_early_threshold').value
        ir_slow = self.get_parameter('ir_slow_threshold').value
        ir_stop = self.get_parameter('ir_stop_threshold').value
        self.IR_AWARE_VEER = float(ir_very_early if ir_very_early is not None else 100.0)
        self.IR_EARLY_THRESHOLD = float(ir_early if ir_early is not None else 150.0)
        self.IR_WARNING_SLOW = float(ir_slow if ir_slow is not None else 250.0)
        self.IR_CLOSE_STOP = float(ir_stop if ir_stop is not None else 500.0)

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
        self.escape_end_time = 0.0               # ESCAPING state end time (wall clock)
        self.pre_escape_state = RobotState.EXPLORING
        self.dock_goal_active = False
        self.last_dock_attempt_ns = 0
        self.dock_retry_interval_ns = int(2.5 * 1e9)
        self.close_red_cycles = 0
        self.red_cycle_limit = 4
        self.turn_lock_until_ns = 0
        self.turn_lock_duration_ns = int(0.6 * 1e9)
        self.turn_lock_sign = 1.0

        self.current_led_color = None  # Track color to avoid redundant publishing
        self.last_led_publish_ns = 0    # Republish same color periodically in case of packet loss
        self.light_pub = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
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

        if self.is_docked and self.state in (RobotState.RETURNING, RobotState.ESCAPING):
            self.state = RobotState.DOCKED
            self.dock_goal_active = False
            self.close_red_cycles = 0
            self.turn_lock_until_ns = 0
            self.send_velocity(0.0, 0.0)
            self.set_leds(0, 255, 0)

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
            self.set_leds(0, 0, 255)
            self.send_velocity(0.0, 0.0)
        elif msg.data == 'AUTO' and self.state == RobotState.MANUAL:
            self.get_logger().info(">> RESUMING AUTO MODE")
            self.state = self.saved_state
            self.set_leds(0, 255, 0)
        elif msg.data in ('DOCK', 'RETURN'):
            self.get_logger().info(">> DOCK COMMAND RECEIVED: Returning to base with avoidance")
            self.start_returning_home("manual command")

    """ 
        When a bump is detected during exploration, the robot immediately backs up
        and performs an evasive rotation to escape the obstacle.
        
        Args:
            msg (HazardDetectionVector): Contains list of detected hazards.
                                         Each hazard has frame_id indicating type
                                         (e.g., 'bump_front', 'cliff_left').
    """
    def hazard_check(self, msg):
        if self.state not in (RobotState.EXPLORING, RobotState.RETURNING):
            return

        if any(detection.header.frame_id.startswith('bump') for detection in msg.detections):
            self.pre_escape_state = self.state
            self.state = RobotState.ESCAPING
            self.escape_end_time = time.time() + 1.2
            self.close_red_cycles = 0
            self.turn_lock_until_ns = 0
            self.set_leds(180, 0, 255)
            self.get_logger().warn("BUMPER CONTACT! Backing up...")

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
        if self.state == RobotState.MANUAL:
            self.set_leds(0, 0, 255)
            return
 
        # Start exploration timer
        if self.state == RobotState.EXPLORING and self.mission_start_time is None:
            self.mission_start_time = self.get_clock().now()

        # Check if mission time expired
        if self.state == RobotState.EXPLORING:
            if self.mission_start_time is not None:
                elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
                if elapsed > self.EXPLORATION_TIME:
                    self.start_docking_mission()
                    return

        # Non-blocking ESCAPING state after bumper contact.
        if self.state == RobotState.ESCAPING:
            if time.time() < self.escape_end_time:
                self.set_leds(180, 0, 255)
                self.send_velocity(-0.1, 1.0)
                return
            self.state = self.pre_escape_state

        if self.state not in (RobotState.EXPLORING, RobotState.RETURNING):
            return

        now_ns = self.get_clock().now().nanoseconds
 
        # ============================================
        # EXTRACT IR SENSOR READINGS
        # ============================================
        readings = [r.value for r in msg.readings]
        max_front = max(readings[2], readings[3], readings[4])
        left_side = (readings[0] + readings[1] + readings[2]) / 3
        right_side = (readings[4] + readings[5] + readings[6]) / 3
        side_max = max(left_side, right_side)
 
        twist = Twist()
        speed_mult = 1.0
        obstacle_detected = max_front > self.IR_AWARE_VEER or side_max > self.IR_AWARE_VEER
        turn_right = left_side > right_side
        turn_sign = -1.0 if turn_right else 1.0
 
        # ============================================
        # PRIORITY 1: FRONT OBSTACLE AVOIDANCE
        # ============================================
        if max_front > self.IR_CLOSE_STOP:
            self.set_leds(255, 0, 0)
            speed_mult = 0.0
            twist.angular.z = turn_sign * self.ANGULAR_SPD * 1.8

        elif max_front > self.IR_WARNING_SLOW:
            self.set_leds(255, 120, 0)
            span = max(1.0, self.IR_CLOSE_STOP - self.IR_WARNING_SLOW)
            speed_mult = (self.IR_CLOSE_STOP - max_front) / span
            speed_mult = max(0.0, min(1.0, speed_mult))
            twist.angular.z = turn_sign * self.ANGULAR_SPD * 1.5

        elif obstacle_detected:
            self.set_leds(255, 255, 0)
            speed_mult = 0.85
            twist.angular.z = turn_sign * self.ANGULAR_SPD * 0.8

        else:
            self.set_leds(0, 255, 0)
            speed_mult = 1.0
            twist.angular.z = 0.0
 
        if side_max > self.IR_WARNING_SLOW:
            speed_mult = min(speed_mult, 0.45)
            if max_front <= self.IR_WARNING_SLOW:
                self.set_leds(255, 120, 0)
                twist.angular.z = turn_sign * self.ANGULAR_SPD * 1.2
        elif side_max > self.IR_AWARE_VEER:
            speed_mult = min(speed_mult, 0.8)

        twist.linear.x = self.LINEAR_SPD * max(0.0, speed_mult)

        if self.state == RobotState.RETURNING:
            twist.linear.x = min(twist.linear.x, self.LINEAR_SPD * 0.8)

            # Anti-oscillation guard: repeated red-zone cycles force a short turn-lock.
            if max_front > self.IR_CLOSE_STOP:
                self.close_red_cycles += 1
            else:
                self.close_red_cycles = 0

            if self.close_red_cycles >= self.red_cycle_limit and now_ns >= self.turn_lock_until_ns:
                self.turn_lock_until_ns = now_ns + self.turn_lock_duration_ns
                self.turn_lock_sign = turn_sign
                self.close_red_cycles = 0

            if now_ns < self.turn_lock_until_ns:
                self.set_leds(255, 0, 0)
                twist.linear.x = 0.0
                twist.angular.z = self.turn_lock_sign * self.ANGULAR_SPD * 1.7

            # When path is clear, periodically retry docking goal.
            if max_front < self.IR_AWARE_VEER and side_max < self.IR_AWARE_VEER:
                self.try_send_dock_goal()
 
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
        self.dock_goal_active = False
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
        self.start_returning_home("mission timeout")

    def start_returning_home(self, reason):
        if self.state == RobotState.DOCKED:
            self.get_logger().info(">> Already docked; ignoring return request")
            return

        self.get_logger().info(f">> Returning to charger ({reason})...")
        self.state = RobotState.RETURNING
        self.try_send_dock_goal()

    def try_send_dock_goal(self):
        if self.state != RobotState.RETURNING or self.dock_goal_active:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.last_dock_attempt_ns) < self.dock_retry_interval_ns:
            return

        if not self.dock_action.wait_for_server(timeout_sec=0.1):
            return

        self.last_dock_attempt_ns = now_ns
        self.dock_goal_active = True
        goal = Dock.Goal()
        self.dock_action.send_goal_async(goal).add_done_callback(self.on_dock_goal_response)

    def on_dock_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.dock_goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self.on_dock_result)

    def on_dock_result(self, _future):
        self.dock_goal_active = False

    def set_leds(self, r, g, b):
        now_ns = self.get_clock().now().nanoseconds
        # Re-publish unchanged color periodically to recover from dropped packets.
        if self.current_led_color == (r, g, b) and (now_ns - self.last_led_publish_ns) < int(0.3 * 1e9):
            return

        # Prevent redundant publishing to save bandwidth
        msg = LightringLeds()
        msg.override_system = True
        leds = []
        
        for _ in range(6): # The Create 3 has 6 LEDs in its ring
            color = LedColor()
            color.red, color.green, color.blue = r, g, b
            leds.append(color)

        msg.leds = leds
            
        self.light_pub.publish(msg)
        self.current_led_color = (r, g, b)
        self.last_led_publish_ns = now_ns

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