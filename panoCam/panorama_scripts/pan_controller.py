#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class PanController(Node):
    def __init__(self):
        super().__init__('pan_controller')
        
        # adjustable parameters & safety limits
        self.declare_parameter("min_angle", -135.0)
        self.declare_parameter("max_angle", 135.0)
        self.declare_parameter("max_speed", 30.0) #deg/s
        self.declare_parameter("acceleration", 60.0) #deg/s^2; speeding up/down of servo motor
        self.declare_parameter("update_rate", 0.05) #Updates every 0.05s or 20Hz
        self.declare_parameter("deadband", 0.5) #If error is smaller than than deadband, then it will stop moving to avoid motor shaking

        self.declare_parameter("lag_warn_threshold", 5.0) # Warn if commanded angle and actual motor angle differs by more than 5 deg


        self.declare_parameter("pan_joint_name", "camera_pan_joint") # Name of joint inside /joint_states message

        # Where motor commands are published
        self.declare_parameter(
            "cmd_topic",
            "/camera_pan_position_controller/commands"
        )

        # If true, then start target at 0 degrees
        self.declare_parameter("center_on_start", True)

        # Initialize parameters
        self.min_angle = self.get_parameter("min_angle").value
        self.max_angle = self.get_parameter("max_angle").value
        self.max_speed = self.get_parameter("max_speed").value
        self.acceleration = self.get_parameter("acceleration").value
        self.update_rate = self.get_parameter("update_rate").value
        self.deadband = self.get_parameter("deadband").value
        self.lag_warn_threshold = self.get_parameter("lag_warn_threshold").value
        self.pan_joint_name = self.get_parameter("pan_joint_name").value
        cmd_topic = self.get_parameter("cmd_topic").value
        center_on_start = self.get_parameter("center_on_start").value

        # FIX 3: Validate acceleration to prevent decel_dist becoming infinite
        if self.acceleration <= 0.0:
            self.get_logger().warn(
                f"Acceleration parameter is {self.acceleration} — must be > 0. Defaulting to 60.0 deg/s^2."
            )
            self.acceleration = 60.0

        # Runtime variables

        self.command_angle = 0.0
        # This is the controller's internal "planned position"
        # It moves gradually toward the target

        self.actual_angle = None
        # This is hardware feedback from /joint_states
        # Starts as None until first message arrives

        self.target_angle = 0.0
        # Desired angle set by user

        self.current_speed = 0.0
        # Current velocity of motion (deg/s)

        self.joint_state_received = False
        # Prevents motion before hardware feedback exists

        '''
        Subscribers
        '''

        # Listens for desired target angle commands
        self.create_subscription(
            Float64,
            '/pan_angle_cmd',
            self.cmd_callback,
            10
        )
        
        # Listens to servo feedback
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        '''
        Publisher
        '''
        # Sends motor commands to servo controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            cmd_topic,
            10
        )

        #Timer (control loop)
        self.timer = self.create_timer(self.update_rate, self.update)
        # Every 0.05 seconds, it calls update()

        '''
        Startup
        '''
        if center_on_start:
            self.target_angle = 0.0
            self.get_logger().info("Startup centering enabled — targeting 0.0 deg")

        self.get_logger().info(
            f"Pan Controller started — publishing to '{cmd_topic}'\n"
            f"  Limits         : [{self.min_angle} deg, {self.max_angle} deg]\n"
            f"  Max speed      : {self.max_speed} deg/s\n"
            f"  Acceleration   : {self.acceleration} deg/s^2\n"
            f"  Deadband       : {self.deadband} deg\n"
            f"  Lag warn       : {self.lag_warn_threshold} deg\n"
            f"  Joint name     : {self.pan_joint_name}"
        )

    
    # JOINT STATE CALLBACK
    def joint_state_callback(self, msg: JointState):

        # Check if desired joint (camera_pan_joint) exists in message
        if self.pan_joint_name not in msg.name:
            return  # ignore unrelated joints

        # Find index of the joint
        idx = msg.name.index(self.pan_joint_name)

        # FIX 5: Guard against malformed JointState where position list is shorter than name list
        if idx >= len(msg.position):
            self.get_logger().warn(
                f"JointState message has no position entry for '{self.pan_joint_name}' — skipping.",
                throttle_duration_sec=3.0
            )
            return

        # Convert radians to degrees
        self.actual_angle = math.degrees(msg.position[idx])

        # first time initialization
        if not self.joint_state_received:

            # Align controller with real hardware position
            self.command_angle = self.actual_angle

            self.joint_state_received = True

            self.get_logger().info(
                f"Joint state received — synced to {self.actual_angle:.2f}°"
            )

    # COMMAND CALLBACK
    def cmd_callback(self, msg):

        # Extract desired angle from message
        angle = msg.data

        # Ensure that desired angle is within limits
        if angle > self.max_angle or angle < self.min_angle:
            self.get_logger().warn(
                f"Command {angle:.1f} deg exceeded limits "
                f"[{self.min_angle:.1f} deg, {self.max_angle:.1f} deg] — clamping.",
                throttle_duration_sec=2.0
            )

        # Clamp value into safe range
        angle = max(min(angle, self.max_angle), self.min_angle)

        # If the new target reverses direction of travel, reset speed to avoid overshoot.
        # Compare sign of old error vs new error — current_speed is always >= 0 so
        # it cannot be used directly to detect direction.
        old_error = self.target_angle - self.command_angle
        new_error = angle - self.command_angle
        if old_error * new_error < 0:
            self.current_speed = 0.0

        # Update target position
        self.target_angle = angle

        # Only log when the target meaningfully changes to avoid spamming logs
        if abs(new_error) > 0.1:
            self.get_logger().info(f"Target pan angle: {angle:.1f}°")

    # CONTROL LOOP (20 Hz)
    def update(self):

        # Do nothing until hardware feedback exists
        if not self.joint_state_received:
            self.get_logger().warn(
                "Waiting for /joint_states ...",
                throttle_duration_sec=3.0
            )
            return

        dt = self.update_rate

        # Error between desired and internal command state
        error = self.target_angle - self.command_angle

        # If close enough, then stop moving
        if abs(error) < self.deadband:
            self.current_speed = 0.0

        else:
            # Determine direction of motion (1 for cw and -1 for ccw)
            direction = 1.0 if error > 0 else -1.0

            # Distance needed to stop safely
            decel_dist = (self.current_speed ** 2) / (2.0 * self.acceleration)

            # Decide whether to accelerate or decelerate
            if abs(error) <= decel_dist:
                # slowing down
                self.current_speed = max(
                    0.0,
                    self.current_speed - self.acceleration * dt
                )
            else:
                # speeding up
                self.current_speed = min(
                    self.current_speed + self.acceleration * dt,
                    self.max_speed
                )

            # Compute movement
            step = min(self.current_speed * dt, abs(error))

            # Update internal commanded position
            self.command_angle += direction * step

        # LAG MONITORING
        if self.actual_angle is not None:
            lag = abs(self.command_angle - self.actual_angle)

            if lag > self.lag_warn_threshold:
                self.get_logger().warn(
                    f"Servo lag: {lag:.1f}° "
                    f"(cmd={self.command_angle:.1f}°, actual={self.actual_angle:.1f}°)",
                    throttle_duration_sec=1.0
                )

            # If lag is severe, re-sync command_angle to hardware to prevent long-term drift
            if lag > self.lag_warn_threshold * 2:
                self.command_angle = self.actual_angle

        # PUBLISH COMMAND
        msg = Float64MultiArray()

        # Convert degrees → radians for servo controller
        msg.data = [math.radians(self.command_angle)]

        self.cmd_pub.publish(msg)


# MAIN
def main(args=None):

    rclpy.init(args=args)
    node = PanController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()