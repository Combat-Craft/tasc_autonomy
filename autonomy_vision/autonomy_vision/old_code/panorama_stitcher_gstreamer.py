#!/usr/bin/env python3

"""
ROS2 Node

Controls an automated camera panorama sweep.
When triggered, this node incrmentally moves the camera servo through a sequence of angles (from -135 to +135 degrees),
captures one compressed camera frame at each angle, stores GPS/heading metadata
for each frame, and then attempts to stitch the captured frames into a panorama.

Main behavior:
    1. Waits for a trigger message on /panorama_trigger.
    2. Publishes servo angle commands to /motor_cmd.
    3. Waits for the servo to move and settle.
    4. Captures a compressed image frame from /arm_cam/image/compressed.
    5. Saves the frame, servo angle, GPS coordinates, heading, and cardinal direction.
    6. Repeats until all sweep angles are complete.
    7. Stitches the captured frames into panorama.jpg.
    8. Resets and waits for the next trigger.

Published topics:
    - /motor_cmd
        Target camera servo angle in degrees, from -135 to +135.
        motor_controller.py subscribes to this topic and sends the mapped
        servo value to the Arduino.


Subscribed topics:
    - /panorama_trigger
        Trigger message used to start the panorama sweep.
        The actual value is not important; receiving a message starts the sweep.

    - /gps/fix
        GPS coordinates used for the panorama overlay.

    - /heading
        the direction the rover body is facing, measured in degrees clockwise from north, \
        where 0° = North, 90° = East, 180° = South, and 270° = West
        
    - /cardinal_compass
        the compass direction the rover body is facing, using a 16 piece compass rose
        i.e. N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW 

Parameters:
    - test_mode
        If true, the node simulates image captures and tests motor movement without requiring real camera frames.        
        
Related nodes:
    - motor_input.py
        Publishes manual servo commands to /motor_cmd and panorama triggers to
        /panorama_trigger.

    - motor_controller.py
        Subscribes to /motor_cmd and sends mapped servo commands to Arduino.

    - GPS/IMU node or manual test publishers
        Provides /gps/fix and /heading.

Sweep configuration note:
    The current sweep uses a fixed number of servo angles (18) and fixed timing values (move time is 1.0s and settle time is 1.0s).

    When the camera is mounted, these values may need to be adjusted based on:
        - camera field of view
        - amount of overlap between neighbouring frames
        - servo speed
        - camera mount vibration
        - image blur during capture
"""

import time
import cv2
import urllib.request as request
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Float32, String

class PanoramaSweeper(Node):

    def __init__(self):
        super().__init__('panorama_stitcher')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        # test_mode allows the servo sweep to be tested without a real camera.
        # When test_mode is true, the node simulates captures instead of waiting
        # for images from /arm_cam/image/compressed.
        self.declare_parameter('test_mode', False)
        self.TEST_MODE = self.get_parameter('test_mode').get_parameter_value().bool_value

        self.get_logger().info(f"TEST_MODE = {self.TEST_MODE}")

        self.cam_url = 'http://192.168.1.117:6688/snapshot/PROFILE_000'

        # -------------------------------------------------
        # Storage for captured panorama data
        # -------------------------------------------------
        self.frames = [] # Stores actual OpenCV image frames captured during the sweep
        self.angles = [] # Stores the servo angle corresponding to each captured frame.
        self.frame_metadata = []  # Stores GPS, heading, and compass metadata for each captured frame.

        # -------------------------------------------------
        # Sweep configuration
        # -------------------------------------------------
        
        # Sweep angles in degrees.
        # The camera moves from -135 degrees to +135 degrees and captures one image at each angle.
        self.sweep_angles = np.linspace(-135, 135, 10) # 270° / 17 gaps = 15.9° per image
        self.sweep_index = 0 # Index of the current sweep angle.

        # -------------------------------------------------
        # Timing configuration
        # -------------------------------------------------
        self.move_time   = 1.0   # how long servo takes to move
        self.settle_time = 1.0   # extra wait before capture
        self.capture_time = 3.0  # time required to get enough h265 frames to decode into an opencv frame
        self.ready_time  = None  # Timestamp when the camera is ready to capture after movement

        # -------------------------------------------------
        # State machine
        # -------------------------------------------------
        # IDLE:
        #     Waiting for panorama trigger.
        #
        # MOVING:
        #     Publishing the next servo angle command.
        #
        # WAIT:
        #     Waiting for the servo to finish moving and settle.
        #
        # CAPTURE:
        #     Waiting for the next camera frame to capture.
        self.state = "IDLE"

        # -------------------------------------------------
        # GPS/IMU data
        # -------------------------------------------------

        # These values are updated continuously by their subscriber callbacks.
        # When a frame is captured, the latest available values are saved with it.
        self.latest_gps = None
        self.latest_heading = None
        self.latest_cardinal = None

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------

        # Publishes target servo angles to motor_controller.py.
        self.servo_pub = self.create_publisher(
            Float64,
            '/motor_cmd',
            10
        )

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------

        # Receives trigger messages to start a panorama sweep.
        self.create_subscription(
            Float64, 
            '/panorama_trigger', 
            self.trigger_callback, 
            10
        )

        # Receives GPS coordinates.
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # Receives numeric heading in degrees.
        self.heading_sub = self.create_subscription(
            Float32,
            '/heading',
            self.heading_callback,
            10
        )
        
        # Receives cardinal compass direction 
        self.heading_sub = self.create_subscription(
            String,
            '/cardinal_compass',
            self.cardinal_callback,
            10
        )

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        # Runs the update loop every 0.1 seconds.
        # This drives the state machine without blocking ROS callbacks. 
        self.timer = self.create_timer(0.1, self.update)
        self.triggered = False # Start-up delay

        self.get_logger().info("PanoramaSweeper ready | Send trigger to /panorama_trigger")


    # -------------------------------------------------
    # Main update loop
    # -------------------------------------------------
    def update(self):
        """
        Main state-machine loop.

        This function is called repeatedly by the ROS2 timer.
        It controls when to move the servo, when to wait, and when to capture.
        """

        now = time.time()

        # If the node is in MOVING state, publish the next servo angle.
        # Else if the servo has had enough time to move and settle, switch to CAPTURE state.
        if self.state == "MOVING":
            self.move_to_next_angle()
        elif self.state == "WAIT" and now >= self.ready_time:
            self.state = "CAPTURE"
            
            # gstreamer addition since no longer subscription callback
            self.get_image()

        # TEST MODE:
        # In test mode, no real camera frames are required.
        # The node pretends a frame was captured and moves to the next angle.
        elif self.state == "CAPTURE" and self.TEST_MODE:
            self.get_logger().info(
                f"[TEST] Simulated capture at {self.sweep_angles[self.sweep_index]} degrees"
                f"({self.sweep_index+1}/{len(self.sweep_angles)})"
            )
            self.sweep_index += 1
            self.state = "MOVING"

    # -------------------------------------------------
    # GPS / IMU callbacks
    # -------------------------------------------------

    def gps_callback(self, msg):
        """
        Stores the latest GPS latitude and longitude.
        """
        self.latest_gps = (msg.latitude, msg.longitude)

    def heading_callback(self, msg):
        """
        Stores the latest numeric heading in degrees.
        """
        self.latest_heading = msg.data
        
    def cardinal_callback(self, msg):
        """
        Stores the latest numeric heading in degrees.
        """
        self.latest_cardinal = msg.data
   
    # -------------------------------------------------
    # Trigger sweep
    # -------------------------------------------------
    def trigger_callback(self, msg):
        """
        Starts the panorama sweep when a trigger message is received.
        If a sweep is already running, the trigger is ignored.
        """
        
        if self.state != "IDLE":
            self.get_logger().warn("Sweep already in progress, ignoring trigger")
            return
        self.get_logger().info("Panorama trigger received")
        self.start_sweep()
    
    # -------------------------------------------------
    # Start sweep
    # -------------------------------------------------
    def start_sweep(self):
        """
        Starts a new panorama sweep by switching the state to MOVING.
        """
        self.get_logger().info("Starting panorama sweep")
        self.state = "MOVING"

    # -------------------------------------------------
    # Command servo
    # -------------------------------------------------
    def move_to_next_angle(self):
        """
        Sends the next servo angle command.

        If all angles have already been visited, the node starts stitching.
        """

        # If all sweep angles have been processed, create the panorama.
        if self.sweep_index >= len(self.sweep_angles):
            self.get_logger().info("Sweep complete → stitching")
            self.stitch_panorama()
            return

        # Get the current target angle.
        angle_deg = self.sweep_angles[self.sweep_index]

        # Publish the target angle to /motor_cmd.
        msg = Float64()
        msg.data = angle_deg
        self.servo_pub.publish(msg)

        self.get_logger().info(
            f"Moving to {angle_deg}° "
            f"({self.sweep_index+1}/{len(self.sweep_angles)})"
        )

        # Set the time when the node should be ready to capture an image.
        self.ready_time = time.time() + self.move_time + self.settle_time

        # Wait before capturing to give the servo time to move and settle.
        self.state = "WAIT"

    # -------------------------------------------------
    # Image Capture
    # -------------------------------------------------
    def get_image(self):
        """
        Handles incoming compressed camera images.

        A frame is only saved if:
            1. test_mode is false
            2. the node is currently in CAPTURE state
            3. the rtsp image can be decoded successfully
        """

        # In test mode, real camera frames are ignored.
        if self.TEST_MODE:
            return

        # Ignore camera frames unless the node is ready to capture in order to
        # prevent unwanted frames from being saved while the servo is moving.
        if self.state != "CAPTURE":
            return
            
        # Retrieve 640x360 JPEG from provided RTSP snapshot url 
        req = request.urlopen(self.cam_url)#'http://192.168.1.117:6688/snapshot/PROFILE_000')

        # Decode the compressed JPEG image into an OpenCV frame
        np_arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
        #frame = cv2.imdecode(np_arr, -1) # 'Load it as it is'
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)        

        # If decoding fails, skip this frame.
        if frame is None:
            return

        #cv2.imwrite(f"frame_{self.sweep_index}.jpg", frame) # save each frame for debugging

        # Get the servo angle associated with this captured frame.
        current_servo_angle = self.sweep_angles[self.sweep_index]

        # Store the captured frame and its servo angle
        self.frames.append(frame)
        self.angles.append(current_servo_angle)

        # Store sensor metadata at the moment of capture.
        metadata = {
            "servo_angle": current_servo_angle,
            "gps": self.latest_gps,
            "heading": self.latest_heading,
        }

        self.frame_metadata.append(metadata)

        self.get_logger().info(
            f"Captured image at {current_servo_angle}° "
            f"({len(self.frames)}/{len(self.sweep_angles)})"
        )

        # Move to the next sweep angle.
        self.sweep_index += 1
        self.state = "MOVING" # Change state from CAPTURE to MOVING

        

    # -------------------------------------------------
    # Stitch panorama
    # -------------------------------------------------
    def stitch_panorama(self):
        """
        Creates and saves the panorama image.

        In test mode, stitching is skipped because no real camera frames exist.
        In normal mode, OpenCV attempts to stitch all captured frames.
        """

        # TEST MODE:
        # In test mode, only the servo sweep is tested.
        # There are no real frames to stitch.
        if self.TEST_MODE:
            self.get_logger().info("[TEST] Sweep complete - skipping stitch since there are no real camera frames")
            
            # Reset for next sweep
            self.reset_sweep()
            return

        # At least two frames are needed for stitching.
        if len(self.frames) < 2:
            self.get_logger().warn("Not enough frames to stitch")
            return

        self.get_logger().info(f"Stitching {len(self.frames)} images")

        # Create OpenCV panorama stitcher and attempt to stitch the frames.
        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA) #Stitcher_create uses CUDA
        status, pano = stitcher.stitch(self.frames)

        self.get_logger().info(f"Called OpenCV stitcher → status {status}")

        # If OpenCV cannot stitch the images, reset and wait for another trigger.
        if status != cv2.Stitcher_OK:
            self.get_logger().error(f"Stitch failed (status {status})")
            self.reset_sweep()
            return

        # -------------------------------------------------
        # Draw overlays on the panorama
        # -------------------------------------------------
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255, 255, 255)         # White text
        shadow_color = (0, 0, 0)        # Black shadow for readability

        h, w = pano.shape[:2]

        # -------------------------------------------------
        # Draw GPS summary at top-left
        # -------------------------------------------------

        # Uses the first captured frame's GPS metadata as the general GPS label
        # for the full panorama.
        if len(self.frame_metadata) > 0:
            first_meta = self.frame_metadata[0]
            gps = first_meta.get("gps")

            if gps is not None:
                lat, lon = gps
                gps_text = f"GPS: {lat:.6f}, {lon:.6f}"
            else:
                gps_text = "GPS: unavailable"

            # Draw black shadow first, then white text on top.
            cv2.putText(pano, gps_text, (20, 30), font, 0.6, shadow_color, 3, cv2.LINE_AA)
            cv2.putText(pano, gps_text, (20, 30), font, 0.6, color, 1, cv2.LINE_AA)


        # -------------------------------------------------
        # Draw heading/cardinal markers at top
        # -------------------------------------------------

        # These labels show the camera direction for each captured frame alongside its corresponding cardinal direction.
        for i, angle in enumerate(self.angles):

            # Spread markers evenly across the panorama width.
            x = int((i / (len(self.angles) - 1)) * (w - 40)) + 20

            # Get the metadata saved at the same time as this frame.
            meta = self.frame_metadata[i] if i < len(self.frame_metadata) else {}

            # This is the rover heading from /heading.
            base_heading = meta.get("heading")

            if base_heading is not None:
                # If /heading is the rover/body heading, then the camera direction is:
                # rover heading + servo angle.
                camera_heading = (base_heading + angle) % 360.0

                cardinal = self.latest_cardinal

                heading_label = f"{camera_heading:.0f}"
                cardinal_label = cardinal
            else:
                heading_label = "N/A"
                cardinal_label = "?"

            # Cardinal label slightly above heading label.
            cv2.putText(pano, cardinal_label, (x - 12, 55), font, 0.5, shadow_color, 3, cv2.LINE_AA)
            cv2.putText(pano, cardinal_label, (x - 12, 55), font, 0.5, color, 1, cv2.LINE_AA)

            cv2.putText(pano, heading_label, (x - 18, 75), font, 0.45, shadow_color, 3, cv2.LINE_AA)
            cv2.putText(pano, heading_label, (x - 18, 75), font, 0.45, color, 1, cv2.LINE_AA)

            # Small top tick mark.
            cv2.line(pano, (x, 85), (x, 105), color, 1)


        # -------------------------------------------------
        # Draw servo angle markers at bottom
        # -------------------------------------------------

        # These labels show the servo angle for each captured frame.
        for i, angle in enumerate(self.angles):

            # Spread markers evenly across the panorama width.
            x = int((i / (len(self.angles) - 1)) * (w - 40)) + 20

            label_angle = f"{int(angle)}"

            # Draw label shadow and text.
            cv2.putText(pano, label_angle, (x - 15, h - 20), font, 0.5, shadow_color, 3, cv2.LINE_AA)
            cv2.putText(pano, label_angle, (x - 15, h - 20), font, 0.5, color, 1, cv2.LINE_AA)

            # Draw bottom tick mark.
            cv2.line(pano, (x, h - 40), (x, h - 10), color, 1)

        # Save the final panorama image to the current working directory.
        cv2.imwrite("panorama.jpg", pano)
        self.get_logger().info("Panorama saved → panorama.jpg")

        # Display the panorama in an OpenCV window.
        # Commented out the code below because it causes issues when trying to open a window
        # cv2.imshow("Panorama", pano)
        # cv2.waitKey(1)

        # Reset for next triggered panorama sweep
        self.reset_sweep()

    # -------------------------------------------------
    # Reset sweep
    # -------------------------------------------------
    def reset_sweep(self):
        """
        Clears stored frames, metadata, and returns the node to IDLE state.
        """
        self.sweep_index = 0
        self.frames = []
        self.angles = []
        self.frame_metadata = []
        self.ready_time = None
        self.state = "IDLE"
        self.get_logger().info("PanoramaSweeper reset → ready for next trigger")

def main(args=None):
    rclpy.init(args=args)
    node = PanoramaSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
