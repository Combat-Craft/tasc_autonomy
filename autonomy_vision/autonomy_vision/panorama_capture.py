#!/usr/bin/env python3

"""
ROS2 Node

Controls an automated camera panorama capture sweep.

When triggered, this node incrementally moves the camera servo through a sequence of angles
(from -110 to +110 degrees), captures one IP camera snapshot at each angle, stores GPS/heading
metadata for each frame, and saves the captured images into a named sweep folder.

Main behavior:
    1. Waits for a trigger message on /panorama_trigger.
    2. Uses the trigger message as the requested sweep folder name.
    3. Creates a unique folder inside ~/panorama_images.
    4. Publishes servo angle commands to /motor_cmd.
    5. Waits for the servo to move and settle.
    6. Captures one JPEG snapshot from the configured IP camera URL.
    7. Saves the image as frame_000.jpg, frame_001.jpg, etc.
    8. Stores the servo angle, GPS coordinates, rover heading, camera heading, and camera cardinal direction.
    9. Repeats until all sweep angles are complete.
    10. Saves metadata.csv inside the sweep folder.
    11. Resets and waits for the next trigger.

Published topics:
    - /motor_cmd
        Target camera servo angle in degrees, from -110 to +110.
        motor_controller.py subscribes to this topic and sends the mapped
        servo value to the Arduino.

Subscribed topics:
    - /panorama_trigger
        Trigger message used to start the panorama sweep.
        The message data is used as the requested output folder name.

        Example from motor_input.py:
            p,test_sweep_01

    - /gps/fix
        GPS coordinates saved with each captured frame.

    - /heading
        The direction the rover body is facing, measured in degrees clockwise from north,
        where 0° = North, 90° = East, 180° = South, and 270° = West.

        The camera-facing heading is calculated as:
            camera_heading = rover_heading + servo_angle

Parameters:
    - test_mode
        Default: False

        If true, the node simulates image captures and tests motor movement
        without requiring real camera frames.

    - cam_url
        Default: http://192.168.1.117:6688/snapshot/PROFILE_000

        IP camera snapshot URL used to capture JPEG images.

    - save_dir
        Default: ~/panorama_images

        Base directory where named sweep folders are created.

Output folder structure:
    ~/panorama_images/
    └── test_sweep_01/
        ├── frame_000.jpg
        ├── frame_001.jpg
        ├── frame_002.jpg
        ├── ...
        └── metadata.csv

metadata.csv columns:
    - frame
    - servo_angle
    - gps_latitude
    - gps_longitude
    - rover_heading
    - camera_heading
    - camera_cardinal
    - image_path

State machine:
    - IDLE
        Waiting for a panorama trigger.

    - MOVING
        Publishes the next servo angle command to /motor_cmd.

    - WAIT
        Waits for the servo to finish moving and settle before capturing.

    - CAPTURE
        Captures one image from the configured IP camera URL and saves it
        into the current sweep folder.

Related nodes/scripts:
    - motor_input.py
        Publishes manual servo commands to /motor_cmd and panorama triggers to
        /panorama_trigger using the format:
            p,folder_name

    - motor_controller.py
        Subscribes to /motor_cmd and sends mapped servo commands to Arduino.

    - GPS/IMU node or manual test publishers
        Provides /gps/fix and /heading.

    - panorama_opencv_stitcher.py
        Standalone non-ROS script that loads frame_*.jpg and metadata.csv from
        a sweep folder, stitches the images, draws metadata labels, and saves panorama.jpg.

Sweep configuration note:
    The current sweep uses 10 fixed servo angles from -110 to +110 degrees.
    The current timing values are:
        - move_time = 1.0s
        - settle_time = 1.0s
        - capture_time = 3.0s

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
import os
from pathlib import Path
import csv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Float32, String

class PanoramaSweeper(Node):

    def __init__(self):
        super().__init__('panorama_capture')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        # Test Mode
        self.declare_parameter('test_mode', False)
        self.TEST_MODE = self.get_parameter('test_mode').get_parameter_value().bool_value
        self.get_logger().info(f"TEST_MODE = {self.TEST_MODE}")

        # IP Camera URL
        self.declare_parameter('cam_url', 'http://192.168.1.117:6688/snapshot/PROFILE_000')
        self.cam_url = self.get_parameter('cam_url').get_parameter_value().string_value
        self.get_logger().info(f"IP Camera URL: {self.cam_url}")

        # Directory to save panorama sweep image folders
        self.declare_parameter('save_dir', '~/panorama_images')
        self.save_dir = Path(os.path.expanduser(self.get_parameter('save_dir').value))
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # Folder for the current triggered sweep
        self.current_sweep_dir = None

        self.get_logger().info(f"Base image save directory: {self.save_dir}")


        # -------------------------------------------------
        # Storage for captured panorama data
        # -------------------------------------------------

        self.frames = [] # Stores actual OpenCV image frames captured during the sweep
        self.angles = [] # Stores the servo angle corresponding to each captured frame.
        self.frame_metadata = []  # Stores GPS, heading, and compass metadata for each captured frame.

        # -------------------------------------------------
        # Sweep configuration
        # -------------------------------------------------

        # The camera moves from -110 degrees to +110 degrees and captures one image at each angle.
        self.sweep_angles = np.linspace(-110, 110, 10) # 220° / 9 gaps = 24.4° per image
        self.sweep_index = 0 # Index of the current sweep angle.

        # -------------------------------------------------
        # Timing configuration
        # -------------------------------------------------
        self.move_time   = 1.0   # how long servo takes to move
        self.settle_time = 1.0   # extra wait before capture
        self.capture_time = 3.0  # time required to get enough h265 frames to decode into an opencv frame
        self.ready_time  = None  # Timestamp when the camera is ready to capture after movement

        # -------------------------------------------------
        # Initialize state
        # -------------------------------------------------
        self.state = "IDLE"

        # -------------------------------------------------
        # GPS/IMU data
        # -------------------------------------------------

        # These values are updated continuously by their subscriber callbacks.
        # When a frame is captured, the latest available values are saved with it.
        self.latest_gps = None
        self.latest_heading = None

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
            String, 
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
    
    # -------------------------------------------------
    # 16-point compass conversion
    # -------------------------------------------------
    def heading_to_cardinal_16(self, heading):
        """
        Converts heading angle to 16-point compass direction.
        0° = N, 90° = E, 180° = S, 270° = W
        """

        directions = [
            "N", "NNE", "NE", "ENE",
            "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW",
            "W", "WNW", "NW", "NNW"
        ]

        index = int((heading + 11.25) / 22.5) % 16
        return directions[index]

    # -------------------------------------------------
    # Trigger sweep
    # -------------------------------------------------
    def trigger_callback(self, msg):
        """
        Starts the panorama sweep when a trigger message is received.
        Expected message format from motor_input.py:
            p,folder_name
        """

        if self.state != "IDLE":
            self.get_logger().warn("Sweep already in progress, ignoring trigger")
            return

        folder_name = msg.data.strip()

        if folder_name == "":
            self.get_logger().warn("Empty folder name received, ignoring trigger")
            return

        self.get_logger().info(f"Panorama trigger received with folder name: {folder_name}")
        self.start_sweep(folder_name)
    
    # -------------------------------------------------
    # Create folder for current sweep
    # -------------------------------------------------
    def create_sweep_folder(self, folder_name):
        """
        Creates a unique folder for each panorama sweep using the requested folder name.

        Example:
            ~/panorama_images/site1_sweep1/
        """

        # Replace unsafe filename characters with underscores.
        safe_name = ""
        for char in folder_name.strip():
            if char.isalnum() or char in ("_", "-"):
                safe_name += char
            else:
                safe_name += "_"

        if safe_name == "":
            safe_name = time.strftime("sweep_%Y%m%d_%H%M%S")

        folder_path = self.save_dir / safe_name

        # If the folder already exists, append a counter to make it unique.
        counter = 1
        while folder_path.exists():
            folder_path = self.save_dir / f"{safe_name}_{counter:03d}"
            counter += 1

        # Create the folder
        folder_path.mkdir(parents=True, exist_ok=False)

        # Store the current sweep folder path for saving images and metadata.
        self.current_sweep_dir = folder_path
        self.get_logger().info(f"Created sweep folder: {self.current_sweep_dir}")

    # -------------------------------------------------
    # Start sweep
    # -------------------------------------------------
    def start_sweep(self, folder_name):
        """
        Starts a new panorama sweep by creating a new save folder
        and switching the state to MOVING.
        """

        # Reset sweep storage at the start of a new sweep
        self.sweep_index = 0
        self.frames = []
        self.angles = []
        self.frame_metadata = []

        # Create a unique folder for this sweep
        self.create_sweep_folder(folder_name)

        self.get_logger().info(f"Starting panorama sweep, saving images to: {self.current_sweep_dir}")
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
            self.get_logger().info(f"Sweep complete → images saved to {self.current_sweep_dir}")
            self.save_metadata_csv() # Save metadata before resetting internal sweep data.

            # Return the camera servo to the center position after the sweep.
            center_msg = Float64()
            center_msg.data = 0.0
            self.servo_pub.publish(center_msg)
            self.get_logger().info("Centering the servo at 0° after sweep")

            self.reset_sweep()
            return

        # Get the current target angle.
        angle_deg = float(self.sweep_angles[self.sweep_index])

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
            
        # Make sure a sweep folder exists.
        # This should already be created in start_sweep(), but this is a safety check.
        if self.current_sweep_dir is None:
            self.get_logger().warn("No current sweep folder found. Creating one now.")
            self.create_sweep_folder(time.strftime("sweep_%Y%m%d_%H%M%S"))

        try:
            # Retrieve 640x360 JPEG from provided RTSP snapshot url
            req = request.urlopen(self.cam_url) #'http://192.168.1.117:6688/snapshot/PROFILE_000')

            # Decode the compressed JPEG image into an OpenCV frame
            np_arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except Exception as e:
            self.get_logger().error(f"Failed to capture image from IP camera: {e}")
            self.sweep_index += 1
            self.state = "MOVING"
            return

        # Skip current frame if decoding fails. This prevents the node from crashing due to a single bad frame.
        if frame is None:
            self.get_logger().error("Failed to decode image from IP camera")
            self.sweep_index += 1
            self.state = "MOVING"
            return

        # Get the servo angle associated with this captured frame.
        current_servo_angle = float(self.sweep_angles[self.sweep_index])

        if self.latest_heading is not None:
            camera_heading = (self.latest_heading + current_servo_angle) % 360.0
            camera_cardinal = self.heading_to_cardinal_16(camera_heading)
        else:
            camera_heading = None
            camera_cardinal = "?"

        # Save the image into the current sweep folder
        image_filename = f"frame_{self.sweep_index:03d}.jpg"
        image_path = self.current_sweep_dir / image_filename

        saved = cv2.imwrite(str(image_path), frame)

        if not saved:
            self.get_logger().error(f"Failed to save image: {image_path}")
            self.sweep_index += 1
            self.state = "MOVING"
            return

        self.get_logger().info(f"Saved image: {image_path}")

        # Store the captured frame and its servo angle
        self.frames.append(frame)
        self.angles.append(current_servo_angle)

        # Store sensor metadata at the moment of capture.
        metadata = {
            "frame": image_filename,
            "servo_angle": current_servo_angle,
            "gps": self.latest_gps,
            "rover_heading": self.latest_heading,
            "camera_heading": camera_heading,
            "camera_cardinal": camera_cardinal,
            "image_path": str(image_path),
        }

        self.frame_metadata.append(metadata)

        self.get_logger().info(
            f"Captured image at {current_servo_angle}° "
            f"({len(self.frames)}/{len(self.sweep_angles)})"
        )

        # Move to the next sweep angle.
        self.sweep_index += 1
        self.state = "MOVING"

    # -------------------------------------------------
    # Save metadata
    # -------------------------------------------------
    def save_metadata_csv(self):
        """
        Saves frame metadata for the current sweep into metadata.csv
        inside the current sweep folder.
        """

        if self.current_sweep_dir is None:
            self.get_logger().warn("No sweep folder available, cannot save metadata")
            return

        metadata_path = self.current_sweep_dir / "metadata.csv"

        try:
            with open(metadata_path, mode="w", newline="") as csv_file:
                writer = csv.writer(csv_file)

                writer.writerow([
                    "frame",
                    "servo_angle",
                    "gps_latitude",
                    "gps_longitude",
                    "rover_heading",
                    "camera_heading",
                    "camera_cardinal",
                    "image_path",
                ])

                for i, item in enumerate(self.frame_metadata):
                    gps = item.get("gps")

                    if gps is not None:
                        lat, lon = gps
                    else:
                        lat, lon = "", ""

                    writer.writerow([
                        item.get("frame", ""),
                        item.get("servo_angle", ""),
                        lat,
                        lon,
                        item.get("rover_heading", ""),
                        item.get("camera_heading", ""),
                        item.get("camera_cardinal", ""),
                        item.get("image_path", ""),
                    ])

            self.get_logger().info(f"Saved metadata → {metadata_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to save metadata CSV: {e}")

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
        self.current_sweep_dir = None
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
