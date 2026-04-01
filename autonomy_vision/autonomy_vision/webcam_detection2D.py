#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesis
import numpy as np
import cv2
from ultralytics import YOLO
import time
 
 
class webcam_detection2D(Node):
    def __init__(self):
        # Initialize
        super().__init__("webcam_detector")
 
        # Load YOLO model
        self.model = YOLO("yolo11n.pt")
 
        # Subscribe to the webcam topic published by usb_cam
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )
 
        # Publishers
        self.pub_det   = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_debug = self.create_publisher(Image, "/debug_image", 10)
 
        # FPS tracking
        self._fps_start = time.time()
        self._fps_count = 0
 
        self.get_logger().info("Webcam Detector Node started — subscribed to /image_raw")
 
    # Convert ROS2 Image message to BGR numpy array
    def image_msg_to_bgr(self, msg: Image):
        encoding = msg.encoding.lower()
 
        channels_map = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}
        dtype_map    = {'bgr8': np.uint8, 'rgb8': np.uint8, 'mono8': np.uint8}
 
        ch = channels_map.get(encoding)
        dt = dtype_map.get(encoding)
 
        if ch is None:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return None
 
        frame = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.width, ch)
 
        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
 
        return frame
 
    # Convert BGR numpy array to ROS2 Image message
    def ndarray_to_image_msg(self, img, ref_msg):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ref_msg.header.frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg
 
    # Main callback whcih runs every time a new frame arrives on the topic
    def image_callback(self, msg: Image):
 
        # Convert ROS2 image message to OpenCV BGR frame
        frame = self.image_msg_to_bgr(msg)
        if frame is None:
            return
 
        # Timestamp for ROS2 messages
        stamp = self.get_clock().now().to_msg()
 
        # Run YOLO object detection
        results = self.model(frame, verbose=False)
 
        # Build Detection2DArray message
        det_array = Detection2DArray()
        det_array.header.stamp = stamp
        det_array.header.frame_id = msg.header.frame_id
 
        # Copy frame for drawing debug boxes
        debug_img = frame.copy()
 
        # Loop through all YOLO detections
        for det in results[0].boxes:
 
            # Bounding box corners in pixels
            x1, y1, x2, y2 = map(int, det.xyxy[0].cpu().numpy())
 
            # Class ID and confidence
            cls_id = int(det.cls.cpu().numpy()[0])
            conf   = float(det.conf.cpu().numpy()[0])
            label  = self.model.names[cls_id]
 
            # Fill Detection2D message
            detection = Detection2D()
            detection.header.stamp = stamp
            detection.header.frame_id = msg.header.frame_id
 
            # Bounding box as center and size (pixel space)
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
 
            # Class label and confidence score
            hypo = ObjectHypothesis()
            hypo.class_id = label
            hypo.score    = conf
            detection.results.append(hypo)
 
            det_array.detections.append(detection)
 
            # Draw bounding box and label on debug image
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                debug_img,
                f"{label} {conf:.2f}",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2
            )
 
        # Publish detections and debug image
        self.pub_det.publish(det_array)
        self.pub_debug.publish(self.ndarray_to_image_msg(debug_img, msg))
 
        # FPS logging every 3 seconds
        self._fps_count += 1
        elapsed = time.time() - self._fps_start
        if elapsed >= 3.0:
            fps = self._fps_count / elapsed
            self.get_logger().info(
                f"FPS: {fps:.1f}  |  Resolution: {msg.width}x{msg.height}"
            )
            self._fps_count = 0
            self._fps_start = time.time()
 
 
def main(args=None):
    rclpy.init(args=args)
    node = webcam_detection2D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
