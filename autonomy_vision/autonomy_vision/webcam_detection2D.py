#!/usr/bin/env python3
# Quick Note:
# Terminal 1: ros2 run autonomy_vision webcam_detection2D
# Terminal 2 (Command used for running usb_cam to get raw image data):
# ros2 run usb_cam usb_cam_node_exe --ros-args   -p video_device:=/dev/video0   -p pixel_format:=mjpeg2rgb   -p image_width:=640   -p image_height:=480   -p framerate:=10.0 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import numpy as np
import cv2
from ultralytics import YOLO
import time
 
 
class webcam_detection2D(Node):
    def __init__(self):
        # Initialize node
        super().__init__("webcam_detector")
 
        # Load YOLO11
        self.model = YOLO("yolo11n.pt")
        self.get_logger().info("YOLOv11n model loaded")
 
        # Subscribes to webcam topic published by usb_cam
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",           #usb_cam publishes webcam feed to /image_raw
            self.image_callback,
            10
        )
 
        # Publishers
        self.pub_det   = self.create_publisher(Detection2DArray, "/detections", 10) # detection results
        self.pub_debug = self.create_publisher(Image, "/debug_image", 10) # bounding boxes
 
        # FPS tracking
        self._fps_start = time.time()
        self._fps_count = 0
 
        self.get_logger().info("Webcam Detector is active")
 
    # Convert ROS2 image message to BGR numpy array
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
 
        # Convert to BGR so it can be used by OpenCV
        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
 
        return frame
 

    # Convert BGR numpy array to ROS2 image message
    def ndarray_to_image_msg(self, img, ref_msg):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ref_msg.header.frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3 
        msg.data = img.tobytes()
        return msg
 
    # Main callback is triggered each time a new frame is published by usb_cam
    def image_callback(self, msg: Image):
 
        #Convert ROS2 image message to OpenCV frame
        frame = self.image_msg_to_bgr(msg)
        if frame is None:
            return
 
        stamp = self.get_clock().now().to_msg()
 
        # run YOLO11n on the frame
        results = self.model(frame, verbose=False)
 
        # Detection2DArray ROS2 message
        det_array = Detection2DArray()
        det_array.header.stamp = stamp
        det_array.header.frame_id = msg.header.frame_id
 
        # Copy frame for adding bounding boxes
        debug_img = frame.copy()
 
        # Identify each detected object
        for det in results[0].boxes:
 
            # Bounding box pixel coordinates
            x1, y1, x2, y2 = map(int, det.xyxy[0].cpu().numpy())
 
            # Class index, confidence score, and human readable label
            cls_id = int(det.cls.cpu().numpy()[0])
            conf   = float(det.conf.cpu().numpy()[0])
            label  = self.model.names[cls_id]
 
            # Individual Detection2D message
            detection = Detection2D()
            detection.header.stamp = stamp
            detection.header.frame_id = msg.header.frame_id
 
            # Bounding box defined by center point and width/height
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
 
            # Attach class label and confidence score
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label
            hypo.hypothesis.score    = conf
            detection.results.append(hypo)
 
            det_array.detections.append(detection)
 
            # Draw bounding box and label the image
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                debug_img,
                f"{label} {conf:.2f}",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2
            )
 
        # Publish detection results and image with bounding boxes
        self.pub_det.publish(det_array)
        self.pub_debug.publish(self.ndarray_to_image_msg(debug_img, msg))
 
        # Log FPS every 3 seconds
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