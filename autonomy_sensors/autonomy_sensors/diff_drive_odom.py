#!/usr/bin/env python3
"""Consumes encoders/left and encoders/right; produces /wheel/odometry.
NEED TO MEASURE: 1) encoder count per rotation, 2) wheel diameter 3) wheel separation (track width) 
How to calibrate later:
Wheel radius scale
Command rover to drive straight 2-5 meters.
Compare odom distance vs tape measure.
Multiply wheel diameter by (true / odom).

Track width
Spin in place ~360° (or 720° is better).
Compare odom yaw vs actual yaw.
Multiply track width by (odom_yaw / true_yaw) or adjust iteratively.
This is the reality-based calibration that beats spec sheets.
"""
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    """Quaternion for yaw-only rotation."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


@dataclass
class EncoderState:
    have: bool = False
    last_count: int = 0
    last_time_sec: float = 0.0


class DiffDriveOdom(Node):
    """
    Differential-drive odometry using encoder *position counts*.

    Subscribes:
      /encoders/left  (std_msgs/Int64)
      /encoders/right (std_msgs/Int64)

    Publishes:
      /wheel/odometry (nav_msgs/Odometry)
      TF odom -> base_link (optional)
    """

    def __init__(self):
        super().__init__("diff_drive_odom")

        # --- Parameters (easy to calibrate later) ---
        self.declare_parameter("ticks_per_rev", 2048)   # change to your encoder
        self.declare_parameter("wheel_diameter_m", 0.20)  # 20 cm default
        self.declare_parameter("track_width_m", 0.40)     # 40 cm default (likely wrong, but OK to start)
        self.declare_parameter("frame_id_odom", "odom")
        self.declare_parameter("frame_id_base", "base_link")
        self.declare_parameter("publish_tf", True)

        self.ticks_per_rev = int(self.get_parameter("ticks_per_rev").value)
        self.wheel_d = float(self.get_parameter("wheel_diameter_m").value)
        self.track_w = float(self.get_parameter("track_width_m").value)
        self.frame_odom = str(self.get_parameter("frame_id_odom").value)
        self.frame_base = str(self.get_parameter("frame_id_base").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.ticks_per_rev <= 0:
            raise ValueError("ticks_per_rev must be > 0")
        if self.wheel_d <= 0.0:
            raise ValueError("wheel_diameter_m must be > 0")
        if self.track_w <= 0.0:
            raise ValueError("track_width_m must be > 0")

        self.wheel_r = 0.5 * self.wheel_d
        self.m_per_tick = (2.0 * math.pi * self.wheel_r) / float(self.ticks_per_rev)

        # --- State ---
        self.left = EncoderState()
        self.right = EncoderState()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # --- ROS I/O ---
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", 50)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Int64, "/encoders/left", self.left_cb, 50)
        self.create_subscription(Int64, "/encoders/right", self.right_cb, 50)

        # Timer to compute/publish at a steady rate
        self.timer = self.create_timer(1.0 / 30.0, self.update)

        self.get_logger().info(
            f"DiffDriveOdom started: ticks_per_rev={self.ticks_per_rev}, "
            f"wheel_d={self.wheel_d:.3f} m, track_w={self.track_w:.3f} m, "
            f"m_per_tick={self.m_per_tick:.6e}"
        )

    def left_cb(self, msg: Int64):
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.left.have:
            self.left.have = True
            self.left.last_count = int(msg.data)
            self.left.last_time_sec = t
            return
        # store latest only; integration happens in update()
        self.left.latest_count = int(msg.data)
        self.left.latest_time_sec = t

    def right_cb(self, msg: Int64):
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.right.have:
            self.right.have = True
            self.right.last_count = int(msg.data)
            self.right.last_time_sec = t
            return
        self.right.latest_count = int(msg.data)
        self.right.latest_time_sec = t

    def update(self):
        # Need both sides initialized and at least one update each
        if not (self.left.have and self.right.have):
            return
        if not (hasattr(self.left, "latest_count") and hasattr(self.right, "latest_count")):
            return

        # Use a common dt based on current node time
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        # Compute delta counts since last integration
        dl_ticks = int(self.left.latest_count) - int(self.left.last_count)
        dr_ticks = int(self.right.latest_count) - int(self.right.last_count)

        # If nothing changed, still publish odom with zero twist (optional)
        if dl_ticks == 0 and dr_ticks == 0:
            self.publish(now, 0.0, 0.0)
            return

        # Compute time step (use time since last update() publish)
        # We'll track it ourselves to avoid weirdness if encoder msgs are bursty.
        if not hasattr(self, "last_update_sec"):
            self.last_update_sec = now_sec
            # don't integrate on first tick batch
            self.left.last_count = int(self.left.latest_count)
            self.right.last_count = int(self.right.latest_count)
            return

        dt = now_sec - self.last_update_sec
        if dt <= 0.0 or dt > 1.0:
            # if paused or clock jumped, reset integration
            self.last_update_sec = now_sec
            self.left.last_count = int(self.left.latest_count)
            self.right.last_count = int(self.right.latest_count)
            return

        self.last_update_sec = now_sec

        # Convert ticks to meters along each side
        ds_l = dl_ticks * self.m_per_tick
        ds_r = dr_ticks * self.m_per_tick

        # Differential drive kinematics
        ds = 0.5 * (ds_r + ds_l)
        dtheta = (ds_r - ds_l) / self.track_w

        # Midpoint integration for better accuracy
        yaw_mid = self.yaw + 0.5 * dtheta
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw += dtheta

        # Normalize yaw to [-pi, pi]
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        v = ds / dt
        w = dtheta / dt

        # update last counts
        self.left.last_count = int(self.left.latest_count)
        self.right.last_count = int(self.right.latest_count)

        self.publish(now, v, w)

    def publish(self, now, v, w):
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(w)

        # Very rough covariances (tune later)
        # Position covariance in x/y; yaw not directly used here but included in pose covariance.
        pos_var = 0.05 ** 2
        yaw_var = (math.radians(2.0)) ** 2
        odom.pose.covariance = [
            pos_var, 0, 0, 0, 0, 0,
            0, pos_var, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,         # z huge (2D)
            0, 0, 0, 1e6, 0, 0,          # roll huge
            0, 0, 0, 0, 1e6, 0,          # pitch huge
            0, 0, 0, 0, 0, yaw_var
        ]

        v_var = 0.1 ** 2
        w_var = (math.radians(10.0)) ** 2
        odom.twist.covariance = [
            v_var, 0, 0, 0, 0, 0,
            0, 1e6, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, w_var
        ]

        self.odom_pub.publish(odom)

        # TF
        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.frame_odom
            tf.child_frame_id = self.frame_base
            tf.transform.translation.x = float(self.x)
            tf.transform.translation.y = float(self.y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = DiffDriveOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
