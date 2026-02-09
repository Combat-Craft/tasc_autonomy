#!/usr/bin/env python3
# To be used in conjunction with the Arduino sketch "imu_gps_simple" which publishes data in the format 
#  GPS,ms,lat,lon,alt,speed,hdop,sats,fix at 1Hz
#  IMU,ms,ax,ay,az,gx,gy,gz at 100Hz
# This is the "No microROS, only Serial" option


import serial
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header

'''Output from estimate_covariance_from_log while sitting on my desk:
IMU (stationary) robust std:
  accel std [m/s^2]: [0.02129755 0.02129755 0.0425951 ]
  gyro  std [rad/s]: [0.00315942 0.00256786 0.00217201]

Suggested Imu message covariances (diagonal):
  linear_acceleration_covariance diag: [0.00045359 0.00045359 0.00181434]
  angular_velocity_covariance     diag: [9.98193853e-06 6.59392141e-06 4.71762310e-06]
  orientation_covariance: set [0]=-1 (unknown)

GPS (fix==1) reference:
  lat0, lon0 = 43.6452805, -79.4367445
  sats median = 5.0
  hdop median = 3.47

GPS robust std over last ~60s (meters):
  east  std [m]: 8.106
  north std [m]: 1.318
  up    std [m]: 0.890

Suggested NavSatFix position_covariance diag (m^2):
  [65.70660303  1.73676908  0.79131699]'''

class SerialImuGpsNode(Node):

    def __init__(self):
        super().__init__('serial_imu_gps')

        # Parameters (easy to override in launch files)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id_imu', 'imu_link')
        self.declare_parameter('frame_id_gps', 'gps_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}")

        self.get_logger().info(f"Reading IMU+GPS from {port} @ {baud}")

        # Start reader thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    # --------------------------------------------------
    # Serial read loop
    # --------------------------------------------------
    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if not line:
                continue

            if line.startswith('IMU,'):
                self.handle_imu(line)

            elif line.startswith('GPS,'):
                self.handle_gps(line)

    # --------------------------------------------------
    # IMU handler
    # Format:
    # IMU,ms,ax,ay,az,gx,gy,gz
    # --------------------------------------------------
    def handle_imu(self, line: str):
        try:
            _, ms, ax, ay, az, gx, gy, gz = line.split(',')
            ax = float(ax)
            ay = float(ay)
            az = float(az)
            gx = float(gx)
            gy = float(gy)
            gz = float(gz)
        except ValueError:
            return

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id_imu').value

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Orientation unknown
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        msg.orientation_covariance[0] = -1.0  # unknown orientation

        msg.linear_acceleration_covariance = [
            4.5359e-4, 0.0, 0.0,
            0.0, 4.5359e-4, 0.0,
            0.0, 0.0, 1.81434e-3
        ]

        msg.angular_velocity_covariance = [
            9.98194e-6, 0.0, 0.0,
            0.0, 6.59392e-6, 0.0,
            0.0, 0.0, 4.71762e-6
        ]
        self.imu_pub.publish(msg)

    # --------------------------------------------------
    # GPS handler
    # Format:
    # GPS,ms,lat,lon,alt,speed,hdop,sats,fix
    # --------------------------------------------------
    def handle_gps(self, line: str):
        try:
            _, ms, lat, lon, alt, speed, hdop, sats, fix = line.split(',')
            fix = int(fix)
        except ValueError:
            return

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id_gps').value

        if fix == 1:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            msg.latitude = float(lat)
            msg.longitude = float(lon)
            msg.altitude = float(alt)

            msg.position_covariance = [
                65.7066, 0.0,    0.0,
                0.0,     65.7066,0.0,
                0.0,     0.0,    25.0    # (5 m)^2
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN


        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(msg)


def main():
    rclpy.init()
    node = SerialImuGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
