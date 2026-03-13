# editing from antonia's gps_imu_broadcaster.py and MNTadro's imu_node.py, to work with current imu_gps_serial.ino 
#
## #  GPS,ms,lat,lon,alt,speed,hdop,sats,fix at 1Hz <- NO CHANGES  
## #  IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz at 100Hz
## # This is the "No microROS, only Serial" option

import serial
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

class SerialImuGpsNode(Node):

    def __init__(self):
        super().__init__('serial_imu_gps')

        # Parameters (easy to override in launch files)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200) #still matches with new arduino sketch
        self.declare_parameter('frame_id_imu', 'imu_link')
        self.declare_parameter('frame_id_gps', 'gps_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', qos)
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
    # IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz
    # --------------------------------------------------
    def handle_imu(self, line: str):
        try:
            _, ms, ax, ay, az, gx, gy, gz, mx, my, mz = line.split(',')
            ax = float(ax)
            ay = float(ay)
            az = float(az)
            gx = float(gx)
            gy = float(gy)
            gz = float(gz)
            mx = float(mx)
            my = float(my)
            mz = float(mz)
        except ValueError:
            return
          
        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.get_parameter('frame_id_imu').value

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        ## -- Orientation unknown --
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        
        imu_msg.orientation_covariance[0] = -1.0  # unknown orientation
        imu_msg.linear_acceleration_covariance = [
            4.5359e-4, 0.0, 0.0,
            0.0, 4.5359e-4, 0.0,
            0.0, 0.0, 1.81434e-3
        ]
        imu_msg.angular_velocity_covariance = [
            9.98194e-6, 0.0, 0.0,
            0.0, 6.59392e-6, 0.0,
            0.0, 0.0, 4.71762e-6
        ]
        self.imu_pub.publish(imu_msg)

        # Publish magnetometer
        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp # let's use same timestamp here
        mag_msg.header.frame_id = self.get_parameter('frame_id_imu').value # same frame ids for both mag and imu in imu_node.py
      
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
            
        self.mag_pub.publish(mag_msg)

        # END handle_imu()

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
