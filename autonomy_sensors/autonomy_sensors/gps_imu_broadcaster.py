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

from math import atan2, PI, sqrt     # for heading, roll, pitch i.e. "compass" angles

def Cardinal_Direction_8(angle):
    if ((0 <= angle <= 22.5) or angle > 337.5):
        return "N"
    if (22.5 < angle <= 67.5):
        return "NE"
    if (67.5 < angle <= 112.5):
        return "E"
    if (112.5 < angle <= 157.5):
        return "SE"
    if (157.5 < angle <= 202.5):
        return "S"
    if (202.5 < angle <= 247.5):
        return "SW"
    if (247.5 < angle <= 292.5):
        return "W"
    if (292.5 < angle <= 337.5):
        return "NW"

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
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 50)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.heading_pub = self.create_publisher(Float32, '/heading', 10) #
        self.compass_pub = self.create_publisher(String, '/cardinal_compass', 10) # N S E W
                                               
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
    #
    # Cardinal Compass and Heading Angle handler
    # Calculated from IMU
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
          
        # Publish IMU data ==============================================================================
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

        # Publish magnetometer ===========================================================================
        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp # let's use same timestamp here
        mag_msg.header.frame_id = self.get_parameter('frame_id_imu').value # same frame ids for both mag and imu in imu_node.py
      
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
            
        self.mag_pub.publish(mag_msg)

        # Publish Heading (commented out: pitch and roll) ================================================
        DECLINATION 10.05 # Declination (degrees) in Boulder, CO.
        
        //roll = atan2(ay, az)
        //pitch = atan2(-ax, sqrt(ay * ay + az * az))
        
        heading = 0
        if (my == 0):
            # C++ code is heading = (mx < 0) ? PI : 0;
            if (mx < 0):
                heading = PI
            else:
                heading = 0
        else:
            heading = atan2(mx, my)
        
        heading -= DECLINATION * PI / 180
        
        if (heading > PI):
            heading -= (2 * PI)
        elif (heading < -PI):
            heading += (2 * PI)
        
        // Convert everything from radians to degrees:
        heading *= 180.0 / PI
        //pitch *= 180.0 / PI
        //roll  *= 180.0 / PI

        heading_msg = Float32()
        heading_msg.data = heading
        self.heading_pub.publish(heading_msg)

        # Publish Cardinal Compass  ======================================================================
        ## will use 8-wind compass rose i.e. N NE E SE S SW W NW clockwise, 45deg each segment
        cardinal_dir = Cardinal_Direction_8(heading)

        compass_msg = String()
        compass_msg.data = heading
        self.compass_pub.publish(compass_msg)

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
