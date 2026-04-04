# editing from antonia's gps_imu_broadcaster.py and MNTadro's imu_node.py, to work with current imu_gps_serial.ino 
#
## #  GPS,ms,lat,lon,alt,speed,hdop,sats,fix at 1Hz <- NO CHANGES  
## #  IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz at 100Hz
## # This is the "No microROS, only Serial" option
#
# Note that this is publishing various topics :
#    - standard ROS2: 
#        - /imu/data_raw
#        - /imu/mag
#        - /gps/fix
#    - custom for overlay: 
#        - /heading (as in pitch, roll, the angle of the Object away frm North)
#        - /cardinal_compass (using 8-rose i.e. N, NE, E, etc)
#    - custom using foxglove (all of TextAnnotation for use on 2D panels): 
#        - /cam_overlay/latitude_longitude, 
#        - /cam_overlay/heading 
#        - /cam_overlay/compass
#

import serial
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, Float32, String
from foxglove_msgs.msg import TextAnnotation

from math import atan2, pi as PI, sqrt     # for heading, roll, pitch i.e. "compass" angles

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
        self.declare_parameter('simulated_data', True)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.simulated_data = self.get_parameter('simulated_data').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 50)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.heading_pub = self.create_publisher(Float32, '/heading', 10) #
        self.compass_pub = self.create_publisher(String, '/cardinal_compass', 10) # N S E W

        self.latlonfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/latitude_longitude', 10)
        self.headfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/heading', 10)
        self.cardinalcompassfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/compass', 10)
                                               
        # Serial
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Reading IMU+GPS from {port} @ {baud}")
        except serial.SerialException as e:
            if self.simulated_data:
                self.get_logger().warn(
                    f"Failed to open serial port {port}: {e}. Falling back to simulated data."
                )
            else:
                raise RuntimeError(f"Failed to open serial port {port}: {e}")

        # Sim timing (IMU 100Hz, GPS 1Hz)
        now = time.monotonic()
        self._sim_next_imu = now
        self._sim_next_gps = now
        self._sim_t0_ms = int(time.time() * 1000)

        # Start reader thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    # --------------------------------------------------
    # Serial read loop
    # --------------------------------------------------
    def read_loop(self):
        while rclpy.ok():
            if self.ser is not None and self.ser.is_open:
                try:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                except Exception as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    continue

                if not line:
                    continue

                if line.startswith('IMU,'):
                    self.handle_imu(line)
                elif line.startswith('GPS,'):
                    self.handle_gps(line)

            elif self.simulated_data:
                now = time.monotonic()
                ms = int(time.time() * 1000) - self._sim_t0_ms

                if now >= self._sim_next_imu:
                    self._sim_next_imu = now + 0.01  # 100 Hz
                    imu_line = f"IMU,{ms},0.0,0.0,9.81,0.0,0.0,0.0,20.0,5.0,-40.0"
                    self.handle_imu(imu_line)

                if now >= self._sim_next_gps:
                    self._sim_next_gps = now + 1.0   # 1 Hz
                    gps_line = f"GPS,{ms},40.0150,-105.2705,1624.0,0.0,1.0,10,1"
                    self.handle_gps(gps_line)

                time.sleep(0.002)
            else:
                time.sleep(0.05)

    # --------------------------------------------------
    # IMU handler
    # Format:
    # IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz
    # 
    # Cardinal Compass and Heading Angle handler
    # Calculated from IMU
    # --------------------------------------------------
    def handle_imu(self, line: str):
        #ACCEL_CONVERSION = 0.001 # it will be coming in milli m/s^2, as it was converted from milli g's
        MAG_CONVERSION = 0.000001 # it will be coming in micro Tesla, need Tesla
                                  # tested in arduino, need to convert here to preserve precision
        try:
            _, ms, ax, ay, az, gx, gy, gz, mx, my, mz = line.split(',')
            ax = float(ax) #* ACCEL_CONVERSION
            ay = float(ay) #* ACCEL_CONVERSION
            az = float(az) #* ACCEL_CONVERSION
            gx = float(gx) # should already be radians/second
            gy = float(gy)
            gz = float(gz)
            mx = float(mx) * MAG_CONVERSION
            my = float(my) * MAG_CONVERSION
            mz = float(mz) * MAG_CONVERSION
            
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
            mag_msg.magnetic_field_covariance[0] = -1.0  # unknown orientation
            self.mag_pub.publish(mag_msg)

            # Publish Heading (commented out: pitch and roll) ================================================
            DECLINATION = 10.05 # Declination (degrees) in Boulder, CO.
            
            #roll = atan2(ay, az)
            #pitch = atan2(-ax, sqrt(ay * ay + az * az))
            
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
            
            # Convert everything from radians to degrees:
            heading *= 180.0 / PI
            #pitch *= 180.0 / PI
            #roll  *= 180.0 / PI

            heading_msg = Float32()
            heading_msg.data = heading
            self.heading_pub.publish(heading_msg)

            # Publish Foxglove Text Annotation for heading =====================
            headingFoxglove_msg = TextAnnotation()
            headingFoxglove_msg.timestamp = imu_msg.header.stamp
            headingFoxglove_msg.position.x = 360.0 # origin is top left corner of the image
            headingFoxglove_msg.position.y = 15.0
            headingFoxglove_msg.text = str(heading)
            headingFoxglove_msg.font_size = 12.0
            headingFoxglove_msg.text_color.r = 1.0
            headingFoxglove_msg.text_color.g = 1.0
            headingFoxglove_msg.text_color.b = 1.0
            headingFoxglove_msg.text_color.a = 1.0 #opaque
            headingFoxglove_msg.background_color.r = 0.0
            headingFoxglove_msg.background_color.g = 0.0
            headingFoxglove_msg.background_color.b = 0.0
            headingFoxglove_msg.background_color.a = 0.01 #0 is transparent
            self.headfox_pub.publish(headingFoxglove_msg)

            # Publish Cardinal Compass  ======================================================================
            ## will use 8-wind compass rose i.e. N NE E SE S SW W NW clockwise, 45deg each segment
            cardinal_dir = Cardinal_Direction_8(heading)

            compass_msg = String()
            compass_msg.data = cardinal_dir
            self.compass_pub.publish(compass_msg)

            # Publish Foxglove Text Annotation for cardinal compass =====================
            cardinalcompassFoxglove_msg = TextAnnotation()
            cardinalcompassFoxglove_msg.timestamp = imu_msg.header.stamp
            cardinalcompassFoxglove_msg.position.x = 360.0 # origin is top left corner of the image
            cardinalcompassFoxglove_msg.position.y = 30.0
            cardinalcompassFoxglove_msg.text = cardinal_dir
            cardinalcompassFoxglove_msg.font_size = 12.0
            cardinalcompassFoxglove_msg.text_color.r = 1.0
            cardinalcompassFoxglove_msg.text_color.g = 1.0
            cardinalcompassFoxglove_msg.text_color.b = 1.0
            cardinalcompassFoxglove_msg.text_color.a = 1.0 #opaque
            cardinalcompassFoxglove_msg.background_color.r = 0.0
            cardinalcompassFoxglove_msg.background_color.g = 0.0
            cardinalcompassFoxglove_msg.background_color.b = 0.0
            cardinalcompassFoxglove_msg.background_color.a = 0.1 #0 = transparent
            self.cardinalcompassfox_pub.publish(cardinalcompassFoxglove_msg)
                        
        except ValueError:
            return
          
        

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
    
            # Publish Foxglove Text Annotation for Lat and Long =====================
            latlonfox_msg = TextAnnotation()
            latlonfox_msg.timestamp = msg.header.stamp
            latlonfox_msg.position.x = 360.0 # origin is top left corner of the image
            latlonfox_msg.position.y = 15.0
            latlonfox_msg.text = "lat: " + str(msg.latitude) + "            " + "lon: " + str(msg.longitude) #12 spaces   
            latlonfox_msg.font_size = 12.0
            latlonfox_msg.text_color.r = 1.0
            latlonfox_msg.text_color.g = 1.0
            latlonfox_msg.text_color.b = 1.0
            latlonfox_msg.text_color.a = 1.0 #opaque
            latlonfox_msg.background_color.r = 0.0
            latlonfox_msg.background_color.g = 0.0
            latlonfox_msg.background_color.b = 0.0
            latlonfox_msg.background_color.a = 0.1 #0 = transparent
            self.latlonfox_pub.publish(latlonfox_msg)
            
        except ValueError:
            return


def main():
    rclpy.init()
    node = SerialImuGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser is not None and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
