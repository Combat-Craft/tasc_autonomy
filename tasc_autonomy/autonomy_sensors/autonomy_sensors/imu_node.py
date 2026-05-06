# editing from gps_imu_broadcaster.py to work with current imu_gps_serial.ino 
#
## #  GPS,ms,lat,lon,alt,speed,hdop,sats,fix at 1Hz <- Changed to fit format in nova
#
# Note that this is publishing various topics :
#    - standard ROS2: 
#        - /imu/data_raw
#        - /imu/mag
#    - custom for overlay: 
#        - /heading (as in pitch, roll, the angle of the Object away frm North)
#        - /cardinal_compass (using 8-rose i.e. N, NE, E, etc)
#    - custom using foxglove (all of TextAnnotation for use on 2D panels): 
#        - /cam_overlay/heading 
#        - /cam_overlay/compass
#

import serial
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
#from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, Float32, String
from foxglove_msgs.msg import TextAnnotation

from math import atan2, pi as PI, sqrt     # for heading, roll, pitch i.e. "compass" angles
from time import sleep # for simulated data, firmware publishes at certain Hz


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters (easy to override in launch files)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200) #still matches with new arduino sketch
        self.declare_parameter('frame_id_imu', 'imu_link')
        self.declare_parameter('simulated_data', True)

        # other internal paramaters
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.serial_port = None

        # Publishers - standard
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 50)
        # Publishers - custom
        self.heading_pub = self.create_publisher(Float32, '/heading', 10) #
        self.compass_pub = self.create_publisher(String, '/cardinal_compass', 10) # N S E W
        # Publishers - foxglove
        self.headingfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/heading', 10)
        self.compassfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/compass', 10)
         
        # Start
        self.init_serial_connection()  # Start Serial, or Simulated 
        self.thread = threading.Thread(target=self.publish_data, daemon=True) # Start all the other functions
        self.thread.start()
        self.get_logger().info("imu_node.py: IMU node started!")
                 
   
    # --------------------------------------------------
    # Serial Read Try function
    # --------------------------------------------------
    def init_serial_connection(self):
        self.get_logger().info(f"imu_node.py: Trying to connect to serial port {self.port} at {self.baud} baud")
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baud, timeout=1.0)
            self.get_logger().info(f"imu_node.py: Connected to {self.port} at {self.baud} baud")
        except Exception as e:
            self.get_logger().warning(f"imu_node.py: Failed to connect to {self.port}: {e}")
            if self.get_parameter('simulated_data').value:
                self.get_logger().info("imu_node.py: Using simulated data fallback")
            else:
                self.get_logger().error("imu_node.py: No GPS connection and no simulation enabled")
            
           
    # --------------------------------------------------
    # IMU handler for Simulated
    # --------------------------------------------------
    def get_simulated_data(self):
        # flat face up on ground, X facing north
        imu_msg = Imu()       
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.get_parameter('frame_id_imu').value
        imu_msg.linear_acceleration.x = -0.0647
        imu_msg.linear_acceleration.y = 0.0125
        imu_msg.linear_acceleration.z = 9.8055
        imu_msg.angular_velocity.x = 0.001 #flat on ground
        imu_msg.angular_velocity.y = 0.001
        imu_msg.angular_velocity.z = 0.001
        imu_msg.orientation.w = 1.0 # Orientation unknown 
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
        
        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp # let's use same timestamp here
        mag_msg.header.frame_id = self.get_parameter('frame_id_imu').value # same frame ids for both mag and imu in imu_node.py        
        mag_msg.magnetic_field.x = 0.0000711 #sample from log
        mag_msg.magnetic_field.y = 0.00013935
        mag_msg.magnetic_field.z = 0.0001338
        mag_msg.magnetic_field_covariance[0] = -1.0  # unknown orientation
        
        heading_msg = Float32()
        heading_msg.data = 0.1 #not true calc, but should be north'ish
        
        compass_msg = String()
        compass_msg.data = "N"
        
        sleep(0.1) # should be 0.01 for 100HZ, but for simulation lets ease off for testing
        
        return [imu_msg, mag_msg, heading_msg, compass_msg]
        
        
        
    # --------------------------------------------------
    # IMU helper functions:
    #    Cardinal_Direction_8(angle)
    #    get_heading_simple()
    # --------------------------------------------------
    def Cardinal_Direction_8(self, angle):
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
      
    def get_heading_simple(self, mx, my): #ax, ay, az, 
        #has the code for roll and yaw, just in case
        
        DECLINATION = 10.05 # Declination (degrees) in Toronto
        
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

        return heading
        
     
    # --------------------------------------------------
    # IMU handler
    #   Format: IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz
    # 
    # Cardinal Compass and Heading Angle handler
    #   Calculated from IMU
    # --------------------------------------------------
    def handle_imu(self, line: str):
        #ACCEL_CONVERSION = 0.001 # it will be coming in milli m/s^2, as it was converted from milli g's
        MAG_CONVERSION = 0.000001 # it will be coming in micro Tesla, need Tesla
                                  # tested in arduino, need to convert here to preserve precision
        try:
            _, ms, ax, ay, az, gx, gy, gz, mx, my, mz = line.split(',')
            ax = float(ax) # * ACCEL_CONVERSION
            ay = float(ay) 
            az = float(az) 
            gx = float(gx) # should already be radians/second
            gy = float(gy)
            gz = float(gz)
            mx = float(mx) * MAG_CONVERSION
            my = float(my) * MAG_CONVERSION
            mz = float(mz) * MAG_CONVERSION
        
          
            # IMU data ==============================================================================
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
         
            imu_msg.orientation.w = 1.0 ## -- Orientation unknown --
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
            
            # Magnetometer data ==========================================================================
            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp # let's use same timestamp here
            mag_msg.header.frame_id = self.get_parameter('frame_id_imu').value # same frame ids for both mag and imu in imu_node.py
          
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz
            mag_msg.magnetic_field_covariance[0] = -1.0  # unknown orientation

            # Heading Calculation ========================================================================
            heading_msg = Float32()
            heading_msg.data = get_heading_simple(mx, my) # in 360deg
            
            # Cardinal Compass  ======================================================================
            ## will use 8-wind compass rose i.e. N NE E SE S SW W NW clockwise, 45deg each segment
            compass_msg = String()
            compass_msg.data = Cardinal_Direction_8(heading) # i.e.N NE E SE S SW W NW
            
            return [imu_msg, mag_msg, heading_msg, compass_msg]
            
        except ValueError:
            return

            
    # --------------------------------------------------     
    # Foxglove Text Annotation
    #   Just Lat and Long
    #   Takes in a NavSatFix Msg
    #
    # ... looking at this, i should refactor since repeat code but later...
    # -------------------------------------------------- 
    def handle_foxgloveHeading(self, msg, timestamp):
        foxglove_msg = TextAnnotation()
        foxglove_msg.timestamp =  timestamp
        foxglove_msg.position.x = 360.0 # origin is top left corner of the image
        foxglove_msg.position.y = 15.0
        foxglove_msg.text = "Heading: " + str(msg.data)
        foxglove_msg.font_size = 12.0
        foxglove_msg.text_color.r = 1.0
        foxglove_msg.text_color.g = 1.0
        foxglove_msg.text_color.b = 1.0
        foxglove_msg.text_color.a = 1.0 #opaque
        foxglove_msg.background_color.r = 0.0
        foxglove_msg.background_color.g = 0.0
        foxglove_msg.background_color.b = 0.0
        foxglove_msg.background_color.a = 0.1 #0 = transparent
             
        return foxglove_msg
        
    def handle_foxgloveCardinalCompass(self, msg, timestamp):

        foxglove_msg = TextAnnotation()
        foxglove_msg.timestamp =  timestamp 
        foxglove_msg.position.x = 360.0 # origin is top left corner of the image
        foxglove_msg.position.y = 30.0
        foxglove_msg.text = msg.data #already string
        foxglove_msg.font_size = 12.0
        foxglove_msg.text_color.r = 1.0
        foxglove_msg.text_color.g = 1.0
        foxglove_msg.text_color.b = 1.0
        foxglove_msg.text_color.a = 1.0 #opaque
        foxglove_msg.background_color.r = 0.0
        foxglove_msg.background_color.g = 0.0
        foxglove_msg.background_color.b = 0.0
        foxglove_msg.background_color.a = 0.1 #0 = transparent
             
        return foxglove_msg
         
    # --------------------------------------------------
    # Top Level MSG Publisher
    # --------------------------------------------------
    def publish_data(self):
        while rclpy.ok():
            msg_list = None 
            
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.get_logger().info(f"imu_node.py: Trying to obtain IMU serial read in publish_data()")
                    line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    if line.startswith('IMU,'):
                        msg_list = self.handle_imu(line)
                        self.get_logger().info(f"imu_node.py: Handling IMU serial read")

                except Exception as e:
                    self.get_logger().error(f"imu_node.py: Serial read error: {e}")
                    self.serial_port = None
                    continue

            # check for serial print first, then go for simulated data
            if msg_list:
                # grab the msgs out of list
                imu_msg, mag_msg, heading_msg, compass_msg = msg_list 
                
                # publish the ros2 msgs
                self.get_logger().info(
                    f"imu_node.py: Publishing IMU msgs - imu_msg, mag_msg, heading_msg, compass_msg"
                )
                self.imu_pub.publish(imu_msg)
                self.mag_pub.publish(mag_msg)
                self.heading_pub.publish(heading_msg)
                self.compass_pub.publish(compass_msg)
                
                #publish the foxglove textannotation msgs
                self.get_logger().info(
                    f"imu_node.py: Publishing IMU foxglove msgs - headingfoxglove_msg, compassfoxglove_msg"
                )
                headingfoxglove_msg = self.handle_foxgloveHeading(heading_msg, imu_msg.header.stamp) 
                self.headingfox_pub.publish(headingfoxglove_msg)               
                compassfoxglove_msg = self.handle_foxgloveCardinalCompass(compass_msg, imu_msg.header.stamp)
                self.compassfox_pub.publish(compassfoxglove_msg)

                
                self.get_logger().info(
                    f"Published Sample: Heading={heading_msg.data:.1f}, Compass={compass_msg.data} "
                )
                
            elif msg_list is None and self.get_parameter('simulated_data').value:
                self.get_logger().warning("imu_node.py: No serial data, using simulated data")
                msg_list = self.get_simulated_data()


def main():
    rclpy.init()
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
