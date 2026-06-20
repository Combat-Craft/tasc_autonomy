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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, MagneticField
#from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, Float32, String

from foxglove_msgs.msg import TextAnnotation

from math import atan2, pi as PI, sqrt  # for heading, roll, pitch i.e. "compass" angles
from time import sleep # for simulated data, firmware publishes at certain Hz

def cardinal_Direction_8(angle):
    if angle < 0.0:
        angle += 360.0
        
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
        
    return "?"
        
def get_heading_simple(mx, my): #ax, ay, az, 
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

class IMUGPSv2Node(Node):
    def __init__(self):
        super().__init__('imu_gps_node_v2')
        
        # Parameters (easy to override in launch files)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200) #still matches with new arduino sketch
        self.declare_parameter('frame_id', 'gps_imu_link')
        self.declare_parameter('simulated_data', False)

        # other internal paramaters
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.serial_port = None

        ## GPS ##################################################################
        # Publishers - standard GPS
        self.NavSatFix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        # Publishers - foxglove GPS
        self.latlonfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/latitude_longitude', 10)      
        ## IMU ##################################################################
        # Publishers - standard IMU
        self.imu_pub = self.create_publisher(Imu, '/imu/acc_gryo', 50)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 50)
        # Publishers - custom IMU
        self.heading_pub = self.create_publisher(Float32, '/heading', 10) #
        self.compass_pub = self.create_publisher(String, '/cardinal_compass', 10) # N S E W
        # Publishers - foxglove IMU
        self.headingfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/heading', 10)
        self.compassfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/compass', 10)
         
        # Start
        self.init_serial_connection()  # Start Serial, or Simulated 
        self.thread = threading.Thread(target=self.publish_data, daemon=True) # Start all the other functions
        self.thread.start()
        self.get_logger().info("imu_gps_node: IMU GPS broadcaster node started!")

    # ----------------------------------------------------------------------------------------------------
    # Serial Read Try function
    # ----------------------------------------------------------------------------------------------------
    def init_serial_connection(self):
        self.get_logger().info(f"imu_gps_node: Trying to connect to ESP32 serial port {self.port} at {self.baud} baud")  
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baud, timeout=1.0)
            self.get_logger().info(f"imu_gps_node: Connected to {self.port} at {self.baud} baud")
        except serial.SerialException as e:
            self.get_logger().warning(f"imu_gps_node: Failed to connect to {self.port} at {self.baud} baud: {e}")
            if self.get_parameter('simulated_data').value:
                self.get_logger().info(f"imu_gps_node: Using simulated data fallback")
            else:
                self.get_logger().error(f"imu_gps_node: No ESP32 serial connection and no simulated data enabled")

        self.get_logger().info(f"imu_gps_node: serial_port is currently {self.serial_port}")  
        # END init_serial_connection()
        
    # --------------------------------------------------
    # Top Level MSG Publisher
    # --------------------------------------------------
    def publish_data(self):
        while rclpy.ok():
            #serial is ok            
            if self.serial_port and self.serial_port.is_open:
                try:
                    #self.get_logger().info( f"imu_gps_node: Trying to obtain ESP32 serial read in publish_data()")
                    line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
    
                except Exception:
                    self.get_logger().error(f"imu_gps_node: Serial read error: {Exception}")
                    self.serial_port = None
                    continue
                    
                #self.get_logger().info(f"imu_gps_node: {line}")          
                if not line:
                    continue
                
                if line.startswith('GPS,'):
                    #self.get_logger().info(f"imu_gps_node: Starting to go to handle_imu() in publish_data()")
                                       
                    self.handle_gps(line)
                    
                    #self.get_logger().info(f"imu_gps_node: Handling GPS serial read")
      
                elif line.startswith("IMU,"):
                    #self.get_logger().info(f"imu_gps_node: Starting to go to handle_imu() in publish_data()")
                  
                    self.handle_imu(line)
                    
                    #self.get_logger().info(f"imu_gps_node: Handling IMU serial read")
                    
            # no serial, but we can simulate
            elif self.get_parameter('simulated_data').value:
                if navsatfix_msg is None:
                    self.get_logger().warning("imu_gps_node: No serial GPS data, using simulated data")
                    self.get_simulated_gps_data() 
                
                if imu_msg_list is None:
                    self.get_logger().warning("imu_gps_node: No serial IMU data, using simulated data")
                    self.get_simulated_imu_data()  
                
            # no serial, no simulate
            else:
                self.get_logger().warning("imu_gps_node: No serial IMU data, no simulated data")
        
    # --------------------------------------------------
    # GPS handler for Simulated
    # --------------------------------------------------
    def get_simulated_gps_data(self):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        msg.latitude = 48.8584
        msg.longitude = 2.2945
        msg.altitude = 32.0
        
        msg.position_covariance = [
                    65.7066, 0.0,    0.0,
                    0.0,     65.7066,0.0,
                    0.0,     0.0,    25.0    # (5 m)^2
                ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        #sleep(1) #  for 1 HZ
        
        self.NavSatFix_pub.publish(msg)
        
        #return msg
        
    # ----------------------------------------------------------------------------------------------------
    # IMU handler for Simulated
    # ----------------------------------------------------------------------------------------------------
    def get_simulated_imu_data(self):
        # flat face up on ground, X facing north
        imu_msg = Imu()       
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.get_parameter('frame_id').value
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
        imu_msg.orientation_covariance = [# [0] = -1.0  # unknown orientation
            1.0e-2, 0.0, 0.0,
            0.0, 1.0e-2, 0.0,
            0.0, 0.0, 1.0e-2
        ]
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
        mag_msg.header.frame_id = self.get_parameter('frame_id').value # same frame ids for both mag and imu in imu_node.py        
        mag_msg.magnetic_field.x = 0.0000711 #sample from log
        mag_msg.magnetic_field.y = 0.00013935
        mag_msg.magnetic_field.z = 0.0001338
        mag_msg.magnetic_field_covariance = [
            1.0e-2, 0.0, 0.0,
            0.0, 1.0e-2, 0.0,
            0.0, 0.0, 1.0e-2
        ]
        
        heading_msg = Float32()
        heading_msg.data = get_heading_simple(mag_msg.magnetic_field.x, mag_msg.magnetic_field.y)
        
        compass_msg = String()
        compass_msg.data = cardinal_Direction_8(heading_msg.data)
        
        #sleep(0.1) # should be 0.01 for 100HZ, but for simulation lets ease off for testing
        #testList = [imu_msg, mag_msg, heading_msg, compass_msg]    
        
        #return testList
        
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        self.heading_pub.publish(heading_msg)
        self.compass_pub.publish(compass_msg) 
        
        self.handle_foxgloveHeading(heading_msg, imu_msg.header.stamp)  
        self.handle_foxgloveCardinalCompass(compass_msg, imu_msg.header.stamp) 
        
    # --------------------------------------------------
    # GPS handler for Serial
    #   Format: GPS,ms,lat,lon,alt,speed,hdop,sats,fi
    # --------------------------------------------------
    def handle_gps(self, line: str):
        try:
            _, ms, lat, lon, alt, speed, hdop, sats, fix = line.split(',')
            fix = int(fix)
        
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value
    
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
    
            
            self.handle_foxgloveGPS(msg)   
            
            self.NavSatFix_pub.publish(msg)
            #return msg
          
        except ValueError:
            self.get_logger().error(f"imu_node.py: Failed try in handle_gps()")
            #return
    
    # --------------------------------------------------
    # IMU handler
    #   Format: IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz
    # 
    # Cardinal Compass and Heading Angle handler
    #   Calculated from IMU
    # --------------------------------------------------
    def handle_imu(self, line: str):
        # ACCEL_CONVERSION = 0.001 # FastIMU outputs Gs, convert to m/s^2, UPDATE: done in firmware
        # GYRO_CONVERSION = PI / 180.0 # FastIMU outputs deg/s, convert to rad/s, UPDATE: done in firmware
        MAG_CONVERSION = 0.000001 # it will be coming in micro Tesla, need Tesla
                                  # tested in arduino, need to convert here to preserve precision
        try:
            _, ms, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz = line.split(',')
            ax = float(ax) # * ACCEL_CONVERSION
            ay = float(ay) 
            az = float(az) 
            gx = float(gx) # should already be radians/second
            gy = float(gy)
            gz = float(gz)
            mx = float(mx) * MAG_CONVERSION
            my = float(my) * MAG_CONVERSION
            mz = float(mz) * MAG_CONVERSION
            qw = float(qw)
            qx = float(qx)
            qy = float(qy)
            qz = float(qz)        
            #self.get_logger().info(f"imu_node.py: Line split success!!")
                 
            # IMU data ==============================================================================
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.get_parameter('frame_id').value

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
         
            imu_msg.orientation.w = qw 
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            
            imu_msg.orientation_covariance = [ # -1.0  # unknown orientation
                1.0e-2, 0.0, 0.0,
                0.0, 1.0e-2, 0.0,
                0.0, 0.0, 1.0e-2
            ]
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
            
            #self.get_logger().info(
            #                f"Acc X,Y,Z ={imu_msg.linear_acceleration.x}, {imu_msg.linear_acceleration.y}, {imu_msg.linear_acceleration.z},"
            #                f"Gryo X,Y,Z ={imu_msg.angular_velocity.x}, {imu_msg.angular_velocity.y}, {imu_msg.angular_velocity.z},"
            #        )
            
            # Magnetometer data ==========================================================================
            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp # let's use same timestamp here
            mag_msg.header.frame_id = self.get_parameter('frame_id').value # same frame ids for both mag and imu in imu_node.py
          
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz
            mag_msg.magnetic_field_covariance = [ #[0] = -1.0  # unknown orientation
                1.0e-2, 0.0, 0.0,
                0.0, 1.0e-2, 0.0,
                0.0, 0.0, 1.0e-2
            ]
            
            #self.get_logger().info(f"Mag X,Y,Z ={mag_msg.magnetic_field.x}, {mag_msg.magnetic_field.y}, {mag_msg.magnetic_field.z}")

            # Heading Calculation ========================================================================
            heading_msg = Float32()
            heading_msg.data = get_heading_simple(mx, my) # in 360deg
            
            # Cardinal Compass  ======================================================================
            ## will use 8-wind compass rose i.e. N NE E SE S SW W NW clockwise, 45deg each segment
            compass_msg = String()
            compass_msg.data = cardinal_Direction_8(heading_msg.data) # i.e.N NE E SE S SW W NW
            
            self.get_logger().info(f"Heading={heading_msg.data:.1f}, Compass={compass_msg.data}")           
            
            self.imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)
            self.heading_pub.publish(heading_msg)
            self.compass_pub.publish(compass_msg) 
            
            self.handle_foxgloveHeading(heading_msg, imu_msg.header.stamp)  
            self.handle_foxgloveCardinalCompass(compass_msg, imu_msg.header.stamp)   
                       
        except ValueError:
            self.get_logger().error(f"imu_node.py: Failed try in handle_imu()")
            #return
    
    # --------------------------------------------------     
    # Foxglove Text Annotation

    # ... looking at this, i should refactor since repeat code but later...
    # -------------------------------------------------- 
    def handle_foxgloveGPS(self, msg):

        foxglove_msg = TextAnnotation()
        foxglove_msg.timestamp =  msg.header.stamp
        foxglove_msg.position.x = 360.0 # origin is top left corner of the image
        foxglove_msg.position.y = 15.0
        foxglove_msg.text = "lat: " + str(msg.latitude) + "            " + "lon: " + str(msg.longitude) #12 spaces   
        foxglove_msg.font_size = 12.0
        foxglove_msg.text_color.r = 1.0
        foxglove_msg.text_color.g = 1.0
        foxglove_msg.text_color.b = 1.0
        foxglove_msg.text_color.a = 1.0 #opaque
        foxglove_msg.background_color.r = 0.0
        foxglove_msg.background_color.g = 0.0
        foxglove_msg.background_color.b = 0.0
        foxglove_msg.background_color.a = 0.1 #0 = transparent   
        
        self.latlonfox_pub.publish(foxglove_msg)     

    
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
        
        self.headingfox_pub.publish(foxglove_msg)               

        
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
        
        self.compassfox_pub.publish(foxglove_msg) 
   

def main():
    rclpy.init()
    node = IMUGPSv2Node()
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
