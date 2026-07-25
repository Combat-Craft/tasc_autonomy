# editing from antonia's gps_imu_broadcaster.py and MNTadro's imu_node.py, to work with current imu_gps_fastimu.ino 
#
## #  GPS,ms,lat,lon,alt,speed,hdop,sats,fix at 1Hz <- NO CHANGES  
## #  IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz at 100Hz
## # This is the "No microROS, only Serial" option
#
# Note that this is publishing various topics :
#    - standard ROS2: 
#        - /imu/data_raw
#        - /imu/mag
#        - /gps/fix
#    - custom for compass, used in panorama: 
#        - /heading (as in pitch, roll, the angle of the Object away frm North)
#        - /cardinal_compass (using 8-rose i.e. N, NE, E, etc)
#

import serial
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, MagneticField
#from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, Float32, String


from math import atan2, pi as PI, sqrt  # for heading, roll, pitch i.e. "compass" angles
from time import sleep # for simulated data, firmware publishes at certain Hz

def get_cardinal_16(heading_angle):
    # alg from https://github.com/mikalhart/TinyGPSPlus/blob/master/src/TinyGPS%2B%2B.cpp
    # 360/16 = 22.5, and we want True North to be 0deg +/- (22.5/2)
    
    cardinal = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    direction = (heading_angle + 11.25) / 22.5;
    return cardinal[direction % 16];
        

def get_heading_quat(qw, qx, qy, qz): 

    # sparkfun for Tait Bryan angles: heading/ψ is rotation about the Z-axis (+/- 180 deg.)
    # assumes the NED orientation for quaternions
    heading = atan2( 2.0*(qw*qz + qy*qx), (1.0 - 2.0*(qy*qy + qz*qz)) ) * 180.0 / PI;
        
    if heading < 0.0:
        heading += 360
    elif heading > 360.0:
        heading -= 360
    
    return heading

class IMUGPSNode(Node):
    def __init__(self):
        super().__init__('imu_gps_node')
        
        # Parameters (easy to override in launch files)
        self.declare_parameter('port', '/dev/autonavESP32')
        self.declare_parameter('baud', 115200) #still matches with new arduino sketch
        self.declare_parameter('frame_id', 'gps_imu_link') # match IMU, gps can be off by 5cm for all i care
        self.declare_parameter('simulated_data', False)

        # other internal paramaters
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.serial_port = None
                
        # Publishers
        ## GPS 
        self.NavSatFix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        self.headingGPS_pub = self.create_publisher(Float32, '/heading', 10) #
        self.compassGPS_pub = self.create_publisher(String, '/cardinal_compass', 10) # N S E W
              
        ## IMU 
        self.imu_pub = self.create_publisher(Imu, '/imu/acc_gyro', 50)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 50)

        #self.heading_pub = self.create_publisher(Float32, '/heading', 10) #
        #self.compass_pub = self.create_publisher(String, '/cardinal_compass', 10) # N S E W
        
        
        # Start Node
        self.init_serial_connection()  # Start Serial, or Simulated 
        self.thread = threading.Thread(target=self.publish_data, daemon=True) # Start all the other functions
        self.thread.start()
        self.get_logger().info("imu_gps_node: GPS IMU node started!")

    # ----------------------------------------------------------------------------------------------------
    # Serial Read Try function
    # ----------------------------------------------------------------------------------------------------
    def init_serial_connection(self):
        self.get_logger().info(
            f"imu_gps_node: Trying to connect to ESP32 serial port {self.port} at {self.baud} baud"
        )      
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baud, timeout=1.0)
            self.get_logger().info(
                f"imu_gps_node: Connected to {self.port} at {self.baud} baud"
            )
        except Exception as e:
            self.get_logger().warning(
                f"imu_gps_node: Failed to connect to {self.port} at {self.baud} baud: {e}"
            )
            if self.get_parameter('simulated_data').value:
                self.get_logger().info(
                    f"imu_gps_node: Using simulated data fallback"
                )
            else:
                self.get_logger().error(
                    f"imu_gps_node: No ESP32 serial connection and no simulated data enabled"
                )
        # END init_serial_connection()
        
    # --------------------------------------------------
    # Top Level MSG Publisher - But going to Publish within handle_XXX() functions
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
                self.get_logger().warning("imu_gps_node: No serial IMU data, no simulated data, restarting serial connections")
                self.init_serial_connection() 
          
       
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
        
        heading_msg = Float32()
        heading_msg.data = 1
        compass_msg = String()
        compass_msg.data = "N"
        self.heading_pub.publish(heading_msg)
        self.compass_pub.publish(compass_msg) 

        #sleep(1) #  for 1 HZ
        
        self.NavSatFix_pub.publish(msg)
        
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
        
        sleep(0.1) # should be 0.01 for 100HZ, but for simulation lets ease off for testing
        
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        
        
    # --------------------------------------------------
    # GPS handler for Serial
    #   Format: GPS,ms,lat,lon,alt,speed,hdop,sats,fi
    # --------------------------------------------------
    def handle_gps(self, line: str):
        try:
            _, ms, lat, lon, alt, speed, hdop, sats, fix, heading, cardinal = line.split(',')
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
                   
            self.NavSatFix_pub.publish(msg)
                          
            # Heading Calculation ========================================================================
            heading_msg = Float32()
            heading_msg.data = float(heading)
            
            # Cardinal Compass  ======================================================================
            ## will use 8-wind compass rose i.e. N NE E SE S SW W NW clockwise, 45deg each segment
            compass_msg = String()
            compass_msg.data = cardinal
            
            #self.get_logger().info(f"GPS Heading={heading_msg.data:.1f}, GPS Compass={compass_msg.data}")           
            
            self.headingGPS_pub.publish(heading_msg)
            self.compassGPS_pub.publish(compass_msg)  
          
        except ValueError:
            self.get_logger().error(f"GPS_imu_node: Failed try in handle_gps()")
           
    
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
        #self.get_logger().info(line)
        try:
            _, ms, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz = line.split(',')
            ax = float(ax) # * ACCEL_CONVERSION
            ay = float(ay) 
            az = float(az) 
            #self.get_logger().info(f"imu_node.py: acc")

            gx = float(gx) # should already be radians/second
            gy = float(gy)
            gz = float(gz)
            #self.get_logger().info(f"imu_node.py: gyro")
            mx = float(mx) * MAG_CONVERSION
            my = float(my) * MAG_CONVERSION
            mz = float(mz) * MAG_CONVERSION
            #self.get_logger().info(f"imu_node.py: mag")
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
            #                f"Gyro X,Y,Z ={imu_msg.angular_velocity.x}, {imu_msg.angular_velocity.y}, {imu_msg.angular_velocity.z},"
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

            # test gps with quat
             # Heading Calculation ========================================================================
            #heading_msg = Float32()
            #heading_msg.data = get_heading_quat(qw, qx, qy, qz)
            
            # Cardinal Compass  ======================================================================
            ## will use 8-wind compass rose i.e. N NE E SE S SW W NW clockwise, 45deg each segment
            #compass_msg = String()
            #compass_msg.data = get_cardinal_16(heading_msg.data)
            
            #self.get_logger().info(f"IMU Heading={heading_msg.data:.1f}, IMU Compass={compass_msg.data}")           
            
            #self.heading_pub.publish(heading_msg)
            #self.compass_pub.publish(compass_msg) 
            
            self.imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)
            
                       
        except ValueError:
            self.get_logger().error(f"imu_node.py: Failed try in handle_imu()")
    

def main():
    rclpy.init()
    node = IMUGPSNode()
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
