# editing from antonia's gps_imu_broadcaster.py to work with current imu_gps_serial.ino 
#
## #  GPS,ms,lat,lon,alt,speed,hdop,sats,fix at 1Hz <- NO CHANGES  
#
# Note that this is publishing various topics :
#    - standard ROS2: 
#        - /gps/fix
#    - custom using foxglove (all of TextAnnotation for use on 2D panels): 
#        - /cam_overlay/latitude_longitude, 
#

import serial
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from foxglove_msgs.msg import TextAnnotation

from time import sleep # for simulated data, firmware publishes at certain Hz

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Parameters (easy to override in launch files)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200) #still matches with new arduino sketch
        self.declare_parameter('frame_id_gps', 'gps_link')
        self.declare_parameter('simulated_data', True)

        # other internal paramaters
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.serial_port = None

        # Publishers
        self.NavSatFix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.latlonfox_pub = self.create_publisher(TextAnnotation, '/cam_overlay/latitude_longitude', 10)
        
        # Start
        self.init_serial_connection()  # Start Serial, or Simulated 
        self.thread = threading.Thread(target=self.publish_data, daemon=True) # Start all the other functions
        self.thread.start()
        self.get_logger().info("gps_node.py: GPS node started!")
                 
   
    # --------------------------------------------------
    # Serial Read Try function
    # --------------------------------------------------
    def init_serial_connection(self):
        self.get_logger().info(
            f"gps_node.py: Trying to connect to serial port {self.port} at {self.baud} baud"
        )
        
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baud, timeout=1.0)
            self.get_logger().info(f"gps_node.py: Connected to {self.port} at {self.baud} baud")
        except Exception as e:
            self.get_logger().warning(f"gps_node.py: Failed to connect to {self.port}: {e}")
            if self.get_parameter('simulated_data').value:
                self.get_logger().info("gps_node.py: Using simulated data fallback")
            else:
                self.get_logger().error("gps_node.py: No GPS connection and no simulation enabled")
            
           
    # --------------------------------------------------
    # GPS handler for Simulated
    # --------------------------------------------------
    def get_simulated_data(self):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id_gps').value
        
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
        
        sleep(1) #  for 1 HZ
        
        return msg
        
     
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
    
            return msg
          
        except ValueError:
            return
            
    # --------------------------------------------------     
    # Publish Foxglove Text Annotation
    #   Just Lat and Long
    #   Takes in a NavSatFix Msg
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
        
        return foxglove_msg
         
    # --------------------------------------------------
    # Top Level MSG Publisher
    # --------------------------------------------------
    def publish_data(self):
        while rclpy.ok():
            navsatfix_msg = None
            
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.get_logger().info(
                        f"gps_node.py: Trying to obtain GPS serial read in publish_data()"
                    )
                    line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    if line.startswith('GPS,'):
                        navsatfix_msg = self.handle_gps(line)
                        self.get_logger().info(f"gps_node.py: Handling GPS serial read")
                
                except Exception:
                    self.get_logger().error(f"gps_node.py: Serial read error: {e}")
                    self.serial_port = None
                    continue

            # check for serial print first, then go for simulated data
            if navsatfix_msg:
                # publish the NavSatFix msg
                self.get_logger().info(f"gps_node.py: Publishing GPS msg - navsatfix_msg")
                self.NavSatFix_pub.publish(navsatfix_msg)

                # publish the foxglove textannotation msgs
                self.get_logger().info(f"gps_node.py: Publishing GPS foxglove msg - latlonfox_pub")
                foxglove_msg = self.handle_foxgloveGPS(navsatfix_msg)            
                self.latlonfox_pub.publish(foxglove_msg)
                
                self.get_logger().info(
                    f"Published Sample: Lat={navsatfix_msg.latitude:.6f}, Lon={navsatfix_msg.longitude:.6f}, "
                    f"Alt={navsatfix_msg.altitude:.1f}m, Status={navsatfix_msg.status.status}"
                )
                    
            if navsatfix_msg is None and self.get_parameter('simulated_data').value:
                self.get_logger().warning("gps_node.py: No serial data, using simulated data")
                navsatfix_msg = self.get_simulated_data()

            


def main():
    rclpy.init()
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
