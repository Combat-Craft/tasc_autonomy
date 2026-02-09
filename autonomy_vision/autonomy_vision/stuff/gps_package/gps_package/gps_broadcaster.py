import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus

import serial
import pynmea2


class SerialGPSNode(Node):
    def __init__(self):
        super().__init__('gps_serial_node')

        # Publisher
        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

        # Serial port (adjust if needed)
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1.0
        )

        # Timer: GPS is low-rate, 2–5 Hz is plenty
        self.timer = self.create_timer(0.2, self.read_gps)

        self.get_logger().info('GPS serial node started')

    def read_gps(self):
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not line.startswith('$'):
                return

            msg = pynmea2.parse(line)

            # We care mainly about GGA (fix + altitude)
            # if isinstance(msg, pynmea2.types.talker.GGA):
            #     nav = NavSatFix()
            #     nav.header.stamp = self.get_clock().now().to_msg()
            #     nav.header.frame_id = 'gps_link'

            #     if msg.gps_qual and int(msg.gps_qual) > 0:
            #         nav.status.status = NavSatStatus.STATUS_FIX
            #         nav.latitude = msg.latitude
            #         nav.longitude = msg.longitude
            #         nav.altitude = float(msg.altitude)
            #     else:
            #         nav.status.status = NavSatStatus.STATUS_NO_FIX
            #         nav.latitude = 0.0
            #         nav.longitude = 0.0
            #         nav.altitude = 0.0

            #     nav.status.service = (
            #         NavSatStatus.SERVICE_GPS |
            #         NavSatStatus.SERVICE_GLONASS |
            #         NavSatStatus.SERVICE_GALILEO
            #     )

            #     self.pub.publish(nav)
            if isinstance(msg, pynmea2.types.talker.GGA):
                nav = NavSatFix()
                nav.header.stamp = self.get_clock().now().to_msg()
                nav.header.frame_id = 'gps_link'

                if msg.gps_qual and int(msg.gps_qual) > 0:
                    nav.status.status = NavSatStatus.STATUS_FIX
                    nav.latitude = msg.latitude
                    nav.longitude = msg.longitude
                    nav.altitude = float(msg.altitude)

                    # --- covariance ---
                    try:
                        hdop = float(msg.horizontal_dil)
                        sigma_uere = 2.0  # meters
                        var_xy = (hdop * sigma_uere) ** 2
                    except:
                        var_xy = 100.0

                    var_z = var_xy * 4.0

                    nav.position_covariance = [
                        var_xy, 0.0,   0.0,
                        0.0,   var_xy, 0.0,
                        0.0,   0.0,   var_z
                    ]

                    nav.position_covariance_type = (
                        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    )

                else:
                    nav.status.status = NavSatStatus.STATUS_NO_FIX

                nav.status.service = (
                    NavSatStatus.SERVICE_GPS |
                    NavSatStatus.SERVICE_GLONASS |
                    NavSatStatus.SERVICE_GALILEO
                )

                self.pub.publish(nav)


        except pynmea2.ParseError:
            pass
        except Exception as e:
            self.get_logger().warn(f"GPS error: {e}")


def main():
    rclpy.init()
    node = SerialGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
