import math
from rplidar import RPLidar

# We found this from your terminal output:
PORT_NAME = '/dev/tty.usbserial-0001'

def run():
    # Connect to the Lidar
    lidar = RPLidar(PORT_NAME)
    print(f"Connected to {PORT_NAME}...")
    print("Streaming coordinates (Press Ctrl+C to stop):")
    
    try:
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                if distance > 0:
                    # Convert raw data to coordinates
                    angle_rad = math.radians(angle)
                    x = distance * math.cos(angle_rad)
                    y = distance * math.sin(angle_rad)
                    
                    print(f"Angle: {angle:.1f}° | Dist: {distance:.0f}mm | X: {x:.1f}, Y: {y:.1f}")
                    
    except KeyboardInterrupt:
        print("\nStopping...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == '__main__':
    run()