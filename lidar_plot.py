import math
import matplotlib.pyplot as plt
from rplidar import RPLidar

# ------------------------------------------------------------------
# SETUP: Use the port you found earlier
# ------------------------------------------------------------------
PORT_NAME = '/dev/tty.usbserial-0001'

def run():
    lidar = RPLidar(PORT_NAME)
    
    # Setup the plot
    plt.ion() # Interactive mode on
    fig, ax = plt.subplots()
    
    print("Scanning... (Press Ctrl+C to stop)")
    
    try:
        # iter_scans() collects a full 360-degree rotation before yielding
        for scan in lidar.iter_scans():
            
            x_points = []
            y_points = []
            
            # Process one full rotation
            for (_, angle, distance) in scan:
                if distance > 0:
                    # Convert to Radians
                    angle_rad = math.radians(angle)
                    
                    # Convert Polar to Cartesian
                    x = distance * math.cos(angle_rad)
                    y = distance * math.sin(angle_rad)
                    
                    x_points.append(x)
                    y_points.append(y)
            
            # Clear the previous frame
            ax.clear()
            
            # Plot the new points (Red dots, size 2)
            ax.scatter(x_points, y_points, s=2, c='red')
            
            # Draw the "Robot" at the center (0,0)
            ax.plot(0, 0, 'bo', markersize=5) 
            
            # Set fixed limits so the "map" doesn't jump around
            # (adjust these values based on your room size, e.g. +/- 2000mm)
            ax.set_xlim(-3000, 3000)
            ax.set_ylim(-3000, 3000)
            ax.grid(True)
            
            # Refresh the plot
            plt.draw()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print("Stopping...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        plt.close()

if __name__ == '__main__':import math
import matplotlib.pyplot as plt
from rplidar import RPLidar

# ------------------------------------------------------------------
# SETUP: Use the port you found earlier
# ------------------------------------------------------------------
PORT_NAME = '/dev/tty.usbserial-0001'

def run():
    lidar = RPLidar(PORT_NAME)
    
    # Setup the plot
    plt.ion() # Interactive mode on
    fig, ax = plt.subplots()
    
    print("Scanning... (Press Ctrl+C to stop)")
    
    try:
        # iter_scans() collects a full 360-degree rotation before yielding
        for scan in lidar.iter_scans():
            
            x_points = []
            y_points = []
            
            # Process one full rotation
            for (_, angle, distance) in scan:
                if distance > 0:
                    # Convert to Radians
                    angle_rad = math.radians(angle)
                    
                    # Convert Polar to Cartesian
                    x = distance * math.cos(angle_rad)
                    y = distance * math.sin(angle_rad)
                    
                    x_points.append(x)
                    y_points.append(y)
            
            # Clear the previous frame
            ax.clear()
            
            # Plot the new points (Red dots, size 2)
            ax.scatter(x_points, y_points, s=2, c='red')
            
            # Draw the "Robot" at the center (0,0)
            ax.plot(0, 0, 'bo', markersize=5) 
            
            # Set fixed limits so the "map" doesn't jump around
            # (adjust these values based on your room size, e.g. +/- 2000mm)
            ax.set_xlim(-3000, 3000)
            ax.set_ylim(-3000, 3000)
            ax.grid(True)
            
            # Refresh the plot
            plt.draw()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print("Stopping...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        plt.close()

if __name__ == '__main__':
    run()
    run()