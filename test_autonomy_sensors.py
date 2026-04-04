#!/usr/bin/env python3
"""
Test script for autonomy_sensors nodes with simulated data.

Launches:
  - imu_node (simulated IMU)
  - gps_node (simulated GPS)

Verifies topics are publishing and displays sample data.
"""

import subprocess
import time
import sys
import signal
import os


def run_command(cmd, description):
    """Run shell command capturing output."""
    print(f"\n[TEST] Starting: {description}")
    print(f"  Command: {cmd}\n")
    proc = subprocess.Popen(
        cmd, 
        shell=True, 
        stdout=subprocess.PIPE, 
        stderr=subprocess.STDOUT, 
        text=True, 
        bufsize=1,
        executable="/bin/bash"
    )
    return proc


def print_output(proc, max_lines=20):
    """Print output from process."""
    lines = 0
    try:
        while lines < max_lines:
            line = proc.stdout.readline()
            if not line:
                break
            print(f"  {line.rstrip()}")
            lines += 1
    except:
        pass


def verify_topic(topic_name, timeout=5):
    """Verify topic has publishers and echo sample."""
    print(f"\n[VERIFY] Checking topic: {topic_name}")
    
    # Check topic info
    info_cmd = f"source /opt/ros/$ROS_DISTRO/setup.bash && source /home/mnt/Desktop/tasc_autonav_combined/install/setup.bash && ros2 topic info {topic_name} 2>&1"
    result = subprocess.run(info_cmd, shell=True, capture_output=True, text=True, timeout=5)
    
    if "Publisher count: 0" in result.stdout:
        print(f"  ✗ FAILED: No publishers on {topic_name}")
        return False
    
    print(f"  ✓ Active: {topic_name}")
    
    # Show sample data
    echo_cmd = f"source /opt/ros/$ROS_DISTRO/setup.bash && source /home/mnt/Desktop/tasc_autonav_combined/install/setup.bash && timeout 2 ros2 topic echo {topic_name} --once 2>&1 | head -20"
    result = subprocess.run(echo_cmd, shell=True, capture_output=True, text=True, timeout=5)
    print(f"  Sample data:\n{result.stdout}")
    
    return True


def main():
    """Run autonomy_sensors tests."""
    
    print("=" * 70)
    print("AUTONOMY SENSORS TEST SUITE (Auto-Detection)")
    print("=" * 70)
    
    # Check for USB port
    usb_port = "/dev/ttyUSB0"
    if os.path.exists(usb_port):
        print(f"\n✓ USB port detected: {usb_port}")
        print("  Using REAL data from device\n")
        use_simulated = "false"
        port = usb_port
    else:
        print(f"\n✗ USB port NOT detected: {usb_port}")
        print("  Using SIMULATED data\n")
        use_simulated = "true"
        port = "/dev/ttyFAKE0"
    
    # Setup environment
    setup_cmd = "cd /home/mnt/Desktop/tasc_autonav_combined && source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash"
    
    processes = []
    
    try:
        # Start IMU node
        imu_cmd = f"{setup_cmd} && ros2 run autonomy_sensors imu_node --ros-args -p simulated_data:={use_simulated} -p port:={port}"
        proc_imu = run_command(imu_cmd, "IMU Node")
        processes.append(proc_imu)
        print_output(proc_imu, max_lines=10)
        time.sleep(1)
        
        # Start GPS node
        gps_cmd = f"{setup_cmd} && ros2 run autonomy_sensors gps_node --ros-args -p simulated_data:={use_simulated} -p port:={port}"
        proc_gps = run_command(gps_cmd, "GPS Node")
        processes.append(proc_gps)
        print_output(proc_gps, max_lines=10)
        time.sleep(1)
        
        # Start Foxglove Bridge
        bridge_cmd = f"{setup_cmd} && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
        proc_bridge = run_command(bridge_cmd, "Foxglove WebSocket Bridge")
        processes.append(proc_bridge)
        print_output(proc_bridge, max_lines=10)
        time.sleep(2)
        
        # Verify topics
        print("\n" + "=" * 70)
        print("TOPIC VERIFICATION")
        print("=" * 70)
        
        topics_to_check = [
            "/gps/fix",
            "/heading",
            "/imu/data_raw",
            "/imu/mag",
            "/cam_overlay/latitude_longitude",
        ]
        
        results = {}
        for topic in topics_to_check:
            try:
                results[topic] = verify_topic(topic)
                # Print any new output from processes
                print_output(proc_imu, max_lines=3)
                print_output(proc_gps, max_lines=3)
            except subprocess.TimeoutExpired:
                print(f"  ✗ TIMEOUT: {topic}")
                results[topic] = False
            except Exception as e:
                print(f"  ✗ ERROR: {topic} - {e}")
                results[topic] = False
        
        # Summary
        print("\n" + "=" * 70)
        print("TEST SUMMARY")
        print("=" * 70)
        passed = sum(1 for v in results.values() if v)
        total = len(results)
        print(f"Passed: {passed}/{total}\n")
        
        for topic, result in results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            print(f"  {status}: {topic}")
        
        print("\n[INFO] Nodes running. Press Ctrl+C to stop.\n")
        print("[FOXGLOVE] Connect to: ws://localhost:8765\n")
        
        # Keep running and display output
        while True:
            print_output(proc_imu, max_lines=3)
            print_output(proc_gps, max_lines=3)
            print_output(proc_bridge, max_lines=3)
            time.sleep(2)
    
    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Terminating nodes...")
    
    finally:
        for proc in processes:
            try:
                proc.terminate()
                proc.wait(timeout=2)
            except:
                proc.kill()
        print("[DONE] All nodes stopped.")


if __name__ == "__main__":
    main()
