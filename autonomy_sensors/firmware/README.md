This folder contains the firmware required for:
 - **SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)**
   -  ICM_20948.h, ICM_20948.cpp
   -  util folder for the previous two code files
 - the GPS unit.

It also contains the Arduino code that will serially output:
- GPS: **latitude**, **longitude**
- IMU: **compass**, **yaw/heading**
- 

The bolded are to be used for the heads-up display, the rest required for standard sensor_msgs/msg/NavSatFix and sensor_msgs/Imu Messages.

https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html
https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html

The example10 code is from the SparkFun github, and imu_gps_serial.ino is the actual code to be used. It is based on imu_gps_simple.ino and example10.ino
