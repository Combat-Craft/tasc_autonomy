# Firmware Info
ESP32 DEvkit Rev2, it seems, since Rx/TX are 16/17, and SDA/SLA were 21/22.

<img src="https://pinoutdiagrams.com/wp-content/images/devkit-v1-esp32-pinout-hrnln.jpg" alt="ESP32 pinout">


## SparkFun GPS NEO-M9N

### Wiring: 
refering to [sparkfun's hookup guide](https://learn.sparkfun.com/tutorials/sparkfun-gps-neo-m9n-hookup-guide).

From Antonia's set up, it is using the UART i.e. RX/TX pins for GPS serial. They are connected to the respective UART pins on the ESP32, 16(RX1) and 17(TX1). Seems to be clearcut connecting the GPS to the ESP.

### Firmware Libraries:
The code uses [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus/tree/master) for Arduino NMEA parsing, and ESP32's [HardwareSerial](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.cpp) to read data from the GPS board.

## SparkFun 9DoF IMU (ICM-20948)

### Wiring: 
[This tutorial](https://www.esp32learning.com/code/esp32-and-icm-20948-motiontracking-device-arduino-example.php) uses different code and some other boards, but the general wiring looks fine.

refering to [sparkfun's hookup guide](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide)).

From Antonia's set up, it orginally used the SDA/21 and SLC/22 pins, so I2C communications. The given [Arduino Code examples](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main/examples/Arduino) appear to assume Quicc which are also I2C. 

However, this new board has more complicated hookup as per [sparkfun's hardware guide](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide#hardware-hookup). The Top is the I2C side (will be labelled as such vs SPI), and will need to connect to the Da (SDA) and CL(SLC) pins from the sparkfun to #ESP32's SDA/21 and SCL/22.

Address of the I2C in client/slave mode is 0x68 as per Sparkfun. (The ICM chip notes it is b110100X where the LSB bit of the 7-bit address is determined by the logic level on pin AD0). Sparkfun uses AD0 = 1

### Firmware Libraries:
The code uses [sparkfun's arduino library](https://github.com/mikalhart/TinyGPSPlus/tree/master) to handle pretty much all the Wire.h based connection, handshaking, and read/write, and DMP access.

### Note on DMP:
Considering using to gain access to quaternion, 6 axis or 9 axis, or game vector data, etc. however, per doc. However because of the note below in the ICM-20948's document

> 4.3 DMP MEMORY ACCESS
> Reading/writing DMP memory and FIFO through I2C in a multithreaded environment can cause wrong data being read.
> To avoid the issue, one may use SPI instead of I2C, or use I2C with mutexes

If we do, we likely have to avoid using myICM.getAGMT() for accel, gyro, magneto, temp values and instead pull all of those from DMP instead.
As well, there aren't example code on all DMP sensors, let alone information on the format each spits out, just their data size. Pages 11-13 gives some info for the a,g,m,t but the rest are ????.
I'm assuming these are in scientific notation... maybe??? we'll have to test the ones we want if do use these.

```
  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)
```
Refer to this [course/tutorial](https://github.com/FastRobotsCornell/FastRobots-2026/blob/main/docs/tutorials/dmp.md) for some extra refinement on DMP code, in particular the publishing rate cause the FIFO can't handle overflow i.e. it crashes the whole chip. 

# Package and Code 

Currently, the main code is **imu_gps_serial.ino**. Eventually, will developed **imu_gps_serial.ino**, which is similar but hopefully we can use the DMP to get the 
This folder contains the firmware required for:
 - **SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)**
   -  ICM_20948.h, ICM_20948.cpp, 
   -  and the /util folder required for the previous two code files
 - the GPS unit


# Notes/Logs
### MArch 17th, 2026
NEED TO TEST IF ENU OR NED due to robot_localization expecting it to be ENU, and MANY sensor fusion libraries, Arduino or Python, expecting NED.
https://github.com/uutzinger/pyIMU
https://github.com/adafruit/Adafruit_AHRS/tree/master

not entirely sure we actually need quaternion at the moment. pause for now.

```
    Acceleration: Be careful with acceleration data. The state estimation nodes in robot_localization assume that an IMU that is placed in its neutral right-side-up position on a flat surface will:

        Measure +9.81 meters per second squared for the Z axis.
        If the sensor is rolled +90 degrees (left side up), the acceleration should be +9.81 meters per second squared for the Y axis.
        If the sensor is pitched +90 degrees (front side down), it should read -9.81 meters per second squared for the X axis.
```

### March 13th, 2026
The bolded are to be used for the heads-up display, the rest required for standard sensor_msgs/msg/NavSatFix and sensor_msgs/Imu Messages.

https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html
https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html

The example10 code is from the SparkFun github, ~~and imu_gps_serial.ino is the actual code to be used. It is based on imu_gps_simple.ino and example10.ino~~

So, i misunderstood compass, its the magnetometer. While quat is from DMP, and prob needed later, for now let's just focus on the accel, gyro and magnetometer. I'll calculate the compass and heading angles from those 3 rather from quat

Us another sparkfun board for later use, has code for both reg serial and DMP code
https://github.com/sparkfun/9DOF_Razor_IMU/blob/master/Firmware/_9DoF_Razor_M0_Firmware/_9DoF_Razor_M0_Firmware.ino
learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide
