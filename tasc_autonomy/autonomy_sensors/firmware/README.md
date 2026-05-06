# Instructions

Currently, for firmware, On arduino
  1) connect the ESP32 with a usb c cable to the jetson
  2) Select the board to be the ```ESP32 Dev Module```
  3) Load and compile```imu_gps_serial_calibrated.ino```. It should be fine, but if the libraries poofed somehow, just add them back in.
  4) Check on Serial Monitor 115200 via Arduino IDE.
     
We likely need to recalibrated once mounted, in particularily the magnetometer tho the difficulting of getting that true sphere of points will be difficult.

**DECLINATION WILL NEED TO BE UPDATED TO COMPETITION LOCATION WHEN WE GET THERE**

~~imu_gps_serial_ahrs.ino needs to be refactored. Right now, it has the calibrated stuff and quaternions from https://github.com/jremington/ICM_20948-AHRS, but honestly i can remove the sensor fusion and just keep the calibrated output. I asked the Maker if he could confirm the calibrated units, but atm I think they are 0.01*m/s^2, rad/sec, and microTesla.~~

# To-Do (IF TIME):
  1) Add Low-pass filters as per example2_advanced to reduce the variance
  2) figure out what to do with covariance
  3) Add teh calibrate code and steps

     
# Firmware Info
ESP32 DEvkit Rev2, it seems, since Rx/TX are 16/17, and SDA/SLA were 21/22.

[ESP32 pinout](https://pinoutdiagrams.com/wp-content/images/devkit-v1-esp32-pinout-hrnln.jpg)


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
Ignore it for now, not worth the effort at the moment.

# Package and Code 

*needs to be updated

# Notes/Logs
### MArch 29th, 2026
Calibrated and add quats to the firware code

TODO: 
  - add quats and heading to serial print
  - rework broadcaster to read new serial print format, rework heading.
  - figure out how to transform the NWU to ENU

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
