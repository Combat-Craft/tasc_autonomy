/*********************************************************************************
 * In general, GPS related code is written before IMU related code
 * 
 * Please use the ESP32 Dev Module board. 
 *  - Used a ESP32-D0WD-V3 chip i.e. ESP32 Devkit v1 
 * 
 * IMU access has been done via FastIMU library, as it provides a simple interface
 *   to Accel, Gyro, Mag, Quaternions via Magdwick, calibratation, and LPF
 * - NOTE: LPF is automatically set for Acc and Gyro as seen in the source code
 *   - https://github.com/LiquidCGS/FastIMU/blob/main/src/sensors/F_ICM20948.cpp
 *   - none for Mag, sadly
 * - pulls from Calibrated_sensor_output and Calibrated_sensor_output examples
 *
 *
 ********************************************************************************/
#include <TinyGPS++.h> // for the GPS - TinyGPSPlus by Mikal Hart
#include "FastIMU.h" // for imu
#include "Madgwick.h" // for quat
#include <Wire.h>

/* =========================
   GPS CONFIG
   ========================= 
// these are the pin #s on the ESP32, we are using UART pins for the GPS 
// i.e. RX/MOsi and TX/MISO pins
#define RXD2 16 
#define TXD2 17
#define GPS_BAUD 38400

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
*/

/* =========================
   IMU CONFIG
   ========================= */
// ESP32 needs the pins. should be same as default, but let's be explicit
#define SDA_PIN 21
#define SCL_PIN 22
#define IMU_ADDRESS 0x69    //Default is not 0x68 but 0x69 for Sparkfun ICM20948
#define PERFORM_CALIBRATION //Comment to disable startup calibration

#define G_TO_MS2 9.80665
ICM20948 IMU;               //Change to the name of any supported IMU! 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
Madgwick filter;



/* =========================
   Print Timing CONFIG
   ========================= */
const unsigned long IMU_PERIOD_MS = 100; // 100 Hz = 10
const unsigned long GPS_PERIOD_MS = 10;  // 1 Hz
unsigned long last_imu_time = 0;
unsigned long last_gps_time = 0;


void setup() {
  Serial.begin(115200); 
  while (!Serial) { //wait for connection
    ;
  }
/*
  // === GPS setup ==================================================
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("# GPS: serial started");
  // === END GPS setup ==============================================
*/
  // === IMU I2C setup - from Fast IMU ==============================
  //Wire.begin();
  //Wire.setClock(400000); //400khz clock
  Wire.begin(SDA_PIN, SCL_PIN, 400000); //ensure ESP32 gets right pins for IMU  
  
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("# IMU:  Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("# IMU: Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("# IMU: Performing calibration");
  if (IMU.hasMagnetometer()) {
    delay(3000);
    Serial.println("# IMU: Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("# IMU: Magnetic calibration done!");
  }
  else {
    delay(10000);
  }

  delay(10000);
  Serial.println("# IMU: Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("# IMU: Calibration done!");
  Serial.println("# IMU: Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("# IMU: Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("# IMU: Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("# IMU: Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(10000);
  IMU.init(calib, IMU_ADDRESS);

  filter.begin(0.2f);
#endif
  // === END IMU setup ==============================================
  Serial.println("# IMU + GPS streaming started");
}

void loop() {
  unsigned long now_milli = millis(); // for IMU and GPS print timing
/*
  // Always parse GPS bytes first
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // -------- GPS OUTPUT --------  //THIS HAS BEEN LEFT ALONE FROM TONI'S
  if (now_milli - last_gps_time >= GPS_PERIOD_MS) {
    last_gps_time = now_milli;
    
    bool fix = gps.location.isValid();
    
    Serial.print("GPS,");
    Serial.print(now_milli); Serial.print(",");
    
    if (fix) {
      Serial.print(gps.location.lat(),6); Serial.print(",");
      Serial.print(gps.location.lng(),6); Serial.print(",");
      Serial.print(gps.altitude.meters(),2); Serial.print(",");
      Serial.print(gps.speed.mps(),2); Serial.print(",");
      Serial.print(gps.hdop.isValid() ? gps.hdop.value()/100.0f : NAN,2);
      Serial.print(",");
      Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
      Serial.print(",");
      Serial.println(1);
    } 
    else {
      Serial.println("nan,nan,nan,nan,nan,0,0");
    }
  }//END GPS OUTPUT
*/
  // -------- IMU OUTPUT --------  // from fast imu
  if (now_milli - last_imu_time >= IMU_PERIOD_MS) {  
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);
    //if (IMU.hasMagnetometer()){} // shouldn't have to check for ICM20948
    IMU.getMag(&magData); 

    // from Calibrated_quat.ino of FastIMU - getting quat
    filter.update(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, accelData.accelX, accelData.accelY, accelData.accelZ, magData.magX, magData.magY, magData.magZ);

    // print all test data data
    Serial.print("ACC: "); // for testing
    Serial.print(accelData.accelX * G_TO_MS2); Serial.print(","); // , is tab
    Serial.print(accelData.accelY * G_TO_MS2); Serial.print(",");
    Serial.print(accelData.accelZ * G_TO_MS2); Serial.print(",");
    Serial.print("GYR: "); // for testing
    Serial.print(gyroData.gyroX * DEG_TO_RAD); Serial.print(",");
    Serial.print(gyroData.gyroY * DEG_TO_RAD); Serial.print(",");
    Serial.print(gyroData.gyroZ * DEG_TO_RAD); Serial.print(",");
    Serial.print("MAG: "); // for testing
    Serial.print(magData.magX); Serial.print(",");
    Serial.print(magData.magY); Serial.print(",");
    Serial.print(magData.magZ); Serial.print(",");
    Serial.print("QUAT: ");
    Serial.print(filter.getQuatW()); Serial.print(",");
    Serial.print(filter.getQuatX()); Serial.print(",");
    Serial.print(filter.getQuatY()); Serial.print(",");
    Serial.print(filter.getQuatZ()); Serial.print(",");

    Serial.println();
  } // END IMU OUTPUT

  
  

}
