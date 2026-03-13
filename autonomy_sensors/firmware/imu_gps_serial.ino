/****************************************************************
 * This code is based on Sparkfun's ICM20948 code "Example10_DMP_FastMultipleSensors.ino" (IMU),
 * "Example9_DMP_MultipleSensors.ino" (IMU), and imu_gps_simple.ino by Antonia.
 *
 * The following comment is from the IMU example:
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 ***************************************************************/
#include "ICM_20948.h" //Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
// I also just flat out included the src folder, but will assume we're installing the actual library @ https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

// #include <Wire.h> // removed as already defined in firmware/src/ICM_20948.h
#include <TinyGPS++.h>

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define SERIAL_PORT Serial

/* =========================
   IMU CONFIG
   ========================= */
//#define USE_SPI       // Uncomment this to use SPI
// #define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
// #define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#ifdef USE_SPI
   ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
   ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// part of old IMU sensor?
//#define MPU_ADDR 0x68
//#define SDA_PIN 21
//#define SCL_PIN 22

const float ACC_SCALE  = 16384.0f;
const float GYRO_SCALE = 131.0f;
const float G = 9.80665f;

/* Biases */
float gbx=0, gby=0, gbz=0;
float abx=0, aby=0, abz=0;

/* Timing */
const unsigned long IMU_PERIOD_MS = 10;    // 100 Hz
const unsigned long GPS_PERIOD_MS = 1000;  // 1 Hz

unsigned long last_imu_time = 0;
unsigned long last_gps_time = 0;

/* =========================
   GPS CONFIG
   ========================= */
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 38400

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);


/* =========================
   MPU FUNCTIONS
   ========================= */
void read_mpu(int16_t &ax, int16_t &ay, int16_t &az,
              int16_t &gx, int16_t &gy, int16_t &gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void setup_imu(){
   // not entirely sure with GPS and IMU, if this will be ok...
   /*
   while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
      SERIAL_PORT.read();

   SERIAL_PORT.println(F("IMU: Press any key to continue..."));
   while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
    ;
 // main setup() has this already, will comment out
   //#ifdef USE_SPI
   //   SPI_PORT.begin();
   //#else
   //   WIRE_PORT.begin();
   //   WIRE_PORT.setClock(400000);
   //#endif
*/
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized){
     // Initialize the ICM-20948
     // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
      #ifdef USE_SPI
          myICM.begin(CS_PIN, SPI_PORT);
      #else
          myICM.begin(WIRE_PORT, AD0_VAL);
      #endif

      SERIAL_PORT.print(F("Initialization of the IMU sensor returned: "));
      SERIAL_PORT.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok){
         SERIAL_PORT.println(F("Trying again..."));
         delay(500);
      }
      else{
         initialized = true;
      }
   }   

   SERIAL_PORT.println(F("IMU Device connected!"));
   bool success = true; // Use success to show if the DMP configuration was successful
   
   // Initialize the DMP. initializeDMP is a weak function. In this example we overwrite it to change the sample rate (see below)
   success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
   
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
   
   // Enable the DMP Game Rotation Vector sensor (Quat6)
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
   
   // Enable additional sensors / features
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
   
   // Configuring DMP to output data at multiple ODRs:
   // DMP is capable of outputting multiple sensor data at different rates to FIFO.
   // Setting value can be calculated as follows:
   // Value = (DMP running rate / ODR ) - 1
   // E.g. For a 225Hz ODR rate when DMP is running at 225Hz, value = (225/225) - 1 = 0.
   // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz
   
   // Enable the FIFO
   success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
   
   // Enable the DMP
   success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
   
   // Reset DMP
   success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
   
   // Reset FIFO
   success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
   
   // Check success
   if (success){
      SERIAL_PORT.println(F("DMP enabled!"));
   }
   else{
      SERIAL_PORT.println(F("Enable DMP failed!"));
      SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      while (1)
      ; // Do nothing more
   }
} // END setup_imu()

void wake_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

void calibrate_imu(int samples=1200) {
  Serial.println("# Calibrating IMU, keep still...");
  delay(1500);

  long sax=0, say=0, saz=0;
  long sgx=0, sgy=0, sgz=0;

  for (int i=0; i<samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    read_mpu(ax, ay, az, gx, gy, gz);
    sax+=ax; say+=ay; saz+=az;
    sgx+=gx; sgy+=gy; sgz+=gz;
    delay(2);
  }

  float axm = sax/(float)samples;
  float aym = say/(float)samples;
  float azm = saz/(float)samples;

  gbx = sgx/(float)samples;
  gby = sgy/(float)samples;
  gbz = sgz/(float)samples;

  float target_az = (azm >= 0) ? ACC_SCALE : -ACC_SCALE;
  abx = axm;
  aby = aym;
  abz = azm - target_az;

  Serial.println("# IMU calibration done");
}

/* =========================
   SETUP
   ========================= */
void setup() {
   Serial.begin(115200);
   delay(2000);

   // start GPS unit
   gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
   Serial.println("# GPS serial started");

   // start IMU
   WIRE_PORT.begin(); // from new IMU example code
   //Wire.begin(SDA_PIN, SCL_PIN); // the SDA and SLA pins for MPU 6050 module, old IMU
   Wire.setClock(400000);
   setup_imu();
   //wake_mpu(); // old imu function
   //calibrate_imu(); // old imu
   
   Serial.println("# IMU + GPS streaming started");
}

/* =========================
   LOOP
   ========================= */
void loop() {

  /* Always parse GPS bytes */
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  unsigned long now = millis();

  /* -------- IMU OUTPUT -------- */
  if (now - last_imu_time >= IMU_PERIOD_MS) {
    last_imu_time = now;

    int16_t ax, ay, az, gx, gy, gz;
    read_mpu(ax, ay, az, gx, gy, gz);

    ax -= abx; ay -= aby; az -= abz;
    gx -= gbx; gy -= gby; gz -= gbz;

    float ax_si = ax * (G / ACC_SCALE);
    float ay_si = ay * (G / ACC_SCALE);
    float az_si = az * (G / ACC_SCALE);

    float gx_si = gx * (M_PI/180.0f) / GYRO_SCALE;
    float gy_si = gy * (M_PI/180.0f) / GYRO_SCALE;
    float gz_si = gz * (M_PI/180.0f) / GYRO_SCALE;

    Serial.print("IMU,");
    Serial.print(now); Serial.print(",");
    Serial.print(ax_si,6); Serial.print(",");
    Serial.print(ay_si,6); Serial.print(",");
    Serial.print(az_si,6); Serial.print(",");
    Serial.print(gx_si,6); Serial.print(",");
    Serial.print(gy_si,6); Serial.print(",");
    Serial.println(gz_si,6);
  }

  /* -------- GPS OUTPUT -------- */
  if (now - last_gps_time >= GPS_PERIOD_MS) {
    last_gps_time = now;

    bool fix = gps.location.isValid();

    Serial.print("GPS,");
    Serial.print(now); Serial.print(",");

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
    } else {
      Serial.println("nan,nan,nan,nan,nan,0,0");
    }
  }
}
