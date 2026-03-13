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
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
   
   // Configuring DMP to output data at multiple ODRs:
   // DMP is capable of outputting multiple sensor data at different rates to FIFO.
   // Setting value can be calculated as follows:
   // Value = (DMP running rate / ODR ) - 1
   // E.g. For a 225Hz ODR rate when DMP is running at 225Hz, value = (225/225) - 1 = 0.
   // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 10) == ICM_20948_Stat_Ok);         // Set to 5Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 10) == ICM_20948_Stat_Ok);  // Set to 5Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 10) == ICM_20948_Stat_Ok); // Set to 5Hz
   
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
      Serial.print("IMU,");
      imu_loop(); // prints Q1,Q2,Q3,ax,ay,az,gx,gy,gz,mag_x,mag_y,mag_z i.e. quaternion, acceleromter, gyroscope, magnetometer
      Serial.println(1);// to ensure newline
   }
   
   /* -------- GPS OUTPUT -------- */
   //THIS HAS BEEN LEFT ALONE
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

void imu_loop(){
   // Read any DMP data waiting in the FIFO
   // Note:
   //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
   //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
   //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
   //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
   //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
   icm_20948_DMP_data_t data;
   myICM.readDMPdataFromFIFO(&data);
   
   if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
   {
      //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
      //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
      //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
      //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
      //SERIAL_PORT.println( data.header, HEX );

      // to get yaw/heading, from Example7_DMP_Quat6_EulerAngles.ino
      // will output only quat, then do yaw/heading calcs in python
      if ((data.header & DMP_header_bitmap_Quat6) > 0) // Check for orientation data (Quat9)
      {
         // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
         // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
         // The quaternion data is scaled by 2^30.
      
         //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);
      
         // Scale to +/- 1
         double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
         double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
         double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

         //Q1,Q2,Q3
         Serial.print(q1, 6); Serial.print(",");
         Serial.print(q2, 6); Serial.print(",");
         Serial.print(q3, 6); Serial.print(",");
      }

      if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
      {
         float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
         float acc_y = (float)data.Raw_Accel.Data.Y;
         float acc_z = (float)data.Raw_Accel.Data.Z;

         Serial.print(acc_x,6); Serial.print(","); 
         Serial.print(acc_y,6); Serial.print(","); 
         Serial.print(acc_z,6); Serial.print(","); 
      }

      if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
      {
         float gy_x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
         float gy_y = (float)data.Raw_Gyro.Data.Y;
         float gy_z = (float)data.Raw_Gyro.Data.Z;

         Serial.print(gy_x,6); Serial.print(","); 
         Serial.print(gy_y,6); Serial.print(","); 
         Serial.print(gy_z,6); Serial.print(","); 
      }   
      
      if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
      {
         float mag_x = (float)data.Compass.Data.X; // Extract the compass data
         float mag_y = (float)data.Compass.Data.Y;
         float mag_z = (float)data.Compass.Data.Z;

         Serial.print(mag_,6); Serial.print(","); 
         Serial.print(mag_y,6); Serial.print(","); 
         Serial.print(mag_z,6); Serial.print(","); 
      }      
   }

   if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
   {
      delay(10);
   }   
   
}// END imu_loop()
