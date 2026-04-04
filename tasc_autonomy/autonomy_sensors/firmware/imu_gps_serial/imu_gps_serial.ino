/****************************************************************
 * This code is based on Sparkfun's ICM20948 code "Example10_DMP_FastMultipleSensors.ino" (IMU),
 * "Example9_DMP_MultipleSensors.ino" (IMU), and imu_gps_simple.ino by Antonia.
 * 
 * Please use the ESP32 Dev Module board. 
 *  - Used a ESP32-D0WD-V3 chip i.e. ESP32 Devkit v1
 *
 * DMP is not used despite providing Quat, as it is far more complicated and needs custom calibrations
 ***************************************************************/
#include "ICM_20948.h" //Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#include <Wire.h>    // don't need to re-include bc ICM_20948.h has it
#include <TinyGPS++.h> // for the GPS / NMEA

#define SERIAL_PORT Serial

/* =========================
   SENSOR FUSION CONFIG
   ========================= */
// if I decide to find quaternions here

/* =========================
   IMU CONFIG
   ========================= */
ICM_20948_I2C myICM; // removed the SPI configuration

#define WIRE_PORT Wire    
#define AD0_VAL 1 // For the ICM_20948 object: The value of the last bit of the I2C address.
                  // On the SparkFun 9DoF IMU breakout the default is 1, 
                  // and when the ADR jumper is closed the value becomes 0


// NOTE: we want as much accuracy as possible
const float ACC_CONVERSION  = 0.00980665f; // m/s^2 | from milli g's : 1g = 9.80665 m/s^2, and 1g = 1000 milli-g
//const float GYRO_CONVERSION = DEG_TO_RAD; // radian/second | from deg/sec :  already defined by Arduino
//const float MAG_CONVERSION = 0.000001f; // Tesla | from microTesla: 1,000,000  micro Tesla = 1 Tesla
                                          // tested printing converted MAG, loose too much precision

// ESP32 needs the pins. should be same as default, but let's be explicit
#define SDA_PIN 21
#define SCL_PIN 22

/* Timing */
const unsigned long IMU_PERIOD_MS = 10;    // 100 Hz
const unsigned long GPS_PERIOD_MS = 1000;  // 1 Hz

unsigned long last_imu_time = 0;
unsigned long last_gps_time = 0;

/* =========================
   GPS CONFIG
   ========================= */
// these are the pin #s on the ESP32, we are using UART pins for the GPS i.e. RX/MOsi and TX/MISO pins
#define RXD2 16 
#define TXD2 17
#define GPS_BAUD 38400

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

/* =========================
   SETUP
   ========================= */
void setup() {
  //serial monitor from ESP32
  Serial.begin(115200);
  delay(2000);

  // GPS setup
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("# GPS serial started");

  // IMU setup
  WIRE_PORT.begin(SDA_PIN, SCL_PIN, 400000);    

  setup_imu();
  
  Serial.println("# IMU + GPS streaming started");
} //END setup()

void setup_imu(){
  // from Example1_Basics.ino, Example2_advanced.ino  

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized){
    myICM.begin(Wire, AD0_VAL);  

    Serial.print(F("Initialization of the IMU sensor returned: "));
    Serial.println(myICM.statusString());
     
    if (myICM.status != ICM_20948_Stat_Ok){ // failed to start imu
      Serial.println("Trying IMU again...");
      delay(500);
    }
    else{
      initialized = true;
    }
  }// END while loop  
   
} // END setup_imu()


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
    //'header' field to indicate this is IMU data, and timestampe for topic info 
     
    // start IMU serial print
    if (myICM.dataReady()){
      myICM.getAGMT();             // The values are only updated when you call 'getAGMT'

      // IMU is does not appear to be ENU, but needs more testing
      // might be due to the fact the MAG axes are different from ACC/GYR?
      //    X_MAG = X_ACC;   Y = -Y_ACC;   Z_MAG = -Z_ACC (i.e. DOWN not UP) 
      //    => I fliped the mag axes to match the acc/gyr axes

      float acc_x = myICM.accX() * ACC_CONVERSION; // unit is now m/s^2
      float acc_y = myICM.accY() * ACC_CONVERSION; 
      float acc_z = myICM.accZ() * ACC_CONVERSION; 
      
      float gyr_x = myICM.gyrX() * DEG_TO_RAD; // unit is now radians/sec
      float gyr_y = myICM.gyrY() * DEG_TO_RAD; 
      float gyr_z = myICM.gyrZ() * DEG_TO_RAD; 

      float mag_x = myICM.magX(); // unit is STILL uT, to preserve precision 
      float mag_y = -myICM.magY();//   - will be converted to T in ROS2
      float mag_z = -myICM.magZ();     

      //Print the data
      Serial.print("IMU,");
      Serial.print(now); Serial.print(",");
      //Serial.print("ACC,"); // for testing
      Serial.print(acc_x,  6); Serial.print(",");
      Serial.print(acc_y,  6); Serial.print(",");
      Serial.print(acc_z,  6); Serial.print(",");
      //Serial.print("GYR,"); // for testing
      Serial.print(gyr_x, 6); Serial.print(",");
      Serial.print(gyr_y, 6); Serial.print(",");
      Serial.print(gyr_z, 6); Serial.print(",");
      //Serial.print("MAG,"); // for testing
      Serial.print(mag_x, 6); Serial.print(",");
      Serial.print(mag_y, 6); Serial.print(",");
      Serial.println(mag_z, 6); 
    }
    else{
      Serial.print("nan,nan,nan,nan,nan,nan,nan,nan,nan,");
      SERIAL_PORT.println("Waiting for IMU data: myICM.dataReady() == False");
    }
  }//END imu if()
  
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
    } 
    else {
      Serial.println("nan,nan,nan,nan,nan,0,0");
    }
  }//END GPS if
  
}//END loop()
