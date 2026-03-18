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

#include <Wire.h>      // placed it back in, but depending on how it goes maybe comment out again
#include <TinyGPS++.h>


#define SERIAL_PORT Serial

/* =========================
   IMU CONFIG
   ========================= */
ICM_20948_I2C myICM; // removed the SPI configuration

#define WIRE_PORT Wire    
// For the ICM_20948 object: The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// NOTE: we want as much accuracy as possible, so I will do the bulk of the conversion in ROS2/python, mainly yhe micro and milli to standard
const float ACC_CONVERSION  = 9.80665f; // micro m/s^2 | from micro g's : 1 g = 9.80665 m/s^2, and 1g = 1000 milli-g
const float GYRO_CONVERSION = DEG_TO_RAD; // radian/second | from deg/sec : apparently already defined by Arduino
//const float MAG_CONVERSION = 0.000001f; // Tesla | from microTesla: 1,000,000  micro Tesla = 1 Tesla


// ESP32 needs the pins and address? but the sparkfun ICM_20948.cpp/h object appaers to handle the wire part...
//#define IMU_ADDR 0x68 
//define SDA_PIN 21
//#define SCL_PIN 22

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
  Wire.begin(SDA_PIN, SCL_PIN); 
  Wire.setClock(400000);
   
  //wake_mpu(), not required with ICM_20948_I2C object, i think
  /*
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); //the PDF has more deets on the actual start sequence but man...
  Wire.write(0x00);
  Wire.endTransmission(); 
  delay(50);
  */
  setup_imu();
  
  Serial.println("# IMU + GPS streaming started");
} //END setup()

void setup_imu(){
  // from Example1_Basics.ino, Example2_advanced.ino  

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized){
    myICM.begin(Wire, AD0_VAL); //handles the wake_mpu() stuff??? as well as some calibration apparently??? 
     //its complex in there i aint touching it... esp with DMP involve, which we'll likely want for quartenionr or what its spelled

    Serial.print(F("Initialization of the IMU sensor returned: "));
    Serial.println(myICM.statusString());
     
    if (myICM.status != ICM_20948_Stat_Ok){ // failed to start imu
      Serial.println("Trying again...");
      delay(500);
    }
    else{
      initialized = true;
    }
  }// END while loop  
   
  // advanced has more setup but idc right now
  // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/Arduino/Example2_Advanced/Example2_Advanced.ino
   
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
    Serial.print("IMU,");
    Serial.print(now); Serial.print(",");
     
    // start IMU serial print
    if (myICM.dataReady()){
      myICM.getAGMT();             // The values are only updated when you call 'getAGMT'
      Serial.print("IMU,");
      Serial.print(now); Serial.print(",");
      Serial.print(agmt.acc.axes.x,6*ACC_CONVERSION); Serial.print(",");
      Serial.print(agmt.acc.axes.y,6*ACC_CONVERSION); Serial.print(",");
      Serial.print(agmt.acc.axes.z,6*ACC_CONVERSION); Serial.print(",");
      Serial.print(agmt.gyr.axes.x,6*GYRO_CONVERSION); Serial.print(",");
      Serial.print(agmt.gyr.axes.y,6*GYRO_CONVERSION); Serial.print(",");
      Serial.print(agmt.gyr.axes.z,6*GYRO_CONVERSION); Serial.print(",");
      Serial.print(agmt.mag.axes.x,6); Serial.print(",");
      Serial.print(agmt.mag.axes.y,6); Serial.print(",");
      Serial.println(agmt.mag.axes.z,6); 
      delay(30);
    }
    else{
      Serial.print("nan,nan,nan,nan,nan,nan,nan,nan,nan,");
      // SERIAL_PORT.println("Waiting for IMU data: myICM.dataReady() == False");
      delay(500);
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


