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

// #include <Wire.h>      // removed as already defined in firmware/src/ICM_20948.h
#include <TinyGPS++.h>

#define WIRE_PORT Wire    // Your desired Wire port.      Used when "USE_SPI" is not defined
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

// from old IMU sensors, not sure if PINS still required
//#define MPU_ADDR 0x68
//#define SDA_PIN 21
//#define SCL_PIN 22

//const float ACC_SCALE  = 16384.0f;
//const float GYRO_SCALE = 131.0f;
//const float G = 9.80665f;

/* Biases */
//float gbx=0, gby=0, gbz=0;
//float abx=0, aby=0, abz=0;

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
  // from Example1_Basics.ino, Example2_advanced.ino  

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized){
    #ifdef USE_SPI
      myICM.begin(CS_PIN, SPI_PORT);
    #else
      myICM.begin(Wire, AD0_VAL);
    #endif

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
   
} // END setup_imu()

/* =========================
   SETUP
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(2000);

  //due to IMU
  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    Wire.begin();
    //Wire.begin(SDA_PIN, SCL_PIN); // the SDA and SLA pins for MPU 6050 module, old IMU
    Wire.setClock(400000);
  #endif
  
  // start GPS unit
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("# GPS serial started");
  
  // start IMU
  setup_imu();
  
  Serial.println("# IMU + GPS streaming started");
} //END setup()

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
    // start IMU serial print
    if (myICM.dataReady()){
      myICM.getAGMT();             // The values are only updated when you call 'getAGMT'
      printScaledAGMT(&myICM);     // This function takes into account the scale settings 
                                   // from when the measurement was made to calculate the values with unit
      delay(30);
    }
    else{
      Serial.print("nan,nan,nan,nan,nan,nan,nan,nan,nan,");
      delay(500);
    }
    Serial.println(""); //IMU stops printing
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

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  printFormattedFloat(sensor->accX(), 4, 2); Serial.print(",");
  printFormattedFloat(sensor->accY(), 4, 2); Serial.print(",");
  printFormattedFloat(sensor->accZ(), 4, 2); Serial.print(",");
   
  printFormattedFloat(sensor->gyrX(), 4, 2); Serial.print(",");
  printFormattedFloat(sensor->gyrY(), 4, 2); Serial.print(",");
  printFormattedFloat(sensor->gyrZ(), 4, 2); Serial.print(",");
  
  printFormattedFloat(sensor->magX(), 4, 2); Serial.print(",");
  printFormattedFloat(sensor->magY(), 4, 2); Serial.print(",");
  printFormattedFloat(sensor->magZ(), 4, 2);
}

// helper function from example1, to print the imu data
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)  {
    Serial.print("-");
  }
  else  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++){
    uint32_t tenpow = 0;
    if (indi < (leading - 1))    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++){
      tenpow *= 10;
    }
    if (aval < tenpow){
      Serial.print("0");
    }
    else{
      break;
    }
  }//END for loop
  if (val < 0){
    Serial.print(-val, decimals);
  }
  else{
    Serial.print(val, decimals);
  }
}
