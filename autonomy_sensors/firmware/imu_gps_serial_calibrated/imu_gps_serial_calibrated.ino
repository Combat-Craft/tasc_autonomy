/****************************************************************
 * This code is based-on/takes code from: 
 *   - tonitonitonitoni (Antonia)'s "imu_gps_simple.ino" for GPS
 *   - Sparkfun's ICM20948 code "Example1_Basics" for IMU
 *   - Sparkfun's ICM20948 code "Example2_Advanced" for IMU
 *   - ICM_20948_Mahony by S.J. Remington (6/2021) 
 * 
 * Please use the ESP32 Dev Module board. 
 *  - Used a ESP32-D0WD-V3 chip i.e. ESP32 Devkit v1
 *
 * NOTE: DMP is not used despite providing Quat, as it is far more 
 *       complicated and needs custom calibrations. Instead,
 *       Mahony sensor fusion was used to calculate them
 *       and the heading
 *       
 *  Requires the Sparkfun ICM_20948 library
 *  Standard sensor orientation X North (yaw=0), Y West, Z up
 * 
 *  To collect data for calibration, use the companion program ICM_20948_get_cal_data
 * 
 ***************************************************************/

//#include <Wire.h>    // don't need to re-include bc ICM_20948.h has it

#include "ICM_20948.h" //Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <TinyGPS++.h> // for the GPS / NMEA

#define SERIAL_PORT Serial
#define WIRE_PORT Wire  

/* Print Timing */
const unsigned long IMU_PERIOD_MS = 100;    // 100 Hz = 10
const unsigned long GPS_PERIOD_MS = 100;  // 1 Hz

unsigned long last_imu_time = 0;
unsigned long last_gps_time = 0;

/* =========================
   SENSOR FUSION CONFIG
   ========================= */
// VERY IMPORTANT!

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = DEG_TO_RAD * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {-83.8, 112.4, -8.0};
// {-80.1, 102.1, -12.1}; //at home (artemis)

//Accel scale
float A_B[3] 
{   31.06,  -37.96,  394.9};

float A_Ainv[3][3]
{ { 0.06094 , 0.00028 , 0.00029 },
  { 0.00028 , 0.06164 , -0.00101 },
  { 0.00029 , -0.00101 , 0.06007 }
};

//Mag scale
float M_B[3] = {-478.73, 679.24, 768.61};

float M_Ainv[3][3]
{ {  5.3561 , -0.12397 , 0.0206 },
  { -0.12397,  6.37031 ,-0.09395},
  {  0.0206 , -0.09395 , 4.7939 }
};

// local magnetic declination in degrees
float DECLINATION = 10.05;


/* =========================
   IMU CONFIG
   ========================= */
ICM_20948_I2C myICM; // removed the SPI configuration
#define AD0_VAL 1 // For the ICM_20948 object: The value of the last bit of the I2C address.
                  // On the SparkFun 9DoF IMU breakout the default is 1, 
                  // and when the ADR jumper is closed the value becomes 0

// ESP32 needs the pins. should be same as default, but let's be explicit
#define SDA_PIN 21
#define SCL_PIN 22


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
  while (!Serial) //wait for connection
  //delay(2000);   //^ is better

  // GPS setup
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("# GPS serial started");

  // IMU I2C setup
  WIRE_PORT.begin(SDA_PIN, SCL_PIN, 400000); //ensure right pins   
  setup_imu();
  Serial.println("# IMU + GPS streaming started");
 
} //END setup()

/* Set up IMU, which includes forcing the magnetometer manually as it sometimes fails to start
 * from Example1_Basics.ino, Example2_advanced.ino  
 */
void setup_imu(){
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

  Serial.println("IMU I2C Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("# IMU Software Reset returned: "));
    Serial.println(myICM.statusString());
  }
  delay(250);

  // Now wake the IMU sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // Esnure the magnetometer starts, cause it doesn't always for some reason
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("# IMU: startupMagnetometer returned: "));
    Serial.println(myICM.statusString());
  }

  Serial.println();
  Serial.println(F("# IMU Configuration complete!"));
   
} // END setup_imu()


/* =========================
   LOOP
   ========================= */
void loop() {
  /* Always parse GPS bytes */
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
   
  unsigned long now_milli = millis();

  // for mahony and calibrated IMU data
  static int loop_counter = 0; //sample & update loop counter
  /*
   * UNITS: 
   *  - Gxyz is rad/s
   *  - Mxyz is apparently microTesla
   *  - Axyz is corrected to m/s^2 (raw from jrem's, which gave it in 100*m/s^2 ) 
   */
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data, [x,y,z]
  static float ANxyz[3], MNxyz[3]; // to hold normalized, as I need un normalized too
  
  /* -------- IMU OUTPUT -------- */
  if (now_milli - last_imu_time >= IMU_PERIOD_MS) {     
    // Update the sensor values whenever new data is available
    if (myICM.dataReady()){
      myICM.getAGMT();             // The values are only updated when you call 'getAGMT'

      get_scaled_IMU(Gxyz, Axyz, Mxyz);

      // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0
      Mxyz[1] = -Mxyz[1]; //reflect Y and Z
      Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

      // print calibrated IMU data
      Serial.print("IMU,");
      Serial.print(now_milli); Serial.print(",");
      //Serial.print("ACC,"); // for testing
      Serial.print(Axyz[0],  6); Serial.print(",");
      Serial.print(Axyz[1],  6); Serial.print(",");
      Serial.print(Axyz[2],  6); Serial.print(",");
      //Serial.print("GYR,"); // for testing
      Serial.print(Gxyz[0], 6); Serial.print(",");
      Serial.print(Gxyz[1], 6); Serial.print(",");
      Serial.print(Gxyz[2], 6); Serial.print(",");
      //Serial.print("MAG,"); // for testing
      Serial.print(Mxyz[0], 6); Serial.print(",");
      Serial.print(Mxyz[1], 6); Serial.print(",");
      Serial.println(Mxyz[2], 6); 
      //Serial.print(Mxyz[2], 6); Serial.print(",");
      //Serial.print("HEADING,"); // for testing
      //Serial.println(get_heading_simple(Mxyz[0],Mxyz[1]), 1); 
   
    }
    else{
      Serial.print("nan,nan,nan,nan,nan,nan,nan,nan,nan,");
      SERIAL_PORT.println("Waiting for IMU data: myICM.dataReady() == False");
    }
  }//END imu if()
  
  /* -------- GPS OUTPUT -------- */
  //THIS HAS BEEN LEFT ALONE
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
  }//END GPS if
  
}//END loop()


//just for testing
float get_heading_simple(float mx, float my){ 
  float heading;
  
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
  
  heading -= DECLINATION * DEG_TO_RAD;
  
  if (heading > PI)
    heading -= (2 * PI);
  else if (heading < -PI)
    heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= RAD_TO_DEG;
  
  return heading;
}

// NOTE: for loops have 1 line, thats why they look like that
// function to subtract offsets and apply scale/correction matrices to IMU data
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Gxyz[0] = Gscale * (myICM.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (myICM.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (myICM.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = myICM.agmt.acc.axes.x;
  Axyz[1] = myICM.agmt.acc.axes.y;
  Axyz[2] = myICM.agmt.acc.axes.z;
  
  Mxyz[0] = myICM.agmt.mag.axes.x;
  Mxyz[1] = myICM.agmt.mag.axes.y;
  Mxyz[2] = myICM.agmt.mag.axes.z;

  //apply accel offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++){
    temp[i] = (Axyz[i] - A_B[i]);
  }
  Axyz[0] = (A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2]) * 0.01;
  Axyz[1] = (A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2]) * 0.01;
  Axyz[2] = (A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2]) * 0.01;

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++){ 
    temp[i] = (Mxyz[i] - M_B[i]);
  }
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];

}
