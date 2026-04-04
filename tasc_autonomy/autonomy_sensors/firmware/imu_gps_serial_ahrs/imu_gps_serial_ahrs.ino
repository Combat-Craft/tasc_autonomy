/****************************************************************
 * This code is based-on/takes code from: 
 *   - tonitonitonitoni (Antonia)'s "imu_gps_simple.ino" for GPS
 *   - Sparkfun's ICM20948 code "Example1_Basics" for IMU
 *   - Sparkfun's ICM20948 code "Example1_Advanced" for IMU
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
 * Comments from " Mahony AHRS for the ICM_20948  S.J. Remington 6/2021"
 *  Requires the Sparkfun ICM_20948 library
 *  Standard sensor orientation X North (yaw=0), Y West, Z up
 *  magnetometer Y and Z axes are reflected to reconcile with accelerometer.

 *  New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc cross Mag) as the orientation reference vectors
 *  heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 *  Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
 *  Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
 *  or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
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
const unsigned long IMU_PERIOD_MS = 10;    // 100 Hz
const unsigned long GPS_PERIOD_MS = 1000;  // 1 Hz

unsigned long last_imu_time = 0;
unsigned long last_gps_time = 0;

/* =========================
   SENSOR FUSION CONFIG
   ========================= */
// VERY IMPORTANT!

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = DEG_TO_RAD * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {-80.1, 102.1, -12.1};

//Accel scale
float A_B[3]
{   31.06,  -37.96,  394.9};

float A_Ainv[3][3]
{ { 0.06094 , 0.00028 , 0.00029 },
  { 0.00028 , 0.06164 , -0.00101 },
  { 0.00029 , -0.00101 , 0.06007 }
};

//Mag scale
float M_B[3]
{ 57.16 , 566.41 , 229.72};

float M_Ainv[3][3]
{ { 2.90738 , -0.14758 , -0.00668 },
  { -0.14758 , 3.29168 , 0.0226 },
  { -0.00668 , 0.0226 , 2.4961 }
};

// local magnetic declination in degrees
float declination = 10.05;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now_micro = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output


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

      // MAHONY 
      loop_counter++;
      get_scaled_IMU(Gxyz, Axyz, Mxyz, ANxyz, MNxyz);

      // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0
      Mxyz[1] = -Mxyz[1]; //reflect Y and Z
      Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

      // print calibrated IMU data
      Serial.print("ICAL,");
      Serial.print(now_milli); Serial.print(",");
      Serial.print("ACC,"); // for testing
      Serial.print(Axyz[0],  6); Serial.print(",");
      Serial.print(Axyz[1],  6); Serial.print(",");
      Serial.print(Axyz[2],  6); Serial.print(",");
      Serial.print("GYR,"); // for testing
      Serial.print(Gxyz[0], 6); Serial.print(",");
      Serial.print(Gxyz[1], 6); Serial.print(",");
      Serial.print(Gxyz[2], 6); Serial.print(",");
      Serial.print("MAG,"); // for testing
      Serial.print(Mxyz[0], 6); Serial.print(",");
      Serial.print(Mxyz[1], 6); Serial.print(",");
      Serial.print(Mxyz[2], 6); Serial.print(",");

      // get deltat for the AHRS only, this is not related to print timers!
      now_micro = micros();
      deltat = (now_micro - last) * 1.0e-6; //seconds since last update
      last = now_micro;
  
      //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
      MahonyQuaternionUpdate(ANxyz[0], ANxyz[1], ANxyz[2], 
                              Gxyz[0],  Gxyz[1],  Gxyz[2],
                             MNxyz[0], MNxyz[1], MNxyz[2], deltat);

      // Define Tait-Bryan angles. Strictly valid only for approximately level movement
      
      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
            
      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
      
      //roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      //pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= RAD_TO_DEG;
      //pitch *= RAD_TO_DEG;
      //roll *= RAD_TO_DEG;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      //     Serial.print("ypr ");
      //Serial.print(yaw, 0);
      //Serial.print(", ");
      //Serial.print(pitch, 0);
      //Serial.print(", ");
      //Serial.print(roll, 0);
      //          Serial.print(", ");  //prints 49 in 300 ms (~160 Hz) with 8 MHz ATmega328
      //          Serial.print(loop_counter);  //sample & update loops per print interval
      loop_counter = 0;
      Serial.print(yaw, 1); // only 1 decimal point cause it doesnt need to be that accurate
      Serial.println();
    
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


//  MAHONY SENSOR FUSION HELPER FUNCTIONS

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

void copy(float* src, float* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}


// NOTE: for loops have 1 line, thats why they look like that
// function to subtract offsets and apply scale/correction matrices to IMU data
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3], float ANxyz[3], float MNxyz[3]) {
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
  copy(Axyz, ANxyz, 3);
  vector_normalize(ANxyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++){ 
    temp[i] = (Mxyz[i] - M_B[i]);
  }
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  copy(Mxyz, MNxyz, 3);
  vector_normalize(MNxyz);
}


// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
