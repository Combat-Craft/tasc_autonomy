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
/* Set up IMU with Digital Low-Pass Filter (DLPF) enabled
 * Based on Example2_Advanced.ino from SparkFun ICM_20948 library
 */
void setup_imu(){
  bool initialized = false;
  
  while (!initialized){
    myICM.begin(WIRE_PORT, AD0_VAL);  // Note: use WIRE_PORT, not "Wire"

    Serial.print(F("Initialization of the IMU sensor returned: "));
    Serial.println(myICM.statusString());
     
    if (myICM.status != ICM_20948_Stat_Ok){
      Serial.println("Trying IMU again...");
      delay(500);
    }
    else{
      initialized = true;
    }
  }

  Serial.println("IMU I2C Device connected!");

  // Software reset to make sure device starts in known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("# IMU Software Reset returned: "));
    Serial.println(myICM.statusString());
  }
  delay(250);

  // Wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // ============================================
  // NEW: DLPF CONFIGURATION (from Example2_Advanced)
  // ============================================
  
  // Set sample mode to continuous
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                       ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(myICM.statusString());
  }
  
  // Set full scale ranges
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm16;      // Accelerometer: ±16g range
  myFSS.g = dps250;     // Gyroscope: ±250 dps range (matches your Gscale)
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(myICM.statusString());
  }
  
  // Configure Digital Low-Pass Filter
  ICM_20948_dlpcfg_t myDLPcfg;
  // Accelerometer DLPF - choose one:
  // acc_d473bw_n499bw (less filtering), acc_d246bw_n265bw, acc_d111bw4_n136bw, 
  // acc_d50bw4_n68bw8, acc_d23bw9_n34bw4, acc_d11bw5_n17bw, acc_d5bw7_n8bw3
  myDLPcfg.a = acc_d111bw4_n136bw;   // 111 Hz bandwidth - good balance
  
  // Gyroscope DLPF - choose one:
  // gyr_d361bw4_n376bw5, gyr_d196bw6_n229bw8, gyr_d151bw8_n187bw6,
  // gyr_d119bw5_n154bw3, gyr_d51bw2_n73bw3, gyr_d23bw9_n35bw9,
  // gyr_d11bw6_n17bw8, gyr_d5bw7_n8bw9
  myDLPcfg.g = gyr_d119bw5_n154bw3;  // 119 Hz bandwidth - good balance
  
  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(myICM.statusString());
  }
  
  // Enable the DLPF
  myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  // ============================================
  // END OF DLPF CONFIGURATION
  // ============================================

  // Ensure the magnetometer starts
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("# IMU: startupMagnetometer returned: "));
    Serial.println(myICM.statusString());
  }

  Serial.println();
  Serial.println(F("# IMU with Hardware DLPF Configuration complete!"));
   
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




Great question! Here's how to test your updated firmware with the DLPF enabled.

## Step 1: Upload the Firmware to ESP32

```bash
# In Arduino IDE:
# 1. Select your ESP32 board (Tools → Board → ESP32 Dev Module)
# 2. Select the correct COM port
# 3. Click Upload (→) button
```

## Step 2: Test the Serial Output

After upload, open the **Serial Monitor** (Tools → Serial Monitor) at **115200 baud**.

You should see output like:
```
# GPS serial started
IMU I2C Device connected!
# IMU with Hardware DLPF Configuration complete!
# IMU + GPS streaming started
IMU,1234,0.01,-0.02,9.81,0.001,0.002,0.001,15.2,-20.1,-30.5
IMU,1254,0.02,-0.01,9.80,0.002,0.001,0.002,15.1,-20.0,-30.4
GPS,1300,43.6532,-79.3832,76.5,0.23,1.2,8,1
...
```

## Step 3: Quick Visual Test

Move the IMU around and watch the Serial Monitor:
- **Shake it** → Values should change but be smoother than before
- **Rotate it** → Gyro values should respond
- **Tilt it** → Accelerometer values should change

## Step 4: Test in ROS

Once the firmware is running, test with your ROS node:

```bash
# Terminal 1: Run your IMU node
ros2 run your_package imu_node.py

# Terminal 2: Check if data is coming through
ros2 topic echo /imu/data_raw --once

# Terminal 3: Check the filtered output (if running EKF)
ros2 topic echo /odometry/filtered --once
```

## Step 5: Compare With/Without DLPF

To see the difference, you can temporarily comment out the DLPF enable lines:

```cpp
// Temporarily disable DLPF to compare
// myICM.enableDLPF(ICM_20948_Internal_Acc, true);
// myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
```

Then re-upload and compare the serial output noise levels.

## Quick Diagnostic Commands

```bash
# Check if ESP32 is sending data
ls /dev/ttyUSB*  # Linux
ls /dev/cu.usbserial*  # Mac

# View raw serial data (Linux/Mac)
screen /dev/ttyUSB0 115200

# Or use minicom
minicom -D /dev/ttyUSB0 -b 115200
```

## What to Look For

**With DLPF enabled:**
- Smoother accelerometer values
- Less gyro drift
- Reduced high-frequency noise

**If DLPF is working correctly:**
- You'll see the message `# IMU with Hardware DLPF Configuration complete!`
- No error messages about setSampleMode, setFullScale, or setDLPcfg

