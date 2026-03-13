/****************************************************************
 * This code is based on Sparkfun's ICM20948 code "Example10_DMP_FastMultipleSensors.ino" (IMU) and imu_gps_simple.ino by Antonia.
 *
 * The following comment is from the IMU example:
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 ***************************************************************/

// #include <Wire.h> // removed as already defined in firmware/src/ICM_20948.h

/* =========================
   GPS CONFIG
   ========================= */
#include <TinyGPS++.h> // moved to GPS CONFIG section
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 38400

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

/* =========================
   IMU CONFIG
   ========================= */
#include "src/ICM_20948.h" //example has the library set up differently, but let's just avoid the hassle...

// assuming these are the ESP32 pins
#define MPU_ADDR 0x68
#define SDA_PIN 21
#define SCL_PIN 22

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

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("# GPS serial started");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  wake_mpu();
  calibrate_imu();

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
