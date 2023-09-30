
#include <Wire.h>
#include <gciSensors.hpp>
#include "rate.hpp"

using namespace BMP390;
using namespace LSM6DSOX;
using namespace LIS3MDL;

constexpr int SIGFIG = 6;

gciBMP390 bmp(&Wire);
gciLIS3MDL mag(&Wire);
gciLSM6DSOX IMU(&Wire);

Hertz hz(100);

// float ac[12]{
//    1.00268927, -0.00056029, -0.00190925, -0.00492348,
//   -0.00138898,  0.99580818, -0.00227335,  0.00503835,
//   -0.01438271,  0.00673172,  0.9998954 , -0.01364759};
// float gc[12]{
//   1,0,0,-0.00889949, 
//   0,1,0,-0.00235061, 
//   0,0,1,-0.00475294};
// float mc[12]{
//   0.017661,0.000000,0.000000,-0.592696,
//   0.000000,0.019208,0.000000,0.149252,
//   0.000000,0.000000,0.017813,-0.079242};

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(100);
  Serial.println("*** ARDUINO START ***");

  Wire.begin();
  Wire.setClock(400000);

  while (true) {
    // 104
    sox_error err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ);
    if (err == sox_error::ERROR_NONE) break;
    Serial.print("* LDS6SOX error: ");
    Serial.println(int(err));
    bool ok = IMU.reboot();
    if (ok == false) Serial.println("reboot failed");
    delay (1000);
  }
  Serial.println("* LDS6SOX Good");

  // 155
  while (!mag.init(RANGE_4GS, ODR_155HZ)) {
    Serial.println("* LIS3MDL error");
    delay (1000);
  }
  Serial.println("* LIS3MDL Good");

  while (!bmp.init(OS_MODE_PRES_16X_TEMP_2X)) { // 23-26Hz, datasheet pg 26
    delay(2000);
    Serial.println("* BMP390 error");
  }
  Serial.println("* BMP390 Good");

  // IMU.set_acal(ac);
  // IMU.set_gcal(gc);
  // mag.set_cal(mc);
}

void loop() {
  // this is the slowest
  // since the imu is set to operate at ~100Hz, s.ok will only be valid
  // when good data is available. Thus this loop will run print at 
  // ~100 Hz
  const sox_t s = IMU.read_cal(); // 104Hz
  if (s.ok == false) return; 

  const mag_t m = mag.read_cal(); // 155Hz
  if (m.ok == false) return;

  const pt_t pt = bmp.read(); // 23-26Hz, datasheet pg 26
  // if (pt.ok == false) return;

  // start --------------------------
  hz.check();
  Serial.print(hz.hertz);
  Serial.print("|");
  // accel --------------------------
  Serial.print(s.ax,SIGFIG); // g's
  Serial.print(",");
  Serial.print(s.ay,SIGFIG);
  Serial.print(",");
  Serial.print(s.az,SIGFIG);
  Serial.print("|");
  // gyro ---------------------------
  Serial.print(s.gx,SIGFIG); // dps
  Serial.print(",");
  Serial.print(s.gy,SIGFIG);
  Serial.print(",");
  Serial.print(s.gz,SIGFIG);
  Serial.print(",");
  Serial.print(s.temp,SIGFIG);
  Serial.print("|");
  // time --------------------------
  Serial.print(static_cast<float>(25e-6*s.ts),SIGFIG); // sec
  // Serial.print(s.ts); // counts
  Serial.print("|");
  // mags ---------------------------
  Serial.print(m.x,SIGFIG); // uT
  Serial.print(",");
  Serial.print(m.y,SIGFIG);
  Serial.print(",");
  Serial.print(m.z,SIGFIG);
  Serial.print("|");
  // pres/temp ----------------------
  Serial.print(pt.press,SIGFIG); // Pa
  Serial.print(",");
  Serial.print(pt.temp,SIGFIG); // C
  // end ----------------------------
  Serial.println("");
}
