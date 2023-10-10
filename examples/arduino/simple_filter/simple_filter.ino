
#include <Wire.h>
#include <gciSensors.hpp>
#include "MadgwickAHRS.h"
#include "rate.hpp"

using namespace LSM6DSOX;
using namespace LIS3MDL;
using gci::sensors::vecf_t;

gciLIS3MDL mag(&Wire);
gciLSM6DSOX IMU(&Wire);
gci::sensors::Hertz hz(500);

// float ac[12]{
//    1.00268927, -0.00056029, -0.00190925, -0.00492348,
//   -0.00138898,  0.99580818, -0.00227335,  0.00503835,
//   -0.01438271,  0.00673172,  0.9998954 , -0.01364759};
// float gc[12]{
//   1,0,0,-0.00889949, 
//   0,1,0,-0.00235061, 
//   0,0,1,-0.00475294};

// these values are only good for the magnetometer
// currently being used and cannot be applied to 
// just any magnetometer
float mc[12]{
  0.017661,0.000000,0.000000,-0.592696,
  0.000000,0.019208,0.000000,0.149252,
  0.000000,0.000000,0.017813,-0.079242};

void flatcal(size_t num) {
  Serial.println("/// GYRO CALIBRATION ///");
  float axsum=0;
  float aysum=0;
  float azsum=0;
  float gxsum=0;
  float gysum=0;
  float gzsum=0;
  size_t count = 0;
  while (true) {
    const sox_t s = IMU.read_cal();
    if (s.ok == false) continue;
    axsum += s.a.x;
    aysum += s.a.y;
    azsum += s.a.z;
    gxsum += s.g.x;
    gysum += s.g.y;
    gzsum += s.g.z;
    if (count++ == num) break;
    Serial.print(".");
    Serial.flush();
  }

  float acal[12]{ // gyro scale/bias
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  };

  float gcal[12]{ // gyro scale/bias
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  };

  acal[3] = axsum/num;
  acal[7] = aysum/num;
  acal[11] = azsum/num - 1.0f;

  IMU.set_acal(acal);
  Serial.print("\n\n// accel: ");
  Serial.print(acal[3]);
  Serial.print(" ");
  Serial.print(acal[7]);
  Serial.print(" ");
  Serial.println(acal[11]);

  gcal[3] = gxsum/num;
  gcal[7] = gysum/num;
  gcal[11] = gzsum/num;

  IMU.set_gcal(gcal);
  Serial.print("// gyro: ");
  Serial.print(gcal[3]);
  Serial.print(" ");
  Serial.print(gcal[7]);
  Serial.print(" ");
  Serial.println(gcal[11]);
}

// This should allow rotation of the magnetic field into the 
// horizontal ground plane, then properly calculate the sensed
// Earth's magnetic field
const vecf_t stablize(vecf_t a, vecf_t m, bool degrees=true) {
  vecf_t ret{0};
  ret.ok = false;
  if (!a.normalize()) return ret;

  ret.x = atan2(a.y, a.z);
  ret.y = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z));

  if (!m.normalize()) return ret;

  // WARNING: ahrs.readthedocs switches symbols for roll/pitch compared
  //          to other authors ... below is correct
  float cr = cos(ret.x); // roll
  float sr = sin(ret.x);
  float cp = cos(ret.y); // pitch
  float sp = sin(ret.y);
  ret.z = atan2(
    m.z*sp - m.y*cp,
    m.x*cr + sr * (m.y * sp + m.z * cp)
  ); // yaw

  if (degrees) {
    ret.x *= 180.0 / M_PI;
    ret.y *= 180.0 / M_PI;
    ret.z *= 180.0 / M_PI;
  }
  ret.ok = true;

  return ret;
}

// So math is backwards from a compass, so we have to
// account for CCW being positive in math, BUT CW 
// being positive for a compass. Also the math functions
// give answers from -180 to +180 so we have to shift 
// that to 0 to 360. This only changes the heading value.
const vecf_t to_compass_degrees(const vecf_t& v) {
  vecf_t c = v;
  c.z *= -1;
  if (c.z < 0.0) c.z = 360 + c.z;
  return c;
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  // while (true) {
  //   int err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ);
  //   if (err == 0) break;
  //   Serial.print("accel/gyro error: ");
  //   Serial.println(err);
  //   delay (1000);
  // }

  // flatcal(200); // not sure this helps, need to do a better calibration

  while (true) {
    int err = mag.init(RANGE_4GAUSS, ODR_1000HZ);
    if (err == 0) break;
    Serial.print("mag error: ");
    Serial.println(err);
    delay (1000);
  }

  // IMU.set_acal(ac);
  // IMU.set_gcal(gc);
  mag.set_cal(mc);
}


vecf_t accels;
vecf_t mags;

void loop() {
  // const sox_t s = IMU.read_cal();
  
  // this is the slowest
  // since the imu is set to operate at ~100Hz, s.ok will only be valid
  // when good data is available. Thus this loop will run print at 
  // ~100 Hz
  // if (s.ok) accels = s.a; 
  // if (s.ok == false) return;

  const mag_t m = mag.read(); // 155Hz
  if (m.ok == false) return;

  // vecf_t v = stablize(accels,mags, true);
  // if (v.ok == false) return;
  // v = to_compass_degrees(v);
  vecf_t v = m;

  if (hz.check()) {
    Serial.print(hz.hertz,1);
    Serial.print(" ");
    Serial.print("rpy: ");
    Serial.print(v.x);
    Serial.print("\t");
    Serial.print(v.y);
    Serial.print("\t");
    Serial.print(v.z);
    Serial.println(" ");
  }

  // delay(5);
}