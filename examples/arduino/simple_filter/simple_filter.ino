
#include <Wire.h>
#include <gciSensors.hpp>
#include "MadgwickAHRS.h"

using namespace LSM6DSOX;
using namespace LIS3MDL;

using gcisensors::vecf_t;

gcisensors::AttitudeA<float> tc;
gcisensors::AttitudeAG<float> tag(0.9);
gcisensors::AttitudeAM<float> tam;
Madgwick ahrs(.1, 833.0);

gciLIS3MDL mag(&Wire);
gciLSM6DSOX IMU(&Wire);

float ac[12]{
   1.00268927, -0.00056029, -0.00190925, -0.00492348,
  -0.00138898,  0.99580818, -0.00227335,  0.00503835,
  -0.01438271,  0.00673172,  0.9998954 , -0.01364759};
float gc[12]{
  1,0,0,-0.00889949, 
  0,1,0,-0.00235061, 
  0,0,1,-0.00475294};
float mc[12]{
  0.96545537,  0,0,-13.15340002,
  0, 0.94936676, 0,29.7714855,
  0,0,0.967698, 0.0645215};

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ)) {
    Serial.println("accel/gyro error");
    delay (1000);
  }

  while (!mag.init(RANGE_4GS, ODR_300HZ)) {
    Serial.println("mag error");
    delay (1000);
  }

  IMU.set_acal(ac);
  IMU.set_gcal(gc);
  mag.set_cal(mc);

  // ahrs.begin(100.0f);
}

uint32_t looptime = 0;

void print_sox(const sox_t& s) {
  uint32_t now = millis();
  Serial.print(1000.0f / float(now - looptime));
  Serial.print("Hz ");
  looptime = now;
  
  Serial.print(s.ax,6);
  Serial.print(" ");
  Serial.print(s.ay,6);
  Serial.print(" ");
  Serial.print(s.az,6);
  Serial.print(" g's ");
  Serial.print(s.gx,6);
  Serial.print(" ");
  Serial.print(s.gy,6);
  Serial.print(" ");
  Serial.print(s.gz,6);
  Serial.print(" dps  ");
  Serial.print(s.temp,2);
  Serial.print(" C  time: ");
  Serial.println(25e-6*s.ts);
}

void print_mag(const mag_t& m) {
  Serial.print(m.x);
  Serial.print("\t");
  Serial.print(m.y);
  Serial.print("\t");
  Serial.print(m.z);
  Serial.print(" uT \t");
  Serial.print(" mag: ");
  Serial.print(m.magnitude());
  Serial.print(" h: ");
  Serial.println(atan2(m.y, m.x)*180.0/M_PI);
}

uint32_t count = 0;
uint32_t epoch = millis();
float hertz = 0;

void loop() {
  const sox_t s = IMU.read();
  if (s.ok == false) return; // this is the slowest
  // since the imu is set to operate at ~100Hz, s.ok will only be valid
  // when good data is available. Thus this loop will run print at 
  // ~100 Hz
  // if (s.ok) print_sox(s); // 104Hz

  // const mag_t m = mag.read(); // 155Hz
  // if (m.ok) print_mag(m);

  // if (s.ok && m.ok) {
  //   ahrs.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, m.x, m.y, m.z);
  //   Serial.print("> ");
  //   Serial.print(ahrs.getRoll());
  //   Serial.print(" ");
  //   Serial.print(ahrs.getPitch());
  //   Serial.print(" ");
  //   Serial.print(ahrs.getYaw());
  //   Serial.println(" ");
  // }
  // else if (s.ok) {
  //   ahrs.updateIMU(s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
  //   Serial.print("> ");
  //   Serial.print(ahrs.getRoll());
  //   Serial.print(" ");
  //   Serial.print(ahrs.getPitch());
  //   Serial.print(" ");
  //   Serial.print(ahrs.getYaw());
  //   Serial.println(" ");
  // }

  // if (m.ok && s.ok) {
  // if (s.ok) {
  if (++count % 300 == 0) {
    uint32_t now = millis();
    hertz = 1000.0f * float(count) / float(now - epoch);
    epoch = now;
    count = 0;
  }

  vecf_t a{s.ax, s.ay, s.az};
  vecf_t g{s.gx, s.gy, s.gz};
  // gcisensors::vecf_t mm{m.x,m.y,m.z};
  // vecf_t rpy = tc.update(a);
  vecf_t rpy = tag.update(a, g, 0.01);
  // vecf_t rpy = tam.update(a);
  
  Serial.print(rpy.x);
  Serial.print(" ");
  Serial.print(rpy.y);
  Serial.print(" ");
  Serial.print(rpy.z);
  Serial.print(" ");
  Serial.print(hertz);
  Serial.println(" ");
  // }
}
