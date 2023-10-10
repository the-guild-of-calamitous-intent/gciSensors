
#include <Wire.h>
#include <gciSensors.hpp>

using namespace LSM6DSOX;
using namespace gci::sensors;

Hertz hz(30);

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (true) {
    uint8_t err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) break;
    Serial.print("imu error: ");
    Serial.println(err);
    delay (1000);
  }

  Serial.println("///// START /////");
}

void loop() {
  sox_t s = IMU.read_cal();
  // since the imu is set to operate at ~100Hz, s.ok will only be valid
  // when good data is available. Thus this loop will run print at 
  // ~100 Hz
  // Serial.println(".");
  if (!s.ok) return;

  if (hz.check()) {
    Serial.print(hz.hertz);
    Serial.print(": ");
    Serial.print(s.a.x,6);
    Serial.print(" ");
    Serial.print(s.a.y,6);
    Serial.print(" ");
    Serial.print(s.a.z,6);
    Serial.print(" g's ");
    Serial.print(s.g.x,6);
    Serial.print(" ");
    Serial.print(s.g.y,6);
    Serial.print(" ");
    Serial.print(s.g.z,6);
    Serial.print(" dps  ");
    Serial.print(s.temp,2);
    Serial.print(" C  time: ");
    Serial.println(25e-6*s.ts);
  }
}
