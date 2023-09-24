
#include <Wire.h>
#include <gciSensors.hpp>

using namespace LSM6DSOX;

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ)) {
    Serial.println("imu error");
    delay (1000);
  }
}

uint32_t looptime = 0;

void loop() {
  sox_t s = IMU.read();
  // since the imu is set to operate at ~100Hz, s.ok will only be valid
  // when good data is available. Thus this loop will run print at 
  // ~100 Hz
  if (!s.ok) return;

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
