#include <gciSensors.hpp>

using namespace LSM6DSOX;

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!IMU.init()) {
    Serial.println("imu error");
    delay (1000);
  }
}

void loop() {
  sox_t s = IMU.read();
  Serial.print(s.ok? "good ":"error ");
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
  Serial.print(" rps  ");
  Serial.print(s.temp,2);
  Serial.print(" ");
  Serial.println(s.ts);

  // delay(5);
  delay(200);
}
