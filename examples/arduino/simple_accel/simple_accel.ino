#include <gciSensors.hpp>

using namespace LSM6DSOX;

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

enum data_type {
  ACCEL,
  GYRO,
  TEMP,
  AGT
};


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
  int val = AGT;
  sox_t s = IMU.read();
  Serial.print(s.ok? "good ":"error ");

  if (val == ACCEL) {
    Serial.print(s.ax);
    Serial.print('\t');
    Serial.print(s.ay);
    Serial.print('\t');
    Serial.println(s.az);
  }
  else if (val == GYRO) {
    Serial.print(s.gx);
    Serial.print('\t');
    Serial.print(s.gy);
    Serial.print('\t');
    Serial.println(s.gz);
  }
  else if (val == TEMP) {
    Serial.println(s.temp);
  }

  else if (val == AGT) {
    Serial.print(s.ok ? "good " : "bad ");
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
    Serial.println(millis());
  }

  delay(5);
}
