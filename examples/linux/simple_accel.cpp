#define USE_GCIMOCK_SERIAL 1

#include <mock_wire.hpp>
#include <mock_serial.hpp>
#include <mock_serial.hpp>
SerialPort Serial;
SerialPort Serial1;
TwoWire Wire;

#include <gciSensors.hpp>
#include <iostream>
#include <string>

// using namespace std;
using namespace LSM6DSOX;

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

enum data_type {
  ACCEL,
  GYRO,
  TEMP,
  COMPFILTER,
  AGT
};


// ComplementaryFilter rpycf;

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!IMU.init()) delay (10);

  Serial.println("setup done ...");
}

void loop() {
  int val = AGT;
  sox_t s = IMU.read();

  if (s.ok == false) {
    Serial.println("crap: " + to_string(s.ax));
    return;
  }

  if (val == ACCEL) {
    Serial.print(s.ax);
    Serial.print("\t");
    Serial.print(s.ay);
    Serial.print("\t");
    Serial.println(s.az);
  }
  else if (val == GYRO) {
    Serial.print(s.gx);
    Serial.print("\t");
    Serial.print(s.gy);
    Serial.print("\t");
    Serial.println(s.gz);
  }
  else if (val == TEMP) {
    Serial.println(s.temp);
  }
  // else if (val == COMPFILTER) {
  //   rpy_t rpy = rpycf.update(s);

  //   Serial.print(rpy.r);
  //   Serial.print("\t");
  //   Serial.print(rpy.p);
  //   Serial.print("\t");
  //   Serial.println(rpy.y);
  // }
  else if (val == AGT) {
    Serial.print(s.ax,6);
    Serial.print("\t");
    Serial.print(s.ay,6);
    Serial.print("\t");
    Serial.print(s.az,6);
    Serial.print("\t");
    Serial.print(s.gx,6);
    Serial.print("\t");
    Serial.print(s.gy,6);
    Serial.print("\t");
    Serial.print(s.gz,6);
    Serial.print("\t");
    Serial.print(s.temp,3);
    Serial.print("\t");
    Serial.println(millis());
  }

  delay(5);
}

int main() {
  setup();
  while (true) {
    loop();
    delay(1000);
  }

  return 0;
}