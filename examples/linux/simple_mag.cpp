#define USE_GCIMOCK_SERIAL 1

#include <mock_arduino.hpp>
#include <mock_wire.hpp>
#include <mock_serial.hpp>
SerialPort Serial;
SerialPort Serial1;
TwoWire Wire;


#include <gciSensors.hpp>
#include <iostream>
#include <string>

using namespace LIS3MDL;

gciLIS3MDL mag(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!mag.init()) delay(10);
}

void loop() {

  mag_t m = mag.read();

  if (m.ok) {
    Serial.print(m.x);
    Serial.print("\t");
    Serial.print(m.y);
    Serial.print("\t");
    Serial.print(m.z);
    Serial.print("\t");
    Serial.println((m.ok)? 1 : -1);
  }
}

int main() {
  setup();
  while (true) {
    loop();
    delay(100);
  }
  return 0;
}
