// #define USE_GCIMOCK_SERIAL 1

// #include <mock_wire.hpp>
// #include <mock_serial.hpp>
// #include <mock_serial.hpp>
// SerialPort Serial;
// SerialPort Serial1;

#include <gciSensors.hpp>
#include <iostream>
#include <string>

TwoWire Wire;

// using namespace std;
using namespace BMP390;

void delay(int d) {}
gciBMP390 bmp(&Wire);

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  // Serial.println("bmp390 init start");

  while (!bmp.init()) {
    delay(500);
    // Serial.println("something wrong ...");
  }

  // Serial.println("setup done ...");
}

void loop() {

  pt_t pt = bmp.read();

  // if (pt.ok) {
  //   Serial.print(pt.press);
  //   Serial.print(" Pa\t");
  //   Serial.print(pt.temp);
  //   Serial.println(" C");
  // }
  // else Serial.println("crap");

}

int main() {
  setup();
  while (true) {
    loop();
    delay(1000);
  }

  return 0;
}
