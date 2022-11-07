#include <gciSensors.hpp>
#include "Arduino.h"
#include "Wire.h"
#include <iostream>
#include <string>

// using namespace std;
using namespace BMP390;

gciBMP390 bmp(&Wire);

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("bmp390 init start");

  while (!bmp.init()) {
    delay(500);
    Serial.println("something wrong ...");
  }

  Serial.println("setup done ...");
}

void loop() {

  pt_t pt = bmp.read();

  if (pt.ok) {
    Serial.print(pt.press);
    Serial.print(" Pa\t");
    Serial.print(pt.temp);
    Serial.println(" C");
  }
  else Serial.println("crap");

}

int main() {
  setup();
  while (true) {
    loop();
    delay(1000);
  }

  return 0;
}
