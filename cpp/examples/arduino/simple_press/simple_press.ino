
#include <gciSensors.hpp>

using namespace BMP390;
using namespace gci::sensors;

gciBMP390 bmp(&Wire);
Hertz hz(30);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("setup: bmp390 init");

  uint8_t err;
  do {
    err = bmp.init(ODR_25_HZ);
    sleep_ms(1000);
    Serial.println("ERROR: gciBMP390::init()");
  } while (err != NO_ERROR);

  Serial.println("setup done ...");
}

void loop() {
  // Serial.print(".");

  pt_t pt = bmp.read();
  if (pt.ok == false) return;

  if (hz.check()) {
    Serial.print(hz.hertz);
    Serial.print(" hz\t");
    Serial.print(pt.press);
    Serial.print(" Pa\t");
    Serial.print(bmp.altitude(pt.press));
    Serial.print(" m\t");
    Serial.print(pt.temp);
    Serial.println(" C");
  }

  // sleep_ms(40);
}
