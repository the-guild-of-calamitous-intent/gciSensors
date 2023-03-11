
#define GCI_SENSORS_DEBUG 0

#include <gciSensors.hpp>

using namespace BMP390;

gciBMP390 bmp(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("setup: bmp390 init");

  while (!bmp.init()) {
    delay(2000);
    // Serial.println("ERROR: gciBMP390::init()");
  }

  Serial.println("setup done ...");
}

void loop() {

  Serial.println(bmp.ready());
  pt_t pt = bmp.read();

  Serial.print(pt.ok? "Good: " : "Error: ");
  Serial.print(" ");
  Serial.print(pt.press);
  Serial.print(" Pa\t");
  Serial.print(bmp.altitude(pt.press));
  Serial.print(" m\t");
  Serial.print(pt.temp);
  Serial.println(" C");

  delay(200);
}
