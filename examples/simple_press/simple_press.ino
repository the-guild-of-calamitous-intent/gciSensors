#include <gciSensors.hpp>

using namespace BMP390;

gciBMP390 bmp(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  Wire.begin();
  Wire.setClock(400000);

  Serial.print("bmp390 init");

  if (!bmp.init()) delay(500);

  Serial.println("setup done ...");
}

void loop() {

  pt_t pt = bmp.read();

  if (pt.ok) {
    Serial.print(pt.press);
    Serial.print("\t");
    Serial.println(pt.temp);
  }
//  else Serial.println("-1\t-1");
}
