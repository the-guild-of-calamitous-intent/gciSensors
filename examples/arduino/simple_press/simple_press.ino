
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

  while (!bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X)) {
    delay(2000);
    // Serial.println("ERROR: gciBMP390::init()");
  }
  for (int i; i< 10; ++i) {
    bmp.read();
    delay(40); // 25 Hz
  }
  // bmp.setOsMode(BMP390::OS_MODE_PRES_16X_TEMP_2X);

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
