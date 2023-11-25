
#include <gciSensors.hpp>

using namespace LIS3MDL;

gciLIS3MDL mag(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!mag.init(RANGE_4GS, ODR_155HZ)) {
    Serial.println("ERROR");
    delay(1000);
  }

  // float sm[12]{
  //   0.864, 0.0, 0.0, 20.553,  
  //   0.0, 0.893, 0.0, 16.331,
  //   0.0, 0.0, 0.988, 30.067};
  // mag.set_cal(sm);
}

void loop() {

  const mag_t m = mag.read();
  // const mag_t m = mag.read_cal();
  if (m.ok == false) return;

  Serial.print(m.x,6);
  Serial.print(",");
  Serial.print(m.y,6);
  Serial.print(",");
  Serial.println(m.z,6);

  // delay(100);
}
