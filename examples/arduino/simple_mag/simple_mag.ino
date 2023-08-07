
#include <gciSensors.hpp>

using namespace LIS3MDL;

gciLIS3MDL mag(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!mag.init()) delay(10);

  float sm[12]{
    0.864, 0.0, 0.0, 20.553,  
    0.0, 0.893, 0.0, 16.331,
    0.0, 0.0, 0.988, 30.067};
  mag.set_cal(sm);
}

void loop() {

  const mag_t m = mag.read_cal();

  Serial.print(m.x);
  Serial.print("\t");
  Serial.print(m.y);
  Serial.print("\t");
  Serial.print(m.z);
  Serial.print(" uT \t");
  Serial.println((m.ok)? 1 : -1);
}
