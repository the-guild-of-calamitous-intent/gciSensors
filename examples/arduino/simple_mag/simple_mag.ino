
#include <gciSensors.hpp>

using namespace LIS3MDL;

gciLIS3MDL mag(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (true) {
    int err = mag.init();
    if (err == 0) break;
    Serial.print("error: ");
    Serial.println(err);
    delay(100);
  }

  float mc[12]{
    0.017661,0.000000,0.000000,-0.592696,
    0.000000,0.019208,0.000000,0.149252,
    0.000000,0.000000,0.017813,-0.079242};
  mag.set_cal(mc);
}

void loop() {

  // const mag_t m = mag.read();
  const mag_t m = mag.read_cal();

  Serial.print(m.x);
  Serial.print("\t");
  Serial.print(m.y);
  Serial.print("\t");
  Serial.print(m.z);
  Serial.print(" uT \t");
  Serial.print(" mag: ");
  Serial.print(m.magnitude());
  Serial.print("\t");
  Serial.print(" ok: ");
  Serial.println((m.ok)? 1 : -1);

  delay(100);
}
