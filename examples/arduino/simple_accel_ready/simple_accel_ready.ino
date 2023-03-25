#include <gciSensors.hpp>


using namespace LSM6DSOX;

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  // uint8_t odr = RATE_12_5_HZ;
  uint8_t odr = RATE_104_HZ;
  // uint8_t odr = RATE_208_HZ;
  uint8_t arange = ACCEL_RANGE_2_G;
  uint8_t grange = GYRO_RANGE_2000_DPS;

  while (!IMU.init(arange,grange,odr)) {
    Serial.println("imu error");
    delay (1000);
  }
}

void loop() {
  bool ok = IMU.ready();

  if (ok) {
    sox_t s = IMU.read();
    // Serial.print(s.ok? "good ":"error ");
    // Serial.print(s.ax,6);
    // Serial.print(" ");
    // Serial.print(s.ay,6);
    // Serial.print(" ");
    // Serial.print(s.az,6);
    // Serial.print(" g's ");
    // Serial.print(s.gx,6);
    // Serial.print(" ");
    // Serial.print(s.gy,6);
    // Serial.print(" ");
    // Serial.print(s.gz,6);
    // Serial.print(" rps  ");
    // Serial.print(s.temp,2);
    // Serial.print(" ");
    Serial.println(s.ts);
  }
  else Serial.print(".");

  // delay(5);
  // delay(1000);
  // Serial.println("loop ---------");
}
