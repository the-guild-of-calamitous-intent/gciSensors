#include <stdio.h>
#include <gciSensors.hpp>

using namespace std;

using namespace LSM6DSOX;
// using namespace gci;
// using namespace gci::sensors;
using namespace sensors;

gciLSM6DSOX IMU(1);

const uint LED_PIN = 25;

int main() {
  printf("/// Accel/Gyros START ///\n");

  while (true) {
    uint8_t err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) break;
    printf("imu error %d", int(err));
    sleep_ms(1000);
  }

  while (1) {
    lsm6dsox_t i = IMU.read();
    if (i.ok == false) continue;

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f rps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temperature);
    printf("Timestamp: %u msec\n", i.timestamp);

  }

  return 0;
}