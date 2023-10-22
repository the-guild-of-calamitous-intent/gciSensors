#include <stdio.h>
#include <string>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#include <gciSensors.hpp>

using namespace LSM6DSOX;
using namespace gci::sensors;

// gciLSM6DSOX IMU(LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU;

const uint LED_PIN = 25;

int main() {
  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  IMU.init_tw(I2C_400KHZ,0,I2C0_SDA_PIN, I2C0_SCL_PIN);

  while (true) {
    uint8_t err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) break;
    puts("imu error");
    sleep_ms(1000);
  }

  while (1) {
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    gpio_put(LED_PIN, 1);
    sleep_ms(500);

    sox_t i = IMU.read();
    if (i.ok == false) continue;

    string accel = to_string(i.a.x) + " " + to_string(i.a.y) + " " + to_string(i.a.z);
    puts(accel.c_str());
    string gyro = to_string(i.g.x) + " " + to_string(i.g.y) + " " + to_string(i.g.z);
    puts(gyro.c_str());
  }
}