#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>

using namespace LSM6DSOX;
using namespace gci::sensors;

// gciLSM6DSOX IMU(LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU;

const uint LED_PIN = 25;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  puts("/// Accel/Gyros START ///\n");

  bool ok = IMU.init_tw(I2C_400KHZ,0,I2C0_SDA_PIN, I2C0_SCL_PIN);
  while (!ok) {
    puts("I2C initialization error");
    sleep_ms(1000);
  }

  while (true) {
    uint8_t err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) break;
    char msg[32];
    snprintf(msg, 32, "imu error %d", int(err));
    msg[31] = '\0';
    puts(msg);
    sleep_ms(1000);
  }

  while (1) {
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    gpio_put(LED_PIN, 1);
    sleep_ms(500);

    sox_t i = IMU.read();
    if (i.ok == false) continue;

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f rps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temp);
    printf("Timestamp: %u msec\n", i.ts);

  }
}