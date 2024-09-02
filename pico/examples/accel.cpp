#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>
#include "two_wire.hpp"

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = 1; // I2C0_SCL_PIN;
constexpr uint i2c_sda = 0; // I2C0_SDA_PIN;

using namespace LSM6DSOX;
using namespace gci::sensors;

TwoWire tw;

// if not using default address or port
// gciLSM6DSOX IMU(LSM6DSOX_ADDRESS,i2c_port);
gciLSM6DSOX IMU;

const uint LED_PIN = 25;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  puts("/// Accel/Gyros START ///\n");

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

    lsm6dsox_t i = IMU.read();
    if (i.ok == false) continue;

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temperature);
    printf("Timestamp: %lu msec\n", i.ts);

  }
}