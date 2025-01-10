#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>
#include "two_wire.hpp"

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = 1;
constexpr uint i2c_sda = 0;
const uint LED_PIN = 25;

using namespace LIS3MDL;
// using namespace gci::sensors;


int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint speed = i2c_bus_init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// Mag Started ///\n");

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  gciLIS3MDL mag(i2c_port);

  while (true) {
    int err = mag.init(RANGE_4GAUSS,ODR_155HZ);
    if (err == 0) break;
    printf("mag error: %d\n", err);
    sleep_ms(1000);
  }

  while (1) {
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    gpio_put(LED_PIN, 1);
    sleep_ms(500);

    const lis3mdl_t m = mag.read_cal();
    if (m.ok == false) continue;

    // printf("-----------------------------\n");
    printf("Mags: %f %f %f (normalized)\n", m.x, m.y, m.z);
    // printf("Temperature: %f C\n", m.temperature);
  }
}