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

using namespace BMP390;
using namespace gci::sensors;

TwoWire tw;
gciBMP390 bmp(0);

const uint LED_PIN = 25;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// Press/Temp Started ///\n");

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  while (true) {
    uint err = bmp.init(
      ODR_100_HZ,
      IIR_FILTER_COEFF_127);
    if (err == 0) break;
    printf("BMP Error: %u\n", err);
    sleep_ms(1000);
  }

  while (1) {
    // gpio_put(LED_PIN, 0);
    // sleep_ms(500);
    // gpio_put(LED_PIN, 1);
    // sleep_ms(500);

    bmp390_t ans = bmp.read();
    if (ans.ok == false) {
      printf("oops ...\n");
      sleep_ms(100);
      continue;
    }

    float alt = pressure_altitude(ans.pressure);
    printf("Press: %8.1f Pa  Temp: %5.2f C  Alt: %7.1f m\n",
      ans.pressure,
      ans.temperature,
      alt);

    sleep_ms(10);
  }
}