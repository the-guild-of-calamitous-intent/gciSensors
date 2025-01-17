#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>
#include "two_wire.hpp"

using namespace std;
using namespace DPS310;
using namespace sensors;

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = 1;
constexpr uint i2c_sda = 0;


int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint speed = i2c_bus_init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  gciDPS310 press(i2c_port);

  uint8_t err;
  while (1) {
    err = press.init(DPS_128HZ);
    if (err == 0) break;
    printf("... oops\n");
    sleep_ms(100);
  }

  printf("/// Press/Temp Started ///\n");

  while (1) {
    dps310_t ans = press.read();
    if (ans.ok == false) {
      sleep_ms(10);
      printf(".");
      continue;
    }
    printf("\n");

    float alt = pressure_altitude(ans.pressure);
    printf("Press: %8.1f Pa  Temp: %5.2f C  Alt: %7.1f m ",
      ans.pressure,
      ans.temperature,
      alt);
    sleep_ms(33);
  }
}