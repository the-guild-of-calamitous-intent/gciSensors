#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>

using namespace std;
using namespace DPS310;

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = I2C0_SCL_PIN;
constexpr uint i2c_sda = I2C0_SDA_PIN;

TwoWire tw;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  gciDPS310 press;

  bool ok = false;
  while (ok == false) {
    ok = press.init(DPS_128HZ);
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

    float alt = press.altitude(ans.pressure);
    printf("Press: %8.1f Pa  Temp: %5.2f C  Alt: %7.1f m ", ans.pressure, ans.temperature, alt);
    sleep_ms(33);
  }
}