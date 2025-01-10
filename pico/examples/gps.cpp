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

using namespace sensors;
using namespace PA1010D;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  // TwoWire tw;
  uint speed = i2c_bus_init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  gciPA1010D gps(0, PA_ADDR); // default is 0, so don't need to do this

  gps.write(PMTK::FULL_POWER, sizeof(PMTK::FULL_POWER));
  gps.write(PMTK::RMCGGAGSA, sizeof(PMTK::RMCGGAGSA));

  printf("///--- GPS Started ---///\n");

  char nema[250];

  while (1) {
    uint32_t num = gps.read(nema,sizeof(nema));
    if (num > 0) printf("GPS[%ld]: %s\n", num, nema);
    else printf("*** Bad read ***\n");
    sleep_ms(100);
  }
}