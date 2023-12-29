#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>
#include "two_wire.hpp"

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = I2C0_SCL_PIN;
constexpr uint i2c_sda = I2C0_SDA_PIN;

using namespace gci::sensors;
using namespace PA1010D;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  TwoWire tw;
  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  gciPA1010D gps(PA_ADDR, i2c_port); // default is 0, so don't need to do this
  char init_command[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
  gps.write(init_command, sizeof(init_command));

  printf("/// GPS Started ///\n");

  char nema[250];

  while (1) {
    gps.read(nema);
    printf("GPS: %s\n", nema);
  }
}