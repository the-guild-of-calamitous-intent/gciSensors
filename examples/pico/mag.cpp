#include <stdio.h>
#include <string>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#include <gciSensors.hpp>


using namespace LIS3MDL;
using namespace gci::sensors;

gciLIS3MDL mag;

const uint LED_PIN = 25;

int main() {
  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  mag.init_tw(I2C_400KHZ,0,I2C0_SDA_PIN, I2C0_SCL_PIN);

  while (true) {
    int err = mag.init(RANGE_4GAUSS,ODR_155HZ);
    if (err == 0) break;
    puts("mag error");
    sleep_ms(1000);
  }

  while (1) {
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    gpio_put(LED_PIN, 1);
    sleep_ms(500);

    const mag_t m = mag.read_cal();
    if (m.ok == false) continue;
    string mags = to_string(m.x) + " " + to_string(m.y) + " " + to_string(m.z);
    puts(mags.c_str());
  }
}