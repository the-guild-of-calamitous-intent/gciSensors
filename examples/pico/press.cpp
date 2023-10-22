#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB

#include <gciSensors.hpp>

using namespace BMP390;
using namespace gci::sensors;

gciBMP390 bmp;

const uint LED_PIN = 25;

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  printf("/// Press/Temp Started ///\n");

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  bmp.init_tw(I2C_400KHZ,0,I2C0_SDA_PIN, I2C0_SCL_PIN);
  while (true) {
    uint err = bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    if (err == 0) break;
    printf("BMP Error: %u\n", err);
    sleep_ms(1000);
  }

  while (1) {
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    gpio_put(LED_PIN, 1);
    sleep_ms(500);

    pt_t pt = bmp.read();
    if (pt.ok == false) continue;

    printf("-----------------------------\n");
    printf("Pressure: %f Pa\n", pt.press);
    printf("Temperature: %f C\n", pt.temp);
  }
}