#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <stdio.h>

// screen /dev/tty.usbmodemXXXXXX
// ctrl-a-d

#define USING_LPS22HB
#define USING_SPI
#define USING_I2C

#include "gciSensors.hpp"

using namespace sensors;

constexpr int SPI_CL = 10;
constexpr int SPI_DO = 11;
constexpr int SPI_DI = 12;
constexpr int SPI_CS = 13;
constexpr int INT    = 16;

volatile bool grab   = false;

void gpio_callback(uint gpio, uint32_t events) { grab = true; }

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  printf("LPS22 Test\n");

  // This example will use SPI1 at 10 MHz.
  spi_init(spi1, LPS22_MAX_SPI_CLK_HZ);

  gpio_set_function(SPI_DI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CL, GPIO_FUNC_SPI);
  gpio_set_function(SPI_DO, GPIO_FUNC_SPI);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1); // CS high is inactive

  // Make the SPI pins available to picotool
  bi_decl(bi_3pins_with_func(SPI_DI, SPI_DO, SPI_CL, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(SPI_CS, "SPI1 CS"));
  // bi_decl(bi_1pin_with_name(INT, "SPI1 Interrupt"));

  LPS22SPI lps22(1);
  lps22.set_cs(SPI_CS);
  bool found = lps22.init();
  if (found) printf("success!\n");
  else printf("failed!!!\n");

  // gpio_init(INT);
  // gpio_set_dir(INT, GPIO_IN);
  gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
  // gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback);
  // gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);
  gpio_pull_down(INT);

  // Sealevel: 1,013.25 hPa
  while (1) {
    // grab = true;
    if (grab) {
      grab = false;
      lps22_t pt = lps22.read();
      float alt  = pressure_altitude(pt.pressure);
      printf("\nPress: %10.3f Temp: %10.3f Alt: %10.3f %lu\n",
        pt.pressure,
        pt.temperature,
        alt, now_ms());
    }
    else printf(".");
    sleep_ms(100);
  }

  return 0;
}