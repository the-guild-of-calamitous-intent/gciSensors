#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "tusb.h"
// I2C Bus Scan
//    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 00 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 10 .  .  .  .  .  .  .  .  .  .  .  .  @  .  .  .
// 20 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 30 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 40 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 50 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 60 .  .  .  .  .  .  .  .  .  .  @  .  .  .  .  .
// 70 .  .  .  .  .  .  .  @  .  .  .  .  .  .  .  .


// This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
#define SDA_PIN  0
#define SCL_PIN  1
#define I2C_PORT &i2c0_inst
// #define SDA_PIN  2
// #define SCL_PIN  3
// #define I2C_PORT &i2c1_inst

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}


int main() {
    // Enable UART so we can print status output
    stdio_init_all();

    while (!tud_cdc_connected()) {
      printf("w");
      sleep_ms(500);
    }

#if !defined(I2C_PORT) || !defined(SDA_PIN) || !defined(SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
    return 1;
#endif

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    while (1) {
      printf(".\n");
      sleep_ms(5000);
    }
    return 0;
}