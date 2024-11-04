#pragma once

#include <stdint.h>
#include "hardware/spi.h"

class SensorSPI {
public:
  SensorSPI(uint32_t port) {
    if (port == 0) spi = spi0;
    else if (port == 1) spi = spi1;
  }

  void set_cs(uint32_t cs) { CS = cs; }

protected:
  spi_inst_t *spi{nullptr};
  uint32_t CS{0};

  inline void spi_cs(bool select) {
    gpio_put(CS, select ? 0 : 1); // Active low
  }

  void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // bool r = spi_is_readable(spi);
    // bool w = spi_is_writable(spi);

    // https://github.com/betaflight/betaflight/blob/c545435f2e1b2561085bbda6c387424db4c383b7/src/main/drivers/bus.c#L123
    // BF ORs with 0x80 also
    reg |= 0x80;

    spi_cs(true);
    // printf("write: %s\n", w ? "true" : "false");
    spi_write_blocking(spi, &reg, 1);
    sleep_ms(10);
    // printf("read: %s\n", r ? "true" : "false");
    spi_read_blocking(spi, 0, buf, len);

    // spi_write_read_blocking(spi, &reg, buf, 1); doesn't work

    spi_cs(false);
    sleep_ms(10);

    // printf("Sent: 0x%02x, Received: 0x%02x\n", reg, buf[0]);
  }

  void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7f; // remove read bit as this is a write
    buf[1] = data;
    spi_cs(true);
    spi_write_blocking(spi, buf, 2);
    spi_cs(false);

    sleep_ms(10);
  }
};