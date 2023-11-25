/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <cstdint> // int types
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#if !defined(i2c_default)
  #warning "// i2c not enabled //"
#endif

constexpr uint I2C0_SDA_PIN = 4;
constexpr uint I2C0_SCL_PIN = 5;
constexpr uint I2C1_SDA_PIN = 14;
constexpr uint I2C1_SCL_PIN = 15;
// constexpr uint I2C_100KHZ = 100 * 1000UL;
// constexpr uint I2C_400KHZ = 400 * 1000UL;
constexpr bool I2C_HOLD_BUS    = true;
constexpr bool I2C_RELEASE_BUS = false;

/*
This is NOT meant to replicate Arduino's TwoWire/Wire library
but do the bare minimum to setup I2C.

If you use both i2c0 and i2c1, this class can initialize both
of those with one instance if you want since it doesn't have
any member variables.

TwoWire tw;
tw.init(0, 400000, 8, 9);
*/
class TwoWire {
public:
  TwoWire() {}
  ~TwoWire() {}

  uint init(const uint port, const uint baud, const uint pin_sda,
            const uint pin_scl) {

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);

    if (port == 0) return i2c_init(&i2c0_inst, baud);
    if (port == 1) return i2c_init(&i2c1_inst, baud);

    return 0;
  }
};

class SensorI2C {
  const uint8_t addr;
  i2c_inst_t *i2c;

public:
  SensorI2C(uint8_t addr, const uint32_t port) : addr(addr) {
    if (port == 0) {
      i2c = &i2c0_inst;
    }
    else if (port == 1) {
      i2c = &i2c1_inst;
    }
  }
  ~SensorI2C() {}

  bool writeRegister(const uint8_t reg, const uint8_t data) {
    uint8_t out[2]{reg, data};
    i2c_write_blocking(i2c, addr, out, 2, I2C_RELEASE_BUS);
    return true;
  }

  bool readRegisters(const uint8_t reg, const size_t data_size,
                     uint8_t *const data) {
    i2c_write_blocking(i2c, addr, &reg, 1, I2C_HOLD_BUS);
    int ret = i2c_read_blocking(i2c, addr, data, data_size, I2C_RELEASE_BUS);
    return (ret < 0) ? false : true;
  }

  // inline
  // Don't like this because "as is" there is no way to return an error
  // if it occurs.
  uint8_t readRegister(const uint8_t reg) {
    uint8_t value;
    // return (readRegisters(reg, 1, &value) == true) ? value : 0;
    if (readRegisters(reg, 1, &value)) return value;
    return 0;
  }

  inline size_t available() { return i2c_get_read_available(i2c); }
};

