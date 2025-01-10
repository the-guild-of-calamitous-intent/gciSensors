/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <cstdint> // int types

#if !defined(i2c_default)
  #warning "// i2c not enabled //"
#endif

#ifndef I2C_BUS_DEFINES
#define I2C_BUS_DEFINES
constexpr bool I2C_HOLD_BUS    = true;
constexpr bool I2C_RELEASE_BUS = false;
#endif


class SensorI2C {
protected:
  const uint8_t addr;
  i2c_inst_t *i2c;

public:
  // TwoWire class in picolib (gci_pico) setups the i2c0/1 so this
  // class doesn't have to. This class just does i2c read/write.
  //
  // addr: address of sensor
  // port: using i2c bus 0 or 1
  SensorI2C(uint8_t addr, const uint32_t port) : addr(addr) {
    if (port == 0) i2c = &i2c0_inst;
    else if (port == 1) i2c = &i2c1_inst;
  }
  ~SensorI2C() {}

  bool writeRegister(const uint8_t reg, const uint8_t data) {
    uint8_t out[2]{reg, data};
    i2c_write_blocking(i2c, addr, out, 2, I2C_RELEASE_BUS);
    return true;
  }

  bool writeRegister(const uint8_t reg, const uint32_t data_size, const uint8_t* data) {
    // uint8_t out[2]{reg, data};
    uint8_t *out = (uint8_t*)malloc(data_size + 1);
    out[0] = reg;
    memcpy(&out[1], data, data_size);
    i2c_write_blocking(i2c, addr, out, data_size, I2C_RELEASE_BUS);
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
  // int16_t readRegister(const uint8_t reg) {
  // int8_t readRegister(const uint8_t reg) {
  //   uint8_t value;
  //   // return (readRegisters(reg, 1, &value) == true) ? value : 0;
  //   if (readRegisters(reg, 1, &value)) return value;
  //   return 0;
  // }

  inline
  bool readRegister(const uint8_t reg, uint8_t *data) {
    return readRegisters(reg, 1, data);
  }

  inline size_t available() { return i2c_get_read_available(i2c); }
};
