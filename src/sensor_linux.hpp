/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <Wire.hpp>
#include <stdint.h> // int types

class SensorI2C {
public:
  SensorI2C(const uint8_t address) : addr(address) {}

  void init_tw(const uint32_t baud) {
    i2c = nullptr;
  }

  bool writeRegister(const uint8_t reg, const uint8_t data) {
    i2c->set(addr);
    return i2c->write(reg, data);
  }

  bool readRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {
    i2c->set(addr);
    return i2c->read(reg, count, data);
  }

  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    if (!readRegisters(reg, 1, &value)) return 0;
    return value;
  }

  const uint8_t addr;

protected:
  TwoWire *i2c;
};
