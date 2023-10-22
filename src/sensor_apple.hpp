/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h> // int types
#include <Wire.hpp>

class SensorI2C {
public:
  SensorI2C(const uint8_t address) : addr(address) {}

  void init_tw(const uint32_t baud) {}

  bool writeRegister(const uint8_t reg, const uint8_t data) { return true; }

  bool readRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {
    return true;
  }

  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    if (!readRegisters(reg, 1, &value)) return 0;
    return value;
  }

  const uint8_t addr;
};
