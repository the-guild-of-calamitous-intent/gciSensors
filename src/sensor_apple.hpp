/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h> // int types

// #if defined(ARDUINO)
//   #include <Arduino.h>
//   #include <Wire.h>
// #else
#include <Wire.hpp>
// #endif

// inline uint32_t to_24b(uint8_t *b) {
//   return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
// }
// inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
//   return ((uint16_t)msb << 8) | (uint16_t)lsb;
// }

// namespace BITS {
// constexpr uint8_t b0 = 1;
// constexpr uint8_t b1 = 2;
// constexpr uint8_t b2 = 4;
// constexpr uint8_t b3 = 8;
// constexpr uint8_t b4 = 16;
// constexpr uint8_t b5 = 32;
// constexpr uint8_t b6 = 64;
// constexpr uint8_t b7 = 128;
// } // namespace BITS

class SensorI2C {
public:
  SensorI2C(const uint8_t address) : addr(address) {}

  void init_tw(const uint32_t baud) {}

  // //[ LINUX
  // ]////////////////////////////////////////////////////////////////////////////////
  // #if defined(__linux__)

  //   bool writeRegister(const uint8_t reg, const uint8_t data) {
  //     i2c->set(addr);
  //     return i2c->write(reg, data);
  //   }

  //   bool readRegisters(const uint8_t reg, const uint8_t count, uint8_t *const
  //   data) {
  //     i2c->set(addr);
  //     return i2c->read(reg, count, data);
  //   }

  // //[ APPLE
  // ]///////////////////////////////////////////////////////////////////////////////
  // #elif defined(__APPLE__)

  bool writeRegister(const uint8_t reg, const uint8_t data) { return true; }

  bool readRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {
    return true;
  }

  // //[ Arduino
  // ]///////////////////////////////////////////////////////////////////////////////
  // #elif defined(ARDUINO)

  //   bool writeRegister(const uint8_t reg, const uint8_t data) {
  //     uint8_t ret_val;
  //     i2c->beginTransmission(addr);
  //     i2c->write(reg);
  //     i2c->write(data);
  //     i2c->endTransmission();

  //     delay(10);
  //     readRegisters(reg, 1, &ret_val);
  //     if (data == ret_val) return true;

  //     // println("data write failed verification: " + String(int(data)) +
  //     //         " != " + String(int(ret_val)));

  //     return false;
  //   }

  //   bool readRegisters(const uint8_t reg, const uint8_t count, uint8_t *const
  //   data) {
  //     i2c->beginTransmission(addr);
  //     i2c->write(reg);
  //     i2c->endTransmission(false);

  //     // delay(500);
  //     // delay(2);

  //     uint8_t bytes_rx = i2c->requestFrom(addr, count);
  //     if (bytes_rx == count) {
  //       for (uint8_t i = 0; i < count; i++) {
  //         data[i] = i2c->read();
  //       }
  //       return true;
  //     }

  //     // println("ReadRegisters::bad read: " + String(int(bytes_rx)) +
  //     //         " expected: " + String(int(count)));

  //     return false;
  //   }

  // #endif

  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    if (!readRegisters(reg, 1, &value)) return 0;
    return value;
  }

  const uint8_t addr;

protected:
  TwoWire *i2c;
};
