/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "../sensor.hpp"

namespace LIS3MDL {

constexpr uint8_t ADDR_PRIM = 0x1C;
constexpr uint8_t ADDR_ALT = 0x1E;

enum Range : uint8_t {
  RANGE_4GS = 0b00,
  RANGE_8GS = 0b01,
  RANGE_12GS = 0b10,
  RANGE_16GS = 0b11
};

enum Odr : uint8_t {
  ODR_155HZ = 0b0001,
  ODR_300HZ = 0b0011,
  ODR_560HZ = 0b0101,
  ODR_1000HZ = 0b0111
};

struct mag_t {
  float x, y, z; // micro Tesla (uT)
  bool ok;       // error?
};

/*
default:
-
*/
class gciLIS3MDL : public SensorI2C {
public:
  gciLIS3MDL(TwoWire *i2c, const uint8_t addr = ADDR_PRIM)
      : SensorI2C(i2c, addr) {}

  bool init();
  bool reboot(); // reboot memory content
  bool reset();  // reset to default
  bool shutdown();
  mag_t read();

  bool setDataRate(const Odr data_rate); // coupled with perfMode
  bool setRange(const Range range);

protected:
  enum PerfMode : uint8_t {
    PERF_MODE_LOW_POWER = 0b00,
    PERF_MODE_MEDIUM_POWER = 0b01,
    PERF_MODE_HIGH = 0b10,
    PERF_MODE_ULTRA_HIGH = 0b11
  };

  bool setPerformanceMode(const PerfMode perf_mode);
  bool setBdu(bool en); // block mode - enabled: don't update until MSB/LSB read
                        // previously
  bool enableTemp();    // why? not dependant on temp changes

  union {
    int16_t s;    // signed short
    uint8_t b[2]; // bytes
  } buff;
  float scale;
};

} // namespace LIS3MDL
