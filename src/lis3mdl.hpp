/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "sensor.hpp"
#include <string.h> // memcpy

namespace LIS3MDL {

constexpr uint8_t REG_WHO_AM_I   = 0x0F;
constexpr uint8_t REG_CTRL_REG1  = 0x20;
constexpr uint8_t REG_CTRL_REG2  = 0x21;
constexpr uint8_t REG_CTRL_REG3  = 0x22;
constexpr uint8_t REG_CTRL_REG4  = 0x23;
constexpr uint8_t REG_CTRL_REG5  = 0x24;
constexpr uint8_t REG_STATUS     = 0x27;
constexpr uint8_t REG_OUT_X_L    = 0x28;
constexpr uint8_t REG_OUT_X_H    = 0x29;
constexpr uint8_t REG_OUT_Y_L    = 0x2A;
constexpr uint8_t REG_OUT_Y_H    = 0x2B;
constexpr uint8_t REG_OUT_Z_L    = 0x2C;
constexpr uint8_t REG_OUT_Z_H    = 0x2D;
constexpr uint8_t REG_TEMP_OUT_L = 0x2E;
constexpr uint8_t REG_TEMP_OUT_H = 0x2F;
constexpr uint8_t REG_INT_CFG    = 0x30;
constexpr uint8_t REG_INT_SRC    = 0x31;
constexpr uint8_t REG_INT_THS_L  = 0x32;
constexpr uint8_t REG_INT_THS_H  = 0x33;
constexpr uint8_t WHO_AM_I       = 0x3D;

constexpr uint8_t ADDR_PRIM = 0x1C;
constexpr uint8_t ADDR_ALT  = 0x1E;


enum Bdu : uint8_t { BDU_OFF = 0x00, BDU_ON = 0x01 };

enum OpMode : uint8_t {
  OP_MODE_CONTINUOUS = 0b00, // OpMode 155 - 1000 Hz
  OP_MODE_SINGLE     = 0b01, // ODR 0.625 - 80 Hz ... won't use this
  OP_MODE_POWERDOWN  = 0b11
};

enum Range : uint8_t {
  RANGE_4GS  = 0b00,
  RANGE_8GS  = 0b01,
  RANGE_12GS = 0b10,
  RANGE_16GS = 0b11
};

enum Odr : uint8_t {
  ODR_155HZ  = 0b0001,
  ODR_300HZ  = 0b0011,
  ODR_560HZ  = 0b0101,
  ODR_1000HZ = 0b0111
};

using mag_t = gcisensors::vecf_t;

struct mag_raw_t {
  int16_t x, y, z; // micro Tesla (uT)
  bool ok;       // error?
};

/*
default:
-
*/
class gciLIS3MDL : public SensorI2C {
public:
  gciLIS3MDL(TwoWire *i2c, const uint8_t addr=ADDR_PRIM)
      : SensorI2C(i2c, addr) {}

  bool init(const Range range=RANGE_4GS,const Odr odr=ODR_155HZ) {
    uint8_t who_am_i;
    // check WHOAMI
    if (!readRegisters(REG_WHO_AM_I, sizeof(who_am_i), &who_am_i)) return false;

    if (who_am_i != WHO_AM_I) return false;
    if (!setRange(range)) return false;
    if (!setDataRate(odr)) return false;

    // Enable block update (MSB/LSB not updated until previous value read)
    // if (!setBdu(true)) return false;

    return true;
  }

  bool reboot() { return writeBits(REG_CTRL_REG1, 0x01, 1, 3); } // reboot memory content
  bool reset() { return writeBits(REG_CTRL_REG1, 0x01, 1, 2); }  // reset to default
  bool shutdown() { return writeBits(REG_CTRL_REG3, OP_MODE_POWERDOWN, 2, 0); }

  void set_cal(float cal[12]) { memcpy(sm, cal, 12*sizeof(float)); }

  const mag_raw_t read_raw() {
    mag_raw_t ret{0};
    ret.ok = false;

    if (!ready()) return ret;

    if (!readRegisters(REG_OUT_X_L, 6, buff.b)) {
      return ret;
    }

    ret.x = buff.s[0]; // counts
    ret.y = buff.s[1];
    ret.z = buff.s[2];

    // static uint8_t buffer[6];
    // if (!readRegisters(REG_OUT_X_L, 6, buffer)) {
    //   return ret;
    // }

    // ret.x = (buffer[1] << 8) | buffer[0];
    // ret.y = (buffer[3] << 8) | buffer[2];
    // ret.z = (buffer[5] << 8) | buffer[4];

    ret.ok = true;

    return ret;
  }

  const mag_t read() {
    const mag_raw_t raw = read_raw();
    mag_t ret{0};
    ret.ok = false;
    if (raw.ok == false) return ret;

    ret.x = raw.x * scale; // uT
    ret.y = raw.y * scale;
    ret.z = raw.z * scale;
    ret.ok = true;

    return ret;
  }

  const mag_t read_cal() {
    const mag_t m = read();
    mag_t ret{0};
    ret.ok = false;
    if (m.ok == false) return ret;

    ret.x  = sm[0] * m.x + sm[1] * m.y + sm[2] * m.z + sm[3];
    ret.y  = sm[4] * m.x + sm[5] * m.y + sm[6] * m.z + sm[7];
    ret.z  = sm[8] * m.x + sm[9] * m.y + sm[10] * m.z + sm[11];
    ret.ok = true;

    return ret;
  }

  bool ready() {
    constexpr uint8_t ZYXDA = BITS::b3;

    uint8_t val = readRegister(REG_STATUS);
    return val & ZYXDA;
  }

protected:
  enum PerfMode : uint8_t {
    PERF_MODE_LOW_POWER    = 0b00,
    PERF_MODE_MEDIUM_POWER = 0b01,
    PERF_MODE_HIGH         = 0b10,
    PERF_MODE_ULTRA_HIGH   = 0b11
  };

  float sm[12]{
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  }; // scale and bias

  bool setDataRate(const Odr odr) { // coupled with perfMode
    bool ok = true;

    switch (odr) {
    case ODR_155HZ:
      ok = setPerformanceMode(PERF_MODE_ULTRA_HIGH);
      break;
    case ODR_300HZ:
      ok = setPerformanceMode(PERF_MODE_HIGH);
      break;
    case ODR_560HZ:
      ok = setPerformanceMode(PERF_MODE_MEDIUM_POWER);
      break;
    case ODR_1000HZ:
      ok = setPerformanceMode(PERF_MODE_LOW_POWER);
      break;
    default:
      return false;
    }

    if (!ok) return false;

    if (!writeBits(REG_CTRL_REG1, odr, 4, 1)) return false;

    return true;
  }

  bool setRange(const Range range) {
    if (!writeBits(REG_CTRL_REG2, range, 2, 5)) {
      return false;
    }

    // 1 g = 0.0001 T = 0.1 mT = 100 uT = 100,000 nT
    // m (1E3) * (1E-4) => (1E-1) = 0.1
    // u (1E6) * (1E-4) => (1E2) = 100 <- this is the 100 below
    // pg8, table 3
    switch (range) {
    case RANGE_4GS:
      scale = 1.0f / 6842.0f * 100.0f;
      break;
    case RANGE_8GS:
      scale = 1.0f / 3421.0f * 100.0f;
      break;
    case RANGE_12GS:
      scale = 1.0f / 2281.0f * 100.0f;
      break;
    case RANGE_16GS:
      scale = 1.0f / 1711.0f * 100.0f;
      break;
    }
    return true;
  }

  bool setPerformanceMode(const PerfMode perf_mode) {
    if (!writeBits(REG_CTRL_REG1, 1, 1, 1)) return false; // fast_odr - new
    if (!writeBits(REG_CTRL_REG1, perf_mode, 2, 5)) return false; // x y axes
    if (!writeBits(REG_CTRL_REG4, perf_mode, 2, 2)) return false; // z axis
    if (!writeBits(REG_CTRL_REG3, OP_MODE_CONTINUOUS, 2, 0)) return false;

    return true;
  }

  bool enableTemp() { return writeBits(REG_CTRL_REG1, 0x01, 1, 7); } // why? not dependant on temp changes

  // block mode - enabled: don't update until MSB/LSB read
  // previously
  bool setBdu(bool en) { return writeBits(REG_CTRL_REG5, (en) ? BDU_ON : BDU_OFF, 1, 6); }
  /*
  Given some data, this will:
  1. read the register to get all the bits
  2. mask out the we don't want to change to protect them
  3. only change the correct bits
  4. write the final value back to the register

  reg - the register we want to change
  data - data that goes into register
  bits - how many bits for mask
  shift - how much to shift data by
  */
  bool
  writeBits(const uint8_t reg, const uint8_t data, const uint8_t bits,
            const uint8_t shift) {
    uint8_t val;
    if (!readRegisters(reg, 1, &val)) {
      return false;
    }
    uint8_t mask = (1 << (bits)) - 1;
    uint8_t d    = data & mask;
    mask <<= shift;
    val &= ~mask;
    val |= d << shift;
    return writeRegister(reg, val);
  }

  union {
    int16_t s[3]; // signed short
    uint8_t b[6]; // bytes
  } buff;
  float scale;
};


} // namespace LIS3MDL
