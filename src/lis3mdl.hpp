/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once


#include "sensor.hpp"
#include <string.h> // memcpy
// #include <stdio.h>

namespace LIS3MDL {

constexpr uint8_t REG_WHO_AM_I   = 0x0F;
constexpr uint8_t REG_CTRL_REG1  = 0x20;
constexpr uint8_t REG_CTRL_REG2  = 0x21;
constexpr uint8_t REG_CTRL_REG3  = 0x22;
constexpr uint8_t REG_CTRL_REG4  = 0x23;
constexpr uint8_t REG_CTRL_REG5  = 0x24;
constexpr uint8_t REG_STATUS_REG = 0x27;
constexpr uint8_t REG_OUT_X_L    = 0x28;
// constexpr uint8_t REG_OUT_X_H    = 0x29;
// constexpr uint8_t REG_OUT_Y_L    = 0x2A;
// constexpr uint8_t REG_OUT_Y_H    = 0x2B;
// constexpr uint8_t REG_OUT_Z_L    = 0x2C;
// constexpr uint8_t REG_OUT_Z_H    = 0x2D;
// constexpr uint8_t REG_TEMP_OUT_L = 0x2E;
// constexpr uint8_t REG_TEMP_OUT_H = 0x2F;
// constexpr uint8_t REG_INT_CFG    = 0x30;
// constexpr uint8_t REG_INT_SRC    = 0x31;
// constexpr uint8_t REG_INT_THS_L  = 0x32;
// constexpr uint8_t REG_INT_THS_H  = 0x33;

constexpr uint8_t WHO_AM_I            = 0x3D;
constexpr uint8_t STATUS_ZYXDA        = 0x08; // 0b00001000;
constexpr uint8_t LIS3MDL_TEMP_EN     = 0x80; // chip default off
constexpr uint8_t LIS3MDL_FAST_ODR_EN = 0x02;
constexpr uint8_t LIS3MDL_BDU_EN      = 0x40; // chip default off

constexpr uint8_t LIS3MDL_LP          = 0x00;
constexpr uint8_t LIS3MDL_MP          = 0x01;
constexpr uint8_t LIS3MDL_HIP         = 0x02;
constexpr uint8_t LIS3MDL_UHP         = 0x03;

constexpr uint8_t LIS3MDL_ADDR        = 0x1C;
constexpr uint8_t LIS3MDL_ADDR_ALT    = 0x1E;

enum Range : uint8_t {
  RANGE_4GAUSS  = 0x00, // default
  RANGE_8GAUSS  = 0x20,
  RANGE_12GAUSS = 0x40,
  RANGE_16GAUSS = 0x60
};

using lis3mdl_t = gci::sensors::vec_msg_t;
using lis3mdl_raw_t = gci::sensors::vec_msg_raw_t;

// struct lis3mdl_t {
//   float x, y, z;
//   // float temperature;
//   bool ok;

//   inline const float magnitude() const { return sqrt(x * x + y * y + z * z); }

//   bool normalize() {
//     float n = 1.0f / magnitude();
//     if (std::isinf(n)) return false;

//     x *= n;
//     y *= n;
//     z *= n;

//     return true;
//   }
// };

// struct lis3mdl_raw_t {
//   int16_t x, y, z;
//   // int16_t temperature;
//   bool ok;
// };

enum Odr : uint8_t {
  ODR_155HZ  = LIS3MDL_UHP, // 3
  ODR_300HZ  = LIS3MDL_HIP, // 2
  ODR_560HZ  = LIS3MDL_MP,  // 1
  ODR_1000HZ = LIS3MDL_LP   // 0
};

enum mdl_error : uint8_t {
  NO_ERROR,
  ERROR_WHOAMI,
  ERROR_REG1,
  ERROR_REG2,
  ERROR_REG3,
  ERROR_REG4,
  ERROR_REG5
};

/*
This outputs a normalized magnetic field.
*/
class gciLIS3MDL : public SensorI2C {
public:
  gciLIS3MDL(const uint32_t port, const uint8_t addr = LIS3MDL_ADDR)
      : SensorI2C(addr, port) {}

  uint8_t init(const Range range = RANGE_4GAUSS, const Odr odr = ODR_155HZ) {
    uint8_t id{0};
    readRegister(REG_WHO_AM_I, &id);
    if (id != WHO_AM_I) return ERROR_WHOAMI;

    uint8_t reg1 = LIS3MDL_FAST_ODR_EN | LIS3MDL_TEMP_EN | (odr << 5);
    uint8_t reg4 = (odr << 2);

    if (!writeRegister(REG_CTRL_REG1, reg1))
      return ERROR_REG1; // enable x/y-axis, temp
    if (!writeRegister(REG_CTRL_REG2, range)) return ERROR_REG2; // set range
    if (!writeRegister(REG_CTRL_REG3, 0x00))
      return ERROR_REG3; // continuous sampling
    if (!writeRegister(REG_CTRL_REG4, reg4)) return ERROR_REG4; // enable z-axis
    if (!writeRegister(REG_CTRL_REG5, 0x00))
      return ERROR_REG5; // continuous sampling / no fast read

    return NO_ERROR;
  }

  // bool reboot() { return writeBits(REG_CTRL_REG1, 0x01, 1, 3); } // reboot
  // memory content bool reset() { return writeBits(REG_CTRL_REG1, 0x01, 1, 2);
  // }  // reset to default

  bool reboot() {
    if (!writeRegister(REG_CTRL_REG3, 0x03)) return false;
    sleep_ms(100);
    return writeRegister(REG_CTRL_REG3, 0x00);
  }

  void set_cal(float cal[12]) { memcpy(sm, cal, 12 * sizeof(float)); }

  const lis3mdl_raw_t read_raw() {
    lis3mdl_raw_t ret;
    ret.ok = false;

    if (!ready()) return ret;

#define READ_MAG 6
#define READ_MAG_TEMP 8

    if (!readRegisters(REG_OUT_X_L, READ_MAG, buff.b)) return ret;
    ret.x = buff.s.x; // counts
    ret.y = buff.s.y;
    ret.z = buff.s.z;
    // ret.temperature = buff.s.temp;
    ret.ok = true;

    return ret;
  }

  const lis3mdl_t read() {
    lis3mdl_t ret;
    ret.ok                  = false;
    const lis3mdl_raw_t raw = read_raw();
    if (raw.ok == false) return ret;

    ret.x = static_cast<float>(raw.x); // gauss
    ret.y = static_cast<float>(raw.y);
    ret.z = static_cast<float>(raw.z);

    // BROKEN????
    // ((float_t)lsb / 8.0f) + (25.0f);
    // https://github.com/STMicroelectronics/lis3mdl-pid/blob/master/lis3mdl_reg.c#L113
    // ret.temperature = (float)(raw.temperature) / 8.0f + 25.0f;
    // static_cast<float>(raw.temperature) / 8.0f + 25.0f; // pg 9, Table 4
    // printf(">> temp raw: %u", uint(raw.temperature));

    // normalize mag readings
    if (!ret.normalize()) return ret;
    ret.ok = true;

    return ret;
  }

  const lis3mdl_t read_cal() {
    const lis3mdl_t m = read();
    if (m.ok == false) return m;

    lis3mdl_t ret;
    ret.x = sm[0] * m.x + sm[1] * m.y + sm[2] * m.z - sm[3];
    ret.y = sm[4] * m.x + sm[5] * m.y + sm[6] * m.z - sm[7];
    ret.z = sm[8] * m.x + sm[9] * m.y + sm[10] * m.z - sm[11];
    // ret.temperature = m.temperature;
    ret.ok = true;

    return ret;
  }

  inline bool ready() {
    // constexpr uint8_t ZYXDA = 0x08; // 0b00001000;

    // uint8_t val             = readRegister(REG_STATUS_REG);
    // return val & ZYXDA;
    uint8_t val{0};
    readRegister(REG_STATUS_REG, &val);
    return (val & STATUS_ZYXDA) > 0;
  }

protected:
  // scale and bias
  float sm[12]{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  union {
    struct {
      int16_t x, y, z, temp;
    } s;
    // int16_t s[4]; // signed short
    uint8_t b[8]; // bytes
  } buff;

  // float scale;
};

} // namespace LIS3MDL

// 1 g = 0.0001 T = 0.1 mT = 100 uT = 100,000 nT
// m (1E3) * (1E-4) => (1E-1) = 0.1
// u (1E6) * (1E-4) => (1E2) = 100 <- this is the 100 below
// pg8, table 3
// switch (range) {
// case RANGE_4GAUSS:
//   scale = 100.0f / 6842.0f;
//   break;
// case RANGE_8GAUSS:
//   scale = 100.0f / 3421.0f;
//   break;
// case RANGE_12GAUSS:
//   scale = 100.0f / 2281.0f;
//   break;
// case RANGE_16GAUSS:
//   scale = 100.0f / 1711.0f;
//   break;
// }