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
constexpr uint8_t WHO_AM_I       = 0x3D;

constexpr uint8_t ADDR_PRIM = 0x1C;
constexpr uint8_t ADDR_ALT  = 0x1E;


enum Range : uint8_t {
  RANGE_4GAUSS  = 0x00, // default
  RANGE_8GAUSS  = 0x20,
  RANGE_12GAUSS = 0x40,
  RANGE_16GAUSS = 0x60
};

// enum Odr : uint8_t {
//   ODR_155HZ,
//   ODR_300HZ,
//   ODR_560HZ,
//   ODR_1000HZ
// };

// using mag_t = gci::sensors::vecf_t;
// using mag_raw_t = gci::sensors::veci_t;

struct mag_t {
  float x,y,z;
  float temperature;
  bool ok;

  inline
  const float magnitude() const {
    return sqrt(x * x + y * y + z * z);
  }

  bool normalize() {
    float n = 1.0 / magnitude();
    if (std::isinf(n)) return false;

    x *= n;
    y *= n;
    z *= n;

    return true;
  }
};

struct mag_raw_t {
  int16_t x,y,z;
  int16_t temperature;
  bool ok;
};

#define LIS3MDL_TEMP_EN       0x80  // chip default off
#define LIS3MDL_FAST_ODR_EN   0x02
#define LIS3MDL_BDU_EN        0x40  // chip default off

#define LIS3MDL_LP   0x00
#define LIS3MDL_MP   0x01
#define LIS3MDL_HIP  0x02
#define LIS3MDL_UHP  0x03


enum Odr : uint8_t {
  ODR_155HZ = LIS3MDL_UHP,
  ODR_300HZ = LIS3MDL_HIP,
  ODR_560HZ = LIS3MDL_MP,
  ODR_1000HZ = LIS3MDL_LP
};

/*
This outputs a normalized magnetic field.

default:
-
*/
class gciLIS3MDL : public SensorI2C {
public:
  gciLIS3MDL(TwoWire *i2c, const uint8_t addr=ADDR_PRIM)
      : SensorI2C(i2c, addr) {}

  uint8_t init(const Range range=RANGE_4GAUSS, const Odr odr=ODR_155HZ) {
    uint8_t who = readRegister(REG_WHO_AM_I);
    if (who != WHO_AM_I) return who;

    uint8_t reg1 = LIS3MDL_FAST_ODR_EN | LIS3MDL_TEMP_EN | (odr << 5);
    uint8_t reg4 = (odr << 2);
    // Serial.println(reg1);

    if (!writeRegister(REG_CTRL_REG1, reg1)) return 1;
    if (!writeRegister(REG_CTRL_REG2, range)) return 2;
    if (!writeRegister(REG_CTRL_REG3, 0x00)) return 3; // continuous sampling
    if (!writeRegister(REG_CTRL_REG4, reg4)) return 4; // enable z-axis
    if (!writeRegister(REG_CTRL_REG5, 0x00)) return 5; // continuous sampling / no fast read

    return 0;
  }

  // bool reboot() { return writeBits(REG_CTRL_REG1, 0x01, 1, 3); } // reboot memory content
  // bool reset() { return writeBits(REG_CTRL_REG1, 0x01, 1, 2); }  // reset to default
  bool reboot() {
    if(!writeRegister(REG_CTRL_REG3, 0x03)) return false;
    delay(100);
    return writeRegister(REG_CTRL_REG3, 0x00);
  }

  void set_cal(float cal[12]) { memcpy(sm, cal, 12*sizeof(float)); }

  const mag_raw_t read_raw() {
    mag_raw_t ret{0};
    ret.ok = false;

    if (!ready()) return ret;

    if (!readRegisters(REG_OUT_X_L, 6, buff.b)) return ret;

    ret.x = buff.s[0]; // counts
    ret.y = buff.s[1];
    ret.z = buff.s[2];
    ret.temperature = buff.s[3];
    ret.ok = true;

    return ret;
  }

  const mag_t read() {
    mag_t ret{0};
    ret.ok = false;
    const mag_raw_t raw = read_raw();
    if (raw.ok == false) return ret;

    // ret.x = raw.x * scale; // uT
    // ret.y = raw.y * scale;
    // ret.z = raw.z * scale;
    ret.x = static_cast<float>(raw.x); // gauss
    ret.y = static_cast<float>(raw.y);
    ret.z = static_cast<float>(raw.z);
    ret.temperature = static_cast<float>(raw.temperature) / 8.0f; // pg 9, Table 4

    if(!ret.normalize()) return ret; // normalized
    ret.ok = true;

    return ret;
  }

  const mag_t read_cal() {
    const mag_t m = read();
    if (m.ok == false) return m;

    mag_t ret{0};
    ret.x  = sm[0] * m.x + sm[1] * m.y + sm[2] * m.z - sm[3];
    ret.y  = sm[4] * m.x + sm[5] * m.y + sm[6] * m.z - sm[7];
    ret.z  = sm[8] * m.x + sm[9] * m.y + sm[10] * m.z - sm[11];
    ret.ok = true;

    return ret;
  }

  bool ready() {
    constexpr uint8_t ZYXDA = 0x08; // 0b00001000; //BITS::b3;

    uint8_t val = readRegister(REG_STATUS_REG);
    return val & ZYXDA;
  }

protected:
  float sm[12]{
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  }; // scale and bias

  union {
    int16_t s[4]; // signed short
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

    // switch (odr) {
    // case ODR_155HZ:
    //   reg1 |= (LIS3MDL_UHP << 5);
    //   reg4 = (LIS3MDL_UHP << 2);
    //   break;
    // case ODR_300HZ:
    //   reg1 |= (LIS3MDL_HIP << 5);
    //   reg4 = (LIS3MDL_HIP << 2);
    //   break;
    // case ODR_560HZ:
    //   reg1 |= (LIS3MDL_MP << 5);
    //   reg4 = (LIS3MDL_MP << 2);
    //   break;
    // case ODR_1000HZ:
    //   reg1 |= (LIS3MDL_LP << 5);
    //   reg4 = (LIS3MDL_LP << 2);
    //   break;
    // default:
    //   return 10;
    // }
