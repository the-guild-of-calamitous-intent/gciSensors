
#pragma once

#include <string.h> // memcpy
// #include "lis3mdl.hpp"

// using namespace LIS3MDL;

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

enum Bdu : uint8_t { BDU_OFF = 0x00, BDU_ON = 0x01 };

enum OpMode : uint8_t {
  OP_MODE_CONTINUOUS = 0b00, // OpMode 155 - 1000 Hz
  OP_MODE_SINGLE     = 0b01, // ODR 0.625 - 80 Hz ... won't use this
  OP_MODE_POWERDOWN  = 0b11
};

bool gciLIS3MDL::init(const Range range, const Odr odr) {
  uint8_t who_am_i;
  // check WHOAMI
  if (!readRegisters(REG_WHO_AM_I, sizeof(who_am_i), &who_am_i)) return false;

  if (who_am_i != WHO_AM_I) return false;

  // set range to +/-4GS
  if (!setRange(range)) return false;

  // set ODR to 155 Hz
  if (!setDataRate(odr)) return false;

  // Enable block update (MSB/LSB not updated until previous value read)
  // if (!setBdu(true)) return false;

  // mag_t ret = read();
  // while (!ret.ok)
  //   ret = read();

  return true;
}

// void gciLIS3MDL::set_cal(float cal[6]) {
//   mm[0] = cal[0];
//   mm[1] = cal[1];
//   mm[2] = cal[2];

//   mbias[0] = cal[3];
//   mbias[1] = cal[4];
//   mbias[2] = cal[5];
// }

void gciLIS3MDL::set_cal(float cal[12]) {
  memcpy(sm, cal, 12*sizeof(float));
}

bool gciLIS3MDL::setDataRate(const Odr odr) {

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

bool gciLIS3MDL::reboot() { return writeBits(REG_CTRL_REG1, 0x01, 1, 3); }

bool gciLIS3MDL::reset() { return writeBits(REG_CTRL_REG1, 0x01, 1, 2); }

bool gciLIS3MDL::setRange(const Range range) {
  if (!writeBits(REG_CTRL_REG2, range, 2, 5)) {
    return false;
  }

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
    scale = 1.0 / 1711.0f * 100.0f;
    break;
  }
  return true;
}

mag_raw_t gciLIS3MDL::read_raw() {
  mag_raw_t ret{0};

  if (!readRegisters(REG_OUT_X_L, 6, buff.b)) {
    ret.ok = false;
    return ret;
  }

  ret.x = buff.s[0];
  ret.y = buff.s[1];
  ret.z = buff.s[2];

  // ret.x = x; // uT
  // ret.y = y;
  // ret.z = z;

  ret.ok = true;

  return ret;
}

mag_t gciLIS3MDL::read() {
  const mag_raw_t raw = read_raw();
  mag_t ret{0};
  ret.ok = false;
  if (raw.ok == false) return ret;

  // if (!readRegisters(REG_OUT_X_L, 6, buff.b)) {
  //   ret.ok = false;
  //   return ret;
  // }

  // float x = buff.s[0] * scale;
  // float y = buff.s[1] * scale;
  // float z = buff.s[2] * scale;

  ret.x = raw.x * scale;
  ret.y = raw.y * scale;
  ret.z = raw.z * scale;

  // ret.x = x; // uT
  // ret.y = y;
  // ret.z = z;

  ret.ok = true;

  return ret;
}

mag_t gciLIS3MDL::read_cal() {
  const mag_t m = read();
  mag_t ret{0};
  ret.ok = false;
  if (m.ok == false) return ret;

  // ret.x = mm[0] * ret.x - mbias[0]; // uT
  // ret.y = mm[1] * ret.y - mbias[1];
  // ret.z = mm[2] * ret.z - mbias[2];

  ret.x  = sm[0] * m.x + sm[1] * m.y + sm[2] * m.z + sm[3];
  ret.y  = sm[4] * m.x + sm[5] * m.y + sm[6] * m.z + sm[7];
  ret.z  = sm[8] * m.x + sm[9] * m.y + sm[10] * m.z + sm[11];
  ret.ok = true;

  return ret;
}

bool gciLIS3MDL::ready() {
  constexpr uint8_t ZYXDA = BITS::b3;

  uint8_t val = readRegister(REG_STATUS);
  return val & ZYXDA;
}

bool gciLIS3MDL::setBdu(bool en) {
  return writeBits(REG_CTRL_REG5, (en) ? BDU_ON : BDU_OFF, 1, 6);
}

bool gciLIS3MDL::shutdown() {
  return writeBits(REG_CTRL_REG3, OP_MODE_POWERDOWN, 2, 0);
}

bool gciLIS3MDL::enableTemp() { return writeBits(REG_CTRL_REG1, 0x01, 1, 7); }

bool gciLIS3MDL::setPerformanceMode(const PerfMode perf_mode) {
  if (!writeBits(REG_CTRL_REG1, perf_mode, 2, 5)) return false; // x y axes
  if (!writeBits(REG_CTRL_REG4, perf_mode, 2, 2)) return false; // z axis
  if (!writeBits(REG_CTRL_REG3, OP_MODE_CONTINUOUS, 2, 0)) return false;

  return true;
}

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
bool gciLIS3MDL::writeBits(const uint8_t reg, const uint8_t data,
                           const uint8_t bits, const uint8_t shift) {
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
