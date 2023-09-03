
#pragma once

#include <string.h> // memcpy
// #include "lsm6dsox.hpp"
#include "units.hpp"

constexpr uint8_t REG_INT1_CTRL = 0x0D;
constexpr uint8_t REG_INT2_CTRL = 0x0E;
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL     = 0x10; // Accel settings
constexpr uint8_t CTRL2_G      = 0x11; // Gyro settings hz and dps
constexpr uint8_t CTRL3_C      = 0x12; // interrupt stuff
constexpr uint8_t CTRL6_C      = 0x15; // Accel perf mode and Gyro LPF
constexpr uint8_t CTRL7_G      = 0x16; // Gyro filtering
constexpr uint8_t CTRL8_XL     = 0x17; // Accel filtering
constexpr uint8_t CTRL9_XL     = 0x18; // Accel filtering
constexpr uint8_t CTRL10_C     = 0x19; // tiimestamp

constexpr uint8_t REG_STATUS   = 0x1E;

constexpr uint8_t REG_OUT_TEMP_L   = 0x20;
// constexpr uint8_t OUT_TEMP_H   = 0x21;

constexpr uint8_t REG_OUTX_L_G     = 0x22; // gyro
// constexpr uint8_t OUTX_H_G     = 0x23;
// constexpr uint8_t OUTY_L_G     = 0x24;
// constexpr uint8_t OUTY_H_G     = 0x25;
// constexpr uint8_t OUTZ_L_G     = 0x26;
// constexpr uint8_t OUTZ_H_G     = 0x27;

constexpr uint8_t REG_OUTX_L_A    = 0x28; // accel
// constexpr uint8_t OUTX_H_XL    = 0x29;
// constexpr uint8_t OUTY_L_XL    = 0x2A;
// constexpr uint8_t OUTY_H_XL    = 0x2B;
// constexpr uint8_t OUTZ_L_XL    = 0x2C;
// constexpr uint8_t OUTZ_H_XL    = 0x2D;

constexpr uint8_t REG_TIMESTAMP0 = 0x40; // 4B timestamp

constexpr uint8_t WHO_AM_I     = 0x6C;

constexpr float TEMP_SCALE     = 1.0f / 256.0f;

using namespace LSM6DSOX;

// bool gciLSM6DSOX::init(uint8_t accel_range, uint8_t accel_odr,
//     uint8_t gyro_range, uint8_t gyro_odr) {

bool gciLSM6DSOX::init(uint8_t accel_range, uint8_t gyro_range, uint8_t odr) {

  if (!(readRegister(REG_WHO_AM_I) == WHO_AM_I)) return false;

  // set the gyroscope control register to work at 104 Hz, 2000 dps and in
  // bypass mode
  //   writeRegister(CTRL2_G, 0x4C);
  if (!setGyro(odr, gyro_range)) return false;

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass
  // mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
  // datasheet)
  if (!setAccel(odr, accel_range)) return false;

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  //   writeRegister(CTRL7_G, 0x00);

  // Set LPF and HPF config register
  if (!writeRegister(CTRL8_XL, 0x00)) return false; // LPF ODR/2, disable HPF

  if (!writeRegister(CTRL10_C, BITS::b5)) return false; // enable timestamp

  // Serial.println("pre-iinterrupt");

  // if (!set_interrupts(true)) return false; // set interrupts

  // Serial.println(">> init done ...");

  return true;
}

void gciLSM6DSOX::set_acal(float cal[12]) {
  memcpy(sm, cal, 12*sizeof(float));
}

void gciLSM6DSOX::set_gcal(float cal[12]) {
  gbias[0] = cal[0];
  gbias[1] = cal[1];
  gbias[2] = cal[2];
}

/*
scale factor in rad/s
*/
bool gciLSM6DSOX::setGyro(uint8_t odr, uint8_t dps) {
  if (dps == GYRO_RANGE_125_DPS) g_scale = 125.0f / 32768.0f * Units::deg2rad;
  else if (dps == GYRO_RANGE_250_DPS)
    g_scale = 250.0f / 32768.0f * Units::deg2rad;
  else if (dps == GYRO_RANGE_500_DPS)
    g_scale = 500.0f / 32768.0f * Units::deg2rad;
  else if (dps == GYRO_RANGE_1000_DPS)
    g_scale = 1000.0f / 32768.0f * Units::deg2rad;
  else if (dps == GYRO_RANGE_2000_DPS)
    g_scale = 2000.0f / 32768.0f * Units::deg2rad;

  uint8_t val = (odr << 4) + (dps);
  return writeRegister(CTRL2_G, val);
}

/*
scale factor in g's
*/
bool gciLSM6DSOX::setAccel(uint8_t odr, uint8_t range) {

  if (range == ACCEL_RANGE_2_G) a_scale = 2.0f / 32768.0f;
  else if (range == ACCEL_RANGE_4_G) a_scale = 4.0f / 32768.0f;
  else if (range == ACCEL_RANGE_8_G) a_scale = 8.0f / 32768.0f;
  else if (range == ACCEL_RANGE_16_G) a_scale = 16.0f / 32768.0f;

  constexpr uint8_t LPF2_XL_EN = 0; // kill LFP2

  uint8_t val                  = (odr << 4) + (range << 2) + (LPF2_XL_EN << 1);
  return writeRegister(CTRL1_XL, val);
}

bool gciLSM6DSOX::ready() {
  uint8_t val = readRegister(REG_STATUS);
  return (val & BITS::b0) && (val & BITS::b1); // bits 0/1 are press/temp ready
}

sox_raw_t gciLSM6DSOX::read_raw() {
  sox_raw_t ret{0};
  ret.ok = false;

  if (!readRegisters(REG_OUTX_L_A, sizeof(data.b), data.b)) {
    return ret;
  }

  ret.ax = data.s[0];
  ret.ay = data.s[1];
  ret.az = data.s[2];

  if (!readRegisters(REG_OUTX_L_G, sizeof(data.b), data.b)) {
    return ret;
  }

  ret.gx = data.s[0];
  ret.gy = data.s[1];
  ret.gz = data.s[2];

  if (readRegisters(REG_OUT_TEMP_L, 2, data.b) != 1) {
    return ret;
  }
  ret.temp = data.s[0];

  if (!readRegisters(REG_TIMESTAMP0, 4, data.b)) {
    return ret;
  }

  ret.ts = data.l; // 25 usec per count
  // ret.ts = data.l - timestamp;
  // timestamp = data.l; // 25 usec per count

  ret.ok   = true;
  return ret;
}

/*
accel - g's
gyro = rad/s
temp - C
*/
sox_t gciLSM6DSOX::read() {
  const sox_raw_t raw = read_raw();
  sox_t ret{0};
  ret.ok = false;
  if (raw.ok == false) return ret;

  // if (!readRegisters(REG_OUTX_L_A, sizeof(data.b), data.b)) {
  //   ret.ok = false;
  //   return ret;
  // }

  ret.ax = raw.ax * a_scale;
  ret.ay = raw.ay * a_scale;
  ret.az = raw.az * a_scale;

  // if (!readRegisters(REG_OUTX_L_G, sizeof(data.b), data.b)) {
  //   ret.ok = false;
  //   return ret;
  // }

  ret.gx = raw.gx * g_scale;
  ret.gy = raw.gy * g_scale;
  ret.gz = raw.gz * g_scale;

  // if (readRegisters(REG_OUT_TEMP_L, 2, data.b) != 1) {
  //   ret.ok = false;
  //   return ret;
  // }
  ret.temp = raw.temp * TEMP_SCALE + 25.0f;

  // if (!readRegisters(REG_TIMESTAMP0, 4, data.b)) {
  //   ret.ok = false;
  //   return ret;
  // }

  ret.ts = raw.ts; // 25 usec per count
  // ret.ts = data.l - timestamp;
  // timestamp = data.l; // 25 usec per count

  ret.ok = true;
  return ret;
}

sox_t gciLSM6DSOX::read_cal() {
  const sox_t m = read();
  sox_t ret{0};
  ret.ok = false;
  if (m.ok == false) return ret;

  // const float x = ret.ax;
  // const float y = ret.ay;
  // const float z = ret.az;

  // ret.ax  = sm[0][0] * m.ax + sm[0][1] * m.ay + sm[0][2] * m.az + sm[0][3];
  // ret.ay  = sm[1][0] * m.ax + sm[1][1] * m.ay + sm[1][2] * m.az + sm[1][3];
  // ret.az  = sm[2][0] * m.ax + sm[2][1] * m.ay + sm[2][2] * m.az + sm[2][3];

  ret.ax  = sm[0] * m.ax + sm[1] * m.ay + sm[2] * m.az + sm[3];
  ret.ay  = sm[4] * m.ax + sm[5] * m.ay + sm[6] * m.az + sm[7];
  ret.az  = sm[8] * m.ax + sm[9] * m.ay + sm[10] * m.az + sm[11];

  // x       = ret.gx;
  // y       = ret.gy;
  // z       = ret.gz;

  ret.gx  = (m.gx - gbias[0]);
  ret.gy  = (m.gy - gbias[1]);
  ret.gz  = (m.gz - gbias[2]);

  ret.ok = true;

  return ret;
}

/*
pg 54
*/
bool gciLSM6DSOX::set_interrupts(bool val) {
  uint8_t INT1_DRDY_XL = BITS::b0;
  uint8_t INT2_DRDY_G = BITS::b1;
  uint8_t DEN_DRDY_flag = 0; //BITS::b7; // need this too? not sure what it is
  // uint8_t LIR = BITS::b0;
  uint8_t IF_INC = BITS::b2; // default enabled
  uint8_t H_LACTIVE = BITS::b5; // 0-high, 1-low
  // uint8_t PP_OD = BITS::b4; // open-drain
  bool ret = true;
  // uint8_t TAP_CFG0 = 0x56;
  ret &= writeRegister(REG_INT1_CTRL, INT1_DRDY_XL | DEN_DRDY_flag); // accel
  // ret &= writeRegister(TAP_CFG0, LIR); // latching
  ret &= writeRegister(CTRL3_C, H_LACTIVE | IF_INC); // INT high
  ret &= writeRegister(REG_INT2_CTRL, INT2_DRDY_G); // gyro

  return ret;
}
