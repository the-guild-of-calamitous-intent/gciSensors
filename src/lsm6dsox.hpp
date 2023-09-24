/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "sensor.hpp"
#include <string.h> // memcpy

namespace LSM6DSOX {

constexpr int LSM6DSOX_ADDRESS = 0x6A;


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

// The accelerometer/gyroscope data rate
enum ODR : uint8_t {
  RATE_SHUTDOWN,
  RATE_12_5_HZ,
  RATE_26_HZ,
  RATE_52_HZ,
  RATE_104_HZ,
  RATE_208_HZ,
  RATE_416_HZ,
  RATE_833_HZ,
  RATE_1_66K_HZ,
  RATE_3_33K_HZ,
  RATE_6_66K_HZ,
};

// The accelerometer data range
enum accel_range : uint8_t {
  ACCEL_RANGE_2_G,
  ACCEL_RANGE_16_G,
  ACCEL_RANGE_4_G,
  ACCEL_RANGE_8_G
};

// The gyro data range
enum gyro_range : uint8_t {
  GYRO_RANGE_125_DPS  = 0b0010,
  GYRO_RANGE_250_DPS  = 0b0000,
  GYRO_RANGE_500_DPS  = 0b0100,
  GYRO_RANGE_1000_DPS = 0b1000,
  GYRO_RANGE_2000_DPS = 0b1100
};

/** The high pass filter bandwidth */
// enum hpf_range: uint8_t {
//   HPF_ODR_DIV_50 = 0,
//   HPF_ODR_DIV_100 = 1,
//   HPF_ODR_DIV_9 = 2,
//   HPF_ODR_DIV_400 = 3,
// };

constexpr float LSM6DSOX_TIMESTEP_RES = 25e-6;

struct sox_t {
  float ax, ay, az, gx, gy, gz, temp;
  uint32_t ts;
  bool ok;
};

struct sox_raw_t {
  int16_t ax, ay, az, gx, gy, gz, temp;
  uint32_t ts;
  bool ok;
};

struct lsm6_available_t {
  bool accel, gyro, temp; // sensor available?
};



/*
Can insert user defined accel bias offsets:
X_OFS_USR
Y_OFS_USR
Z_OFS_USR
*/
class gciLSM6DSOX : public SensorI2C {
public:
  gciLSM6DSOX(TwoWire *wire, uint8_t addr = LSM6DSOX_ADDRESS)
      : SensorI2C(wire, addr) {}
  ~gciLSM6DSOX() {}

  bool init(
        uint8_t accel_range=ACCEL_RANGE_4_G,
        uint8_t gyro_range=GYRO_RANGE_2000_DPS,
        uint8_t odr=RATE_104_HZ) {

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

  void set_acal(float cal[12]) { memcpy(acal, cal, 12*sizeof(float)); }
  void set_gcal(float cal[12]) { memcpy(gcal, cal, 12*sizeof(float)); }

  const sox_raw_t read_raw() { // accel - g's, gyro - dps, temp - C
    sox_raw_t ret{0};
    ret.ok = false;

    if (!ready()) return ret;

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

  sox_t read() { // accel - g's, gyro - dps, temp - C
    const sox_raw_t raw = read_raw();
    sox_t ret{0};
    ret.ok = false;
    if (raw.ok == false) return ret;

    ret.ax = raw.ax * a_scale;
    ret.ay = raw.ay * a_scale;
    ret.az = raw.az * a_scale;
    ret.gx = raw.gx * g_scale;
    ret.gy = raw.gy * g_scale;
    ret.gz = raw.gz * g_scale;
    ret.temp = raw.temp * TEMP_SCALE + 25.0f;
    ret.ts = raw.ts; // 25 usec per count
    ret.ok = true;

    return ret;
  }

  sox_t read_cal() { // accel - g's, gyro - dps, temp - C
    const sox_t m = read();
    sox_t ret{0};
    ret.ok = false;

    if (m.ok == false) return ret;

    ret.ax  = acal[0] * m.ax + acal[1] * m.ay + acal[2] * m.az + acal[3];
    ret.ay  = acal[4] * m.ax + acal[5] * m.ay + acal[6] * m.az + acal[7];
    ret.az  = acal[8] * m.ax + acal[9] * m.ay + acal[10] * m.az + acal[11];

    ret.gx  = acal[0] * m.gx + acal[1] * m.gy + acal[2] * m.gz + acal[3];
    ret.gy  = acal[4] * m.gx + acal[5] * m.gy + acal[6] * m.gz + acal[7];
    ret.gz  = acal[8] * m.gx + acal[9] * m.gy + acal[10] * m.gz + acal[11];

    ret.ok = true;

    return ret;
  }

  bool ready() {
    uint8_t val = readRegister(REG_STATUS);
    return (val & BITS::b1) && (val & BITS::b2); // bits 1/2 are gyro/accel ready, pg 69
    // return val && (BITS::b1 | BITS::b2); // not sure this is right
  }

  bool set_interrupts(bool val) {
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

private:
  bool setGyro(uint8_t odr, uint8_t dps) {
    if (dps == GYRO_RANGE_125_DPS)
      g_scale = 125.0f / 32768.0f;
    else if (dps == GYRO_RANGE_250_DPS)
      g_scale = 250.0f / 32768.0f;
    else if (dps == GYRO_RANGE_500_DPS)
      g_scale = 500.0f / 32768.0f;
    else if (dps == GYRO_RANGE_1000_DPS)
      g_scale = 1000.0f / 32768.0f;
    else if (dps == GYRO_RANGE_2000_DPS)
      g_scale = 2000.0f / 32768.0f;

    uint8_t val = (odr << 4) + (dps);
    return writeRegister(CTRL2_G, val);
  }

  bool setAccel(uint8_t odr, uint8_t range) {
    a_scale = 1.0f / 32768.0f;
    switch (range) {
      case ACCEL_RANGE_2_G:
        a_scale *= 2.0f;
        break;
      case ACCEL_RANGE_4_G:
        a_scale *= 4.0f;
        break;
      case ACCEL_RANGE_8_G:
        a_scale *= 8.0f;
        break;
      case ACCEL_RANGE_16_G:
        a_scale *= 16.0f;
        break;
    }

    uint8_t LPF2_XL_EN = 0; // kill LFP2
    uint8_t val = (odr << 4) + (range << 2) + (LPF2_XL_EN << 1);
    return writeRegister(CTRL1_XL, val);
  }

  float g_scale, a_scale;

  float acal[12]{ // accel scale/bias
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  };

  float gcal[12]{ // gyro scale/bias
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  };

  // converting between short, byte, and long
  union {
    int16_t s[3]; // signed shorts
    uint8_t b[6]; // bytes
    uint32_t l;   // long - timestamp
  } data;
};


} // namespace LSM6DSOX
