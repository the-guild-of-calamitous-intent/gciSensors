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

constexpr uint8_t REG_FIFO_CTRL4  = 0x0A;
constexpr uint8_t REG_INT1_CTRL = 0x0D;
constexpr uint8_t REG_INT2_CTRL = 0x0E;
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL     = 0x10; // Accel settings
constexpr uint8_t CTRL2_G      = 0x11; // Gyro settings hz and dps
constexpr uint8_t REG_CTRL3_C  = 0x12; // interrupt stuff
constexpr uint8_t CTRL6_C      = 0x15; // Accel perf mode and Gyro LPF
constexpr uint8_t CTRL7_G      = 0x16; // Gyro filtering
constexpr uint8_t REG_CTRL8_XL     = 0x17; // Accel filtering
constexpr uint8_t REG_CTRL9_XL     = 0x18; // Accel filtering
constexpr uint8_t REG_CTRL10_C     = 0x19; // tiimestamp

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

constexpr uint8_t WHO_AM_I     = 0x6C; // 01101100

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

// The high pass filter bandwidth
// enum hpf_range: uint8_t {
//   HPF_ODR_DIV_50 = 0,
//   HPF_ODR_DIV_100 = 1,
//   HPF_ODR_DIV_9 = 2,
//   HPF_ODR_DIV_400 = 3,
// };

constexpr float LSM6DSOX_TIMESTEP_RES = 25e-6;

using gci::sensors::vecf_t;
using gci::sensors::veci_t;

struct sox_t {
  // float ax, ay, az, gx, gy, gz, temp;
  vecf_t a, g;
  float temp;
  uint32_t ts;
  bool ok;
};

struct sox_raw_t {
  // int16_t ax, ay, az, gx, gy, gz, temp;
  veci_t a, g;
  int16_t temp;
  uint32_t ts;
  bool ok;
};

// struct lsm6_available_t {
//   bool accel, gyro, temp; // sensor available?
// };

enum sox_available_t: uint8_t {
  SOX_NONE = 0,
  SOX_ACCEL = 1,
  SOX_GYRO = 2,
  SOX_ACCEL_GYRO = 3,
  SOX_TEMP = 4,
  SOX_ACCEL_GYRO_TEMP = 7,
};

enum sox_error: uint8_t {
  ERROR_NONE = 0,
  ERROR_WHOAMI = 1,
  ERROR_GYRO_RANGE = 2,
  ERROR_ACCEL_RANGE = 3,
  ERROR_ENABLE_FILTER = 4,
  ERROR_ENABLE_TIMESTAMP = 5,
  ERROR_ENABLE_BDU = 6,
  ERROR_DISABLE_FIFO = 7,
  ERROR_DISABLE_I3C = 8
};

constexpr int MAX_CHECK = 10;

class gciLSM6DSOX : public SensorI2C {
public:
  gciLSM6DSOX(TwoWire *wire, uint8_t addr = LSM6DSOX_ADDRESS)
      : SensorI2C(wire, addr) {}
  ~gciLSM6DSOX() {}

  uint8_t init(
        uint8_t accel_range=ACCEL_RANGE_4_G,
        uint8_t gyro_range=GYRO_RANGE_2000_DPS,
        uint8_t odr=RATE_104_HZ) {

    // if (!(readRegister(REG_WHO_AM_I) == WHO_AM_I)) return ERROR_WHOAMI;
    for (int i=0; i <= MAX_CHECK; ++i) {
      uint8_t who = readRegister(REG_WHO_AM_I);
      if (who == WHO_AM_I) break;
      if (i == MAX_CHECK) return who; //ERROR_WHOAMI;
      delay(10);
    }

    // diable I3C
    // LSM6DSOX_CTRL9_XL
    // LSM6DSOX_I3C_BUS_AVB
    uint8_t val = 0b1110000;
    if (!writeRegister(REG_CTRL9_XL, val)) return ERROR_DISABLE_I3C;

    // auto-increament during multi-byte reads
    // LSM6DSOX_CTRL3_C - set by default
    // enable BDU
    // LSM6DSOX_CTRL3_C
    // uint8_t reg = readRegister(REG_FIFO_CTRL4);
    // reg &= ~(1<<6); // really the setting is 0x04 which does multi-byte reads
    uint8_t BDU = 0; // continous sampling
    uint8_t IF_INC = 1; // addr incremented during multi-byte reads
    val = (BDU << 6) + (IF_INC << 2);
    if (!writeRegister(REG_CTRL3_C, val)) return ERROR_ENABLE_BDU;

    // disable fifo
    // LSM6DSOX_FIFO_CTRL4 bypassmode (0)
    // if (!writeRegister(REG_FIFO_CTRL4, 0x00)) return ERROR_DISABLE_FIFO;

    // set the gyroscope control register to work at 104 Hz, 2000 dps and in
    // bypass mode
    //   writeRegister(CTRL2_G, 0x4C);
    if (!setGyro(odr, gyro_range)) return ERROR_GYRO_RANGE;

    // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass
    // mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
    // datasheet)
    if (!setAccel(odr, accel_range)) return ERROR_ACCEL_RANGE;

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    //   writeRegister(CTRL7_G, 0x00);

    // Set LPF and HPF config register
    // if (!writeRegister(REG_CTRL8_XL, 0x00)) return ERROR_ENABLE_FILTER; // LPF ODR/2, disable HPF

    if (!writeRegister(REG_CTRL8_XL, 0x02)) return ERROR_ENABLE_FILTER;

    if (!writeRegister(REG_CTRL10_C, BITS::b5)) return ERROR_ENABLE_TIMESTAMP; // enable timestamp

    // Serial.println("pre-iinterrupt");
    // if (!set_interrupts(true)) return false; // set interrupts
    // Serial.println(">> init done ...");

    return ERROR_NONE;
  }

  // MSB 10000101 LSB = 128 + 4 + 1 = 133
  bool reboot() { return writeRegister(REG_CTRL3_C, 133); }

  void set_acal(float cal[12]) { memcpy(acal, cal, 12*sizeof(float)); }
  void set_gcal(float cal[12]) { memcpy(gcal, cal, 12*sizeof(float)); }

  const sox_raw_t read_raw() {
    sox_raw_t ret{0};
    ret.ok = false;

    if (!ready()) return ret;

    if (!readRegisters(REG_OUT_TEMP_L, sizeof(block.b), block.b)) return ret;

    ret.a.x = block.a.x;
    ret.a.y = block.a.y;
    ret.a.z = block.a.z;
    ret.g.x = block.g.x;
    ret.g.y = block.g.y;
    ret.g.z = block.g.z;
    ret.temp = block.temperature;

    if (!readRegisters(REG_TIMESTAMP0, 4, block.b)) return ret;

    ret.ts = block.timestamp; // 25 usec per count

    ret.ok   = true;
    return ret;
  }

  sox_t read() { // accel - g's, gyro - dps, temp - C
    const sox_raw_t raw = read_raw();
    sox_t ret{0};
    ret.ok = false;
    if (raw.ok == false) return ret;

    ret.a.x = raw.a.x * a_scale;
    ret.a.y = raw.a.y * a_scale;
    ret.a.z = raw.a.z * a_scale;
    ret.g.x = raw.g.x * g_scale;
    ret.g.y = raw.g.y * g_scale;
    ret.g.z = raw.g.z * g_scale;
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

    // accel = A * accel_meas - bias
    ret.a.x  = acal[0] * m.a.x + acal[1] * m.a.y + acal[2] * m.a.z - acal[3];
    ret.a.y  = acal[4] * m.a.x + acal[5] * m.a.y + acal[6] * m.a.z - acal[7];
    ret.a.z  = acal[8] * m.a.x + acal[9] * m.a.y + acal[10] * m.a.z - acal[11];

    // gyro = A * gyro_meas - bias
    ret.g.x  = gcal[0] * m.g.x + gcal[1] * m.g.y + gcal[2] * m.g.z - gcal[3];
    ret.g.y  = gcal[4] * m.g.x + gcal[5] * m.g.y + gcal[6] * m.g.z - gcal[7];
    ret.g.z  = gcal[8] * m.g.x + gcal[9] * m.g.y + gcal[10] * m.g.z - gcal[11];

    ret.ts = m.ts; // 25 usec per count
    ret.temp = m.temp;

    ret.ok = true;

    return ret;
  }

  bool ready() {
    // TDA: temperature
    // GDA: gyro
    // XLDA: accel
    //                             4   2    1
    // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
    uint8_t val = readRegister(REG_STATUS);
    return val >= 3;
  }

  sox_available_t available() {
    // TDA: temperature
    // GDA: gyro
    // XLDA: accel
    //                             4   2    1
    // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
    uint8_t val = readRegister(REG_STATUS);
    return static_cast<sox_available_t>(val);
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
    ret &= writeRegister(REG_CTRL3_C, H_LACTIVE | IF_INC); // INT high
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

    uint8_t LPF2_XL_EN = 0; // kill LFP2 (default)
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

  struct vec_t {
    uint16_t x,y,z;
  };

  union {
    struct {
      uint16_t temperature;  // 2b
      vec_t g; // 2*3 = 6b
      vec_t a; // 2*3 = 6b
    }; // 14b
    uint32_t timestamp;
    uint8_t b[14];
  } block;
};


} // namespace LSM6DSOX
