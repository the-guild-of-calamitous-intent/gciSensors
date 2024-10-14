/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "sensor.hpp"
#include <string.h> // memcpy

namespace LSM6DSOX {

constexpr int LSM6DSOX_ADDRESS   = 0x6A;

constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;
constexpr uint8_t REG_INT1_CTRL  = 0x0D;
constexpr uint8_t REG_INT2_CTRL  = 0x0E;
constexpr uint8_t REG_WHO_AM_I   = 0x0F;
constexpr uint8_t REG_CTRL1_XL   = 0x10; // Accel settings
constexpr uint8_t REG_CTRL2_G    = 0x11; // Gyro settings hz and dps
constexpr uint8_t REG_CTRL3_C    = 0x12; // interrupt stuff
constexpr uint8_t REG_CTRL4_C    = 0x13;
constexpr uint8_t REG_CTRL5_C    = 0x14;
constexpr uint8_t REG_CTRL6_C    = 0x15; // Accel perf mode and Gyro LPF
constexpr uint8_t REG_CTRL7_G    = 0x16; // Gyro filtering
constexpr uint8_t REG_CTRL8_XL   = 0x17; // Accel filtering
constexpr uint8_t REG_CTRL9_XL   = 0x18; // Accel filtering
constexpr uint8_t REG_CTRL10_C   = 0x19; // tiimestamp

constexpr uint8_t REG_STATUS     = 0x1E;

constexpr uint8_t REG_OUT_TEMP_L = 0x20; // termperature
constexpr uint8_t REG_OUTX_L_G   = 0x22; // gyro
constexpr uint8_t REG_OUTX_L_A   = 0x28; // accel
constexpr uint8_t REG_TIMESTAMP0 = 0x40; // 4B timestamp

// Betaflight values
// Accel:
//   833Hz ODR, 16G, use LPF1 output
//   high performance mode
// Gyro:
//   6664Hz ODR, 2000dps
//   LPF1 cutoff 335.5Hz
//
// latch LSB/MSB at reads, pins high, pins push/pull, auto-increment reads
// disable i3c interface


// The accelerometer/gyroscope data rate
enum ODR : uint8_t {
  RATE_SHUTDOWN = 0x00,
  RATE_104_HZ   = 0x40,
  RATE_208_HZ   = 0x50,
  RATE_416_HZ   = 0x60,
  RATE_833_HZ   = 0x70,
  RATE_1_66_KHZ = 0x80,
  RATE_3_33_KHZ = 0x90,
  RATE_6_66_KHZ = 0xA0,
};

// The accelerometer data range
enum accel_range : uint8_t {
  ACCEL_RANGE_2_G  = (0x00 << 2),
  ACCEL_RANGE_16_G = (0x01 << 2),
  ACCEL_RANGE_4_G  = (0x02 << 2),
  ACCEL_RANGE_8_G  = (0x03 << 2)
};

// The gyro data range
enum gyro_range : uint8_t {
  GYRO_RANGE_125_DPS  = (0x01 << 1),
  GYRO_RANGE_250_DPS  = (0x00 << 1),
  GYRO_RANGE_500_DPS  = (0x02 << 1),
  GYRO_RANGE_1000_DPS = (0x04 << 1),
  GYRO_RANGE_2000_DPS = (0x06 << 1)
};

constexpr uint8_t WHO_AM_I     = 0x6C; // 01101100
constexpr uint8_t IF_INC       = 0x04;
constexpr uint8_t XL_FS_MODE   = 0x02; // new mode, default 0
constexpr uint8_t TIMESTAMP_EN = 0x20;
constexpr uint8_t LPF2_XL_EN   = 0x02; // output from LPF2 second
                                       // filtering stage selected
                                       // (not default)
constexpr uint8_t INT_DRDY_XL   = 0x01; // accel data ready INT pin
constexpr uint8_t INT_DRDY_G    = 0x02; // gyro data ready INT pin
constexpr uint8_t INT_DRDY_TEMP = 0x04; // temperature data ready INT pin
// constexpr uint8_t H_LACTIVE    = 0x20; // 0-high, 1-low - don't set this

constexpr float LSM6DSOX_TIMESTEP_RES = 25e-6f;
constexpr float TEMP_SCALE            = 1.0f / 256.0f;


// struct sox_regs_t {
//   uint8_t CTRL1_XL;
//   uint8_t CTRL2_G;
//   uint8_t CTRL3_C;
//   uint8_t CTRL4_C;
//   uint8_t CTRL5_C;
//   uint8_t CTRL6_C;
//   uint8_t CTRL7_G;
//   uint8_t CTRL8_XL;
//   uint8_t CTRL9_XL;
//   uint8_t CTRL10_C;
// };

using lsm6dsox_t = gci::sensors::imu_t;
using lsm6dsox_raw_t = gci::sensors::imu_raw_t;

// TDA: temperature
// GDA: gyro
// XLDA: accel
//                             4   2    1
// STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
enum sox_available_t : uint8_t {
  SOX_NONE            = 0,
  SOX_ACCEL           = 1,
  SOX_GYRO            = 2,
  SOX_ACCEL_GYRO      = 3,
  SOX_TEMP            = 4,
  SOX_ACCEL_GYRO_TEMP = 7,
};

enum sox_error : uint8_t {
  ERROR_NONE             = 0,
  ERROR_WHOAMI           = 1,
  ERROR_GYRO_RANGE       = 2,
  ERROR_ACCEL_RANGE      = 3,
  ERROR_ENABLE_INT_ACCEL = 9,
  ERROR_ENABLE_INT_GYRO  = 10,
  ERROR_CTRL1_XL = 11,
  ERROR_CTRL2_G = 12,
  ERROR_CTRL3_C = 13,
  ERROR_CTRL4_C = 14,
  ERROR_CTRL5_C = 15,
  ERROR_CTRL6_C = 16,
  ERROR_CTRL7_G = 17,
  ERROR_CTRL8_XL = 18,
  ERROR_CTRL9_XL = 19,
  ERROR_CTRL10_C = 20
};

// constexpr int MAX_CHECK = 10;

class gciLSM6DSOX : public SensorI2C {
public:
  gciLSM6DSOX(const uint32_t port, uint8_t addr = LSM6DSOX_ADDRESS)
      : SensorI2C(addr, port) {}
  ~gciLSM6DSOX() {}

  uint8_t init(uint8_t accel_range = ACCEL_RANGE_4_G,
               uint8_t gyro_range  = GYRO_RANGE_2000_DPS,
               uint8_t odr         = RATE_104_HZ) {

    uint8_t id{0};
    readRegister(REG_WHO_AM_I, &id);
    if (!(id == WHO_AM_I)) return ERROR_WHOAMI;

    // reset memory
    // MSB [ BOOT BDU H_LACTIVE PP_OD SIM IF_INC 0 SW_RESET ] LSB
    // if (!writeRegister(REG_CTRL3_C, 0x84)) return 99;

    // Set the Accelerometer control register to work at 104 Hz, 4 g,and in
    // bypass mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
    // datasheet)
    // kill LFP2 (default)
    switch (accel_range) {
    case ACCEL_RANGE_2_G:
      a_scale = 2.0f / 32768.0f;
      break;
    case ACCEL_RANGE_4_G:
      a_scale = 4.0f / 32768.0f;
      break;
    case ACCEL_RANGE_8_G:
      a_scale = 8.0f / 32768.0f;
      break;
    case ACCEL_RANGE_16_G:
      a_scale = 16.0f / 32768.0f;
      break;
    default:
      return ERROR_ACCEL_RANGE;
    }
    if (!writeRegister(REG_CTRL1_XL, odr | accel_range))
      return ERROR_CTRL1_XL;

    if (gyro_range == GYRO_RANGE_125_DPS) g_scale = 125.0f / 32768.0f;
    else if (gyro_range == GYRO_RANGE_250_DPS) g_scale = 250.0f / 32768.0f;
    else if (gyro_range == GYRO_RANGE_500_DPS) g_scale = 500.0f / 32768.0f;
    else if (gyro_range == GYRO_RANGE_1000_DPS) g_scale = 1000.0f / 32768.0f;
    else if (gyro_range == GYRO_RANGE_2000_DPS) g_scale = 2000.0f / 32768.0f;
    else return ERROR_GYRO_RANGE;

    if (!writeRegister(REG_CTRL2_G, odr | gyro_range)) return ERROR_CTRL2_G;

    // auto-increament during multi-byte reads
    // continous sampling BDU = 0
    if (!writeRegister(REG_CTRL3_C, IF_INC)) return ERROR_CTRL3_C;

    // disable fifo
    // LSM6DSOX_FIFO_CTRL4 bypassmode (0)
    uint8_t DRDY_MASK = 0x08;
    // if (!writeRegister(REG_FIFO_CTRL4, 0x00)) return ERROR_DISABLE_FIFO;
    if (!writeRegister(REG_FIFO_CTRL4, DRDY_MASK)) return ERROR_CTRL4_C;

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    if (!writeRegister(REG_CTRL7_G, 0x00)) return ERROR_CTRL7_G;

    // Set LPF and HPF config register
    // LPF ODR/2, HPCF_XL = 0, LPF2_XL_EN = 0
    // disable HPF, HP_REF_MODE_XL = 0x00
    if (!writeRegister(REG_CTRL8_XL, 0x00)) return ERROR_CTRL8_XL;
    // if (!writeRegister(REG_CTRL8_XL, XL_FS_MODE)) return ERROR_CTRL8_XL;

    // disable I3C
    // LSM6DSOX_CTRL9_XL
    // LSM6DSOX_I3C_BUS_AVB
    // uint8_t val = 0xD0; // 0b1110000; // these are default
    // if (!writeRegister(REG_CTRL9_XL, val)) return ERROR_CTRL9_XL;

    // enable timestamp
    if (!writeRegister(REG_CTRL10_C, TIMESTAMP_EN))
      return ERROR_CTRL10_C;

    // enable INT1 and INT2 pins when data is ready
    if (!writeRegister(REG_INT1_CTRL, INT_DRDY_XL))
      return ERROR_ENABLE_INT_ACCEL; // accel
    if (!writeRegister(REG_INT2_CTRL, INT_DRDY_G))
      return ERROR_ENABLE_INT_GYRO; // gyro

    return ERROR_NONE;
  }

  // MSB 10000101 LSB = 128 + 4 + 1 = 133
  bool reboot() { return writeRegister(REG_CTRL3_C, 133); }

  void set_accel_cal(float cal[12]) { memcpy(acal, cal, 12 * sizeof(float)); }
  void set_gyro_cal(float cal[12]) { memcpy(gcal, cal, 12 * sizeof(float)); }

  const lsm6dsox_raw_t read_raw() {
    lsm6dsox_raw_t ret;
    ret.ok = false;

    // Serial.println(ready());
    if (!ready()) return ret;

    if (!readRegisters(REG_OUT_TEMP_L, sizeof(block.b), block.b)) return ret;

    ret.a.x  = block.a.x;
    ret.a.y  = block.a.y;
    ret.a.z  = block.a.z;
    ret.g.x  = block.g.x;
    ret.g.y  = block.g.y;
    ret.g.z  = block.g.z;
    ret.temperature = block.temperature; // 52Hz, pg13, Table 4

    if (!readRegisters(REG_TIMESTAMP0, 4, block.b)) return ret;

    // pg 13, Table 4, temp ODR is ~52Hz
    ret.ts = block.timestamp; // 25 usec per count
    ret.ok = true;

    return ret;
  }

  lsm6dsox_t read() { // accel - g's, gyro - dps, temp - C
    const lsm6dsox_raw_t raw = read_raw();
    lsm6dsox_t ret;
    ret.ok = false;
    if (raw.ok == false) return ret;

    ret.a.x  = raw.a.x * a_scale;
    ret.a.y  = raw.a.y * a_scale;
    ret.a.z  = raw.a.z * a_scale;
    ret.g.x  = raw.g.x * g_scale;
    ret.g.y  = raw.g.y * g_scale;
    ret.g.z  = raw.g.z * g_scale;
    ret.temperature = static_cast<float>(raw.temperature) * TEMP_SCALE + 25.0f;
    ret.ts   = raw.ts; // 25 usec per count
    ret.ok   = true;

    return ret;
  }

  lsm6dsox_t read_cal() { // accel - g's, gyro - dps, temp - C
    const lsm6dsox_t m = read();
    if (m.ok == false) return m;

    lsm6dsox_t ret;
    ret.ok = false;

    // accel = A * accel_meas - bias
    ret.a.x = acal[0] * m.a.x + acal[1] * m.a.y + acal[2] * m.a.z - acal[3];
    ret.a.y = acal[4] * m.a.x + acal[5] * m.a.y + acal[6] * m.a.z - acal[7];
    ret.a.z = acal[8] * m.a.x + acal[9] * m.a.y + acal[10] * m.a.z - acal[11];

    // gyro = A * gyro_meas - bias
    ret.g.x  = gcal[0] * m.g.x + gcal[1] * m.g.y + gcal[2] * m.g.z - gcal[3];
    ret.g.y  = gcal[4] * m.g.x + gcal[5] * m.g.y + gcal[6] * m.g.z - gcal[7];
    ret.g.z  = gcal[8] * m.g.x + gcal[9] * m.g.y + gcal[10] * m.g.z - gcal[11];

    ret.ts   = m.ts; // 25 usec per count
    ret.temperature = m.temperature;
    ret.ok   = true;

    return ret;
  }

  bool ready() {
    // TDA: temperature
    // GDA: gyro
    // XLDA: accel
    //                             4   2    1
    // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
    // return readRegister(REG_STATUS);
    // uint8_t val = readRegister(REG_STATUS) & 3;
    uint8_t val{0};
    readRegister(REG_STATUS, &val);
    return val < 3 ? false : true;
    // return val == 3;
  }

  sox_available_t available() {
    // TDA: temperature
    // GDA: gyro
    // XLDA: accel
    //                             4   2    1
    // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
    uint8_t val{0};
    readRegister(REG_STATUS, &val);
    return static_cast<sox_available_t>(val);
  }

  // sox_regs_t getRegs() {
  //   uint8_t reg;
  //   sox_regs_t regs;
  //   readRegister(REG_CTRL1_XL, &reg);
  //   regs.CTRL1_XL = reg;
  //   readRegister(REG_CTRL2_G);
  //   regs.CTRL2_G  = reg;
  //   readRegister(REG_CTRL3_C);
  //   regs.CTRL3_C  = reg;
  //   regs.CTRL4_C  = readRegister(REG_CTRL4_C);
  //   regs.CTRL5_C  = readRegister(REG_CTRL5_C);
  //   regs.CTRL6_C  = readRegister(REG_CTRL6_C);
  //   regs.CTRL7_G  = readRegister(REG_CTRL7_G);
  //   regs.CTRL8_XL = readRegister(REG_CTRL8_XL);
  //   regs.CTRL9_XL = readRegister(REG_CTRL9_XL);
  //   regs.CTRL10_C = readRegister(REG_CTRL10_C);
  //   return regs;
  // }

private:
  float g_scale, a_scale;

  // accel scale/bias
  float acal[12]{1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
  // gyro scale/bias
  float gcal[12]{1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};

  union block_t {
    struct {
      int16_t temperature;       // 2b, -40C to 80C
      gci::sensors::vec_raw_t g; // 2*3 = 6b
      gci::sensors::vec_raw_t a; // 2*3 = 6b
    };                           // 14b
    uint32_t timestamp;
    uint8_t b[14];
  } block;
};

} // namespace LSM6DSOX













