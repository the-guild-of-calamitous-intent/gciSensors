#include "lsm6dsox.hpp"
#include "../units.hpp"

constexpr uint8_t WHO_AM_I_REG = 0x0F;
constexpr uint8_t WHO_AM_I     = 0x6C;
constexpr uint8_t CTRL1_XL     = 0x10; // Accel settings
constexpr uint8_t CTRL2_G      = 0x11; // Gyro settings hz and dps

constexpr uint8_t STATUS_REG   = 0x1E;

constexpr uint8_t CTRL6_C      = 0x15; // Accel perf mode and Gyro LPF
constexpr uint8_t CTRL7_G      = 0x16; // Gyro filtering
constexpr uint8_t CTRL8_XL     = 0x17; // Accel filtering

constexpr uint8_t OUT_TEMP_L   = 0x20;
constexpr uint8_t OUT_TEMP_H   = 0x21;

constexpr uint8_t OUTX_L_G     = 0x22;
constexpr uint8_t OUTX_H_G     = 0x23;
constexpr uint8_t OUTY_L_G     = 0x24;
constexpr uint8_t OUTY_H_G     = 0x25;
constexpr uint8_t OUTZ_L_G     = 0x26;
constexpr uint8_t OUTZ_H_G     = 0x27;

constexpr uint8_t OUTX_L_XL    = 0x28;
constexpr uint8_t OUTX_H_XL    = 0x29;
constexpr uint8_t OUTY_L_XL    = 0x2A;
constexpr uint8_t OUTY_H_XL    = 0x2B;
constexpr uint8_t OUTZ_L_XL    = 0x2C;
constexpr uint8_t OUTZ_H_XL    = 0x2D;

constexpr float TEMP_SCALE     = 1.0f / 256.0f;

using namespace LSM6DSOX;

bool gciLSM6DSOX::init() {

  if (!(readRegister(WHO_AM_I_REG) == WHO_AM_I)) {
    return false;
  }

  // set the gyroscope control register to work at 104 Hz, 2000 dps and in
  // bypass mode
  //   writeRegister(CTRL2_G, 0x4C);
  bool ok = setGyro(RATE_104_HZ, GYRO_RANGE_2000_DPS);
  if (!ok) return false;

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass
  // mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
  // datasheet)
  ok = setAccel(RATE_104_HZ, ACCEL_RANGE_4_G);
  if (!ok) return false;

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  //   writeRegister(CTRL7_G, 0x00);

  // Set the ODR config register
  // ok = writeRegister(CTRL8_XL, 0x09); // ODR/4
  ok = writeRegister(CTRL8_XL, 0x00); // ODR/2
  if (!ok) return false;

  // printf(">> init done ...");

  return true;
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

  if (range == ACCEL_RANGE_2_G) a_scale = 2.0 / 32768.0f;
  else if (range == ACCEL_RANGE_4_G) a_scale = 4.0 / 32768.0f;
  else if (range == ACCEL_RANGE_8_G) a_scale = 8.0 / 32768.0f;
  else if (range == ACCEL_RANGE_16_G) a_scale = 16.0 / 32768.0f;

  constexpr uint8_t LPF2_XL_EN = 0; // kill LFP2

  uint8_t val                  = (odr << 4) + (range << 2) + (LPF2_XL_EN << 1);
  return writeRegister(CTRL1_XL, val);
}

sensor_available_t gciLSM6DSOX::sensorsAvailable() {
  uint8_t val = readRegister(STATUS_REG);
  sensor_available_t ret;
  ret.accel = val & 0x01;
  ret.gyro  = val & 0x02;
  ret.temp  = val & 0x04;
  return ret;
}

/*
accel - g's
gyro = rad/s
temp - C
*/
sox_t gciLSM6DSOX::read() {
  sox_t ret;

  if (!readRegisters(OUTX_L_XL, sizeof(data.b), data.b)) {
    ret.ok = false;
    return ret;
  }

  float x = data.s[0] * a_scale;
  float y = data.s[1] * a_scale;
  float z = data.s[2] * a_scale;

#if IMU_USE_UNCALIBRATED_DATA
  ret.ax = x;
  ret.ay = y;
  ret.az = z;
#else
  ret.ax = sm[0][0] * x + sm[0][1] * y + sm[0][2] * z + sm[0][3];
  ret.ay = sm[1][0] * x + sm[1][1] * y + sm[1][2] * z + sm[1][3];
  ret.az = sm[2][0] * x + sm[2][1] * y + sm[2][2] * z + sm[2][3];
#endif

  if (!readRegisters(OUTX_L_G, sizeof(data.b), data.b)) {
    ret.ok = false;
    return ret;
  }

  x = data.s[0] * g_scale;
  y = data.s[1] * g_scale;
  z = data.s[2] * g_scale;

#if IMU_USE_UNCALIBRATED_DATA
  ret.gx = x;
  ret.gy = y;
  ret.gz = z;
#else
  ret.gx = (x - gbias[0]);
  ret.gy = (y - gbias[1]);
  ret.gz = (z - gbias[2]);
#endif

  if (readRegisters(OUT_TEMP_L, 2, data.b) != 1) {
    ret.ok = false;
    return ret;
  }
  ret.temp = data.s[0] * TEMP_SCALE + 25.0f;

  ret.ok   = true;
  return ret;
}
