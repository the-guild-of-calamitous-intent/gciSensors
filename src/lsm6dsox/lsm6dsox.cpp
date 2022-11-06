#include "lsm6dsox.hpp"
#include "../units.hpp"

constexpr int WHO_AM_I_REG = 0X0F;
constexpr int WHO_AM_I = 0X6C;
constexpr int CTRL1_XL = 0X10; // Accel settings
constexpr int CTRL2_G = 0X11;  // Gyro settings hz and dps

constexpr int STATUS_REG = 0X1E;

constexpr int CTRL6_C = 0X15;  // Accel perf mode and Gyro LPF
constexpr int CTRL7_G = 0X16;  // Gyro filtering
constexpr int CTRL8_XL = 0X17; // Accel filtering

constexpr int OUT_TEMP_L = 0X20;
constexpr int OUT_TEMP_H = 0X21;

constexpr int OUTX_L_G = 0X22;
constexpr int OUTX_H_G = 0X23;
constexpr int OUTY_L_G = 0X24;
constexpr int OUTY_H_G = 0X25;
constexpr int OUTZ_L_G = 0X26;
constexpr int OUTZ_H_G = 0X27;

constexpr int OUTX_L_XL = 0X28;
constexpr int OUTX_H_XL = 0X29;
constexpr int OUTY_L_XL = 0X2A;
constexpr int OUTY_H_XL = 0X2B;
constexpr int OUTZ_L_XL = 0X2C;
constexpr int OUTZ_H_XL = 0X2D;

constexpr float TEMP_SCALE = 1.0f / 256.0f;

using namespace LSM6DSOX;

int gciLSM6DSOX::init() {
  //     uint8_t wai = readRegister(WHO_AM_I_REG);
  //   if (!(wai == WHO_AM_I /*|| wai == 0x69*/)) {
  //     // end();
  //     return 0;
  //   }

  if (!(readRegister(WHO_AM_I_REG) == WHO_AM_I)) {
    return 0;
  }

  // set the gyroscope control register to work at 104 Hz, 2000 dps and in
  // bypass mode
  //   writeRegister(CTRL2_G, 0x4C);
  setGyro(RATE_104_HZ, GYRO_RANGE_2000_DPS);

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass
  // mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
  // datasheet)
  //   writeRegister(CTRL1_XL, 0x4A);
  setAccel(RATE_104_HZ, ACCEL_RANGE_4_G);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  //   writeRegister(CTRL7_G, 0x00);

  // Set the ODR config register
  //   writeRegister(CTRL8_XL, 0x09); // ODR/4
  writeRegister(CTRL8_XL, 0x00); // ODR/2

  return 1;
}

/*
scale factor in rad/s
*/
int gciLSM6DSOX::setGyro(uint8_t odr, uint8_t dps) {
  if (dps == GYRO_RANGE_125_DPS)
    g_scale = 125.0f / 32768.0f * Units::deg2rad;
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
int gciLSM6DSOX::setAccel(uint8_t odr, uint8_t range) {

  if (range == ACCEL_RANGE_2_G)
    a_scale = 2.0 / 32768.0f;
  else if (range == ACCEL_RANGE_4_G)
    a_scale = 4.0 / 32768.0f;
  else if (range == ACCEL_RANGE_8_G)
    a_scale = 8.0 / 32768.0f;
  else if (range == ACCEL_RANGE_16_G)
    a_scale = 16.0 / 32768.0f;

  constexpr uint8_t LPF2_XL_EN = 0; // kill LFP2

  uint8_t val = (odr << 4) + (range << 2) + (LPF2_XL_EN << 1);
  return writeRegister(CTRL1_XL, val);
}

sensor_available_t gciLSM6DSOX::sensorsAvailable() {
  uint8_t val = readRegister(STATUS_REG);
  sensor_available_t ret;
  ret.accel = val & 0x01;
  ret.gyro = val & 0x02;
  ret.temp = val & 0x04;
  return ret;
}

// void gciLSM6DSOX::end() {
//   //   if (_spi != NULL) {
//   //     _spi->end();
//   //     digitalWrite(_csPin, LOW);
//   //     pinMode(_csPin, INPUT);
//   //   } else {
//   writeRegister(CTRL2_G, 0x00);
//   writeRegister(CTRL1_XL, 0x00);
// //   _wire->end();
//   //   }
// }

/*
accel - g's
gyro = rad/s
temp - C
*/
sox_t gciLSM6DSOX::read() {
  sox_t ret;

  if (!readRegisters(OUTX_L_XL, data.b, sizeof(data.b))) {
    ret.ok = false;
    return ret;
  }

  ret.ax = data.s[0] * a_scale;
  ret.ay = data.s[1] * a_scale;
  ret.az = data.s[2] * a_scale;

  if (!readRegisters(OUTX_L_G, data.b, sizeof(data.b))) {
    ret.ok = false;
    return ret;
  }

  ret.gx = data.s[0] * g_scale;
  ret.gy = data.s[1] * g_scale;
  ret.gz = data.s[2] * g_scale;

  if (readRegisters(OUT_TEMP_L, data.b, 2) != 1) {
    ret.ok = false;
    return ret;
  }
  ret.temp = data.s[0] * TEMP_SCALE + 25.0f;

  ret.ok = true;
  return ret;
}
