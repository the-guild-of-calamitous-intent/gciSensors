/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "../sensor.hpp"

namespace LSM6DSOX {

constexpr int LSM6DSOX_ADDRESS = 0x6A;

/** The accelerometer data rate */
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

/** The accelerometer data range */
enum accel_range : uint8_t {
  ACCEL_RANGE_2_G,
  ACCEL_RANGE_16_G,
  ACCEL_RANGE_4_G,
  ACCEL_RANGE_8_G
};

// constexpr uint8_t  ACCEL_RANGE_2_G = 0;
// constexpr uint8_t  ACCEL_RANGE_16_G = 1;
// constexpr uint8_t  ACCEL_RANGE_4_G = 2;
// constexpr uint8_t  ACCEL_RANGE_8_G = 3;

/** The gyro data range */
enum gyro_range : uint8_t {
  GYRO_RANGE_125_DPS = 0b0010,
  GYRO_RANGE_250_DPS = 0b0000,
  GYRO_RANGE_500_DPS = 0b0100,
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

struct sox_t {
  float ax, ay, az, gx, gy, gz, temp;
  bool ok;
};


struct sensor_available_t {
  bool accel, gyro, temp; // sensor available?
};

class gciLSM6DSOX : public Sensor {
public:
  gciLSM6DSOX(TwoWire *wire, uint8_t addr = LSM6DSOX_ADDRESS)
      : Sensor(wire, addr) {}
  ~gciLSM6DSOX() {}

  bool init();

  sox_t read(); // accel - g's, gyro - rad/s, temp - C
  bool setGyro(uint8_t odr, uint8_t dps);
  bool setAccel(uint8_t odr, uint8_t range);
  sensor_available_t sensorsAvailable();

private:
  float g_scale, a_scale;

  union {
    int16_t s[3]; // signed shorts
    uint8_t b[6]; // bytes
  } data;
};

} // namespace LSM6DSOX
