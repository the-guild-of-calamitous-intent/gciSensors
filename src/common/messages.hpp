/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h>
#include <math.h>
#include "quaternions.hpp"
#include "vectors.hpp"

namespace gci {
namespace sensors {


///////////////////////////////////////////////////////////
// IMU (Accels, Gyros)
///////////////////////////////////////////////////////////

struct imu_t {
  vec_t a,g;
  float temperature;
  bool ok;
  uint32_t ts;
};

struct imu_raw_t {
  vec_raw_t a,g;
  int16_t temperature; // lsm6dsox is only int16_t
  bool ok;
  uint32_t ts;
};


///////////////////////////////////////////////////////////
// Vector Message
///////////////////////////////////////////////////////////

struct vec_msg_t {
  float x, y, z;
  bool ok;  // error?

  inline const float magnitude() const { return sqrtf(x * x + y * y + z * z); }

  bool normalize() {
    float n = 1.0f / magnitude();
    if (std::isinf(n)) return false;

    x *= n;
    y *= n;
    z *= n;

    return true;
  }

  float operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};

struct vec_msg_raw_t {
  int16_t x, y, z;
  bool ok;  // error?

  int16_t operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};


///////////////////////////////////////////////////////////
// Pressure / Temperature
///////////////////////////////////////////////////////////

struct pt_raw_t {
  int32_t pressure, temperature;
  bool ok;

  int32_t operator[](size_t i) {
    return (i == 0) ? pressure : temperature;
  }
};

struct pt_t {
  float pressure, temperature;
  bool ok;

  float operator[](size_t i) {
    return (i == 0) ? pressure : temperature;
  }
};


} // namespace sensors
} // namespace gci