/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <string.h> // memcpy
#include <cmath> // isinf

// #if defined(ARDUINO)
//   #include <Arduino.h>
//   #include <Wire.h>
// #else
//   #include <Wire.hpp>
// #endif

namespace gcisensors {

template<typename T>
struct vec_t {
  T x, y, z;
  bool ok;  // error?

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

using vecf_t = vec_t<float>;
using vecd_t = vec_t<double>;




} // gcisensors

// sensor drivers
// #include "sensor.hpp"
// #include "units.hpp"
#include "bmp3.hpp"
#include "lis3mdl.hpp"
#include "lsm6dsox.hpp"
#include "filters.hpp"
