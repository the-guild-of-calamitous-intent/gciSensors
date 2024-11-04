/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h>
#include <math.h>


// namespace gci {
namespace sensors {


///////////////////////////////////////////////////////////
// Vector (X,Y,Z) Sensors
///////////////////////////////////////////////////////////

// template <typename T>
struct __attribute__((packed)) vec_t {
  vec_t(): x(0.0f), y(0.0f), z(0.0f) {}
  vec_t(float x, float y, float z): x(x), y(y), z(z) {}
  float x, y, z;

  inline float magnitude() const { return sqrtf(x * x + y * y + z * z); }

  bool normalize() {
    float n = 1.0f / magnitude();
    if (std::isinf(n)) return false;

    x *= n;
    y *= n;
    z *= n;

    return true;
  }
};


struct vec_raw_t {
  int16_t x, y, z;
  // bool ok;  // error?

  int16_t operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};


} // namespace sensors
// } // namespace gci