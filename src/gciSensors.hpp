/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <cmath> // isinf
#include <cstdint>
#include <string.h> // memcpy

#if !defined(ARDUINO)
inline void delay(int) {} // FIXME: do a better way!

inline uint32_t millis() { return 0; }
#else
  #include <Arduino.h>
#endif

inline void sleep_ms(uint32_t ms) {
#if defined(ARDUINO)
  delay(ms);
#endif
}

inline void sleep_us(uint16_t us) {
#if defined(ARDUINO)
  delayMicroseconds(us);
#endif
}

namespace gci {

namespace sensors {
template <typename T> struct vec_t {
  T x, y, z;
  // bool ok;  // error?

  inline const T magnitude() const { return sqrt(x * x + y * y + z * z); }

  bool normalize() {
    T n = 1.0 / magnitude();
    if (std::isinf(n)) return false;

    x *= n;
    y *= n;
    z *= n;

    return true;
  }
};

using vecf_t = vec_t<float>;
using vecd_t = vec_t<double>;
using veci_t = vec_t<int16_t>;

class Hertz {
public:
  Hertz(uint32_t v = 300) : threshold(v), epoch(millis()) {}

  bool check() {
    if (++count % threshold == 0) {
      uint32_t now = millis();
      hertz        = 1000.0f * float(count) / float(now - epoch);
      epoch        = now;
      count        = 0;
      return true;
    }

    return false;
  }

  float hertz{0.0f};

protected:
  uint32_t epoch;
  uint32_t count{0};
  const uint32_t threshold;
};

} // namespace sensors
} // namespace gci

// sensor drivers
#include "bmp3.hpp"
#include "lis3mdl.hpp"
#include "lsm6dsox.hpp"
