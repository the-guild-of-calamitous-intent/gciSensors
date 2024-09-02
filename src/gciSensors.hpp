/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <cmath> // isinf
#include <cstdint>
// #include <stdio.h>  // FIXME: remove
#include <string.h> // memcpy

#if !defined(PICO_SDK) // FIXME: handle better
#include <unistd.h>
void sleep_ms(uint32_t ms) { usleep(ms*1000); }
void sleep_us(uint64_t us) { usleep(us); }
#endif

// namespace gci {

// namespace sensors {
// template <typename T> struct vec_t {
//   T x, y, z;
//   bool ok;  // error?

//   inline const T magnitude() const { return sqrt(x * x + y * y + z * z); }

//   bool normalize() {
//     T n = 1.0 / magnitude();
//     if (std::isinf(n)) return false;

//     x *= n;
//     y *= n;
//     z *= n;

//     return true;
//   }

//   T operator[](size_t i) {
//     return (i == 0) ? x : (i == 1) ? y : z;
//   }
// };

// using vecf_t = vec_t<float>;
// using vecd_t = vec_t<double>;
// using veci_t = vec_t<int16_t>;

// // class Hertz {
// // public:
// //   Hertz(uint32_t v = 300) : threshold(v), epoch(millis()) {}

// //   bool check() {
// //     if (++count % threshold == 0) {
// //       uint32_t now = millis();
// //       hertz        = 1000.0f * float(count) / float(now - epoch);
// //       epoch        = now;
// //       count        = 0;
// //       return true;
// //     }

// //     return false;
// //   }

// //   float hertz{0.0f};

// // protected:
// //   uint32_t epoch;
// //   uint32_t count{0};
// //   const uint32_t threshold;
// // };

// } // namespace sensors
// } // namespace gci

#include "common.hpp"

inline uint32_t to_24b(uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
}

inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

// returns altitude in meters
float altitude(float pressure, float seaLevelhPa = 1013.25) {
  float alt = 44330 * (1.0 - pow((pressure / 100) / seaLevelhPa, 0.1903));
  return alt;
}

float altitude2(const float p) {
  // Probably best not to run here ... very computational.
  // pre compute some of this?
  // call atmospalt() ... like matlab?
  // same as mean sea level (MSL) altitude
  // Altitude from pressure:
  // https://www.mide.com/air-pressure-at-altitude-calculator
  // const float Tb = 15; // temperature at sea level [C] - doesn't work
  // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
  constexpr float Tb  = 288.15f;           // temperature at sea level [K]
  constexpr float Lb  = -0.0065f;          // lapse rate [K/m]
  constexpr float Pb  = 101325.0f;         // pressure at sea level [Pa]
  constexpr float R   = 8.31446261815324f; // universal gas const [Nm/(mol K)]
  constexpr float M   = 0.0289644f; // molar mass of Earth's air [kg/mol]
  constexpr float g0  = 9.80665f;   // gravitational const [m/s^2]

  constexpr float exp = -R * Lb / (g0 * M);
  constexpr float scale  = Tb / Lb;
  constexpr float inv_Pb = 1.0f / Pb;

  return scale * (pow(p * inv_Pb, exp) - 1.0);
}

// sensor drivers
#include "bmp3.hpp"
#include "dps310.hpp"
#include "lis3mdl.hpp"
#include "lsm6dsox.hpp"
#include "pa1010d.hpp"
