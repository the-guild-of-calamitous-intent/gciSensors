/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/

#pragma once

#include <stdint.h>

// // returns altitude in meters
// float altitude(float pressure, float seaLevelhPa = 1013.25) {
//   float alt = 44330 * (1.0 - pow((pressure / 100) / seaLevelhPa, 0.1903));
//   return alt;
// }

// float altitude(const float p) {
//   // Probably best not to run here ... very computational.
//   // pre compute some of this?
//   // call atmospalt() ... like matlab?
//   // same as mean sea level (MSL) altitude
//   // Altitude from pressure:
//   // https://www.mide.com/air-pressure-at-altitude-calculator
//   // const float Tb = 15; // temperature at sea level [C] - doesn't work
//   // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
//   constexpr float Tb  = 288.15f;           // temperature at sea level [K]
//   constexpr float Lb  = -0.0065f;          // lapse rate [K/m]
//   constexpr float Pb  = 101325.0f;         // pressure at sea level [Pa]
//   constexpr float R   = 8.31446261815324f; // universal gas const [Nm/(mol K)]
//   constexpr float M   = 0.0289644f; // molar mass of Earth's air [kg/mol]
//   constexpr float g0  = 9.80665f;   // gravitational const [m/s^2]

//   constexpr float exp = -R * Lb / (g0 * M);
//   constexpr float scale  = Tb / Lb;
//   constexpr float inv_Pb = 1.0f / Pb;

//   return scale * (powf(p * inv_Pb, exp) - 1.0f);
// }

// returns altitude in meters ... same as below, but more details
// plus, most is computed at compile time, so no performance difference
// float altitude1(const float pressure) {
//   // constexpr float inv_seaLevelhPa = 1.0f / 1013.25f;
//   constexpr float inv_seaLevelPa = 1.0f / 101325.0f;
//   // return 44330.0f * (1.0f - powf((pressure) / (seaLevelhPa*100.0f), 0.1903f));
//   return 44330.0f * (1.0f - powf(pressure * inv_seaLevelPa, 0.1903f));
// }

// Same as mean sea level (MSL) altitude
// Altitude from pressure:
// https://www.mide.com/air-pressure-at-altitude-calculator
//
// Returns meters above sealevel
float pressure_altitude(const float p) {
  // constexpr float Tb = 15; // temperature at sea level [C] - doesn't work
  // constexpr float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... diff exp for pow??
  constexpr float Tb  = 288.15f;           // temperature at sea level [K]
  constexpr float Lb  = -0.0065f;          // lapse rate [K/m]
  constexpr float R   = 8.31446261815324f; // universal gas const [Nm/(mol K)]
  constexpr float M   = 0.0289644f; // molar mass of Earth's air [kg/mol]
  constexpr float g0  = 9.80665f;   // gravitational const [m/s^2]
  constexpr float Pb  = 101325.0f;         // pressure at sea level [Pa]

  constexpr float exp = -R * Lb / (g0 * M);
  constexpr float scale  = Tb / Lb;
  const float inv_Pb = 1.0f / Pb;

  return scale * (powf(p * inv_Pb, exp) - 1.0f);
}


// static
// void average(LSM6DSOX::lsm6dsox_t ave, LSM6DSOX::lsm6dsox_t d, uint8_t window, uint8_t cnt) {
//   for (size_t i=0; i<3; ++i) {
//     ave.a[i] += d.a[i];
//     ave.g[i] += d.g[i];
//   }
//   const float win = 1.0f / float(window);
//   if (cnt % window == 0) {
//     for (size_t i=0; i<3; ++i) {
//       ave.a[i] *= win;
//       ave.g[i] *= win;
//     }
//   }
// }


// First-order low-pass filter structure
struct lpf_t {
  float alpha{0.0f};  // Smoothing factor
  float y_prev{0.0f}; // Previous output

  lpf_t(float cutoffFrequency, float samplingFrequency) {
    float tau = 1.0f / (2.0f * 3.141592653589793f * cutoffFrequency); // Time constant
    alpha = 1.0f / (1.0f + tau * samplingFrequency);
    y_prev = 0.0f;
  }

  // Apply the low-pass filter to a new input sample
  float filter(float input) {
    // Update the filter
    float output = alpha * input + (1.0f - alpha) * y_prev;

    // Save the current output for the next iteration
    y_prev = output;

    return output;
  }
};