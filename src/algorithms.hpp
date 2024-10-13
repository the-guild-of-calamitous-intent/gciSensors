#pragma once

#include <stdint.h>
// #include "common.hpp"

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


// // First-order low-pass filter structure
// struct LowPassFilter {
//   float alpha;  // Smoothing factor
//   float y_prev; // Previous output
// };

// // Initialize the low-pass filter
// static
// void initLowPassFilter(LowPassFilter *filter, float cutoffFrequency, float samplingFrequency) {
//   float tau = 1.0f / (2.0f * 3.141592653589793f * cutoffFrequency); // Time constant
//   filter->alpha = 1.0f / (1.0f + tau * samplingFrequency);
//   filter->y_prev = 0.0f;
// }

// // Apply the low-pass filter to a new input sample
// static
// float filterSample(LowPassFilter *filter, float input) {
//   // Update the filter
//   float output = filter->alpha * input + (1.0f - filter->alpha) * filter->y_prev;

//   // Save the current output for the next iteration
//   filter->y_prev = output;

//   return output;
// }