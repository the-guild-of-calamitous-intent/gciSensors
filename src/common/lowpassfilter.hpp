/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h>
#include <math.h>

namespace sensors {


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


} // end sensors namespace