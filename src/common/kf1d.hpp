/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <math.h>
#include "common/vectors.hpp"

namespace gs = sensors;

/*
1D Kalman Filter
*/
struct kf1d_t {
  float P{1.0f};  // Initial estimate error
  float F{1.0f};  // Assuming constant velocity
  float x{1.0f};  // state estimate
  float Q;       // processes noise
  float R;       // measurement noise

  kf1d_t(float q, float r): Q(q), R(r) {}

  float filter(float z) {
    // Prediction
    float xx = x * F;
    float P = P + Q;

    // Update
    float K = P / (P + R);
    x = xx + K * (z - xx);
    P = (1.0f - K) * P;

    return x;
  }
};


/*
1D Kalman Filter * 3, these are all independent 1D filters
*/
struct kf1d3_t {
  gs::vec_t P{1.0f,1.0f,1.0f};  // Initial estimate error
  gs::vec_t F{1.0f,1.0f,1.0f};  // Assuming constant velocity
  gs::vec_t x{1.0f,1.0f,1.0f};  // state estimate
  gs::vec_t Q;       // processes noise
  gs::vec_t R;       // measurement noise

  kf1d3_t(gs::vec_t q, gs::vec_t r): Q(q), R(r) {}

  const gs::vec_t filter(gs::vec_t z) {
    // Prediction
    gs::vec_t xx;
    xx.x = x.x * F.x;
    xx.y = x.y * F.y;
    xx.z = x.z * F.z;

    gs::vec_t P;
    P.x = P.x + Q.x;
    P.y = P.y + Q.y;
    P.z = P.z + Q.z;

    // Update
    gs::vec_t K;
    K.x = P.x / (P.x + R.x);
    K.y = P.y / (P.y + R.y);
    K.z = P.z / (P.z + R.z);

    x.x = xx.x + K.x * (z.x - xx.x);
    x.y = xx.y + K.y * (z.y - xx.y);
    x.z = xx.z + K.z * (z.z - xx.z);

    P.x = (1.0f - K.x) * P.x;
    P.y = (1.0f - K.y) * P.y;
    P.z = (1.0f - K.z) * P.z;

    return x;
  }
};