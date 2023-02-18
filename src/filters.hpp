/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

/********************************************
Attitude from gravity: https://ahrs.readthedocs.io/en/latest/filters/tilt.html
Quaternion complementary filter:
https://ahrs.readthedocs.io/en/latest/filters/complementary.html
*********************************************/

#include "lsm6dsox/lsm6dsox.hpp"
#include "timers.hpp"
#include "units.hpp"

/*
https://en.wikipedia.org/wiki/Low-pass_filter
*/
class LowPass {
public:
  LowPass(float cut) : cutoff(cut), val(0.0f) {}

  float update(float in) {
    float b = 2.0f * static_cast<float>(M_PI) * cutoff * dt.now();
    float a = b / (1.0f + b);
    val = val + a * (in - val);
    return val;
  }

protected:
  float cutoff;
  float val; // current value
  DT dt;
};

/*
http://en.wikipedia.org/wiki/High-pass_filter
*/
class HighPass {
public:
  HighPass(float cut) : cutoff(cut), val(0.0f), last(0.0f) {}

  float update(const float in) {
    float b = 2.0f * static_cast<float>(M_PI) * cutoff * dt.now();
    float a = b / (1.0f + b);
    val = a * (val + in - last);
    last = in;
    return val;
  }

protected:
  float cutoff;
  float val;  // current value
  float last; // previous value
  DT dt;
};

struct rpy_t {
  float r, p, y;
};