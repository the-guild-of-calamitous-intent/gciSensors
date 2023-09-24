/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once
#include "gciSensors.hpp"

// Calibration
// https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

// Equations pulled from:
// https://ahrs.readthedocs.io/en/latest/filters/complementary.html

namespace gcisensors {

template<typename T>
struct AttitudeA {
  const vec_t<T> update(gcisensors::vecf_t a, bool degrees=true) {
    vec_t<T> ret{0};
    if (!a.normalize()) return ret;
    ret.x  = atan2(a.y, a.z);
    ret.y = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z));
    ret.z = 0.0;

    if (degrees) {
      ret.x *= 180.0 / M_PI;
      ret.y *= 180.0 / M_PI;
    }

    return ret;
  }

  const vec_t<T> update(gcisensors::vecf_t a, gcisensors::vecf_t m, bool degrees=true) {
    vec_t<T> ret{0};
    if (!a.normalize()) return ret;
    ret.x  = atan2(a.y, a.z);
    ret.piytch = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z));

    if (!m.normalize()) return ret;

    // WARNING: ahrs.readthedocs switches symbols for roll/pitch compared
    //          to other authors ... below is correct
    T cr = cos(ret.x); // roll
    T sr = sin(ret.x);
    T cp = cos(ret.y); // pitch
    T sp = sin(ret.y);
    ret.z = atan2(
      m.z*sp - m.y*cp,
      m.x*cr + sr * (m.y * sp + m.z * cp)
    ); // yaw

    if (degrees) {
      ret.x *= 180.0 / M_PI;
      ret.y *= 180.0 / M_PI;
      ret.z *= 180.0 / M_PI;
    }

    return ret;
  }
};

template<typename T>
struct AttitudeAG {
  AttitudeAG(const T alpha): alpha(alpha) {}

  // a: G's, this will be normalized so units aren't important
  // w: dps
  // dt: seconds
  // return: roll, pitch, yaw in degrees
  const vec_t<T> update(vecf_t a, vecf_t w, const T dt) {
    vec_t<T> ret{0};

    vec_t<T> aa;
    if (!a.normalize()) return ret;

    aa.x  = atan2(a.y, a.z) * 180.0 / M_PI; // deg
    aa.y = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z)) * 180.0 / M_PI;
    aa.z = 0.0;

    vec_t<T> ww;
    ww.x = euler.x + w.x * dt; // deg = deg + dps * sec
    ww.y = euler.y + w.y * dt;
    ww.z = euler.z + w.z * dt;

    euler.x = alpha*ww.x + (1.0 - alpha) * aa.x;
    euler.y = alpha*ww.y + (1.0 - alpha) * aa.y;
    // ret.z = alpha*ww.z + (1.0 - alpha) * aa.z; // should this only be ww.z?
    euler.z = ww.z;

    return euler;
  }

  vec_t<T> euler{0.0,0.0,0.0}; // degrees
  T alpha; // [0 - 1]
};


template<typename T>
struct AttitudeAM {
  const vec_t<T> update(gcisensors::vecf_t a, gcisensors::vecf_t m, bool degrees=true) {
    vec_t<T> ret{0};
    if (!a.normalize()) return ret;
    ret.x  = atan2(a.y, a.z);
    ret.piytch = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z));

    if (!m.normalize()) return ret;

    // WARNING: ahrs.readthedocs switches symbols for roll/pitch compared
    //          to other authors ... below is correct
    T cr = cos(ret.x); // roll
    T sr = sin(ret.x);
    T cp = cos(ret.y); // pitch
    T sp = sin(ret.y);
    ret.z = atan2(
      m.z*sp - m.y*cp,
      m.x*cr + sr * (m.y * sp + m.z * cp)
    ); // yaw

    if (degrees) {
      ret.x *= 180.0 / M_PI;
      ret.y *= 180.0 / M_PI;
      ret.z *= 180.0 / M_PI;
    }

    return ret;
  }
};

} // end namespace

/////////////////////////////////////////////////////////////////////////////
// Useful?

/*
Calculates the time change since it was last called in seconds.
*/
// class DT {
// public:
//   DT() : last(millis()) {}

//   void touch() { last = millis(); }

//   float now() {
//     uint32_t n = millis();
//     float dt   = static_cast<float>(n - last) * 0.001f;
//     last       = n;
//     return dt;
//   }

// protected:
//   uint32_t last;
// };

/*
https://en.wikipedia.org/wiki/Low-pass_filter
*/
// class LowPass {
// public:
//   LowPass(float cut) : cutoff(cut), val(0.0f) {}

//   float update(float in, const float dt) {
//     float b = 2.0f * static_cast<float>(M_PI) * cutoff * dt;
//     float a = b / (1.0f + b);
//     val     = val + a * (in - val);
//     return val;
//   }

// protected:
//   float cutoff;
//   float val; // current value
//   // DT dt;
// };

/*
http://en.wikipedia.org/wiki/High-pass_filter
*/
// class HighPass {
// public:
//   HighPass(float cut) : cutoff(cut), val(0.0f), last(0.0f) {}

//   float update(const float in, const float dt) {
//     float b = 2.0f * static_cast<float>(M_PI) * cutoff * dt;
//     float a = b / (1.0f + b);
//     val     = a * (val + in - last);
//     last    = in;
//     return val;
//   }

// protected:
//   float cutoff;
//   float val;  // current value
//   float last; // previous value
//   // DT dt;
// };

// struct rpy_t {
//   float r, p, y;
// };