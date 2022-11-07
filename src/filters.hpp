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

#if 1
#include "squaternion.hpp"
/*!
 */
class QCF {
public:
  QCF(float a = 0.02f) : alpha(a) {}

  Quaternion update(float ax, float ay, float az, float wx, float wy, float wz,
                    float dt) {
    qw = q + 0.5 * dt * Quaternion(0.0f, wx, wy, wz);
    float an = 1.0f / sqrtf(ax * ax + ay * ay + az * az);

    if (isinf(an))
      return q;

    ax *= an;
    ay *= an;
    az *= an;

    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f;

    qam = Quaternion::from_euler(roll, pitch, yaw);

    q = (1.0f - alpha) * qw + alpha * qam;
    return q;
  }

  Quaternion q; // current state estimate

protected:
  float alpha;    // ratio between the two quaternion estimates
  Quaternion qw;  // quaternion from gyros
  Quaternion qam; // quaternion from accels
};

#endif

////////////////////////////////////////////////////////////

// class ComplementaryFilter {
//     public:
//     ComplementaryFilter(float alpha=0.02f, float ang=0.0f): a(alpha),
//     angle(ang) {}

//     float update(float accel, float gyro, float dt) {
//         // float y0 = y[0], y1 = y[1];
//         // y0 = (1.0f-a)*y
//         angle = (1.0f - a) * (angle + gyro * dt) + a * accel;
//         // angle = angle + gyro * dt;
//         return angle;
//     }

//     protected:
//     const float a;
//     float angle;
// };

// class ComplementaryFilter {
// public:
//   ComplementaryFilter(float alpha = 0.02f) : a(alpha) {}

//   rpy_t rpy(float ax, float ay, float az) {
//     float amag = 1.0f / sqrtf(ax * ax + ay * ay + az * az);

//     if (isinf(amag)) rpy_t{0.0,0.0,0.0};

//     ax *= amag;
//     ay *= amag;

//     rpy_t ret;
//     ret.y = 0.0f;
//     ret.p = asinf(-ax);

//     if (fabs(ret.p) < M_PI / 2.0f) {
//       ret.r = asinf((ay) / cosf(ret.p));
//     } else
//       ret.r = 0.0f;

//     return ret;
//   }

//   rpy_t update(gci::sox_t &s) {
// #if 0
//         float dt = delta.now();
//         float ax=s.ax, ay=s.ay, az=s.az - 1.0f;
//         float gx=s.gx, gy=s.gy, gz=s.gz;

//         float amag = sqrtf(ax*ax + ay*ay + az*az);

//         r = (1.0f - a) * (r + gx * dt) + a * ax / amag;
//         p = (1.0f - a) * (p + gy * dt) + a * ay / amag;
//         y = (1.0f - a) * (y + gz * dt) + a * az / amag;
//         // angle = angle + gyro * dt;

//         rpy_t rpy{r,p,y};

//         return rpy;

// #else

//     float dt = delta.now();
//     float ax = s.ax, ay = s.ay, az = s.az - 1.0f;
//     float gx = s.gx, gy = s.gy, gz = s.gz;

//     // float amag = sqrtf(ax*ax + ay*ay + az*az);
//     rpy_t rr = rpy(ax, ay, az);
//     // static float rad2deg = 180.0f / M_PI;

//     r = (1.0f - a) * (r + gx * dt) + a * rr.r * Units::rad2deg;
//     p = (1.0f - a) * (p + gy * dt) + a * rr.p * Units::rad2deg;
//     y = (1.0f - a) * (y + gz * dt) + a * rr.y * Units::rad2deg;
//     // angle = angle + gyro * dt;

//     rpy_t rpy{r, p, y};

//     return rpy;

// #endif
//   }

// protected:
//   const float a;
//   float r, p, y;
//   DT delta;
// };
