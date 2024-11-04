/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <math.h>

// namespace gci {
namespace sensors {

///////////////////////////////////////////////////////////
// Orientation
///////////////////////////////////////////////////////////

constexpr float deg2rad = static_cast<float>(M_PI) / 180.0f;
constexpr float rad2deg = 180.0f / static_cast<float>(M_PI);

struct quat_t {
  quat_t(): w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
  quat_t(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {}
  float w, x, y, z;

  inline float magnitude() const { return sqrtf(x * x + y * y + z * z + w * w); }

  bool normalize() {
    float n = 1.0f / magnitude();
    if (std::isinf(n)) return false;

    w *= n;
    x *= n;
    y *= n;
    z *= n;

    return true;
  }

  float& operator[](size_t i) {
    return (i == 0) ? w : (i == 1) ? x : (i == 2) ? y : z;
  }

  // anwer = q(this) * r
  quat_t operator*(const quat_t &r) const {
    const quat_t *q = this;
    return quat_t(
      r.w * q->w - r.x * q->x - r.y * q->y - r.z * q->z,
      r.w * q->x + r.x * q->w - r.y * q->z + r.z * q->y,
      r.w * q->y + r.x * q->z + r.y * q->w - r.z * q->x,
      r.w * q->z - r.x * q->y + r.y * q->x + r.z * q->w);
  }
  // answer = q(this) / scalar
  quat_t operator/(float scalar) const {
    return quat_t(w / scalar, x / scalar, y / scalar, z / scalar);
  }

  // answer = q(this) * r
  quat_t operator*(float scalar) const {
    return quat_t(w * scalar, x * scalar, y * scalar, z * scalar);
  }

  // answer = q(this) + r
  quat_t operator+(const quat_t &r) const {
    return quat_t(w + r.w, x + r.x, y + r.y, z + r.z);
  }

  // answer = q(this) - r
  quat_t operator-(const quat_t &r) const {
    return quat_t(w - r.w, x - r.x, y - r.y, z - r.z);
  }

  // Returns the Euler angles as a tuple(roll, pitch, yaw)
  // This is a modified version of this:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  void to_euler(float *roll, float *pitch, float *yaw,
                const bool degrees = false) const {

    float ysqr = y * y;

    float t0   = +2.0f * (w * x + y * z);
    float t1   = +1.0f - 2.0f * (x * x + ysqr);
    *roll       = atan2f(t0, t1);

    float t2   = +2.0f * (w * y - z * x);

    if (t2 > 1.0f) t2 = 1.0f;
    else if (t2 < -1.0f) t2 = -1.0f;
    *pitch    = asinf(t2);

    float t3 = +2.0f * (w * z + x * y);
    float t4 = +1.0f - 2.0f * (ysqr + z * z);
    *yaw      = atan2f(t3, t4);

    if (degrees) {
      *roll *= rad2deg;
      *pitch *= rad2deg;
      *yaw *= rad2deg;
    }
  }

  static
  quat_t from_euler(float roll, float pitch, float yaw, bool degrees = false) {
    // Euler angles euler2quat(roll, pitch, yaw, degrees=False), default is
    // radians, but set degrees True if giving degrees This is a modified
    // version of this:
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    if (degrees) {
      roll *= deg2rad;
      pitch *= deg2rad;
      yaw *= deg2rad;
    }

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    float w  = cy * cr * cp + sy * sr * sp;
    float x  = cy * sr * cp - sy * cr * sp;
    float y  = cy * cr * sp + sy * sr * cp;
    float z  = sy * cr * cp - cy * sr * sp;

    return quat_t(w, x, y, z);
  }

};

// answer = scalar * q(this)
// quat_t operator*(double scalar, const quat_t &q) {
//   return quat_t(q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar);
// }
quat_t operator*(float scalar, const quat_t &q) {
  return quat_t(q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar);
}

} // namespace sensors
// } // namespace gci