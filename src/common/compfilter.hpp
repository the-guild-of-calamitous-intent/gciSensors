/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/

#pragma once


namespace gci {
namespace sensors {

/*!
Quaternion complementary filter (QCF) blends the sensor readings from an
accelerometer and a gyro together to get a stable reading and more reliable
overall state of a system.

Ref: https://ahrs.readthedocs.io/en/latest/filters/complementary.html
 */
struct compfilter_t {
  // alpha: [0.0. 1.0]
  compfilter_t(float a) : alpha(a) {}
  quat_t q; // current state estimate
  float alpha;   // ratio between the two quaternion estimates

  const quat_t update(vec_t a, vec_t w, vec_t m, float dt) {
    quat_t qw = q + 0.5f * dt * q * quat_t(0.0f, w.x, w.y, w.z);

    if (!a.normalize()) return q;

    float roll  = atan2f(a.y, a.z);
    float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));

    // m = mm;
    if (!m.normalize()) return q;

    // WARNING: ahrs.readthedocs switches symbols for roll/pitch compared
    //          to other authors ... below is correct
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float yaw = atan2f(
      m.z*sp - m.y*cp,
      m.x*cr + sr * (m.y * sp + m.z * cp)
    );

    quat_t qam = quat_t::from_euler(roll, pitch, yaw);

    q = alpha * qw + (1.0f - alpha) * qam;
    return q;
  }

  const quat_t update(vec_t a, const vec_t w, const float dt) {
    quat_t qw = q + 0.5f * dt * q * quat_t(0.0f, w.x, w.y, w.z);

    a.normalize();

    float roll  = atan2f(a.y, a.z);
    float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));
    float yaw{0.0f}; // need magnetometer to calculate this, so default to 0.0

    quat_t qam = quat_t::from_euler(roll, pitch, yaw);

    q = alpha * qw + (1.0f - alpha) * qam; // match ahrs.readthedocs
    return q;
  }
};

/*
Ref: https://ahrs.readthedocs.io/en/latest/filters/tilt.html
*/
struct tilt_compass_t {

  const quat_t update(vec_t a, vec_t m) {
    // a.normalize();
    if (!a.normalize()) return q;
    float roll  = atan2f(a.y, a.z);
    float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));

    if (!m.normalize()) return q;

    // WARNING: ahrs.readthedocs switches symbols for roll/pitch compared
    //          to other authors ... below is correct
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float yaw = atan2f(
      m.z*sp - m.y*cp,
      m.x*cr + sr * (m.y * sp + m.z * cp)
    );

    q = quat_t::from_euler(roll, pitch, yaw);
    return q;
  }

  quat_t q;
};


} // namespace sensors
} // namespace gci