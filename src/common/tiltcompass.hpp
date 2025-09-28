/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "sensor.hpp"

struct tiltcompass_t {
  float roll, pitch, yaw;
} __attribute__((packed));

tiltcompass_t TiltCompass(const sensors::vec_t &accel, const sensors::vec_t &mag) {
  sensors::vec_t a = accel;
  a.normalize();

  sensors::vec_t m = mag;
  m.normalize();

  
  float pitch = asinf(a.x);

  float sp = sinf(pitch);
  float cp = cosf(pitch);

  float roll = asinf(-a.y / cp);

  float sr = sinf(roll);
  float cr = cosf(roll);

  float yaw = atan2f(
    m.z * sr - m.y * cr,
    m.x * cp + m.y * sp * sr + m.z * sp * cr);
          
  return {roll, pitch, yaw};
}
