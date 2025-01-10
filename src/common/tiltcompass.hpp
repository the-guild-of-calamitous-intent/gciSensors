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
  float roll = asinf(-a.y / cosf(pitch));
  float yaw = atan2f(m.z * sinf(roll) - m.y * cosf(roll),
          m.x * cosf(pitch) + m.y * sinf(pitch) * sinf(roll)
          + m.z * sinf(pitch) * cosf(roll));
          
  return {roll, pitch, yaw};
}
