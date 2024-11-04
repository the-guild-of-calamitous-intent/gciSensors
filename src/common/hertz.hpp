/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h>
#include "common/time.hpp"

#if defined(PICO_BOARD)
// Simple class for calculating the hertz rate of a loop
// when profiling code
class Hertz {
public:
  Hertz(uint32_t v = 300) : threshold(v) {
    epoch = now_ms();
  }

  // moved to common/time.hpp
  // uint32_t now_ms() {
  //   absolute_time_t t = get_absolute_time();
  //   return to_ms_since_boot(t);
  // }

  bool check() {
    if (++count % threshold == 0) {
      uint32_t now = now_ms();
      hertz        = 1000.0f * static_cast<float>(count) / static_cast<float>(now - epoch);
      epoch        = now;
      count        = 0;
      return true;
    }

    return false;
  }

  float hertz{0.0f};

protected:
  uint32_t epoch;
  uint32_t count{0};
  const uint32_t threshold;
};

#endif