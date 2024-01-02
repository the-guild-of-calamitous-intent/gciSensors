/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <cstdint> // int types

inline uint32_t to_24b(uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
}

inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

#if defined(PICO_SDK)
  #include "sensor_pico.hpp"
#elif defined(ARDUINO)
  #include "sensor_arduino.hpp"
#elif defined(__linux__)
  #include "sensor_linux.hpp"
#elif defined(__APPLE__)
  #include "sensor_apple.hpp"
#else
  #warning "Unknown Platform ... didn't include SensorI2C"
#endif