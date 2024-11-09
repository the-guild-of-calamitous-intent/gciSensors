/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <cmath> // isinf
#include <cstdint>
// #include <stdio.h>  // FIXME: remove
#include <string.h> // memcpy

// namespace gci {
// namespace sensors {

struct bus_t {
  uint32_t port{0};
  // uint32_t DI{0},DO{0},CLK{0},CS{0};
  uint32_t spi_cs{0};
  uint8_t i2c_addr{0};
};


// } // namespace sensors
// } // namespace gci
#include "common/time.hpp"
#include "common/quaternions.hpp"
#include "common/vectors.hpp"
#include "common/messages.hpp"
#include "common/algorithms.hpp"
#include "common/hertz.hpp"

// namespace gci {
// namespace sensors {

inline uint32_t to_24b(uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
}

inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

// } // namespace sensors
// } // namespace gci

// i2c
#include "bmp390.hpp"
#include "dps310.hpp"
#include "lis3mdl.hpp"
#include "lsm6dsox.hpp"
#include "pa1010d.hpp"
// spi
#include "lps22hb.hpp"
