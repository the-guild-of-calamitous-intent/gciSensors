/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "sensor/sensor.hpp"
// #include "hardware/i2c.h"
// #include "pico/binary_info.h"
// #include "pico/stdlib.h"
// #include "string.h"
#include <cstdio>
#include <cstring>

namespace PA1010D {

constexpr uint8_t PA_ADDR   = 0x10;
constexpr uint32_t MAX_READ = 250;

class gciPA1010D : public SensorI2C {
  // const int addr = 0x10;
  // const uint32_t MAX_READ{250};
  uint8_t buffer[MAX_READ]{0};

public:
  gciPA1010D(const uint8_t addr = PA_ADDR, uint32_t port = 0)
      : SensorI2C(addr, port) {}

  void write(const char command[], int com_length) {
    // Convert character array to bytes for writing
    uint8_t int_command[com_length];

    for (int i = 0; i < com_length; ++i) {
      int_command[i] = command[i];
      i2c_write_blocking(i2c_default, addr, &int_command[i], 1, true);
    }
  }

  uint32_t read(char numcommand[]) {

    uint32_t i    = 0;
    bool complete = false;

    i2c_read_blocking(i2c_default, addr, buffer, MAX_READ, false);

    // Convert bytes to characters
    while (i < (MAX_READ - 1) && complete == false) {
      numcommand[i] = buffer[i];
      // Stop converting at end of message
      if (buffer[i] == 10 && buffer[i + 1] == 10) {
        complete = true;
      }
      i++;
    }

    return i;
  }

};

} // end namespace