/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once


#include "sensor.hpp"
#include "pmtk.hpp"
#include <string.h>

namespace PA1010D {

inline
bool ascii_nema(const char c) {
  return (c >= '0' && c <= '9') || (c>='A' && c<='Z') || c == '*' || c == ',';
}

constexpr uint8_t PA_ADDR   = 0x10;
constexpr uint32_t I2C_BUFFER_SIZE = 32;
constexpr uint32_t MAX_NEMA_SIZE = 82;


class gciPA1010D : public SensorI2C {
  uint8_t buffer[I2C_BUFFER_SIZE]{0};
  static constexpr uint8_t LOOP_FAIL{5};

public:
  gciPA1010D(const uint32_t port, const uint8_t addr = PA_ADDR)
      : SensorI2C(addr, port) {}

  // inline
  // int write(char command[], uint32_t cmd_size) {
  //   // return i2c_write_blocking(i2c, addr, (uint8_t*)command, cmd_size, false);
  //   return writeRegisters(addr, cmd_size, (uint8_t*)command) ? 0 : 1;
  // }

  inline
  int write(const uint8_t* command, uint32_t cmd_size) {
    // return i2c_write_blocking(i2c, addr, command, cmd_size, false);
    return writeRegisters(addr, cmd_size, (uint8_t*)command) ? 0 : 1;
  }

  // Get message from GPS and return the message string
  // with '\0' appended to end
  uint32_t read(char buff[], const uint32_t buff_size) {
    uint32_t i = 10000;
    uint32_t iw = 0;
    // uint32_t start = 0;
    // uint32_t end = 0;
    uint8_t loop_fail = 0;
    char c = 0;

    while (c != '$') {
      if (i >= I2C_BUFFER_SIZE) {
        // if we do this too many time fail
        if (loop_fail++ >= LOOP_FAIL) return 0;
        i = 0;
        memset(buffer, 0, I2C_BUFFER_SIZE);
        i2c_read_blocking(i2c, addr, buffer, I2C_BUFFER_SIZE, false);
      }
      c = buffer[i++];
    }
    buff[iw++] = c; // save
    loop_fail = 0;

    // find end char '\r' and '\n'
    while (iw < buff_size-1) {
      if (i >= I2C_BUFFER_SIZE) {
        // if we do this too many time fail
        if (loop_fail++ >= LOOP_FAIL) return 0;
        i = 0;
        memset(buffer, 0, I2C_BUFFER_SIZE);
        i2c_read_blocking(i2c, addr, buffer, I2C_BUFFER_SIZE, false);
      }
      c = buffer[i++];

      if (c == '$') return 0; // BAD
      else if (c == '\r') { // end chars
        buff[iw++] = c;
        c = buffer[i++];
        if (c != '\n') {
          return 0;
        }
        buff[iw++] = c;
        buff[iw] = '\0';
        return iw;
      }
      else if (ascii_nema(c)) buff[iw++] = c;
    }
    return 0; // how get here?
  }
};

} // end namespace