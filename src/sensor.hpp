/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

// #if defined(__linux__)
// not sure what to do
#if defined(ARDUINO)
  #include <Arduino.h>
  #include <Wire.h>
// #elif defined(__APPLE__)
//   #include <mock_arduino.hpp>
//   #include <mock_wire.hpp>
// #elif defined(linux)
#endif

// THIS DOESN'T SEEM TO WORK RIGHT
// Put DEBUG in main.cpp before importing this header
// Example: #define DEBUG 1
#if defined(GCI_SENSORS_DEBUG)
  #if defined(ARDUINO)
    static void println(const String &s) { Serial.println(s); }
    static void print(const String &s) { Serial.print(s); }
  #else // apple linux
    #if defined(__APPLE__) || defined(linux)
      typedef std::string String;
    #endif
      static void println(const String &s) {}
      static void print(const String &s) {}
  #endif
#else
  #if defined(__APPLE__) || defined(linux)
    typedef std::string String;
  #endif
  static void println(const String &s) {}
  static void print(const String &s) {}
#endif

#include <stdint.h> // int types

inline uint32_t to_24b(uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
}
inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

/*
I don't like some of this ... need to clean it up!
move to camel case and
*/
class SensorI2C {
public:
  SensorI2C(TwoWire *tw, const uint8_t address) : addr(address), i2c(tw) {}

  bool writeRegister(const uint8_t reg, const uint8_t data); // FIXME

  bool readRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data);
  // bool readRegister(const uint8_t reg, uint8_t *const data); // do this
  // instead?
  uint8_t readRegister(uint8_t reg);

  // inline bool checkErr(int val) { return (val < 0) ? false : true; }

  TwoWire *i2c;
  uint8_t addr;
};
