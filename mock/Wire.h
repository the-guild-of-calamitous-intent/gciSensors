#pragma once

// https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h

// #ifndef TwoWire_h
// #define TwoWire_h

#include <stdint.h>
#include <stddef.h>

class TwoWire {
  public:
  TwoWire() {}

    void    begin(int sda, int scl) {}
    void    begin(int sda, int scl, uint8_t address) {}

    void    setClock(uint32_t) {}
    void    setClockStretchLimit(uint32_t) {}
    void    beginTransmission(uint8_t) {}
    void    beginTransmission(int) {}
    uint8_t endTransmission(void) {return 0;}
    uint8_t endTransmission(uint8_t) {return 0;}
    size_t  requestFrom(uint8_t address, size_t size, bool sendStop) {return 0;}
    // uint8_t status();

    uint8_t requestFrom(uint8_t, uint8_t) {return 0;}
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t) {return 0;}
    uint8_t requestFrom(int, int) {return 0;}
    uint8_t requestFrom(int, int, int) {return 0;}

  size_t write(uint8_t) {return 0;}
    size_t write(const uint8_t*, size_t) {return 0;}
    uint8_t read(void) {return 0;}
};

extern TwoWire Wire;

// #endif