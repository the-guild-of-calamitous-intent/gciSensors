/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

/*
I don't like some of this ... need to clean it up!
*/
class Sensor {
public:
  Sensor(TwoWire *tw, const uint8_t address) : addr(address), i2c(tw) {}

  /*
  reg - the register we want to change
  bits - how many bits for mask
  shift - how much to shift data by
  data - returned value pointer
  */
  bool Read(const uint8_t reg, const uint8_t bits, const uint8_t shift,
            uint8_t *const data) {
    uint8_t val;
    if (!ReadRegisters(reg, 1, &val)) {
      return false;
    }
    val >>= shift;
    uint8_t mask_ = (1 << (bits)) - 1;
    *data = val & mask_;
    return true;
  }

  /*
  Given some data, this will:
  1. read the register to get all the bits
  2. mask out the we don't want to change to protect them
  3. only change the correct bits
  4. write the final value back to the register

  reg - the register we want to change
  data - data that goes into register
  bits - how many bits for mask
  shift - how much to shift data by
  */
  // bool updateCtrlReg()
  bool Write(const uint8_t reg, const uint8_t data, const uint8_t bits,
             const uint8_t shift) {
    uint8_t val;
    if (!ReadRegisters(reg, 1, &val)) {
      return false;
    }
    uint8_t mask = (1 << (bits)) - 1;
    uint8_t d = data & mask;
    mask <<= shift;
    val &= ~mask;
    val |= d << shift;
    return WriteRegister(reg, val);
  }

  /*!
   * @details sets register and verifies it was correct
   *
   * @param[in] reg : starting register adress
   * @param[in] data : returned data pointer
   *
   * @return true (success) or false (fail)
   * @retval false fail
   * @retval true success
   */
  bool WriteRegister(const uint8_t reg, const uint8_t data) {
    uint8_t ret_val;
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->write(data);
    i2c->endTransmission();

    delay(10);
    ReadRegisters(reg, sizeof(ret_val), &ret_val);
    if (data == ret_val) {
      return true;
    } else {
      return false;
    }
  }

  /*!
   * @details Reads the number of bytes starting at address of register
   *
   * @param[in] reg : starting register adress
   * @param[in] count : number of bytes to read
   * @param[in] data : returned data pointer
   *
   * @return true (success) or false (fail)
   * @retval false fail
   * @retval true success
   */
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->endTransmission(false);
    uint8_t bytes_rx_ = i2c->requestFrom(static_cast<uint8_t>(addr), count);
    if (bytes_rx_ == count) {
      for (size_t i = 0; i < count; i++) {
        data[i] = i2c->read();
      }
      return true;
    } else
      return false;
  }

  /*
  Returns the register value and returns the entire register.
  */
  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    if (ReadRegisters(reg, sizeof(value), &value) != 1)
      return -1;
    return value;
  }

  // inline bool checkErr(int val) { return (val < 0) ? false : true; }

  TwoWire *i2c;
  uint8_t addr;
};