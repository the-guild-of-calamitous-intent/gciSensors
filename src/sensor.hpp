/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h> // int types

/*
I don't like some of this ... need to clean it up!
move to camel case and
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
            uint8_t *const data) ;
  bool readBits(const uint8_t reg, const uint8_t bits, const uint8_t shift,
            uint8_t *const data) {return Read(reg,bits,shift,data);} // FIXME

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
             const uint8_t shift);
  bool writeBits(const uint8_t reg, const uint8_t data, const uint8_t bits,
             const uint8_t shift) {return Write(reg,data,bits,shift);} // FIXME

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
  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool writeRegister(const uint8_t reg, const uint8_t data){return WriteRegister(reg, data);} // FIXME

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
                     uint8_t *const data);
  bool readRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {return ReadRegisters(reg, count, data);}

  /*
  Returns the register value and returns the entire register.
  */
  uint8_t readRegister(uint8_t reg);
  bool readRegister(uint8_t reg, uint8_t* data) {return true;} // do this instead?

  // inline bool checkErr(int val) { return (val < 0) ? false : true; }

  TwoWire *i2c;
  uint8_t addr;
};