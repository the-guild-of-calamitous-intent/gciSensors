/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

// #include <Wire.hpp>
#include <stdint.h> // int types
// #include "i2c.hpp"

#include "hardware/i2c.h"

constexpr uint I2C0_SDA_PIN = 8;
constexpr uint I2C0_SCL_PIN = 9;
constexpr uint I2C1_SDA_PIN = 14;
constexpr uint I2C1_SCL_PIN = 15;
// constexpr uint I2C_100KHZ = 100 * 1000;
// constexpr uint I2C_400KHZ = 400 * 1000;
constexpr bool I2C_HOLD_BUS = true;
constexpr bool I2C_RELEASE_BUS = false;

class TwoWire {};

class SensorI2C {
  const uint8_t addr;
  i2c_inst_t* i2c;
  static bool initialized0;
  static bool initialized1;

public:
  SensorI2C(uint8_t addr): addr(addr) {}
  ~SensorI2C() { i2c_deinit(i2c); }

  void init_tw(uint baud, uint8_t port, uint8_t pin_sda, uint8_t pin_scl) {
    if (port == 0) {
      i2c = i2c0;
      if (initialized0 == false) i2c_init(i2c, baud);
      initialized0 = true;
    }
    else if (port == 1) {
      i2c = i2c1;
      if (initialized1 == false) i2c_init(i2c, baud);
      initialized1 = true;
    }

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);
  }

  bool writeRegister(const uint8_t reg, const uint8_t data) {
    uint8_t out[2]{reg, data};
    i2c_write_blocking(i2c, addr, out, 2, I2C_RELEASE_BUS);
    return true;
    // uint8_t ret_val;
    // i2c->beginTransmission(addr);
    // i2c->write(reg);
    // i2c->write(data);
    // i2c->endTransmission();

    // delay(10);
    // readRegisters(reg, 1, &ret_val);
    // if (data == ret_val) return true;

    // // println("data write failed verification: " + String(int(data)) +
    // //         " != " + String(int(ret_val)));

    // return false;
  }

  bool readRegisters(const uint8_t reg, const uint8_t data_size,
                     uint8_t *const data) {
    i2c_write_blocking(i2c, addr, &reg, 1, I2C_HOLD_BUS);
    int ret = i2c_read_blocking(i2c, addr, data, data_size, I2C_RELEASE_BUS);
    if (ret < 0) return false;
    return true;
    // i2c->beginTransmission(addr);
    // i2c->write(reg);
    // i2c->endTransmission(false);

    // // delay(500);
    // // delay(2);

    // uint8_t bytes_rx = i2c->requestFrom(addr, count);
    // if (bytes_rx == count) {
    //   for (uint8_t i = 0; i < count; i++) {
    //     data[i] = i2c->read();
    //   }
    //   return true;
    // }

    // // println("ReadRegisters::bad read: " + String(int(bytes_rx)) +
    // //         " expected: " + String(int(count)));

    // return false;
  }

  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    if (!readRegisters(reg, 1, &value)) return 0;
    return value;
  }

  inline
  size_t available() { return i2c_get_read_available(i2c); }

  // const uint8_t addr;

// protected:
//   TwoWire *i2c;
};

bool SensorI2C::initialized0 = false;
bool SensorI2C::initialized1 = false;