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
#include "hardware/gpio.h"

#if !defined(i2c_default)
  #warning "// i2c not enabled //"
#endif

constexpr uint I2C0_SDA_PIN = 4;
constexpr uint I2C0_SCL_PIN = 5;
constexpr uint I2C1_SDA_PIN = 14;
constexpr uint I2C1_SCL_PIN = 15;
// constexpr uint I2C_100KHZ = 100 * 1000;
// constexpr uint I2C_400KHZ = 400 * 1000;
constexpr bool I2C_HOLD_BUS = true;
constexpr bool I2C_RELEASE_BUS = false;

class TwoWire {
  public:
  TwoWire() {}
  ~TwoWire() {}

  uint init(const uint port, const uint baud, const uint pin_sda, const uint pin_scl) {
    // bi_decl(bi_2pins_with_func(pin_sda, pin_scl, GPIO_FUNC_I2C));
    // bi_decl(bi_program_description("SensorI2C init_tw"));
    // bi_decl(bi_2pins_with_func(4, 5, GPIO_FUNC_I2C));
    // bi_decl(bi_program_description("SensorI2C init_tw"));

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);

    // uint ret;
    if (port == 0) return i2c_init(&i2c0_inst, baud);
    if (port == 1) return i2c_init(&i2c1_inst, baud);

    // printf(">> i2c instance: %u buad: %u\n", port, ret);
    // printf(">> i2c SDA: %u SCL: %u\n", pin_sda, pin_scl);

    return 0;
  }
};

class SensorI2C {
  const uint8_t addr;
  i2c_inst_t* i2c;
  // static bool initialized0;
  // static bool initialized1;

public:
  SensorI2C(uint8_t addr): addr(addr) {}
  ~SensorI2C() { /*i2c_deinit(i2c);*/ }

  bool init_tw(/*const uint baud,*/ const uint port/*, const uint pin_sda, const uint pin_scl*/) {
    uint ret;
    if (port == 0) {
      i2c = &i2c0_inst;
      // if (initialized0 == false) ret = i2c_init(i2c, baud);
      // initialized0 = true;
    }
    else if (port == 1) {
      i2c = &i2c1_inst;
      // if (initialized1 == false) ret = i2c_init(i2c, baud);
      // initialized1 = true;
    }
    else return false;
    // printf(">> i2c instance: %u buad: %u\n", port, ret);
    // printf(">> i2c SDA: %u SCL: %u\n", pin_sda, pin_scl);

    // // bi_decl(bi_2pins_with_func(pin_sda, pin_scl, GPIO_FUNC_I2C));
    // // bi_decl(bi_program_description("SensorI2C init_tw"));
    // // bi_decl(bi_2pins_with_func(4, 5, GPIO_FUNC_I2C));
    // // bi_decl(bi_program_description("SensorI2C init_tw"));

    // gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    // gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    // gpio_pull_up(pin_sda);
    // gpio_pull_up(pin_scl);

    return true;
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

  bool readRegisters(const uint8_t reg, const size_t data_size,
                     uint8_t *const data) {
    i2c_write_blocking(i2c, addr, &reg, 1, I2C_HOLD_BUS);
    int ret = i2c_read_blocking(i2c, addr, data, data_size, I2C_RELEASE_BUS);
    return (ret < 0) ? false : true;

    // if (ret < 0) return false;
    // return true;

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

  // inline
  uint8_t readRegister(const uint8_t reg) {
    uint8_t value;
    // return (readRegisters(reg, 1, &value) == true) ? value : 0;
    if (readRegisters(reg, 1, &value)) return value;
    return 0;
  }

  inline
  size_t available() { return i2c_get_read_available(i2c); }

  // const uint8_t addr;

// protected:
//   TwoWire *i2c;
};

// bool SensorI2C::initialized0 = false;
// bool SensorI2C::initialized1 = false;
