
#pragma once

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <cstdint> // int types

#if !defined(i2c_default)
  #warning "// i2c not enabled //"
#endif

constexpr uint I2C0_SDA_PIN = 4;
constexpr uint I2C0_SCL_PIN = 5;
constexpr uint I2C1_SDA_PIN = 14;
constexpr uint I2C1_SCL_PIN = 15;
constexpr uint I2C_100KHZ = 100 * 1000UL;
constexpr uint I2C_400KHZ = 400 * 1000UL;
// constexpr bool I2C_HOLD_BUS    = true;
// constexpr bool I2C_RELEASE_BUS = false;

/*
This is NOT meant to replicate Arduino's TwoWire/Wire library
but do the bare minimum to setup I2C.

If you use both i2c0 and i2c1, this class can initialize both
of those with one instance if you want since it doesn't have
any member variables.

TwoWire tw;
tw.init(0, 400000, 8, 9);
*/
class TwoWire {
public:
  TwoWire() {}
  ~TwoWire() {}

  uint init(const uint port, const uint baud, const uint pin_sda,
            const uint pin_scl) {

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);

    if (port == 0) return i2c_init(&i2c0_inst, baud);
    if (port == 1) return i2c_init(&i2c1_inst, baud);

    return 0;
  }
};