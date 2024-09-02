
#pragma once

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <cstdint> // int types

#if !defined(i2c_default)
  #warning "// i2c not enabled //"
#endif

// constexpr uint I2C0_SDA_PIN = 4;
// constexpr uint I2C0_SCL_PIN = 5;
// constexpr uint I2C1_SDA_PIN = 14;
// constexpr uint I2C1_SCL_PIN = 15;
constexpr uint32_t I2C_100KHZ = 100 * 1000UL;
constexpr uint32_t I2C_400KHZ = 400 * 1000UL;
// constexpr bool I2C_HOLD_BUS    = true;
// constexpr bool I2C_RELEASE_BUS = false;


// determine if the pins passed in are valid for I2C
constexpr uint32_t sda_valid[2] = { // i2c0, i2c1
    (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) | (1 << 16) | (1 << 20) |
        (1 << 24) | (1 << 28),
    (1 << 2) | (1 << 6) | (1 << 10) | (1 << 14) | (1 << 18) | (1 << 22) |
        (1 << 26)};
constexpr uint32_t scl_valid[2] = { // i2c0, i2c1
    (1 << 1) | (1 << 5) | (1 << 9) | (1 << 13) | (1 << 17) | (1 << 21) |
        (1 << 25) | (1 << 29),
    (1 << 3) | (1 << 7) | (1 << 11) | (1 << 15) | (1 << 19) | (1 << 23) |
        (1 << 27)};


enum ErrorI2C:uint32_t {
  NONE = 0,
  INVALID_SCL_PIN,
  INVALID_SDA_PIN,
  INVALID_PORT,
  ALREADY_INITIALIZED,
  UNKNOWN
};

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

  uint32_t init(const uint32_t port, const uint baud, const uint32_t pin_sda,
            const uint32_t pin_scl) {
    if (port != 0 && port != 1) return INVALID_PORT;
    if (!(sda_valid[port] & (1 << pin_sda))) return INVALID_SDA_PIN;
    if (!(scl_valid[port] & (1 << pin_scl))) return INVALID_SCL_PIN;

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);
    // Make the I2C pins available to picotool
    // THIS  WON'T WORK, cannot use variables, only constants
    // bi_decl(bi_2pins_with_func(pin_sda, pin_scl, GPIO_FUNC_I2C));

    if (port == 0) return i2c_init(&i2c0_inst, baud);
    else if (port == 1) return i2c_init(&i2c1_inst, baud);

    return UNKNOWN;
  }
};