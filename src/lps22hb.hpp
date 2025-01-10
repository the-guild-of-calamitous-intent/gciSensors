/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
/*
References:
- https://www.st.com/resource/en/datasheet/dm00140895.pdf
*/

#pragma once

#include <math.h>
#include "sensor.hpp"

// #if defined(__USE_SENSOR_LPS22HB__)

// 10 MHz max SPI frequency
constexpr uint32_t LPS22_MAX_SPI_CLK_HZ = 10000000;

// Registers and fields
constexpr uint8_t LPS22_WHO_AM_I               = 0x0F;
constexpr uint8_t LPS22_CHIP_ID                = 0xB1;
constexpr uint8_t LPS22_CTRL_REG1              = 0x10;
constexpr uint8_t LPS22_CTRL_REG1_BDU_EN       = (1 << 1);
constexpr uint8_t LPS22_CTRL_REG1_LPFP_DISABLE = (1 << 2);
constexpr uint8_t LPS22_CTRL_REG1_LPFP_DIV_9   = (2 << 2);
constexpr uint8_t LPS22_CTRL_REG1_LPFP_DIV_20  = (3 << 2);
constexpr uint8_t LPS22_CTRL_REG1_ODR_PWR_DOWN = 0;
constexpr uint8_t LPS22_CTRL_REG1_ODR_1HZ      = (1 << 4);
constexpr uint8_t LPS22_CTRL_REG1_ODR_10HZ     = (2 << 4);
constexpr uint8_t LPS22_CTRL_REG1_ODR_25HZ     = (3 << 4);
constexpr uint8_t LPS22_CTRL_REG1_ODR_50HZ     = (4 << 4);
constexpr uint8_t LPS22_CTRL_REG1_ODR_75HZ     = (5 << 4);
constexpr uint8_t LPS22_CTRL_REG2              = 0x11;
constexpr uint8_t LPS22_CTRL_REG2_SWRESET      = (1 << 2);
constexpr uint8_t LPS22_CTRL_REG2_I2C_DIS      = (1 << 3);
constexpr uint8_t LPS22_CTRL_REG2_BOOT         = (1 << 7);
constexpr uint8_t LPS22_CTRL_REG2_ONESHOT      = 0x01;
constexpr uint8_t LPS22_CTRL_REG3              = 0x12;
constexpr uint8_t LPS22_CTRL_REG3_DRDY_EN      = (1 << 2);
constexpr uint8_t LPS22_CTRL_REG3_PUSH_PULL    = (0 << 6);
constexpr uint8_t LPS22_CTRL_REG3_OPEN_DRAIN   = (1 << 6);
constexpr uint8_t LPS22_CTRL_REG3_ACTIVE_HI    = (0 << 7);
constexpr uint8_t LPS22_CTRL_REG3_ACTIVE_LO    = (1 << 7);
constexpr uint8_t LPS22_CTRL_REG3_INT_S_DRDY   = 0x00;
constexpr uint8_t LPS22_REF_P_XL               = 0x15;
constexpr uint8_t LPS22_REF_P_L                = 0x16;
constexpr uint8_t LPS22_REF_P_H                = 0x17;
constexpr uint8_t LPS22_RPDS_L                 = 0x18;
constexpr uint8_t LPS22_RPDS_H                 = 0x19;
constexpr uint8_t LPS22_INT_SOURCE             = 0x25;
constexpr uint8_t LPS22_INT_SOURCE_BOOT_ON     = 0x80;
constexpr uint8_t LPS22_INT_SOURCE_IA          = 0x04;
constexpr uint8_t LPS22_INT_SOURCE_PL          = 0x02;
constexpr uint8_t LPS22_INT_SOURCE_PH          = 0x01;
constexpr uint8_t LPS22_STATUS                 = 0x27;
constexpr uint8_t LPS22_STATUS_T_OR            = 0x20;
constexpr uint8_t LPS22_STATUS_P_OR            = 0x10;
constexpr uint8_t LPS22_STATUS_T_DA            = 0x02;
constexpr uint8_t LPS22_STATUS_P_DA            = 0x01;
constexpr uint8_t LPS22_PRESSURE_OUT_XL        = 0x28;
constexpr uint8_t LPS22_PRESSURE_OUT_L         = 0x29;
constexpr uint8_t LPS22_PRESSURE_OUT_H         = 0x2A;
constexpr uint8_t LPS22_TEMP_OUT_L             = 0x2B;
constexpr uint8_t LPS22_TEMP_OUT_H             = 0x2C;

struct lps22_t {
  int32_t p, t;
  float pressure, temperature;
};

// Adafruit LPS22 Pressure Sensor - STEMMA QT / Qwiic - LPS22HB
// https://www.adafruit.com/product/4633
// https://www.st.com/resource/en/datasheet/dm00140895.pdf
//
// interesting:
// https://github.com/STMicroelectronics/stm32-lps22hb/blob/main/lps22hb.c

template<typename SensorT>
class LPS22 : public SensorT {
public:
  LPS22(uint32_t port) : SensorT(port) {}

  bool init() {
    // // This example will use SPI1 at 10 MHz.
    // spi_init(spi, LPS22_MAX_SPI_CLK_HZ);

    // gpio_set_function(DI, GPIO_FUNC_SPI);
    // gpio_set_function(CLK, GPIO_FUNC_SPI);
    // gpio_set_function(DO, GPIO_FUNC_SPI);

    // // Chip select is active-low, so we'll initialise it to a driven-high state
    // gpio_init(CS);
    // gpio_set_dir(CS, GPIO_OUT);
    // gpio_put(CS, 1); // CS high is inactive

    // SensorT::CS = CS;

    uint8_t id;
    SensorT::read_registers(LPS22_WHO_AM_I, &id, 1);
    // printf("id: %d\n", (int)id); // 0xB1 == 177

    if ((id != LPS22_CHIP_ID)) {
      return false;
    }

    // SWRESET - reset regs to default
    uint8_t reg = LPS22_CTRL_REG2_SWRESET;
    SensorT::write_register(LPS22_CTRL_REG2, reg);

    // Enable continous, 75Hz, LPF (ODR/9 = 8Hz)
    reg = LPS22_CTRL_REG1_ODR_1HZ;
    reg |= LPS22_CTRL_REG1_LPFP_DIV_9;
    reg |= LPS22_CTRL_REG1_BDU_EN;
    SensorT::write_register(LPS22_CTRL_REG1, reg);

    // disable I2C
    reg = LPS22_CTRL_REG2_I2C_DIS;
    SensorT::write_register(LPS22_CTRL_REG2, reg);

    // Enable DRDY on pin when data ready
    // CTRL_REG3, INT_H_L, defaults to 0 -> active high
    //            PP_OD, defaults to 0 -> push-pull on the pin
    reg = LPS22_CTRL_REG3_DRDY_EN;
    reg |= LPS22_CTRL_REG3_INT_S_DRDY;
    // reg |= LPS22_CTRL_REG3_OPEN_DRAIN;
    SensorT::write_register(LPS22_CTRL_REG3, reg);

    // FIFO_CTRL = 0, bypass mode

    return true;
  }

  lps22_t read() {
    uint8_t sensor_data[5]{0,0,0,0,0};
    SensorT::read_registers(LPS22_PRESSURE_OUT_XL, sensor_data, 5);

    int32_t p = (int32_t)sensor_data[0];
    p |= (int32_t)(sensor_data[1] << 8);
    p |= (int32_t)(sensor_data[2] << 16);
    // 2's complement fix?
    // if(up & 0x00800000) up |= 0xFF000000;
    // int32_t p = (int32_t) up;

    int32_t t = (int32_t)(sensor_data[3]);
    t |= (int32_t)(sensor_data[4] << 8);

    lps22_t ret{
        p, t,
        (float)(p) / 4096.0f, // hPa
        (float)(t) / 100.0f  // C
    };
    return ret;
  }
};

using LPS22SPI = LPS22<SensorSPI>;

// #endif // use_sensor_lps22hb