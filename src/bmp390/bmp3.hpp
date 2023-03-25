/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
// https://www.mide.com/air-pressure-at-altitude-calculator

#pragma once

#include "../sensor.hpp"

namespace BMP390 {

constexpr uint8_t OVERSAMPLING_1X      = 0x00;
constexpr uint8_t OVERSAMPLING_2X      = 0x01;
constexpr uint8_t OVERSAMPLING_4X      = 0x02;
constexpr uint8_t OVERSAMPLING_8X      = 0x03;
constexpr uint8_t OVERSAMPLING_16X     = 0x04;
constexpr uint8_t OVERSAMPLING_32X     = 0x05;

constexpr uint8_t IIR_FILTER_DISABLE   = 0x00;
constexpr uint8_t IIR_FILTER_COEFF_1   = 0x01;
constexpr uint8_t IIR_FILTER_COEFF_3   = 0x02;
constexpr uint8_t IIR_FILTER_COEFF_7   = 0x03;
constexpr uint8_t IIR_FILTER_COEFF_15  = 0x04;
constexpr uint8_t IIR_FILTER_COEFF_31  = 0x05;
constexpr uint8_t IIR_FILTER_COEFF_63  = 0x06;
constexpr uint8_t IIR_FILTER_COEFF_127 = 0x07;

constexpr uint8_t ODR_200_HZ           = 0x00;
constexpr uint8_t ODR_100_HZ           = 0x01;
constexpr uint8_t ODR_50_HZ            = 0x02;
constexpr uint8_t ODR_25_HZ            = 0x03;
constexpr uint8_t ODR_12_5_HZ          = 0x04;
constexpr uint8_t ODR_6_25_HZ          = 0x05;
constexpr uint8_t ODR_3_1_HZ           = 0x06;
constexpr uint8_t ODR_1_5_HZ           = 0x07;
constexpr uint8_t ODR_0_78_HZ          = 0x08;
constexpr uint8_t ODR_0_39_HZ          = 0x09;
constexpr uint8_t ODR_0_2_HZ           = 0x0A;
constexpr uint8_t ODR_0_1_HZ           = 0x0B;
constexpr uint8_t ODR_0_05_HZ          = 0x0C;
constexpr uint8_t ODR_0_02_HZ          = 0x0D;
constexpr uint8_t ODR_0_01_HZ          = 0x0E;
constexpr uint8_t ODR_0_006_HZ         = 0x0F;
constexpr uint8_t ODR_0_003_HZ         = 0x10;
constexpr uint8_t ODR_0_001_HZ         = 0x11;

constexpr uint8_t SOFT_RESET           = 0xB6;
constexpr uint8_t LEN_CALIB_DATA       = 21;

enum OsMode : uint8_t {
  OS_MODE_PRES_1X_TEMP_1X,
  OS_MODE_PRES_2X_TEMP_1X,
  OS_MODE_PRES_4X_TEMP_1X,
  OS_MODE_PRES_8X_TEMP_1X,
  OS_MODE_PRES_16X_TEMP_2X,
  OS_MODE_PRES_32X_TEMP_2X
};

enum FilterCoef : uint8_t {
  FILTER_COEF_OFF = IIR_FILTER_DISABLE,
  FILTER_COEF_2   = IIR_FILTER_COEFF_1,
  FILTER_COEF_4   = IIR_FILTER_COEFF_3,
  FILTER_COEF_8   = IIR_FILTER_COEFF_7,
  FILTER_COEF_16  = IIR_FILTER_COEFF_15,
  FILTER_COEF_32  = IIR_FILTER_COEFF_31,
  FILTER_COEF_64  = IIR_FILTER_COEFF_63,
  FILTER_COEF_128 = IIR_FILTER_COEFF_127,
};

constexpr uint8_t MODE_NORMAL  = 0x03; // continous sampling
constexpr uint8_t ADDR_I2C     = 0x77;
constexpr uint8_t ADDR_I2C_ALT = 0x76;

struct pt_t {
  float press, temp;
  bool ok;
};

struct bmp3_available_t {
  bool press, temp; // sensor available?
};

class gciBMP390 : public SensorI2C {
public:
  gciBMP390(TwoWire *i2c, const uint8_t addr = ADDR_I2C);

  bool init(const OsMode mode=OS_MODE_PRES_2X_TEMP_1X);

  bool setOsMode(const OsMode mode);
  bool setOverSampling(uint8_t posr, uint8_t tosr);
  bool setODR(uint8_t odr);
  bool setIIR(uint8_t iir);
  bool setInterrupt();
  bool setPowerMode(uint8_t mode);

  pt_t read_raw();
  inline pt_t read() { return read_raw(); }
  bool ready();

  float altitude(const float p);

  inline bool reset() { return soft_reset(); }

  bool found;

protected:
  uint8_t buffer[LEN_CALIB_DATA];

  float compensate_temperature(const uint32_t uncomp_temp); // datasheet pg 55
  float compensate_pressure(const uint32_t uncomp_press);   // datasheet pg 56
  bool get_calib_data();

  bool sleep();

  bool soft_reset();

  struct bmp3_reg_calib_data {
    float par_t1;
    float par_t2;
    float par_t3;

    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;

    float t_lin; // was int64_t??
  } calib;
};

} // namespace BMP390
