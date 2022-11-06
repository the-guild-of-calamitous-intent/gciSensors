/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "bosch/bmp3.h"
#include "../sensor.hpp"

namespace BMP390 {

enum OsMode : uint8_t {
  OS_MODE_PRES_1X_TEMP_1X,
  OS_MODE_PRES_2X_TEMP_1X,
  OS_MODE_PRES_4X_TEMP_1X,
  OS_MODE_PRES_8X_TEMP_1X,
  OS_MODE_PRES_16X_TEMP_2X,
  OS_MODE_PRES_32X_TEMP_2X
};

enum FilterCoef : uint8_t {
  FILTER_COEF_OFF = BMP3_IIR_FILTER_DISABLE,
  FILTER_COEF_2 = BMP3_IIR_FILTER_COEFF_1,
  FILTER_COEF_4 = BMP3_IIR_FILTER_COEFF_3,
  FILTER_COEF_8 = BMP3_IIR_FILTER_COEFF_7,
  FILTER_COEF_16 = BMP3_IIR_FILTER_COEFF_15,
  FILTER_COEF_32 = BMP3_IIR_FILTER_COEFF_31,
  FILTER_COEF_64 = BMP3_IIR_FILTER_COEFF_63,
  FILTER_COEF_128 = BMP3_IIR_FILTER_COEFF_127,
};

struct pt_t {
  float press, temp;
  bool ok;
};

class gciBMP390 : public Sensor {
public:
  gciBMP390(TwoWire *i2c, const uint8_t addr = BMP3_ADDR_I2C_SEC);

  bool init();

  bool setOsMode(const OsMode mode);
  bool setFilterCoef(const FilterCoef val);

  pt_t read();

  float altitude(const float p) {
    // Probably best not to run here ... very computational.
    // pre compute some of this?
    // call atmospalt() ... like matlab?
    // same as mean sea level (MSL) altitude
    // Altitude from pressure:
    // https://www.mide.com/air-pressure-at-altitude-calculator
    // const float Tb = 15; // temperature at sea level [C] - doesn't work
    // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
    constexpr float Tb = 288.15f;          // temperature at sea level [K]
    constexpr float Lb = -0.0065f;         // lapse rate [K/m]
    constexpr float Pb = 101325.0f;        // pressure at sea level [Pa]
    constexpr float R = 8.31446261815324f; // universal gas const [Nm/(mol K)]
    constexpr float M = 0.0289644f;        // molar mass of Earth's air [kg/mol]
    constexpr float g0 = 9.80665f;         // gravitational const [m/s^2]

    constexpr float exp = -R * Lb / (g0 * M);
    constexpr float scale = Tb / Lb;
    constexpr float inv_Pb = 1.0f / Pb;

    return scale * (std::pow(p * inv_Pb, exp) - 1.0);
  }

  inline bool reset() { return (bmp3_soft_reset() == BMP3_OK); }
  inline int8_t get_error_code() const { return err; }

  bool found;

protected:
  int8_t err;
  bmp3_settings req_settings, settings;
};

} // namespace gci
