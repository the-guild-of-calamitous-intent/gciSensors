/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once


// #if defined(__USE_SENSOR_DPS310__)

#include "sensor.hpp"
#include <stdint.h>

namespace DPS310 {

// pg 25, Table 15
constexpr uint8_t PRS_B2        = 0x00; // Highest byte of pressure data
constexpr uint8_t TMP_B2        = 0x03; // Highest byte of temperature data
constexpr uint8_t PRS_CFG       = 0x06; // Pressure configuration
constexpr uint8_t TMP_CFG       = 0x07; // Temperature configuration
constexpr uint8_t MEAS_CFG      = 0x08; // Sensor configuration
constexpr uint8_t CFG_REG       = 0x09; // Interrupt/FIFO configuration
constexpr uint8_t RESET         = 0x0C; // Soft reset
constexpr uint8_t COEF          = 0x10; // coefficients
constexpr uint8_t PRODID        = 0x0D; // Register that contains the part ID
constexpr uint8_t TMP_COEF_SRCE = 0x28; // Temperature calibration src

// for pressure and temperature, but pressure determines the
// overall datarate
constexpr uint8_t DPS_128HZ_ALT = 8; // alternate 128Hz, max speed
constexpr uint8_t DPS_128HZ     = 7;
constexpr uint8_t DPS_64HZ      = 6;
constexpr uint8_t DPS_32HZ      = 5;
constexpr uint8_t DPS_16HZ      = 4;
constexpr uint8_t DPS_8HZ       = 3;

// pg 15, Table 9, for pressure and temperature
constexpr float OVERSAMPLE_1X_SF   = 524288;
constexpr float OVERSAMPLE_2X_SF   = 1572864;
constexpr float OVERSAMPLE_4X_SF   = 3670016;
constexpr float OVERSAMPLE_8X_SF   = 7864320;
constexpr float OVERSAMPLE_16X_SF  = 253952;
constexpr float OVERSAMPLE_32X_SF  = 516096;
constexpr float OVERSAMPLE_64X_SF  = 1040384;
constexpr float OVERSAMPLE_128X_SF = 2088960;

// appears to be straight averaging, nothing fancy
constexpr uint32_t OVERSAMPLE_1X   = 0;
constexpr uint32_t OVERSAMPLE_2X   = 1;
constexpr uint32_t OVERSAMPLE_4X   = 2;
constexpr uint32_t OVERSAMPLE_8X   = 3;
constexpr uint32_t OVERSAMPLE_16X  = 4;
constexpr uint32_t OVERSAMPLE_32X  = 5;
constexpr uint32_t OVERSAMPLE_64X  = 6;
constexpr uint32_t OVERSAMPLE_128X = 7;

// wish they had a WHO_AM_I like EVERYONE else???
// constexpr uint8_t PROD_ID     = 0x00;

constexpr uint8_t DPS310_ADDR = 0x77; // default


enum dps_error : uint8_t {
  NONE = 0,
  ERROR_MEAS_CFG,
  ERROR_COEFF_NOT_READY,
  ERROR_COEFF_FAIL,
  ERROR_TMP_CFG,
  ERROR_PRS_CFG
};

struct dps310_t {
  float pressure;
  float temperature;
  bool ok;
};

class gciDPS310 : public SensorI2C {
public:
  gciDPS310(const uint32_t port, const uint8_t addr = DPS310_ADDR)
      : SensorI2C(addr, port) {}
  ~gciDPS310() {}

  uint8_t init(uint8_t sample_rate) {
    reset();
    do {
      sleep_ms(15); // pg 10, Table 8
    } while (sensor_ready() == false);

    // set to idle/stop
    if (!writeRegister(MEAS_CFG, 0x00)) return ERROR_MEAS_CFG;
    sleep_ms(50);                  // how long?

    uint8_t pos;
    uint8_t prate;
    uint8_t tos;
    uint8_t trate;

    /*
    OK, there are a lot of ways you can mix and match how
    you oversample and set your datarate, so I picked some
    that seemed good.
    pg 30, Table 16
    Rate_temp * MeasTime_temp + Rate_pres * MeasTime_pres < 1 second
    |    | Press   | Temp    |     |     |
    |    |----|----|----|----| Pa  |  cm |
    | Hz | OS | Hz | OS | Hz | RMS | RMS |
    |----|----|----|----|----|-----|-----|
    |128F| 1  | 128| 1  | 128| 2.5 | 20.8|
    | 128| 2  | 128| 2  | 64 | 1.0 | 8.3 |
    | 64 | 4  | 64 | 2  | 64 | 0.5 | 4.2 |
    | 32 | 8  | 32 | 4  | 32 | 0.4 | 3.3 |
    | 16 | 16 | 16 | 16 | 16 | 0.35| 2.9 |
    |  8 | 32 |  8 | 32 |  8 | 0.3 | 2.5 |
    */
    switch (sample_rate) {
    case DPS_128HZ_ALT:
      pscale = OVERSAMPLE_1X_SF;
      tscale = OVERSAMPLE_1X_SF;
      pos    = OVERSAMPLE_1X;
      tos    = OVERSAMPLE_1X;
      prate  = DPS_128HZ << 4;
      trate  = DPS_128HZ << 4;
      break;
    case DPS_128HZ: // mix datarate here
      pscale = OVERSAMPLE_2X_SF;
      tscale = OVERSAMPLE_2X_SF;
      pos    = OVERSAMPLE_2X;
      tos    = OVERSAMPLE_2X;
      prate  = DPS_128HZ << 4;
      trate  = DPS_64HZ << 4;
      break;
    case DPS_64HZ: // mix OS here
      pscale = OVERSAMPLE_4X_SF;
      tscale = OVERSAMPLE_2X_SF;
      pos    = OVERSAMPLE_4X;
      tos    = OVERSAMPLE_2X;
      prate  = DPS_64HZ << 4;
      trate  = DPS_64HZ << 4;
      break;
    case DPS_32HZ:
      pscale = OVERSAMPLE_8X_SF;
      tscale = OVERSAMPLE_8X_SF;
      pos    = OVERSAMPLE_8X;
      tos    = OVERSAMPLE_8X;
      prate  = DPS_32HZ << 4;
      trate  = DPS_32HZ << 4;
      break;
    case DPS_16HZ: // FIXME: enable pres/temp shift for higher OS
      pscale = OVERSAMPLE_16X_SF;
      tscale = OVERSAMPLE_16X_SF;
      pos    = OVERSAMPLE_16X;
      tos    = OVERSAMPLE_16X;
      prate  = DPS_16HZ << 4;
      trate  = DPS_16HZ << 4;
      break;
    case DPS_8HZ:
      pscale = OVERSAMPLE_32X_SF;
      tscale = OVERSAMPLE_32X_SF;
      pos    = OVERSAMPLE_32X;
      tos    = OVERSAMPLE_32X;
      prate  = DPS_8HZ << 4;
      trate  = DPS_8HZ << 4;
      break;
    default:
      return false;
    }

    uint8_t ext{0};
    readRegister(TMP_COEF_SRCE, &ext);
    ext &= 0x80; // ext already shifted

    if (!writeRegister(TMP_CFG, ext | trate | tos)) return ERROR_TMP_CFG;
    if (!writeRegister(PRS_CFG, prate | pos)) return ERROR_PRS_CFG;

    int32_t incr = 10;
    do {
      sleep_ms(45); // pg 10, Table 8
      if (incr-- == 0) return ERROR_COEFF_NOT_READY;
    } while (coeffs_ready() == false);
    if (get_coeffs() == false) return ERROR_COEFF_FAIL;

    // continous press/temp
    if (!writeRegister(MEAS_CFG, 0x07)) return ERROR_MEAS_CFG;

    return NONE;
  }

  // press ready to read: [TMP_RDY-2, PRS_RDY-1]
  bool ready() {
    uint8_t rdy{0};
    readRegister(MEAS_CFG,&rdy);
    return (bool)(rdy & 0x10);
  }

  // sensor init from reboot is ready
  bool sensor_ready() {
    uint8_t rdy{0};
    readRegister(MEAS_CFG,&rdy);
    return rdy & 0x40;
  }

  // coefficients ready to read
  bool coeffs_ready() {
    uint8_t rdy{0};
    readRegister(MEAS_CFG,&rdy);
    return rdy & 0x80;
  }

  bool stop() {
    return writeRegister(MEAS_CFG, 0x00); // set to idle/stop
  }

  dps310_t read() {
    uint8_t buf[6]{0};
    int32_t t_raw, p_raw;
    float temp{0.0f};
    float pres{0.0f};
    float A, B;

    dps310_t ret;
    ret.ok = false;
    if (ready() == false) return ret;
    if (readRegisters(PRS_B2, 6, buf) == false) return ret;

    // 4.9.2
    t_raw = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) |
            buf[5]; // MSB first???
    t_raw = twosComplement((uint32_t)t_raw, 24); // CHANGED ... OK?
    temp  = c0 * 0.5f + c1 * (float)t_raw / tscale;

    // 4.9.1
    p_raw           = (buf[0] << 16) | (buf[1] << 8) | buf[2]; // MSB first???
    p_raw           = twosComplement((uint32_t)p_raw, 24); // CHANGED ... OK
    pres            = (float)p_raw / pscale;
    A               = pres * (c10 + pres * (c20 + pres * c30));
    B               = (static_cast<float>(t_raw) / tscale) * (c01 + pres * (c11 + pres * c21));
    pres            = c00 + A + B;

    ret.temperature = temp;
    ret.pressure    = pres;
    ret.ok          = true;

    return ret;
  }

  void reset() { writeRegister(RESET, 0x09); }

private:
  // // coefficients
  // int32_t c0;
  // int32_t c1;
  // int32_t c00;
  // int32_t c10;
  // int32_t c01;
  // int32_t c11;
  // int32_t c20;
  // int32_t c21;
  // int32_t c30;
  float c0{0.0f};
  float c1{0.0f};
  float c00{0.0f};
  float c10{0.0f};
  float c01{0.0f};
  float c11{0.0f};
  float c20{0.0f};
  float c21{0.0f};
  float c30{0.0f};

  // scaling
  float pscale;
  float tscale;

  bool get_coeffs() {
    uint8_t coeffs[18];
    if (!readRegisters(COEF, 18, coeffs)) return false;

    int32_t ic0;
    int32_t ic1;
    int32_t ic00;
    int32_t ic10;
    int32_t ic01;
    int32_t ic11;
    int32_t ic20;
    int32_t ic21;
    int32_t ic30;

    // ic0  = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
    // ic0  = twosComplement(ic0, 12);

    // ic1  = (((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2];
    // ic1  = twosComplement(ic1, 12);

    // ic00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
    //       (((uint32_t)coeffs[5] >> 4) & 0x0F);
    // ic00 = twosComplement(ic00, 20);

    // ic10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
    //       (uint32_t)coeffs[7];
    // ic10 = twosComplement(ic10, 20);

    // ic01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
    // ic11 =
    //     twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
    // ic20 =
    //     twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
    // ic21 =
    //     twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
    // ic30 =
    //     twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);


    // See section 8.11, Calibration Coefficients (COEF), of datasheet

    // 0x11 c0 [3:0] + 0x10 c0 [11:4]
    ic0 = twosComplement(((uint32_t)coeffs[0] << 4) | (((uint32_t)coeffs[1] >> 4) & 0x0F), 12);

    // 0x11 c1 [11:8] + 0x12 c1 [7:0]
    ic1 = twosComplement((((uint32_t)coeffs[1] & 0x0F) << 8) | (uint32_t)coeffs[2], 12);

    // 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
    ic00 = twosComplement(((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) | (((uint32_t)coeffs[5] >> 4) & 0x0F), 20);

    // 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
    ic10 = twosComplement((((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) | (uint32_t)coeffs[7], 20);

    // 0x18 c01 [15:8] + 0x19 c01 [7:0]
    ic01 = twosComplement(((uint32_t)coeffs[8] << 8) | (uint32_t)coeffs[9], 16);

    // 0x1A c11 [15:8] + 0x1B c11 [7:0]
    ic11 = twosComplement(((uint32_t)coeffs[10] << 8) | (uint32_t)coeffs[11], 16);

    // 0x1C c20 [15:8] + 0x1D c20 [7:0]
    ic20 = twosComplement(((uint32_t)coeffs[12] << 8) | (uint32_t)coeffs[13], 16);

    // 0x1E c21 [15:8] + 0x1F c21 [7:0]
    ic21 = twosComplement(((uint32_t)coeffs[14] << 8) | (uint32_t)coeffs[15], 16);

    // 0x20 c30 [15:8] + 0x21 c30 [7:0]
    ic30 = twosComplement(((uint32_t)coeffs[16] << 8) | (uint32_t)coeffs[17], 16);

    c0 = static_cast<float>(ic0);
    c1 = static_cast<float>(ic1);
    c00 = static_cast<float>(ic00);
    c10 = static_cast<float>(ic10);
    c01 = static_cast<float>(ic01);
    c11 = static_cast<float>(ic11);
    c20 = static_cast<float>(ic20);
    c21 = static_cast<float>(ic21);
    c30 = static_cast<float>(ic30);

    return true;
  }

  // int32_t twosComplement(int32_t val, uint8_t bits) {
  //   // // before
  //   // if (val > ((1U << (bits - 1)) - 1)) { // double check, put paraenthese around (bits-1)
  //   //   val -= (1U << bits);
  //   // }

  //   if (val > ((1U << (bits - 1)) - 1)) { // double check, put paraenthese around (bits-1)
  //     val -= static_cast<int32_t>(1U << bits);
  //   }

  //   return val;
  // }

  // was uint32_t raw -> int32_t
  int32_t twosComplement(uint32_t raw, const uint8_t length) {
    if (raw & ((int)1 << (length - 1))) {
      return ((int32_t)raw) - ((int32_t)1 << length);
    }
    return (int32_t)raw;
  }
};

} // namespace DPS310

// #endif // use_sensor_dps310