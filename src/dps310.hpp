
#pragma once

#include "sensor.hpp"
#include <cstdint>

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

struct dps310_t {
  float pressure;
  float temperature;
  bool ok;
};

class gciDPS310 : public SensorI2C {
public:
  gciDPS310(const uint8_t addr = DPS310_ADDR, uint32_t port = 0)
      : SensorI2C(addr, port) {}
  ~gciDPS310() {}

  bool init(uint8_t sample_rate) {
    reset();
    do {
      sleep_ms(15); // pg 10, Table 8
    } while (sensor_ready() == false);

    writeRegister(MEAS_CFG, 0x00); // set to idle/stop
    sleep_ms(50); // how long?

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

    uint8_t ext = readRegister(TMP_COEF_SRCE) & 0x80; // ext already shifted

    writeRegister(TMP_CFG, ext | trate | tos);
    writeRegister(PRS_CFG, prate | pos);

    do {
      sleep_ms(45); // pg 10, Table 8
    } while (coeffs_ready() == false);
    if (get_coeffs() == false) return false;

    writeRegister(MEAS_CFG, 0x07); // continous press/temp

    return true;
  }

  // press ready to read: [TMP_RDY-2, PRS_RDY-1]
  bool ready() {
    uint8_t rdy = readRegister(MEAS_CFG);
    return (bool)(rdy & 0x10);
  }

  // sensor init from reboot is ready
  bool sensor_ready() {
    uint8_t rdy = readRegister(MEAS_CFG);
    return rdy & 0x40;
  }

  // coefficients ready to read
  bool coeffs_ready() {
    uint8_t rdy = readRegister(MEAS_CFG);
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

    dps310_t ret{0};
    ret.ok = false;
    if (ready() == false) return ret;
    if (readRegisters(PRS_B2, 6, buf) == false) return ret;

    // 4.9.2
    t_raw  = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | buf[5]; // MSB first???
    t_raw = twosComplement(t_raw, 24);
    temp = c0 * 0.5f + c1 * (float)t_raw / tscale;

    // 4.9.1
    p_raw  = (buf[0] << 16) | (buf[1] << 8) | buf[2]; // MSB first???
    p_raw = twosComplement(p_raw, 24);
    pres = (float)p_raw / pscale;
    A    = pres * (c10 + pres * (c20 + pres * c30));
    B    = (t_raw / tscale) * (c01 + pres * (c11 + pres * c21));
    pres = c00 + A + B;

    ret.temperature = temp;
    ret.pressure    = pres;
    ret.ok          = true;

    return ret;
  }

  void reset() { writeRegister(RESET, 0x09); }

  // returns altitude in meters
  float altitude(float pressure, float seaLevelhPa = 1013.25) {
    float alt = 44330 * (1.0 - pow((pressure / 100) / seaLevelhPa, 0.1903));
    return alt;
  }

private:
  // coefficients
  int32_t c0;
  int32_t c1;
  int32_t c00;
  int32_t c10;
  int32_t c01;
  int32_t c11;
  int32_t c20;
  int32_t c21;
  int32_t c30;

  float pscale;
  float tscale;

  bool get_coeffs() {

    uint8_t coeffs[18];
    bool ret = readRegisters(COEF, 18, coeffs);
    if (ret == false) return false;

    c0  = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
    c0  = twosComplement(c0, 12);

    c1  = (((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2];
    c1  = twosComplement(c1, 12);

    c00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
          (((uint32_t)coeffs[5] >> 4) & 0x0F);
    c00 = twosComplement(c00, 20);

    c10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
          (uint32_t)coeffs[7];
    c10 = twosComplement(c10, 20);

    c01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
    c11 =
        twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
    c20 =
        twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
    c21 =
        twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
    c30 =
        twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);

    return true;
  }

  int32_t twosComplement(int32_t val, uint8_t bits) {
    if (val > ((1U << bits-1)-1)) {
      val -= 1U << bits;
    }
    return val;
  }
};

} // namespace DPS310