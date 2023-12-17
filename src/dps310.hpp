
#pragma once

#include "sensor.hpp"
#include <cstdint>

namespace DPS310 {

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

constexpr uint8_t PROD_ID       = 0x00;

constexpr uint8_t DPS310_ADDR   = 0x77; // default

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

  bool init() {
    uint8_t id = readRegister(PRODID);
    if ((id & 0x0F) == PROD_ID) return false;

    reset();
    sleep_ms(100); // ??

    // standby MSR_CTRL

    writeRegister(MEAS_CFG, 0x07); // continous press/temp

    scale        = 3670016; // 4x oversample

    uint8_t os   = 2; // 4x oversample, 8.4 msec
    uint8_t rate = 7 << 4;
    uint8_t ext  = readRegister(TMP_COEF_SRCE); // already shifted

    writeRegister(TMP_CFG, ext | rate | os);

    os   = 2; // 4x oversample, 8.4 msec
    rate = 7 << 4;
    writeRegister(PRS_CFG, rate | os);

    uint8_t src = readRegister(TMP_COEF_SRCE) >> 7;
    read_coeffs();
  }

  // temp/press ready to read
  bool ready() {
    uint8_t rdy = readRegister(MEAS_CFG);
    return rdy & 0x30;
  }

  // sensor reboot is ready
  bool sensor_ready() {
    uint8_t rdy = readRegister(MEAS_CFG);
    return rdy & 0x40;
  }

  // coefficients ready to read
  bool coeffs_ready() {
    uint8_t rdy = readRegister(MEAS_CFG);
    return rdy & 0x80;
  }

  dps310_t read() {
    uint8_t buf[6];
    int32_t raw;

    dps310_t ret{0};
    ret.ok = false;

    if (readRegisters(PRS_B2, 6, buf) == false) return ret;
    // FIXME: I think this is out of order!!!

    // 4.9.2
    raw        = (buf[3] << 16) | (buf[4] << 8) | buf[5];
    float temp = c0Half + c1 * (float)raw / scale;

    // 4.9.1
    raw             = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    float pres      = (float)raw / scale;
    float A         = pres * (c10 + pres * (c20 + pres * c30));
    float B         = temp * (c01 + pres * (c11 + pres * c21));
    pres            = c00 + A + B;

    ret.temperature = temp;
    ret.pressure    = pres;
    ret.ok          = true;

    return ret;
  }

  void reset() { writeRegister(RESET, 0x09); }

  // returns altitude in meters
  float altitude(float pressure, float seaLevelhPa = 1013.25) {
    float altitude =
        44330 * (1.0 - pow((pressure / 100) / seaLevelhPa, 0.1903));
    return altitude;
  }

private:
  // compensation coefficients
  int32_t c0Half;
  int32_t c0;
  int32_t c1;
  int32_t c00;
  int32_t c10;
  int32_t c01;
  int32_t c11;
  int32_t c20;
  int32_t c21;
  int32_t c30;

  uint32_t scale;

  float calc_temp(int32_t raw) {
    // FIXME: I think this is out of order!!!

    // 4.9.2
    float temp = c0Half + c1 * (float)raw / scale;

    // 4.9.1
    float pres = (float)raw / scale;
    float A    = pres * (c10 + pres * (c20 + pres * c30));
    float B    = temp * (c01 + pres * (c11 + pres * c21));
    pres       = c00 + A + B;

    return pres;
  }

  void read_coeffs() {
    // TODO: remove magic number
    uint8_t buffer[18];
    // read COEF registers to buffer
    int16_t ret = readRegisters(COEF, 18, buffer);

    // compose coefficients from buffer content
    c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    getTwosComplement(&c0Half, 12);
    // c0 is only used as c0*0.5, so c0_half is calculated immediately
    c0Half = c0Half / 2U;

    // now do the same thing for all other coefficients
    c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    getTwosComplement(&c1, 12);
    c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) |
          (((uint32_t)buffer[5] >> 4) & 0x0F);
    getTwosComplement(&c00, 20);
    c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) |
          (uint32_t)buffer[7];
    getTwosComplement(&c10, 20);

    c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    getTwosComplement(&c01, 16);

    c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    getTwosComplement(&c11, 16);
    c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    getTwosComplement(&c20, 16);
    c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    getTwosComplement(&c21, 16);
    c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    getTwosComplement(&c30, 16);
    // return DPS__SUCCEEDED;
  }

  void getTwosComplement(int32_t *raw, uint8_t length) {
    if (*raw & ((uint32_t)1 << (length - 1))) {
      *raw -= (uint32_t)1 << length;
    }
  }

  void coeffs_alt() {

    uint8_t coeffs[18];
    int16_t ret = readRegisters(COEF, 18, coeffs);

    c0  = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
    c0  = twosComplement(c0, 12);

    c1  = twosComplement((((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2], 12);

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
  }
  int32_t twosComplement(int32_t val, uint8_t bits) {
    if (val & ((uint32_t)1 << (bits - 1))) {
      val -= (uint32_t)1 << bits;
    }
    return val;
  }
};

} // namespace DPS310