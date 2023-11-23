/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
/*

https://www.mide.com/air-pressure-at-altitude-calculator

*/

/*

Table 10, datasheet
Use Case   | Mode | Res     | P   | T  | IIR | ODR | Noise RMS [cm] |
---------------------------------------------------------------------
Indoor Nav | Norm | Ultr Hi | x16 | x2 | 4   | 25  | 5
Drone      | Norm | Std res | x8  | x1 | 2   | 50  | 11

Table 23, datasheet
Oversamp | P   | T  | Hz Typ |
------------------------------
Low Pwr  | x2  | x1 | 146 | << can set ODR 100Hz
Std Res  | x4  | x1 | 92  | << max ODR is 50Hz
Hi Res   | x8  | x1 | 53  |
Ultr Hi  | x16 | x2 | 27  |
*/
#pragma once

#include "sensor.hpp"
#include <math.h>
#include <string.h> // memcpy

namespace BMP390 {

constexpr uint8_t OVERSAMPLING_1X      = 0x00;
constexpr uint8_t OVERSAMPLING_2X      = 0x01;
constexpr uint8_t OVERSAMPLING_4X      = 0x02;
constexpr uint8_t OVERSAMPLING_8X      = 0x03;
constexpr uint8_t OVERSAMPLING_16X     = 0x04;
constexpr uint8_t OVERSAMPLING_32X     = 0x05;

constexpr uint8_t IIR_FILTER_DISABLE   = 0x00; // 000 0
constexpr uint8_t IIR_FILTER_COEFF_1   = 0x02; // 001 0
constexpr uint8_t IIR_FILTER_COEFF_3   = 0x04; // 010 0
constexpr uint8_t IIR_FILTER_COEFF_7   = 0x06; // 011 0
constexpr uint8_t IIR_FILTER_COEFF_15  = 0x08; // 100 0
constexpr uint8_t IIR_FILTER_COEFF_31  = 0x0A; // 101 0
constexpr uint8_t IIR_FILTER_COEFF_63  = 0x0C; // 110 0
constexpr uint8_t IIR_FILTER_COEFF_127 = 0x0E; // 111 0

// datasheet Table 45, p 38 oversample hz
constexpr uint8_t ODR_200_HZ     = 0x00;
constexpr uint8_t ODR_100_HZ     = 0x01;
constexpr uint8_t ODR_50_HZ      = 0x02;
constexpr uint8_t ODR_25_HZ      = 0x03;
constexpr uint8_t ODR_12_5_HZ    = 0x04;

constexpr uint8_t REG_WHO_AM_I   = 0x00;
constexpr uint8_t REG_ERR        = 0x02;
constexpr uint8_t REG_STATUS     = 0x03;
constexpr uint8_t REG_DATA       = 0x04;
constexpr uint8_t REG_INT_STATUS = 0x11;
constexpr uint8_t REG_INT_CTRL   = 0x19;
constexpr uint8_t REG_PWR_CTRL   = 0x1B;
constexpr uint8_t REG_OSR        = 0x1C;
constexpr uint8_t REG_ODR        = 0x1D;
constexpr uint8_t REG_IIR_FILTER = 0x1F;
constexpr uint8_t REG_SENSORTIME = 0x0C;
constexpr uint8_t REG_CALIB_DATA = 0x31;
constexpr uint8_t REG_CMD        = 0x7E;

constexpr uint8_t WHO_AM_I       = 0x60;
constexpr uint8_t CMD_RDY        = 0x10;
constexpr uint8_t PRES_READY_BIT = (1 << 5);
constexpr uint8_t TEMP_READY_BIT = (1 << 6);
constexpr uint8_t SOFT_RESET     = 0xB6;
constexpr uint8_t LEN_CALIB_DATA = 21;

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
};

constexpr uint8_t ADDR_I2C     = 0x77;
constexpr uint8_t ADDR_I2C_ALT = 0x76;

struct pt_t {
  float press, temp;
  // uint32_t time; // not sure value of counter
  bool ok;
};

enum bmp_error : uint8_t {
  NO_ERROR,
  ERROR_WHOAMI,
  ERROR_RESET,
  ERROR_CAL_DATA,
  ERROR_OS_MODE,
  ERROR_IIR_FILTER,
  ERROR_INT_PIN,
  ERROR_PWR_MODE
};

class gciBMP390 : public SensorI2C {
public:
  gciBMP390(const uint8_t addr = ADDR_I2C) : SensorI2C(addr) {}

  uint8_t init(const uint8_t odr = ODR_50_HZ,
               const uint8_t iir = IIR_FILTER_COEFF_7) {
    bool ok;

    if (!(readRegister(REG_WHO_AM_I) == WHO_AM_I)) return ERROR_WHOAMI;
    if (!soft_reset()) return ERROR_RESET;
    if (!get_calib_data()) return ERROR_CAL_DATA;
    // if (!setOsMode(mode)) return ERROR_OS_MODE;
    if (!setODR(odr)) return ERROR_OS_MODE;

    /*
    IIR Filter
    Figure 6, pg 16, datasheet
        Off:  1 step delay
    coeff 1: 10 step delay
    coeff 3: 20 step delay
    coeff 7: 40 step delay

    Table 6, p 14
                OSR    Pa   1   3   7 15 31 63 127
    ULP          x1  2.64   2 1.2 0.8
    LP           x2  1.32 1.5 0.9 0.5
    Standard     x4  0.66 1.1 0.7 0.4
    High Res     x8  0.33 0.9 0.6 0.3
    UH Res      x16  0.17
    Highest Res x32 0.085
    */
    if (!writeRegister(REG_IIR_FILTER, iir)) return ERROR_IIR_FILTER;

    // Enable interrupt pin
    // int_od: 0 = push-pull
    // int_latch: 0 = disable
    // 1 = enable pressure/temperature interrupt in INT_STATUS reg
    uint8_t DRDY_EN      = (1 << 6);
    uint8_t INT_LEVEL_HI = (1 << 1); // 1 = active high
    // latch int pin and status reg ... do I need this?
    uint8_t INT_LATCH_EN = (1 << 2);
    ok = writeRegister(REG_INT_CTRL, DRDY_EN | INT_LEVEL_HI | INT_LATCH_EN);
    if (!ok) return ERROR_INT_PIN;

    uint8_t MODE_NORMAL = (0x03 << 4); // continous sampling
    uint8_t PRESS_EN    = 0x01;
    uint8_t TEMP_EN     = 0x02;
    ok = writeRegister(REG_PWR_CTRL, MODE_NORMAL | TEMP_EN | PRESS_EN);
    if (!ok) return ERROR_PWR_MODE;

    return NO_ERROR;
  }

  const pt_t read_raw() {
    pt_t ret = {0};
    ret.ok   = false;

    if (!ready()) return ret;

    bool ok = readRegisters(REG_DATA, 6, buffer);
    if (!ok) return ret;

    uint32_t press = to_24b(buffer);
    uint32_t temp  = to_24b(&buffer[3]);

    ret.temp       = compensate_temperature(temp); // do temp 1st!!!
    ret.press      = compensate_pressure(press);

    // value?
    // bool ok  = readRegisters(REG_SENSORTIME, 3, buffer);
    // if (!ok) return ret;
    // uint32_t time = to_24b(buffer);

    ret.ok = true;
    return ret;
  }

  inline const pt_t read() { return read_raw(); }

  bool ready() {
    // constexpr uint8_t DATA_READY_BIT = BITS::b3;
    // if (((readRegister(REG_INT_STATUS) & DATA_READY_BIT) == 0)) return false;
    // return true;

    uint8_t reg = readRegister(REG_STATUS);
    return (PRES_READY_BIT & reg) && (TEMP_READY_BIT & reg);
  }

  // bmp_available_t available() {
  //   uint8_t val = readRegister(REG_STATUS) & (3 << 5);
  //   return static_cast<bmp_available_t>(val);
  // }

  float altitude(const float p) {
    // Probably best not to run here ... very computational.
    // pre compute some of this?
    // call atmospalt() ... like matlab?
    // same as mean sea level (MSL) altitude
    // Altitude from pressure:
    // https://www.mide.com/air-pressure-at-altitude-calculator
    // const float Tb = 15; // temperature at sea level [C] - doesn't work
    // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
    constexpr float Tb  = 288.15f;           // temperature at sea level [K]
    constexpr float Lb  = -0.0065f;          // lapse rate [K/m]
    constexpr float Pb  = 101325.0f;         // pressure at sea level [Pa]
    constexpr float R   = 8.31446261815324f; // universal gas const [Nm/(mol K)]
    constexpr float M   = 0.0289644f; // molar mass of Earth's air [kg/mol]
    constexpr float g0  = 9.80665f;   // gravitational const [m/s^2]

    constexpr float exp = -R * Lb / (g0 * M);
    constexpr float scale  = Tb / Lb;
    constexpr float inv_Pb = 1.0f / Pb;

    return scale * (pow(p * inv_Pb, exp) - 1.0);
  }

  inline bool reset() { return soft_reset(); }

protected:
  uint8_t buffer[LEN_CALIB_DATA];
  bmp3_reg_calib_data calib;

  bool setODR(const uint8_t odr) {
    uint8_t press_os, temp_os;

    // based oon sec 3.9.1, pg 26
    // and pg 14, section 3.4.1
    switch (odr) {
    case ODR_200_HZ:              // 24 cm
      press_os = OVERSAMPLING_1X; // 2.64 Pa
      temp_os  = OVERSAMPLING_1X; // 0.0050 C
      break;
    case ODR_100_HZ:              // 6 cm
      press_os = OVERSAMPLING_4X; // 0.66 Pa
      temp_os  = OVERSAMPLING_1X; // 0.0050 C
      break;
    case ODR_50_HZ:               // 3cm
      press_os = OVERSAMPLING_8X; // 0.33 Pa
      temp_os  = OVERSAMPLING_1X; // 0.0050 C
      break;
    case ODR_25_HZ:
      press_os = OVERSAMPLING_16X; // 0.17 Pa
      temp_os  = OVERSAMPLING_2X;  // 0.0025 C
      break;
    case ODR_12_5_HZ:
      press_os = OVERSAMPLING_32X;
      temp_os  = OVERSAMPLING_2X;
      break;
    }
    if (!writeRegister(REG_OSR, (temp_os << 3) | press_os)) return false;
    if (!writeRegister(REG_ODR, odr)) return false;

    return true;
  }

  float compensate_temperature(const uint32_t uncomp_temp) { // datasheet pg 55
    float pd1   = (float)uncomp_temp - calib.par_t1;
    float pd2   = pd1 * calib.par_t2;
    calib.t_lin = pd2 + (pd1 * pd1) * calib.par_t3;
    return (float)calib.t_lin;
  }

  float compensate_pressure(const uint32_t uncomp_press) { // datasheet pg 56
    float pd1, pd2, pd3, pd4, po1, po2, comp_press;
    const float up = (float)uncomp_press;
    pd1 = calib.par_p6 * calib.t_lin;
    pd2 = calib.par_p7 * (calib.t_lin * calib.t_lin);
    pd3 = calib.par_p8 * (calib.t_lin * calib.t_lin * calib.t_lin);
    po1 = calib.par_p5 + pd1 + pd2 + pd3;

    pd1 = calib.par_p2 * calib.t_lin;
    pd2 = calib.par_p3 * (calib.t_lin * calib.t_lin);
    pd3 = calib.par_p4 * (calib.t_lin * calib.t_lin * calib.t_lin);
    po2 = up * (calib.par_p1 + pd1 + pd2 + pd3);

    pd1 = up * up;
    pd2 = calib.par_p9 + calib.par_p10 * calib.t_lin;
    pd3 = pd1 * pd2;
    pd4 = pd3 + up * up * up * calib.par_p11;
    comp_press = po1 + po2 + pd4;

    return comp_press;
  }

  bool get_calib_data() {
    bool ok = readRegisters(REG_CALIB_DATA, LEN_CALIB_DATA, buffer);
    if (!ok) return false;

    calib.par_t1 = (float)to_16b(buffer[1], buffer[0]) / powf(2, -8);
    calib.par_t2 = (float)to_16b(buffer[3], buffer[2]) / powf(2, 30);
    calib.par_t3 = (float)buffer[4] / powf(2, 48);

    calib.par_p1 =
        ((float)to_16b(buffer[6], buffer[5]) - powf(2, 14)) / powf(2, 20);
    calib.par_p2 =
        ((float)to_16b(buffer[8], buffer[7]) - powf(2, 14)) / powf(2, 29);
    calib.par_p3  = (float)buffer[9] / powf(2, 32);
    calib.par_p4  = (float)buffer[10] / powf(2, 37);
    calib.par_p5  = (float)to_16b(buffer[12], buffer[11]) / powf(2, -3);
    calib.par_p6  = (float)to_16b(buffer[14], buffer[13]) / powf(2, 6);
    calib.par_p7  = (float)buffer[15] / powf(2, 8);
    calib.par_p8  = (float)buffer[16] / powf(2, 15);
    calib.par_p9  = (float)to_16b(buffer[18], buffer[17]) / powf(2, 48);
    calib.par_p10 = (float)buffer[19] / powf(2, 48);
    calib.par_p11 = (float)buffer[20] / powf(2, 65);

    return true;
  }

  bool sleep() {
    // uint8_t op_mode = readRegister(REG_PWR_CTRL);
    // keep bits 0-1, temp/press enable, mode = 00 (sleep)
    // op_mode = op_mode & (0x01 | 0x02);
    // return writeRegister(REG_PWR_CTRL, op_mode);
    return writeRegister(REG_PWR_CTRL, 0x00); // sleep, disable temp/press
  }

  bool soft_reset() {
    bool ok;

    // Check for command ready status
    uint8_t cmd_rdy_status = readRegister(REG_STATUS);

    // Device is ready to accept new command
    if (cmd_rdy_status & CMD_RDY) {
      // println("cmd_rdy_status is CMD_RDY");
      // Write the soft reset command in the sensor
      // datasheet, p 39, table 47, register ALWAYS reads 0x00
      writeRegister(REG_CMD, SOFT_RESET);
      sleep_ms(2); // was 2 ... too quick?
      // Read for command error status
      if (readRegister(REG_ERR) & REG_CMD) return false;
      return true;
    }
    return false;
  }
};

} // namespace BMP390
