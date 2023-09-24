/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
// https://www.mide.com/air-pressure-at-altitude-calculator

#pragma once

#include <string.h> // memcpy
#include <math.h>
#include "sensor.hpp"

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


constexpr uint8_t REG_WHO_AM_I    = 0x00;
constexpr uint8_t REG_ERR         = 0x02;
constexpr uint8_t REG_STATUS      = 0x03;
constexpr uint8_t REG_DATA        = 0x04;
constexpr uint8_t REG_INT_STATUS  = 0x11;
constexpr uint8_t REG_INT_CTRL    = 0x19;
constexpr uint8_t REG_PWR_CTRL    = 0x1B;
constexpr uint8_t REG_OSR         = 0x1C;
constexpr uint8_t REG_ODR         = 0x1D;
constexpr uint8_t REG_IIR_FILTER  = 0x1F;
constexpr uint8_t REG_CALIB_DATA  = 0x31;
constexpr uint8_t REG_CMD         = 0x7E;

constexpr uint8_t WHO_AM_I        = 0x60;
constexpr uint8_t LEN_P_T_DATA    = 6;
constexpr uint8_t CMD_RDY         = 0x10;


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

// struct pt_t {
//   int16_t press, temp;
//   bool ok;
// };

struct bmp3_available_t {
  bool press, temp; // sensor available?
};

class gciBMP390 : public SensorI2C {
public:
  gciBMP390(TwoWire *i2c, const uint8_t addr = ADDR_I2C) : SensorI2C(i2c, addr) {
    found = false;
  }

  bool init(const OsMode mode=OS_MODE_PRES_2X_TEMP_1X) {
    bool ok;

    if (!(readRegister(REG_WHO_AM_I) == WHO_AM_I)) return false;

    ok = soft_reset();
    if (!ok) return false;
    ok = get_calib_data();
    if (!ok) return false;

    setOsMode(mode);

    ok = setIIR(IIR_FILTER_COEFF_1);
    if (!ok) return false;

    ok = setInterrupt();
    if (!ok) return false;

    ok = setPowerMode(MODE_NORMAL); // continuous sampling
    if (!ok) return false;

    return true;
  }

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


  Figure 6, datasheet
  Off: 1 step delay
  2: 10 step delay
  4: 20 step delay
  */
  bool setOsMode(const OsMode mode) {
    uint8_t press_os, temp_os, odr;

    // println("fixme");

    switch (mode) {
    case OS_MODE_PRES_1X_TEMP_1X:
      press_os = OVERSAMPLING_1X;
      temp_os  = OVERSAMPLING_1X;
      odr      = ODR_200_HZ;
      break;

    case OS_MODE_PRES_2X_TEMP_1X:
      press_os = OVERSAMPLING_2X;
      temp_os  = OVERSAMPLING_1X;
      odr      = ODR_100_HZ;
      break;

    case OS_MODE_PRES_4X_TEMP_1X:
      press_os = OVERSAMPLING_4X;
      temp_os  = OVERSAMPLING_1X;
      odr      = ODR_100_HZ; // was 50
      break;

    case OS_MODE_PRES_8X_TEMP_1X:
      press_os = OVERSAMPLING_8X;
      temp_os  = OVERSAMPLING_1X;
      odr      = ODR_50_HZ;
      break;

    case OS_MODE_PRES_16X_TEMP_2X:
      press_os = OVERSAMPLING_16X;
      temp_os  = OVERSAMPLING_2X;
      odr      = ODR_25_HZ;
      break;

    case OS_MODE_PRES_32X_TEMP_2X:
      press_os = OVERSAMPLING_32X;
      temp_os  = OVERSAMPLING_2X;
      odr      = ODR_12_5_HZ;
      break;
    }

    bool ok = setOverSampling(press_os, temp_os);
    if (!ok) return false;
    ok = setODR(odr);
    if (!ok) return false;

    return true;
  }

  bool setOverSampling(uint8_t posr, uint8_t tosr) {
    uint8_t val = (tosr << 3) | posr;
    return writeRegister(REG_OSR, val);
  }

  bool setODR(uint8_t odr) { return writeRegister(REG_ODR, odr); }

  bool setIIR(uint8_t iir) {
    uint8_t val = iir << 1;
    return writeRegister(REG_IIR_FILTER, val);
  }

  bool setInterrupt() {
    // int_od: 0 = push-pull
    // int_level: 1 = active high
    // int_latch: 0 = disable
    // drdy_en: 1 = enable pressure/temperature interrupt in INT_STATUS reg
    // uint8_t val = (drdy_en << 6) | (int_level << 1);
    constexpr uint8_t drdy_en = BITS::b6; // 1 = enable pressure/temperature interrupt in INT_STATUS reg
    constexpr uint8_t int_level = BITS::b1; // 1 = active high
    uint8_t val = drdy_en | int_level;
    return writeRegister(REG_INT_CTRL, val);
  }

  bool setPowerMode(uint8_t mode) {
    bool ok;

    constexpr uint8_t PRESS_EN = 0x01;
    constexpr uint8_t TEMP_EN  = 0x02;
    uint8_t val                = (mode << 4) | TEMP_EN | PRESS_EN;
    return writeRegister(REG_PWR_CTRL, val);
  }

  const pt_t read_raw()  {
    pt_t ret = {0};
    ret.ok   = false;

    bool ok  = readRegisters(REG_DATA, LEN_P_T_DATA, buffer);
    if (!ok) return ret;

    uint32_t press = to_24b(&buffer[0]);
    uint32_t temp  = to_24b(&buffer[3]);

    // println("good read");

    ret.ok    = true;
    ret.temp  = compensate_temperature(temp); // do temp 1st!!!
    ret.press = compensate_pressure(press);
    return ret;
  }

  inline pt_t read() { return read_raw(); }
  bool ready() {
    // constexpr uint8_t DATA_READY_BIT = BITS::b3;
    // if (((readRegister(REG_INT_STATUS) & DATA_READY_BIT) == 0)) return false;
    // return true;

    constexpr uint8_t TEMP_READY_BIT = BITS::b5; // bit 5
    constexpr uint8_t PRES_READY_BIT = BITS::b6; // bit 6
    bmp3_available_t ret;
    ret.press = readRegister(REG_STATUS) & PRES_READY_BIT;
    ret.temp = readRegister(REG_STATUS) & TEMP_READY_BIT;
    return ret.press && ret.temp;
  }

  float altitude(const float p) {
    // Probably best not to run here ... very computational.
    // pre compute some of this?
    // call atmospalt() ... like matlab?
    // same as mean sea level (MSL) altitude
    // Altitude from pressure:
    // https://www.mide.com/air-pressure-at-altitude-calculator
    // const float Tb = 15; // temperature at sea level [C] - doesn't work
    // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
    constexpr float Tb    = 288.15f;           // temperature at sea level [K]
    constexpr float Lb    = -0.0065f;          // lapse rate [K/m]
    constexpr float Pb    = 101325.0f;         // pressure at sea level [Pa]
    constexpr float R     = 8.31446261815324f; // universal gas const [Nm/(mol K)]
    constexpr float M     = 0.0289644f; // molar mass of Earth's air [kg/mol]
    constexpr float g0    = 9.80665f;   // gravitational const [m/s^2]

    constexpr float exp   = -R * Lb / (g0 * M);
    constexpr float scale = Tb / Lb;
    constexpr float inv_Pb = 1.0f / Pb;

    return scale * (pow(p * inv_Pb, exp) - 1.0);
  }

  inline bool reset() { return soft_reset(); }

  bool found;

protected:
  uint8_t buffer[LEN_CALIB_DATA];

  float compensate_temperature(const uint32_t uncomp_temp) { // datasheet pg 55
    float pd1   = (float)uncomp_temp - calib.par_t1;
    float pd2   = pd1 * calib.par_t2;
    calib.t_lin = pd2 + (pd1 * pd1) * calib.par_t3;
    return (float)calib.t_lin;
  }

  float compensate_pressure(const uint32_t uncomp_press) {   // datasheet pg 56
    float pd1 = calib.par_p6 * calib.t_lin;
    float pd2 = calib.par_p7 * (calib.t_lin * calib.t_lin);
    float pd3 = calib.par_p8 * (calib.t_lin * calib.t_lin * calib.t_lin);
    float po1 = calib.par_p5 + pd1 + pd2 + pd3;

    pd1       = calib.par_p2 * calib.t_lin;
    pd2       = calib.par_p3 * (calib.t_lin * calib.t_lin);
    pd3       = calib.par_p4 * (calib.t_lin * calib.t_lin * calib.t_lin);
    float po2 = (float)uncomp_press * (calib.par_p1 + pd1 + pd2 + pd3);

    pd1       = (float)uncomp_press * (float)uncomp_press;
    pd2       = calib.par_p9 + calib.par_p10 * calib.t_lin;
    pd3       = pd1 * pd2;
    float pd4 =
        pd3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) *
                  calib.par_p11;
    float comp_press = po1 + po2 + pd4;

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

      // println("wrote SOFT_RESET");
  #if defined(ARDUINO)
      delay(2); // was 2 ... too quick?
  #endif
      // Read for command error status
      if (readRegister(REG_ERR) & REG_CMD) return false;

      // print("No command errors");

      return true;
    }

    // println("cmd_rdy_status failed CMD_RDY");

    return false;
  }

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
