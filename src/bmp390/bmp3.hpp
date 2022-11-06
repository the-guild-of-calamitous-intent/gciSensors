/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

// #include <Wire.h>
// #include <Arduino.h>
// #include <cstddef>
// #include <cstdint>

#include "bosch/bmp3_defs.h"
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


constexpr uint8_t MODE_NORMAL = 0x03;

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

  inline bool reset() { return soft_reset(); }
  // inline int8_t get_error_code() const { return err_; }

  bool found;

protected:
  // int8_t err_;
  uint8_t buffer[BMP3_LEN_CALIB_DATA];


  inline uint32_t to_24b(uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1] << 8| (uint32_t)b[2] << 16;
  }

  // datasheet pg 28
float compensate_temperature(const uint32_t uncomp_temp) {
  float pd1 = (float)uncomp_temp - calib.par_t1;
  float pd2 = pd1 * calib.par_t2;
  calib.t_lin = pd2 + (pd1 * pd1) * calib.par_t3;
  return calib.t_lin;
}

// datasheet pg 28
float compensate_pressure(const uint32_t uncomp_press) {
  float pd1 = calib.par_p6 * calib.t_lin;
  float pd2 = calib.par_p7 * (calib.t_lin * calib.t_lin);
  float pd3 = calib.par_p8 * (calib.t_lin * calib.t_lin * calib.t_lin);
  float po1 = calib.par_p5 + pd1 + pd2 + pd3;

  pd1 = calib.par_p2 * calib.t_lin;
  pd2 = calib.par_p3 * (calib.t_lin * calib.t_lin);
  pd3 = calib.par_p4 * (calib.t_lin * calib.t_lin * calib.t_lin);
  float po2 = (float)uncomp_press * (calib.par_p1 + pd1 + pd2 + pd3);

  pd1 = (float)uncomp_press * (float)uncomp_press;
  pd2 = calib.par_p9 + calib.par_p10 * calib.t_lin;
  pd3 = pd1*pd2;
  float pd4 = pd3 +((float)uncomp_press * (float)uncomp_press) * calib.par_p11;
  float comp_press = po1 + po2 + pd4;

  return comp_press;
}
  // void parse_calib_data(const uint8_t *reg_data) {}

  /**
  struct bmp3_reg_calib_data {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;
  };
  #define BMP3_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)
  */
  // inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {return ((uint16_t)msb << 8) | (uint16_t)lsb;}

  inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {return ((uint16_t)msb << 8) | (uint16_t)lsb;}
  // inline uint32_t CONCAT_BYTES(uint8_t mmsb, uint8_t msb, uint8_t lsb) {return (uint32_t)mmsb << 8)| (uint32_t)msb << 8) | (uint32_t)lsb;}

  bool get_calib_data() {
    bool ok = ReadRegisters(BMP3_REG_CALIB_DATA,BMP3_LEN_CALIB_DATA,buffer);
    if (!ok) return false;

    // being cast to signed integers
    // calib.par_t1 = to_16b(buffer[1], buffer[0]);
    // calib.par_t2 = to_16b(buffer[3], buffer[2]);
    // calib.par_t3 = (int8_t)buffer[4];
    // calib.par_p1 = (int16_t)to_16b(buffer[6], buffer[5]);
    // calib.par_p2 = (int16_t)to_16b(buffer[8], buffer[7]);
    // calib.par_p3 = (int8_t)buffer[9];
    // calib.par_p4 = (int8_t)buffer[10];
    // calib.par_p5 = to_16b(buffer[12], buffer[11]);
    // calib.par_p6 = to_16b(buffer[14], buffer[13]);
    // calib.par_p7 = (int8_t)buffer[15];
    // calib.par_p8 = (int8_t)buffer[16];
    // calib.par_p9 = (int16_t)to_16b(buffer[18], buffer[17]);
    // calib.par_p10 = (int8_t)buffer[19];
    // calib.par_p11 = (int8_t)buffer[20];


    calib.par_t1 = (float)to_16b(buffer[1], buffer[0]) / powf(2,-8);
    calib.par_t2 = (float)to_16b(buffer[3], buffer[2]) / powf(2,30);
    calib.par_t3 = (float)buffer[4] / powf(2,48);

    calib.par_p1 = (int16_t)to_16b(buffer[6], buffer[5]);
    calib.par_p2 = (int16_t)to_16b(buffer[8], buffer[7]);
    calib.par_p3 = (int8_t)buffer[9];
    calib.par_p4 = (int8_t)buffer[10];
    calib.par_p5 = to_16b(buffer[12], buffer[11]);
    calib.par_p6 = to_16b(buffer[14], buffer[13]);
    calib.par_p7 = (int8_t)buffer[15];
    calib.par_p8 = (int8_t)buffer[16];
    calib.par_p9 = (int16_t)to_16b(buffer[18], buffer[17]);
    calib.par_p10 = (int8_t)buffer[19];
    calib.par_p11 = (int8_t)buffer[20];

    Serial.println("got calib data");

    return true;
  }

  bool sleep() {
    // int8_t rslt;
    uint8_t reg_addr = BMP3_REG_PWR_CTRL;

    /* Temporary variable to store the value read from op-mode register */
    uint8_t op_mode_reg_val;

    // rslt = bmp3_get_regs(BMP3_REG_PWR_CTRL, &op_mode_reg_val, 1 /*, dev*/);
    op_mode_reg_val = readRegister(BMP3_REG_PWR_CTRL);

    // if (rslt == BMP3_OK) {
      /* Set the power mode */
      // op_mode_reg_val = op_mode_reg_val & (~(BMP3_OP_MODE_MSK));
      op_mode_reg_val = op_mode_reg_val & 0x03; // keep bits 0-1

      /* Write the power mode in the register */
      return writeRegister(BMP3_REG_PWR_CTRL, op_mode_reg_val);
    // }

    // return true;
  }

  // int8_t bmp3_get_op_mode(uint8_t *op_mode) {} // value?
  // int8_t bmp3_set_op_mode(uint8_t curr_mode) {
  //   // int8_t rslt;
  //   // uint8_t last_set_mode;

  //   /* Check for null pointer in the device structure */
  //   // rslt = null_ptr_check(dev);

  //     // uint8_t curr_mode = settings->op_mode;

  //     // rslt = bmp3_get_op_mode(&last_set_mode /*, dev*/);
  //     bool ok;

  //     // get opmode
  //   // uint8_t last_set_mode = readRegister(BMP3_REG_PWR_CTRL);

  //   /* Assign the power mode in the device structure */
  //   // last_set_mode = BMP3_GET_BITS(last_set_mode, BMP3_OP_MODE);

  //     /* If the sensor is not in sleep mode put the device to sleep
  //     * mode */
  //     // if (last_set_mode != BMP3_MODE_SLEEP ) {
  //       /* Device should be put to sleep before transiting to
  //       * forced mode or normal mode */
  //       ok = put_device_to_sleep();
  //       delay(5);
  //     // }

  //     /* Set the power mode */
  //     // if (rslt == BMP3_OK) {
  //       if (curr_mode == BMP3_MODE_NORMAL) {
  //         ok = set_normal_mode();
  //       }
  //       // else if (curr_mode == BMP3_MODE_FORCED) {
  //       //   /* Set forced mode */
  //       //   ok = write_power_mode();
  //       // }
  //     // }


  //   return true;

  // }

  // bool set_normal_mode() {
  //   // int8_t rslt;
  //   // uint8_t conf_err_status;

  //   // rslt = validate_normal_mode_settings(settings /*, dev*/);

  //   /* If OSR and ODR settings are proper then write the power mode */
  //   // if (rslt == BMP3_OK) {
  //     bool ok = write_power_mode(BMP3_MODE_NORMAL);

  //   //   /* check for configuration error */
  //   //   // if (rslt == BMP3_OK) {
  //   //     /* Read the configuration error status */
  //   //     // ok = bmp3_get_regs(BMP3_REG_ERR, &conf_err_status, 1 /*, dev*/);
  //   //     ok = readRegisters(BMP3_REG_ERR, &conf_err_status, 1);

  //   //     /* Check if conf. error flag is set */
  //   //     if (ok) {
  //   //       if (conf_err_status & BMP3_ERR_CONF) {
  //   //         /* OSR and ODR configuration is not proper */
  //   //         rslt = BMP3_E_CONFIGURATION_ERR;
  //   //       }
  //   //     }
  //   //   // }
  //   // // }

  //   return true;
  // }

  // inline SET_BITS(reg_data, bitname, data) {
  //   return((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))
  // }

  // constexpr uint8_t MODE_NORMAL = 0x03;
  bool setPowerMode(uint8_t mode=MODE_NORMAL) {
    // int8_t rslt;
    // uint8_t reg_addr = BMP3_REG_PWR_CTRL;
    // uint8_t op_mode = settings->op_mode;

    /* Temporary variable to store the value read from op-mode register */
    // uint8_t op_mode_reg_val;

    // /* Read the power mode register */
    // // rslt = bmp3_get_regs(reg_addr, &op_mode_reg_val, 1 /*, dev*/);
    // bool ok = readRegisters(BMP3_REG_PWR_CTRL, 1, &op_mode_reg_val);
    // if (!ok) return false;

    // /* Set the power mode */
    // if (ok) {
    //   op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);

    //   /* Write the power mode in the register */
    //   // rslt = bmp3_set_regs(&reg_addr, &op_mode_reg_val, 1 /*, dev*/);
    //   ok = writeRegister(BMP3_REG_PWR_CTRL, op_mode_reg_val);
    //   if (!ok) return false;
    // }
    bool ok;

    ok = sleep();
    delay(5);

    constexpr uint8_t PRESS_EN = 0x01;
    constexpr uint8_t TEMP_EN = 0x02;

    uint8_t val = (mode << 4) | TEMP_EN | PRESS_EN;
    ok = writeRegister(BMP3_REG_PWR_CTRL, val);
    if (!ok) return false;

    return true;
  }

  bool soft_reset() {
    bool ok;

    // Check for command ready status
    uint8_t cmd_rdy_status = readRegister(BMP3_REG_SENS_STATUS);

    // Device is ready to accept new command
    if (cmd_rdy_status & BMP3_CMD_RDY) {
      // Write the soft reset command in the sensor
      ok = WriteRegister(BMP3_REG_CMD, BMP3_SOFT_RESET);
      if (!ok) return false;

        delay(2);

        // Read for command error status
        if (readRegister(BMP3_REG_ERR) & BMP3_REG_CMD) return false;

        return true;

    }
    return false;
  }

  bool setOverSampling(uint8_t posr, uint8_t tosr) {
    uint8_t val = (tosr << 3) | posr;
    return writeRegister(BMP3_REG_OSR, val);
  }

  bool setODR(uint8_t odr) {
    return writeRegister(BMP3_REG_ODR, odr);
  }

  bool setIIR(uint8_t iir) {
    constexpr uint8_t BMP3_REG_IIR_FILTER = 0x1F;
    uint8_t val = iir << 1;
    return writeRegister(BMP3_REG_IIR_FILTER, val);
  }

  bool setInterrupt(uint8_t drdy_en, uint8_t int_level) {
    // int_level: 1 = active high
    uint8_t val = (drdy_en << 6) | (int_level << 1);
    return writeRegister(BMP3_REG_INT_CTRL, val);
  }

  bool bmp3_set_sensor_settings() {
    // int8_t rslt = BMP3_OK;

    bool ok;

    ok = setPowerMode();
    if (!ok) return false;

    // constexpr uint8_t PRESS_EN = 0x01;
    // constexpr uint8_t TEMP_EN = 0x02;
    // constexpr uint8_t MODE_NORMAL = 0x03;

    // if (settings != NULL) {

      // if (are_settings_changed(BMP3_POWER_CNTL, desired_settings)) {
        /* Set the power control settings */
        // rslt = set_pwr_ctrl_settings(desired_settings, settings /*, dev*/);
        // uint8_t val = (MODE_NORMAL << 4) | TEMP_EN | PRESS_EN;
        // bool ok = writeRegister(BMP3_REG_PWR_CTRL, val);
        // if (!ok) return false;
      // }

      // if (are_settings_changed(BMP3_ODR_FILTER, desired_settings)) {
        /* Set the over sampling, ODR and filter settings */
        // rslt = set_odr_filter_settings(desired_settings, settings /*, dev*/);
        // uint8_t vals[4];
        // ok = readRegisters(BMP3_REG_OSR, 4, vals);

        uint8_t posr = BMP3_OVERSAMPLING_2X;
        uint8_t tosr = BMP3_OVERSAMPLING_1X;
        ok = setOverSampling(posr, tosr);
      //   uint8_t val = (tosr << 3) | posr;
      //   ok = writeRegister(BMP3_REG_OSR, val);
      if (!ok) return false;

        // val = BMP3_ODR_100_HZ;
        // ok = writeRegister(BMP3_REG_ODR, val);
        ok = setODR(BMP3_ODR_100_HZ);
      if (!ok) return false;
        // val[2] = 0; // no 0x1e reg

        // constexpr uint8_t BMP3_REG_IIR_FILTER = 0x1F;
        ok = setIIR(BMP3_IIR_FILTER_COEFF_1);
      if (!ok) return false;
        // val = BMP3_IIR_FILTER_COEFF_1 << 1;
        // ok = writeRegister(BMP3_REG_IIR_FILTER, val);
      // }

      // if (are_settings_changed(BMP3_INT_CTRL, desired_settings)) {
        /* Set the interrupt control settings */
        // rslt = set_int_ctrl_settings(desired_settings, settings /*, dev*/);

        // drdy enable = 1 << 6
        // non-latch = 0
        // active-high = 1 << 1
        // pin push/pull = 0
        // val = (1 << 6) | (1 << 1);
        // ok = writeRegister(BMP3_REG_INT_CTRL, val);
        ok = setInterrupt(1,1);
      if (!ok) return false;
      // }

      // if (are_settings_changed(BMP3_ADV_SETT, desired_settings)) {
        /* Set the advance settings */
        // rslt = set_advance_settings(desired_settings, settings /*, dev*/);
      // }
    // } else {
    //   rslt = BMP3_E_NULL_PTR;
    // }

    return true;
  }

  struct bmp3_reg_calib_data {
    float par_t1;
    float par_t2;
    float par_t3;

    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    float t_lin; // was int64_t??
  } calib;

};

} // namespace gci
