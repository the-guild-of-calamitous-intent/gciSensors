
#include "bmp3.hpp"
// #if defined(ARDUINO)
// #include "Wire.h"
// #include <Arduino.h>
// #endif
// #include <cstddef>
// #include <cstdint>

using namespace BMP390;


gciBMP390::gciBMP390(TwoWire *i2c, const uint8_t addr) : Sensor(i2c, addr) {
  found = false;
}

bool gciBMP390::init() {
  uint8_t whoammi = readRegister(BMP3_REG_CHIP_ID);
  if ((whoammi == BMP3_CHIP_ID) || (whoammi == BMP390_CHIP_ID)) {
    soft_reset();
    get_calib_data();
  }
  // err_ = bmp3_init(this->addr);
  // if (err_ != BMP3_OK) {
  //   return false;
  // }
  // Set defaults
  req_settings_.press_en = BMP3_ENABLE;
  req_settings_.temp_en = BMP3_ENABLE;
  req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_2X; // was 8
  req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
  req_settings_.odr_filter.odr = BMP3_ODR_100_HZ; // was 50
  req_settings_.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_1;
  req_settings_.int_settings.drdy_en = BMP3_ENABLE;
  req_settings_.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
  req_settings_.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
  req_settings_.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
  uint32_t settings_select_ = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                     BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR |
                     BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_LATCH |
                     BMP3_SEL_DRDY_EN;
  err_ = bmp3_set_sensor_settings(settings_select_, &req_settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }
  // Get the sensor config
  // Serial.println("get settings");
  err_ = bmp3_get_sensor_settings(&settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }
  // Set the power mode
  req_settings_.op_mode = BMP3_MODE_NORMAL;
  err_ = bmp3_set_op_mode(&req_settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }
  // Get the power mode
  // Serial.println("get mode");
  // uint8_t power_mode_;
  // err_ = bmp3_get_op_mode(&power_mode_ );
  // if (err_ != BMP3_OK) {
  //   return false;
  // }

  // OsMode os_mode_;
  // os_mode_ = OS_MODE_PRES_8X_TEMP_1X;
  settings_.op_mode = power_mode_;

  found = true;
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
bool gciBMP390::setOsMode(const OsMode mode) {
  switch (mode) {
  case OS_MODE_PRES_1X_TEMP_1X:
    req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_1X;
    req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings_.odr_filter.odr = BMP3_ODR_200_HZ;
    break;

  case OS_MODE_PRES_2X_TEMP_1X:
    req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings_.odr_filter.odr = BMP3_ODR_100_HZ;
    break;

  case OS_MODE_PRES_4X_TEMP_1X:
    req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings_.odr_filter.odr = BMP3_ODR_100_HZ; // was 50
    break;

  case OS_MODE_PRES_8X_TEMP_1X:
    req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings_.odr_filter.odr = BMP3_ODR_50_HZ;
    break;

  case OS_MODE_PRES_16X_TEMP_2X:
    req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_16X;
    req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    req_settings_.odr_filter.odr = BMP3_ODR_25_HZ;
    break;

  case OS_MODE_PRES_32X_TEMP_2X:
    req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_32X;
    req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    req_settings_.odr_filter.odr = BMP3_ODR_12_5_HZ;
    break;
  }

  uint32_t settings_select_ = BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
  err_ = bmp3_set_sensor_settings(settings_select_, &req_settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }

  // Get the sensor config
  err_ = bmp3_get_sensor_settings(&settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }

  return true;
}

bool gciBMP390::setFilterCoef(const FilterCoef val) {
  req_settings_ = settings_;
  req_settings_.odr_filter.iir_filter = val;
  uint32_t settings_select_ = BMP3_SEL_IIR_FILTER;
  err_ = bmp3_set_sensor_settings(settings_select_, &req_settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }
  // Get the sensor config
  err_ = bmp3_get_sensor_settings(&settings_ );
  if (err_ != BMP3_OK) {
    return false;
  }
  return true;
}

pt_t gciBMP390::read() {
  pt_t ret;
  ret.ok = false;
  bmp3_status status_;
  // err_ = bmp3_get_status(&status_ );
  // if (err_ != BMP3_OK) {
  //   return ret;
  // }
  if (1){ //(status_.intr.drdy) {
    bmp3_data data_;
    err_ = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data_ );
    if (err_ != BMP3_OK) {
      return ret;
    }
    // why?
    // err_ = bmp3_get_status(&status_ );
    // if (err_ != BMP3_OK) {
    //   return ret;
    // }
    // if you get to here, all is good
    ret.ok = true;
    ret.press = data_.pressure;
    ret.temp = data_.temperature;
    return ret;
  }
  return ret;
}

// #define BOB
// #pragma message "=>> hi" BMP3_FLOAT_COMPENSATION BOB

///////////////////////////////////////////////////////////////////////////////////////////

#if 0

// read/write

// // WriteRegister(const uint8_t reg, const uint8_t data)
// int8_t I2cWriteRegisters(uint8_t reg, const uint8_t *data,
//                                     uint32_t len, void *intf) {
//   /* NULL pointer check */
//   if ((!data) || (!intf)) {
//     return BMP3_E_NULL_PTR;
//   }
//   /* Check length */
//   if (len == 0) {
//     return BMP3_E_INVALID_LEN;
//   }
//   I2cIntf *iface = static_cast<I2cIntf *>(intf);
//   iface->i2c->beginTransmission(iface->addr);
//   if (iface->i2c->write(reg) != 1) {
//     return BMP3_E_COMM_FAIL;
//   }
//   if (iface->i2c->write(data, len) != len) {
//     return BMP3_E_COMM_FAIL;
//   }
//   iface->i2c->endTransmission();
//   return BMP3_OK;
// }

// // ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t *const data)
// int8_t I2cReadRegisters(uint8_t reg, uint8_t *data, uint32_t len,
//                                    void *intf) {
//   /* NULL pointer check */
//   if ((!data) || (!intf)) {
//     return BMP3_E_NULL_PTR;
//   }
//   /* Check length */
//   if (len == 0) {
//     return BMP3_E_INVALID_LEN;
//   }
//   I2cIntf *iface = static_cast<I2cIntf *>(intf);
//   iface->i2c->beginTransmission(iface->addr);
//   if (iface->i2c->write(reg) != 1) {
//     return BMP3_E_COMM_FAIL;
//   }
//   iface->i2c->endTransmission(false);
//   if (iface->i2c->requestFrom(iface->addr, len) != len) {
//     return BMP3_E_COMM_FAIL;
//   }
//   for (size_t i = 0; i < len; i++) {
//     data[i] = iface->i2c->read();
//   }
//   return BMP3_OK;
// }

/////////////////////

// int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data,
//                      uint32_t len /*, struct bmp3_dev *dev*/) {
//   // int8_t rslt;
//   // uint32_t idx;

//   // /* Check for null pointer in the device structure */
//   // rslt = null_ptr_check(dev);

//   /* Proceed if null check is fine */
//   // if ((rslt == BMP3_OK) && (reg_data != NULL)) {
//     uint32_t temp_len = len; // + dev->dummy_byte;
//     uint8_t temp_buff[len /*+ dev->dummy_byte*/];

//     /* If interface selected is SPI */
//     // if (dev->intf != BMP3_I2C_INTF)
//     // {
//     //     reg_addr = reg_addr | 0x80;

//     //     /* Read the data from the register */
//     //     dev->intf_rslt = dev->read(reg_addr, temp_buff, temp_len,
//     //     dev->intf_ptr); for (idx = 0; idx < len; idx++)
//     //     {
//     //         reg_data[idx] = temp_buff[idx + dev->dummy_byte];
//     //     }
//     // }
//     // else
//     // {
//     /* Read the data using I2C */
//     dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);
//     // }

//   //   /* Check for communication error */
//   //   if (dev->intf_rslt != BMP3_INTF_RET_SUCCESS) {
//   //     rslt = BMP3_E_COMM_FAIL;
//   //   }
//   // } else {
//   //   rslt = BMP3_E_NULL_PTR;
//   // }

//   // return rslt;
// }

///////////////////////////////

// BMP3_FLOAT_COMPENSATION is false/empty

// static int8_t get_calib_data(struct bmp3_dev *dev) {
//   int8_t rslt;
//   uint8_t reg_addr = BMP3_REG_CALIB_DATA;

//   /* Array to store calibration data */
//   uint8_t calib_data[BMP3_LEN_CALIB_DATA] = {0};

//   /* Read the calibration data from the sensor */
//   rslt = bmp3_get_regs(reg_addr, calib_data, BMP3_LEN_CALIB_DATA /*, dev*/);

//   /* Parse calibration data and store it in device structure */
//   parse_calib_data(calib_data /*, dev*/);

//   return rslt;
// }


// static void
// parse_calib_data(const uint8_t *reg_data /*, struct bmp3_dev *dev*/) {
//   /* Temporary variable to store the aligned trim data */
//   struct bmp3_reg_calib_data *reg_calib_data = &dev->calib_data.reg_calib_data;

//   reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
//   reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
//   reg_calib_data->par_t3 = (int8_t)reg_data[4];
//   reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
//   reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
//   reg_calib_data->par_p3 = (int8_t)reg_data[9];
//   reg_calib_data->par_p4 = (int8_t)reg_data[10];
//   reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
//   reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
//   reg_calib_data->par_p7 = (int8_t)reg_data[15];
//   reg_calib_data->par_p8 = (int8_t)reg_data[16];
//   reg_calib_data->par_p9 =
//       (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
//   reg_calib_data->par_p10 = (int8_t)reg_data[19];
//   reg_calib_data->par_p11 = (int8_t)reg_data[20];
// }

///////////////////////////////

// // datasheet pg 28
// float compensate_temperature(const uint32_t uncomp_temp) {
//   float pd1 = (float)(uncomp_temp - calib.par_t1);
//   float pd2 = (float)(pd1 * calib.par_t2);
//   calib.t_lin = pd2 + (pd1*pd1) * calib.par_t3;
//   return calib.t_lin;
// }

// // datasheet pg 28
// float compensate_pressure(uint32_t uncomp_press) {
//   float pd1 = calib.par_p6 * calib.t_lin;
//   float pd2 = calib.par_p7 * (calib.t_lin * calib.t_lin);
//   float pd3 = calib.par_p8 * (calib.t_lin * calib.t_lin * calib.t_lin);
//   float po1 = calib.par_p5 + pd1 + pd2 + pd3;

//   pd1 = calib.par_p2 * calib.t_lin;
//   pd2 = calib.par_p3 * (calib.t_lin * calib.t_lin);
//   pd3 = calib.par_p4 * (calib.t_lin * calib.t_lin * calib.t_lin);
//   float po2 = (float)uncomp_press * (calib.par_p1 + pd1 +pd2 + pd3);

//   pd1 = (float)uncomp_press * (float)uncomp_press;
//   pd2 = calib.par_p9 + calib.par_p10 * calib.t_lin;
//   pd3 = pd1*pd2;
//   float pd4 = pd3 +((float)uncomp_press * (float)uncomp_press) * calib.par_p11;
//   float comp_press = po1 + po2 + pd4;

//   return comp_press;
// }

// static int8_t compensate_data(uint8_t sensor_comp,
//                               const struct bmp3_uncomp_data *uncomp_data,
//                               struct bmp3_data *comp_data,
//                               struct bmp3_calib_data *calib_data) {
//   // int8_t rslt = BMP3_OK;

//   // if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
//     /* If pressure and temperature component is selected */
//     // if (sensor_comp == BMP3_PRESS_TEMP) {
//       /*
//        * NOTE : Temperature compensation must be done first.
//        * Followed by pressure compensation
//        * Compensated temperature updated in calib structure,
//        * is needed for pressure calculation
//        */

//       /* Compensate pressure and temperature data */
//       rslt = compensate_temperature(&comp_data->temperature, uncomp_data,
//                                     calib_data);

//       // if (rslt == BMP3_OK) {
//         rslt =
//             compensate_pressure(&comp_data->pressure, uncomp_data, calib_data);
//       // }
//   //   } else if (sensor_comp == BMP3_PRESS) {
//   //     /*
//   //      * NOTE : Temperature compensation must be done first.
//   //      * Followed by pressure compensation
//   //      * Compensated temperature updated in calib structure,
//   //      * is needed for pressure calculation.
//   //      * As only pressure is enabled in 'sensor_comp', after calculating
//   //      * compensated temperature, assign it to zero.
//   //      */
//   //     (void)compensate_temperature(&comp_data->temperature, uncomp_data,
//   //                                  calib_data);
//   //     comp_data->temperature = 0;

//   //     /* Compensate the pressure data */
//   //     rslt = compensate_pressure(&comp_data->pressure, uncomp_data, calib_data);
//   //   } else if (sensor_comp == BMP3_TEMP) {
//   //     /* Compensate the temperature data */
//   //     rslt = compensate_temperature(&comp_data->temperature, uncomp_data,
//   //                                   calib_data);

//   //     /*
//   //      * As only temperature is enabled in 'sensor_comp'
//   //      * make compensated pressure as zero
//   //      */
//   //     comp_data->pressure = 0;
//   //   } else {
//   //     comp_data->pressure = 0;
//   //     comp_data->temperature = 0;
//   //   }
//   // } else {
//   //   rslt = BMP3_E_NULL_PTR;
//   // }

//   return rslt;
// }

// static void parse_sensor_data(const uint8_t *reg_data,
//                               struct bmp3_uncomp_data *uncomp_data) {
//   /* Temporary variables to store the sensor data */
//   uint32_t data_xlsb;
//   uint32_t data_lsb;
//   uint32_t data_msb;

//   /* Store the parsed register values for pressure data */
//   data_xlsb = (uint32_t)reg_data[0];
//   data_lsb = (uint32_t)reg_data[1] << 8;
//   data_msb = (uint32_t)reg_data[2] << 16;
//   uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

//   /* Store the parsed register values for temperature data */
//   data_xlsb = (uint32_t)reg_data[3];
//   data_lsb = (uint32_t)reg_data[4] << 8;
//   data_msb = (uint32_t)reg_data[5] << 16;
//   uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
// }

// int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) {
//   int8_t rslt;
//   uint32_t idx;

//   /* Check for null pointer in the device structure */
//   // rslt = null_ptr_check(dev);

//   /* Proceed if null check is fine */
//   // if ((rslt == BMP3_OK) && (reg_data != NULL)) {
//     // uint32_t temp_len = len; // + dev->dummy_byte; not used
//     // uint8_t temp_buff[len /*+ dev->dummy_byte*/];

//     /* Read the data using I2C */
//     dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);
//     // }

//     /* Check for communication error */
//     // if (dev->intf_rslt != BMP3_INTF_RET_SUCCESS) {
//     //   rslt = BMP3_E_COMM_FAIL;
//     // }
//   // } else {
//   //   rslt = BMP3_E_NULL_PTR;
//   // }

//   // return rslt;
// }

// int8_t bmp3_get_sensor_data(uint8_t sensor_comp,
//                      struct bmp3_data *comp_data) {
//   int8_t rslt;

//   /* Array to store the pressure and temperature data read from
//    * the sensor */
//   uint8_t reg_data[BMP3_LEN_P_T_DATA] = {0};
//   struct bmp3_uncomp_data uncomp_data = {0};

//   if (comp_data != NULL) {
//     /* Read the pressure and temperature data from the sensor */
//     rslt = bmp3_get_regs(BMP3_REG_DATA, reg_data, BMP3_LEN_P_T_DATA /*, dev*/);

//     if (rslt == BMP3_OK) {
//       /* Parse the read data from the sensor */
//       parse_sensor_data(reg_data, &uncomp_data);

//       /* Compensate the pressure/temperature/both data read
//        * from the sensor */
//       rslt = compensate_data(sensor_comp, &uncomp_data, comp_data,
//                              &dev->calib_data);
//     }
//   } else {
//     rslt = BMP3_E_NULL_PTR;
//   }

//   return rslt;
// }

int8_t bmp3_set_sensor_settings(
    uint32_t desired_settings,
    struct bmp3_settings *settings /*, struct bmp3_dev *dev*/) {
  int8_t rslt = BMP3_OK;

  if (settings != NULL) {

    if (are_settings_changed(BMP3_POWER_CNTL, desired_settings)) {
      /* Set the power control settings */
      rslt = set_pwr_ctrl_settings(desired_settings, settings /*, dev*/);
    }

    if (are_settings_changed(BMP3_ODR_FILTER, desired_settings)) {
      /* Set the over sampling, ODR and filter settings */
      rslt = set_odr_filter_settings(desired_settings, settings /*, dev*/);
    }

    if (are_settings_changed(BMP3_INT_CTRL, desired_settings)) {
      /* Set the interrupt control settings */
      rslt = set_int_ctrl_settings(desired_settings, settings /*, dev*/);
    }

    if (are_settings_changed(BMP3_ADV_SETT, desired_settings)) {
      /* Set the advance settings */
      rslt = set_advance_settings(desired_settings, settings /*, dev*/);
    }
  } else {
    rslt = BMP3_E_NULL_PTR;
  }

  return rslt;
}

int8_t bmp3_get_sensor_settings(
    struct bmp3_settings *settings /*, struct bmp3_dev *dev*/) {
  int8_t rslt;
  uint8_t settings_data[BMP3_LEN_GEN_SETT];

  if (settings != NULL) {
    rslt = bmp3_get_regs(BMP3_REG_INT_CTRL, settings_data,
                         BMP3_LEN_GEN_SETT /*, dev*/);

    if (rslt == BMP3_OK) {
      /* Parse the settings data */
      parse_sett_data(settings_data, settings);
    }
  } else {
    rslt = BMP3_E_NULL_PTR;
  }

  return rslt;
}

#endif
