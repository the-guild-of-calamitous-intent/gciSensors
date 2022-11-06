
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
  bool ok;

  if (!(readRegister(BMP3_REG_CHIP_ID) == BMP390_CHIP_ID)) return false;

  // if ((whoammi == BMP3_CHIP_ID) || (whoammi == BMP390_CHIP_ID)) {
  ok = soft_reset();
  if (!ok) return false;
  ok = get_calib_data();
  if (!ok) return false;
  Serial.println("reset ... got calib ...");
  // }
  // else return false;
  // err_ = bmp3_init(this->addr);
  // if (err_ != BMP3_OK) {
  //   return false;
  // }
  // Set defaults
  // req_settings_.press_en = BMP3_ENABLE;
  // req_settings_.temp_en = BMP3_ENABLE;
  // req_settings_.odr_filter.press_os = BMP3_OVERSAMPLING_2X; // was 8
  // req_settings_.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
  // req_settings_.odr_filter.odr = BMP3_ODR_100_HZ; // was 50
  // req_settings_.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_1;
  // req_settings_.int_settings.drdy_en = BMP3_ENABLE;
  // req_settings_.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
  // req_settings_.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
  // req_settings_.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
  // uint32_t settings_select_ = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
  //                    BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR |
  //                    BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_LATCH |
  //                    BMP3_SEL_DRDY_EN;
  ok = bmp3_set_sensor_settings();
  if (!ok) return false;
  // Get the sensor config
  // Serial.println("get settings");
  // ok = bmp3_get_sensor_settings(&settings_ );
  // if (!ok) return false;
  // Set the power mode
  // req_settings_.op_mode = BMP3_MODE_NORMAL;
  // ok = bmp3_set_op_mode(BMP3_MODE_NORMAL);
  // if (!ok) return false;

  // Get the power mode
  // Serial.println("get mode");
  // uint8_t power_mode_;
  // err_ = bmp3_get_op_mode(&power_mode_ );
  // if (err_ != BMP3_OK) {
  //   return false;
  // }

  // OsMode os_mode_;
  // os_mode_ = OS_MODE_PRES_8X_TEMP_1X;
  // settings_.op_mode = power_mode_;

  // found = true;
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
  uint8_t press_os, temp_os, odr;

  Serial.println("fixme");

  switch (mode) {
  case OS_MODE_PRES_1X_TEMP_1X:
    press_os = BMP3_OVERSAMPLING_1X;
    temp_os = BMP3_OVERSAMPLING_1X;
    odr = BMP3_ODR_200_HZ;
    break;

  case OS_MODE_PRES_2X_TEMP_1X:
    press_os = BMP3_OVERSAMPLING_2X;
    temp_os = BMP3_OVERSAMPLING_1X;
    odr = BMP3_ODR_100_HZ;
    break;

  case OS_MODE_PRES_4X_TEMP_1X:
    press_os = BMP3_OVERSAMPLING_4X;
    temp_os = BMP3_OVERSAMPLING_1X;
    odr = BMP3_ODR_100_HZ; // was 50
    break;

  case OS_MODE_PRES_8X_TEMP_1X:
    press_os = BMP3_OVERSAMPLING_8X;
    temp_os = BMP3_OVERSAMPLING_1X;
    odr = BMP3_ODR_50_HZ;
    break;

  case OS_MODE_PRES_16X_TEMP_2X:
    press_os = BMP3_OVERSAMPLING_16X;
    temp_os = BMP3_OVERSAMPLING_2X;
    odr = BMP3_ODR_25_HZ;
    break;

  case OS_MODE_PRES_32X_TEMP_2X:
    press_os = BMP3_OVERSAMPLING_32X;
    temp_os = BMP3_OVERSAMPLING_2X;
    odr = BMP3_ODR_12_5_HZ;
    break;
  }

  // FIXME: make it set things right

  // uint32_t settings_select_ = BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
  // err_ = bmp3_set_sensor_settings(settings_select_, &req_settings_ );
  // if (err_ != BMP3_OK) {
  //   return false;
  // }

  // Get the sensor config
  // err_ = bmp3_get_sensor_settings(&settings_ );
  // if (err_ != BMP3_OK) {
  //   return false;
  // }

  return true;
}

bool gciBMP390::setFilterCoef(const FilterCoef val) {
  Serial.println("fixme");
  // req_settings_ = settings_;
  // req_settings_.odr_filter.iir_filter = val;
  // uint32_t settings_select_ = BMP3_SEL_IIR_FILTER;
  // err_ = bmp3_set_sensor_settings(settings_select_, &req_settings_ );
  // if (err_ != BMP3_OK) {
  //   return false;
  // }
  // Get the sensor config
  // err_ = bmp3_get_sensor_settings(&settings_ );
  // if (err_ != BMP3_OK) {
  //   return false;
  // }
  return true;
}

pt_t gciBMP390::read() {
  pt_t ret;
  ret.ok = false;
  // bmp3_status status_;
  // err_ = bmp3_get_status(&status_ );
  // if (err_ != BMP3_OK) {
  //   return ret;
  // }
  // if (1){ //(status_.intr.drdy) {
    // bmp3_data data_;
    // bool ok = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data_ );

    bool ok = ReadRegisters(BMP3_REG_DATA, BMP3_LEN_P_T_DATA, buffer);
    if (!ok) return ret;

    uint32_t press = to_24b(&buffer[0]);
    uint32_t temp = to_24b(&buffer[3]);

    Serial.println("good read");

    ret.ok = true;
    ret.temp  = compensate_temperature(temp); // do temp 1st!!!
    ret.press = compensate_pressure(press);
    return ret;
  // }
  return ret;
}
