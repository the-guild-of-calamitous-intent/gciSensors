
#include "bmp3.hpp"

using namespace BMP390;


gciBMP390::gciBMP390(TwoWire *i2c, const uint8_t addr) : Sensor(i2c, addr) {
  found = false;
}

bool gciBMP390::init() {
  err = bmp3_init(this->addr);
  if (err != BMP3_OK) {
    Serial.println("init failed ...");
    return false;
  }
  // Set defaults
  req_settings.press_en = BMP3_ENABLE;
  req_settings.temp_en = BMP3_ENABLE;
  req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X; // was 8
  req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
  req_settings.odr_filter.odr = BMP3_ODR_100_HZ; // was 50
  req_settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_1;
  req_settings.int_settings.drdy_en = BMP3_ENABLE;
  req_settings.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
  req_settings.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
  req_settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
  uint32_t settings_select = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                     BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR |
                     BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_LATCH |
                     BMP3_SEL_DRDY_EN;
  err = bmp3_set_sensor_settings(settings_select, &req_settings );
  if (err != BMP3_OK) {
    return false;
  }
  // Get the sensor config
  // Serial.println("get settings");
  err = bmp3_get_sensor_settings(&settings );
  if (err != BMP3_OK) {
    return false;
  }
  // Set the power mode
  req_settings.op_mode = BMP3_MODE_NORMAL;
  err = bmp3_set_op_mode(&req_settings );
  if (err != BMP3_OK) {
    return false;
  }
  // Get the power mode
  // Serial.println("get mode");
  uint8_t power_mode_;
  err = bmp3_get_op_mode(&power_mode_ );
  if (err != BMP3_OK) {
    return false;
  }

  // OsMode os_mode_;
  // os_mode_ = OS_MODE_PRES_8X_TEMP_1X;
  settings.op_mode = power_mode_;

  Serial.println("bmp3::init done ...");

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
    req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_1X;
    req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings.odr_filter.odr = BMP3_ODR_200_HZ;
    break;

  case OS_MODE_PRES_2X_TEMP_1X:
    req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings.odr_filter.odr = BMP3_ODR_100_HZ;
    break;

  case OS_MODE_PRES_4X_TEMP_1X:
    req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings.odr_filter.odr = BMP3_ODR_100_HZ; // was 50
    break;

  case OS_MODE_PRES_8X_TEMP_1X:
    req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_1X;
    req_settings.odr_filter.odr = BMP3_ODR_50_HZ;
    break;

  case OS_MODE_PRES_16X_TEMP_2X:
    req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_16X;
    req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    req_settings.odr_filter.odr = BMP3_ODR_25_HZ;
    break;

  case OS_MODE_PRES_32X_TEMP_2X:
    req_settings.odr_filter.press_os = BMP3_OVERSAMPLING_32X;
    req_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    req_settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
    break;
  }

  uint32_t settings_select = BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
  err = bmp3_set_sensor_settings(settings_select, &req_settings );
  if (err != BMP3_OK) {
    return false;
  }

  // Get the sensor config
  err = bmp3_get_sensor_settings(&settings );
  if (err != BMP3_OK) {
    return false;
  }

  return true;
}

bool gciBMP390::setFilterCoef(const FilterCoef val) {
  req_settings = settings;
  req_settings.odr_filter.iir_filter = val;
  uint32_t settings_select = BMP3_SEL_IIR_FILTER;
  err = bmp3_set_sensor_settings(settings_select, &req_settings );
  if (err != BMP3_OK) {
    return false;
  }
  // Get the sensor config
  err = bmp3_get_sensor_settings(&settings );
  if (err != BMP3_OK) {
    return false;
  }
  return true;
}

pt_t gciBMP390::read() {
  pt_t ret;
  ret.ok = false;
  bmp3_status status_;
  // err = bmp3_get_status(&status_ );
  // if (err != BMP3_OK) {
  //   return ret;
  // }
  if (1){ //(status_.intr.drdy) {
    bmp3_data data_;
    err = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data_ );
    if (err != BMP3_OK) {
      return ret;
    }
    // why?
    // err = bmp3_get_status(&status_ );
    // if (err != BMP3_OK) {
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
