
#include "bmp3.hpp"

using namespace BMP390;


gciBMP390::gciBMP390(TwoWire *i2c, const uint8_t addr) : Sensor(i2c, addr) {
  found = false;
}

bool gciBMP390::init() {
  bool ok;

  if (!(readRegister(BMP3_REG_CHIP_ID) == BMP390_CHIP_ID)) return false;

  ok = soft_reset();
  if (!ok) return false;
  ok = get_calib_data();
  if (!ok) return false;
  Serial.println("reset ... got calib ...");

  ok = setPowerMode();
  if (!ok) return false;

  uint8_t posr = BMP3_OVERSAMPLING_2X;
  uint8_t tosr = BMP3_OVERSAMPLING_1X;
  ok = setOverSampling(posr, tosr);
  if (!ok) return false;

  ok = setODR(BMP3_ODR_100_HZ);
  if (!ok) return false;

  ok = setIIR(BMP3_IIR_FILTER_COEFF_1);
  if (!ok) return false;

  // drdy enable = 1 << 6
  // non-latch = 0
  // active-high = 1 << 1
  // pin push/pull = 0
  // val = (1 << 6) | (1 << 1);
  // ok = writeRegister(BMP3_REG_INT_CTRL, val);
  ok = setInterrupt(1,1);
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

  bool ok = setOverSampling(press_os, temp_os);
  if (!ok) return false;
  ok = setODR(odr);
  if (!ok) return false;

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


  bool gciBMP390::setOverSampling(uint8_t posr, uint8_t tosr) {
    uint8_t val = (tosr << 3) | posr;
    return writeRegister(BMP3_REG_OSR, val);
  }

  bool gciBMP390::setODR(uint8_t odr) {
    return writeRegister(BMP3_REG_ODR, odr);
  }

  bool gciBMP390::setIIR(uint8_t iir) {
    constexpr uint8_t BMP3_REG_IIR_FILTER = 0x1F;
    uint8_t val = iir << 1;
    return writeRegister(BMP3_REG_IIR_FILTER, val);
  }

  bool gciBMP390::setInterrupt(uint8_t drdy_en, uint8_t int_level) {
    // int_level: 1 = active high
    uint8_t val = (drdy_en << 6) | (int_level << 1);
    return writeRegister(BMP3_REG_INT_CTRL, val);
  }

// bool gciBMP390::setFilterCoef(const FilterCoef val) {
//   Serial.println("fixme");
//   // req_settings_ = settings_;
//   // req_settings_.odr_filter.iir_filter = val;
//   // uint32_t settings_select_ = BMP3_SEL_IIR_FILTER;
//   // err_ = bmp3_set_sensor_settings(settings_select_, &req_settings_ );
//   // if (err_ != BMP3_OK) {
//   //   return false;
//   // }
//   // Get the sensor config
//   // err_ = bmp3_get_sensor_settings(&settings_ );
//   // if (err_ != BMP3_OK) {
//   //   return false;
//   // }
//   return true;
// }

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
