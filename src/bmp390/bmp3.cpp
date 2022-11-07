
#include "bmp3.hpp"

using namespace BMP390;

constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_ERR = 0x02;
constexpr uint8_t REG_SENS_STATUS = 0x03;
constexpr uint8_t REG_DATA = 0x04;
constexpr uint8_t REG_INT_STATUS = 0x11;
constexpr uint8_t REG_INT_CTRL = 0x19;
constexpr uint8_t REG_PWR_CTRL = 0x1B;
constexpr uint8_t REG_OSR = 0x1C;
constexpr uint8_t REG_ODR = 0x1D;
constexpr uint8_t REG_CALIB_DATA = 0x31;
constexpr uint8_t REG_CMD = 0x71;

constexpr uint8_t WHO_AM_I = 0x60;
constexpr uint8_t LEN_P_T_DATA = 0x21;
constexpr uint8_t CMD_RDY = 0x10;

gciBMP390::gciBMP390(TwoWire *i2c, const uint8_t addr) : Sensor(i2c, addr) {
  found = false;
}

bool gciBMP390::init() {
  bool ok;

  if (!(readRegister(REG_WHO_AM_I) == WHO_AM_I))
    return false;

  ok = soft_reset();
  if (!ok)
    return false;
  ok = get_calib_data();
  if (!ok)
    return false;
  Serial.println("reset ... got calib ...");

  ok = setPowerMode();
  if (!ok)
    return false;

  uint8_t posr = OVERSAMPLING_2X;
  uint8_t tosr = OVERSAMPLING_1X;
  ok = setOverSampling(posr, tosr);
  if (!ok)
    return false;

  ok = setODR(ODR_100_HZ);
  if (!ok)
    return false;

  ok = setIIR(IIR_FILTER_COEFF_1);
  if (!ok)
    return false;

  // drdy enable = 1 << 6
  // non-latch = 0
  // active-high = 1 << 1
  // pin push/pull = 0
  // val = (1 << 6) | (1 << 1);
  // ok = writeRegister(REG_INT_CTRL, val);
  ok = setInterrupt(1, 1);
  if (!ok)
    return false;

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
    press_os = OVERSAMPLING_1X;
    temp_os = OVERSAMPLING_1X;
    odr = ODR_200_HZ;
    break;

  case OS_MODE_PRES_2X_TEMP_1X:
    press_os = OVERSAMPLING_2X;
    temp_os = OVERSAMPLING_1X;
    odr = ODR_100_HZ;
    break;

  case OS_MODE_PRES_4X_TEMP_1X:
    press_os = OVERSAMPLING_4X;
    temp_os = OVERSAMPLING_1X;
    odr = ODR_100_HZ; // was 50
    break;

  case OS_MODE_PRES_8X_TEMP_1X:
    press_os = OVERSAMPLING_8X;
    temp_os = OVERSAMPLING_1X;
    odr = ODR_50_HZ;
    break;

  case OS_MODE_PRES_16X_TEMP_2X:
    press_os = OVERSAMPLING_16X;
    temp_os = OVERSAMPLING_2X;
    odr = ODR_25_HZ;
    break;

  case OS_MODE_PRES_32X_TEMP_2X:
    press_os = OVERSAMPLING_32X;
    temp_os = OVERSAMPLING_2X;
    odr = ODR_12_5_HZ;
    break;
  }

  bool ok = setOverSampling(press_os, temp_os);
  if (!ok)
    return false;
  ok = setODR(odr);
  if (!ok)
    return false;

  return true;
}

bool gciBMP390::setOverSampling(uint8_t posr, uint8_t tosr) {
  uint8_t val = (tosr << 3) | posr;
  return writeRegister(REG_OSR, val);
}

bool gciBMP390::setODR(uint8_t odr) { return writeRegister(REG_ODR, odr); }

bool gciBMP390::setIIR(uint8_t iir) {
  constexpr uint8_t REG_IIR_FILTER = 0x1F;
  uint8_t val = iir << 1;
  return writeRegister(REG_IIR_FILTER, val);
}

bool gciBMP390::setInterrupt(uint8_t drdy_en, uint8_t int_level) {
  // int_level: 1 = active high
  uint8_t val = (drdy_en << 6) | (int_level << 1);
  return writeRegister(REG_INT_CTRL, val);
}

pt_t gciBMP390::read() {
  pt_t ret;
  ret.ok = false;

  bool ok = readRegisters(REG_DATA, LEN_P_T_DATA, buffer);
  if (!ok)
    return ret;

  uint32_t press = to_24b(&buffer[0]);
  uint32_t temp = to_24b(&buffer[3]);

  Serial.println("good read");

  ret.ok = true;
  ret.temp = compensate_temperature(temp); // do temp 1st!!!
  ret.press = compensate_pressure(press);
  return ret;
  // }
  return ret;
}

// datasheet pg 28
float gciBMP390::compensate_temperature(const uint32_t uncomp_temp) {
  double pd1 = (double)uncomp_temp - calib.par_t1;
  double pd2 = pd1 * calib.par_t2;
  calib.t_lin = pd2 + (pd1 * pd1) * calib.par_t3;
  return (float)calib.t_lin;
}

// datasheet pg 28
float gciBMP390::compensate_pressure(const uint32_t uncomp_press) {
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
  pd3 = pd1 * pd2;
  float pd4 =
      pd3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) *
                calib.par_p11;
  float comp_press = po1 + po2 + pd4;

  return comp_press;
}

bool gciBMP390::get_calib_data() {
  bool ok = readRegisters(REG_CALIB_DATA, LEN_CALIB_DATA, buffer);
  if (!ok)
    return false;

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

  calib.par_t1 = (float)to_16b(buffer[1], buffer[0]) / powf(2, -8);
  calib.par_t2 = (float)to_16b(buffer[3], buffer[2]) / powf(2, 30);
  calib.par_t3 = (float)buffer[4] / powf(2, 48);

  calib.par_p1 =
      ((float)to_16b(buffer[6], buffer[5]) - powf(2, 14)) / powf(2, 20);
  calib.par_p2 =
      ((float)to_16b(buffer[8], buffer[7]) - powf(2, 14)) / powf(2, 29);
  calib.par_p3 = (float)buffer[9] / powf(2, 32);
  calib.par_p4 = (float)buffer[10] / powf(2, 37);
  calib.par_p5 = (float)to_16b(buffer[12], buffer[11]) / powf(2, -3);
  calib.par_p6 = (float)to_16b(buffer[14], buffer[13]) / powf(2, 6);
  calib.par_p7 = (float)buffer[15] / powf(2, 8);
  calib.par_p8 = (float)buffer[16] / powf(2, 15);
  calib.par_p9 = (float)to_16b(buffer[18], buffer[17]) / powf(2, 48);
  calib.par_p10 = (float)buffer[19] / powf(2, 48);
  calib.par_p11 = (float)buffer[20] / powf(2, 65);

  // calib.par_t1 = (double)to_16b(buffer[1], buffer[0]) / pow(2,-8);
  // calib.par_t2 = (double)to_16b(buffer[3], buffer[2]) / pow(2,30);
  // calib.par_t3 = (double)buffer[4] / pow(2,48);

  // calib.par_p1 = ((float)to_16b(buffer[6], buffer[5]) - pow(2,14)) /
  // pow(2,20); calib.par_p2 = ((float)to_16b(buffer[8], buffer[7]) -
  // pow(2,14)) / pow(2,29); calib.par_p3 = (float)buffer[9] / pow(2,32);
  // calib.par_p4 = (float)buffer[10] / pow(2,37);
  // calib.par_p5 = (float)to_16b(buffer[12], buffer[11]) / pow(2,-3);
  // calib.par_p6 = (float)to_16b(buffer[14], buffer[13]) / pow(2,6);
  // calib.par_p7 = (float)buffer[15] / pow(2,8);
  // calib.par_p8 = (float)buffer[16] / pow(2,15);
  // calib.par_p9 = (float)to_16b(buffer[18], buffer[17]) / pow(2,48);
  // calib.par_p10 = (float)buffer[19] / pow(2,48);
  // calib.par_p11 = (float)buffer[20] / pow(2,65);

  // Serial.println("got calib data");

  return true;
}

bool gciBMP390::sleep() {
  uint8_t op_mode = readRegister(REG_PWR_CTRL);
  op_mode = op_mode & 0x03; // keep bits 0-1, temp/press enable
  return writeRegister(REG_PWR_CTRL, op_mode);
}

bool gciBMP390::setPowerMode(uint8_t mode) {
  bool ok;

  ok = sleep();
  delay(5);

  constexpr uint8_t PRESS_EN = 0x01;
  constexpr uint8_t TEMP_EN = 0x02;

  uint8_t val = (mode << 4) | TEMP_EN | PRESS_EN;
  ok = writeRegister(REG_PWR_CTRL, val);
  if (!ok)
    return false;

  return true;
}

bool gciBMP390::soft_reset() {
  bool ok;

  // Check for command ready status
  uint8_t cmd_rdy_status = readRegister(REG_SENS_STATUS);

  // Device is ready to accept new command
  if (cmd_rdy_status & CMD_RDY) {
    // Write the soft reset command in the sensor
    ok = writeRegister(REG_CMD, SOFT_RESET);
    if (!ok)
      return false;

    delay(2);

    // Read for command error status
    if (readRegister(REG_ERR) & REG_CMD)
      return false;

    return true;
  }
  return false;
}

float gciBMP390::altitude(const float p) {
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

  return scale * (pow(p * inv_Pb, exp) - 1.0);
}
