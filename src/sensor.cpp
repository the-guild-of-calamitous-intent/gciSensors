#include "sensor.hpp"

/*
reg - the register we want to change
bits - how many bits for mask
shift - how much to shift data by
data - returned value pointer
*/
bool SensorI2C::readBits(const uint8_t reg, const uint8_t bits, const uint8_t shift,
                  uint8_t *const data) {
  uint8_t val;
  if (!readRegisters(reg, 1, &val)) {
    return false;
  }
  val >>= shift;
  uint8_t mask = (1 << (bits)) - 1;
  *data = val & mask;
  return true;
}

/*
Given some data, this will:
1. read the register to get all the bits
2. mask out the we don't want to change to protect them
3. only change the correct bits
4. write the final value back to the register

reg - the register we want to change
data - data that goes into register
bits - how many bits for mask
shift - how much to shift data by
*/
bool SensorI2C::writeBits(const uint8_t reg, const uint8_t data, const uint8_t bits,
                   const uint8_t shift) {
  uint8_t val;
  if (!readRegisters(reg, 1, &val)) {
    return false;
  }
  uint8_t mask = (1 << (bits)) - 1;
  uint8_t d = data & mask;
  mask <<= shift;
  val &= ~mask;
  val |= d << shift;
  return writeRegister(reg, val);
}

// /*!
//  * @details sets register and verifies it was correct
//  *
//  * @param[in] reg : starting register adress
//  * @param[in] data : returned data pointer
//  *
//  * @return true (success) or false (fail)
//  * @retval false fail
//  * @retval true success
//  */
// bool SensorI2C::writeRegister(const uint8_t reg, const uint8_t data) {
// #if defined(__linux__)
//   i2c->set(addr);
//   return i2c->write(reg, data);
// #elif defined(ARDUINO)
//   uint8_t ret_val;
//   i2c->beginTransmission(addr);
//   i2c->write(reg);
//   i2c->write(data);
//   i2c->endTransmission();

//   delay(10);
//   readRegisters(reg, 1, &ret_val);
//   if (data == ret_val)
//     return true;
//   return false;

// #endif
// }

// /*!
//  * @details Reads the number of bytes starting at address of register
//  *
//  * @param[in] reg : starting register adress
//  * @param[in] count : number of bytes to read
//  * @param[in] data : returned data pointer
//  *
//  * @return true (success) or false (fail)
//  * @retval false fail
//  * @retval true success
//  */
// bool SensorI2C::readRegisters(const uint8_t reg, const uint8_t count,
//                            uint8_t *const data) {
// #if defined(__linux__)
//   i2c->set(addr);
//   return i2c->read(reg, count, data);
// #elif defined(ARDUINO)
//   i2c->beginTransmission(addr);
//   i2c->write(reg);
//   i2c->endTransmission(false);
//   // delay(500);
//   uint8_t bytes_rx = i2c->requestFrom(static_cast<uint8_t>(addr), count);
//   if (bytes_rx == count) {
//     for (uint8_t i = 0; i < count; i++) {
//       data[i] = i2c->read();
//     }
//     return true;
//   }
//   // Serial.println("ReadRegisters::bad read: " + std::to_string(bytes_rx) +
//   //                " expected: " + std::to_string(count));
//   return false;

// #endif
// }

/*
Returns the register value and returns the entire register.
*/
uint8_t SensorI2C::readRegister(uint8_t reg) {
  uint8_t value;
  if (!readRegisters(reg, 1, &value))
    return 0;
  return value;
}

bool SensorI2C::readRegister(uint8_t reg, uint8_t* value) {
  return readRegisters(reg, 1, value);
}

#if !(defined(linux) || defined(ARDUINO))

/*!
 * @details sets register and verifies it was correct
 *
 * @param[in] reg : starting register adress
 * @param[in] data : returned data pointer
 *
 * @return true (success) or false (fail)
 * @retval false fail
 * @retval true success
 */
bool SensorI2C::writeRegister(const uint8_t reg, const uint8_t data) { return true; }

/*!
 * @details Reads the number of bytes starting at address of register
 *
 * @param[in] reg : starting register adress
 * @param[in] count : number of bytes to read
 * @param[in] data : returned data pointer
 *
 * @return true (success) or false (fail)
 * @retval false fail
 * @retval true success
 */
bool SensorI2C::readRegisters(const uint8_t reg, const uint8_t count,
                           uint8_t *const data) { return true; }

#endif