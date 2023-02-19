#include "sensor.hpp"

#if defined(__linux__)

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
bool SensorI2C::writeRegister(const uint8_t reg, const uint8_t data) {
  i2c->set(addr);
  return i2c->write(reg, data);
}

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
                              uint8_t *const data) {
  i2c->set(addr);
  return i2c->read(reg, count, data);
}

#endif