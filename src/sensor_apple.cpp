#include "sensor.hpp"

#if defined(__APPLE__)

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