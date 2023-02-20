#include "sensor.hpp"

#if defined(ARDUINO)

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
  uint8_t ret_val;
  i2c->beginTransmission(addr);
  i2c->write(reg);
  i2c->write(data);
  i2c->endTransmission();

  delay(10);
  readRegisters(reg, 1, &ret_val);
  if (data == ret_val) return true;
  return false;
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
  i2c->beginTransmission(addr);
  i2c->write(reg);
  i2c->endTransmission(false);
  // delay(500);
  uint8_t bytes_rx = i2c->requestFrom(static_cast<uint8_t>(addr), count);
  if (bytes_rx == count) {
    for (uint8_t i = 0; i < count; i++) {
      data[i] = i2c->read();
    }
    return true;
  }
  // Serial.println("ReadRegisters::bad read: " + std::to_string(bytes_rx) +
  //                " expected: " + std::to_string(count));
  return false;
}

#endif