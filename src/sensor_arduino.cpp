#include "sensor.hpp"

#if defined(ARDUINO)

// #define DEBUG 1

// #if DEBUG
// inline void println(const String& s) {
//   Serial.println(s);
// }
// #else
// void println(const String& s) {}
// #endif

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

  // println("data write failed verification: " + String(int(data)) +
  //         " != " + String(int(ret_val)));

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
  // delay(2);

  uint8_t bytes_rx = i2c->requestFrom(addr, count);
  if (bytes_rx == count) {
    for (uint8_t i = 0; i < count; i++) {
      data[i] = i2c->read();
    }
    return true;
  }

  // println("ReadRegisters::bad read: " + String(int(bytes_rx)) +
  //         " expected: " + String(int(count)));

  return false;
}

#endif