#include "sensor.hpp"

/*
Returns the register value and returns the entire register.
*/
uint8_t SensorI2C::readRegister(uint8_t reg) {
  uint8_t value;
  if (!readRegisters(reg, 1, &value)) return 0;
  return value;
}

// bool SensorI2C::readRegister(uint8_t reg, uint8_t *value) {
//   return readRegisters(reg, 1, value);
// }
