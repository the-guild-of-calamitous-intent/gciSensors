#include "sensor.hpp"

#if defined(__APPLE__)


bool SensorI2C::writeRegister(const uint8_t reg, const uint8_t data) {
  return true;
}

bool SensorI2C::readRegisters(const uint8_t reg, const uint8_t count,
                              uint8_t *const data) {
  return true;
}

#endif