#include "sensor.hpp"

#if defined(__linux__)

bool SensorI2C::writeRegister(const uint8_t reg, const uint8_t data) {
  i2c->set(addr);
  return i2c->write(reg, data);
}

bool SensorI2C::readRegisters(const uint8_t reg, const uint8_t count,
                              uint8_t *const data) {
  i2c->set(addr);
  return i2c->read(reg, count, data);
}

#endif