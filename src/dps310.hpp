
#pragma once

#include <cstdint>
#include "sensor.hpp"

namespace DPS310 {

struct dps310_t {
  float pressure;
  float temperature;
  bool ok;
};

class gciDPS310 : public SensorI2C {
  public:
  gciDPS310(const uint8_t addr = 0x00, uint32_t port=0) : SensorI2C(addr, port) {}
  ~gciDPS310() {}

  bool ready() { return false; }

  dps310_t read() {
    dps310_t ret{0};
    return ret;
  }
};

} // end namespace dps310