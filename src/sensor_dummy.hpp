/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

class SensorI2C {
public:
  SensorI2C(uint8_t addr, const uint32_t): addr(addr) {}
  ~SensorI2C() {}

  bool writeRegister(const uint8_t, const uint8_t) { return false; }
  bool writeRegister(const uint8_t, const uint8_t, uint8_t*) { return false; }
  bool readRegisters(const uint8_t, const size_t,
                     uint8_t *const) { return false; }
  inline bool readRegister(const uint8_t, uint8_t*) { return false;}
  inline size_t available() { return 0; }
protected:
  uint8_t addr;
};



class SensorSPI {
public:
  SensorSPI(const uint32_t) {}
  ~SensorSPI() {}

  void set_cs(uint32_t) {}

  bool write_registers(const uint8_t, const uint8_t) { return false; }
  bool read_registers(const uint8_t, const size_t,
                     uint8_t *const) { return false; }
  inline bool read_register(const uint8_t, uint8_t*) { return false;}
};
