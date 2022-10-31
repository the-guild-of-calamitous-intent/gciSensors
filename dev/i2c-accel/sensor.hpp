/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

// #include <Arduino.h>
// #include <Wire.h>
#include <stdint.h>
#ifdef __linux__
#include <stdint.h>     // int types
#include <unistd.h>			// file read/write
#include <fcntl.h>		  // file open
#include <sys/ioctl.h>	// Needed for I2C port
#include <cstdio>       // printf
#include <cstring>      // memset
constexpr uint8_t I2C_MAX_BUFFER_SIZE = 32;
#endif

// sudo apt-get install libi2c-dev
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}


#include <math.h>
namespace Units {

// Angular
constexpr float rad2deg = 180.0f/M_PI;
constexpr float deg2rad = M_PI/180.0f;
}

class TwoWire {
  public:
  TwoWire(){}
};



#include <unistd.h>			// file read/write usleep
static void delay(uint32_t ms) {usleep(1000*ms);}

/*
I don't like some of this ... need to clean it up!
*/
class Sensor {
public:
  Sensor(TwoWire *tw, const uint8_t address) : addr(address), i2c(tw), fd(0) {
    const char* device = "/dev/i2c-1"; // -0 is used for other stuff
    if ((fd = open (device, O_RDWR)) < 0) {
      printf("Fail open %s\n", device);
		}
    if (ioctl (fd, I2C_SLAVE, addr) < 0) {
      printf("write error\n");
      close(fd); // something is wrong, so stop?
    }
  }

  /*
  reg - the register we want to change
  bits - how many bits for mask
  shift - how much to shift data by
  data - returned value pointer
  */
  bool Read(const uint8_t reg, const uint8_t bits, const uint8_t shift,
            uint8_t *const data) {
    uint8_t val;
    if (!ReadRegisters(reg, 1, &val)) {
      return false;
    }
    val >>= shift;
    uint8_t mask_ = (1 << (bits)) - 1;
    *data = val & mask_;
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
  // bool updateCtrlReg()
  bool Write(const uint8_t reg, const uint8_t data, const uint8_t bits,
             const uint8_t shift) {
    uint8_t val;
    if (!ReadRegisters(reg, 1, &val)) {
      return false;
    }
    uint8_t mask = (1 << (bits)) - 1;
    uint8_t d = data & mask;
    mask <<= shift;
    val &= ~mask;
    val |= d << shift;
    return WriteRegister(reg, val);
  }

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
  bool WriteRegister(const uint8_t reg, const uint8_t data) {
  #ifdef __linux__
      outbuf[0] = reg;
      outbuf[1] = data;
      // memcpy(&outbuf[1], &data, len);

      msgs[0].addr = addr;
      msgs[0].flags = 0;
      msgs[0].len = 2;
      msgs[0].buf = outbuf;

      i2c_data.msgs = msgs;
      i2c_data.nmsgs = 1;

      if (ioctl(fd, I2C_RDWR, &i2c_data) < 0) {
          perror("ioctl(I2C_RDWR) in i2c_write");
          return false;
      }

      return true;
  #else
    uint8_t ret_val;
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->write(data);
    i2c->endTransmission();

    delay(10);
    ReadRegisters(reg, sizeof(ret_val), &ret_val);
    if (data == ret_val) {
      return true;
    } else {
      return false;
    }
  #endif
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
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {
  #ifdef __linux__
  // if (count > I2C_MAX_BUFFER_SIZE) count = I2C_MAX_BUFFER_SIZE;

      // send out to sensor
      msgs[0].addr = addr;
      msgs[0].flags = 0;
      msgs[0].len = 1;
      msgs[0].buf = outbuf;
      outbuf[0] = reg;

      // read in from sensor
      msgs[1].addr = addr;
      msgs[1].flags = I2C_M_RD; // read data, from slave to master
      msgs[1].len = count;
      msgs[1].buf = data; //inbuf;

      i2c_data.msgs = msgs;
      i2c_data.nmsgs = 2;

      if (ioctl(fd, I2C_RDWR, &i2c_data) < 0) {
          perror("ioctl(I2C_RDWR) in i2c_read");
          return false;
      }

      return true;
  #else
    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->endTransmission(false);
    // delay(500);
    uint8_t bytes_rx_ = i2c->requestFrom(static_cast<uint8_t>(addr), count);
    if (bytes_rx_ == count) {
      for (size_t i = 0; i < count; i++) {
        data[i] = i2c->read();
      }
      return true;
    } else {
      Serial.println("ReadRegisters::bad read " + std::to_string(bytes_rx_) + " expected: " + std::to_string(count));
      return false;
    }
  #endif
  }

  /*
  Returns the register value and returns the entire register.
  */
  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    if (ReadRegisters(reg, sizeof(value), &value) != 1)
      return -1;
    return value;
  }

  // inline bool checkErr(int val) { return (val < 0) ? false : true; }

  TwoWire *i2c;
  uint8_t addr;

  #ifdef __linux__
  int fd;
  uint8_t outbuf[2];
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data i2c_data;
  #endif
};