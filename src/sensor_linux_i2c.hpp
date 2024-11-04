/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
// https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h

// not useful
// https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiI2C.c

// https://docs.kernel.org/driver-api/i2c.html

// https://elinux.org/images/1/1e/I2C-SPI-ELC-2020.pdf

/*
https://www.kernel.org/doc/Documentation/i2c/dev-interface

ioctl(file, I2C_SMBUS, struct i2c_smbus_ioctl_data *args)
  If possible, use the provided i2c_smbus_* methods described below instead
  of issuing direct ioctls.

You can do plain i2c transactions by using read(2) and write(2) calls.
You do not need to pass the address byte; instead, set it through
ioctl I2C_SLAVE before you try to access the device.

You can do SMBus level transactions (see documentation file smbus-protocol
for details) through the following functions:
  __s32 i2c_smbus_write_quick(int file, __u8 value);
  __s32 i2c_smbus_read_byte(int file);
  __s32 i2c_smbus_write_byte(int file, __u8 value);
  __s32 i2c_smbus_read_byte_data(int file, __u8 command);
  __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);
  __s32 i2c_smbus_read_word_data(int file, __u8 command);
  __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);
  __s32 i2c_smbus_process_call(int file, __u8 command, __u16 value);
  __s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values);
  __s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length,
                                   __u8 *values);
All these transactions return -1 on failure; you can read errno to see
what happened. The 'write' transactions return 0 on success; the
'read' transactions return the read value, except for read_block, which
returns the number of values read. The block buffers need not be longer
than 32 bytes.

The above functions are made available by linking against the libi2c library,
which is provided by the i2c-tools project.  See:
https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/.

*/

/**
set i2c speed
/boot/config.txt
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
*/
#pragma once

// #include <Wire.hpp>
#include <string>
#include <stdint.h> // int types

// sudo apt-get install libi2c-dev
extern "C" {
  #include <i2c/smbus.h>
  #include <linux/i2c-dev.h>
}

#include <stddef.h>    // size_t
#include <cstdio>      // printf / perror
#include <cstring>     // memset
#include <fcntl.h>     // file open
#include <stdint.h>    // int types
#include <sys/ioctl.h> // Needed for I2C port
#include <unistd.h>    // file read/write

struct i2c_inst_t {

  int fd;

  i2c_inst_t(const std::string& dev) {
    open(dev);
  }
  ~i2c_inst_t() {
    fdclose();
  }

  void open(const std::string& device) {
    if ((fd = ::open(device.c_str(), O_RDWR)) < 0) {
      fd = 0;
      // printf("*** i2c_inst_t failed to open %s ***\n", device.c_str());
    }
  }

  void fdclose() {
    if (fd != 0) sensors::close(fd);
    fd = 0;
  }
};

// static i2c_inst_t i2c0_inst("/dev/i2c-0");
static i2c_inst_t i2c1_inst("/dev/i2c-1");
// static i2c_inst_t i2c2_inst("/dev/i2c-2");
// static i2c_inst_t i2c3_inst("/dev/i2c-3");


constexpr uint8_t I2C_MAX_BUFFER_SIZE = 32;

class SensorI2C {
public:
  SensorI2C(const uint8_t address, uint32_t) : addr(address) {
    i2c = &i2c1_inst; // rpi ONLY has this i2c available

    // if (port == 1) i2c = &i2c1_inst;
    // else if (port == 0) i2c = &i2c0_inst;
    // else if (port == 2) i2c = &i2c2_inst;
    // else if (port == 3) i2c = &i2c3_inst;
    // else printf("*** SensorI2C failed, %u is invalid port\n", port);

    // i2c = new i2c_inst_t;

    // if ((i2c->fd = open(device, O_RDWR)) < 0) {
    //   printf("Fail open %s\n", device);
    // }
  }
  ~SensorI2C() { i2c->fdclose(); }

  // void init_tw(const uint32_t baud) { i2c = nullptr; }

  bool writeRegister(const uint8_t reg, const uint8_t data) {
    // i2c->set(addr);
    // return i2c->write(reg, data);

    // set(addr);
    if (ioctl(i2c->fd, I2C_SLAVE, addr) < 0) {
      // printf("write error\n");
      // ::close(i2c->fd); // something is wrong, so stop?
      i2c->fdclose();
    }

    outbuf[0] = reg;
    outbuf[1] = data;
    // memcpy(&outbuf[1], &data, len);

    msgs[0].addr   = addr;
    msgs[0].flags  = 0;
    msgs[0].len    = 2;
    msgs[0].buf    = outbuf;

    i2c_data.msgs  = msgs;
    i2c_data.nmsgs = 1;

    if (ioctl(i2c->fd, I2C_RDWR, &i2c_data) < 0) {
      // perror("ioctl(I2C_RDWR) in i2c_write");
      return false;
    }

    return true;
  }

  bool readRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t *const data) {
    // i2c->set(addr);
    // return i2c->read(reg, count, data);
    // if (count > I2C_MAX_BUFFER_SIZE) count = I2C_MAX_BUFFER_SIZE;

    // set(addr);

    if (ioctl(i2c->fd, I2C_SLAVE, addr) < 0) {
      // printf("write error\n");
      // ::close(i2c->fd); // something is wrong, so stop?
      i2c->fdclose();
    }

    // send out to sensor
    msgs[0].addr  = addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = outbuf;
    outbuf[0]     = reg;

    // read in from sensor
    msgs[1].addr   = addr;
    msgs[1].flags  = I2C_M_RD; // read data, from slave to master
    msgs[1].len    = count;
    msgs[1].buf    = data; // inbuf;

    i2c_data.msgs  = msgs;
    i2c_data.nmsgs = 2;

    if (ioctl(i2c->fd, I2C_RDWR, &i2c_data) < 0) {
      // perror("ioctl(I2C_RDWR) in i2c_read");
      return false;
    }

    return true;
  }

  inline
  bool readRegister(const uint8_t reg, uint8_t *data) {
    return readRegisters(reg, 1, data);
  }

  // void set(uint8_t address) {
  //   // addr = address;
  //   if (ioctl(i2c->fd, I2C_SLAVE, addr) < 0) {
  //     printf("write error\n");
  //     close(i2c->fd); // something is wrong, so stop?
  //   }
  // }

  int i2c_write_blocking (i2c_inst_t*, uint8_t, const uint8_t*, size_t, bool) {
    return 0;
  }
  int i2c_read_blocking (i2c_inst_t*, uint8_t, uint8_t*, size_t, bool) {
    return 0;
  }

  // uint8_t readRegister(uint8_t reg) {
  //   uint8_t value;
  //   if (!readRegisters(reg, 1, &value)) return 0;
  //   return value;
  // }

  // inline size_t available() { return 0; }

protected:
  i2c_inst_t *i2c{nullptr};
  const uint8_t addr;
  uint8_t outbuf[2];
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data i2c_data;
};
