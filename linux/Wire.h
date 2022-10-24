#pragma once

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



#include <stdint.h>     // int types
#include <unistd.h>			// file read/write
#include <fcntl.h>		  // file open
#include <sys/ioctl.h>	// Needed for I2C port
#include <cstdio>       // printf
#include <cstring>      // memset


// sudo apt-get install libi2c-dev
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

constexpr uint8_t I2C_BUFFER_SIZE = 32;

// #ifndef I2C_SET_SPEED
// #define I2C_SET_SPEED 0x0758  // Set I2C Bus Speed - is this right?
// #endif

class TwoWire {
  public:
  TwoWire(): counter(0), fd(0), readcntr(0) {}
  ~TwoWire() {
    end();
  }

  void end() {
    if (fd >= 0) close(fd);
    printf("TwoWire bye\n");
  }

  void    begin(int sda=0, int scl=0, uint8_t address=0) {
    const char* device = "/dev/i2c-1"; // -0 is used for other stuff
    if ((fd = open (device, O_RDWR)) < 0) {
      printf("Fail open %s\n", device);
		}
  }

  // NOT sure this works, either 100000 or 400000 ... just set /boot/config.txt to 400k
  // void    setClock(uint32_t speed) {
  //   ioctl(fd, I2C_SET_SPEED, speed);
  // }
  void    setClock(uint32_t speed) {} // just set /boot/config.txt to 400k
  void    setClockStretchLimit(uint32_t) {}

  void    beginTransmission(uint8_t addr) {
    counter = 0;
    if (ioctl (fd, I2C_SLAVE, addr) < 0) {
      printf("write error\n");
      close(fd); // something is wrong, so stop?
    }
  }

  uint8_t endTransmission(uint8_t stop=0) {
    // i2c_smbus_write_block_data(fd,)
    ::write(fd, buffer, counter);
    return 0;
  }

  uint8_t  requestFrom(uint8_t addr, uint8_t length, uint8_t sendStop=0) {
    int32_t num;
    // if((num = i2c_smbus_read_i2c_block_data(fd, addr, length, buffer)) < 0) printf("block read error\n");
    num = ::read(fd, buffer, counter);
    readcntr = 0;
    return num;
  }
  // uint8_t requestFrom(int, int, int=0) {return 0;}

  uint8_t write(uint8_t data) {
    buffer[counter++] = data;
    return 1;
  }

  uint8_t write(const uint8_t* data, uint8_t len) {
    memcpy(buffer, data, len);
    counter += len;
    return len;
  }

  uint8_t read(void) {
    if (readcntr >= counter) return 0x00;
    return buffer[readcntr++];
  }

  // read block of data
  uint8_t readBlock(uint8_t reg, uint8_t len=1) {
    if (len > I2C_BUFFER_SIZE) len = I2C_BUFFER_SIZE;
    memset(buffer, 0 , I2C_BUFFER_SIZE);
    int32_t ans = i2c_smbus_read_i2c_block_data(fd, reg, len, buffer);
    return (ans & 0xFF);
  }

  // value?
  uint8_t writeBlock(uint8_t reg, uint8_t len, uint8_t* data) {
    int32_t ans = i2c_smbus_write_block_data(fd,reg, len, data);
    // ::write(fd, data, len);
    return (ans & 0xFF);
  }

  uint8_t buffer[I2C_BUFFER_SIZE];

  protected:
  int fd;
  uint8_t counter;
  uint8_t readcntr;
};

extern TwoWire Wire; // declared in C++ file
