#include "Wire.h"

TwoWire::TwoWire(){
  const char* device = "/dev/i2c-1"; // -0 is used for other stuff
  if ((fd = open (device, O_RDWR)) < 0) {
    printf("Fail open %s\n", device);
  }
}

TwoWire::~TwoWire() {
  close(fd);
}

void TwoWire::set(uint8_t address) {
  addr = address;
  if (ioctl (fd, I2C_SLAVE, addr) < 0) {
    printf("write error\n");
    close(fd); // something is wrong, so stop?
  }
}

bool TwoWire::read(const uint8_t reg, const uint8_t count, uint8_t *const data) {
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
}

bool TwoWire::write(const uint8_t reg, const uint8_t data) {
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
}

TwoWire Wire;