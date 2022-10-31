// sudo i2cdetect -y 1

// // Defines for struct i2c_msg - flags field
// #define I2C_M_RD 0x0001 /* read data, from slave to master */
// #define I2C_M_TEN 0x0010 /* this is a ten bit chip address */
// #define I2C_M_RECV_LEN 0x0400 /* length will be first received byte */
// #define I2C_M_NO_RD_ACK 0x0800 /* if I2C_FUNC_PROTOCOL_MANGLING */
// #define I2C_M_IGNORE_NAK 0x1000 /* if I2C_FUNC_PROTOCOL_MANGLING */
// #define I2C_M_REV_DIR_ADDR 0x2000 /* if I2C_FUNC_PROTOCOL_MANGLING */
// #define I2C_M_NOSTART 0x4000 /* if I2C_FUNC_NOSTART */
// #define I2C_M_STOP 0x8000 /* if I2C_FUNC_PROTOCOL_MANGLING */

// struct i2c_msg {
// __u16 addr; /* slave address */
// __u16 flags; /* see above for flag definitions */
// __u16 len; /* msg length */
// __u8 *buf; /* pointer to msg data */
// };
// (Defined in uapi/linux/i2c.h)
// This data structure provides for the definition of the device address,
// if it's a transmit or receive, pointer to data to send/receive, and size of data.

// Another common data structure is struct i2c_rdwr_ioctl_data

// This is the structure as used in the I2C_RDWR ioctl call
// struct i2c_rdwr_ioctl_data {
// struct i2c_msg __user *msgs; /* pointers to i2c_msgs */
// __u32 nmsgs; /* number of i2c_msgs */
// };

#include <stdint.h>
#include <string>
#include <iostream>
#include <stdlib.h>
// #include <stdint.h>     // int types
// #include <unistd.h>			// file read/write usleep
// #include <fcntl.h>		  // file open
// #include <sys/ioctl.h>	// Needed for I2C port
// #include <cstdio>       // printf
// #include <cstring>      // memset


// // sudo apt-get install libi2c-dev
// extern "C" {
//   #include <linux/i2c-dev.h>
//   #include <i2c/smbus.h>
// }

#include "Wire.h"

static void delay(uint32_t ms) {usleep(1000*ms);}

using namespace std;

// accel
constexpr uint8_t WHO_AM_I_REG = 0x0F;
constexpr uint8_t ACCEL_WHO_AM_I = 0x6C;
constexpr uint8_t ACCEL = 0x6A;
constexpr uint8_t OUTX_L_XL = 0X28;
constexpr uint8_t CTRL1_XL = 0x10; // Accel settings
constexpr uint8_t CTRL8_XL = 0x17; // Accel filtering

/** The accelerometer data rate */
enum ODR : uint8_t {
  RATE_SHUTDOWN,
  RATE_12_5_HZ,
  RATE_26_HZ,
  RATE_52_HZ,
  RATE_104_HZ,
  RATE_208_HZ,
  RATE_416_HZ,
  RATE_833_HZ,
  RATE_1_66K_HZ,
  RATE_3_33K_HZ,
  RATE_6_66K_HZ,
};

/** The accelerometer data range */
enum accel_range : uint8_t {
  ACCEL_RANGE_2_G,
  ACCEL_RANGE_16_G,
  ACCEL_RANGE_4_G,
  ACCEL_RANGE_8_G
};

union Data {
  uint8_t b[6];
  int16_t s[3];
} buff;


int main() {
  cout << "start" << endl;

  uint8_t ans = 0;
  int e = i2c_read(ACCEL,WHO_AM_I_REG,&ans);
  if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  else cout << "accel crap" << endl;

  ans = 0;
  e = i2c_read(ACCEL,WHO_AM_I_REG,&ans);
  if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  else cout << "accel crap" << endl;

  printf("----------\n");
  uint8_t val = (RATE_104_HZ << 4) + (ACCEL_RANGE_4_G << 2);
  int ok = 0;
  ok = i2c_write(ACCEL,CTRL1_XL,val);
  if (ok < 0) printf("crap i2c_write\n");

  val = 0x00;
  ok = i2c_write(ACCEL,CTRL8_XL,val);
  if (ok < 0) printf("crap i2c_write\n");


  for (int ii; ii < 3000; ++ii) {
    int err =0;

    err = i2c_read(ACCEL,OUTX_L_XL,buff.b, 6);
    if (err < 0) printf("i2c_read error\n");

    float ascale = 4.0f / 32768.0f;

    cout << buff.s[0] * ascale << " ";
    cout << buff.s[1] * ascale << " ";
    cout << buff.s[2] * ascale << " ";
    cout << endl;

    delay(10);
  }

  return 0;
}