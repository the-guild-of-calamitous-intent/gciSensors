// sudo i2cdetect -y 1

#include <stdint.h>
#include <string>
#include <iostream>
#include <stdlib.h>
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

int fd;




// I2C definitions

#define I2C_SLAVE	0x0703
#define I2C_SMBUS	0x0720	/* SMBus-level access */

#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

// SMBus transaction types

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */
#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */

int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}

int wiringPiI2CReadReg8 (int fd, int reg, uint8_t *val)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data)) {
    // printf("error i2c_smbus_access");
    return -1 ;
  }
  *val = data.byte & 0xFF ;

  return 0;
}

int wiringPiI2CReadReg16 (int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data))
    return -1 ;
  else
    return data.word & 0xFFFF ;
}

int wiringPiI2CWriteReg8 (int fd, int reg, int value)
{
  union i2c_smbus_data data ;

  data.byte = value ;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data) ;
}

int main() {
  cout << "start" << endl;

  const char* device = "/dev/i2c-1"; // -0 is used for other stuff
  if ((fd = open (device, O_RDWR)) < 0) {
    printf("Fail open %s\n", device);
    return -1;
  }
  if (ioctl (fd, I2C_SLAVE, ACCEL) < 0) {
    printf("write error\n");
    close(fd); // something is wrong, so stop?
    return -1;
  }

  uint8_t ans = 0;
  int e = wiringPiI2CReadReg8(ACCEL,WHO_AM_I_REG,&ans);
  if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  else cout << "accel crap" << endl;

  // ans = 0;
  // ans = wiringPiI2CReadReg8(ACCEL,WHO_AM_I_REG);
  // if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  // else cout << "accel crap" << endl;

  // ans = 0;
  // ans = wiringPiI2CReadReg8(ACCEL,WHO_AM_I_REG);
  // if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  // else cout << "accel crap" << endl;

  printf("----------\n");
  uint8_t val = (RATE_104_HZ << 4) + (ACCEL_RANGE_4_G);
  int ok = wiringPiI2CWriteReg8(ACCEL,CTRL1_XL,val);
  if (ok < 0) printf("crap i2c_write\n");

  val = 0x00;
  ok = wiringPiI2CWriteReg8(ACCEL,CTRL8_XL,val);
  if (ok < 0) printf("crap i2c_write\n");


  for (int ii; ii < 3; ++ii) {
    // Wire.beginTransmission(ACCEL);
    // ans = Wire.readBlock(OUTX_L_XL, 6);
    // memcpy(buff.b, Wire.buffer, 6);
    int err =0;

    // wiringPiI2CReadReg8(ACCEL,OUTX_L_XL,  &buff.b[0]);
    // wiringPiI2CReadReg8(ACCEL,OUTX_L_XL+1,&buff.b[1]);
    err = wiringPiI2CReadReg8(ACCEL,OUTX_L_XL,&buff.b[0]);
    err = wiringPiI2CReadReg8(ACCEL,OUTX_L_XL+1,&buff.b[1]);

    err = wiringPiI2CReadReg8(ACCEL,OUTX_L_XL+2,&buff.b[2]);
    err = wiringPiI2CReadReg8(ACCEL,OUTX_L_XL+3,&buff.b[3]);

    err = wiringPiI2CReadReg8(ACCEL,OUTX_L_XL+4,&buff.b[4]);
    err = wiringPiI2CReadReg8(ACCEL,OUTX_L_XL+5,&buff.b[5]);

    // Wire.beginTransmission(ACCEL);
    // Wire.write(REG_OUT_X_L);
    // Wire.endTransmission();
    // ans = Wire.requestFrom(ACCEL, 6, false);
    // cout << "Read: " << int(ans) << endl;
    // for (int i=0; i < ans; i++) cout << int(Wire.buffer[i]) << " ";
    // cout << endl;
    cout << buff.s[0] * 4.0f / 32768.0f << " ";
    cout << buff.s[1] * 4.0f / 32768.0f << " ";
    cout << buff.s[2] * 4.0f / 32768.0f << " ";
    // cout << endl;
    // for (int i=0; i < 2; i+=2) cout << ((buff.b[i+1]<<8) + buff.b[i]) * 4.0f / 32768.0f << " ";
    // for (int i=0; i < 2; i+=2) cout << ((buff.b[i]<<8) + buff.b[i+1]) * 4.0 / 32768.0f << " ";
    cout << endl;
  }

  return 0;
}