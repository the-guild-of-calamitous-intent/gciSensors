#include <stdint.h>
#include <string>
#include <iostream>
#include <stdlib.h>

#include "Wire.h"

using namespace std;

// TwoWire tw;

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

//mag
constexpr uint8_t MAG_REG_WHO_AM_I = 0x0F;
constexpr uint8_t MAG_WHO_AM_I = 0x3D;
constexpr uint8_t MAG = 0x1C;
constexpr uint8_t REG_OUT_X_L = 0x28;


union Data {
  uint8_t b[6];
  uint16_t s[3];
} buff;

int main() {
  cout << "start" << endl;

  Wire.begin();

  // Wire.beginTransmission(ACCEL);
  // Wire.write(WHO_AM_I_REG);
  // Wire.endTransmission();
  // Wire.requestFrom(ACCEL, 1, false);
  // uint8_t ans = Wire.read();
  uint8_t ans = 0;
  Wire.i2c_read(ACCEL,WHO_AM_I_REG,&ans);
  if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  else cout << "accel crap" << endl;


  // Wire.beginTransmission(MAG);
  // Wire.write(MAG_REG_WHO_AM_I);
  // Wire.endTransmission();
  // Wire.requestFrom(MAG, 1, false);
  // ans = Wire.read();
  // if (ans == MAG_WHO_AM_I) cout << "mag good " << int(ans) << endl;
  // else cout << "crap" << endl;


  // ans = Wire.readBlock(MAG_REG_WHO_AM_I);
  // ans = Wire.buffer[0];
  // if (ans == MAG_WHO_AM_I) cout << "mag test good " << int(ans) << endl;
  // else cout << "crap " << int(ans) << endl;

  printf("----------\n");
  uint8_t val = (RATE_104_HZ << 4) + (ACCEL_RANGE_4_G);
  int ok = Wire.i2c_write(ACCEL,CTRL1_XL,val);
  if (!ok) printf("crap i2c_write\n");

  val = 0x00;
  ok = Wire.i2c_write(ACCEL,CTRL8_XL,val);
  if (!ok) printf("crap i2c_write\n");


  for (int ii; ii < 4; ++ii) {
    // Wire.beginTransmission(ACCEL);
    // ans = Wire.readBlock(OUTX_L_XL, 6);
    // memcpy(buff.b, Wire.buffer, 6);


    Wire.i2c_read(ACCEL,OUTX_L_XL,  &buff.b[0]);
    Wire.i2c_read(ACCEL,OUTX_L_XL+1,&buff.b[1]);

    // Wire.beginTransmission(ACCEL);
    // Wire.write(REG_OUT_X_L);
    // Wire.endTransmission();
    // ans = Wire.requestFrom(ACCEL, 6, false);
    // cout << "Read: " << int(ans) << endl;
    // for (int i=0; i < ans; i++) cout << int(Wire.buffer[i]) << " ";
    // cout << endl;
    for (int i=0; i < 1; i++) cout << buff.s[i] * 4.0f / 32768.0f << " ";
    // cout << endl;
    // for (int i=0; i < 2; i+=2) cout << ((buff.b[i+1]<<8) + buff.b[i]) * 4.0f / 32768.0f << " ";
    // for (int i=0; i < 2; i+=2) cout << ((buff.b[i]<<8) + buff.b[i+1]) * 4.0 / 32768.0f << " ";
    cout << endl;
  }

  return 0;
}