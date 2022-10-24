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

//mag
constexpr uint8_t MAG_REG_WHO_AM_I = 0x0F;
constexpr uint8_t MAG_WHO_AM_I = 0x3D;
constexpr uint8_t MAG = 0x1C;
constexpr uint8_t REG_OUT_X_L = 0x28;

int main() {
  cout << "start" << endl;

  Wire.begin();

  Wire.beginTransmission(ACCEL);
  Wire.write(WHO_AM_I_REG);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL, 1, false);
  uint8_t ans = Wire.read();
  if (ans == ACCEL_WHO_AM_I) cout << "accel good " << int(ans) << endl;
  else cout << "crap" << endl;


  Wire.beginTransmission(MAG);
  Wire.write(MAG_REG_WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(MAG, 1, false);
  ans = Wire.read();
  if (ans == MAG_WHO_AM_I) cout << "mag good " << int(ans) << endl;
  else cout << "crap" << endl;


  ans = Wire.readBlock(MAG_REG_WHO_AM_I);
  ans = Wire.buffer[0];
  if (ans == MAG_WHO_AM_I) cout << "mag test good " << int(ans) << endl;
  else cout << "crap " << int(ans) << endl;

  // Wire.beginTransmission(ACCEL);
  ans = Wire.readBlock(REG_OUT_X_L, 6);
  cout << "Read: " << int(ans) << endl;
  for (int i=0; i < ans; i++) cout << int(Wire.buffer[i]) << " ";
  cout << endl;

  return 0;
}