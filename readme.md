# gciSensors

## Why?

I love Adafruit sensors with QWIIC but their drivers are geared towards hobbiest and
favor low power consumption over performace. I try to strip out unnecessary abstraction
and enable high data rate.

## LSM6DSOX

- Accelerometer ±2/±4/±8/±16 g at 1.6 Hz to 6.7KHz update rate
- Gyroscope: ±125/±250/±500/±1000/±2000 dps at 12.5 Hz to 6.7 KHz
- 32 bit timer at 25 usec resolution
- I2C Address `0x6A` or `0x6B`
- ``init()``:
    - set accel to ``RATE_104_HZ`` ``ACCEL_RANGE_4_G``
    - sets gyro to ``RATE_104_HZ`` ``GYRO_RANGE_2000_DPS``

```c++
LSM6DSOX::gciLSM6DSOX imu(&Wire);
imu.init();
bool ok = imu.ready(); // true when new data available
if (ok) sox_t s = imu.read();
// s.ok => good read true/false
// s.ax => accel x,y,z in g's
// s.gx => gyro x,y,z in rads/sec
// s.temp => temperature in C
```

## LIS3MDL

- ±4/±8/±12/±16 gauss selectable magnetic full scales
- 16-bit data output
- Interrupt generator
- I2C Address `0x1C` or `0x1E`
- 400 kHz max
- ``init()``:
    - set range to ``RANGE_4GS``
    - set ODR to ``ODR_155HZ``

```c++
LIS3MDL::gciLIS3MDL mag(&Wire);
mag.init();
bool ok = mag.ready(); // true when new data available
mag_t m = mag.read();
// m.x => mag x,y,z in uT
// m.ok => good read true/false
```

## BMP390

- ``init()``: sets up the sensor
    - Does a soft reset of sensor
    - Enables both pressure and temperature
        - pressure oversample 2x
        - temperature oversample 1x
    - Sets power mode to ``MODE_NORMAL`` or continous reading
    - ODR set to ``ODR_100_HZ``
    - IIR filter set to ``IIR_FILTER_COEFF_1``

```c++
BMP390::gciBMP390 bmp(&Wire);
bmp.init();
bool ok = bmp.ready(); // true when new data available
pt_t pt = bmp.read();
// pt.ok => good read true/false
// pt.press => pressure in
// pt.temp => temperature in C
```

## Other Stuff

- KF code, look at: /Users/kevin/tmp/inertial-navigation
- Eigen KF code at: /Users/kevin/tmp/kf-eigen
    - Probably not good for uC

## Todo

- [x] Breakout apple, linux and arduion implementations cleaner
- [x] LSM6DSOX Accel and gyro
- [x] LIS3MDL Magnetometer
- [x] BMP390 Barometer
- [ ] DPS310 Barometer

# MIT License

**Copyright (c) 2022 The Guild of Calamitous Intent**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
