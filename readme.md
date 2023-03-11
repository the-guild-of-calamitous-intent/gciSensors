# gciSensors

## Why?

I love Adafruit sensors with QWIIC but their drivers are geared towards hobbiest and
favor low power consumption over performace. I try to strip out unnecessary abstraction
and enable high data rate.

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
pt_t pt = bmp.read();
// pt.ok => good read true/false
// pt.press => pressure in
// pt.temp => temperature in C
```



## Filters (`filters.hpp`)

- Low/High pass filter

## Helpers

- Unit conversions `units.hpp`
- Earth parameters `earth.hpp`

## Apple and Linux

To use mock serial for testing, do:

```cpp
#define USE_GCIMOCK_SERIAL 1
```

Else do:

```cpp
#include <serialcomm>
SerialPort Serial;
```

## Other Stuff

- KF code, look at: /Users/kevin/tmp/inertial-navigation
- Eigen KF code at: /Users/kevin/tmp/kf-eigen
    - Probably not good for uC

## Todo

- [ ] convert `Alarm` from reading `millis()` to using HW timer
- [ ] should `timer.hpp`, `earth.hpp`, or `units.hpp` be in this library? maybe move to `gecko2` (or what ever)
- [x] Breakout apple, linux and arduion implementations cleaner
- [x] LSM6DSOX Accel and gyro
- [x] LIS3MDL Magnetometer
- [x] BMP390 Barometer
- [ ] DPS310 Barometer
- [x] TFmini LIDAR

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
