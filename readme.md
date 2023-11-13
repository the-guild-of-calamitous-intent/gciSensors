# gciSensors

## Why?

I love Adafruit sensors with QWIIC but their drivers are geared towards hobbiest and
favor low power consumption over performace. I try to strip out unnecessary abstraction
and enable high data rate.

## Bit Stuff

```c
N |= 1 << K;    // set bit
n &= ~(1 << k); // clear bit
n ^= 1 << k;    // toggle bit
```

## Units

| Sensor Type | Units        | Abbreviation
|-------------|--------------|--------------|
| Accels      | gravity      | g
| Gyros       | degrees/sec  | dps
| Mags        | micro Teslas | uT
| Temperature | Celcius      | C
| Pressure    | Pascal       | Pa
| Time        | seconds      | sec

## LSM6DSOX

- Accelerometer ±2/±4/±8/±16 g at 1.6 Hz to 6.7KHz update rate
- Gyroscope: ±125/±250/±500/±1000/±2000 dps at 12.5 Hz to 6.7 KHz
- 32 bit timer at 25 usec resolution
- I2C Address `0x6A` or `0x6B`
- ``init()``:
    - set accel to ``RATE_104_HZ`` ``ACCEL_RANGE_4_G``
    - sets gyro to ``RATE_104_HZ`` ``GYRO_RANGE_2000_DPS``

```c++
LSM6DSOX::gciLSM6DSOX imu;
imu.init_tw(I2C_400KHZ); // setup i2c
imu.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ); // setup sensor
bool ok = imu.ready(); // true when new data available
if (ok) sox_t s = imu.read();
// s.ok => good read true/false
// s.ax => accel x,y,z in g's
// s.gx => gyro x,y,z in deg/sec
// s.temp => temperature in C
```

## LIS3MDL

> I am having issues with this sensor. I think it must be calibrated to produce
> any useable data. No matter what I do, I can not use it as a compass ... I
> am doing something wrong either here or in a filter that turns the measurements
> into heading!

- ±4/±8/±12/±16 gauss selectable magnetic full scales
- 16-bit data output
- Interrupt generator
- I2C Address `0x1C` or `0x1E`
- 400 kHz max
- ``init()``:
    - set range to ``RANGE_4GS``
    - set ODR to ``ODR_155HZ``

```c++
LIS3MDL::gciLIS3MDL mag;
mag.init_tw(I2C_400KHZ); // setup i2c
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
BMP390::gciBMP390 bmp;
bmp.init_tw(I2C_400KHZ); // setup i2c
bmp.init();
bool ok = bmp.ready(); // true when new data available
pt_t pt = bmp.read();
// pt.ok => good read true/false
// pt.press => 24-bit, pressure in Pa
// pt.temp => 24-bit, temperature in C
```

## Filters

So these aren't working too great right now ... mainly due to the
magnitometer issue

- [AHRS](https://ahrs.readthedocs.io/en/latest/filters.html)

## Pi Pico

```
[100%] Linking CXX executable press.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       28156 B         2 MB      1.34%
             RAM:        9620 B       256 KB      3.67%
       SCRATCH_X:          0 GB         4 KB      0.00%
       SCRATCH_Y:          0 GB         4 KB      0.00%
-------------------------------------
File press.uf2:

Program Information
 name:          press
 features:      USB stdin / stdout
 binary start:  0x10000000
 binary end:    0x10006dfc

Fixed Pin Information
 none

Build Information
 sdk version:       1.5.1
 pico_board:        pico
 boot2_name:        boot2_w25q080
 build date:        Oct 22 2023
 build attributes:  Release
-------------------------------------
```

## Other Stuff

- KF code, look at: /Users/kevin/tmp/inertial-navigation
- Eigen KF code at: /Users/kevin/tmp/kf-eigen
    - Probably not good for uC

| Type     | Significant digits | Number of bytes
|----------|--------------------|-----------------|
| `float`  | 6 - 7              | 4
| `double` | 15 - 16            | 8

- Ref: [microsoft.com](https://learn.microsoft.com/en-us/cpp/c-language/type-float?view=msvc-170#floating-point-types)

## Todo

- [ ] Breakout apple, linux and arduion implementations cleaner
- [x] LSM6DSOX Accel and gyro
- [x] LIS3MDL Magnetometer
- [x] BMP390 Barometer
- [ ] DPS310 Barometer
- [ ] Add unit tests
- [ ] Add some simple filters that use these sensors
- [ ] Update Arduino examples
- [ ] Do a better job of cross platform dev/test (apple, linux, arduino)
- [ ] Investigate fixed point math for calibration, probably FP(16,16) would work
      nicely since the range [-32k,+32k] is good enough for sensors
      - accels: +/- 16 g
      - gyros: +/- 2000 dps
      - mags: +/- 1 unitless (every compase algorithm normalizes this first)
      - pressure: 300-1010 hPa (only needed for converting to altitude, 0-8km (5mi))
          - Mt Everest is around ~5.5mi high, ~313 hPa
      - temperature: 0-100 C (no calibration to do here)

## Filter

```
FilteredAngle_k = alpha * GyroscopeAngle + (1 − alpha) * AccelerometerAngle
alpha = tau / (tau + dt)
GyroscopeAngle = FilteredAngle_k-1 + w * dt
```

dt = sampling rate, tau = time constant greater than timescale of typical accelerometer noise

I had a sampling rate of about 0.04 seconds and chose a time constant of about 1 second, giving alpha = 0.96. [ref](https://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/)

## References

- Alternative: [Betafight](https://github.com/betaflight/betaflight/tree/master)
- ozzmaker.com: [Compass1](https://ozzmaker.com/compass1/)
- ozzmaker.com: [Compass2](https://ozzmaker.com/compass2/)
- [Raspberry Pico Libs](https://github.com/earlephilhower/arduino-pico/tree/master)
- STMicro github: [lsm6dsox](https://github.com/STMicroelectronics/lsm6dsox-pid/tree/master)

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
