/*
*/


#include <stdio.h>

using namespace std;

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB
#include <gciSensors.hpp>
#include "two_wire.hpp"

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = 1; // I2C0_SCL_PIN;
constexpr uint i2c_sda = 0; // I2C0_SDA_PIN;

using namespace LSM6DSOX;
using namespace gci::sensors;

// TwoWire tw;
gciLSM6DSOX IMU(i2c_port);

void average(lsm6dsox_t ave, lsm6dsox_t d, uint8_t window, uint8_t cnt) {
  for (size_t i=0; i<3; ++i) {
    ave.a[i] += d.a[i];
    ave.g[i] += d.g[i];
  }
  const float win = 1.0f / float(window);
  if (cnt % window == 0) {
    for (size_t i=0; i<3; ++i) {
      ave.a[i] *= win;
      ave.g[i] *= win;
    }
  }
}


// First-order low-pass filter structure
typedef struct {
    float alpha;  // Smoothing factor
    float y_prev; // Previous output
} LowPassFilter;

// Initialize the low-pass filter
void initLowPassFilter(LowPassFilter *filter, float cutoffFrequency, float samplingFrequency) {
    float tau = 1.0f / (2.0f * 3.141592653589793f * cutoffFrequency); // Time constant
    filter->alpha = 1.0f / (1.0f + tau * samplingFrequency);
    filter->y_prev = 0.0f;
}

// Apply the low-pass filter to a new input sample
float filterSample(LowPassFilter *filter, float input) {
    // Update the filter
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->y_prev;

    // Save the current output for the next iteration
    filter->y_prev = output;

    return output;
}

int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  // uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);
  uint speed = i2c_bus_init(i2c_port, I2C_1MHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// Accel/Gyros START ///\n");

  while (true) {
    printf("Configure IMU\n");
    // Betaflight values
    // Accel:
    //   833Hz ODR, 16G, use LPF1 output
    //   high performance mode
    // Gyro:
    //   6664Hz ODR, 2000dps
    //   LPF1 cutoff 335.5Hz
    //
    // This seems to top out around ~1900Hz in this loop
    //
    uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_416_HZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_833_HZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_1_66_KHZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_3_33_KHZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_6_66_KHZ);
    if (err == 0) break;
    printf("imu error %d\n", (int)err);
    sleep_ms(1000);
  }
  printf("IMU configured\n");

  lsm6dsox_t ave;
  Hertz hz(1000);

  LowPassFilter lpfx;
  initLowPassFilter(&lpfx, 100, 833);
  LowPassFilter lpfy;
  initLowPassFilter(&lpfy, 100, 833);
  LowPassFilter lpfz;
  initLowPassFilter(&lpfz, 100, 833);

  while (1) {
    lsm6dsox_t i = IMU.read();
    if (i.ok == false) {
      // printf(".");
      continue;
    }

    // average(ave, i, 10, cnt++);
    // if (cnt != 0) continue;
    ave.a.x = filterSample(&lpfx, i.a.x);
    ave.a.y = filterSample(&lpfy, i.a.y);
    ave.a.z = filterSample(&lpfz, i.a.z);

    hz.check();

    // sleep_ms(10);

    // printf("-----------------------------\n");
    printf("Accels: %6.3f %6.3f %6.3f Hz: %.2f\r", ave.a.x, ave.a.y, ave.a.z, hz.hertz);
    // printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
    // printf("Temperature: %f C\n", i.temperature);
    // printf("Timestamp: %lu msec\n", i.ts);
  }
}