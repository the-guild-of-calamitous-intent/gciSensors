// #include <stdio.h>


#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB
#include <gciSensors.hpp>
#include "two_wire.hpp"
#include <array>

constexpr uint i2c_port = 0;
constexpr uint i2c_scl = 1; // I2C0_SCL_PIN;
constexpr uint i2c_sda = 0; // I2C0_SDA_PIN;

using namespace std;
using namespace LSM6DSOX;
using namespace BMP390;
using namespace gci::sensors;

template<std::size_t N>
class StaticCalibrate {
  std::array<float[3],N> buffer;
  float x{0.0f}, y{0.0f}, z{0.0f};
  size_t cnt{0};

  public:
  void push(float x, float y, float z) {
    buffer[cnt][0] = x;
    buffer[cnt][1] = y;
    buffer[cnt++][2] = z;
  }

  void calibrate() {
    float size = (float)buffer.size();
    x = y = z = 0.0f;
    for (size_t i=buffer.size()-1; i >= 0; --i) {
      x += buffer[i][0];
      y += buffer[i][1];
      z += buffer[i][2];
    }
    x /= size;
    y /= size;
    z /= size;
  }

  const size_t size() const { return buffer.size(); }

  const float get_x() const { return x; }
  const float get_y() const { return y; }
  const float get_z() const { return z; }
};

struct nav_t {
  vec_t a, g, m;
  quat_t q;
  float pressure;
  float temperature;
  float altitude;
  float heading;
  uint32_t timestamp;
} state;

gciLSM6DSOX IMU(i2c_port);
gciBMP390 bmp(i2c_port);

StaticCalibrate<256> cal;
float bx, by, bz;

void setup() {
  // setup serial IO to computer
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  // initialize I2C
  uint speed = i2c_bus_init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);
  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  // configure IMU sensor
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
    uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_104_HZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_416_HZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_833_HZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_1_66_KHZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_3_33_KHZ);
    // uint8_t err = IMU.init(ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS, RATE_6_66_KHZ);
    if (err == 0) break;
    printf("imu error %d\n", (int)err);
    sleep_ms(500);
  }

  for (size_t i=0; i<cal.size(); ++i) {
    lsm6dsox_t ret = IMU.read();
    while (ret.ok == false) {
      ret = IMU.read();
      sleep_ms(100);
      printf(".");
    }
    cal.push(ret.a.x, ret.a.y, ret.a.z);

    // state.a.x = ret.a.x;
    // state.a.y = ret.a.y;
    // state.a.z = ret.a.z;

    // state.g.x = ret.g.x;
    // state.g.y = ret.g.y;
    // state.g.z = ret.g.z;
    sleep_ms(1);
  }
  cal.calibrate();
  bx = cal.get_x();
  by = cal.get_y();
  bz = cal.get_z();

  printf("IMU configured: %.3f %.3f %.3f\n", bx, by, bz);

  // configure pressure/temperature sensor
  while (true) {
    uint err = bmp.init(
      ODR_100_HZ,
      IIR_FILTER_COEFF_127);
    if (err == 0) break;
    printf("BMP Error: %u\n", err);
    sleep_ms(500);
  }
}

void get_imu() {
  lsm6dsox_t ret = IMU.read();
  if (ret.ok == false) return;

  state.a.x = ret.a.x;
  state.a.y = ret.a.y;
  state.a.z = ret.a.z;

  state.g.x = ret.g.x;
  state.g.y = ret.g.y;
  state.g.z = ret.g.z;

  // printf("Accels: %6.3f %6.3f %6.3f\n", ret.a.x, ret.a.y, ret.a.z);
  // printf("Gyros: %f %f %f dps\n", ret.g.x, ret.g.y, ret.g.z);
  // printf("Temperature: %f C\n", ret.temperature);
  // printf("Timestamp: %lu msec\n", ret.ts);
}

void get_bmp() {
  bmp390_t ans = bmp.read();
  if (ans.ok == false) return;

  float alt = pressure_altitude(ans.pressure);
  // printf("Press: %8.1f Pa  Temp: %5.2f C  Alt: %7.1f m\n",
  //   ans.pressure,
  //   ans.temperature,
  //   alt);
}

void loop() {
  get_imu();
  get_bmp();

  sleep_ms(10);
}

// main loop, doesn't do much
int main() {
  setup();
  while(1) loop();
  return 0;
}