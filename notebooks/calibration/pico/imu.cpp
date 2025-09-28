#include <stdio.h>


// #define __USE_SENSOR_LSM6DSOX__
// #define __USE_SENSOR_BMP390__
// #define __USE_SENSOR_DPS310__

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "tusb.h" // wait for USB
#include <gciSensors.hpp>
// #include "two_wire.hpp"

constexpr uint i2c_port = 0;
constexpr uint i2c_scl  = 1; // I2C0_SCL_PIN;
constexpr uint i2c_sda  = 0; // I2C0_SDA_PIN;
// constexpr uint LED_PIN  = 25;


using namespace std;
using namespace LIS3MDL;
using namespace LSM6DSOX;
using namespace sensors;


struct __attribute__((packed)) imu_t {
  sensors::vec_t a,g,m;
  float temperature;
  uint64_t timestamp;
};


void write_stdout(const uint8_t *buffer, size_t size) {
  // if (tud_cdc_connected() == false) return;

  size_t sent = 0;
  while (size > 0) {
    sent += fwrite(&buffer[sent], 1, size, stdout);
    size -= sent;
  }

  // char c = '\n';
  // fwrite(&c,1,1,stdout); // send '\n' to flush
}


int main() {
  stdio_init_all();

  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint speed = i2c_bus_init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  // gpio_init(LED_PIN);
  // gpio_set_dir(LED_PIN, GPIO_OUT);

  printf("/// Accel/Gyros/Mag START ///\n");


  gciLIS3MDL mag(i2c_port);
  gciLSM6DSOX IMU(i2c_port);

  while (true) {
    uint8_t err = IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) break;
    printf("imu error %d\n", (int)err);
    sleep_ms(1000);
  }
  while (true) {
    int err = mag.init(RANGE_4GAUSS,ODR_300HZ);
    if (err == 0) break;
    printf("mag error: %d\n", err);
    sleep_ms(1000);
  }

  imu_t imu;
  char header[2]{'A','A'};

  while (1) {
    // gpio_put(LED_PIN, 0);
    // sleep_ms(500);
    // gpio_put(LED_PIN, 1);
    // sleep_ms(500);

    lsm6dsox_t i = IMU.read();
    if (i.ok == true) {
      // printf("-----------------------------\n");
      // printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
      // printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
      // printf("Temperature: %f C\n", i.temperature);
      // printf("Timestamp: %llu msec\n", i.timestamp_us);

      imu.a.x = i.a.x;
      imu.a.y = i.a.y;
      imu.a.z = i.a.z;
      imu.g.x = i.g.x;
      imu.g.y = i.g.y;
      imu.g.z = i.g.z;
      imu.timestamp = i.timestamp_us;
      imu.temperature = i.temperature;
    }

    const lis3mdl_t m = mag.read_cal();
    if (m.ok == true) {
      // printf("Mags: %f %f %f (normalized)\n", m.x, m.y, m.z);
      imu.m.x = m.x;
      imu.m.y = m.y;
      imu.m.z = m.z;
    }

    if (m.ok || i.ok) {
      // write_stdout(static_cast<uint8_t*>(&imu), sizeof(imu_t));
      write_stdout(reinterpret_cast<uint8_t*>(header), sizeof(header));
      // write_stdout((uint8_t*)&imu, sizeof(imu_t));
      write_stdout(reinterpret_cast<uint8_t*>(&imu), sizeof(imu_t));
    }

    sleep_ms(2);
  }
}