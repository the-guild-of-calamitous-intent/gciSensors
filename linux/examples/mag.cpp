#include <stdio.h>
#include <gciSensors.hpp>

using namespace std;
using namespace LIS3MDL;
using namespace sensors;

gciLIS3MDL mag(1);

int main() {
  printf("/// Mag Started ///\n");

  while (true) {
    int err = mag.init(RANGE_4GAUSS,ODR_155HZ);
    if (err == 0) break;
    printf("mag error: %d\n", err);
    sleep_ms(1000);
  }

  while (1) {
    const lis3mdl_t m = mag.read_cal();
    if (m.ok == false) continue;

    printf("Mags: %f %f %f (normalized)\n", m.x, m.y, m.z);
    // printf("Temperature: %f C\n", m.temperature);
  }
}