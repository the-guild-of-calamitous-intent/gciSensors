#include <stdio.h>
#include <gciSensors.hpp>

using namespace std;
using namespace BMP390;
using namespace gci::sensors;

gciBMP390 bmp;

int main() {
  printf("/// Press/Temp Started ///\n");

  while (true) {
    uint err = bmp.init(ODR_100_HZ, IIR_FILTER_COEFF_127);
    if (err == 0) break;
    printf("BMP Error: %u\n", err);
    sleep_ms(1000);
  }

  while (1) {
    bmp390_t pt = bmp.read();
    if (pt.ok == false) {
      printf("oops ...\n");
      sleep_ms(100);
      continue;
    }

    printf("-----------------------------\n");
    printf("Pressure: %f Pa\n", pt.press);
    printf("Temperature: %f C\n", pt.temp);
    printf("Altitude: %6.2f m\n", altitude(pt.press));

    sleep_ms(10);
  }
}