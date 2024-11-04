#include <stdio.h>
#include <gciSensors.hpp>

using namespace std;
using namespace DPS310;
using namespace sensors;

int main() {
  gciDPS310 press(1);

  uint8_t ok = 1;
  while (ok != 0) {
    ok = press.init(DPS_128HZ);
    printf("... oops\n");
    sleep_ms(100);
  }

  printf("/// Press/Temp Started ///\n");

  while (1) {
    dps310_t ans = press.read();
    if (ans.ok == false) {
      sleep_ms(10);
      printf(".");
      continue;
    }
    printf("\n");

    float alt = pressure_altitude(ans.pressure);
    printf("Press: %8.1f Pa  Temp: %5.2f C  Alt: %7.1f m ", ans.pressure, ans.temperature, alt);
    sleep_ms(33);
  }
}