#include <stdio.h>
#include <gciSensors.hpp>

using namespace std;
using namespace DPS310;

int main() {
  gciDPS310 press;

  bool ok = false;
  while (ok == false) {
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

    float alt = altitude(ans.pressure);
    printf("Press: %8.1f Pa  Temp: %5.2f C  Alt: %7.1f m ", ans.pressure, ans.temperature, alt);
    sleep_ms(33);
  }
}