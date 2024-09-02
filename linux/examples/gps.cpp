#include <stdio.h>
#include <gciSensors.hpp>


using namespace std;
using namespace gci::sensors;
using namespace PA1010D;

int main() {
  gciPA1010D gps;

  gps.write(PMTK::FULL_POWER, sizeof(PMTK::FULL_POWER));
  gps.write(PMTK::RMCGGAGSA, sizeof(PMTK::RMCGGAGSA));

  printf("///--- GPS Started ---///\n");

  char nema[250];

  while (1) {
    uint32_t num = gps.read(nema,250);
    if (num > 0) printf("GPS[%d]: %s\n", num, nema);
    else printf("*** Bad read ***\n");
    sleep_ms(100);
  }
}