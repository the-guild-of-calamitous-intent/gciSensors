
// Super simple test ... if this doesn't work,
// something is really broke
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

int main() {
  stdio_init_all();

  while(1) {
    puts("hello");
    sleep_ms(1000);
  }
}