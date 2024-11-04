/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once


namespace sensors {

#if defined(__linux__) // FIXME: handle better
#include <unistd.h>
inline void sleep_ms(uint32_t ms) { usleep(ms*1000); }
inline void sleep_us(uint64_t us) { usleep(us); }
// void sleep(uint32_t sec) - already defined

#elif defined(ARDUINO)
inline void sleep_ms(uint32_t ms) { delay(ms); }
inline void sleep_us(uint64_t us) { delayMicroseconds(us); }
inline void sleep(uint32_t sec) { delay(1000*sec); }

#elif defined(PICO_BOARD)
inline uint32_t now_ms() {
  absolute_time_t t = get_absolute_time();
  return to_ms_since_boot(t);
}
#endif

} // end namespace sensors