
#pragma once

class Hertz {
  public:
  Hertz(uint32_t v): val(v), epoch(millis()) {}

  bool check() {
    if (++count % val == 0) {
      uint32_t now = millis();
      hertz = 1000.0f * float(count) / float(now - epoch);
      epoch = now;
      count = 0;
      return true;
    }

    return false;
  }

  float hertz{0.0f};

  protected:
  uint32_t epoch;
  uint32_t count{0};
  uint32_t val;
};