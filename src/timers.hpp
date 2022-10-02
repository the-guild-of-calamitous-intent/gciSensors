#pragma once


/*
Calculates the time change since it was last called in seconds.
*/
class DT {
public:
  DT(): last(millis()) {}

  void touch() { last = millis(); }

  float now() {
    uint32_t n = millis();
    float dt = static_cast<float>(n - last) * 0.001f;
    last = n;
    return dt;
  }

protected:
  uint32_t last;
};


/*
Returns true after a set amount of time (in msec) has ellapsed.

Don't like the name ... Timer?
*/
class Alarm {
  public:
  Alarm(const uint32_t delaytime): epoch(millis()), dt(delaytime) {}

  bool check(){
    uint32_t now = millis();
    if (now > epoch){
      epoch = now + dt;
      return true;
    }
    return false;
  }

  protected:
  const uint32_t dt;
  uint32_t epoch;
};