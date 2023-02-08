#pragma once

// https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Arduino.h

#include <cmath>
#include <stdint.h>
#include <stddef.h>
#include <string>

// using namespace std;
using std::string;
using std::to_string;

static void pinMode(uint8_t pin, uint8_t mode) {}
static void digitalWrite(uint8_t pin, uint8_t val) {}
static int digitalRead(uint8_t pin) {return 0;}
static int analogRead(uint8_t pin) {return 0;}
static void analogReference(uint8_t mode) {}
static void analogWrite(uint8_t pin, int32_t val) {}

static uint32_t millis(void) {return 0;}
static uint32_t micros(void) {return 0;}
static void delay(uint32_t ms) {}
static void delayMicroseconds(uint32_t us) {}


struct SerialPort {
  void begin(int r) {}
  void print(string a) {}
  void print(float a, int v=0) {}
  void println(float a, int v=0) {}
  void println(string a) {}
  void setTimeout(int) {}
  int available() { return 1; }
  int read() { return 60; }
  int write(uint8_t*,int sz) { return sz; }
  int write(uint8_t) { return 1; }

  // this isn't real, so always return "true"
  inline explicit operator bool() const noexcept {return true;}
};

extern SerialPort Serial;
extern SerialPort Serial1;
extern SerialPort Serial2;
