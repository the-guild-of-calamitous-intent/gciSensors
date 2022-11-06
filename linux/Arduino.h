#pragma once

// https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Arduino.h

#include <cmath> // math functions
#include <stdint.h> // int types
// #include <stddef.h>
#include <unistd.h> // usleep
#include <iostream>
#include <string>

using std::string;
using std::to_string;
using std::cout;
using std::endl;

static void pinMode(uint8_t pin, uint8_t mode) {}
static void digitalWrite(uint8_t pin, uint8_t val) {}
static int digitalRead(uint8_t pin) {return 0;}
static int analogRead(uint8_t pin) {return 0;}
static void analogReference(uint8_t mode) {}
static void analogWrite(uint8_t pin, int32_t val) {}

static uint32_t millis(void) {return 0;}
static uint32_t micros(void) {return 0;}
static void delay(uint32_t ms) {usleep(1000*ms);}
static void delayMicroseconds(uint32_t us) {usleep(us);}



struct SerialPort {
  void begin(int r) {}

  // void print(int a) {
  //   std::cout << a;
  // }

  void print(string a) {
    cout << a;
  }

  void print(float a, int v=0) {
    cout << a;
  }

  // void println(unsigned long a) {
  //   std::cout << a << std::endl;
  // }

  // void println(int a) {
  //   std::cout << a << std::endl;
  // }

  void println(float a, int v=0) {
    cout << a << endl;
  }

  void println(string a) {
    cout << a << endl;
  }

  // this isn't real, so always return "true"
  inline explicit operator bool() const noexcept {return true;}
};

extern SerialPort Serial;