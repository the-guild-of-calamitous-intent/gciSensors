#pragma once

// https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Arduino.h

#include <cmath>
#include <stdint.h>
#include <stddef.h>

void pinMode(uint8_t pin, uint8_t mode) {}
void digitalWrite(uint8_t pin, uint8_t val) {}
int digitalRead(uint8_t pin) {return 0;}
int analogRead(uint8_t pin) {return 0;}
void analogReference(uint8_t mode) {}
void analogWrite(uint8_t pin, int32_t val) {}

uint32_t millis(void) {return 0;}
uint32_t micros(void) {return 0;}
void delay(uint32_t ms) {}
void delayMicroseconds(uint32_t us) {}



// class Serial {
//   public:
//   void print()
// }