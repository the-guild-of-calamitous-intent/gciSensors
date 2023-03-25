#include <gciSensors.hpp>

const int interruptPin = digitalPinToInterrupt(10); // A2
using namespace LSM6DSOX;
volatile bool interrupt = false;

// gciLSM6DSOX IMU(&Wire, LSM6DSOX_ADDRESS); // if not using default address
gciLSM6DSOX IMU(&Wire);

// interrupt service routine
void isr() {
  interrupt = true;
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  while (!IMU.init()) {
    Serial.println("imu error");
    delay (1000);
  }

  // setup external interrupt
  // INPUT_PULLUP
  pinMode(interruptPin, INPUT_PULLUP);
  /*
  LOW to trigger the interrupt whenever the pin is low,
  CHANGE to trigger the interrupt whenever the pin changes value
  RISING to trigger when the pin goes from low to high,
  FALLING for when the pin goes from high to low.
  HIGH to trigger the interrupt whenever the pin is high.
  */
  // attachInterrupt(interruptPin, isr, RISING);
  attachInterrupt(interruptPin, isr, FALLING);
}

void loop() {
  if (interrupt) {
    sox_t s = IMU.read();
    Serial.print(s.ok? "good ":"error ");
    Serial.print(s.ax,6);
    Serial.print(" ");
    Serial.print(s.ay,6);
    Serial.print(" ");
    Serial.print(s.az,6);
    Serial.print(" g's ");
    Serial.print(s.gx,6);
    Serial.print(" ");
    Serial.print(s.gy,6);
    Serial.print(" ");
    Serial.print(s.gz,6);
    Serial.print(" rps  ");
    Serial.print(s.temp,2);
    Serial.print(" ");
    Serial.println(s.ts);

    interrupt = false;
  }

  // delay(5);
  delay(1000);
  Serial.println("loop ---------");
}
