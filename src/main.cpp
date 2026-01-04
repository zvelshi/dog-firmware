#include "Arduino.h"
#include "dog.h"

Dog dog;
DogCommand cmd;

void setup() {
  dog.begin();
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t now = micros();
  if (now - last_time < (1000000 / CONTROL_LOOP_HZ)) {
    return;
  }
  last_time = now;

  DogState state = dog.getState();

  /* Start of Looping Code */

  /* End of Looping Code */

  dog.setCommand(cmd);
}