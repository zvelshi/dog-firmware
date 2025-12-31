#include <Arduino.h>
#include "dog.hpp"

Dog dog;
DogCommand cmd;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  dog.begin();
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t now = millis();
  if (now - last_time < (1000 / CONTROL_LOOP_HZ)) {
    return;
  }
  last_time = now;

  DogState state = dog.getState();

  Serial.print("Pos: ");
  Serial.println(state.fr.shoulder.position, 3);

  // Sinusoidal motion example
  float t = now / 1000.0f;
  cmd.fr.shoulder.p_des = 0.60f * sinf(2.0f * PI * 0.33f * t);
  cmd.fr.shoulder.kp = 1.0f;
  cmd.fr.shoulder.kd = 1.0f;
  cmd.fr.shoulder.max_torque = 85.0f;

  dog.setCommand(cmd);
}