#include <Arduino.h>
#include "dog.hpp"
#include "kinematics.hpp"

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

  // FK
  Serial.print("Foot Pos: (");
  Serial.print(state.fl.foot.position.x);
  Serial.print(", ");
  Serial.print(state.fl.foot.position.y);
  Serial.print(", ");
  Serial.print(state.fl.foot.position.z);
  Serial.print(") mm ");

  Serial.print(" | Leg Q: (");
  Serial.print(state.fl.shoulder.position);
  Serial.print(", ");
  Serial.print(state.fl.hip.position);
  Serial.print(", ");
  Serial.print(state.fl.knee.position);
  Serial.println(") rad");

  // Sinusoidal motion example
  // float t = now / 1000.0f;

  //Shoulder
  // cmd.fl.shoulder.p_des = 0.5f * sinf(2.0f * PI * 0.5f * t);
  // cmd.fl.shoulder.kp = 1.0f;
  // cmd.fl.shoulder.kd = 1.0f;
  // cmd.fl.shoulder.max_torque = 85.0f;

  //Hip
  // cmd.fl.hip.p_des = 0.5f * cosf(2.0f * PI * 0.5f * t);
  // cmd.fl.hip.kp = 1.0f;
  // cmd.fl.hip.kd = 1.0f;
  // cmd.fl.hip.max_torque = 60.0f;
  
  //Knee
  // cmd.fl.knee.p_des = 0.5f * sinf(2.0f * PI * 0.5f * t);
  // cmd.fl.knee.kp = 1.0f;
  // cmd.fl.knee.kd = 1.0f;
  // cmd.fl.knee.max_torque = 60.0f;

  dog.setCommand(cmd);
}