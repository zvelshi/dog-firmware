#include "Arduino.h"
#include "dog.h"
#include "trajectory.h"
#include "kinematics.h"

Dog dog;
DogCommand cmd;
MinJerkGenerator traj_gen;

void setup() {
  dog.begin();

  DogState initial_state = dog.getState();
  
  Vector3f start_pos = initial_state.legs[0].foot.position;;
  traj_gen.reset(start_pos);
  traj_gen.setLimits(0.2f, 2.0f);

  Vector3f p0 = start_pos;
  Vector3f p1 = p0 + Vector3f(0.0f, 0.0f, 0.05f);
  Vector3f p2 = p1 + Vector3f(0.1f, 0.0f, 0.0f);
  Vector3f p3 = p2 + Vector3f(0.0f, 0.0f, -0.05f);

  traj_gen.addWaypoint(p1);
  traj_gen.addWaypoint(p2);
  traj_gen.addWaypoint(p3);
  traj_gen.addWaypoint(p0);
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t now = micros();
  float dt = (now - last_time) / 1000000.0f;
  if (dt < (1.0f / CONTROL_LOOP_HZ)) return;
  last_time = now;

  /* Start of Looping Code */

  // Check if trajectory is finished
  if (traj_gen.isFinished()) return;

  // Update trajectory generator
  CartesianState des_cart = traj_gen.update(dt);

  // Inverse kinematics to get joint positions
  Vector3f q_des = Kinematics::inverseKinematics(des_cart.p);

  // Compute joint velocities via Jacobian
  Vector3f qd_des = Kinematics::cartesianToJointVelocity(q_des, des_cart.v);

  // Set commands for front-left leg (leg index 0)
  for (int i = 0; i < 3; i++) {
    cmd.legs[0].joints[i].p_des = q_des(i);
    cmd.legs[0].joints[i].v_des = qd_des(i);
  }

  /* End of Looping Code */

  dog.setCommand(cmd);
}