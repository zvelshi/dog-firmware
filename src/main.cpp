#include <Arduino.h>
#include "config.h"
#include "Bus.h"
#include "Joint.h"
#include "Kinematics.h"
#include "Leg.h"

Joint joints[DOF_PER_LEG] = {
    Joint(AXIS_IDS[0][0]),
    Joint(AXIS_IDS[0][1])
};

Kinematics kin(L1, L2);
Leg leg(joints, &kin);

uint32_t last_ctrl_us = 0;
uint32_t last_fdbk_us = 0;
uint32_t last_print_us = 0;

void setup() {

    // initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    // initialize the bus
    Bus::i().begin(BAUD_RATE);

    // initialize joints
    // auto-calibrate if config says not calibrated
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        joints[i].begin();
    }

    // initialize leg
    leg.begin();
    leg.setMode(LegMode::IDLE);

    uint32_t now = micros();
    last_ctrl_us  = now;
    last_fdbk_us  = now;
    last_print_us = now;
}

void loop() {
    Bus::i().poll();
    uint32_t now_us = micros();

    // control loop
    if (now_us - last_ctrl_us >= static_cast<uint32_t>(CTRL_DT_US)) {
        uint32_t dt_us = now_us - last_ctrl_us;
        last_ctrl_us   = now_us;

        float dt = dt_us * 1e-6f;
        leg.update(dt);
    }

    // feedback requests
    if (now_us - last_fdbk_us >= static_cast<uint32_t>(FDBK_DT_US)) {
        last_fdbk_us = now_us;
        for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
            joints[i].update();
        }
    }

    // printing
    if (now_us - last_print_us >= static_cast<uint32_t>(PRINT_DT_US)) {
        last_print_us = now_us;

        const JointState j0 = joints[0].state();
        const JointState j1 = joints[1].state();

        Serial.print("q0: ");
        Serial.print(j0.q, 3);
        Serial.print(" dq0: ");
        Serial.print(j0.dq, 3);
        Serial.print(" | q1: ");
        Serial.print(j1.q, 3);
        Serial.print(" dq1: ");
        Serial.print(j1.dq, 3);

        float p[3];
        float angles[DOF_PER_LEG] = { j0.q, j1.q };
        kin.fk(angles, p);

        Serial.print(" | foot x: ");
        Serial.print(p[0], 3);
        Serial.print(" y: ");
        Serial.print(p[1], 3);
        Serial.print(" z: ");
        Serial.print(p[2], 3);
        Serial.println();
    }
}