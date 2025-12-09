#include <Arduino.h>
#include "Config.h"
#include "Bus.h"
#include "Body.h"

uint32_t last_ctrl_us = 0;
uint32_t last_print_us = 0;

void setup() {

    // initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    // initialize the bus
    Bus::i().begin(BAUD_RATE);

    // initialize the body
    Body::i().begin();
    Body::i().setAllLegMode(LegMode::POSITION);

    uint32_t now = micros();
    last_ctrl_us  = now;
    last_print_us = now;
}

void loop() {
    Bus::i().poll();
    uint32_t now_us = micros();

    // control loop
    if (now_us - last_ctrl_us >= static_cast<uint32_t>(CTRL_DT_US)) {
        last_ctrl_us   = now_us;        
        // example: simple sinusoid 
        // ----------------------------------------
        float t = now_us * 1e-6f;
        float q[DOF_PER_LEG];
        q[0] = 19.0f / 2.0f * PI * sinf(1.0f * PI * t); // hip
        // q[1] = 0.4f * sinf(2.0f * PI * 0.2f * t); // knee
        // Serial.print("q_input: ");
        // Serial.println(q[0]);
        // Serial.println(", ");
        // Serial.println(q[1]);
        Body::i().setLegPosRef(0, q); // leg 0
        // ----------------------------------------

        Body::i().update();
    }

    // printing
    if (now_us - last_print_us >= static_cast<uint32_t>(PRINT_DT_US)) {
        last_print_us = now_us;

        // for leg 0 only
        
        // print joint states
        JointState js[DOF_PER_LEG];
        Body::i().getLegJointState(0, js);

        Serial.print("Leg 0 Joints:");
        for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
            Serial.print(" [");
            Serial.print(i);
            Serial.print("] q=");
            Serial.print(js[i].q, 4);
            Serial.print(" dq=");
            Serial.println(js[i].dq, 4);
        }

        // print foot position
        // float p[3];
        // Body::i().getLegFootPos(0, p);

        // Serial.print(" Foot Pos: x=");
        // Serial.print(p[0], 4);
        // Serial.print(" y=");
        // Serial.print(p[1], 4);
        // Serial.print(" z=");
        // Serial.print(p[2], 4);
        // Serial.println();
    }
}