#pragma once
#include <Arduino.h>

// --- SYSTEM ---
#define SERIAL_BAUD 115200
#define CONTROL_LOOP_HZ 100

// --- CAN BUS ---
#define CAN_CLOCK CLK_60MHz
#define CAN_BAUD_ARB 1000000
#define CAN_BAUD_DATA 5000000
#define CAN_SAMPLE_POINT 66.6

// --- MOTOR ID MAPPING ---
namespace MotorID {
    // Front Right
    constexpr int FR_SHOULDER = 1;
    constexpr int FR_HIP = 2;
    constexpr int FR_KNEE = 3;

    // Front Left
    constexpr int FL_SHOULDER = 4;
    constexpr int FL_HIP = 5;
    constexpr int FL_KNEE = 6;

    // Rear Right
    constexpr int RR_SHOULDER = 7;
    constexpr int RR_HIP = 8;
    constexpr int RR_KNEE = 9;

    // Rear Left
    constexpr int RL_SHOULDER = 10;
    constexpr int RL_HIP = 11;
    constexpr int RL_KNEE = 12;
}

// --- JOINT PROPERTIES ---
namespace JointProperties {
    // Gear Ratios
    constexpr float RATIO_SHOULDER = 17.0f;
    constexpr float RATIO_HIP = (56.0f / 18.0f) * (56.0f / 14.0f);
    constexpr float RATIO_KNEE = (56.0f / 18.0f) * (56.0f / 14.0f);

    // Joint Limits [rad]
    constexpr float SHOULDER_MIN = -0.65f;
    constexpr float SHOULDER_MAX = 0.65f;
    constexpr float HIP_MIN = 0.0f;
    constexpr float HIP_MAX = 0.0f;
    constexpr float KNEE_MIN = 0.0f;
    constexpr float KNEE_MAX = 0.0f;
}