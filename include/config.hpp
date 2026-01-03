#pragma once
#include <Arduino.h>
#include <math.h>
#include "types.hpp"

// --- SYSTEM ---
#define SERIAL_BAUD 115200
#define CONTROL_LOOP_HZ 1000

// --- CAN BUS ---
#define CAN_CLOCK CLK_60MHz
#define CAN_BAUD_ARB 1000000
#define CAN_BAUD_DATA 5000000
#define CAN_SAMPLE_POINT 66.6

// --- MOTOR ID MAPPING ---
namespace MotorID {
    // Front Left
    constexpr int FL_SHOULDER = 1;
    constexpr int FL_HIP = 2;
    constexpr int FL_KNEE = 3;

    // Front Right
    constexpr int FR_SHOULDER = 4;
    constexpr int FR_HIP = 5;
    constexpr int FR_KNEE = 6;

    // Rear Left
    constexpr int RL_SHOULDER = 7;
    constexpr int RL_HIP = 8;
    constexpr int RL_KNEE = 9;

    // Rear Right
    constexpr int RR_SHOULDER = 10;
    constexpr int RR_HIP = 11;
    constexpr int RR_KNEE = 12;
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
    constexpr float HIP_MIN = -100.0f;
    constexpr float HIP_MAX = 100.0f;
    constexpr float KNEE_MIN = -100.0f;
    constexpr float KNEE_MAX = 100.0f;
}

// --- GEOMETRY ---
namespace Geometry {
    // Joint points in global frame, seated pose [mm]
    constexpr P3_XYZ ORIGIN = {0.0f, 0.0f, 0.0f};
    constexpr P3_XYZ FL_ORIGIN = {3.500f, 68.000f, 0.000f};
    constexpr P3_XYZ FL_SHOULDER = {62.500f, 68.000f, 0.000f};
    constexpr P3_XYZ FL_HIP = {176.500f, 84.393f, 0.000f};
    constexpr P3_XYZ FL_KNEE = {28.500f, 139.828, 0.000f}; 
    constexpr P3_XYZ FL_FOOT = {204.500f, 147.893f, 0.000f};

    // Lengths [mm]
    constexpr float L_SHOULDER = abs(176.500f - 62.500f); // Shoulder to Hip
    constexpr float L_HIP = abs(28.500f - 176.500f); // Hip to Knee
    constexpr float L_KNEE = abs(204.500f - 28.500f); // Knee to Foot
}