#pragma once
#include <Arduino.h>
#include "types.h"
#include "utils.h"

// --- SYSTEM ---
#define SERIAL_BAUD 115200
#define CONTROL_LOOP_HZ 500

// --- CAN BUS ---
#define CAN_CLOCK CLK_60MHz
#define CAN_BAUD_ARB 1000000
#define CAN_BAUD_DATA 5000000
#define CAN_SAMPLE_POINT 66.6

// --- MOTOR ID MAPPING --- (also q-mapping)
namespace MotorID {
    // Front Left
    const int FL_SHOULDER = 1;
    const int FL_HIP = 2;
    const int FL_KNEE = 3;

    // Front Right
    const int FR_SHOULDER = 4;
    const int FR_HIP = 5;
    const int FR_KNEE = 6;

    // Rear Left
    const int RL_SHOULDER = 7;
    const int RL_HIP = 8;
    const int RL_KNEE = 9;

    // Rear Right
    const int RR_SHOULDER = 10;
    const int RR_HIP = 11;
    const int RR_KNEE = 12;
}

// --- JOINT PROPERTIES ---
namespace JointProperties {
    // Gear Ratios
    const float RATIO_SHOULDER = 17.0f;
    const float RATIO_HIP = (56.0f / 18.0f) * (56.0f / 14.0f);
    const float RATIO_KNEE = (56.0f / 18.0f) * (56.0f / 14.0f);

    // Torque Limits [Nm]
    const float MAX_TORQUE_SHOULDER = 85.0f;
    const float MAX_TORQUE_HIP = 60.0f;
    const float MAX_TORQUE_KNEE = 60.0f;

    // Joint Limits [rad]
    const float SHOULDER_MIN = -0.65f;
    const float SHOULDER_MAX = 0.65f;
    const float HIP_MIN = -3.14/2;
    const float HIP_MAX = 3.14/2;
    const float KNEE_MIN = -3.14/2;
    const float KNEE_MAX = 3.14/2;
}

// --- GEOMETRY ---
namespace Geometry {
    // Joint points in global frame, seated pose [mm]
    const Vector3f ORIGIN(0.0f, 0.0f, 0.0f);
    const Vector3f FL_ORIGIN(3.500f, 68.000f, 0.000f);
    const Vector3f FL_SHOULDER(62.500f, 68.000f, 0.000f);
    const Vector3f FL_HIP(176.500f, 84.393f, 0.000f);
    const Vector3f FL_KNEE(28.500f, 139.828, 0.000f); 
    const Vector3f FL_FOOT(204.500f, 147.893f, 0.000f);

    // Lengths [mm]
    const float L_SHOULDER = abs(176.500f - 62.500f); // Shoulder to Hip
    const float L_HIP = abs(28.500f - 176.500f); // Hip to Knee
    const float L_KNEE = abs(204.500f - 28.500f); // Knee to Foot
}