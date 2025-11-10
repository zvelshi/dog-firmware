#pragma once

#include <Arduino.h>
#include "Enum.h"

constexpr uint8_t NUM_LEGS = 1;
constexpr uint8_t DOF_PER_LEG = 2;

constexpr uint32_t BAUD_RATE = 500000;

constexpr float CTRL_HZ = 200.0f;
constexpr float CTRL_DT = 1.0f / CTRL_HZ;    // [sec]
constexpr float CTRL_DT_US = CTRL_DT * 1e6f; // [us]

constexpr float PRINT_HZ = 2.0f;
constexpr float PRINT_DT = 1.0f / PRINT_HZ;    // [sec]
constexpr float PRINT_DT_US = PRINT_DT * 1e6f; // [us]

constexpr float L1 = 0.144f; // [m]
constexpr float L2 = 0.165f; // [m]

constexpr uint32_t AXIS_IDS[NUM_LEGS][DOF_PER_LEG] = {
    {1, 2}, // LEG 0, joints 0 and 1
};

constexpr bool AXIS_CALIBRATION_STATUS[NUM_LEGS][DOF_PER_LEG] = {
    {true, true}, // LEG 0
};

constexpr float AXIS_ENCODER_OFFSETS[NUM_LEGS][DOF_PER_LEG] = {
    {-0.21966934204101f, -0.65679931640625f}, // LEG 0
};