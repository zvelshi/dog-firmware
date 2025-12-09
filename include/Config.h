#pragma once

#include <Arduino.h>
#include "Enum.h"

constexpr bool TEST_MODE = true;

constexpr uint8_t NUM_LEGS = 1;
constexpr uint8_t DOF_PER_LEG = 1;

constexpr uint32_t BAUD_RATE = 500000;

constexpr float CTRL_HZ = 1000.0f;
constexpr float CTRL_DT = 1.0f / CTRL_HZ;    // [sec]
constexpr float CTRL_DT_US = CTRL_DT * 1e6f; // [us]

constexpr float PRINT_HZ = 15.0f;
constexpr float PRINT_DT = 1.0f / PRINT_HZ;    // [sec]
constexpr float PRINT_DT_US = PRINT_DT * 1e6f; // [us]

constexpr float L1 = 0.144f; // [m]
constexpr float L2 = 0.165f; // [m]

constexpr float GR1 = (18.0f / 56.0f) * (14.0f / 56.0f); // hip/knee speed reduction
constexpr float GR2 = (1.0f / 19.0f);                    // shoulder speed reduction

constexpr uint32_t AXIS_IDS[NUM_LEGS][DOF_PER_LEG] = {
    {2},
    //{1, 2}, // LEG 0
};

constexpr bool AXIS_CALIBRATION_STATUS[NUM_LEGS][DOF_PER_LEG] = {
    {true},
    //{true, true}, // LEG 0
};

constexpr float AXIS_ENCODER_OFFSETS[NUM_LEGS][DOF_PER_LEG] = {
    {0.0f},
    //{-0.44671177864074707f, -0.21636962890625f}, // LEG 0
};