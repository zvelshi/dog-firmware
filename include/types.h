#pragma once
#include <array>
#include <cstdint>

struct JointCommand; 
struct JointState;

// --- GEOMETRY ---
struct P3_XYZ {
    float x;
    float y;
    float z;
};

// --- JOINT ---
struct JointState {
    float position = 0.0f; // [rad]
    float velocity = 0.0f; // [rad/s]
    float torque = 0.0f; // [Nm]
    bool connected = false;
};

struct JointCommand {
    float p_des = 0.0f; // desired pos [rad]
    float v_des = 0.0f; // desired vel [rad/s]
    float kp = 1.0f; // stiffness -- set to 1.0 to use internal scaling
    float kd = 1.0f; // damping -- set to 1.0 to use internal scaling
    float feedforward = 0.0f; // feedforward torque [Nm]
    float max_torque = 60.0f;  // [Nm]
};

// --- FOOT STATE ---
struct FootState {
    P3_XYZ position; 
    bool in_contact = false; 
};

// --- LEG ---
struct LegState {
    std::array<JointState, 3> joints;
    FootState foot;

    JointState& shoulder() { return joints[0]; }
    JointState& hip()      { return joints[1]; }
    JointState& knee()     { return joints[2]; }
};

struct LegCommand {
    std::array<JointCommand, 3> joints;

    JointCommand& shoulder() { return joints[0]; }
    JointCommand& hip()      { return joints[1]; }
    JointCommand& knee()     { return joints[2]; }
};

// --- DOG ---
struct DogState {
    std::array<LegState, 4> legs;
};

struct DogCommand {
    std::array<LegCommand, 4> legs;
};