#pragma once

// --- JOINT ---
struct JointState {
    float position = 0.0f; // [rev]
    float velocity = 0.0f; // [rev/s]
    float torque = 0.0f; // [Nm]
    bool connected = false; // connection status [bool]
};

struct JointCommand {
    float p_des = 0.0f; // desired pos [rev]
    float v_des = 0.0f; // desired vel [rev/s]
    float kp = 1.0f; // stiffness -- set to 1.0 to use internal scaling
    float kd = 1.0f; // damping -- set to 1.0 to use internal scaling
    float feedforward = 0.0f; // feedforward torque [Nm]
    float max_torque = 0.0f;  // [Nm]
};

// --- LEG ---
struct LegState {
    JointState shoulder;
    JointState hip;
    JointState knee;
};

struct LegCommand {
    JointCommand shoulder;
    JointCommand hip;
    JointCommand knee;
};

// --- DOG ---
struct DogState {
    LegState fr;
    LegState fl;
    LegState rr;
    LegState rl;
};

struct DogCommand {
    LegCommand fr;
    LegCommand fl;
    LegCommand rr;
    LegCommand rl;
};