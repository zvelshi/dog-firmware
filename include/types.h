#pragma once
#include <Arduino.h>
#include <ArduinoEigen.h>

struct JointState;
struct JointCommand; 

// --- MATH TYPES ---
using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;

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

// --- FOOT ---
struct FootState {
    Vector3f position; 
    bool in_contact = false; 
};

// --- LEG ---
struct LegState {
    std::array<JointState, 3> joints;
    FootState foot;
};

struct LegCommand {
    std::array<JointCommand, 3> joints;
};

// --- DOG ---
struct DogState {
    std::array<LegState, 4> legs;
};

struct DogCommand {
    std::array<LegCommand, 4> legs;
};