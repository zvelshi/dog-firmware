#include "dog.hpp"
#include <Arduino.h>

Dog::Dog() {
    for(int i=0; i<13; i++) _active_motors[i] = false;
}

void Dog::begin() {
    driver.begin();
    delay(100);
    
    Serial.println("--- DETECTING MOTORS ---");

    checkJoint(MotorID::FR_SHOULDER, "FR Shoulder");
    checkJoint(MotorID::FL_SHOULDER, "FL Shoulder");
    checkJoint(MotorID::RR_SHOULDER, "RR Shoulder");
    checkJoint(MotorID::RL_SHOULDER, "RL Shoulder");

    checkJoint(MotorID::FR_HIP, "FR Hip");
    checkJoint(MotorID::FL_HIP, "FL Hip");
    checkJoint(MotorID::RR_HIP, "RR Hip");
    checkJoint(MotorID::RL_HIP, "RL Hip");

    checkJoint(MotorID::FR_KNEE, "FR Knee");
    checkJoint(MotorID::FL_KNEE, "FL Knee");
    checkJoint(MotorID::RR_KNEE, "RR Knee");
    checkJoint(MotorID::RL_KNEE, "RL Knee");
}

void Dog::setCommand(const DogCommand& cmd_in) {
    DogCommand safe_cmd = cmd_in;

    // Clamp positions to joint limits
    auto clampJoint = [&](JointCommand& j, float min, float max) {
        j.p_des = constrain(j.p_des, min, max);
    };

    clampJoint(safe_cmd.fr.shoulder, JointProperties::SHOULDER_MIN, JointProperties::SHOULDER_MAX);
    clampJoint(safe_cmd.fl.shoulder, JointProperties::SHOULDER_MIN, JointProperties::SHOULDER_MAX);
    clampJoint(safe_cmd.rr.shoulder, JointProperties::SHOULDER_MIN, JointProperties::SHOULDER_MAX);
    clampJoint(safe_cmd.rl.shoulder, JointProperties::SHOULDER_MIN, JointProperties::SHOULDER_MAX);

    clampJoint(safe_cmd.fr.hip, JointProperties::HIP_MIN, JointProperties::HIP_MAX);
    clampJoint(safe_cmd.fl.hip, JointProperties::HIP_MIN, JointProperties::HIP_MAX);
    clampJoint(safe_cmd.rr.hip, JointProperties::HIP_MIN, JointProperties::HIP_MAX);
    clampJoint(safe_cmd.rl.hip, JointProperties::HIP_MIN, JointProperties::HIP_MAX);

    clampJoint(safe_cmd.fr.knee, JointProperties::KNEE_MIN, JointProperties::KNEE_MAX);
    clampJoint(safe_cmd.fl.knee, JointProperties::KNEE_MIN, JointProperties::KNEE_MAX);
    clampJoint(safe_cmd.rr.knee, JointProperties::KNEE_MIN, JointProperties::KNEE_MAX);
    clampJoint(safe_cmd.rl.knee, JointProperties::KNEE_MIN, JointProperties::KNEE_MAX);

    // Send commands only to active motors
    auto sendIfAlive = [&](int id, const JointCommand& j_cmd) {
        if (_active_motors[id]) {
            driver.sendJointCommand(id, j_cmd);
        }
    };

    sendIfAlive(MotorID::FR_SHOULDER, safe_cmd.fr.shoulder);
    sendIfAlive(MotorID::FR_HIP, safe_cmd.fr.hip);
    sendIfAlive(MotorID::FR_KNEE, safe_cmd.fr.knee);

    sendIfAlive(MotorID::FL_SHOULDER, safe_cmd.fl.shoulder);
    sendIfAlive(MotorID::FL_HIP, safe_cmd.fl.hip);
    sendIfAlive(MotorID::FL_KNEE, safe_cmd.fl.knee);

    sendIfAlive(MotorID::RR_SHOULDER, safe_cmd.rr.shoulder);
    sendIfAlive(MotorID::RR_HIP, safe_cmd.rr.hip);
    sendIfAlive(MotorID::RR_KNEE, safe_cmd.rr.knee);

    sendIfAlive(MotorID::RL_SHOULDER, safe_cmd.rl.shoulder);
    sendIfAlive(MotorID::RL_HIP, safe_cmd.rl.hip);
    sendIfAlive(MotorID::RL_KNEE, safe_cmd.rl.knee);
}

DogState Dog::getState() {
    while(driver.poll()); // Clear buffer

    DogState state;
    
    // Fill the state struct
    auto fill = [&](int id) {
        JointState js = driver.getJointState(id);
        js.connected = _active_motors[id];
        return js;
    };

    state.fr.shoulder = fill(MotorID::FR_SHOULDER);
    state.fr.hip = fill(MotorID::FR_HIP);
    state.fr.knee = fill(MotorID::FR_KNEE);

    state.fl.shoulder = fill(MotorID::FL_SHOULDER);
    state.fl.hip = fill(MotorID::FL_HIP);
    state.fl.knee = fill(MotorID::FL_KNEE);

    state.rr.shoulder = fill(MotorID::RR_SHOULDER);
    state.rr.hip = fill(MotorID::RR_HIP);
    state.rr.knee = fill(MotorID::RR_KNEE);

    state.rl.shoulder = fill(MotorID::RL_SHOULDER);
    state.rl.hip = fill(MotorID::RL_HIP);
    state.rl.knee = fill(MotorID::RL_KNEE);

    return state;
}

void Dog::checkJoint(int id, const char* name) {
    bool alive = driver.probe(id);
    _active_motors[id] = alive;
    Serial.print("[ID "); 
    Serial.print(id); 
    Serial.print("] ");
    Serial.print(name);
    if (alive) {
        Serial.println(": ONLINE");
        Serial.print("    Sending STOP command to ID ");
        Serial.println(id);
        driver.sendStopCommand(id);
    } else {
        Serial.println(": MISSING");
    }
}