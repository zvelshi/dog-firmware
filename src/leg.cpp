#include "leg.h"
#include "utils.h"
#include "config.h"
#include "kinematics.h"

Leg::Leg(int shoulder_id, int hip_id, int knee_id, Driver* driver_ptr) 
    : _driver(driver_ptr) {
    _ids[0] = shoulder_id;
    _ids[1] = hip_id;
    _ids[2] = knee_id;
}

void Leg::begin() {
    for (int i = 0; i < 3; i++) {
        bool online = _driver->probe(_ids[i]);

        Serial.print("  Joint ");
        Serial.print(_ids[i]);
        Serial.print(": ");

        if (online) {
            _driver->sendStopCommand(_ids[i]);
            Serial.println("[ONLINE]");
        } else {
            Serial.println("[OFFLINE]");
        }
    }
}

void Leg::command(const LegCommand& cmd) {
    for (int i = 0; i < 3; i++) {
        JointCommand j_cmd = cmd.joints[i];
        std::array<float, 2> j_limits = Utils::getJointLimits(_ids[i]);
        j_cmd.p_des = Utils::clamp(j_cmd.p_des, j_limits[0], j_limits[1]);
        _driver->sendJointCommand(_ids[i], j_cmd);
    }
}

LegState Leg::getState() {
    LegState state;
    
    float q[3];
    for (int i = 0; i < 3; i++) {
        state.joints[i] = _driver->getJointState(_ids[i]);
        q[i] = state.joints[i].position;
    }

    state.foot.position = Kinematics::forwardKinematics(q);
    state.foot.in_contact = false; // TODO: Add contact estimation logic

    return state;
}