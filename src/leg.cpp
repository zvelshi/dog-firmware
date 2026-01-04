#include "leg.h"
#include "utils.h"
#include "config.h"
#include "kinematics.h"

const Leg::Limits Leg::JOINT_LIMITS[3] = {
    {JointProperties::SHOULDER_MIN, JointProperties::SHOULDER_MAX},
    {JointProperties::HIP_MIN,      JointProperties::HIP_MAX},
    {JointProperties::KNEE_MIN,     JointProperties::KNEE_MAX}
};

Leg::Leg(int shoulder_id, int hip_id, int knee_id, Driver* driver_ptr) 
    : _driver(driver_ptr) {
    _ids[0] = shoulder_id;
    _ids[1] = hip_id;
    _ids[2] = knee_id;
}

void Leg::begin() {
    for (int i = 0; i < 3; i++) {
        bool online = _driver->probe(_ids[i]);
        if (online) {
            _driver->sendStopCommand(_ids[i]);
        }
    }
}

void Leg::command(const LegCommand& cmd) {
    for (int i = 0; i < 3; i++) {
        JointCommand j_cmd = cmd.joints[i];
        j_cmd.p_des = Utils::clamp(j_cmd.p_des, JOINT_LIMITS[i].min, JOINT_LIMITS[i].max);
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