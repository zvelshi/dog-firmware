#include "Leg.h"

Leg::Leg(Joint* joints, Kinematics* kin, uint8_t leg_index)
: joints_(joints),
  kin_(kin),
  leg_index_(leg_index),
  mode_(LegMode::IDLE) {
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        q_ref_[i] = 0.0f;
        tau_ref_[i] = 0.0f;
    }
}

void Leg::begin() {
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        joints_[i].begin();
        joints_[i].setAxisState(odrv::IDLE);
    }
}

void Leg::setMode(LegMode m) {
    mode_ = m;
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        switch (mode_) {
        case LegMode::IDLE:
            joints_[i].setAxisState(odrv::IDLE);
            break;
        case LegMode::TORQUE:
            joints_[i].setModes(odrv::TORQUE_CONTROL, odrv::PASSTHROUGH);
            joints_[i].setAxisState(odrv::CLOSED_LOOP_CONTROL);
            break;
        case LegMode::POSITION:
            joints_[i].setModes(odrv::POSITION_CONTROL, odrv::PASSTHROUGH);
            joints_[i].setAxisState(odrv::CLOSED_LOOP_CONTROL);
            break;
        }
    }
}

void Leg::setPosRef(const float q[DOF_PER_LEG]) {
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        q_ref_[i] = q[i];
    }
}

void Leg::setTauRef(const float tau[DOF_PER_LEG]) {
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        tau_ref_[i] = tau[i];
    }    
}

void Leg::update(float /*dt*/) {
    // refresh encoder estimates
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        joints_[i].update();
    }

    switch (mode_) {
    case LegMode::IDLE:
        // do nothing
        break;

    case LegMode::TORQUE:
        for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
            joints_[i].setTau(tau_ref_[i]);
        }
        break;

    case LegMode::POSITION:
        for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
            joints_[i].setPos(q_ref_[i]);
        }
        break;
    }
}

bool Leg::setFootPosRef(const float p[3], bool elbow_down) {
    float q[DOF_PER_LEG];
    if (!kin_->ik(p, q, elbow_down)) {
        return false;
    }
    setPosRef(q);
    return true;
}

void Leg::getJointState(JointState out[DOF_PER_LEG]) const {
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        out[i] = joints_[i].state();
    }
}

void Leg::footPos(float p[3]) const {
    float q[DOF_PER_LEG];
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        q[i] = joints_[i].state().q;
    }
    kin_->fk(q, p);
}