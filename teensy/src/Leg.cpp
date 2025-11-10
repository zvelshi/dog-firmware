#include "Leg.h"

Leg::Leg(Joint* joints, Kinematics* kin)
: joints_(joints),
  kin_(kin),
  mode_(LegMode::POS) {
    for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
        q_ref_[i] = 0.0f;
        tau_ref_[i] = 0.0f;
    }
}

void Leg::begin() {
    // nothing to do
}

void Leg::setMode(LegMode m) {
    mode_ = m;
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
    switch (mode_) {
        case LegMode::IDLE:
            // do nothing
            break;
        case LegMode::TORQUE:
            for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
                joints_[i].setTau(tau_ref_[i]);
            }
            break;

        case LegMode::POS:
            for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
                joints_[i].setPos(q_ref_[i]);
            }
            break;
    }
}
