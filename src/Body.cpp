#include "Body.h"
#include "Joint.h"
#include "Kinematics.h"
#include "Config.h"
#include "ODriveCAN.h"

static Joint joints[NUM_LEGS][DOF_PER_LEG] = {
    { Joint(AXIS_IDS[0][0]), Joint(AXIS_IDS[0][1]) }, // leg 0
};

static Kinematics leg_kin[NUM_LEGS] = {
    Kinematics(L1, L2),
};

static Leg legs[NUM_LEGS] = {
    Leg(joints[0], &leg_kin[0], 0),
};

Body& Body::i() {
    static Body instance(legs);
    return instance;
}

Body::Body(Leg* legs)
: legs_(legs) {}


void Body::begin() {
    for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        legs_[leg].begin();
    }
}

void Body::update(float dt) {
    for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        legs_[leg].update(dt);
    }
}

void Body::setAllLegMode(LegMode m) {
    for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        legs_[leg].setMode(m);
    }
}

void Body::setLegMode(uint8_t leg_idx, LegMode m) {
    if (leg_idx < NUM_LEGS) {
        legs_[leg_idx].setMode(m);
    }
}

void Body::setLegPosRef(uint8_t leg_idx, const float q[DOF_PER_LEG]) {
    if (leg_idx < NUM_LEGS) {
        legs_[leg_idx].setPosRef(q);
    }
}

void Body::setLegTauRef(uint8_t leg_idx, const float tau[DOF_PER_LEG]) {
    if (leg_idx < NUM_LEGS) {
        legs_[leg_idx].setTauRef(tau);
    }
}

void Body::setLegFootPosRef(uint8_t leg_idx, const float p[3], bool knee_down) {
    if (leg_idx < NUM_LEGS) {
        legs_[leg_idx].setFootPosRef(p, knee_down);
    }
}

void Body::getLegJointState(uint8_t leg_idx, JointState out[DOF_PER_LEG]) const {
    if (leg_idx < NUM_LEGS) {
        legs_[leg_idx].getJointState(out);
    }
}

void Body::getLegFootPos(uint8_t leg_idx, float p[3]) const {
    if (leg_idx < NUM_LEGS) {
        legs_[leg_idx].footPos(p);
    } else {
        p[0] = p[1] = p[2] = 0.0f;
    }
}

void Body::onCan(const CAN_message_t& msg) {
    uint8_t node_id = odcan::node(static_cast<uint16_t>(msg.id));

    for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        for (uint8_t j = 0; j < DOF_PER_LEG; ++j) {
            if (joints[leg][j].axis() == node_id) {
                joints[leg][j].onCan(msg);
            }
        }
    }
}