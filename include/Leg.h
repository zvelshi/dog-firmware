#pragma once

#include "Config.h"
#include "Joint.h"
#include "Kinematics.h"

enum class LegMode : uint8_t {
    IDLE     = 0,
    TORQUE   = 1,
    POSITION = 2,

};

class Leg {
    public:
        Leg(Joint* joints, Kinematics* kin, uint8_t leg_index);

        // initialization
        void begin();
        void update();

        // control mode
        void setMode(LegMode m);

        // reference setters
        void setPosRef(const float q[DOF_PER_LEG]);
        void setTauRef(const float tau[DOF_PER_LEG]);
        bool setFootPosRef(const float p[3], bool elbow_down);

        // state getters
        void getJointState(JointState out[DOF_PER_LEG]) const;
        void footPos(float p[3]) const;

    private:
        Joint* joints_;
        Kinematics* kin_;
        uint8_t leg_index_;

        LegMode mode_;

        float q_ref_[DOF_PER_LEG];
        float tau_ref_[DOF_PER_LEG];
};
