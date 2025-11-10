#pragma once

#include "Config.h"
#include "Joint.h"
#include "Kinematics.h"

enum class LegMode : uint8_t {
    IDLE = 0,
    TORQUE = 1,
    POS = 2,

};

class Leg {
    public:
        Leg(Joint* joints, Kinematics* kin);

        void begin();

        void setMode(LegMode m);
        void setPosRef(const float q[DOF_PER_LEG]);
        void setTauRef(const float tau[DOF_PER_LEG]);

        void update(float dt);

    private:
        Joint* joints_;
        Kinematics* kin_;

        LegMode mode_;

        float q_ref_[DOF_PER_LEG];
        float tau_ref_[DOF_PER_LEG];
};
