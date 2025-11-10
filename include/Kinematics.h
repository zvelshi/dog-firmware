#pragma once

#include "Config.h"

class Kinematics {
    public:
        Kinematics(float l1, float l2);
        
        // q[0]=hip, q[1]=knee; p[0]=x, p[1]=y, p[2]=z
        void fk(const float q[DOF_PER_LEG], float p[3]) const;

        // returns true if the position is reachable
        bool ik(const float p[3], float q[DOF_PER_LEG], knee_down = true) const;

    private:
        float l1_;
        float l2_;
};
