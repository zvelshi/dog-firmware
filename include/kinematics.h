#pragma once
#include <array>
#include "types.h"

class Kinematics {
public:
    static P3_XYZ forwardKinematics(float q[3]);
    static std::array<float, 3> inverseKinematics(P3_XYZ p);
};