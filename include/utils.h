#pragma once
#include <cmath>
#include <cstdint>
#include "types.h"

namespace Utils {
    // Clamp helper
    float clamp(float val, float min, float max);

    // Motor ID helpers
    int getJointType(int id);
    float getGearRatio(int id);
    float getMaxTorque(int id);
    std::array<float, 2> getJointLimits(int id);
    float getLinkLength(int id);
}