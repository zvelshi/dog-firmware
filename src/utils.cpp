#include <algorithm>
#include "utils.h"
#include "config.h"

namespace Utils {
    float clamp(float val, float min, float max) {
        return std::clamp(val, min, max);
    }

    int getJointType(int id) {
        return id % 3;
    }

    float getGearRatio(int id) {
        int type = getJointType(id);
        if (type == 1) return JointProperties::RATIO_SHOULDER;
        if (type == 2) return JointProperties::RATIO_HIP;
        if (type == 0) return JointProperties::RATIO_KNEE;
        return 1.0f;
    }
}