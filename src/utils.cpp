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

    float getMaxTorque(int id) {
        int type = getJointType(id);
        if (type == 1) return JointProperties::MAX_TORQUE_SHOULDER;
        if (type == 2) return JointProperties::MAX_TORQUE_HIP;
        if (type == 0) return JointProperties::MAX_TORQUE_KNEE;
        return 0.0f;
    }

    std::array<float, 2> getJointLimits(int id) {
        int type = getJointType(id);
        std::array<float, 2> limits;
        if (type == 1) {
            limits[0] = JointProperties::SHOULDER_MIN;
            limits[1] = JointProperties::SHOULDER_MAX;
        } else if (type == 2) {
            limits[0] = JointProperties::HIP_MIN;
            limits[1] = JointProperties::HIP_MAX;
        } else if (type == 0) {
            limits[0] = JointProperties::KNEE_MIN;
            limits[1] = JointProperties::KNEE_MAX;
        } else {
            limits[0] = 0.0f;
            limits[1] = 0.0f;
        }
        return limits;
    }

    float getLinkLength(int id) {
        int type = getJointType(id);
        if (type == 1) return Geometry::L_SHOULDER;
        if (type == 2) return Geometry::L_HIP;
        if (type == 0) return Geometry::L_KNEE;
        return 0.0f;
    }
}