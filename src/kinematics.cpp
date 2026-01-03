#include "kinematics.hpp"
#include "types.hpp"
#include "config.hpp"
#include <cmath>
#include <algorithm>
#include <array>

P3_XYZ Kinematics::forwardKinematics(float q[3]) {
    float x_plane = Geometry::L_HIP * sin(q[1]) + Geometry::L_KNEE * sin(q[1] + q[2]);
    float z_plane = -Geometry::L_HIP * cos(q[1]) - Geometry::L_KNEE * cos(q[1] + q[2]);

    P3_XYZ pos;
    pos.x = x_plane;
    pos.y = Geometry::L_SHOULDER * cos(q[0]) - z_plane * sin(q[0]);
    pos.z = Geometry::L_SHOULDER * sin(q[0]) + z_plane * cos(q[0]);
    return pos;
}

std::array<float, 3> Kinematics::inverseKinematics(P3_XYZ target) {
    std::array<float, 3> q;

    float l_yz = sqrt(target.y * target.y + target.z * target.z);
    float l_yz_proj = sqrt(std::max(0.0f, l_yz * l_yz - Geometry::L_SHOULDER * Geometry::L_SHOULDER));
    q[0] = atan2(target.y, -target.z) - atan2(Geometry::L_SHOULDER, l_yz_proj);

    float x_local = target.x;
    float z_local = -l_yz_proj;

    float r_sq = x_local * x_local + z_local * z_local;
    float r = sqrt(r_sq);

    float D = (r_sq - Geometry::L_HIP * Geometry::L_HIP - Geometry::L_KNEE * Geometry::L_KNEE) / (2.0f * Geometry::L_HIP * Geometry::L_KNEE);

    q[2] = -acos(std::clamp(D, -1.0f, 1.0f));
    float phi = atan2(x_local, -z_local);
    
    float val = (Geometry::L_HIP * Geometry::L_HIP + r_sq - Geometry::L_KNEE * Geometry::L_KNEE) / (2.0f * Geometry::L_HIP * r);
    float alpha = acos(std::clamp(val, -1.0f, 1.0f));

    q[1] = phi + alpha;

    return q;
}