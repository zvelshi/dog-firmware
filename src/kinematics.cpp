#include <algorithm>
#include "kinematics.h"
#include "config.h"

Vector3f Kinematics::forwardKinematics(const Vector3f& q) {
    float x_plane = Geometry::L_HIP * sin(q[1]) + Geometry::L_KNEE * sin(q[1] + q[2]);
    float z_plane = -Geometry::L_HIP * cos(q[1]) - Geometry::L_KNEE * cos(q[1] + q[2]);

    Vector3f pos;
    pos.x() = x_plane;
    pos.y() = Geometry::L_SHOULDER * cos(q[0]) - z_plane * sin(q[0]);
    pos.z() = Geometry::L_SHOULDER * sin(q[0]) + z_plane * cos(q[0]);

    return pos;
}

Vector3f Kinematics::inverseKinematics(const Vector3f& target) {
    Vector3f q;

    float l_yz = sqrt(target.y() * target.y() + target.z() * target.z());
    float l_yz_proj = sqrt(std::max(0.0f, l_yz * l_yz - Geometry::L_SHOULDER * Geometry::L_SHOULDER));
    
    q(0) = atan2(target.y(), -target.z()) - atan2(Geometry::L_SHOULDER, l_yz_proj);

    float x_local = target.x();
    float z_local = -l_yz_proj;
    float r_sq = x_local * x_local + z_local * z_local;
    float r = sqrt(r_sq);

    float D = (r_sq - Geometry::L_HIP * Geometry::L_HIP - Geometry::L_KNEE * Geometry::L_KNEE) / (2.0f * Geometry::L_HIP * Geometry::L_KNEE);

    q(2) = -acos(Utils::clamp(D, -1.0f, 1.0f));

    float phi = atan2(x_local, -z_local);
    float val = (Geometry::L_HIP * Geometry::L_HIP + r_sq - Geometry::L_KNEE * Geometry::L_KNEE) / (2.0f * Geometry::L_HIP * r);
    float alpha = acos(Utils::clamp(val, -1.0f, 1.0f));

    q(1) = phi + alpha;

    return q;
}

Matrix3f Kinematics::computeJacobian(const Vector3f& q) {
    Matrix3f J;

    // Row 1: d(Pos.x) / dq
    J(0, 0) = 0.0f; 
    J(0, 1) = Geometry::L_HIP * cos(q[1]) + Geometry::L_KNEE * cos(q[1] + q[2]);
    J(0, 2) = Geometry::L_KNEE * cos(q[1] + q[2]);

    // Row 2: d(Pos.y) / dq
    // Note: The term (-L_HIP*cos(q[1]) - L_KNEE*cos(q[1]+q[2])) is the z_plane component
    J(1, 0) = -Geometry::L_SHOULDER * sin(q[0]) - (-Geometry::L_HIP * cos(q[1]) - Geometry::L_KNEE * cos(q[1] + q[2])) * cos(q[0]);
    J(1, 1) = -(Geometry::L_HIP * sin(q[1]) + Geometry::L_KNEE * sin(q[1] + q[2])) * sin(q[0]);
    J(1, 2) = -(Geometry::L_KNEE * sin(q[1] + q[2])) * sin(q[0]);

    // Row 3: d(Pos.z) / dq
    J(2, 0) = Geometry::L_SHOULDER * cos(q[0]) + (-Geometry::L_HIP * cos(q[1]) - Geometry::L_KNEE * cos(q[1] + q[2])) * (-sin(q[0]));
    J(2, 1) = (Geometry::L_HIP * sin(q[1]) + Geometry::L_KNEE * sin(q[1] + q[2])) * cos(q[0]);
    J(2, 2) = (Geometry::L_KNEE * sin(q[1] + q[2])) * cos(q[0]);

    return J;
}

Vector3f Kinematics::cartesianToJointVelocity(const Vector3f& q, const Vector3f& v) {
    Matrix3f J = computeJacobian(q);

    return J.inverse() * v;
}