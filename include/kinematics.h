#pragma once
#include "types.h"

class Kinematics {
public:
    static Vector3f forwardKinematics(const Vector3f& q);
    static Vector3f inverseKinematics(const Vector3f& p);
    static Matrix3f computeJacobian(const Vector3f& q);
    static Vector3f cartesianToJointVelocity(const Vector3f& q, const Vector3f& v);
};