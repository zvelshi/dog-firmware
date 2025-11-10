#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(float l1, float l2)
: l1_(l1),
  l2_(l2) {}

void Kinematics::fk(const float q[DOF_PER_LEG], float p[3]) const {
    
    // planar 2R arm in XZ plane
    float x = l1_ * cosf(q[0]) + l2_ * cosf(q[0] + q[1]);
    float z = l1_ * sinf(q[0]) + l2_ * sinf(q[0] + q[1]);

    p[0] = x;
    p[1] = 0.0f; // no y component
    p[2] = z;
}
