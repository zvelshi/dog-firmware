#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(float l1, float l2)
: l1_(l1),
  l2_(l2) {}

// planar 2R arm
void Kinematics::fk(const float q[DOF_PER_LEG], float p[3]) const {
    float x = l1_ * cosf(q[0]) + l2_ * cosf(q[0] + q[1]);
    float z = l1_ * sinf(q[0]) + l2_ * sinf(q[0] + q[1]);

    p[0] = x;
    p[1] = 0.0f; // planar, no y component
    p[2] = z;
}

// planar 2R arm inverse kinematics, closed form analytical solution
bool Kinematics::ik(const float p[3], float q[DOF_PER_LEG], bool elbow_down = false) const {
    // target position
    const float x = p[0];
    const float z = p[2];

    // distance from hip to target
    float r2 = x * x + z * z;
    float r = sqrtf(r2);

    // check reachability
    float max_reach = l1_ + l2_; // fully extended
    float min_reach = fabsf(l1_ - l2_); // fully folded
    if (r > max_reach || r < min_reach) {
        return false; // unreachable
    }

    // solve for knee angle via cosine law
    float cos_q1 = (r2 - l1_ * l1_ - l2_ * l2_) / (2.0f * l1_ * l2_);
    
    // clamp to valid range
    if (cos_q1 >  1.0f) cos_q1 =  1.0f;
    if (cos_q1 < -1.0f) cos_q1 = -1.0f;

    // select elbow up or down solution
    float sin_q1 = sqrtf(fmaxf(0.0f, 1.0f - cos_q1 * cos_q1));
    if (!elbow_down) {
        sin_q1 = -sin_q1;
    }
    float q1 = atan2f(sin_q1, cos_q1); // knee angle

    // solve for hip angle
    float phi = atan2f(z, x);
    float k = atan2f(l2_ * sin_q1, l1_ + l2_*cos_q1);
    float q0 = phi - k;

    q[0] = q0; // hip
    q[1] = q1; // knee

    return true;
}