#pragma once
#include <deque>
#include <ArduinoEigen.h> 
#include "types.h"

// Refactored to use Eigen Vectors
// p = position, v = velocity, a = acceleration
struct CartesianState {
    Vector3f p = Vector3f::Zero();
    Vector3f v = Vector3f::Zero();
    Vector3f a = Vector3f::Zero();
};

class MinJerkGenerator {
public:
    MinJerkGenerator();

    void setLimits(float max_vel, float max_acc);
    void addWaypoint(const Vector3f& target);
    void clearWaypoints();
    void reset(const Vector3f& start_pos);
    
    // Returns the desired P, V, A for the current moment
    CartesianState update(float dt);

    bool isFinished();

private:
    std::deque<Vector3f> _waypoints;
    
    CartesianState _current_state;
    CartesianState _start_state;
    Vector3f _target_pos;

    float _time_elapsed = 0.0f;
    float _duration = 0.0f;
    bool _idle = true;
    
    float _v_max = 0.5f; 
    float _a_max = 5.0f; 

    // Vectorized Polynomial Solvers
    Vector3f computePoly(float t, const Vector3f& p0, const Vector3f& v0, const Vector3f& a0, const Vector3f& pf);
    Vector3f computePolyVel(float t, const Vector3f& p0, const Vector3f& v0, const Vector3f& a0, const Vector3f& pf);
    
    float calculateDuration(const Vector3f& start, const Vector3f& end);
};