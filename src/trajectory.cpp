#include "trajectory.h"
#include <algorithm>

MinJerkGenerator::MinJerkGenerator() {
    _idle = true;
}

void MinJerkGenerator::setLimits(float max_vel, float max_acc) {
    _v_max = max_vel;
    _a_max = max_acc;
}

void MinJerkGenerator::addWaypoint(const Vector3f& target) {
    _waypoints.push_back(target);
}

void MinJerkGenerator::clearWaypoints() {
    _waypoints.clear();
}

void MinJerkGenerator::reset(const Vector3f& start_pos) {
    _current_state.p = start_pos;
    _current_state.v.setZero();
    _current_state.a.setZero();
    
    _target_pos = start_pos;
    _idle = true;
    clearWaypoints();
}

bool MinJerkGenerator::isFinished() {
    return _idle && _waypoints.empty();
}

float MinJerkGenerator::calculateDuration(const Vector3f& start, const Vector3f& end) {
    float dist = (end - start).norm();
    
    // Avoid divide by zero
    if (dist < 1e-4) return 0.05f; // Minimum duration = 50ms

    float t_vel = dist / _v_max;
    float t_acc = sqrt(dist / _a_max);
    
    float T = std::max(t_vel, t_acc) * 1.2f; // Safety factor = 1.2
    return std::max(T, 0.05f); 
}

CartesianState MinJerkGenerator::update(float dt) {
    if (_idle) {
        if (!_waypoints.empty()) { 
            // Start next segment
            _start_state = _current_state;
            _target_pos = _waypoints.front();
            _waypoints.pop_front();
            
            _duration = calculateDuration(_start_state.p, _target_pos);
            _time_elapsed = 0.0f;
            _idle = false;
        } else { 
            // Hold position
            _current_state.v.setZero();
            _current_state.a.setZero();
            return _current_state;
        }
    }

    _time_elapsed += dt;

    if (_time_elapsed >= _duration) {
        // Snap to target
        _current_state.p = _target_pos;
        _current_state.v.setZero();
        _current_state.a.setZero();
        
        _idle = true; 

        // Immediately attempt next segment for smoothness
        return update(0.0f);
    }

    // --- VECTORIZED QUINTIC SPLINE ---
    // We assume final velocity (vf) and accel (af) are 0 for stop-and-go waypoints.
    
    _current_state.p = computePoly(_time_elapsed, _start_state.p, _start_state.v, _start_state.a, _target_pos);
    _current_state.v = computePolyVel(_time_elapsed, _start_state.p, _start_state.v, _start_state.a, _target_pos);

    return _current_state;
}

// Position p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
Vector3f MinJerkGenerator::computePoly(float t, const Vector3f& p0, const Vector3f& v0, const Vector3f& a0, const Vector3f& pf) {
    // Target boundary conditions: vf=0, af=0
    Vector3f vf = Vector3f::Zero(); 
    Vector3f af = Vector3f::Zero();

    float T = _duration;
    float T2 = T*T; 
    float T3 = T2*T; 
    float T4 = T3*T; 
    float T5 = T4*T;

    Vector3f h = pf - p0;
    
    Vector3f c0 = p0;
    Vector3f c1 = v0;
    Vector3f c2 = 0.5f * a0;
    Vector3f c3 = (10.0f*h - (6.0f*v0 + 4.0f*vf)*T - (3.0f*a0 - af)*T2) / (2.0f*T3);
    Vector3f c4 = (-15.0f*h + (8.0f*v0 + 7.0f*vf)*T + (3.0f*a0 - 2.0f*af)*T2) / (2.0f*T4);
    Vector3f c5 = (6.0f*h - (3.0f*v0 + 3.0f*vf)*T - 0.5f*(a0 - af)*T2) / (2.0f*T5);

    float t2 = t*t; 
    float t3 = t2*t; 
    float t4 = t3*t; 
    float t5 = t4*t;

    return c0 + c1*t + c2*t2 + c3*t3 + c4*t4 + c5*t5;
}

// Velocity p(t)' = c1 + 2c2*t + 3c3*t^2 + 4c4*t^3 + 5c5*t^4
Vector3f MinJerkGenerator::computePolyVel(float t, const Vector3f& p0, const Vector3f& v0, const Vector3f& a0, const Vector3f& pf) {
    Vector3f vf = Vector3f::Zero();
    Vector3f af = Vector3f::Zero();

    float T = _duration;
    float T2 = T*T; 
    float T3 = T2*T; 
    float T4 = T3*T; 
    float T5 = T4*T;

    Vector3f h = pf - p0;
    
    // Velocity coeffs derivative: p'(t) = c1 + 2c2*t ...
    Vector3f c1 = v0;
    Vector3f c2 = 0.5f * a0;
    Vector3f c3 = (10.0f*h - (6.0f*v0 + 4.0f*vf)*T - (3.0f*a0 - af)*T2) / (2.0f*T3);
    Vector3f c4 = (-15.0f*h + (8.0f*v0 + 7.0f*vf)*T + (3.0f*a0 - 2.0f*af)*T2) / (2.0f*T4);
    Vector3f c5 = (6.0f*h - (3.0f*v0 + 3.0f*vf)*T - 0.5f*(a0 - af)*T2) / (2.0f*T5);

    float t2 = t*t; 
    float t3 = t2*t; 
    float t4 = t3*t;

    return c1 + 2.0f*c2*t + 3.0f*c3*t2 + 4.0f*c4*t3 + 5.0f*c5*t4;
}