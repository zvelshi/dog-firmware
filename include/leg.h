#pragma once
#include <array>
#include "types.h"
#include "driver.h"

class Leg {
public:
    Leg(int shoulder_id, int hip_id, int knee_id, Driver* driver_ptr);

    void begin();
    void command(const LegCommand& cmd);

    LegState getState();

private:
    std::array<int, 3> _ids; // 0:Shoulder, 1:Hip, 2:Knee
    Driver* _driver;
    
    struct Limits { float min; float max; };
    static const Limits JOINT_LIMITS[3];
};