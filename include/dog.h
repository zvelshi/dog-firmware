#pragma once
#include "driver.h"
#include "leg.h"
#include "types.h"
#include "config.h"

class Dog {
public:
    Dog();
    void begin();
    void setCommand(DogCommand& cmd);
    DogState getState();

private:
    Driver driver;
    
    // Leg Instances
    Leg leg_fl;
    Leg leg_fr;
    Leg leg_rl;
    Leg leg_rr;
};