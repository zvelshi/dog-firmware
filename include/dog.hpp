#pragma once
#include "driver.hpp"
#include "types.hpp"
#include "config.hpp"

class Dog {
public:
    Dog();
    void begin();
    void setCommand(const DogCommand& cmd);
    DogState getState();

private:
    Driver driver;
    bool _active_motors[13];
    void checkJoint(int id, const char* name);
};