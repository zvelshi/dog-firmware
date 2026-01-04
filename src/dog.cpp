#include "Arduino.h"
#include "dog.h"

Dog::Dog() : 
    driver(),
    leg_fl(MotorID::FL_SHOULDER, MotorID::FL_HIP, MotorID::FL_KNEE, &driver),
    leg_fr(MotorID::FR_SHOULDER, MotorID::FR_HIP, MotorID::FR_KNEE, &driver),
    leg_rl(MotorID::RL_SHOULDER, MotorID::RL_HIP, MotorID::RL_KNEE, &driver),
    leg_rr(MotorID::RR_SHOULDER, MotorID::RR_HIP, MotorID::RR_KNEE, &driver)
{
}

void Dog::begin() {
    driver.begin();
    delay(100);
    
    Serial.println("--- INITIALIZING LEGS ---");

    leg_fl.begin();
    leg_fr.begin();
    leg_rl.begin();
    leg_rr.begin();
}

void Dog::setCommand(DogCommand& cmd) {
    leg_fl.command(cmd.legs[0]);
    leg_fr.command(cmd.legs[1]);
    leg_rl.command(cmd.legs[2]);
    leg_rr.command(cmd.legs[3]);
}

DogState Dog::getState() {
    while(driver.poll()); 

    DogState state;
    state.legs[0] = leg_fl.getState();
    state.legs[1] = leg_fr.getState();
    state.legs[2] = leg_rl.getState();
    state.legs[3] = leg_rr.getState();

    return state;
}