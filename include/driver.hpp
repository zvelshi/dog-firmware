#pragma once
#include <FlexCAN_T4.h>
#include "config.hpp"
#include "types.hpp"

class Driver {
public:
    Driver();
    void begin();
    void sendJointCommand(int id, const JointCommand& cmd);
    void sendStopCommand(int id);
    bool poll();
    bool probe(int id, uint32_t timeout_ms = 10);
    JointState getJointState(int id);

private:
    FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> can_bus;
    JointState _motor_cache[13];
    void parse_reply(const CANFD_message_t& msg);
    
    static const uint8_t REG_MODE = 0x000;
    static const uint8_t REG_POSITION = 0x001;
    static const uint8_t REG_CMD_POSITION = 0x020;
    static const uint8_t REG_MAX_TORQUE = 0x025;
};