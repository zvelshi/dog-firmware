#pragma once

#include <Arduino.h>
#include "Enum.h"

namespace odcan {

    // Build arbitration ID: can_id = node_id << 5 | cmd_id
    inline uint16_t id(uint8_t node_id, odrv::CANCommandID cmd) {
        return static_cast<uint16_t>( (static_cast<uint16_t>(node_id) << 5) | (static_cast<uint16_t>(cmd) & 0x1Fu) );
    }

    // Extract node_id (upper 6 bits)
    inline uint8_t node(uint16_t can_id) {
        return static_cast<uint8_t>(can_id >> 5);
    }

    // Extract cmd_id (lower 5 bits)
    inline odrv::CANCommandID cmd(uint16_t can_id) {
        return static_cast<odrv::CANCommandID>(can_id & 0x1Fu);
    }

}