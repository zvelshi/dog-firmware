#include "Bus.h"
#include "Joint.h"
#include "Config.h"
#include "ODriveCAN.h"

extern Joint joints[DOF_PER_LEG];

Bus& Bus::i() {
    static Bus b;
    return b;
}

void Bus::begin(uint32_t baud) {
    can_.begin();
    can_.setBaudRate(baud);
}

void Bus::poll() {
    CAN_message_t msg;
    while (can_.read(msg)) {
        uint8_t node_id = odcan::node(static_cast<uint16_t>(msg.id));

        for (uint8_t i = 0; i < DOF_PER_LEG; ++i) {
            if (joints[i].axis() == node_id) {
                joints[i].onCan(msg);
            }
        }
    }
}

bool Bus::send(uint32_t id, const uint8_t* data, uint8_t len) {
    CAN_message_t msg = {0};
    msg.id  = id;
    msg.len = len;
    memcpy(msg.buf, data, len);
    return can_.write(msg);
}
