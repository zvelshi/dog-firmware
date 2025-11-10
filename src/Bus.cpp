#include "Bus.h"
#include "Body.h"

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
        Body::i().onCan(msg);
    }
}

bool Bus::send(uint32_t id, const uint8_t* data, uint8_t len) {
    CAN_message_t msg = {0};
    msg.id  = id;
    msg.len = len;
    memcpy(msg.buf, data, len);
    return can_.write(msg);
}
