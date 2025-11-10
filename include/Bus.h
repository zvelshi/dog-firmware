#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

class Bus {
    public:
        static Bus& i();

        void begin(uint32_t baud);
        void poll();
        bool send(uint32_t id, const uint8_t* data, uint8_t len);

        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& raw() { 
            return can_; 
        }

    private:
        Bus() = default;
        Bus(const Bus&) = delete;
        Bus& operator = (const Bus&) = delete;

        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_;
};
