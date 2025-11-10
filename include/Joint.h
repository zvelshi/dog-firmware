#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Config.h"
#include "Bus.h"
#include "Enum.h"

struct JointState {
    float q;
    float dq;
    float tau;
};

class Joint {
    public:
        explicit Joint(uint32_t axis);

        void begin();
        void update();
        void calibrate();

        void setTau(float tau);
        void setPos(float q, float dq_ff = 0.0f, float tau_ff = 0.0f);

        void setModes(odrv::ControlMode ctrl, odrv::InputMode in);
        void setAxisState(odrv::AxisState state);

        void onCan(const CAN_message_t& msg);

        uint32_t axis() const { 
            return axis_; 
        }

        const JointState& state() const { 
            return state_; 
        }

    private:
        uint32_t   axis_;
        JointState state_;
        bool       is_calibrated_;
        float      encoder_offset_;
};