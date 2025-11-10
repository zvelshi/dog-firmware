#pragma once

#include <FlexCAN_T4.h>
#include "Config.h"
#include "Leg.h"

class Body {
public:
    static Body& i();

    void begin();
    void update(float dt);

    void setAllLegMode(LegMode m);
    void setLegMode(uint8_t leg_idx, LegMode m);

    void setLegPosRef(uint8_t leg_idx, const float q[DOF_PER_LEG]);
    void setLegTauRef(uint8_t leg_idx, const float tau[DOF_PER_LEG]);
    void setLegFootPosRef(uint8_t leg_idx, const float p[3], bool elbow_down);

    void getLegJointState(uint8_t leg_idx, JointState out[DOF_PER_LEG]) const;
    void getLegFootPos(uint8_t leg_idx, float p[3]) const;

    void onCan(const CAN_message_t& msg);

private:
    explicit Body(Leg* legs);

    Leg* legs_;
};
