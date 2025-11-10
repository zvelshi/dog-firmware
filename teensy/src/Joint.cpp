#include "Joint.h"
#include "ODriveCAN.h"
#include "Enum.h"

// convert float to int16_t with scaling factor of 0.001
static int16_t to_i16_1e3(float x) {
    float scaled = x * 1000.0f; // factor 0.001 in table => *1000 for encoding
    if (scaled >  32767.0f) scaled =  32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return static_cast<int16_t>(scaled);
}

Joint::Joint(uint32_t axis)
: axis_(axis),
  is_calibrated_(false) {
    state_.q   = 0.0f;
    state_.dq  = 0.0f;
    state_.tau = 0.0f;
}

void Joint::begin() {
    is_calibrated_ = false;
    encoder_offset_ = 0.0f;

    // Look up calibration status and encoder offset from config
    for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        for (uint8_t j = 0; j < DOF_PER_LEG; ++j) {
            if (AXIS_IDS[leg][j] == axis_) {
                is_calibrated_ = AXIS_CALIBRATION_STATUS[leg][j];
                encoder_offset_ = AXIS_ENCODER_OFFSETS[leg][j];
            }
        }
    }

    if (is_calibrated_) {
        // already calibrated: just set modes and go closed-loop
        setModes(CTRL_MODE, INPUT_MODE);
        setAxisState(odrv::IDLE);
    } else {
        // not calibrated: send calibration sequence
        calibrate();
    }
}

void Joint::update() {
    CAN_message_t req = {0};
    req.id = odcan::id(axis_, odrv::GET_ENCODER_EST);
    req.len = 0;
    req.flags.remote = 1;
    Bus::i().raw().write(req);
}

void Joint::calibrate(){
    // 1) Clear errors
    uint32_t zero = 0;
    Bus::i().send(
        odcan::id(axis_, odrv::CLEAR_ERRORS),
        reinterpret_cast<uint8_t*>(&zero),
        sizeof(zero)
    );

    // 2) Request FULL_CALIBRATION_SEQUENCE (state = 3)
    setAxisState(odrv::FULL_CALIBRATION_SEQUENCE);
}

void Joint::setTau(float tau) {
    // CMD 0x00E: Set Input Torque
    //  - bytes 0..3: float torque
    uint8_t data[8] = {0};
    memcpy(&data[0], &tau, sizeof(float));

    Bus::i().send(odcan::id(axis_, odrv::INPUT_TORQUE), data, 8);
}

void Joint::setPos(float q, float dq_ff, float tau_ff) {
    // CMD 0x00C: Set Input Pos
    //  - bytes 0..3: float pos
    //  - bytes 4..5: int16 vel_ff, factor 0.001
    //  - bytes 6..7: int16 torque_ff, factor 0.001
    uint8_t data[8] = {0};

    float pos = q - encoder_offset_;
    int16_t v_i16 = to_i16_1e3(dq_ff);
    int16_t t_i16 = to_i16_1e3(tau_ff);

    memcpy(&data[0], &pos, sizeof(float));
    memcpy(&data[4], &v_i16, sizeof(int16_t));
    memcpy(&data[6], &t_i16, sizeof(int16_t));

    Bus::i().send(
        odcan::id(axis_, odrv::INPUT_POS), 
        data, 
        8
    );
}

void Joint::onCan(const CAN_message_t& msg) {
    auto c = odcan::cmd(msg.id);
    // Serial.println("Joint " + String(axis_) + " received CAN cmd: " + String(static_cast<uint16_t>(c)));

    // 0x009: Get Encoder Estimates
    //  bytes 0..3: float pos_est
    //  bytes 4..7: float vel_est
    if (c == odrv::GET_ENCODER_EST) {
        float pos, vel;
        memcpy(&pos, &msg.buf[0], sizeof(float));
        memcpy(&vel, &msg.buf[4], sizeof(float));
        state_.q = pos - encoder_offset_;
        state_.dq = vel;
    }
}

void Joint::setModes(odrv::ControlMode ctrl, odrv::InputMode in) {
    int32_t c = static_cast<int32_t>(ctrl);
    int32_t m = static_cast<int32_t>(in);

    uint8_t data[8];
    memcpy(&data[0], &c, sizeof(int32_t));
    memcpy(&data[4], &m, sizeof(int32_t));

    Bus::i().send(
        odcan::id(axis_, odrv::SET_CTRL_MODES), 
        data, 
        8
    );
}

void Joint::setAxisState(odrv::AxisState state) {
    uint32_t s = static_cast<uint32_t>(state);
    Bus::i().send(
        odcan::id(axis_, odrv::SET_AXIS_STATE),
        reinterpret_cast<uint8_t*>(&s),
        sizeof(s)
    );
}