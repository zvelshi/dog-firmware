#pragma once

#include <Arduino.h>

namespace odrv {
    // ODrive CAN command IDs
    enum CANCommandID: uint16_t {
        HEARTBEAT        = 0x001,
        MOTOR_ERROR      = 0x003,
        ENCODER_ERROR    = 0x004,
        SENSORLESS_ERROR = 0x005,
        SET_NODE_ID      = 0x006,
        SET_AXIS_STATE   = 0x007,
        GET_ENCODER_EST  = 0x009,
        GET_ENCODER_CNT  = 0x00A,
        SET_CTRL_MODES   = 0x00B,
        INPUT_POS        = 0x00C,
        INPUT_VEL        = 0x00D,
        INPUT_TORQUE     = 0x00E,
        SET_LIMITS       = 0x00F,
        START_ANTICOG    = 0x010,
        SET_TRAJ_VEL_LIM = 0x011,
        SET_TRAJ_ACC_LIM = 0x012,
        SET_TRAJ_INERTIA = 0x013,
        GET_IQ           = 0x014,
        GET_SENSORLESS   = 0x015,
        REBOOT           = 0x016,
        GET_VBUS         = 0x017,
        CLEAR_ERRORS     = 0x018,
        SET_LINEAR_COUNT = 0x019,
        SET_POS_GAIN     = 0x01A,
        SET_VEL_GAINS    = 0x01B,
    };

    // ODrive axis states
    enum AxisState: uint32_t {
        UNDEFINED                           = 0x0,
        IDLE                                = 0x1,
        STARTUP_SEQUENCE                    = 0x2,
        FULL_CALIBRATION_SEQUENCE           = 0x3,
        MOTOR_CALIBRATION                   = 0x4,
        ENCODER_CALIBRATION                 = 0x5,
        ENCODER_INDEX_SEARCH                = 0x6,
        ENCODER_OFFSET_CALIBRATION          = 0x7,
        CLOSED_LOOP_CONTROL                 = 0x8,
        LOCKIN_SPIN                         = 0x9,
        ENCODER_DIR_FIND                    = 0xA,
        HOMING                              = 0xB,
        ENCODER_HALL_POLARITY_CALIBRATION   = 0xC,
        ENCODER_HALL_PHASE_CALIBRATION      = 0xD,
    };

    // ODrive controller input modes
    enum InputMode: uint8_t {
        INACTIVE        = 0x0,
        PASSTHROUGH     = 0x1,
        VEL_RAMP        = 0x2,
        POS_FILTER      = 0x3,
        MIX_CHANNELS    = 0x4,
        TRAP_TRAJ       = 0x5,
        TORQUE_RAMP     = 0x6,
        MIRROR          = 0x7,
        TUNING          = 0x8,
    };

    // ODrive controller control modes
    enum ControlMode: uint8_t {
        VOLTAGE_CONTROL  = 0x0,
        TORQUE_CONTROL   = 0x1,
        VELOCITY_CONTROL = 0x2,
        POSITION_CONTROL = 0x3,
    };

} // namespace odrv