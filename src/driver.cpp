#include "driver.hpp"

// Union helper for byte packing
union FloatBytes { float f; uint8_t b[4]; };

// Helper to get joint type
int get_joint_type(int id) {
    int type = id % 3;
    return type;
}

// Helper to get the right ratio based on motor id
float get_ratio(int id) {
    int type = get_joint_type(id);
    
    if (type == 1) {
        return JointProperties::RATIO_SHOULDER;
    } else if (type == 2) {
        return JointProperties::RATIO_HIP;
    } else if (type == 0) {
        return JointProperties::RATIO_KNEE;
    } 
    return 1.0f; 
}

Driver::Driver() {}

void Driver::begin() {
    Serial.println("--- INITIALIZING DRIVER CAN BUS ---");
    can_bus.begin();
    CANFD_timings_t config;
    config.clock = CAN_CLOCK;
    config.baudrate = CAN_BAUD_ARB;
    config.baudrateFD = CAN_BAUD_DATA;
    config.sample = CAN_SAMPLE_POINT;
    config.propdelay = 190;
    config.bus_length = 1;
    can_bus.setBaudRate(config);
    can_bus.setRegions(64);
}

void Driver::sendJointCommand(int id, const JointCommand& cmd) {
    CANFD_message_t msg;
    
    msg.id = 0x8000 | id; 
    msg.flags.extended = 1; 
    msg.len = 64; 
    msg.brs = 1; 
    msg.edl = 1;
    
    memset(msg.buf, 0, 64);
    int ptr = 0;
    FloatBytes f_conv;

    // --- MATH CONVERSION ---
    float ratio = get_ratio(id);
    
    // Position: [rad] -> [rev]
    float motor_pos = (cmd.p_des / (2.0f * PI)) * ratio;
    
    // Velocity: [rad/s] -> [rev/s]
    float motor_vel = (cmd.v_des / (2.0f * PI)) * ratio;
    
    // Feedforward Torque: [Nm] -> [Nm]
    float motor_ff  = cmd.feedforward / ratio;

    // Max Torque: [Nm] -> [Nm]
    float motor_max_torque = cmd.max_torque / ratio;

    msg.buf[ptr++] = 0x01; // Write Int8 (Count 1)
    msg.buf[ptr++] = REG_MODE; // Reg 0x000 (Mode)
    msg.buf[ptr++] = 10; // Position Mode

    // Write all control parameters in one go
    msg.buf[ptr++] = 0x0C; // Write Float (Count 6)
    msg.buf[ptr++] = 6; // Count = 6 registers
    msg.buf[ptr++] = REG_CMD_POSITION; // Start at Reg 0x020 (Cmd Position)
    
    // Position
    f_conv.f = motor_pos; 
    memcpy(&msg.buf[ptr], f_conv.b, 4); ptr += 4;
    
    // Velocity
    f_conv.f = motor_vel; 
    memcpy(&msg.buf[ptr], f_conv.b, 4); ptr += 4;
    
    // Feedforward Torque
    f_conv.f = motor_ff; 
    memcpy(&msg.buf[ptr], f_conv.b, 4); ptr += 4;

    // Kp Scale (Pass raw stiffness command)
    f_conv.f = cmd.kp; 
    memcpy(&msg.buf[ptr], f_conv.b, 4); ptr += 4;

    // Kd Scale (Pass raw damping command)
    f_conv.f = cmd.kd; 
    memcpy(&msg.buf[ptr], f_conv.b, 4); ptr += 4;

    // Max Torque
    f_conv.f = motor_max_torque; 
    memcpy(&msg.buf[ptr], f_conv.b, 4); ptr += 4;

    // Request State Feedback
    msg.buf[ptr++] = 0x1F; // Read Float (Count 3)
    msg.buf[ptr++] = REG_POSITION; // Start at Reg 0x001 (Position)

    can_bus.write(msg);
}

void Driver::sendStopCommand(int id) {
    CANFD_message_t msg;
    msg.id = 0x8000 | id;
    msg.flags.extended = 1; 
    msg.len = 64; msg.brs = 1; msg.edl = 1;
    memset(msg.buf, 0, 64);
    
    int ptr = 0;
    
    msg.buf[ptr++] = 0x01; // Write Int8 (Count 1)
    msg.buf[ptr++] = 0x00; // Reg 0x000 (Mode)
    msg.buf[ptr++] = 0; // Value 0 (Stopped)

    can_bus.write(msg);
}

bool Driver::poll() {
    CANFD_message_t msg;
    if (can_bus.read(msg)) {
        parse_reply(msg);
        return true;
    }
    return false;
}

bool Driver::probe(int id, uint32_t timeout_ms) {
    // Send a limp command to get a response
    JointCommand limp; 
    limp.max_torque = 0.0f;
    sendJointCommand(id, limp);

    elapsedMillis timer = 0;
    while (timer < timeout_ms) {
        CANFD_message_t msg;
        if (can_bus.read(msg)) {
            int reply_source_id = (msg.id >> 8) & 0xFF;
            
            // Debug print to see what we actually hear
            // Serial.print("Heard ID: "); Serial.println(reply_source_id);

            if (reply_source_id == id) {
                parse_reply(msg); 
                return true;
            }
        }
    }
    return false;
}

void Driver::parse_reply(const CANFD_message_t& msg) {
    int id = (msg.id >> 8) & 0xFF;
    if (id > 12) return;

    int ptr = 0;
    while(ptr < msg.len - 4) {
        uint8_t val = msg.buf[ptr];
        if (val >= 0x20 && val <= 0x2F) { // Reply Float
             uint8_t start_reg = msg.buf[ptr+1];
             if (start_reg == REG_POSITION) {
                ptr += 2;
                FloatBytes f;
                
                float ratio = get_ratio(id);

                // Position: [rev] -> [rad] / ratio
                memcpy(f.b, &msg.buf[ptr], 4); 
                float motor_pos = f.f;
                _motor_cache[id].position = (motor_pos / ratio) * (2.0f * PI);
                ptr += 4;

                // Velocity: [rev/s] -> [rad/s] / ratio
                memcpy(f.b, &msg.buf[ptr], 4); 
                float motor_vel = f.f;
                _motor_cache[id].velocity = (motor_vel / ratio) * (2.0f * PI);
                ptr += 4;

                // Torque: [Nm] -> [Nm] * ratio
                memcpy(f.b, &msg.buf[ptr], 4); 
                float motor_trq = f.f;
                _motor_cache[id].torque = motor_trq * ratio;
                ptr += 4;

                _motor_cache[id].connected = true;
                return;
             }
        }
        ptr++;
    }
}

JointState Driver::getJointState(int id) {
    if (id > 12) {
        return JointState();
    } 
    return _motor_cache[id];
}