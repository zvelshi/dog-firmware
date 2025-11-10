#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// Global Variables
uint8_t axis_id = 0;

// State flags
bool calibration_requested = false;
bool in_calibration = false;
bool calibrated = false;
bool closed_loop_requested = false;
bool in_closed_loop = false;
bool input_commanded = false;

// Tracking Variables
uint32_t prev_axis_error = 0;
uint8_t prev_state = 0;
uint8_t prev_controller_status = 0;
uint32_t prev_motor_error = 0;
uint32_t prev_encoder_error = 0;
uint32_t prev_controller_error = 0;

void request_errors() {
  CAN_message_t req;
  req.len = 0;
  req.flags.remote = 1; // RTR

  req.id = (axis_id << 5) | 0x003; // motor error
  Can0.write(req);

  req.id = (axis_id << 5) | 0x004; // encoder error
  Can0.write(req);

  req.id = (axis_id << 5) | 0x005; // sensorless error
  Can0.write(req);
}

bool send_can_msg(uint32_t id, const void* data, uint8_t len) {
  CAN_message_t msg = {0};
  msg.id = id;
  msg.len = len;
  memcpy(msg.buf, data, len);
  return Can0.write(msg);
}

void request_state(uint32_t new_state) {
  send_can_msg((axis_id << 5) | 0x007, &new_state, sizeof(new_state));
  Serial.print("Requested state: ");
  Serial.println(new_state);
}

void send_input_pos(float pos, int16_t vel_ff = 0, int16_t torque_ff = 0) {
  uint8_t data[8];
  memcpy(&data[0], &pos, sizeof(float));
  memcpy(&data[4], &vel_ff, sizeof(int16_t));
  memcpy(&data[6], &torque_ff, sizeof(int16_t));
  send_can_msg((axis_id << 5) | 0x00C, data, 8);
  Serial.print("Commanded position: ");
  Serial.println(pos, 2);
}

void send_input_vel(float vel, int16_t torque_ff = 0) {
  uint8_t data[8];
  memcpy(&data[0], &vel, sizeof(float));
  memcpy(&data[4], &torque_ff, sizeof(int16_t));
  send_can_msg((axis_id << 5) | 0x00D, data, 6);
  Serial.print("Commanded velocity: ");
  Serial.println(vel, 2);
}

void request_encoder_estimates(){
  CAN_message_t req = {0};
  req.id = (axis_id << 5) | 0x009;
  req.len = 0;
  req.flags.remote = 1; // RTR request
  Can0.write(req);
}

void set_controller_mode(uint32_t control_mode, uint32_t input_mode) {
  uint8_t data[8];
  memcpy(&data[0], &control_mode, sizeof(uint32_t));
  memcpy(&data[4], &input_mode, sizeof(uint32_t));
  send_can_msg((axis_id << 5) | 0x00B, data, 8);
}

void clear_errors(){
  uint32_t zero = 0;
  send_can_msg((axis_id << 5) | 0x018, &zero, sizeof(zero));
  Serial.println("Cleared errors");
}

void print_fixed(float val, int width, int decimals) {
  char buf[16];
  dtostrf(val, width, decimals, buf); // format into buffer
  Serial.print(buf);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Can0.begin();
  Can0.setBaudRate(500000);

  Serial.println("Teensy CAN Init Done - Listening for Heartbeat...");
  clear_errors();
}

void loop() {
  CAN_message_t msg;
  if (Can0.read(msg)) {
    uint32_t heartbeat_id = (axis_id << 5) | 0x001;

    if (msg.id == heartbeat_id) {
      uint32_t axis_error;
      memcpy(&axis_error, &msg.buf[0], sizeof(axis_error));
      uint8_t current_state;
      memcpy(&current_state, &msg.buf[4], sizeof(current_state));
      uint8_t current_controller_status;
      memcpy(&current_controller_status, &msg.buf[7], sizeof(current_controller_status));

      // print only on change
      if (axis_error != prev_axis_error || current_state != prev_state || current_controller_status != prev_controller_status) {
        prev_axis_error = axis_error;
        prev_state = current_state;
        prev_controller_status = current_controller_status;
        Serial.print("Heartbeat         | Err: ");
        print_fixed(axis_error, 6, 0);
        Serial.print(" | State: ");
        print_fixed(current_state, 4, 0);
        Serial.print(" | Ctrl: ");
        print_fixed(current_controller_status, 5, 0);
        Serial.println();
      }

      if (axis_error != 0) {
        request_errors();
      }

      request_encoder_estimates();

      // request calibration
      if (!calibration_requested && current_state == 1) {
        request_state(3); // FULL_CALIBRATION_SEQUENCE
        calibration_requested = true;
        Serial.println("Calibration requested...");
      }

      // detect calibration finished successfully 
      if (current_state == 3 || current_state == 4 || current_state == 6 || current_state == 7) { 
        in_calibration = true; 
      }

      // detect calibration finished (back to IDLE with no errors)
      if (in_calibration && !calibrated && (current_state == 1)) {
        in_calibration = false;
        calibrated = true;
        Serial.println("Calibration completed successfully!");
      }

      // request CLOSED_LOOP_CONTROL after calibration
      if (calibrated && !closed_loop_requested && current_state == 1) {
        // set_controller_mode(2, 1); // VELOCITY_CONTROL + IN_PASSTHROUGH
        set_controller_mode(3, 1); // POSITION_CONTROL + IN_PASSTHROUGH
        request_state(8); // CLOSED_LOOP_CONTROL
        closed_loop_requested = true;
      }

      // confirm CLOSED_LOOP_CONTROL active
      if (closed_loop_requested && !in_closed_loop && current_state == 8) {
        in_closed_loop = true;
      }

      // command position once in closed loop
      if (in_closed_loop && !input_commanded) {
        // send_input_vel(45.0f, 0);
        send_input_pos(-2.0f);
        input_commanded = true;
      }
    }

    // encoder estimates response
    if (msg.id == ((axis_id << 5) | 0x009)) {
      float pos, vel;
      memcpy(&pos, &msg.buf[0], sizeof(float));
      memcpy(&vel, &msg.buf[4], sizeof(float));

      Serial.print("Encoder Estimates | Pos: ");
      print_fixed(pos, 6, 2);
      Serial.print(" | Vel: ");
      print_fixed(vel, 6, 2);
      Serial.println();
    }

    // error responses
    if (msg.id == ((axis_id << 5) | 0x003)) {
      uint32_t motor_err;
      memcpy(&motor_err, &msg.buf[0], sizeof(motor_err));
      if (motor_err != prev_motor_error) {
        prev_motor_error = motor_err;
        Serial.print("Errors            | Mot: ");
        print_fixed(motor_err, 6, 0);
        Serial.print(" | Enc: ");
        print_fixed(prev_encoder_error, 6, 0);
        Serial.print(" | Ctrl: ");
        print_fixed(prev_controller_error, 5, 0);
        Serial.println();
      }
    }

    if (msg.id == ((axis_id << 5) | 0x004)) {
      uint32_t enc_err;
      memcpy(&enc_err, &msg.buf[0], sizeof(enc_err));
      if (enc_err != prev_encoder_error) {
        prev_encoder_error = enc_err;
        Serial.print("Errors            | Mot: ");
        print_fixed(prev_motor_error, 6, 0);
        Serial.print(" | Enc: ");
        print_fixed(enc_err, 6, 0);
        Serial.print(" | Ctrl: ");
        print_fixed(prev_controller_error, 5, 0);
        Serial.println();
      }
    }

    if (msg.id == ((axis_id << 5) | 0x005)) {
      uint32_t ctrl_err;
      memcpy(&ctrl_err, &msg.buf[0], sizeof(ctrl_err));
      if (ctrl_err != prev_controller_error) {
        prev_controller_error = ctrl_err;
        Serial.print("Errors            | Mot: ");
        print_fixed(prev_motor_error, 6, 0);
        Serial.print(" | Enc: ");
        print_fixed(prev_encoder_error, 6, 0);
        Serial.print(" | Ctrl: ");
        print_fixed(ctrl_err, 5, 0);
        Serial.println();
      }
    }
  }
}