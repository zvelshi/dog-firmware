#!/usr/bin/env python3

import time
import odrive
from odrive.enums import *

class DogMotor:
    def __init__(self):
        # Find the ODrive board
        odrv0 = odrive.find_any()
        self.axis = odrv0.axis0
        self.controller = self.axis.controller
        self.motor = self.axis.motor
        self.encoder = self.axis.encoder

        # Apply configuration settings
        # self.config_odrive(odrv0)

    def calibrate(self):
        # Request full calibration sequence
        self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("Calibration started...")

        # Wait until calibration is complete or timeout after 30 seconds
        start_time = time.time()
        timeout = 30  # seconds

        while self.axis.current_state != AXIS_STATE_IDLE:
            if time.time() - start_time > timeout:
                print("Calibration timed out!")
                break
            time.sleep(0.1)

        # Check calibration status
        motor_calibrated = self.motor.is_calibrated
        encoder_ready = self.encoder.is_ready
        print("Motor calibrated:", motor_calibrated)
        print("Encoder ready:", encoder_ready)

        if motor_calibrated and encoder_ready:
            print("Calibration successful and ready!")
        else:
            print("Calibration failed or incomplete.")
            print("Axis error code:", self.axis.error)

    def config_odrive(self, odrv0):
        print("Applying configuration settings to axis0...")
        # Motor configuration
        self.motor.config.pole_pairs = 7                     # Adjust according to your motor specs
        self.motor.config.motor_type = 1                     # 0 for high inductance, 1 for low inductance
        self.motor.config.resistance_calib_max_voltage = 4.0   # Voltage for calibration
        self.motor.config.requested_current_range = 25.0     # Maximum current range
        self.motor.config.calibration_current = 10.0         # Calibration current (A)
        self.motor.config.current_lim = 50.0                 # Continuous current limit

        # Encoder configuration
        self.encoder.config.cpr = 4096                       # Counts per revolution (adjust to your encoder)
        self.encoder.config.mode = 1                         # Mode for incremental encoders
        self.encoder.config.use_index = True                 # Use the index signal during calibration
        self.encoder.config.idx_search_unidirectional = True # Index search direction

        # Controller configuration
        self.controller.config.control_mode = 1              # 1 for velocity control, 3 for position control
        self.controller.config.vel_limit = 20000             # Maximum velocity (encoder counts per second)
        self.controller.config.pos_gain = 20.0               # Proportional gain for position control
        self.controller.config.vel_gain = 0.16               # Proportional gain for velocity control
        self.controller.config.vel_integrator_gain = 0.32    # Integrator gain for velocity control

        # Save configuration to flash (will reboot the board)
        print("Saving configuration to flash. The board will now reboot.")
        odrv0.save_configuration()
        # Wait for the board to reboot before proceeding (adjust sleep time as needed)
        time.sleep(5)
        print("Configuration applied and board rebooted.")

def main():
    dog_motor = DogMotor()
    dog_motor.calibrate()

if __name__ == '__main__':
    main()
