#!/usr/bin/env python3
"""
Live ODrive monitor (fw/odrivetool 0.5.4)

Shows:
- Iq_measured (A)
- Iq_setpoint (A)
- encoder.pos_estimate
- encoder.vel_estimate

Plotted live with matplotlib.
"""

import time
from collections import deque

import odrive  # from odrivetool install
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Optional: handle specific fibre errors if available
try:
    import fibre
except ImportError:
    fibre = None

# --------- CONFIG ---------
AXIS_INDEX = 0          # 0 for axis0, 1 for axis1
SAMPLE_PERIOD = 0.02    # seconds between samples (50 Hz)
WINDOW_SECONDS = 10     # how many seconds of data to show in the plot
# --------------------------


def main():
    print("Searching for ODrive (fw 0.5.4)...")
    odrv0 = odrive.find_any()
    print(f"Connected to ODrive, serial: {hex(odrv0.serial_number)}")

    axis = odrv0.axis0 if AXIS_INDEX == 0 else odrv0.axis1

    # Data buffers
    maxlen = int(WINDOW_SECONDS / SAMPLE_PERIOD)
    t_data   = deque(maxlen=maxlen)
    iq_meas  = deque(maxlen=maxlen)
    iq_set   = deque(maxlen=maxlen)
    pos_est  = deque(maxlen=maxlen)
    vel_est  = deque(maxlen=maxlen)

    t0 = time.time()

    # Set up matplotlib figure
    fig, (ax_iq, ax_pos, ax_vel) = plt.subplots(3, 1, sharex=True)
    fig.suptitle("ODrive Live Monitor (axis{})".format(AXIS_INDEX))

    # Lines for each plot
    line_iq_meas, = ax_iq.plot([], [], label="Iq_measured (A)")
    line_iq_set,  = ax_iq.plot([], [], linestyle="--", label="Iq_setpoint (A)")
    ax_iq.set_ylabel("Current (A)")
    ax_iq.legend(loc="upper right")
    ax_iq.grid(True)

    line_pos, = ax_pos.plot([], [], label="pos_estimate")
    ax_pos.set_ylabel("Position (turns)")
    ax_pos.legend(loc="upper right")
    ax_pos.grid(True)

    line_vel, = ax_vel.plot([], [], label="vel_estimate")
    ax_vel.set_ylabel("Velocity (turn/s)")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.legend(loc="upper right")
    ax_vel.grid(True)

    plt.tight_layout()

    last_sample_time = [0.0]   # mutable so closure can modify
    disconnected = [False]     # flag so we only spam the error once

    # Build tuple of exceptions to catch, including fibre-specific ones if present
    odrive_exceptions = (Exception,)
    if fibre is not None:
        odrive_exceptions = (
            Exception,
            getattr(fibre, "ObjectLostError", Exception),
            getattr(getattr(fibre, "protocol", fibre), "ChannelBrokenException", Exception),
        )

    def update(frame):
        now = time.time()

        # If we've already lost connection, just keep returning current lines
        if disconnected[0]:
            return (line_iq_meas, line_iq_set, line_pos, line_vel)

        # Only sample at SAMPLE_PERIOD, even if animation calls faster
        if now - last_sample_time[0] < SAMPLE_PERIOD:
            return (line_iq_meas, line_iq_set, line_pos, line_vel)

        last_sample_time[0] = now
        t = now - t0

        try:
            # Read values from ODrive
            iq_m = axis.motor.current_control.Iq_measured
            iq_s = axis.motor.current_control.Iq_setpoint
            p_e  = axis.encoder.pos_estimate
            v_e  = axis.encoder.vel_estimate
        except odrive_exceptions as e:
            if not disconnected[0]:
                print("\n*** Lost connection to ODrive / axis (motor unplugged or board reset?)")
                print(f"    Error: {e}")
                print("    Freezing plot; close the window to exit.\n")
                disconnected[0] = True
            # Do not append new data; just keep existing lines as-is
            return (line_iq_meas, line_iq_set, line_pos, line_vel)

        # Normal path: append data
        t_data.append(t)
        iq_meas.append(iq_m)
        iq_set.append(iq_s)
        pos_est.append(p_e)
        vel_est.append(v_e)

        # Update line data
        line_iq_meas.set_data(t_data, iq_meas)
        line_iq_set.set_data(t_data, iq_set)
        line_pos.set_data(t_data, pos_est)
        line_vel.set_data(t_data, vel_est)

        # Adjust x-axis to show last WINDOW_SECONDS
        if len(t_data) > 1:
            t_min = max(0, t_data[-1] - WINDOW_SECONDS)
            t_max = t_data[-1]
            ax_iq.set_xlim(t_min, t_max)
            ax_pos.set_xlim(t_min, t_max)
            ax_vel.set_xlim(t_min, t_max)

            # Optional: auto-scale y each update
            for ax_, data in [(ax_iq, list(iq_meas) + list(iq_set)),
                              (ax_pos, pos_est),
                              (ax_vel, vel_est)]:
                if len(data) > 1:
                    y_min = min(data)
                    y_max = max(data)
                    if y_min == y_max:
                        # avoid zero-height axis
                        y_min -= 1.0
                        y_max += 1.0
                    ax_.set_ylim(y_min, y_max)

        return (line_iq_meas, line_iq_set, line_pos, line_vel)

    ani = FuncAnimation(fig, update, interval=int(SAMPLE_PERIOD * 1000), blit=False)

    print("Starting live plot. Close the window or Ctrl+C in terminal to stop.")
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Exiting...")


if __name__ == "__main__":
    main()
