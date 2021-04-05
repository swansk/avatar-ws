import struct
import time

import odrive
from odrive.enums import *

CONTROL_MODE_VOLTAGE_CONTROL = 0
CONTROL_MODE_TORQUE_CONTROL = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3

ENVIRONMENTAL_I = 0.5

CURRENT_TORQUE_K = 8.269933431
KV_RATING = 85.0

odrv = odrive.find_any()

exo_axis = odrv.axis1
ava_axis = odrv.axis0

ava_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
exo_axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

while True:
    exo_axis_pos = exo_axis.encoder.pos_cpr
    ava_axis.controller.input_pos = exo_axis_pos

    current = ava_axis.motor.current_control.Iq_setpoint
    # Need to get only resisted current, how do we do this?
    if abs(current) > 1:
        applied_torque = (CURRENT_TORQUE_K * current) / KV_RATING
        print(current, applied_torque, exo_axis.motor.current_control.Iq_setpoint)
        exo_axis.controller.input_torque = applied_torque
        exo_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    else:
        exo_axis.controller.input_torque = 0
        exo_axis.requested_state = AXIS_STATE_IDLE
    # self.odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    # else:
    #     self.odrive.axis0.requested_state = AXIS_STATE_IDLE

    time.sleep(0.001)
