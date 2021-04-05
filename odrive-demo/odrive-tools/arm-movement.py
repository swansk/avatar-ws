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

axis0 = odrv.axis0
axis1 = odrv.axis1

axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

while True:
    cmd = input()

    axis0.controller.input_position
    axis1.controller.i

    time.sleep(0.1)

    axis0.controller.input_torque = 0
    axis1.controller.input_torque = 0
