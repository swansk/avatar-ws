import struct
import time

import odrive
from odrive.enums import *


odrv_encoder = odrive.find_any(serial_number="2055399D4D4D")
odrv_motor = odrive.find_any(serial_number="207A399A4D4D")

odrv_motor.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv_encoder.axis0.requested_state = AXIS_STATE_IDLE

while True:
    pos_estimate = odrv_encoder.axis0.encoder.pos_estimate
    odrv_motor.axis0.controller.input_pos = pos_estimate
    time.sleep(0.01)