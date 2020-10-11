import struct
import time

import odrive
from odrive.enums import *


odrv_motor = odrive.find_any()

print("Found ODrive!")

odrv_motor.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

import socket

unpacker = struct.Struct("d")

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serv.bind(("0.0.0.0", 12000))
serv.listen(1)
while True:
    conn, addr = serv.accept()
    while True:
        data = conn.recv(unpacker.size)
        if not data:
            break
        pos_estimate = unpacker.unpack(data)
        odrv_motor.axis0.controller.input_pos = pos_estimate[0]
        # conn.send(b"I am SERVER<br>")
    conn.close()
    print("client disconnected")
