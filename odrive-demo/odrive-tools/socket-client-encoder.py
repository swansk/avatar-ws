import struct
from socket import *
import time

import odrive
from odrive.enums import *

odrv_encoder = odrive.find_any()
print("Found ODrive!")
odrv_encoder.axis0.requested_state = AXIS_STATE_IDLE

serverName = "192.168.1.156"
serverPort = 12000

clientSocket = socket(AF_INET, SOCK_STREAM)
while True:
    print("trying to connect")
    try:
        clientSocket.connect((serverName, serverPort))
    except ConnectionRefusedError:
        print("connection failed, trying again")
    else:
        print(
            "connected to {} on port {}".format(
                clientSocket.getpeername()[0], clientSocket.getsockname()[1]
            )
        )
        break
    time.sleep(5)
    serverPort += 1

packer = struct.Struct("d")

while True:
    pos_estimate = odrv_encoder.axis0.encoder.pos_estimate
    time.sleep(0.1)

    # msg = input("Enter a stupid thing you want to send: ").encode('utf-8')
    try:
        clientSocket.send(bytearray(packer.pack(pos_estimate)))
    except BrokenPipeError:
        print("Connection closed by server probably")
        break
    # resp = clientSocket.recv(1024)
    # if resp.decode() == 'goodbye':
    #     print('closing this shit')
    #     clientSocket.close()
    #     break
