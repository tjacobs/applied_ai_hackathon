#!/usr/bin/python

# Import
from lib.motors import Motors


# Motor port
CAN_PORT = "/dev/tty.usbmodem206F308B35411"

# Motors
motor_names = ['head_yaw', 'head_pitch']
motor_ids = [0x10, 0x11]

# Global state
motor_port = None
motors = {}

# Run
def main():
    global motor_port, motors

    init()
    

def init():
    print("Init motors")


# Run main
if __name__ == "__main__":
    main()

