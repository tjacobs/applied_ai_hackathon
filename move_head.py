#!/usr/bin/python

# Import
import time
from lib.motors import Motors


# Motor port
CAN_PORT = "/dev/ttyACM0"

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

    # Init CAN
    motor_port = Motors(channel=CAN_PORT)

    # Create map of motor names to motors
    initialized_motor_count = 0
    for name, motor_id in zip(motor_names, motor_ids):
        if motor_port.ping_motor(motor_id):
            motors[name] = motor_port.init_motor(motor_id)
            initialized_motor_count += 1
            time.sleep(0.01)
            print(f"{name} motor with ID 0x{motor_id:02x} initialized")
        else:
            print(f"*** Warning: Could not find motor {name} with ID 0x{motor_id:02x}")

    # Check if all expected motors were initialized
    total_expected_motors = len(motor_names)
    print(f"\nMotors initialized: {initialized_motor_count} / {total_expected_motors}")
    if initialized_motor_count != total_expected_motors:
        print(f"WARNING: Not all expected motors were initialized! {initialized_motor_count} out of {total_expected_motors}.")


# Run main
if __name__ == "__main__":
    main()

