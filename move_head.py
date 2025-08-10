#!/usr/bin/python

# Import
import time
import traceback
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

    # Init
    init()

    # Run
    run()


def init():
    global motor_port, motors

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


def run():
    global motor_port, motors

    # Move
    try:
        # Set position mode so we can check positions before enabling motors
        for name, motor in motors.items(): motor.set_run_mode(motor_port.MODE_POSITION)

        # Get time
        time_last = time.time()
        time_start = time_last

        # Move
        step = 0
        while True:
            # Process incoming CAN packets
            motor_port.process_packet()

            # Read motor status
            for name, motor in motors.items():
                position, abs_position, mech_position, rotation_count = motor.get_motor_status()

            print(position);

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping motors...")
        stop_motors()
    except Exception as e:
        print(f"\nError: {e}: {traceback.format_exc()}")
        stop_motors()
    finally:
        stop_motors()


# Stop all motors
def stop_motors():
    global motors
    for motor in motors.values(): motor.stop_motor()
    time.sleep(1)
    print("Motors stopped.")


# Run main
if __name__ == "__main__":
    main()

