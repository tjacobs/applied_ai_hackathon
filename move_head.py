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

# Position limits
POSITION_LIMITS = {
    'head_yaw':       {'min': -0.5, 'max': 0.5, 'step':  0.05,  'offset': 0},
    'head_pitch':     {'min': -0.7, 'max': 0.4, 'step':  0.05,  'offset': 0},
}

# Global state
motor_port = None
motors = {}

# Run
def main():
    global motor_port, motors

    # Init
    init()

    # Zero the motors at their initial positions
    if False:
        for name, motor in motors.items():
            motor.set_mech_position_to_zero()
            print(f"{name} motor zeroed")

    # Run
    run()

    # Move
    while True:
        try:
            # Move
            send_positions({"head_yaw": 0, "head_pitch": 0})
            time.sleep(5)

            # Nod
            nod()

        except KeyboardInterrupt:
            print("\nCtrl+C pressed. Stopping motors...")
            stop_motors()
            exit()
        except Exception as e:
            print(f"\nError: {e}: {traceback.format_exc()}")
            stop_motors()
            exit()

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

        # Enable motors
        print("\nEnabling motors...\n")
        for name in motors.keys():
            motor = motors[name]
            motor.set_run_mode(motor_port.MODE_POSITION)
            motor.enable_motor()

        # Get time
        time_last = time.time()
        time_start = time_last

        # Move
        step = 0
        if True:
            # Process incoming CAN packets
            motor_port.process_packet()

            # Read motor status
            for name, motor in motors.items():
                position, abs_position, mech_position, rotation_count = motor.get_motor_status()
                print(position);
    # Done
    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping motors...")
        stop_motors()
    except Exception as e:
        print(f"\nError: {e}: {traceback.format_exc()}")
        stop_motors()
    finally:
        pass


# Stop all motors
def stop_motors():
    global motors
    for motor in motors.values(): motor.stop_motor()
    time.sleep(1)
    print("Motors stopped.")


# Send joint positions to motors
def send_positions(positions):
    global motor_port, motors, POSITION_LIMITS

    # Init motors if not already
    if not motor_port: init()

    # Process incoming CAN packets
    motor_port.process_packet()

    try:
        # Send to each motor
        for name, motor in motors.items():
            # Apply position limits
            positions[name] = max(min(positions[name], POSITION_LIMITS[name]['max'] + POSITION_LIMITS[name]['offset']), POSITION_LIMITS[name]['min'] + POSITION_LIMITS[name]['offset'])

            # Get command
            command_position = 0.0
            if name in positions: command_position = positions[name]

            # Get motor status
            position, abs_position, mech_position, rotation_count = motor.get_motor_status()

            # Print status
            if name == 'head_pitch': print(f"\t\t\t{name}:\tCommand: {command_position:+.3f},\tPosition: {position:+.3f},\tAbs_position: {abs_position:+.3f},\tMech_position: {mech_position:+.3f},\tRotation_count: {rotation_count}", end='\r\n')

            # Send position command
            if True: motor.send_position_command(command_position)

    except Exception as e:
        print(f"\nError: {e}: {traceback.format_exc()}")
        stop_motors()
        exit()

def nod():
    global motor_port, motors

    # Nod head
    print("Nod")
    send_positions({"head_yaw": 0, "head_pitch": 0})
    time.sleep(1)
    send_positions({"head_yaw": 0, "head_pitch": 0.4})
    time.sleep(1)
    send_positions({"head_yaw": 0, "head_pitch": 0})


# Run main
if __name__ == "__main__":
    main()

