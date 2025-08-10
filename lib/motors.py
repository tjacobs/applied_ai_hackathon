# Controls RobStride/Cybergear motors via CAN bus

# sudo ln -s /dev/tty.usbmodem206933BB55311 /tmp/ttyUSB_CAN
CAN_PORT = "/tmp/ttyUSB_CAN"

import can
import time
from struct import pack, unpack
from typing import List
from math import pi
import threading

class Motors:
    # Command constants
    CMD_GET_MCU_ID = 0
    CMD_POSITION = 1
    CMD_RESPONSE = 2
    CMD_ENABLE = 3
    CMD_RESET = 4
    CMD_SET_MECH_POSITION_TO_ZERO = 6
    CMD_CHANGE_CAN_ID = 7
    CMD_RAM_READ = 17
    CMD_RAM_WRITE = 18
    CMD_GET_MOTOR_FAIL = 21

    # Address constants
    ADDR_RUN_MODE = 0x7005
    ADDR_IQ_REF = 0x7006
    ADDR_SPEED_REF = 0x700A
    ADDR_LIMIT_TORQUE = 0x700B
    ADDR_CURRENT_KP = 0x7010
    ADDR_CURRENT_KI = 0x7011
    ADDR_CURRENT_FILTER_GAIN = 0x7014
    ADDR_LOC_REF = 0x7016
    ADDR_LIMIT_SPEED = 0x7017
    ADDR_LIMIT_CURRENT = 0x7018
    ADDR_MECH_POS = 0x7019
    ADDR_IQF = 0x701A
    ADDR_MECH_VEL = 0x701B
    ADDR_VBUS = 0x701C
    ADDR_ROTATION = 0x701D
    ADDR_LOC_KP = 0x701E
    ADDR_SPD_KP = 0x701F
    ADDR_SPD_KI = 0x7020
    ADDR_ZERO_STA = 0x7029

    # Mode constants
    MODE_MOTION = 0x00
    MODE_POSITION = 0x01
    MODE_SPEED = 0x02
    MODE_CURRENT = 0x03

    # Parameter limits
    P_MIN = -12.5
    P_MAX = 12.5
    V_MIN = -30.0
    V_MAX = 30.0
    KP_MIN = 0.0
    KP_MAX = 500.0
    KI_MIN = 0.0
    KI_MAX = 5.0
    KD_MIN = 0.0
    KD_MAX = 5.0
    T_MIN = -12.0
    T_MAX = 12.0
    IQ_MIN = -27.0
    IQ_MAX = 27.0
    CURRENT_FILTER_GAIN_MIN = 0.0
    CURRENT_FILTER_GAIN_MAX = 1.0

    IQ_REF_MAX = 23.0
    IQ_REF_MIN = -23.0
    SPD_REF_MAX = 30.0
    SPD_REF_MIN = -30.0
    LIMIT_TORQUE_MAX = 12.0
    LIMIT_TORQUE_MIN = 0.0
    CUR_KP_MAX = 200.0
    CUR_KP_MIN = 0.0
    CUR_KI_MAX = 200.0
    CUR_KI_MIN = 0.0
    LOC_KP_MAX = 200.0
    LOC_KP_MIN = 0.0
    SPD_KP_MAX = 200.0
    SPD_KP_MIN = 0.0
    LIMIT_SPD_MAX = 30.0
    LIMIT_SPD_MIN = 0.0
    LIMIT_CURRENT_MAX = 27.0
    LIMIT_CURRENT_MIN = 0.0

    # Default values
    DEFAULT_CURRENT_KP = 0.065
    DEFAULT_CURRENT_KI = 0.065
    DEFAULT_CURRENT_FILTER_GAIN = 0.0
    DEFAULT_POSITION_KP = 30.0
    DEFAULT_VELOCITY_KP = 2.0
    DEFAULT_VELOCITY_KI = 0.002
    DEFAULT_VELOCITY_LIMIT = 2.0
    DEFAULT_CURRENT_LIMIT = 1.0
    DEFAULT_TORQUE_LIMIT = 12.0

    # Response codes
    RET_OK = 0x00
    RET_MSG_NOT_AVAIL = 0x01
    RET_INVALID_CAN_ID = 0x02
    RET_INVALID_PACKET = 0x03

    # Timing
    RESPONSE_TIME_USEC = 250

    # Direction constants
    CW = 1
    CCW = -1

    def __init__(self, interface: str = 'slcan', channel: str = CAN_PORT, bitrate: int = 1000000, master_can_id: int = 0, debug: bool = False):
        """
        Initialize the motor controller.

        Parameters:
        -----------
        interface : str, optional
            The CAN interface to use. Examples:
            - 'socketcan' for Raspberry Pi (channel='can0')
            - 'slcan' for Cannable (channel='/dev/ttyUSB0')
            - 'slcan' for Serial CAN (channel='/dev/tty.usbserial-130')
        channel : str, optional
            The CAN channel to use. Examples:
            - 'can0' for Raspberry Pi
            - '/dev/ttyUSB0' for Cannable
            - '/dev/tty.usbserial-130' for Serial CAN
        bitrate : int, optional
            The CAN bus bitrate (default: 1000000).
        debug : bool, optional
            Enable debug mode to print CAN messages (default: False).
        """
        try:
            self.__canbus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate, extended=True)
        except Exception as e:
            print(f"Error initializing CAN bus: {e}")
            self.__canbus = None
        self.debug = debug
        self.master_can_id = master_can_id & 0xFF # Store the master ID
        self.can_lock = threading.Lock()  # Mutex for thread-safe CAN communication
        self.motors = {}  # Dictionary to track initialized motors: {motor_id: Motor}

    def init_motor(self, motor_id: int, master_can_id: int = 0):
        """
        Initialize a motor instance.

        Parameters:
        -----------
        motor_id : int
            The CAN ID of the motor.

        Returns:
        --------
        Motor
            An instance of the Motor class.
        """
        motor = Motor(self.__canbus, motor_id, master_can_id, self.debug, self.can_lock)
        self.motors[motor_id] = motor
        return motor

    def receive_motor_data(self) -> bool:
        with self.can_lock:  # Use the lock for thread-safe CAN access
            try:
                response = self.__canbus.recv(timeout=Motors.RESPONSE_TIME_USEC / 1000000)  # Read the next CAN message
                if response is None:
                    return False

                # Extract motor CAN ID
                motor_can_id = (response.arbitration_id >> 8) & 0xFF

                # Check if the motor ID is in the initialized motors dictionary
                if motor_can_id not in self.motors:
                    return False

                # Update motor status
                return self.motors[motor_can_id].update_motor_status(response.arbitration_id, response.data, len(response.data))

            except Exception as e:
                if self.debug:
                    print(f"Error processing CAN packet: {e}")
                return False

    def process_packet(self) -> bool:
        """
        Process incoming CAN packets and update motor status for all initialized motors.
        Returns True if any motor status was updated, False otherwise.
        """
        is_updated = False
        while True:  # Check if there are messages available
            if not self.receive_motor_data():
                break  # Exit early if no more data
            is_updated = True
        return is_updated

    def get_motor_status(self, motor_id):
        """
        Retrieve the latest motor status for the specified motor ID.
        """
        if motor_id not in self.motors: return None

        motor = self.motors[motor_id]
        return motor.motor_id, motor.get_motor_status()
    
    def ping_motor(self, motor_id: int, timeout: float = 0.01) -> bool:
        """Sends a GET_MCU_ID command (Type 0) and waits for a matching response."""
        target_id = motor_id & 0xFF
        cmd_type = Motors.CMD_GET_MCU_ID
        id_opt = self.master_can_id
        data = bytes(8)
        try:
            arbitration_id = (cmd_type & 0x1F) << 24 | (id_opt & 0xFFFF) << 8 | target_id
            ping_msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        except Exception as e:
            if self.debug: print(f"Error constructing ping message for ID {target_id}: {e}")
            return False
        with self.can_lock:
            try:
                if self.debug: print(f'TX PING to {target_id:02X}: id={ping_msg.arbitration_id:08x}')
                self.__canbus.send(ping_msg)
            except Exception as e:
                if self.debug: print(f"Error sending ping to ID {target_id}: {e}")
                return False
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.can_lock:
                response = self.__canbus.recv(timeout=0.01)
            if response is not None:
                if self.debug: print(f"RX during ping wait: id={response.arbitration_id:08x} data={response.data.hex(':')} ext={response.is_extended_id}")
                if response.is_extended_id:
                    response_cmd_type = (response.arbitration_id >> 24) & 0x1F
                    responding_motor_id_in_opt = (response.arbitration_id >> 8) & 0xFF
                    if response_cmd_type == Motors.CMD_GET_MCU_ID and responding_motor_id_in_opt == target_id:
                        if self.debug: print(f"Ping SUCCESS for ID {target_id}")
                        return True
        return False

    def scan_for_motors(self, scan_range: range = range(1, 128), timeout: float = 0.05) -> List[int]:
        """
        Scans a range of CAN IDs by pinging each one, shows dot progress.
        Prints detected IDs as hex strings, returns them as integers.

        Args:
            scan_range: Range of IDs to scan (e.g., range(1, 128)).
            timeout: Timeout (seconds) for each individual ping.

        Returns:
            A sorted list of detected motor CAN IDs as integers (e.g., [1, 124]).
        """
        detected_ids_int = []
        print(f"Scanning IDs: ", end='', flush=True)

        for motor_id_to_scan in scan_range:
            if self.ping_motor(motor_id_to_scan, timeout=timeout):
                detected_ids_int.append(motor_id_to_scan)
        print("Done.")

        sorted_detected_ids_int = sorted(detected_ids_int)

        detected_ids_hex = [f"0x{motor_id:02x}" for motor_id in sorted_detected_ids_int]

        if self.debug:
             print(f"Scan complete. Detected (hex): {detected_ids_hex}")
        elif not detected_ids_hex:
             print("No motors detected.")
        else:
            print(f"Detected: {detected_ids_hex}")

        return sorted_detected_ids_int

    def close(self):
        """Shutdown the CAN bus."""
        try:
            self.__canbus.shutdown()
        except Exception as e:
            #print(f"Error shutting down CAN bus: {e}")
            pass

class Motor:
    def __init__(self, canbus: can.Bus, motor_id: int, master_can_id: int = 0, debug: bool = False, can_lock: threading.Lock = None):
        """
        Initialize a motor instance.

        Parameters:
        -----------
        canbus : can.Bus
            The shared CAN bus interface.
        motor_id : int
            The CAN ID of the motor - default CAN id is 0x7F
        master_can_id : int
            The MASTER CAN ID - default host id is 0x00
        debug : bool, optional
            Enable debug mode to print CAN messages (default: False).
        """
        self.__canbus = canbus
        self.master_can_id = master_can_id & 0xFF
        self.motor_id = motor_id & 0xFF
        self.debug = debug
        self.position = 0.0
        self.raw_position = 0
        self.velocity = 0.0
        self.effort = 0.0
        self.temperature = 0.0
        self.rotation_count = 0
        self.mech_position = 0.0  # Mechanical position
        self.can_lock = can_lock  # Pass the lock to the Motor class
        self.reset_motor()

    def __send_command(self, cmd: int, id_opt: int = 0, data: bytes = bytes([0] * 8)) -> bool:
        """Send a CAN message without waiting for a response."""
        with self.can_lock:  # Use the shared lock for CAN operations
            try:
                msg_id = (cmd & 0x1F) << 24 | (id_opt & 0xFFFF) << 8 | (self.motor_id & 0xFF)
                msg = can.Message(arbitration_id=msg_id, data=data, is_extended_id=True)
                if self.debug:
                    print(f'TX: id={msg.arbitration_id:08x} data=', ':'.join(f'{x:02x}' for x in msg.data))
                self.__canbus.send(msg)
                return True
            except Exception as e:
                if self.debug:
                    print(f"Error sending CAN message for motor {self.motor_id:02X}: {e}")
                return False

    def update_motor_status(self, can_id, data, data_len) -> bool:
        """
        Update motor status based on the received CAN message.
        Returns True if the status was updated, False otherwise.
        """
        if data_len != 8:
            if self.debug:
                print(f"Invalid data length: {data_len} bytes")
            return False

        # Handle RAM_READ replies
        cmd = (can_id >> 24) & 0x1F
        if cmd == Motors.CMD_RAM_READ:
            index = int.from_bytes(data[0:2], 'little')
            raw = data[4:8]
            motor_can_id = (can_id >> 8) & 0xFF
            if index == Motors.ADDR_ROTATION:
                rotation_uint16 = int.from_bytes(raw[0:2], 'little', signed=True)
                self.rotation_count = rotation_uint16
                if False and motor_can_id == 0x10: print(f"Motor CAN ID: 0x{motor_can_id:02X}, rotation: {self.rotation_count}")
            elif index == Motors.ADDR_MECH_POS:
                # Parse mechanical position as float (4 bytes starting at offset 4)
                mech_pos_bytes = raw[0:4]
                self.mech_position = unpack('<f', mech_pos_bytes)[0]
                if False and self.motor_id == 0x10: print(f"Motor CAN ID: 0x{motor_can_id:02X}, mech_pos: {self.mech_position:.3f}")
            elif index == Motors.ADDR_ZERO_STA:
                self.zero_sta = raw[0]
                if True and self.motor_id == 0x10: print(f"Motor CAN ID: 0x{motor_can_id:02X}, zero_sta: {self.zero_sta}")
            return True

        try:
            # Extract raw values from the CAN message
            raw_position = data[0] << 8 | data[1]
            raw_velocity = data[2] << 8 | data[3]
            raw_effort = data[4] << 8 | data[5]
            raw_temperature = data[6] << 8 | data[7]
            self.raw_position = raw_position

            # Convert raw values to floats
            self.position = self.uint_to_float(raw_position, Motors.P_MIN, Motors.P_MAX)
            self.velocity = self.uint_to_float(raw_velocity, Motors.V_MIN, Motors.V_MAX)
            self.effort = self.uint_to_float(raw_effort, Motors.T_MIN, Motors.T_MAX)
            self.temperature = self.uint_to_float(raw_temperature, 0, 6553.5)

            if self.debug:
                print(f"Motor {self.motor_id:02X}: Position={self.position}, Velocity={self.velocity}, Effort={self.effort}, Temperature={self.temperature}")

            return True
        except Exception as e:
            if self.debug:
                print(f"Error updating motor status for motor {self.motor_id:02X}: {e}")
            return False

    def uint_to_float(self, x: int, min_val: float, max_val: float) -> float:
        """
        Convert a uint16 value to a float within a specified range.

        Parameters:
        -----------
        x : int
            The uint16 value to convert.
        min_val : float
            The minimum value of the range.
        max_val : float
            The maximum value of the range.

        Returns:
        --------
        float
            The converted float value.
        """
        return min_val + (max_val - min_val) * (x / 65535.0)

    def float_to_uint(self, x: float, min_val: float, max_val: float, bits: int) -> int:
        """
        Convert a float to an unsigned integer within a specified range.

        Parameters:
        -----------
        x : float
            The float value to convert.
        min_val : float
            The minimum value of the range.
        max_val : float
            The maximum value of the range.
        bits : int
            The number of bits for the output integer.

        Returns:
        --------
        int
            The converted unsigned integer.
        """
        span = max_val - min_val
        offset = min_val
        return int(((x - offset) / span) * ((1 << bits) - 1))

    def request_param(self, index: int):
        """Send a RAM_READ request for a given address and store under name."""
        data = index.to_bytes(2, 'little') + b'\x00' * 6
        return self.__send_command(Motors.CMD_RAM_READ, self.master_can_id, data=data)

    def write_param(self, index: int, data, width: str = 'B') -> bool:
        """
        Write a parameter to the motor.

        Parameters:
        -----------
        index : int
            The address/index of the parameter.
        data : int or float
            The value to write.
        width : str, optional
            The data type of the parameter. Options: 'B', 'h', 'H', 'l', 'L', 'f'.
        """
        if width == 'B':
            data_bytes = pack('<HxxBxxx', index, data)
        elif width == 'h':
            data_bytes = pack('<Hxxhxx', index, data)
        elif width == 'H':
            data_bytes = pack('<HxxHxx', index, data)
        elif width == 'l':
            data_bytes = pack('<Hxxl', index, data)
        elif width == 'L':
            data_bytes = pack('<HxxL', index, data)
        elif width == 'f':
            data_bytes = pack('<Hxxf', index, data)
        else:
            raise ValueError(f"Unsupported width: {width}")

        return self.__send_command(Motors.CMD_RAM_WRITE, self.master_can_id, data=data_bytes)

    def set_zero_sta(self, value: int = 1) -> bool:
        """zero_sta: 0 => 0–2π, 1 => -π–π."""
        return self.write_param(Motors.ADDR_ZERO_STA, value, width='B')

    def set_run_mode(self, mode: int) -> bool:
        """Set the run mode for the motor."""
        return self.write_param(Motors.ADDR_RUN_MODE, mode, width='B')

    def set_speed_limit(self, limit: float) -> bool:
        """Set the speed limit for the motor."""
        return self.write_param(Motors.ADDR_LIMIT_SPEED, limit, width='f')

    def set_torque_limit(self, limit: float) -> bool:
        """Set the torque limit for the motor."""
        return self.write_param(Motors.ADDR_LIMIT_TORQUE, limit, width='f')

    def set_current_limit(self, limit: float) -> bool:
        """Set the current limit for the motor."""
        return self.write_param(Motors.ADDR_LIMIT_CURRENT, limit, width='f')

    def set_position_control_gain(self, kp: float) -> bool:
        """Set the position control gain for the motor."""
        return self.write_param(Motors.ADDR_LOC_KP, kp, width='f')

    def set_velocity_control_gain(self, kp: float, ki: float) -> bool:
        """Set the velocity control gain for the motor."""
        return self.write_param(Motors.ADDR_SPD_KP, kp, width='f') and \
               self.write_param(Motors.ADDR_SPD_KI, ki, width='f')

    def set_current_control_param(self, kp: float, ki: float, gain: float) -> bool:
        """Set the current control parameters for the motor."""
        if kp > 0.055 or ki > 0.055:
            print(f"WARNING: Current control parameters are too high: kp={kp}, ki={ki}, gain={gain}")
            exit()
        return self.write_param(Motors.ADDR_CURRENT_KP, kp, width='f') and \
               self.write_param(Motors.ADDR_CURRENT_KI, ki, width='f') and \
               self.write_param(Motors.ADDR_CURRENT_FILTER_GAIN, gain, width='f')

    def enable_motor(self) -> bool:
        """Enable the motor."""
        return self.__send_command(Motors.CMD_ENABLE, self.master_can_id, data=bytes([0] * 8))

    def reset_motor(self) -> bool:
        """Reset the motor."""
        return self.__send_command(Motors.CMD_RESET, self.master_can_id, data=bytes([0] * 8))

    def stop_motor(self, fault: bool = False) -> bool:
        """Stop the motor."""
        data = bytes([1 if fault else 0] + [0] * 7)
        return self.__send_command(Motors.CMD_RESET, self.master_can_id, data=data)

    def send_position_command(self, position: float) -> bool:
        """Send a position command to the motor."""
        return self.write_param(Motors.ADDR_LOC_REF, position, width='f')

    def send_speed_command(self, speed: float) -> bool:
        """Send a speed command to the motor."""
        return self.write_param(Motors.ADDR_SPEED_REF, speed, width='f')

    def send_current_command(self, current: float) -> bool:
        """Send a current command to the motor."""
        return self.write_param(Motors.ADDR_IQ_REF, current, width='f')

    def send_motion_command(self, position: float, speed: float, torque: float, kp: float, kd: float) -> bool:
        """
        Send a motion command to the motor.

        Parameters:
        -----------
        position : float
            Target position (radians).
        speed : float
            Target speed (rad/sec).
        torque : float
            Target torque (Nm).
        kp : float
            Motion control kp.
        kd : float
            Motion control kd.

        Returns:
        --------
        bool
            True if the command was sent successfully, False otherwise.
        """
        # Convert floats to 16-bit unsigned integers
        pos_int = self.float_to_uint(position, Motors.P_MIN, Motors.P_MAX, 16)
        speed_int = self.float_to_uint(speed, Motors.V_MIN, Motors.V_MAX, 16)
        kp_int = self.float_to_uint(kp, Motors.KP_MIN, Motors.KP_MAX, 16)
        kd_int = self.float_to_uint(kd, Motors.KD_MIN, Motors.KD_MAX, 16)
        torque_int = self.float_to_uint(torque, Motors.T_MIN, Motors.T_MAX, 16)

        # Pack the data into bytes
        data = bytes([
            (pos_int >> 8) & 0xFF,  # Position high byte
            pos_int & 0xFF,         # Position low byte
            (speed_int >> 8) & 0xFF,  # Speed high byte
            speed_int & 0xFF,       # Speed low byte
            (kp_int >> 8) & 0xFF,   # Kp high byte
            kp_int & 0xFF,          # Kp low byte
            (kd_int >> 8) & 0xFF,   # Kd high byte
            kd_int & 0xFF           # Kd low byte
        ])

        # Send the motion command
        return self.__send_command(Motors.CMD_POSITION, self.master_can_id, data=data)

    def set_mech_position_to_zero(self) -> bool:
        """Set the mechanical position to zero for the motor."""
        # Ensure the data format matches the motor's expectations
        data = bytes([0x01] + [0x00] * 7)  # First byte is 0x01, rest are 0x00
        if self.debug:
            print(f"Setting mechanical position to zero. Data: {':'.join(f'{x:02x}' for x in data)}")
        
        # Send the command and return the result
        return self.__send_command(Motors.CMD_SET_MECH_POSITION_TO_ZERO, self.master_can_id, data=data)

    def get_output_angle_rad(self):

        return (self.raw_position - 32768) / 2620.0
        WITHIN = pi / 4  # 45 degrees
        HALF_WITHIN = WITHIN / 2
        frac = self.raw_position / 65535.0
        within_angle = (frac * WITHIN) - HALF_WITHIN
        motor_angle = self.rotation_count * WITHIN + within_angle
        return motor_angle

    def get_motor_status(self):
        """
        Retrieve the motor status for the current motor
        """
        abs_position = self.get_output_angle_rad()
        return self.position, abs_position, self.mech_position, self.rotation_count

    def change_motor_can_id(self, new_can_id: int):
        """Sends the command to the motor to change its CAN ID."""
        # Calculate the value for the id_opt parameter (bits 23-8 of Arbitration ID)
        id_option = ((new_can_id & 0xFF) << 8) | (self.master_can_id & 0xFF)
        # Prepare the required data payload (Byte 0 = 1)
        data = bytes([0x01] + [0x00] * 7)
        return self.__send_command(cmd=Motors.CMD_CHANGE_CAN_ID, id_opt=id_option, data=data)
