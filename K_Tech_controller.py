# -----------------------------------------------------------
# Project : LK Motors Control
# Author  : Axel Praplan
# Purpose : Script for controlling and processing data related
#           to LK motors (e.g., extracting motor index, etc.)
# Date    : 2025-07-02
# -----------------------------------------------------------

import serial
import struct

START_BYTE = 0x3E
START_BYTE_BROADCAST = 0x02

HEAD = 0
CMD = 1
ID = 2
DATA_LENGTH = 3
CMD_SUM = 4

MAX_NUMBER_DRIVER = 65

OFF = -1
LOW = 0
MEDIUM = 1
HIGH = 2

# Make a set debug level function
# To do: repeat all reading while data are None
# To do: handle pid reading and setting
# To do check if can state one contains current

class K_Tech_RS485:
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

        self.debug_level = MEDIUM

        self.gear_ratios = [None] * MAX_NUMBER_DRIVER # Store gear ratios on init
        self.temperatures = [None] * MAX_NUMBER_DRIVER
        self.voltages = [None] * MAX_NUMBER_DRIVER
        # self.currents = [0.0] * MAX_NUMBER_DRIVER
        self.motor_states = [None] * MAX_NUMBER_DRIVER
        self.error_states = [None] * MAX_NUMBER_DRIVER
        self.iqs = [None] * MAX_NUMBER_DRIVER
        self.speeds = [None] * MAX_NUMBER_DRIVER
        self.multi_turn_angles = [None] * MAX_NUMBER_DRIVER
        self.single_turn_angles = [None] * MAX_NUMBER_DRIVER
        self.encoder_values = [None] * MAX_NUMBER_DRIVER
        self.encoder_raws = [None] * MAX_NUMBER_DRIVER
        self.encoder_offsets = [None] * MAX_NUMBER_DRIVER
        self.currents_A = [None] * MAX_NUMBER_DRIVER
        self.currents_B = [None] * MAX_NUMBER_DRIVER
        self.currents_C = [None] * MAX_NUMBER_DRIVER

    def _debug_print(self, *args, level=0):
        if self.debug_level >= level: print(*args)

    def _checksum(self, data):
        """
        Calculate the checksum of a list of bytes.

        Args:
            data (list): List of integer byte values

        Returns:
            int: Checksum value (sum of bytes modulo 256)
        """
        return sum(data) & 0xFF
    
    def _build_frame(self, cmd, motor_id, data=b''):
        """
        Build a communication frame for sending to a motor.

        Args:
            cmd (int): Command byte
            motor_id (int): ID of the target motor
            data (bytes, optional): Payload data (default is empty bytes)

        Returns:
            bytearray: Constructed frame including checksums
        """
        frame = bytearray([START_BYTE, cmd, motor_id, len(data)])
        frame.append(self._checksum(frame))
        frame += data
        if data:
            frame.append(self._checksum(data))
        return frame

    def _send_command(self, cmd, motor_id, data=b''):
        """
        Send a command frame to the motor and handle the response.

        Args:
            cmd (int): Command byte
            motor_id (int): ID of the target motor
            data (bytes, optional): Payload data to send (default is empty bytes)

        Returns:
            bytes | bool | None: 
                - `bytes`: Received data if response is valid and contains data  
                - `True`: If response is valid but contains no data  
                - `False`: If response header is invalid  
                - `None`: If data checksum is incorrect
        """
        # Send command
        frame = self._build_frame(cmd, motor_id, data)

        self._debug_print("TX:", ' '.join(f'{byte:02X}' for byte in frame), level=HIGH)
        
        self.serial.write(frame)

        # Handle response
        start_ms = time.perf_counter() * 1000
        # time.sleep(5/1000)
        response = self.serial.read(5)
        end_ms = time.perf_counter() * 1000
        print(f"Elapsed time 1: {end_ms - start_ms:.3f} ms")
        self._debug_print("RX:", ' '.join(f'{byte:02X}' for byte in response), level=HIGH)

        if response[HEAD] == START_BYTE and response[CMD_SUM] == self._checksum(response[0:4]):

            if response[DATA_LENGTH] != 0:
                data = self.serial.read(response[DATA_LENGTH]+1)
                self._debug_print("RX:", ' '.join(f'{byte:02X}' for byte in data), level=HIGH)
                if data[response[DATA_LENGTH]] == self._checksum(data[0:response[DATA_LENGTH]]):
                    # Correct data
                    return data
                else:
                    self._debug_print("[Communication Handler] Data Comm. Error", level=LOW)
                    self.serial.reset_input_buffer()  # Clear input buffer
                    # Wrong data
                    return None
            else:
                # Correct echo no data
                return True     

        else: 
            # Wrong echo
            self._debug_print("[Communication Handler] Comm. Error", level=LOW)
            self.serial.reset_input_buffer()  # Clear input buffer
            return False
        
    # def _send_broadcast_command(self, cmd, data):
    #     """
    #     Send a broadcast command frame to all motors and handle the responses.

    #     Args:
    #         cmd (int): Broadcast command byte (e.g., 0x80 for torque, 0x81 for speed, etc.)
    #         data (bytes): 8-byte payload for 4 motors (2 bytes each)

    #     Returns:
    #         list[bytes] | bool | None:
    #             - list[bytes]: List of responses from motors if all replies are valid
    #             - True: If responses are expected but contain no payloads
    #             - False: If response header is invalid
    #             - None: If a response has a bad checksum
    #     """
    #     if len(data) != 8:
    #         raise ValueError("Broadcast command requires exactly 8 bytes of data.")

    #     frame = bytearray()
    #     frame.append(START_BYTE_BROADCAST)      # Head
    #     frame.append(cmd)                       # Command byte
    #     frame.extend(data)                      # 8 bytes of motor data
    #     checksum = sum(frame) & 0xFF
    #     frame.append(checksum)

    #     self._debug_print("TX (Broadcast):", ' '.join(f'{byte:02X}' for byte in frame), level=HIGH)
    #     self.serial.write(frame)

    #     responses = []
    #     for i in range(4):  # Expect responses from up to 4 motors
    #         response_header = self.serial.read(5)
    #         if len(response_header) < 5:
    #             self._debug_print(f"[Broadcast] Incomplete header from motor ID {i+1}", level=LOW)
    #             return False
    #         self._debug_print(f"RX (Header M{i+1}):", ' '.join(f'{b:02X}' for b in response_header), level=HIGH)

    #         if response_header[HEAD] != START_BYTE or response_header[CMD_SUM] != self._checksum(response_header[0:4]):
    #             self._debug_print(f"[Broadcast] Invalid header checksum from motor ID {i+1}", level=LOW)
    #             return False

    #         data_len = response_header[DATA_LENGTH]
    #         if data_len > 0:
    #             payload = self.serial.read(data_len + 1)
    #             self._debug_print(f"RX (Payload M{i+1}):", ' '.join(f'{b:02X}' for b in payload), level=HIGH)
    #             if payload[-1] != self._checksum(payload[:-1]):
    #                 self._debug_print(f"[Broadcast] Invalid data checksum from motor ID {i+1}", level=LOW)
    #                 return None
    #             responses.append(payload[:-1])
    #         else:
    #             responses.append(b'')

    #     return responses if any(responses) else True

    def _parse_state_1(self, motor_id, data):
        """
        Parse the response data from the motor.

        This function updates:
        - Motor temperature (int8_t, 1 °C/LSB)
        - Motor voltage (int16_t, 0.01 V/LSB)
        - Motor state (uint8_t)
            | Byte | Description |
            |------|-------------|
            | 0x00 | Opened      |
            | 0x10 | Closed      |
        - Error state (uint8_t)
            | Bit |     Description    | 0 |              1                |
            |-----|--------------------|---|-------------------------------|
            |  0  | Low voltage        | 0 | Low voltage protection        |
            |  1  | High voltage       | 0 | High voltage protection       |
            |  2  | Driver temperature | 0 | Driver temperature over limit |
            |  3  | Motor temperature  | 0 | Motor temperature over limit  |
            |  4  | Current            | 0 | Over current                  |
            |  5  | Short circuit      | 0 | Short circuit                 |
            |  6  | Stall              | 0 | Stall                         |
            |  7  | Input signal       | 0 | Input signal timeout          |

        Args:
            response (list): Response data from the motor
        """

        self.temperatures[motor_id] = struct.unpack('b', data[0:1])[0]
        self.voltages[motor_id] = struct.unpack('<H', data[1:3])[0] * 0.01
        self.motor_states[motor_id] = data[5]
        self.error_states[motor_id] = data[6]

        self._debug_print(f"[State1] Temp: {self.temperatures[motor_id]}°C, Voltage: {self.voltages[motor_id]:.2f}V, Motor {'ON' if self.motor_states[motor_id]==0x00 else 'OFF'}, ErrorState: 0x{self.error_states[motor_id]:02X}", level=MEDIUM)

        error = [
            "Low voltage protection",
            "High voltage protection",
            "Driver temperature over limit",
            "Motor temperature over limit",
            "Over current",
            "Short circuit",
            "Stall",
            "Input signal timeout"
        ]

        for bit in range(8):
            if self.error_states[motor_id] & (1 << bit):
                self._debug_print(f"[State1] Error: {error[bit]}", level=LOW)
    
    def _parse_state_2(self, motor_id, data):
        """
        Parse the response data from the motor.

        This function updates:
        - Motor temperature (int8_t, 1 °C/LSB)
        - Motor iq (int16_t)
            | Model |     Unit      |
            |-------|---------------|
            | MF    | 33/4096 A/LSB |
            | MG    | 66/4096 A/LSB |
        - Motor speed (int16_t, 1 dps/LSB)
        - Encoder value (uint16_t)
            | Model |   Range   |
            |-------|-----------|
            | 14bit | 0 ~ 16383 |
            | 15bit | 0 ~ 32767 |
            | 16bit | 0 ~ 65535 |

        Args:
            response (list): Response data from the motor
        """ 
        self.temperatures[motor_id] = struct.unpack('b', data[0:1])[0]
        self.iqs[motor_id] = struct.unpack('<h', data[1:3])[0]  # signed int16
        self.speeds[motor_id] = struct.unpack('<h', data[3:5])[0]
        self.encoder_values[motor_id] = struct.unpack('<H', data[5:7])[0] 

        self._debug_print(f"[State2] Temp: {self.temperatures[motor_id]}°C, Torque/Power: {self.iqs[motor_id]}, Speed: {self.speeds[motor_id]} dps, Encoder: {self.encoder_values[motor_id]}", level=MEDIUM)     

    def _parse_state_3(self, motor_id, data):
        """
        Parse the response data from the motor.

        This function updates:
        - Motor temperature (int8_t, 1 °C/LSB)
        - Phase A, B, C current (int16_t)
            | Model |     Unit      |
            |-------|---------------|
            | MF    | 33/4096 A/LSB |
            | MG    | 66/4096 A/LSB |

        Args:
            response (list): Response data from the motor
        """

        self.temperatures[motor_id] = struct.unpack('b', data[0:1])[0]
        self.currents_A[motor_id] = struct.unpack('<h', data[1:3])[0] / 64.0  # 1A = 64 LSB
        self.currents_B[motor_id] = struct.unpack('<h', data[3:5])[0] / 64.0
        self.currents_C[motor_id] = struct.unpack('<h', data[5:7])[0] / 64.0
        
        self._debug_print(f"[State3] Temp: {self.temperatures[motor_id]}°C, Phase Currents: A={self.currents_A[motor_id]:.2f}A, B={self.currents_B[motor_id]:.2f}A, C={self.currents_C[motor_id]:.2f}A", level=MEDIUM)

    # === High-level Commands ===

    def set_debug_level(self, level):
        """
        Set the debug level for logging.
        
        Args:
            level (int): Debug level (OFF, LOW, MEDIUM, HIGH)
        """
        self.debug_level = level

    def close(self):
        """
        Close the serial connection.

        Args:
            None

        Returns:
            None
        """
        self.serial.close()

    def read_state1(self, motor_id):
        """
        Read and parse the first state packet from the motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            tuple: 
                - int: Temperature in °C  
                - float: Voltage in volts  
                - int: Motor state (0x00 = ON, otherwise OFF)  
                - int: Error state byte (each bit represents a different error condition)
        """
        data = self._send_command(0x9A, motor_id)

        if data:
            self._parse_state_1(motor_id, data)

        return self.temperatures[motor_id], self.voltages[motor_id], self.motor_states[motor_id], self.error_states[motor_id] 

    def clear_error(self, motor_id):
        """
        Clear the error state of the specified motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            bool: 
                - True if the command was acknowledged  
                - False if there was a communication error
        """
        return self._send_command(0x9B, motor_id)

    def read_state2(self, motor_id):
        """
        Read and parse the second state packet from the motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value
        """
        data = self._send_command(0x9C, motor_id)

        if data:
            self._parse_state_2(motor_id, data)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def read_state3(self, motor_id):
        """
        Read and parse the third state packet from the motor.
        
        Args:
            motor_id (int): ID of the target motor
            
        Returns:
            tuple:
                - int: Temperature in °C
                - float: Phase A current in Amperes
                - float: Phase B current in Amperes
                - float: Phase C current in Amperes
        """
        data = self._send_command(0x9D, motor_id)

        if data:
            self._parse_state_3(motor_id, data)

        return self.temperatures[motor_id], self.currents_A[motor_id], self.currents_B[motor_id], self.currents_C[motor_id]

    def motor_off(self, motor_id):
        """
        Turn off the specified motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            bool:
                - True if the command was acknowledged
                - False if there was a communication error
        """
        self._debug_print("[Driver Info] Motor ", motor_id, " OFF", level=MEDIUM)
        return self._send_command(0x80, motor_id)

    def motor_on(self, motor_id):
        """
        Turn on the motor and retrieve driver and motor information.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            bool: 
                - True if the motor was successfully turned on and gear ratio was retrieved  
                - False if communication failed or gear ratio could not be extracted

        Raises:
            ValueError: If gear ratio cannot be extracted from the motor name string
        """
        turned_on = self._send_command(0x88, motor_id)
        self._debug_print("[Driver Info] Motor ", motor_id, " ON", level=MEDIUM)

        # Extract gear ratio
        _, motor, _, _, _, _ = self.read_driver_info(motor_id)

        if motor:
            index = motor.find('-i')
            if index != -1:
                gr = ''
                i = index + 2  # start just after '-i'
                while i < len(motor) and motor[i].isdigit():
                    gr += motor[i]
                    i += 1
                gr = float(gr)
            else: raise ValueError("Impossible to extract the motor gear ratio.")

            self._debug_print(f"- Gear Ratio: {gr}", level=MEDIUM)

            self.gear_ratios[motor_id] = gr
            return turned_on

        else:
            self._debug_print("[Driver Info] No response.", level=LOW)
            return False

    def motor_stop(self, motor_id):
        """
        Stop the motor immediately.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            bool:
                - True if the command was acknowledged
                - False if there was a communication error
        """
        self._debug_print("[Driver Info] Motor ", motor_id, " STOP", level=MEDIUM)
        return self._send_command(0x81, motor_id)

    def motor_brake(self, motor_id, mode):
        """
        Control or query the motor brake state.

        Args:
            motor_id (int): ID of the target motor
            mode (int): Brake operation mode  
                - 0: Engage brake (motor locked)  
                - 1: Release brake (motor free to move)  
                - 2: Read brake state

        Returns:
            bool | None:  
                - For modes 0 and 1, returns the result of the command (True if acknowledged, False otherwise)  
                - For mode 2, returns None after printing the brake state  
                - Raises ValueError for invalid mode
        """


        if mode == 0:
            data = bytes([0x00])  # engage brake
            self._debug_print("[Brake] Engaging brake (motor locked).", level=MEDIUM)
            return self._send_command(0x8C, motor_id, data)

        elif mode == 1:
            data = bytes([0x01])  # release brake
            self._debug_print("[Brake] Releasing brake (motor free to move).", level=MEDIUM)
            return self._send_command(0x8C, motor_id, data)

        elif mode == 2:
            data = bytes([0x10])  # read brake state
            response = self._send_command(0x8C, motor_id, data)
            if response:
                brake_byte = response[0]
                status = "released" if brake_byte == 0x01 else "engaged"
                self._debug_print(f"[Brake] Holding brake is {status}.", level=MEDIUM)
        else:
            raise ValueError("Invalid mode. Use 0 (lock), 1 (release), 2 (query).")

    def torque_control(self, motor_id, iq_control):
        """
        Send a torque control command to the motor.

        Args:
            motor_id (int): ID of the target motor  
            iq_control (int): Desired torque value (int16), range -2048 to +2048  
                (e.g., ±33A for MG series)

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value

        Raises:
            ValueError: If iq_control is outside the valid range
        """
        if not -2048 <= iq_control <= 2048:
            raise ValueError("iq_control must be between -2048 and +2048")

        data = struct.pack('<h', iq_control)  # 2-byte signed little endian
        self._debug_print(f"[Torque] Sent iqControl = {iq_control}", level=MEDIUM)

        response = self._send_command(0xA1, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def speed_control(self, motor_id, speed_dps):
        """
        Send a speed control command to the motor.

        Args:
            motor_id (int): ID of the target motor  
            speed_dps (int | float): Desired speed in degrees per second (°/s)  
                Converted to 0.01 dps/LSB internally and scaled by gear ratio

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value

        Raises:
            ValueError: If speed_dps is not a number

        Notes:
            1. Speed is limited by "Max Speed" setting in the LK Motor Tool  
            2. Acceleration is limited by "Max Acceleration" setting  
            3. Torque or power is limited by motor type-specific limits in the LK Motor Tool
        """
        if not isinstance(speed_dps, (int, float)):
            raise ValueError("speed_dps must be a number")
        
        speed_val = int(speed_dps * 100 * self.gear_ratios[motor_id])  # 0.01 dps/LSB
        data = struct.pack('<i', speed_val)
        self._debug_print(f"[Speed] Sent speed = {speed_dps:.2f}°/s ({speed_val} LSB)", level=MEDIUM)

        response = self._send_command(0xA2, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def multi_loop_angle1(self, motor_id, angle_deg):
        """
        Move the motor to an absolute multi-turn angle position.

        Args:
            motor_id (int): ID of the target motor  
            angle_deg (float): Target angle in degrees (360.00 = one full turn)  
                Internally converted to 0.01 deg/LSB and scaled by gear ratio

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value
        """
        angle_val = int(angle_deg * 100 * self.gear_ratios[motor_id])  # 0.01 deg/LSB
        data = struct.pack('<q', angle_val)  # 8 bytes int64
        self._debug_print(f"[Multi-loop 1] Moving to {angle_deg:.2f}°", level=MEDIUM)
        
        response = self._send_command(0xA3, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def multi_loop_angle2(self, motor_id, angle_deg, max_speed_dps):
        """
        Move the motor to an absolute multi-turn angle with a speed limit.

        Args:
            motor_id (int): ID of the target motor  
            angle_deg (float): Target angle in degrees (360.00 = one full turn)  
            max_speed_dps (float): Maximum allowed speed in degrees per second

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value
        """
        angle_val = int(angle_deg * 100 * self.gear_ratios[motor_id])
        speed_val = int(max_speed_dps * 100 * self.gear_ratios[motor_id])

        data = struct.pack('<qI', angle_val, speed_val)
        self._debug_print(f"[Multi-loop 2] {angle_deg:.2f}° @ ≤ {max_speed_dps:.2f}°/s", level=MEDIUM)
        
        response = self._send_command(0xA4, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)
            
        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def single_loop_angle1(self, motor_id, angle_deg, direction):
        """
        Move the motor to a specified angle within a single 0-360° rotation and return motor state.

        Args:
            motor_id (int): ID of the target motor  
            angle_deg (float): Target angle in degrees, range [0.00 - 359.99)  
            direction (int): Rotation direction  
                - 0: Clockwise (CW)  
                - 1: Counter-clockwise (CCW)

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value

        Raises:
            ValueError: If angle_deg is outside [0, 360) or direction is not 0 or 1
        """
        if not (0 <= angle_deg < 360):
            raise ValueError("angle_deg must be in [0, 360)")
        if direction not in (0, 1):
            raise ValueError("direction must be 0 (CW) or 1 (CCW)")

        angle_val = int(angle_deg * 100 * self.gear_ratios[motor_id])
        data = bytes([direction]) + struct.pack('<H', angle_val) + b'\x00'
        self._debug_print(f"[Single-loop 1] {angle_deg:.2f}° {'CW' if direction == 0 else 'CCW'}", level=MEDIUM)
        
        response = self._send_command(0xA5, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def single_loop_angle2(self, motor_id, angle_deg, direction, max_speed_dps):
        """
        Move the motor to a specified angle within a single 0-360° rotation, with a speed limit.  
        Also returns updated motor state after execution.

        Args:
            motor_id (int): ID of the target motor  
            angle_deg (float): Target angle in degrees, range [0.00 - 359.99)  
            direction (int): Rotation direction  
                - 0: Clockwise (CW)  
                - 1: Counter-clockwise (CCW)  
            max_speed_dps (float): Maximum allowed speed in degrees per second

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value

        Raises:
            ValueError: 
                - If angle_deg is outside [0, 360)  
                - If direction is not 0 or 1
        """
        if not (0 <= angle_deg < 360):
            raise ValueError("angle_deg must be in [0, 360]")
        if direction not in (0, 1):
            raise ValueError("direction must be 0 (CW) or 1 (CCW)")

        angle_val = int(angle_deg * 100 * self.gear_ratios[motor_id])
        speed_val = int(max_speed_dps * 100 * self.gear_ratios[motor_id])

        data = bytes([direction]) + struct.pack('<H', angle_val) + b'\x00' + struct.pack('<I', speed_val)
        self._debug_print(f"[Single-loop 2] {angle_deg:.2f}° {'CW' if direction == 0 else 'CCW'} @ ≤ {max_speed_dps:.2f}°/s", level=MEDIUM)

        response = self._send_command(0xA6, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]
    
    def increment_angle1(self, motor_id, delta_deg):
        """
        Move the motor by a relative angle increment and return updated motor state.

        Args:
            motor_id (int): ID of the target motor  
            delta_deg (float): Relative angle in degrees  
                - Positive values: Clockwise (CW)  
                - Negative values: Counter-clockwise (CCW)

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value
        """
        delta_val = int(delta_deg * 100 * self.gear_ratios[motor_id])  # 0.01 deg/LSB
        data = struct.pack('<i', delta_val)  # int32
        self._debug_print(f"[Increment 1] Moving by {delta_deg:.2f}°", level=MEDIUM)

        response = self._send_command(0xA7, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def increment_angle2(self, motor_id, delta_deg, max_speed_dps):
        """
        Move the motor by a relative angle with a specified speed limit.  
        Returns the updated motor state after execution.

        Args:
            motor_id (int): ID of the target motor  
            delta_deg (float): Relative angle in degrees  
                - Positive values: Clockwise (CW)  
                - Negative values: Counter-clockwise (CCW)  
            max_speed_dps (float): Maximum allowed speed in degrees per second

        Returns:
            tuple:
                - int: Temperature in °C  
                - int: Torque/Power (Iq value)  
                - int: Speed in degrees per second (dps)  
                - int: Encoder value
        """
        delta_val = int(delta_deg * 100* self.gear_ratios[motor_id])
        speed_val = int(max_speed_dps * 100* self.gear_ratios[motor_id])

        data = struct.pack('<iI', delta_val, speed_val)  # int32 delta, uint32 speed
        self._debug_print(f"[Increment 2] Δ = {delta_deg:.2f}°, max speed = {max_speed_dps:.2f}°/s", level=MEDIUM)

        response = self._send_command(0xA8, motor_id, data)

        if response:
            self._parse_state_2(motor_id, response)

        return self.temperatures[motor_id], self.iqs[motor_id], self.speeds[motor_id], self.encoder_values[motor_id]

    def read_encoder(self, motor_id):
        """
        Read the encoder data from the motor and update internal state.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            tuple:
                - int: Encoder value (scaled position)  
                - int: Raw encoder count  
                - int: Encoder offset
        """
        data = self._send_command(0x90, motor_id)

        if data:
            self.encoder_values[motor_id] = struct.unpack('<H', data[0:2])[0]
            self.encoder_raws[motor_id] = struct.unpack('<H', data[2:4])[0]
            self.encoder_offsets[motor_id] = struct.unpack('<H', data[4:6])[0]

        self._debug_print(f"[Encoder] Value: {self.encoder_values[motor_id]}, Raw: {self.encoder_raws[motor_id]}, Offset: {self.encoder_offsets[motor_id]}", level=MEDIUM)

        return self.encoder_values[motor_id], self.encoder_raws[motor_id], self.encoder_offsets[motor_id]

    # def set_zero_rom(self, motor_id): 
    #     """
    #     This command will write the zero point to the driver's FLASH, multiple writes will affect
    #     the chip life, and frequent use is not recommended.
    #     """
    #     data = self._send_command(0x19, motor_id)
    #     self._debug_print("[Set Zero ROM] Sent. (Flashes ROM!)", level=MEDIUM)

    def read_multi_loop_angle(self, motor_id):
        """
        Read the absolute multi-turn angle from the motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            float: Multi-turn angle in degrees (absolute position)
        """
        data = self._send_command(0x92, motor_id)
        
        if data:
            self.multi_turn_angles[motor_id] = struct.unpack('<q', data[0:8])[0] * 0.01 / self.gear_ratios[motor_id]

        self._debug_print(f"[Multi-loop Angle] {self.multi_turn_angles[motor_id]:.2f}°", level=MEDIUM)

        return self.multi_turn_angles[motor_id]

    def clear_multi_loop_angle(self, motor_id):
        """
        Clear the stored multi-turn angle value in the motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            bool or None: Result of the clear command, typically True if successful.
        """
        self._debug_print("[Multi-loop Angle] Cleared.", level=MEDIUM)
        return self._send_command(0x93, motor_id)
        
    def read_single_loop_angle(self, motor_id):
        """
        Read the absolute single-turn (0-360°) angle from the motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            float: Single-turn angle in degrees within one rotation (0° to <360°)
        """
        data = self._send_command(0x94, motor_id)
        
        if data:
            self.single_turn_angles[motor_id] = struct.unpack('<I', data[0:4])[0] * 0.01 / self.gear_ratios[motor_id]

        self._debug_print(f"[Single-loop Angle] {self.single_turn_angles[motor_id]:.2f}°", level=MEDIUM)

        return self.single_turn_angles[motor_id]

    def set_zero_ram(self, motor_id):
        """
        Set the current motor position as zero in volatile RAM.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            bool or None: Result of the command, typically True if successful.
        """
        self._debug_print("[Set Zero RAM] Position set as zero (volatile).", level=MEDIUM)
        return self._send_command(0x95, motor_id)

    # def read_pid(self, motor_id, param_id):
    #     data = struct.pack('<BB', param_id, 0x00)
    #     raw = self.send_command(0x40, motor_id, data, expect_len=13)
    #     if len(raw) < 13:
    #         print("[Read PID] No response.")
    #         return
    #     print(f"[Read PID] ParamID: {param_id}, Data: {list(raw[6:13])}")

    # def write_pid_ram(self, motor_id, param_id, param_bytes):
    #     if len(param_bytes) != 6:
    #         raise ValueError("param_bytes must be 6 bytes")
    #     data = bytes([param_id]) + param_bytes
    #     self.send_command(0x42, motor_id, data, expect_len=10)
    #     print(f"[Write PID RAM] ParamID: {param_id} written (volatile)")

    # def write_pid_rom(self, motor_id, param_id, param_bytes):
    #     if len(param_bytes) != 6:
    #         raise ValueError("param_bytes must be 6 bytes")
    #     data = bytes([param_id]) + param_bytes
    #     self.send_command(0x44, motor_id, data, expect_len=10)
    #     print(f"[Write PID ROM] ParamID: {param_id} written to flash")

    def read_driver_info(self, motor_id):
        """
        Read and parse driver information from the motor.

        Args:
            motor_id (int): ID of the target motor

        Returns:
            tuple:
                - str: Driver name or model (ASCII)
                - str: Motor name or model (ASCII)
                - str: Serial number (ASCII)
                - float: Hardware version (e.g., 1.0)
                - float: Motor version (e.g., 1.0)
                - float: Firmware version (e.g., 1.0)
        """
        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            data = self._send_command(0x12, motor_id)
            if data:
                driver = data[0:20].decode('ascii', errors='ignore').strip('\x00')
                motor = data[20:40].decode('ascii', errors='ignore').strip('\x00')
                serial = data[40:52].decode('ascii', errors='ignore').strip('\x00')
                hw_ver = struct.unpack('<H', data[52:54])[0] / 10.0
                motor_ver = struct.unpack('<H', data[54:56])[0] / 10.0
                fw_ver = struct.unpack('<H', data[56:58])[0] / 10.0

                self._debug_print(f"[Driver Info]\n- Driver: {driver}\n- Motor: {motor}\n- SN: {serial}", level=MEDIUM)
                self._debug_print(f"- HW Ver: {hw_ver:.1f}, Motor Ver: {motor_ver:.1f}, FW Ver: {fw_ver:.1f}", level=MEDIUM)

                return driver, motor, serial, hw_ver, motor_ver, fw_ver

        raise RuntimeError("Failed to read driver information after 3 attempts. Check motor connection or ID.")


if __name__=="__main__":

    import time

    motor = K_Tech_RS485('/dev/ttyUSB0')  # or COMx on Windows
    motor.set_debug_level(OFF)
    motor_id = 1
    motor_id_2 = 2

    motor.motor_on(motor_id)
    motor.motor_on(motor_id_2)

    angle1 = motor.read_multi_loop_angle(motor_id)
    angle2 = motor.read_multi_loop_angle(motor_id_2)
    time.sleep(0.5)

    motor.multi_loop_angle2(motor_id, angle1, 200)
    motor.multi_loop_angle2(motor_id_2, angle2, 200)

    while True:
        start = time.time()
        for i in range(50):
            motor.multi_loop_angle2(motor_id, angle1, 200)
            motor.multi_loop_angle2(motor_id_2, angle2, 200)
            angle1 += 0.01
            angle2 += 0.01
        end = time.time()

        print("Command frequency:", (50 / (end - start)), "Hz")

    motor.motor_off(motor_id)
        
    motor.close()