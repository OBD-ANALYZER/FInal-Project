###########################################################################
# ELM327-emulator
# ELM327 Emulator for testing software interfacing OBDII via ELM327 adapter
# https://github.com/Ircama/ELM327-emulator
# (C) Ircama 2021 - CC-BY-NC-SA-4.0
###########################################################################

import logging
import logging.config
import os
import re
import socket
from enum import Enum
import serial
import threading
import time
import traceback
import errno
from random import choices
import string
from xml.etree.ElementTree import fromstring, ParseError
import importlib
import pkgutil
import inspect

if not os.name == 'nt':
    import pty
    import tty

# Local imports
from .car_emulator import Car
from .real_time_handler import RealTimeHandler, DataUpdateMode, VehicleData, PID
from .obd_message import ObdMessage, ELM_R_UNKNOWN, ST, ECU_ADDR_E

# Configuration constants__________________________________________________
FORWARD_READ_TIMEOUT = 0.2  # seconds
SERIAL_BAUDRATE = 38400  # bps
NETWORK_INTERFACES = ""
PLUGIN_DIR = __package__ + ".plugins"
MAX_TASKS = 20
ISO_TP_MULTIFRAME_MODULE = 'ISO-TP request pending'
# Minimum size to use a UDS header with additional length byte (ISO 14230-2)
MIN_SIZE_UDS_LENGTH = 20
INTERRUPT_TASK_IF_NOT_HEX = False
ELM_VALID_CHARS = r"^[a-zA-Z0-9 \n\r\b\t@,.?]*$"
ECU_TASK = "task_ecu_"
DEFAULT_ECU_TASK = 'Default ECU Task module'
ELM_VERSION = "ELM327 v1.5"
ELM_HEADER_VERSION = "\r\r"

"""
Ref. to ISO 14229-1 and ISO 14230, this is a list of SIDs (UDS service
identifiers) which have additional sub-function bytes in the related
positive answer. The value indicates the number of bytes to add to the
answer for each requested SID. Not included SIDs in this list have 0
additional bytes in the answer.
"""
uds_sid_pos_answer = {
    "01": 1,  # Show current data
    "02": 1,  # Show freeze frame data
    "05": 1,  # Test results, oxygen sensor monitoring
    "09": 1,  # Request vehicle information
    "10": 1,  # Diagnostic Session Control (DSC)
    "11": 1,  # ECU Reset (ER)
    "14": 0,  # Clear Diagnostic Information DTC (CDTCI)
    "19": 2,  # Read DTC Information
    "21": 1,  # Read Data by Local Id
    "22": 2,  # Read Data By Identifier (RDBI)
    "23": 0,  # Read memory by address (RMBA)
    "24": 0,  # Read Scaling Data By Identifier
    "27": 1,  # Security Access (SA)
    "2A": 0,  # Read Data By Periodic Identifier
    "2C": 0,  # Dynamically Define Data Identifier
    "2E": 2,  # Write Data By Identifier (WDBI)
    "2F": 0,  # Input Output Control By Identifier
    "30": 1,  # IO Control by Local Id
    "31": 1,  # Routine Control - Start Routine by Local ID (RC)
    "38": 0,  # Start Routine by Address
    "3B": 1,  # ?
    "3D": 0,  # Write Memory by Address (WMBA)
    "3E": 1,  # Tester Present (TP)
    "85": 1,  # Control DTC Setting
}


# End of configuration constants_______________________________________________


class Tasks:
    """
    Base class for tasks.
    All tasks/plugins shall implement a class named Task derived from Tasks.
    """

    class RETURN:
        """
        Return values for all Tasks methods
        """
        TERMINATE = False
        CONTINUE = True
        ERROR = (None, TERMINATE, None)
        INCOMPLETE = (None, CONTINUE, None)

        def PASSTHROUGH(cmd): return None, Tasks.RETURN.TERMINATE, cmd

        def TASK_CONTINUE(cmd): return None, Tasks.RETURN.CONTINUE, cmd

        def ANSWER(answer): return answer, Tasks.RETURN.TERMINATE, None

    def __init__(self, emulator, pid, header, ecu, request, attrib,
                 do_write=False):
        self.emulator = emulator  # reference to the emulator namespace
        self.shared = None  # A ISO-TP Multiframe special task will not use a shared namespace
        if ecu in self.emulator.task_shared_ns:
            self.shared = self.emulator.task_shared_ns[ecu]  # shared namespace
        if pid:  # None if ECU Task, pid if ELM command Task
            self.pid = pid  # PID label
            self.header = header  # request header
            # original request data (stored before running the start() method)
            self.request = request
            self.attrib = attrib  # dictionary element (None if not pertinent)
            self.do_write = do_write  # (boolean) will write to the application
            self.frame = None  # ISO-TP Multiframe request frame counter
            self.length = None  # ISO-TP Multiframe request length counter
            self.flow_control = 0  # ISO-TP Multiframe request flow control
            self.flow_control_end = 0x20  # ISO-TP Multiframe request flow control repetitions
            self.ecu = ecu  # ECU name
        else:
            self.shared = self  # ECU Task
        self.logging = emulator.logger  # logger reference
        # timer (to be used to simulate background processing)
        self.time_started = time.time()

    def HD(self, header):
        """
        Generates the XML tag related to the header byte of the response (ECU ID)
        :param size: header (ECU ID)
        :return: XML tag related to the header of the response
        """
        return ('<header>' + header + '</header>')

    def SZ(self, size):
        """
        Generates the XML tag related to the size byte of the response
        :param size: string including the size byte
        :return: XML tag related to the size byte of the response
        """
        return ('<size>' + size + '</size>')

    def DT(self, data):
        """
        Generates the XML tag related to the data part of the response
        :param data: data part (string of hex data spaced every two bytes)
        :return: XML tag related to the data part of the response
        """
        return ('<data>' + data + '</data>')

    def AW(self, answer):
        """
        Generates the XML tag related to the response, which will be
        automatically translated in header, size and data.
        :param answer: data part (string of hex data)
        :return: XML tag related to the response
        """
        return ('<answer>' + answer + '</answer>')

    def PA(self, pos_answer):
        """
        Generates a positive answer XML tag, including header, size and data.
        :param answer: data part (string of hex data)
        :return: XML tag related to the response
        """
        return ('<pos_answer>' + pos_answer + '</pos_answer>')

    def NA(self, neg_answer):
        """
        Generates a negative answer XML tag, including header, size and data.
        :param answer: data part (string of hex data)
        :return: XML tag related to the response
        """
        return ('<neg_answer>' + neg_answer + '</neg_answer>')

    def task_get_request(self):
        """
        Get the original request command that initiated the task (used to
        generate the answers)
        :return: return the original request string
        """
        return self.request

    def task_request_matched(self, request):
        """
        Check whether the request in the argument matches the original request
        that invoked the task.
        :param request:
        :return: boolean (true if the given request matches the original task request)
        """
        if not self.attrib:
            return None
        return re.match(self.attrib['REQUEST'], request)

    def start(self, cmd, length=None, frame=None):
        """
        This method is executed when the task is started.
        If not overridden, it calls run()
        :param cmd: request to process
        :return: tuple of three values:
            - XML response (or None for no output)
            - boolean to terminate the task or to keep it active
            - request to be subsequently processed after outputting the XML
                response in the first element (or Null to disable subsequent
                processing)
        """
        return self.run(cmd, length, frame)

    def stop(self, cmd, length=None, frame=None):
        """
        This method is executed when the task is interrupted by an error.
        If not overridden, it returns an error.
        :param cmd: request to process
        :return: tuple of three values:
            - XML response (or None for no output)
            - boolean to terminate the task or to keep it active
            - request to be subsequently processed after outputting the XML
                response in the first element (or Null to disable subsequent
                processing)
        """
        return Tasks.RETURN.ERROR

    def run(self, cmd, length=None, frame=None):
        """
        Main method to be overridden by the actual task; it is always run
        if start and stop are not overridden, otherwise it is run for the
        subsequent frames after the first one
        :param cmd: request to process
        :return: tuple of three values:
            - XML response (or None for no output)
            - boolean to terminate the task or to keep it active
            - request to be subsequently processed after outputting the XML
                response in the first element (or Null to disable subsequent
                processing)
        """
        return Tasks.RETURN.PASSTHROUGH(cmd)


class EcuTasks(Tasks):
    """
    ECU Task (same as normal tasks, but return is set to continue by default)
    """

    def run(self, cmd, length=None, frame=None):
        return EcuTasks.RETURN.TASK_CONTINUE(cmd)


class IsoTpMultiframe(Tasks):
    """
    Special task to aggregate an ISO-TP Multiframe request into a single string
    before processing the request.
    """

    def run(self, cmd, length=None, frame=None):
        """
        Compose a ISO-TP Multiframe request. Call it on each request fragment,
        passing the standard method parameters, until data is returned.

        :param cmd: frame data (excluding header and length)
        :param length: decimal value of the length byte of a ISO-TP Multiframe frame
        :param frame: can be None (single frame), 0 (First Frame) or > 0 (subsequent frame)
        :return:
            error = Tasks.TASK.ERROR
            incomplete request = Tasks.TASK.INCOMPLETE
            complete request = Tasks.TASK.PASSTHROUGH(cmd)
        """
        if frame is not None and frame == 0 and length > 0:  # First Frame (FF)
            if self.frame or self.length:
                self.logging.error('Invalid initial frame %s %s', length, cmd)
                return Tasks.RETURN.ERROR
            self.req = cmd
            self.frame = 1
            self.length = length
        elif (frame is not None and frame > 0 and self.frame == frame and
              length is None):  # valid Consecutive Frame (CF)
            self.req += cmd
            self.frame += 1
        elif (frame is not None and frame == -1 and self.frame == 16 and
              length is None):  # valid Consecutive Frame (CF) - 20 after 2F
            self.req += cmd
            self.frame = 1  # re-cycle the input frame count to 21
        elif ((length is None or length > 0) and
              frame is None and self.frame is None):  # Single Frame (SF)
            self.req = cmd
            self.length = length
            if length:
                return Tasks.RETURN.PASSTHROUGH(self.req[:self.length * 2])
            else:
                return Tasks.RETURN.PASSTHROUGH(self.req)
        else:
            self.logging.error(
                'Invalid consecutive frame %s with data %s, stored frame: %s',
                frame, repr(cmd), self.frame)
            return Tasks.RETURN.ERROR

        # Process Flow Control (FC)
        if self.flow_control:
            self.flow_control -= 1
        else:
            if ('cmd_cfc' not in self.emulator.counters or
                    self.emulator.counters['cmd_cfc'] == 1):
                resp = self.emulator.handle_response(
                    ('<flow>' + hex(self.flow_control_end)[2:].upper() +
                     ' 00</flow>'),
                    do_write=self.do_write,
                    request_header=self.header,
                    request_data=cmd)
                if not self.do_write:
                    self.logging.warning("Output data: %s", repr(resp))
            self.flow_control = self.flow_control_end - 1

        if self.length * 2 <= len(self.req):
            self.frame = None
            return Tasks.RETURN.PASSTHROUGH(self.req[:self.length * 2])
        return Tasks.RETURN.INCOMPLETE


def is_hex_sp(s):
    """
    Validate a string containing hex (in any number, not necessarily
    grouped into digit pairs because the header might have three digits),
    or spaces, or newlines.
    For instance, if containing a PID, it returns True, if containing
    ST or AT commands, it returns False.
    :param s: string to validate
    :return: True if matching, otherwise False
    """
    return re.match(r"^[0-9a-fA-F \t\r\n]*$", s or "") is not None


def len_hex(s):
    """
    Check that the argument string is hexadecimal (digit pairs). If not,
    return False. If hex, return the number of hex bytes (digit pairs).
    :param s: hex string 
    :return: either the number of hex bytes (0 or more bytes) or False (invalid
    digit, or digits not grouped into pairs).
    """
    try:
        return len(bytearray.fromhex(s))
    except Exception:
        return False


class Elm:
    """
    Main class of the ELM327-emulator
    """

    class THREAD:
        """
        Possible states for the Context Manager thread
        """
        STOPPED = 0
        STARTING = 1
        ACTIVE = 2
        PAUSED = 3
        TERMINATED = 4

    def set_defaults(self):
        """Initialize default values for the emulator."""
        self.counters = {}
        self.tasks = {}
        self.task_shared_ns = {}
        self.answer = {}
        self.scenario = "DEFAULT"
        self.delay = 0
        self.max_req_timeout = 5
        self.multiframe_timer = 5
        self.interbyte_out_delay = 0
        self.threadState = self.THREAD.STOPPED

    def __init__(
            self,
            batch_mode=False,
            newline=False,
            no_echo=False,
            serial_port=None,
            device_port=None,
            serial_baudrate="",
            net_port=None,
            forward_net_host=None,
            forward_net_port=None,
            forward_serial_port=None,
            forward_serial_baudrate=None,
            forward_timeout=None):
        """Initialize the emulator."""
        self.logger = logging.getLogger(__name__)
        self.last_update = time.time()
        self.car = Car()
        self.database = {
            "rpm": 0,
            "speed": 0,
            "engine_temp": 70,
            "fuel_level": 100.0,
            "gear": 1,
            "gear_position": "N",
        }
        self.version = ELM_VERSION
        self.header_version = ELM_HEADER_VERSION
        self.presets = {}
        self.ObdMessage = ObdMessage
        self.ELM_R_UNKNOWN = ELM_R_UNKNOWN
        self.set_defaults()
        self.set_sorted_obd_msg()
        self.batch_mode = batch_mode
        self.newline = newline
        self.no_echo = no_echo
        self.serial_port = serial_port
        self.device_port = device_port
        self.serial_baudrate = serial_baudrate
        self.net_port = net_port
        self.forward_net_host = forward_net_host
        self.forward_net_port = forward_net_port
        self.forward_serial_port = forward_serial_port
        self.forward_serial_baudrate = forward_serial_baudrate
        self.forward_timeout = forward_timeout
        self.reset(0)
        self.slave_name = None  # pty port name, if pty is used
        # pty port FD, if pty is used, or device com port FD (IO)
        self.master_fd = None
        self.slave_fd = None  # pty side used by the client application
        self.serial_fd = None  # serial COM port file descriptor (pySerial)
        self.sock_inet = None
        self.fw_sock_inet = None
        self.fw_serial_fd = None
        self.sock_conn = None
        self.sock_addr = None
        self.thread = None
        self.plugins = {}
        self.request_timer = {}
        self.choice_mode = self.Choice.SEQUENTIAL
        self.choice_weights = [1]
        self.net_port = net_port
        self.socket = None
        self.client = None
        self.running = False
        self.real_time_handler = RealTimeHandler()
        
    class Choice(Enum):
        SEQUENTIAL = 0
        RANDOM = 1

    def __enter__(self):
        self.real_time_handler.start()
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.real_time_handler.stop()
        if self.socket:
            self.socket.close()
            
    def socket_server(self):
        """Initialize and start the socket server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # Enable keepalive
            self.socket.bind(('', self.net_port))
            self.socket.listen(1)
            logging.info(f"Socket server started on port {self.net_port}")
            return True
        except Exception as e:
            logging.error(f"Failed to create socket server: {e}")
            return False
            
    def handle_pid_request(self, pid):
        """Handle OBD-II PID requests"""
        try:
            value = self.real_time_handler.get_pid_value(pid)
            if value is not None:
                # Format response according to OBD-II protocol
                if pid == "01 0C":  # RPM
                    # RPM = ((256*A + B)/4)
                    rpm = int(value)  # Send raw value, let app apply the formula
                    a = rpm // 256
                    b = rpm % 256
                    return f"41 0C {a:02X} {b:02X}"
                elif pid == "01 0D":  # Speed
                    # Speed = A (km/h)
                    return f"41 0D {int(value):02X}"
                elif pid == "01 05":  # Engine temp
                    # Temp = A - 40 (°C)
                    temp = int(value + 40)
                    return f"41 05 {temp:02X}"
                elif pid == "01 11":  # Throttle
                    # Throttle = (100/255)*A (%)
                    throttle = int((value * 255) / 100)
                    return f"41 11 {throttle:02X}"
                elif pid == "01 2F":  # Fuel level
                    # Fuel = (100/255)*A (%)
                    fuel = int((value * 255) / 100)
                    return f"41 2F {fuel:02X}"
                elif pid == "01 5E":  # Fuel rate
                    # Fuel rate = ((256*A + B)/20) (L/h)
                    rate = int(value * 20)
                    a = rate // 256
                    b = rate % 256
                    return f"41 5E {a:02X} {b:02X}"
            return "NO DATA"
        except Exception as e:
            logging.error(f"Error handling PID request {pid}: {e}")
            return "NO DATA"

    def handle_command(self, command):
        """Handle incoming OBD commands"""
        try:
            command = command.strip().upper()
            
            if command == "ATZ":  # Reset
                self.reset(0)  # Perform a full reset
                return "ELM327 v1.5"
            elif command == "ATE0":  # Echo off
                self.counters['cmd_echo'] = 0
                return "OK"
            elif command == "ATL0":  # Linefeeds off
                self.counters['cmd_linefeed'] = 0
                return "OK"
            elif command == "ATS0":  # Spaces off
                self.counters['cmd_spaces'] = 0
                return "OK"
            elif command == "ATH0":  # Headers off
                self.counters['cmd_header'] = 0
                return "OK"
            elif command == "ATSP0":  # Auto protocol
                return "OK"
            elif command.startswith("01"):  # Mode 01 (current data)
                pid = command[2:].strip()
                return self.handle_pid_request(command)
            else:
                return "?"
        except Exception as e:
            logging.error(f"Error handling command {command}: {e}")
            return "?"

    def run(self):
        """Main run loop"""
        self.running = True
        while self.running:
            try:
                if not self.client:
                    self.client, _ = self.socket.accept()
                    self.client.settimeout(1.0)  # Set timeout to 1 second
                    logging.info("Client connected")
                    # Initialize connection
                    self.reset(0)
                    
                try:
                    data = self.client.recv(1024).decode().strip()
                    if not data:
                        logging.info("Client disconnected")
                        self.client.close()
                        self.client = None
                        continue
                        
                    response = self.handle_command(data)
                    if response:
                        try:
                            self.client.send((response + "\r\n").encode())
                        except socket.error as e:
                            logging.error(f"Error sending response: {e}")
                            self.client.close()
                            self.client = None
                            continue
                except socket.timeout:
                    # This is normal, just continue
                    continue
                except socket.error as e:
                    logging.error(f"Socket error while handling client: {e}")
                    if self.client:
                        self.client.close()
                        self.client = None
                    continue
                    
            except Exception as e:
                logging.error(f"Error in run loop: {e}")
                if self.client:
                    self.client.close()
                    self.client = None
                time.sleep(1)  # Wait before trying to accept new connections

    def stop(self):
        """Stop the emulator"""
        self.running = False
        if self.client:
            self.client.close()
        if self.socket:
            self.socket.close()

    def connect_serial(self):
        """
        Shall be called after get_pty() and before a read operation.
        It opens the serial port, if not yet opened.
        It is expected to be blocking.
        Returns True if the serial or pty port is opened,
        or None in case of error.
        """

        # if the port is already opened, return True...
        if self.slave_name or self.master_fd or self.serial_fd:
            return True

        # else open the port
        if self.device_port:  # os IO
            try:
                self.master_fd = os.open(
                    self.device_port,
                    os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
            except Exception as e:
                logging.critical("Error while opening device %s:\n%s",
                                 repr(self.device_port), e)
                return None
            return True
        elif self.serial_port:  # pySerial COM
            try:
                self.serial_fd = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.serial_baudrate or SERIAL_BAUDRATE)
                self.slave_name = self.get_port_name(extended=True)
            except Exception as e:
                logging.critical("Error while opening serial COM %s:\n%s",
                                 repr(self.serial_port), e)
                return None
            return True
        else:
            return False

    def get_pty(self):
        """
        Return the opened pty port, or None if the pty port
        cannot be opened (non UNIX system).
        In case of UNIX system and if the port is not yet opened, open it.
        It is not blocking.
        """

        # if the port is already opened, return the port name...
        if self.slave_name:
            return self.slave_name
        elif self.master_fd and self.device_port:
            return self.device_port
        elif self.serial_fd and self.serial_port:
            return self.serial_port
        elif self.master_fd:
            logging.critical("Internal error, no configured device port.")
            return None
        elif self.serial_fd:
            logging.critical("Internal error, no configured COM port.")
            return None

        # ...else, with a UNIX system, make a new pty
        self.slave_fd = None

        if os.name == 'nt':
            self.slave_fd = None
            return None
        else:
            if not self.device_port and not self.serial_port:
                self.master_fd, self.slave_fd = pty.openpty()
                tty.setraw(self.slave_fd)
                self.slave_name = os.ttyname(self.slave_fd)
                logging.debug("Pty name: %s", self.slave_name)

        return self.slave_name

    def choice(self, values):
        """
        Select one of the values in the list argument according to the adopted
        method, which can be sequential or random.
        :param values: list of possible values
        :return: selected value
        """
        if not isinstance(values, (list, tuple)):
            logging.error(
                'Invalid usage of "choice" function, which needs a list.')
            return ""
        if self.choice_mode == self.Choice.RANDOM:
            len_weights = len(self.choice_weights)
            len_values = len(values)
            return choices(values, [self.choice_weights[i] if i < len_weights
                                    else 1
                                    for i in range(len_values)])[0]
        elif self.choice_mode == self.Choice.SEQUENTIAL:
            if "cmd_last_pid" not in self.counters:
                logging.error(
                    'Internal error - Invalid choice usage; '
                    'missing "cmd_last_pid" counter.')
            return (
                values[int((self.counters[self.counters["cmd_last_pid"]] - 1) /
                           self.choice_weights[0]) % len(values)])
        else:
            logging.error(
                "Internal error - Invalid choice mode.")

    def send_real_time_data(self):
        """Format and send real-time vehicle data as OBD-II PID responses with improved stability."""
        data = self.car.get_real_time_data()
        
        # Add data validation and smoothing
        def validate_and_smooth(value, min_val, max_val, multiplier=1):
            try:
                val = float(value) * multiplier
                return max(min_val, min(max_val, val))
            except (ValueError, TypeError):
                return min_val
                
        # Format data as OBD-II PID responses with proper format and validation
        responses = {
            # RPM: Ensure valid range and smooth conversion
            "01 0C": (lambda rpm: f"41 0C {int(validate_and_smooth(rpm, 0, 16383.75, 4) * 4) >> 8:02X} "
                                f"{int(validate_and_smooth(rpm, 0, 16383.75, 4) * 4) & 0xFF:02X}")(data['rpm']),
            
            # Speed: Validate range 0-255 km/h
            "01 0D": f"41 0D {int(validate_and_smooth(data['speed'], 0, 255)):02X}",
            
            # Engine temp: Valid range -40 to 215 °C
            "01 05": f"41 05 {int(validate_and_smooth(data['engine_temp'], -40, 215) + 40):02X}",
            
            # Gear: Valid range 1-6
            "01 A4": f"41 A4 {int(validate_and_smooth(data['gear'], 1, 6)):02X}",
            
            # Gear position: Ensure valid position character
            "01 A5": (lambda pos: f"41 A5 {ord(pos[0] if pos in ['P', 'R', 'N', 'D'] else 'N'):02X}")(data['gear_position']),
            
            # Throttle: Valid range 0-100%
            "01 11": f"41 11 {int(validate_and_smooth(data['throttle_position'], 0, 100)):02X}",
            
            # Brake: Valid range 0-100%
            "01 12": f"41 12 {int(validate_and_smooth(data['brake_position'], 0, 100)):02X}",
            
            # Fuel level: Valid range 0-100%
            "01 2F": f"41 2F {int(validate_and_smooth(data['fuel_level'], 0, 100) * 255 / 100):02X}",
            
            # Fuel consumption: Valid range 0-3276.75 L/h
            "01 5E": f"41 5E {int(validate_and_smooth(data['fuel_consumption_rate'], 0, 3276.75, 20)):04X}"
        }
        
        return responses

    def handle_obd_commands(self):
        """Handle OBD commands and return responses with improved validation."""
        return self.send_real_time_data()  # Reuse the validated data from send_real_time_data

    def accept_connection(self):
        """
        Perform the "accept" socket method of an INET connection.
        Return when a connection is accepted.
        :return: True if a connection is accepted. False if error.
        """
        if self.sock_conn is None or self.sock_addr is None:

            # Accept network connections
            try:
                logging.debug(
                    "Waiting for connection at %s", self.get_port_name())
                (self.sock_conn, self.sock_addr) = self.sock_inet.accept()
            except OSError as msg:
                if msg.errno == errno.EINVAL:  # [Errno 22] invalid argument
                    return False
                logging.error("Failed accepting connection: %s", msg)
                return False
            logging.debug("Connected by %s", self.sock_addr)
        return True

    def serial_client(self):
        """
        Internally used by send_receive_forward().
        Open the forwarded port if serial mode is used..
        :return: True when successfully opened, otherwise False
        """
        if self.fw_serial_fd:
            return True
        try:
            self.fw_serial_fd = serial.Serial(
                port=self.forward_serial_port,
                baudrate=int(self.forward_serial_baudrate)
                if self.forward_serial_baudrate else SERIAL_BAUDRATE,
                timeout=self.forward_timeout or FORWARD_READ_TIMEOUT)
            return True
        except Exception as e:
            logging.error('Cannot open forward port: %s', e)
            return False

    def net_client(self):
        """
        Internally used by send_receive_forward()
        Open a socket connection if socket mode is used.
        :return: (not really used)
        """
        if (self.fw_sock_inet
                or self.forward_net_host is None
                or self.forward_net_port is None):
            return False
        s = None
        for res in socket.getaddrinfo(
                self.forward_net_host, self.forward_net_port,
                socket.AF_UNSPEC, socket.SOCK_STREAM):
            af, socktype, proto, canonname, sa = res
            try:
                s = socket.socket(af, socktype, proto)
                self.sock_inet.setsockopt(
                    socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock_inet.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError as msg:
                s = None
                continue
            try:
                s.connect(sa)
                s.settimeout(self.forward_timeout or FORWARD_READ_TIMEOUT)
            except OSError as msg:
                s.close()
                s = None
                continue
            break
        if s is None:
            logging.critical(
                "Cannot connect to host %s with port %s",
                self.forward_net_host, self.forward_net_port)
            self.terminate()
            return False
        self.fw_sock_inet = s
        return True

    def send_receive_forward(self, i):
        """
            If a forwarder is active, send data if it is not None
            and try receiving data until a timeout.
            Then received data are logged and returned.

            return False: no connection
            return None: no data
            return data: decoded string
        """

        if self.forward_serial_port:
            if self.fw_serial_fd is None:
                if not self.serial_client():
                    return False
            if self.fw_serial_fd:
                if i:
                    self.fw_serial_fd.write(i)
                    logging.info(
                        "Write forward data: %s", repr(i))
                proxy_data = self.fw_serial_fd.read(1024)
                logging.info(
                    "Read forward data: %s", repr(proxy_data))
                return repr(proxy_data)
            return False

        if not self.forward_net_host or not self.forward_net_port:
            return False
        if self.fw_sock_inet is None:
            self.net_client()
        if self.fw_sock_inet:
            if i:
                try:
                    self.fw_sock_inet.sendall(i)
                    logging.info(
                        "Write forward data: %s", repr(i))
                except BrokenPipeError:
                    logging.error(
                        "The network link of the OBDII interface dropped.")
            try:
                proxy_data = self.fw_sock_inet.recv(1024)
                logging.info(
                    "Read forward data: %s", repr(proxy_data))
                return proxy_data.decode("utf-8", "ignore")
            except socket.timeout:
                logging.info(
                    "No forward data received.")
                return None
        return False

    def get_port_name(self, extended=False):
        """
        Returns the name of the opened port.
        :param extended: False or True
        :return: string
        """
        if self.sock_inet:
            if self.net_port:
                postfix = ''
                if extended:
                    postfix = '\nWarning: the socket is bound ' \
                              'to all interfaces.'
                return ('TCP/IP network port ' + str(self.net_port) + '.'
                        + postfix)
            else:
                return ('Unopened TCP/IP network port ' +
                        str(self.net_port) + '.')

        if self.device_port:
            if os.name == 'nt':
                return ('(invalid) OS communication device "' +
                        self.device_port + '".')
            else:
                return ('OS communication device "' +
                        self.device_port + '".')

        if self.serial_port:
            postfix = ''
            baudrate = ''
            if extended:
                postfix = ' of com0com COM port pair'
                if self.serial_baudrate:
                    baudrate = " with baud rate " + str(
                        self.serial_baudrate or SERIAL_BAUDRATE)

            if os.name == 'nt':
                if self.serial_port == 'COM3':
                    return ('Windows serial COM port "' + self.serial_port + '"'
                            + postfix + baudrate + '.')
                else:
                    return ('serial COM port "' + self.serial_port + '"'
                            + baudrate + '.')
            else:
                return 'serial communication port "' + self.serial_port + '".'

        if self.slave_name:
            if os.name == 'nt':
                return "(invalid) Windows PTY " + self.slave_name + '.'
            else:
                return ('pseudo-tty port "' +
                        self.slave_name + '".')

        return 'unknown port.'

    def read_from_device(self, bytes):
        """
        Read from the port; returns up to bytes characters (generally 1).
        Manage socket, serial or device output.
        Process echo; returns None in case of error

        :param bytes: max number of bytes to read (we use 1 byte a time)
        :return: Read character(s) or None if error.
        """

        # Process inet
        c = None
        if self.sock_inet:
            if not self.accept_connection():
                return None
            try:
                c = self.sock_conn.recv(bytes)
                if len(c) == 0:
                    logging.debug(
                        "TCP/IP communication terminated by the client.")
                    self.sock_conn = None
                    self.sock_addr = None
                    self.reset(0)
                    return None
            except ConnectionResetError:
                logging.warning(
                    "Session terminated by the client.")
                self.sock_conn = None
                self.sock_addr = None
                self.reset(0)
                return None
            except UnicodeDecodeError as msg:
                logging.error(
                    "UTF8 decode error: %s", msg)
                return None
            except Exception as msg:
                logging.error(
                    "Error while reading from network: %s", msg)
                return None
            if 'cmd_echo' not in self.counters or (
                    'cmd_echo' in self.counters and
                    self.counters['cmd_echo']):
                self.sock_conn.sendall(c)
            return c

        # Process serial (COM or device)
        try:
            if not self.connect_serial():
                self.terminate()
                return None

            # Serial COM port (uses pySerial)
            if self.serial_fd and self.serial_port:
                try:
                    c = self.serial_fd.read(bytes)
                except Exception:
                    logging.debug(
                        'Error while reading from %s', self.get_port_name())
                    return None
                if 'cmd_echo' not in self.counters or (
                        'cmd_echo' in self.counters and
                        self.counters['cmd_echo']):
                    self.serial_fd.write(c)

            # Device port (use os IO)
            else:
                if not self.master_fd:
                    logging.critical(
                        "PANIC - Internal error, missing device FD")
                    self.terminate()
                    return None
                c = os.read(self.master_fd, bytes)
                if 'cmd_echo' not in self.counters or (
                        'cmd_echo' in self.counters and
                        self.counters['cmd_echo']):
                    try:
                        os.write(self.master_fd, c)
                    except OSError as e:
                        # [Errno 9] Bad file descriptor/[Errno 5] Input/output error
                        if e.errno == errno.EBADF or e.errno == errno.EIO:
                            logging.debug("Read interrupted. Terminating.")
                            self.terminate()
                            return None
                        else:
                            logging.critical(
                                "PANIC - Internal OSError in read(): %s",
                                e, exc_info=True)
                            self.terminate()
                            return None
        except UnicodeDecodeError as e:
            logging.warning("Invalid character received: %s", e)
            return None
        except OSError:
            return None
        return c

    def normalized_read_line(self):
        """
            Read the next newline delimited command invoking read_from_device()
            Concatenate read characters until newline.
            Manage req_timeout input UDS P4 timer.
            Manage send_receive_forward()
            returns a normalized string command
        """
        buffer = ""
        first = True

        req_timeout = self.max_req_timeout
        try:
            req_timeout = float(self.counters['req_timeout'])
        except Exception as e:
            if 'req_timeout' in self.counters:
                logging.error("Improper configuration of\n\"self.counters"
                              "['req_timeout']\": '%s' (%s). "
                              "Resetting it to %s",
                              self.counters['req_timeout'], e,
                              self.max_req_timeout
                              )
            self.counters['req_timeout'] = req_timeout
        while True:
            prev_time = time.time()
            c = self.read_from_device(1)
            if c is None:
                return None
            c = c.decode("utf-8", "ignore")
            if prev_time + req_timeout < time.time() and first == False:
                buffer = ""
                logging.debug(
                    "'req_timeout' timeout while reading data: %s", c)
            if c == '\r':
                if self.newline:
                    continue
                break
            if c == '\n':
                if self.newline:
                    break
                continue  # ignore newlines
            first = False
            buffer += c

        try:
            self.send_receive_forward((buffer + '\r').encode())
        except Exception as e:
            logging.error('Forward Write error: %s', e)
        return buffer

    def write_to_device(self, i):
        """Write a response to the port (no data returned)."""
        if self.client:
            try:
                if self.interbyte_out_delay:
                    for j in i:
                        self.client.send(bytes([j]))
                        self.client.flush()  # Ensure data is sent immediately
                        time.sleep(self.interbyte_out_delay)
                else:
                    self.client.send(i)
                    self.client.flush()  # Ensure data is sent immediately
            except socket.error as e:
                logging.error(f"Socket error writing to device: {e}")
                self.client.close()
                self.client = None
            except Exception as e:
                logging.error(f"Error writing to device: {e}")
                self.client.close()
                self.client = None

    def uds_answer(
            self,
            data, request_header, use_headers, cra_pattern, sp, nl,
            is_flow_control=None):
        """
        Generate an UDS envelope and answer basing on information included in
        parameters.
        The format should be compliant with ISO-TP 11 bit header or KWP2000.
        :param data: data bytes of the answer
        :param request_header: string containing the header used in the request
                (to be used to compute the response header)
        :param use_headers: boolean to indicate whether the header shall be
                included
        :param cra_pattern: ATCRA match string for header
        :param sp: space string
        :param nl: newline string
        :param is_flow_control: string including the flow control byte
        :return: string including the formatted UDS answer
        """
        answer = ""
        if request_header is None and "cmd_set_header" in self.counters:
            request_header = self.counters['cmd_set_header']
        request_header = (request_header or '').translate(
            (request_header or '').maketrans('', '', string.whitespace)).upper()
        if not request_header:
            logging.error('Invalid request header; request %s', repr(data))
            return ""
        try:
            length = len(bytearray.fromhex(data))
            data = (sp.join('{:02x}'.format(x)
                            for x in bytearray.fromhex(data)).upper())
        except ValueError:
            logging.error('Invalid data in answer: %s', repr(data))
            return ""
        answer_header = hex(int(request_header, 16) + 8)[2:].upper()
        if not re.match(cra_pattern, answer_header):
            logging.debug(
                'Skipping answer which does not match ATCRA: '
                'request_header=%s, answer_header=%s, cra_pattern=%s.',
                repr(request_header), repr(answer_header), repr(cra_pattern))
            return ""
        if len(request_header) == 3 and is_flow_control:  # 11 bit header + FC
            if use_headers:
                answer = answer_header + sp
            answer += is_flow_control + sp + data
        elif len(request_header) == 3:  # ISO-TP 11 bit CAN identifier
            if use_headers:
                if length > 7:  # produce a multframe output
                    answer = (answer_header +
                              sp + "10" + sp + "%02X" % length + sp)
                    bytes_to_add = 6
                    answer += data[:(3 if sp else 2) * bytes_to_add] + nl
                    remaining_length = length - bytes_to_add
                    remaining_data = data[(3 if sp else 2) * bytes_to_add:]
                    bytes_to_add += 1
                    frame_count = 1
                    while remaining_length > 0:
                        if frame_count == 0x10:
                            frame_count = 0
                        answer += (answer_header
                                   + sp + "%02X" % (frame_count + 0x20) + sp)
                        remaining_length -= bytes_to_add
                        answer += (remaining_data[
                                   :(3 if sp else 2) * bytes_to_add] + nl)
                        remaining_data = remaining_data[
                            (3 if sp else 2) * bytes_to_add:]
                        frame_count += 1
                    answer = answer.rstrip(sp + nl)
                else:
                    answer = answer_header + sp + "%02X" % length + sp + data
            else:
                if length > 7:  # produce a multframe output
                    if ('cmd_caf' in self.counters and
                            not self.counters[
                                'cmd_caf']):  # PCI byte in requests
                        answer = "10" + sp + "%02X" % length + sp
                        pci = True
                    else:
                        pci = False
                        answer = "%03X" % length + nl + "0: "
                    bytes_to_add = 6
                    answer += data[:(3 if sp else 2) * bytes_to_add] + nl
                    remaining_length = length - bytes_to_add
                    remaining_data = data[(3 if sp else 2) * bytes_to_add:]
                    bytes_to_add += 1
                    frame_count = 1
                    while remaining_length > 0:
                        if frame_count == 0x10:
                            frame_count = 0
                        if pci:
                            answer += "%02X" % (frame_count + 0x20) + sp
                        else:
                            answer += "%01X" % frame_count + ": "
                        remaining_length -= bytes_to_add
                        answer += (remaining_data[
                                   :(3 if sp else 2) * bytes_to_add] + nl)
                        remaining_data = remaining_data[
                            (3 if sp else 2) * bytes_to_add:]
                        frame_count += 1
                    answer = answer.rstrip(sp + nl)
                else:
                    if ('cmd_caf' in self.counters and
                            not self.counters[
                                'cmd_caf']):  # PCI byte in requests
                        answer = "%02X" % length + sp + data
                    else:
                        answer = data
        elif len(request_header) == 6 and is_flow_control:  # KWP2000 FC
            logging.error(
                'KWP2000 format with flow control: unimplemented case.')
            return ""
        elif len(request_header) == 6:  # KWP2000 encoding including length and checksum
            if not use_headers:
                logging.error(
                    'KWP2000 format without headers: unimplemented case.')
                return ""
            if length < MIN_SIZE_UDS_LENGTH:
                answer = (("%02X" % (128 + length) + sp +
                           request_header[4:6] + sp +
                           request_header[2:4]) + sp + data)
            else:
                answer = ("80" + sp +  # Extra length byte follows
                          request_header[4:6] + sp +
                          request_header[2:4] + sp +
                          "%02X" % length + sp + data)
            try:  # calculate checksum
                answer += sp + "%02X" % (sum(bytearray.fromhex(answer)) % 256)
            except ValueError as e:
                logging.error("Error in generated answer %s from HEX data "
                              "%s with header %s: %s",
                              answer, repr(data), repr(request_header), e)
        else:
            logging.error('Invalid request header: %s', repr(request_header))
        return answer + sp + nl

    def handle_response(self,
                        resp,
                        do_write=False,
                        request_header=None,
                        request_data=None):
        """
        Compute the response and returns data written to the device.
        :param resp: XML response string to compute
        :param do_write: True if the computed response is written to the
                output device. False is for testing the computed response
                without writing data to the communication port.
        :param request_header: Header of the request
                (used to compute the response header)
        :param request_data: data of the request
                (used to compute the positive and negative response data)
        :return: computed response, or empty (no output) or None (error).
        """

        logging.debug("Processing: %s", repr(resp))

        # Compute cra_pattern (ATCRA filter)
        cra_pattern = r'[0-9A-F]+'
        cra = self.counters["cmd_cra"] if "cmd_cra" in self.counters else None
        if cra:
            cra_pattern = (r'^' + cra
                           .replace('X', '[0-9A-F]').replace('W', '[0-9A-F]+')
                           + r'$')

        # Compute use_headers
        use_headers = ("cmd_use_header" in self.counters and
                       self.counters["cmd_use_header"])

        # Compute sp (space)
        if ('cmd_spaces' in self.counters
                and self.counters['cmd_spaces'] == 0):
            sp = ''
        else:
            sp = ' '

        # Compute nl (newline)
        nl_type = {
            0: "\r",
            1: "\r\n",
            2: "\n",
            3: "\r",
            4: "\r\n",
            5: "\n"
        }
        nl = "\r"
        if 'cmd_linefeeds' in self.counters:
            try:
                nl = nl_type[int(self.counters['cmd_linefeeds'])]
            except Exception:
                logging.error(
                    'Invalid "cmd_linefeeds" value: %s.',
                    repr(self.counters['cmd_linefeeds']))

        # Generate string
        incomplete_resp = False
        root = None
        resp = resp.replace('\x00', '\\x00').replace('\x0d', '&#13;')
        try:
            root = fromstring('<xml>' + resp + '</xml>')
            s = iter(root)
        except ParseError as e:
            incomplete_resp = True
            logging.error(
                'Wrong response format for "%s"; %s', resp, e)
        answ = root.text.strip() if root is not None and root.text else ""
        answers = False
        i = None

        while not incomplete_resp:
            try:
                i = next(s)
            except StopIteration:
                answ += i.tail.strip() if i is not None and i.tail else ""
                break
            if i.tag.lower() == 'rh':
                request_header = (i.text or "")
            elif i.tag.lower() == 'rd':
                request_data = (i.text or "")
            elif i.tag.lower() == 'string':
                answ += (i.text or "")
            elif i.tag.lower() == 'writeln':
                answ += (i.text or "") + nl
            elif i.tag.lower() == 'space':
                answ += (i.text or "") + sp
            elif (i.tag.lower() == 'eval' or
                  i.tag.lower() == 'exec'):
                answ = answ.replace('\\x00', '\x00')
                logging.debug("Write: %s", repr(answ))
                if i.tag.lower() == 'exec' and do_write:
                    self.write_to_device(answ.encode())
                    answ = ""
                if i.text is None:
                    continue
                msg = i.text.strip()
                if msg:
                    try:
                        evalmsg = eval(msg)
                        logging.debug(
                            "Evaluated command: %s -> %s",
                            msg, repr(evalmsg))
                        if evalmsg != None:
                            answ += str(evalmsg)
                    except Exception:
                        try:
                            exec(msg, globals())
                            logging.debug("Executed command: %s", msg)
                        except Exception as e:
                            logging.error("Cannot execute '%s': %s", msg, e)
                else:
                    logging.debug(
                        "Missing command to execute: %s", resp)

            elif i.tag.lower() == 'flow':
                answ += self.uds_answer(data=i.text or "",
                                        request_header=request_header,
                                        use_headers=use_headers,
                                        cra_pattern=cra_pattern,
                                        sp=sp,
                                        nl=nl,
                                        is_flow_control='30')

            elif i.tag.lower() == 'answer':
                answ += self.uds_answer(data=i.text or "",
                                        request_header=request_header,
                                        use_headers=use_headers,
                                        cra_pattern=cra_pattern,
                                        sp=sp,
                                        nl=nl)
            elif i.tag.lower() == 'pos_answer' or i.tag.lower() == 'neg_answer':
                if not request_data:
                    logging.error(
                        'Missing request with <%s> tag: %s.',
                        i.tag.lower(), repr(resp))
                    break

                # Calculate uds_pos_answ for uds_pos_answer
                uds_pos_answ = ''
                try:
                    rd = ''.join(request_data.split())
                    for sid in uds_sid_pos_answer:
                        if rd.startswith(sid):
                            uds_pos_answ = (
                                sp.join(
                                    '{:02x}'.format(x) for x in
                                    bytearray.fromhex(
                                        rd[
                                            2:2 + 2 * uds_sid_pos_answer[sid]])
                                ).upper()
                            )
                            break
                except:
                    uds_pos_answ = None

                if i.tag.lower() == 'pos_answer' and uds_pos_answ is None:
                    logging.error(
                        'Invalid <%s> tag: %s.', i.tag.lower(), repr(resp))
                    break
                try:
                    request_data = (''.join('{:02x}'.format(x)
                                            for x in bytearray.fromhex(
                        request_data[:4])).upper())
                except ValueError as e:
                    logging.error('Invalid request %s related to response %s '
                                  'including <%s> tag: %s',
                                  repr(request_data), repr(resp),
                                  i.tag.lower(), e)
                    return ""
                if i.tag.lower() == 'pos_answer':
                    data = ("%02X" % (bytearray.fromhex(request_data[:2])[0]
                                      | 0x40) +
                            uds_pos_answ + (i.text or ""))
                else:  # Generate a negative response UDS SID
                    data = "7F" + sp + request_data[:2] + (i.text or "")
                answ += self.uds_answer(data=data,
                                        request_header=request_header,
                                        use_headers=use_headers,
                                        cra_pattern=cra_pattern,
                                        sp=sp,
                                        nl=nl)

            elif i.tag.lower() == 'header':
                answers = True
                incomplete_resp = True
                try:
                    size = next(s)
                    data = next(s)
                except StopIteration:
                    logging.error(
                        'Missing <size> or <data>/<subd> tags '
                        'after <header> tag in %s.', repr(resp))
                    break
                # check that the tags are valid
                if (size.tag.lower() != 'size' or
                        (data.tag.lower() != 'data' and
                         data.tag.lower() != 'subd')):
                    logging.error(
                        'In %s, <size> and <data>/<subd> tags '
                        'must follow the <header> tag.', repr(resp))
                    break

                # check validity of the content fields
                try:
                    int_size = int(size.text, 16)
                except ValueError as e:
                    logging.error(
                        'Improper size %s for response %s: %s.',
                        repr(size.text), repr(resp), e)
                    break
                if not data.text:
                    logging.error('Missing data for response %s.',
                                  repr(resp))
                    break
                unspaced_data = (data.text or "").translate(
                    (data.text or "").maketrans('', '', string.whitespace))
                if int_size < 16 and len(unspaced_data) != int_size * 2:
                    logging.error(
                        'In response %s, mismatch between number of data '
                        'digits %s and related length field %s.',
                        repr(resp), repr(data.text), repr(size.text))
                    break
                incomplete_resp = False
                if re.match(cra_pattern, i.text.upper()):
                    # concatenate answ from header, size and data/subd
                    answ += ((((i.text or "") + sp + (size.text or "") + sp)
                              if use_headers else "") +
                             ((data.text or "") if sp else unspaced_data) +
                             sp + (nl if data.tag.lower() == 'data' else ""))
                else:
                    logging.debug(
                        'Skipping answer which does not match ATCRA: '
                        'header=%s, cra_pattern=%s.',
                        repr(i.text), repr(cra_pattern))
            else:
                logging.error(
                    'Unknown tag "%s" in response "%s"', i.tag, resp)
            answ += i.tail.strip() if i is not None and i.tail else ""
        if incomplete_resp or (answers and not answ):
            answ = "NO DATA" + nl
        if not answ:
            logging.debug(
                'Null response received after processing "%s".', resp)
            return None
        if ('cmd_linefeeds' in self.counters and
                self.counters['cmd_linefeeds'] > 2):
            answ += ">"
        else:
            answ += nl + ">"
        answ = answ.replace('\\x00', '\x00')
        if do_write:
            logging.debug("Write: %s", repr(answ))
            self.write_to_device(answ.encode())
        return answ

    def task_action(
            self, header, ecu, do_write, task_method, cmd, length, frame,
            is_ecu=False):
        """
        Call a task method (start(), run(), or stop()), manage the exception,
         pre-process r_task and r_cont return values and return a tuple with
         all the three return values of the invoked method.

        :param header: header string
        :param ecu: ECU string
        :param do_write: boolean set to True if the output has to be produced
        :param task_method: pointer to the method to be executed
        :param length: length byte extracted by handle_request
        :param frame: frame number extracted by handle_request
        :param cmd: request string produced by the ISO-TP data link
        :param is_ecu: True if ECU Task
        :return: a tuple of three elements with the same return parameters
                as the Task methods
        """
        if not is_ecu:
            logging.debug(
                "Running task %s.%s(%s, %s, %s) for ECU %s",
                self.tasks[ecu][-1].__module__,
                task_method.__name__, cmd, length,
                frame, ecu)

        r_cmd = None
        r_task = Tasks.RETURN.TERMINATE
        r_cont = None
        try:  # Run the task method
            r_cmd, r_task, r_cont = task_method(cmd, length, frame)
        except Exception as e:
            if is_ecu:
                logging.critical(
                    'Error in ECU task "%s", ECU="%s", '
                    'method=run(): %s',
                    self.task_shared_ns[ecu].__module__,
                    ecu,
                    e, exc_info=True)
                del self.task_shared_ns[ecu]
            else:
                logging.critical(
                    'Error in task "%s", ECU="%s", '
                    'method=%s(): %s',
                    self.tasks[ecu][-1].__module__,
                    ecu,
                    task_method.__name__,
                    e, exc_info=True)
                del self.tasks[ecu][-1]
            return Tasks.RETURN.ERROR
        if not is_ecu:
            logging.debug(
                "r_cmd=%s, r_task=%s, r_cont=%s", r_cmd, r_task, r_cont)
        if r_cont is not None and r_cmd is not None:
            resp = self.handle_response(
                r_cmd,
                do_write=do_write,
                request_header=header,
                request_data=cmd if is_ecu else
                self.tasks[ecu][-1].task_get_request())
            if not do_write:
                logging.warning(
                    "%sTask for ECU %s returned %s",
                    "ECU " if is_ecu else "",
                    ecu, repr(resp))
        if r_task is Tasks.RETURN.TERMINATE:
            if is_ecu:
                logging.debug(
                    'Terminated ECU task "%s" for ECU "%s"',
                    self.task_shared_ns[ecu].__module__, ecu)
                del self.task_shared_ns[ecu]
            else:
                logging.debug(
                    'Terminated task "%s" for ECU "%s"',
                    self.tasks[ecu][-1].__module__, ecu)
                self.account_task(ecu)
                del self.tasks[ecu][-1]
        if r_cont is not None and not is_ecu:
            logging.debug(
                "Continue processing command %s after execution of task "
                "for ECU %s.", repr(r_cont), ecu)
        return r_cmd, r_task, r_cont

    def account_task(self, ecu):
        """
        Create and increase the task counter.
        :param ecu: ECU string
        :return: (none)
        """
        if ecu not in self.tasks:
            return
        try:
            task_name = self.tasks[ecu][-1].__module__[12:]
        except Exception:
            return
        if task_name not in self.counters:
            self.counters[task_name] = 0
        self.counters[task_name] += 1

    def handle_request(self, cmd, do_write=False):
        """
        Generate an XML response by processing a request,
        returning a string to be processed by process_response(),
        which creates the final output format. Return None
        if this there are no data to be processed.

        In some cases, process_response() is implicitly called,
        passing do_write. Task processing is included.

        :param cmd: the request to be processed
        :param do_write: passed to process_response() when
                        implicitly called, or used for logging.
        :return: (header, request, None or an XML string)
        """

        org_cmd = cmd
        # Sanitize cmd (request has all unspaced uppercase chars)
        cmd = (cmd or '').translate(
            (cmd or '').maketrans('', '', string.whitespace)).upper()

        # Increment 'commands' counter

        if 'commands' not in self.counters:
            self.counters['commands'] = 0
        self.counters['commands'] += 1

        # *****************************************

        # Handle custom PIDs for gear and gear position
        if cmd == "010C":  # Engine RPM
            rpm = self.car.rpm
            obd_rpm = int(rpm * 4)  # Convert RPM to OBD-II format
            obd_rpm_hex = f"{obd_rpm:04X}"  # Convert to 4-digit hex
            response = f"41 0C {obd_rpm_hex}"
            return header, cmd, response

        if cmd == "010D":  # Vehicle Speed
            speed = self.car.speed
            obd_speed = int(speed)  # Convert speed to OBD-II format
            obd_speed_hex = f"{obd_speed:02X}"  # Convert to 2-digit hex
            response = f"41 0D {obd_speed_hex}"
            return header, cmd, response

        if cmd == "0120":  # Custom PID for Gear Position
            gear_position = self.car.gear_position
            # Convert to ASCII value
            obd_gear_position = int(ord(gear_position[0]))
            # Convert to 2-digit hex
            obd_gear_position_hex = f"{obd_gear_position:02X}"
            response = f"41 20 {obd_gear_position_hex}"
            return header, cmd, response

        if cmd == "0121":  # Custom PID for Gear
            gear = self.car.gear
            obd_gear = int(gear)  # Convert gear to OBD-II format
            obd_gear_hex = f"{obd_gear:02X}"  # Convert to 2-digit hex
            response = f"41 21 {obd_gear_hex}"
            return header, cmd, response

        # cmd_can is experimental (to be removed)
        if ('cmd_can' in self.counters and
                self.counters['cmd_can']
                and is_hex_sp(cmd[:3])):
            self.counters['cmd_set_header'] = cmd[:3]
            header = cmd[:3]
            self.counters['cmd_caf'] = False
            self.counters['cmd_use_header'] = True
            cmd = cmd[3:]

        # Set header and ecu
        header = None
        ecu = None
        if "cmd_set_header" in self.counters:
            header = self.counters['cmd_set_header']
            if len(header) == 6:
                ecu = header[2:]
            else:
                ecu = header

        # Manage the UDS P2 delay timer
        logging.debug("Handling: %s, header %s, ECU %s",
                      repr(cmd), repr(header), repr(ecu))
        if self.delay > 0:
            time.sleep(self.delay)

        if len(org_cmd) > 1 and cmd[1] == 'T' and org_cmd.upper()[1] != 'T':
            # AT or ST shall be unspaced
            logging.error("Improper AT or ST command %s.", repr(org_cmd))
            return header, cmd, ""

        if self.scenario not in self.ObdMessage:
            logging.error("Unknown scenario %s", repr(self.scenario))
            return header, cmd, ""
        # Handle PID 0105 (Engine Coolant Temperature)
        if cmd == "0105":
            # Get the current engine temperature from the database
            # Default to 70°C if not set
            temp = self.database.get("engine_temp", 70)

        # Convert temperature to OBD-II format (offset by 40°C)
            obd_temp = int(temp) - 40
            obd_temp_hex = f"{obd_temp:02X}"  # Convert to 2-digit hex

        # Format the OBD-II response (41 05 <temperature>)
            response = f"41 05 {obd_temp_hex}"
            logging.debug("PID 0105 Response: %s", response)
            return header, cmd, response
        # Handle fuel_consumpustion PID
        if cmd == "012F":
            fuel_level_percent = int(
                (self.car.database["fuel_level"] / self.car.FUEL_TANK_CAPACITY) * 100)
            # Format: 41 <PID> <value>
            response = f"41 2F {fuel_level_percent:02X}"
            return header, cmd, response
        elif cmd == "015E":  # PID 0x5E - Engine Fuel Rate
            # Convert L/s to L/h
            fuel_rate_lph = int(
                self.car.database["fuel_consumption_rate"] * 3600)
            response = f"41 5E {fuel_rate_lph:04X}"  # Format: 41 <PID> <value>
            return header, cmd, response

        #  Manage ECU task and shared namespace
        if ecu in self.task_shared_ns:  # ECU task exists with its namespace
            r_cmd, *_, r_cont = self.task_action(header, ecu, do_write,
                                                 self.task_shared_ns[ecu].run,
                                                 cmd, None, None,
                                                 is_ecu=True)
            if r_cont is None:
                return header, cmd, r_cmd
            else:
                cmd = r_cont
        else:  # create the ECU task and shared namespace for the ECU
            plugin = None
            for i in self.plugins:
                i_pattern = (r'^' +
                             i.upper()
                             .replace('X', '[0-9A-F]')
                             .replace('W', '[0-9A-F]+') +
                             r'$')
                if (i.startswith(ECU_TASK) and
                        re.match(i_pattern, ECU_TASK.upper() + ecu)):
                    plugin = i
                    break
            try:  # use the plugin if existing, else directly use EcuTasks()
                if plugin:
                    self.task_shared_ns[ecu] = self.plugins[plugin].Task(
                        emulator=self, pid=None, header=header, ecu=ecu,
                        request=cmd, attrib=None, do_write=do_write)
                else:  # Create a default ECU task
                    self.task_shared_ns[ecu] = EcuTasks(
                        emulator=self, pid=None, header=header, ecu=ecu,
                        request=cmd, attrib=None, do_write=do_write)
                    self.task_shared_ns[ecu].__module__ = DEFAULT_ECU_TASK
            except Exception as e:
                logging.critical(
                    'Cannot instantiate ECU task "%s", ECU="%s": %s',
                    ECU_TASK + ecu, ecu, e, exc_info=True)
                return header, cmd, None
            logging.debug('Instantiating ECU task "%s" for ECU "%s"',
                          self.task_shared_ns[ecu].__module__, ecu)
            r_cmd, *_, r_cont = self.task_action(header, ecu, do_write,
                                                 self.task_shared_ns[ecu].start,
                                                 cmd, None, None,
                                                 is_ecu=True)
            if r_cont is None:
                return header, cmd, r_cmd
            else:
                cmd = r_cont
        self.shared = None
        if ecu and ecu in self.task_shared_ns:
            self.shared = self.task_shared_ns[ecu]

        # Manage cmd_caf, length, frame & process UDS ISO-TP Multiframe data link
        size = cmd[:2]
        length = None  # No byte length in request
        frame = None  # Single Frame by default
        if ecu and is_hex_sp(cmd):  # Not AT or ST command
            if (ecu in self.request_timer and
                    self.request_timer[ecu] + self.multiframe_timer <
                    time.time()):
                if ecu in self.tasks and len(self.tasks[ecu]):
                    logging.warning(
                        "UDS P3 timer expired, removing active tasks.")
                    for i in reversed(self.tasks[ecu]):
                        self.task_action(
                            header, ecu, do_write, i.stop, cmd, length, frame,
                            is_ecu=False)
                    del self.tasks[ecu]
                if ecu in self.task_shared_ns:
                    logging.debug(
                        'UDS P3 timer expired: running stop() method for '
                        'ECU task "%s", ECU="%s".',
                        self.task_shared_ns[ecu].__module__,
                        ecu)
                    try:  # Run the stop() method
                        r_cmd, r_task, r_cont = self.task_shared_ns[ecu].stop(
                            None)
                        if (r_task is Tasks.RETURN.TERMINATE and
                                r_cont == 'DELETE'):
                            del self.task_shared_ns[ecu]
                    except Exception as e:
                        logging.critical(
                            'Error while running stop() method for ECU '
                            'task "%s", ECU="%s": %s',
                            self.task_shared_ns[ecu].__module__,
                            ecu,
                            e, exc_info=True)
            self.request_timer[ecu] = time.time()
        if ('cmd_caf' in self.counters and
                not self.counters['cmd_caf'] and  # PCI byte in requests
                is_hex_sp(cmd)):  # not AT or ST command
            try:
                int_size = int(size, 16)
            except ValueError as e:
                logging.error('Improper size %s for request %s: %s',
                              repr(size), repr(org_cmd), e)
                return header, cmd, ""
            payload = cmd[2:]
            if not payload:
                logging.error('Missing data for request %s', repr(org_cmd))
                return header, cmd, ""
            if size[0] == '0':  # Single-Frame
                if int_size < 8:  # valid value
                    if len(payload) < int_size * 2:
                        logging.error(
                            'In request %s, data %s has an improper length '
                            'of %s bytes', repr(org_cmd), repr(payload), size)
                        return header, cmd, ""
                    cmd = payload[:int_size * 2]
                    length = int_size
                    logging.debug(
                        "Single-Frame. Length: %s, frame: %s, header: %s, "
                        "cmd: %s", length, frame, header, cmd)
                else:
                    logging.error('Invalid ISO-TP Single frame with size '
                                  'greater than 7 bytes. %s',
                                  repr(size))
                    return header, cmd, ""
            elif size[0] == '1':  # E.g., 10 = first frame of an ISO-TP Multiframe Request
                try:
                    length = int(cmd[1:4], 16)  # read ISO-TP Multiframe length
                    logging.debug(
                        'ISO-TP Multiframe message with length 0x%s = (int) %s '
                        '(message %s)', repr(cmd[1:4]), length, repr(cmd))
                    logging.debug(
                        "First-Frame. Length: %s, frame: %s, header: %s, "
                        "cmd: %s", length, frame, header, cmd)
                except ValueError as e:
                    logging.error('Improper size %s for request %s: %s',
                                  repr(cmd[2:4]), repr(org_cmd), e)
                    return header, cmd, ""
                cmd = cmd[4:]
                frame = 0
            elif size[0] == '2':  # E.g., 21 = ISO-TP Consecutive Frame
                cmd = cmd[2:]
                if int_size == 32:  # The frame includes 20 after 1F
                    frame = -1  # Marker for ISO-TP Multiframe() to detect a recycle
                if 32 < int_size < 48:  # from 21 to 2F
                    frame = int_size - 32  # compute the multiframe count
                logging.debug(
                    "Consecutive-Frame. Length: %s, frame: %s, header: %s, "
                    "cmd: %s", length, frame, header, cmd)
            elif size[0] == '3':  # E.g., from 30 on = flow control of a ISO-TP Multiframe Request
                try:
                    self.shared.flow_control_fc_flag = int(size[1])
                    self.shared.flow_control_block_size = int(cmd[2:4], 16)
                    self.shared.flow_control_separation_time = int(
                        cmd[4:6], 16)
                except Exception as e:
                    logging.error(
                        'Improper Flow-control-Frame %s: %s', repr(org_cmd), e)
                    return header, cmd, ""
                if self.shared.flow_control_fc_flag == 0:  # Clear To Send
                    if self.shared.flow_control_block_size > 0:
                        logging.debug(
                            'Flow-control-Frame. Input is ignored by now: '
                            'int_size: %s, length: %s, frame: %s, header: %s, '
                            'cmd: %s, fc_flag: %s, block_size: %s, '
                            'separation_time: %s.',
                            int_size, length, frame, header, cmd,
                            self.shared.flow_control_fc_flag,
                            self.shared.flow_control_block_size,
                            self.shared.flow_control_separation_time)
                elif self.shared.flow_control_fc_flag == 1:  # Wait
                    sleep = self.shared.flow_control_separation_time
                    if sleep > 240:
                        sleep = ((self.shared.flow_control_separation_time -
                                  240) * 100)
                    logging.debug(
                        'Sleeping for %s milliseconds', sleep)
                    time.sleep(sleep / 1000)
                elif self.shared.flow_control_fc_flag == 2:  # 2 = Overflow/abort
                    logging.error('Overflow-abort received in ISO-TP '
                                  'Flow-control-Frame. %s', repr(org_cmd))
                    return header, cmd, ""
                else:
                    logging.error(
                        'Improper Flow-control-Frame FC flag %s. %s: %s',
                        self.shared.flow_control_fc_flag, repr(org_cmd))
                    return header, cmd, ""
                return header, cmd, None
            else:
                logging.error('Invalid ISO-TP type %s for frame %s.',
                              repr(size[0]), repr(org_cmd))
                return header, cmd, ""

        # Manage ISO-TP Multiframe
        if length is not None and frame is not None:  # ISO-TP Multiframe condition
            if ecu not in self.tasks:
                self.tasks[ecu] = []
            if len(self.tasks[ecu]) > MAX_TASKS:
                logging.critical(
                    'Too many active tasks for ECU %s while adding '
                    'a ISO-TP Multiframe frame. Latest task was %s.',
                    ecu, self.tasks[ecu][-1].__module__)
                return header, cmd, ""
            if (len(self.tasks[ecu]) and
                    self.tasks[ecu][-1].__module__ == ISO_TP_MULTIFRAME_MODULE):
                logging.error(
                    'Improper frame within ISO-TP ISO-TP Multiframe. ECU: %s, '
                    'data length: %s, frame: %s, data: %s',
                    ecu, length, frame, cmd)
                return header, cmd, ""
            self.tasks[ecu].append(
                IsoTpMultiframe(
                    self, "ISO-TP-Multiframe", header, ecu, cmd, None, do_write)
            )
            self.tasks[ecu][-1].__module__ = ISO_TP_MULTIFRAME_MODULE

        # Manage active tasks
        if ecu in self.tasks and self.tasks[ecu]:  # if a task exists
            if len_hex(cmd):
                r_cmd, *_, r_cont = self.task_action(header, ecu, do_write,
                                                     self.tasks[ecu][-1].run,
                                                     cmd, length, frame,
                                                     is_ecu=False)
                if r_cont is None:
                    return header, cmd, r_cmd
                else:
                    cmd = r_cont
                    frame = None
                    length = None
            else:  # AT or ST command
                frame = None
                if INTERRUPT_TASK_IF_NOT_HEX:
                    logging.warning('Interrupted task "%s" for ECU "%s"',
                                    self.tasks[ecu][-1].__module__, ecu)
                    r_cmd, *_, r_cont = self.task_action(header, ecu, do_write,
                                                         self.tasks[ecu][
                                                             -1].stop, cmd,
                                                         length, frame,
                                                         is_ecu=False)
                    if ecu in self.tasks and self.tasks[ecu]:
                        del self.tasks[ecu][-1]
                    if r_cont is None:
                        return header, cmd, r_cmd
                    else:
                        cmd = r_cont
                else:
                    logging.debug(
                        'Non-hex request "%s" will not be passed to active '
                        'task "%s" for ECU "%s".',
                        cmd, self.tasks[ecu][-1].__module__, ecu)

        if frame is not None:
            logging.error("Invalid multiframe %s", repr(org_cmd))
            return header, cmd, ""

        # Process response for data stored in cmd
        i_obd_msg = iter(self.sortedOBDMsg)
        chained_command = 0
        while True:
            try:
                key, val = next(i_obd_msg)
            except StopIteration:
                break
            uc_val = {k.upper(): v for k, v in val.items()}
            if ('REQUEST' in uc_val and
                    re.match(uc_val['REQUEST'], cmd)):
                if ('HEADER' in uc_val and header and
                        uc_val['HEADER'].upper() !=
                        self.counters["cmd_set_header"]):
                    continue
                pid = key if key else 'UNKNOWN'
                self.counters["cmd_last_pid"] = pid
                if pid not in self.counters:
                    self.counters[pid] = 0
                self.counters[pid] += 1
                # Handle PID 0105 (Engine Coolant Temperature)
                if pid == "0105":
                    # Get the current engine temperature from the database
                    # Default to 70°C if not set
                    temp = self.database.get("engine_temp", 70)

                    # Convert temperature to OBD-II format (offset by 70°C)
                    obd_temp = int(temp) - 70
                    obd_temp_hex = f"{obd_temp:02X}"  # Convert to 2-digit hex

                    # Format the OBD-II response (41 05 <temperature>)
                    response = f"41 05 {obd_temp_hex}"
                    return header, cmd, response

                if 'ACTION' in uc_val and uc_val['ACTION'] == 'skip':
                    logging.info("Received %s. PID %s. Action=%s", cmd, pid,
                                 uc_val['ACTION'])
                    continue
                if 'DESCR' in uc_val:
                    logging.debug("Description: %s, PID %s (%s)",
                                  uc_val['DESCR'], pid, cmd)
                else:
                    logging.warning(
                        "Internal error - Missing description for %s, PID %s",
                        cmd, pid)
                if pid in self.answer:
                    try:
                        return header, cmd, self.answer[pid]
                    except Exception as e:
                        logging.error(
                            "Error while processing '%s' for PID %s (%s)",
                            self.answer, pid, e)
                if 'TASK' in uc_val:
                    if uc_val['TASK'] not in self.plugins:
                        logging.error(
                            'Unexisting plugin %s for pid %s',
                            repr(uc_val['TASK']), repr(pid))
                        return header, cmd, None
                    if uc_val['TASK'].startswith('task_ecu_'):
                        logging.error(
                            'ECU Tasks are not expected to be run by '
                            'standard requests. Plugin %s, pid %s',
                            repr(uc_val['TASK']), repr(pid))
                        return header, cmd, None
                    if ecu not in self.tasks:
                        self.tasks[ecu] = []
                    if len(self.tasks[ecu]) > MAX_TASKS:
                        logging.critical(
                            'Too many active tasks for ECU %s. '
                            'Latest one was %s.',
                            ecu, self.tasks[ecu][-1].__module__)
                        return header, cmd, ""
                    try:
                        self.tasks[ecu].append(
                            self.plugins[uc_val['TASK']].Task(
                                emulator=self, pid=pid, header=header, ecu=ecu,
                                request=cmd, attrib=uc_val, do_write=do_write)
                        )
                    except Exception as e:
                        logging.critical(
                            'Cannot add task "%s", ECU="%s": %s',
                            uc_val['TASK'], ecu, e, exc_info=True)
                        return header, cmd, None
                    logging.debug('Starting task "%s" for ECU "%s"',
                                  self.tasks[ecu][-1].__module__, ecu)
                    r_cmd, *_, r_cont = self.task_action(
                        header, ecu, do_write,
                        self.tasks[ecu][-1].start, cmd, length, frame,
                        is_ecu=False)
                    if r_cont is None:
                        return header, cmd, r_cmd
                    else:  # chain a subsequent command
                        if cmd == r_cont:  # no transformation performed
                            logging.debug(
                                'Passthrough task executed: '
                                'continue processing %s for ECU %s.',
                                cmd, ecu)
                        else:  # newly reprocess the changed request
                            chained_command += 1
                            if chained_command > MAX_TASKS:
                                logging.critical(
                                    'Too many subsequent chained commands '
                                    'for ECU %s. Latest task was %s.',
                                    ecu, uc_val['TASK'])
                                return header, cmd, ""
                            cmd = r_cont
                            i_obd_msg = iter(self.sortedOBDMsg)
                            continue  # restart the loop from the beginning
                if 'EXEC' in uc_val:
                    try:
                        exec(uc_val['EXEC'])
                    except Exception as e:
                        logging.error(
                            "Cannot execute '%s' for PID %s (%s)",
                            uc_val['EXEC'], pid, e, exc_info=True)
                log_string = ""
                if 'INFO' in uc_val:
                    log_string = "logging.info(%s)" % uc_val['INFO']
                if 'WARNING' in uc_val:
                    log_string = "logging.warning(%s)" % uc_val['WARNING']
                if 'LOG' in uc_val:
                    log_string = "logging.debug(%s)" % uc_val['LOG']
                if log_string:
                    try:
                        exec(log_string)
                    except Exception as e:
                        logging.error(
                            "Error while logging '%s' for PID %s (%s)",
                            log_string, pid, e, exc_info=True)
                if any(x in uc_val for x in
                       ['RESPONSE', 'RESPONSEHEADER', 'RESPONSEFOOTER']):
                    r_header = ''
                    if 'RESPONSEHEADER' in uc_val:
                        try:
                            r_header = uc_val['RESPONSEHEADER'](
                                self, cmd, pid, uc_val)
                        except Exception as e:
                            logging.error(
                                "Error while running 'ResponseHeader' %s '"
                                "for PID %s (%s)",
                                uc_val['RESPONSEHEADER'], pid, e, exc_info=True)
                    r_footer = ''
                    if 'RESPONSEFOOTER' in uc_val:
                        try:
                            r_footer = uc_val['RESPONSEFOOTER'](
                                self, cmd, pid, uc_val)
                        except Exception as e:
                            logging.error(
                                "Error while running 'ResponseFooter' %s '"
                                "for PID %s (%s)",
                                uc_val['RESPONSEHEADER'], pid, e, exc_info=True)
                    r_response = ''
                    if 'RESPONSE' in uc_val:
                        r_response = uc_val['RESPONSE']
                    if not any([r_response, r_header, r_footer]):
                        return header, cmd, None
                    if isinstance(r_response, (list, tuple)):
                        r_response = self.choice(r_response)
                    return header, cmd, r_header + r_response + r_footer
                else:
                    logging.error(
                        "Internal error - Missing response for %s, PID %s",
                        cmd, pid)
                    return header, cmd, None
        # Here cmd is unknown
        if "unknown_" + repr(cmd) not in self.counters:
            self.counters["unknown_" + repr(cmd)] = 0
        self.counters["unknown_" + repr(cmd)] += 1
        if cmd == '':
            logging.info("No ELM command")
            return header, cmd, ""
        fw_data = self.send_receive_forward((cmd + '\r').encode())
        if fw_data is not False:
            self.counters["unknown_" + repr(cmd) + "_R"] = repr(fw_data)
        if (fw_data is not False and
                re.match(r"^NO DATA *\r", fw_data or "") is None and
                re.match(r"^\? *\r", fw_data or "") is None and
                self.counters["unknown_" + repr(cmd)] == 1):
            logging.warning(
                'Missing data in dictionary: %s. Answer:\n%s',
                repr(cmd), repr(fw_data))
        if len_hex(cmd):
            if header:
                logging.info("Unknown request: %s, header=%s",
                             repr(cmd), self.counters["cmd_set_header"])
            else:
                logging.info("Unknown request: %s", repr(cmd))
            return header, cmd, ST('NO DATA')
        if header:
            logging.info("Unknown ELM command: %s, header=%s",
                         repr(cmd), self.counters["cmd_set_header"])
        else:
            logging.info("Unknown ELM command: %s", repr(cmd))
        return header, cmd, self.ELM_R_UNKNOWN

    def set_sorted_obd_msg(self):
        """Sort OBD messages by request pattern length."""
        if self.scenario not in self.ObdMessage:
            self.sortedOBDMsg = []
            return
        # Sort by length of REQUEST pattern (longest first)
        self.sortedOBDMsg = sorted(
            self.ObdMessage[self.scenario].items(),
            key=lambda x: len(x[1].get('REQUEST', '')),
            reverse=True
        )

    def reset(self, mode=0):
        """Reset the emulator state.
        Args:
            mode: Reset mode (0=normal, 1=warm start)
        """
        self.set_defaults()
        if mode == 0:  # Cold start
            self.counters = {
                'commands': 0,
                'cmd_echo': 1,  # Echo on by default
                'cmd_linefeed': 0,
                'cmd_spaces': 1,  # Spaces on by default
                'cmd_header': 0,
                'cmd_can': 0,
                'cmd_caf': 1,
                'cmd_memory': 0,
                'cmd_adaptive_timing': 1,
                'cmd_protocol': 0,
                'cmd_voltage': 12.0,
                'cmd_wakeup_msg': 0,
                'cmd_wakeup_interval': 0,
                'cmd_low_power': 0,
                'cmd_timeout': 0,
                'cmd_responses': 0,
                'cmd_elm_version': self.version,
                'cmd_device_description': "OBDII to RS232 Interpreter",
                'cmd_id': "ELM327 v1.5",
                'cmd_identifier': "ELM327 v1.5",
                'cmd_prompt': ">",
                'cmd_protocol_number': "6",  # ISO 15765-4 CAN (11 bit ID, 500 kbaud)
                'cmd_voltage_level': "12.0V",
                'cmd_device_identifier': "ELM327 v1.5",
                'cmd_programmable_parameters': "FF FF FF FF FF FF",
                'cmd_read_voltage': "12.0V",
                'cmd_ignition': 1,
                'cmd_memory_size': "100",
                'cmd_buffer_dump': "00 00 00 00 00 00 00 00",
                'cmd_status': "OK",
                'cmd_can_status': "OK",
                'cmd_can_show_status': 0,
                'cmd_can_extended_addressing': 0,
                'cmd_can_priority': 0,
                'cmd_can_flow_control': 0,
                'cmd_can_auto_formatting': 1,
                'cmd_can_variable_dls': 0,
                'cmd_can_flow_control_clear': 0,
                'cmd_can_flow_control_id': "00000000",
                'cmd_can_flow_control_data': "00 00 00",
                'cmd_can_flow_control_mode': 0,
                'cmd_can_flow_control_interval': 0,
                'cmd_can_flow_control_idle_time': 0,
                'cmd_can_flow_control_separation_time': 0,
                'cmd_can_flow_control_block_size': 0,
                'cmd_can_flow_control_timeout': 0,
                'cmd_can_flow_control_wakeup': 0,
                'cmd_can_flow_control_filter': 0,
                'cmd_can_flow_control_mask': 0,
                'cmd_can_flow_control_pattern': 0,
                'cmd_can_flow_control_range': 0,
                'cmd_can_flow_control_range_low': 0,
                'cmd_can_flow_control_range_high': 0,
            }
        else:  # Warm start
            self.counters['cmd_echo'] = 1
            self.counters['cmd_spaces'] = 1
            self.counters['cmd_linefeed'] = 0
            self.counters['cmd_header'] = 0
