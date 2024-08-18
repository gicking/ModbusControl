#!/usr/bin/python3
# -*- coding: utf-8 -*-
# pylint: disable=too-many-arguments, line-too-long
"""A Python base class for remote control of Modbus RTU clients via ModbusControl high-level protocol.

COM port driver need to be installed and device has to show up as virtual COM port (VCP).
Works transparently on Windows and Posix (Linux, Mac OS X).

For Modbus low-level protocol see https://en.wikipedia.org/wiki/Modbus

Custom ModbusControl high-level protocol, on top of Modbus RTU low-level protocol:
    - All registers are 16-bit (Modbus standard)
    - Only functions READ_INPUT_REGISTERS, WRITE_MULTIPLE_HOLDING_REGISTERS, and READ_HOLDING_REGISTERS used
    - Any other Modbus function codes return error code ILLEGAL_FUNCTION

1) Read static values or time-critical reads with low communication overhead:
    - Use Modbus command READ_INPUT_REGISTERS
    - Direct read of input registers via target address
    - No parameters other than address can be passed
    - Any valid address within [0,(MODBUS_NUM_INPUT_REG-1)] is allowed
    - Address 0 is reserved for ModbusControl protocol version MODBUSCONTROL_PROTOCOL. Is checked on opening port
    - Measured execution time for read command is ~4.0ms (Debian Linux 18.04 and USB2.0)

2) Trigger command execution on client with optional parameters:
    - Use combination of Modbus commands WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS
    - Write to and read from holding register holdReg[] must start at address 0
    - Command code is written to holdReg[0], parameters to holdReg[1...(N-1)]
    - A pending command is indicated via holdReg[0] bit 15 is set by master
    - After command completion, holdReg[0] bit 15 is cleared by slave
    - After successful command execution, return values are stored in holdReg[1...(N-1)]
    - Client indicates an error by setting holdReg[0] bit 14. In this case the error code is stored in holdReg[1]
    - Measured execution time for complete command (=write+read) is ~8.1ms (Debian Linux 18.04 and USB2.0)


Index and search
-------------------

* :ref:`genindex`
* :ref:`search`


Version
-------

- 2021-06-20 (v1.0) initial release of ModbusControl v1.0


Links
-----

- `Modbus low-level protocol: <https://en.wikipedia.org/wiki/Modbus>`_

.. note:: required modules need to be in $PYTHONPATH


Class
-----
"""
# pylint: enable=line-too-long


###########
# IMPORT REQUIRED MODULES
###########
import inspect
import serial.tools.list_ports
import modbus_tk.modbus as modbus
import modbus_tk.modbus_rtu as modbus_rtu
import modbus_tk.defines as modbus_defines
from modbus_tk.exceptions import ModbusInvalidResponseError
import time
from enum import Enum
import logging
logger = logging.getLogger(__name__)


########################################################
# MODBUSCONTROL CONSTANTS
########################################################

#######################
# Misc constants
#######################
MODBUSCONTROL_PROTOCOL = "1.0"       # ModbusControl protocol version (must match client)
MODBUSCONTROL_ADDR_PROTOCOL = 0         # Modbus input register address: ModbusControl protocol version


#######################
# Modbus low-level return codes
#######################
class ModbusStatus(Enum):
    OK = 0
    ILLEGAL_FUNCTION = 1
    ILLEGAL_DATA_ADDRESS = 2
    ILLEGAL_DATA_VALUE = 3
    SLAVE_DEVICE_FAILURE = 4
    ACKNOWLEDGE = 5
    SLAVE_DEVICE_BUSY = 6
    NEGATIVE_ACKNOWLEDGE = 7
    MEMORY_PARITY_ERROR = 8
    GATEWAY_PATH_UNAVAILABLE = 9
    GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 10
    UNKNOWN = 255


#######################
# ModbusControl high-level return codes
#######################
class ModbusControlStatus(Enum):
    ILLEGAL_CMD = -1        # command not supported
    ILLEGAL_PARAM = -2      # illegal parameter value / range


#######################
# ModbusControl Exceptions
#######################
class ModbusControlError(Exception):
    pass


########################################################
# ModbusControl base class for master
########################################################
class BaseClient:
    """
    Base class for controlling a Modbus RTU client via ModbusControl high-level protocol.

    Parameters
    ----------
    port : string
        Name of COM port (string). On error lists available ports
    baud : int
        Communication speed [Baud]. Must be supported by driver
    timeout_modbus : float
        Default Modbus RTU receive timeout [s]
    timeout_command : float
        Default command execution timeout [s]

    Returns
    -------
        ModbusControl object with opened port

    Examples
    --------
    >>> device = BaseClient(port="COM6", baud=115200)
    """

    #########
    # constructor
    #########
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, timeout_modbus: float = 0.1,
                 timeout_command: float = 0.2):
        """
        Create an object to control a Modbus RTU client via ModbusControl high-level protocol.

        Parameters
        ----------
        port : string
            Name of COM port (string). On error lists available ports
        baud : int
            Communication speed [Baud]. Must be supported by driver
        timeout_modbus : float
            Default Modbus RTU receive timeout [s]
        timeout_command : float
            Default command execution timeout [s]

        Returns
        -------
            ModbusControl object with opened port

        Examples
        --------
        >>> device = BaseClient(port="COM6", baud=115200)
        """

        # try opening serial port for Modbus RTU
        try:

            # store parameters
            self._port = port
            self._baud = baud
            self._timeout_modbus = timeout_modbus
            self._timeout_command = timeout_command

            # open port to Modbus client
            self.client = modbus_rtu.RtuMaster(serial.Serial(port=port, baudrate=baud,
                                               bytesize=8, parity='N', stopbits=float(1), xonxoff=False))

            # set Modbus receive timeout [s]
            self.client.set_timeout(timeout_modbus)

            # enable Modbus RTU logging
            self.client.set_verbose(True)

        # opening COM port failed -> list ports and raise exception
        except BaseException as _err:
            _msg = "Opening port '%s' failed with %s\n" % (port, str(_err))
            _msg += "Available ports are: "
            _ports = serial.tools.list_ports.comports()
            for _port in _ports:
                _msg += "  %s" % _port.device
            raise ModbusControlError(_msg)

        # return Modbus object with COM port configured and protocol versions checked
        return


    #########
    # Read an input register from ModbusControl client (don't modify)
    #########
    def read_values(self, slave: int = 1, address: int = None, num_out: int = None, 
                    timeout_modbus: float = None, timeout_command: float = None) -> dict:
        """Read the content of a Modbus RTU client input register via READ_INPUT_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        address : int
            index in the client input register
        num_out : int
            number of values to read
        timeout_modbus : float
           Modbus RTU receive timeout [s]
        timeout_command : float
           command execution timeout [s]

        Returns
        -------
        dict
            "values": list of returned values (int16)
        """

        # Set Modbus receive timeout [s]. Required for long command execution times
        if timeout_modbus is not None:
            self.client.set_timeout(timeout_modbus)

        # Set command timeout and start timeout. Is required for long command execution times
        _timeout_command = timeout_command if timeout_command is not None else self._timeout_command
        _start_time = time.time()

        # Retry loop for READ_INPUT_REGISTERS with timeout
        while True:

            try:
                # read input register
                _reg = self.client.execute(slave=slave, function_code=modbus_defines.READ_INPUT_REGISTERS,
                                           starting_address=address, quantity_of_x=num_out)
                logger.info("slave %d: read %dB from inputReg address %d -> %s" % (slave, num_out, address, str(_reg)))

                # Restore Modbus timeout
                if timeout_modbus is not None:
                    self.client.set_timeout(self._timeout_modbus)

                # return result
                return {"values": _reg}

            # check for acknowledge timeout
            # noinspection PyUnresolvedReferences
            except (modbus.ModbusError, ModbusInvalidResponseError, BaseException) as _err:
                if type(_err) == modbus.ModbusError:
                    _err = ModbusStatus(_err.get_exception_code()).name
                if time.time() - _start_time < _timeout_command:
                    logger.warning("%s(): Read failed (%s), retry" % 
                                   (inspect.stack()[0].function, str(_err)))
                    time.sleep(0.05)  # Short delay before retrying
                else:
                    _msg = "%s(): Read timeout after %1.1fs (%s), abort" % \
                        (inspect.stack()[0].function, (time.time()-_start_time), str(_err))
                    logger.error(_msg)
                    if timeout_modbus is not None:
                        self.client.set_timeout(self._timeout_modbus)            
                    raise ModbusControlError(_msg)


    #########
    # Trigger a command on ModbusControl client (don't modify)
    #########
    def execute_command(self, slave: int = 1, command: int = None, param_in: list = None,
                        num_out: int = None, timeout_modbus: float = None, timeout_command: float = None):
        """Trigger a command on the Modbus RTU client by writing via WRITE_MULTIPLE_HOLDING_REGISTERS
           and reading results via READ_HOLDING_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        command : int
            application command code
        param_in : list(int16)
            input parameters
        num_out : int
            number of return values
        timeout_modbus : float
           Modbus RTU receive timeout [s]
        timeout_command : float
           command execution timeout [s]

        Returns
        -------
        dict
            "values": list of returned parameters (int16)
        """

        # Set Modbus receive timeout [s]. Required for long command execution times
        if timeout_modbus is not None:
            self.client.set_timeout(timeout_modbus)

        # Set command timeout and start timeout. Is required for long command execution times
        _timeout_command = timeout_command if timeout_command is not None else self._timeout_command
        _start_time = time.time()

        # Retry loop for WRITE_MULTIPLE_REGISTERS with timeout
        while True:

            try:
                # Write command (in reg[0]) with parameters (in reg[1..N])
                self.client.execute(slave=slave, function_code=modbus_defines.WRITE_MULTIPLE_REGISTERS,
                                    starting_address=0, output_value=[command] + param_in)
                logger.info("slave %d: write %s to holdReg address 0" % (slave, str([hex(command)] + param_in)))
                
                # exit loop if write was successful (=no exception)
                break

            # check for acknowledge timeout
            except (modbus.ModbusError, ModbusInvalidResponseError, BaseException) as _err:
                if type(_err) == modbus.ModbusError:
                    _err = ModbusStatus(_err.get_exception_code()).name
                if time.time() - _start_time < _timeout_command:
                    logger.warning("%s(): Command request failed (%s), retry" % 
                                   (inspect.stack()[0].function, str(_err)))
                    time.sleep(0.05)  # Short delay before retrying
                else:
                    _msg = "%s(): Command request timeout after %1.1fs (%s), abort" % \
                        (inspect.stack()[0].function, (time.time()-_start_time), str(_err))
                    logger.error(_msg)
                    if timeout_modbus is not None:
                        self.client.set_timeout(self._timeout_modbus)            
                    raise ModbusControlError(_msg)


        # Read until command is completed (reg[0] bit15 = 0) or timeout
        _return = None
        _reg = None
        while True:

            try:
                # Read holding registers
                _reg = self.client.execute(slave=slave, function_code=modbus_defines.READ_HOLDING_REGISTERS,
                                            starting_address=0, quantity_of_x=max(1 + num_out, 2))
                logger.info("slave %d: read %dB from holdReg address 0 -> %s" %
                            (slave, max(1 + num_out, 2), _reg))

                # Check if the response length is valid
                if len(_reg) < 1:
                    logger.error("Response length is invalid %d" % len(_reg))
                    if timeout_modbus is not None:
                        self.client.set_timeout(self._timeout_modbus)            
                    raise ModbusInvalidResponseError("Response length is invalid %d" % len(_reg))

                # exit loop if command is completed (reg[0] bit15 = 0)
                _return = _reg[0]
                if _return & 0x8000 == 0:
                    break

            # check for read timeout
            except (modbus.ModbusError, ModbusInvalidResponseError, BaseException) as _err:
                if type(_err) == modbus.ModbusError:
                    _err = ModbusStatus(_err.get_exception_code()).name
                if time.time() - _start_time < _timeout_command:
                    logger.warning("Command response failed (%s), retry" % str(_err))
                    time.sleep(0.05)  # Short delay before retrying
                else:
                    _msg = "%s(): Command response timeout after %1.1fs (%s), abort" % \
                        (inspect.stack()[0].function, (time.time()-_start_time), str(_err))
                    logger.error(_msg)
                    if timeout_modbus is not None:
                        self.client.set_timeout(self._timeout_modbus)            
                    raise ModbusControlError(_msg)


        # Check for high-level error: reg[0] bit14 = 1, error code in reg[1]
        if _return & 0x4000 != 0:
            if len(_reg) < 2:
                logger.error("Expected error code in response but response length is too short, abort")
                if timeout_modbus is not None:
                    self.client.set_timeout(self._timeout_modbus)            
                raise ModbusControlError("Response too short to contain error code, abort")
            _err = _reg[1]
            if _err > 2 ** 15:  # Convert to uint16_t to int16_t
                _err = -(2 ** 16 - _err)
            msg = "_execute_command() failed with error (%s), abort" % ModbusControlStatus(_err)
            logger.error(msg)
            if timeout_modbus is not None:
                self.client.set_timeout(self._timeout_modbus)            
            raise ModbusControlError(msg)

        # Restore Modbus timeout
        if timeout_modbus is not None:
            self.client.set_timeout(self._timeout_modbus)
        
        # No error -> return success and read parameters (starts at reg[1])
        return {"values": _reg[1:1 + num_out]}


    #########
    # check client ModbusControl protocol version
    #########
    def check_protocol_version(self, slave: int = 1):
        """Check client ModbusControl protocol version against MODBUSCONTROL_PROTOCOL.
        Is performed as read w/o parameters via READ_INPUT_REGISTERS to address 0.
        On error raise exception 'ModbusControlError'

        Parameters
        ----------
        slave : int
            Modbus slave identifier

         Returns
         -------
         nothing. On error raise exception 'ModbusControlError'
        """
        _return = self.read_values(slave=slave, address=MODBUSCONTROL_ADDR_PROTOCOL, num_out=1)
        _version = str(round(float(_return["values"][0]*0.1), 1))
        if _version != MODBUSCONTROL_PROTOCOL:
            _msg = "ModbusControl protocol version mismatch %s vs. %s" % (MODBUSCONTROL_PROTOCOL, _version)
            logger.error("%s() failed with error: %s" % (inspect.stack()[0].function, _msg))
            raise ModbusControlError(_msg)


####################################################################
# MODULE TEST
####################################################################
# only execute this block of code if running this module directly,
# *not* if importing it. See http://effbot.org/pyfaq/tutor-what-is-if-name-main-for.htm
if __name__ == "__main__":

    # import additional libraries
    import platform
    import argparse

    # commandline parameters with defaults
    parser = argparse.ArgumentParser(description="ModbusControl test")
    if platform.system() == "Windows":
        parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='COM3')
    else:
        # parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='/dev/ttyUSB0')
        parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='/dev/ttyACM0')
    parser.add_argument('-b', '--baud', type=int, help='baud', required=False, default=115200)
    parser.add_argument('-i', '--id', type=int, help='slave ID', required=False, default=1)
    args = parser.parse_args()

    # set logging level
    logging.basicConfig(level=logging.ERROR)
    logger = logging.getLogger(__name__)

    # connect to the Modbus client
    client = BaseClient(port=args.port, baud=args.baud)

    # avoid USB communication while Arduino is still in bootloader
    print("wait for Arduino bootloader ... ", end="", flush=True)
    time.sleep(2.5)
    print("done")

    # print delimiter
    print("------------------------------")

    # assert ModbusControl protocol version. Exit on failure
    client.check_protocol_version()

    # read input register address 0 -> ok
    print("valid register read -> ", end="", flush=True)
    try:
        status = client.read_values(address=0, num_out=1)
        print(status, flush=True)
    except ModbusControlError as error:
        print(error, flush=True)

    # execute valid command with valid parameters (set pin 13) -> ok
    print("valid command -> ", end="", flush=True)
    try:
        status = client.execute_command(command=0x8001, param_in=[13, 1], num_out=2)
        print(status, flush=True)
    except ModbusControlError as error:
        print(error, flush=True)

    # print delimiter
    print("------------------------------")

    # read 1B from input register address 1000 -> fail
    print("invalid register address -> ", end="", flush=True)
    try:
        status = client.read_values(address=1000, num_out=1)
        print(status, flush=True)
    except ModbusControlError as error:
        print(error, flush=True)

    # read 1000B input register address 0 -> fail
    print("invalid register count -> ", end="", flush=True)
    try:
        status = client.read_values(address=0, num_out=1000)
        print(status, flush=True)
    except ModbusControlError as error:
        print(error, flush=True)

    # execute valid command with invalid parameters (set pin 1000) -> fail
    print("invalid command parameter -> ", end="", flush=True)
    try:
        status = client.execute_command(command=0x8001, param_in=[1000, 1], num_out=2)
        print(status, flush=True)
    except ModbusControlError as error:
        print(error, flush=True)

    # execute invalid command 0x8FFF -> fail
    print("invalid command code -> ", end="", flush=True)
    try:
        status = client.execute_command(command=0x8FFF, param_in=[], num_out=2)
        print(status, flush=True)
    except ModbusControlError as error:
        print(error, flush=True)
