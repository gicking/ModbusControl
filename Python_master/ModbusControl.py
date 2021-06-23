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
    - Command code is written to holdReg[0], parameters to holdReg[1..(N-1)]
    - A pending command is indicated via holdReg[0] bit 15 is set by master
    - After command completion, holdReg[0] bit 15 is cleared by slave
    - After successful command execution, return values are stored in holdReg[1..(N-1)]
    - An error on client side is indicated by setting holdReg[0] bit 14. In this case the error code is stored in holdReg[1]
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
import platform
import time
import serial
import serial.tools.list_ports
import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
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
    timeout : float
        Default Modbus RTU receive timeout [s]

    Returns
    -------
        ModbusControl object with opened port

    Examples
    --------
    >>> import ModbusControl
    >>> device = ModbusControl.BaseClient(port="COM6", baud=115200)
    """

    #########
    # constructor
    #########
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.2):
        """
        Create an object to control a Modbus RTU client via ModbusControl high-level protocol.

        Parameters
        ----------
        port : string
            Name of COM port (string). On error lists available ports
        baud : int
            Communication speed [Baud]. Must be supported by driver
        timeout : float
            Default Modbus RTU receive timeout [s]

        Returns
        -------
            ModbusControl object with opened port

        Examples
        --------
        >>> import ModbusControl
        >>> device = ModbusControl.BaseClient(port="COM6", baud=115200)
        """

        # try opening serial port for Modbus RTU
        try:

            # store parameters
            self._port = port
            self._baud = baud
            self._timeout = timeout

            # open port to Modbus client
            self.client = modbus_rtu.RtuMaster(serial.Serial(port=port, baudrate=baud,
                                               bytesize=8, parity='N', stopbits=1, xonxoff=0))

            # set Modbus receive timeout [s]
            self.client.set_timeout(timeout)

            # enable Modbus RTU logging
            self.client.set_verbose(True)

        # opening COM port failed -> list ports and raise exception
        except BaseException as error:
            msg = "Opening port '%s' failed with %s\n" % (port, str(error))
            msg += "Available ports are: "
            ports = serial.tools.list_ports.comports()
            for port in ports:
                msg += "  %s" % port.device
            raise ModbusControlError(msg)

        # avoid communication while Arduino is still in bootloader
        time.sleep(1.5)

        # assert that ModbusControl protocol versions match
        self._check_protocol_version()

        # return Modbus object with COM port configured and protocol versions checked
        return

    #########
    # Read an input register from ModbusControl client (don't modify)
    #########
    def _read_values(self, slave=1, address=None, num_out=None):
        """Read the content of a Modbus RTU client input register via READ_INPUT_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        address:
            index in the client input register
        num_out : int
            number of values to read

        Returns
        -------
        dict
            "values": list of returned values (int16)
        """

        # read input register. On error raise exception 'ModbusControlError'
        try:

            # read input register
            reg = self.client.execute(slave=slave, function_code=cst.READ_INPUT_REGISTERS,
                                      starting_address=address, quantity_of_x=num_out)
            logger.info("slave %d: read %dB from inputReg address %d -> %s" % (slave, num_out, address, str(reg)))

            # return result
            return {"values": reg}

        # handle low-level Modbus errors
        except modbus_tk.modbus.ModbusError as error:
            error_code = error.get_exception_code()
            msg = "_read_values() failed with error: %s" % ModbusStatus(error_code).name
            logger.warning(msg)
            raise ModbusControlError(msg)
        except BaseException as error:
            msg = "_read_values() failed with error: %s" % (str(error))
            logger.warning(msg)
            raise ModbusControlError(msg)

    #########
    # Trigger a command on ModbusControl client (don't modify)
    #########
    def _execute_command(self, slave=1, command=None, param_in=None, num_out=None, timeout=None):
        """Trigger a command on the Modbus RTU client by writing via WRITE_MULTIPLE_HOLDING_REGISTERS
           and reading results via READ_HOLDING_REGISTERS.

         Parameters
         ----------
         slave : int
             Modbus slave identifier
         command:
             application command code
         param_in : list(int16)
             input parameters
         num_out : int
             number of return values
         timeout : float
             command execution timeout [s]

         Returns
         -------
         dict
            "values": list of returned parameters (int16)
         """

        # execute command. On error raise exception 'ModbusControlError'
        try:

            # set Modbus receive timeout [s]. Is required for long command execution times
            if timeout is not None:
                self.client.set_timeout(timeout)

            # write command (in reg[0]) with parameters (in reg[1..N])
            self.client.execute(slave=slave, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=0,
                                output_value=[command]+param_in)
            logger.info("slave %d: write %s to holdReg address 0" % (slave, str([hex(command)]+param_in)))

            # read until command is completed: reg[0] bit15 = 0
            while True:
                reg = self.client.execute(slave=slave, function_code=cst.READ_HOLDING_REGISTERS, starting_address=0,
                                          quantity_of_x=max(1 + num_out, 2))
                logger.info("slave %d: read %dB from holdReg address 0 -> %s" % (slave, max(1 + num_out, 2), str(reg)))
                result = reg[0]
                if result & 0x8000 == 0:
                    break

            # check for high-level error: reg[0] bit14 = 1, error code in reg[1]
            if result & 0x4000 != 0:
                error = reg[1]
                if error > 2 ** 15:  # convert to uint16_t to int16_t
                    error = -(2 ** 16 - error)
                msg = "_execute_command() failed with error: %s" % ModbusControlStatus(error)
                logger.warning(msg)
                raise ModbusControlError(msg)

            # no error -> return success and read parameters (starts at reg[1])
            return {"values": reg[1:1+num_out]}

        # handle low-level Modbus error returns
        except modbus_tk.modbus.ModbusError as error:
            error_code = error.get_exception_code()
            logger.warning(ModbusStatus(error_code))
            raise ModbusControlError(ModbusStatus(error_code))
        except BaseException as error:
            logger.warning(str(error))
            raise ModbusControlError(str(error))

        # restore Modbus timeout before leaving function, see
        # https://stackoverflow.com/questions/11604699/is-there-a-way-to-do-more-work-after-a-return-statement
        finally:
            if timeout is not None:
                self.client.set_timeout(self._timeout)

    #########
    # read client firmware version
    #########
    def _check_protocol_version(self, slave=1):
        """Check client ModbusControl protocol version.
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
        result = self._read_values(slave=slave, address=MODBUSCONTROL_ADDR_PROTOCOL, num_out=1)
        version = str(round(float(result["values"][0]*0.1), 1))
        if version != MODBUSCONTROL_PROTOCOL:
            msg = "ModbusControl protocol version mismatch %s vs. %s" % (MODBUSCONTROL_PROTOCOL, version)
            logger.error("_check_protocol_version() failed with error: %s" % msg)
            raise ModbusControlError(msg)


####################################################################
# MODULE TEST
####################################################################
# only execute this block of code if running this module directly,
# *not* if importing it. See http://effbot.org/pyfaq/tutor-what-is-if-name-main-for.htm
if __name__ == "__main__":

    # import additional libraries
    import argparse

    # commandline parameters with defaults
    parser = argparse.ArgumentParser(description="ModbusControl test")
    parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='/dev/ttyACM0')
    parser.add_argument('-b', '--baud', type=int, help='baud', required=False, default=115200)
    parser.add_argument('-i', '--id', type=int, help='slave ID', required=False, default=1)
    args = parser.parse_args()

    # set logging level
    logging.basicConfig(level=logging.CRITICAL)
    logger = logging.getLogger(__name__)

    # connect to the Modbus client
    client = BaseClient(port=args.port, baud=args.baud, timeout=0.2)

    # read input register address 0 -> ok
    try:
        status = client._read_values(address=0, num_out=1)
        print("valid register read -> %s" % (str(status)))
    except ModbusControlError as error:
        print(error)

    # execute valid command with valid parameters (set pin 13) -> ok
    try:
        status = client._execute_command(command=0x8001, param_in=[13, 1], num_out=2)
        print("valid command -> %s" % (str(status)))
    except ModbusControlError as error:
        print(error)

    # print delimiter
    print("------------------------------")

    # read 1B from input register address 1000 -> fail
    try:
        status = client._read_values(address=1000, num_out=1)
        print(status)
    except ModbusControlError as error:
        print(error)

    # read 1000B input register address 0 -> fail
    try:
        status = client._read_values(address=0, num_out=1000)
        print(status)
    except ModbusControlError as error:
        print(error)

    # execute valid command with valid parameters (set pin 1000) -> fail
    try:
        status = client._execute_command(command=0x8001, param_in=[1000, 1], num_out=2)
        print(status)
    except ModbusControlError as error:
        print(error)

    # execute invalid command 0x8FFF -> fail
    try:
        status = client._execute_command(command=0x8FFF, param_in=[], num_out=2)
        print(status)
    except ModbusControlError as error:
        print(error)
