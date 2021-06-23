#!/usr/bin/python3
# -*- coding: utf-8 -*-
# pylint: disable=too-many-arguments, line-too-long
"""A Python example for remote controlling an Arduino via USB and ModbusControl.

Example how to control a device using ModbusControl protocol.

COM port driver need to be installed and device has to show up as virtual COM port (VCP).
Works transparently on Windows and Posix (Linux, Mac OS X).

Version
-------

- 2021-06-13 (v1.0) initial version

Class
-----
"""
# pylint: enable=line-too-long


###########
# IMPORT REQUIRED MODULES
###########
import ModbusControl
import logging
logger = logging.getLogger(__name__)


class Client(ModbusControl.BaseClient):
    """
    Class for controlling an Arduino via ModbusControl protocol and USB.
    Derived from ModbusControl base class

    Parameters
    ----------
    port : string
        Name of COM port (string). On error lists available ports
    baud : int
        Communication speed [Baud]. Must be supported by driver
    slave : int
        Default Modbus slave id
    timeout : float
        Default Modbus RTU receive timeout [s]

    Returns
    -------
        Device object with opened port

    Examples
    --------
    >>> import example
    >>> device = example.Client(port="COM6", baud=115200)
    """

    # ModbusControl client input register addresses (only read, no command trigger)
    # Note: address 0 is reserved for ModbusControl protocol version
    ADDRESS_VERSION = 1                 # Modbus input register address: client firmware version
    ADDRESS_MILLIS = 2                  # Modbus input register address: time [ms]

    # ModbusControl client commands
    # Note: bit 15 must be 1 (=command pending flag), bit 14 must be cleared (=error flag)
    COMMAND_SET_PIN = 0x8001            # Remote command: corresponds to digitalWrite(pin, state)
    COMMAND_GET_PIN = 0x8002            # Remote command: corresponds to digitalRead(pin)

    #########
    # constructor
    #########
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.2):
        """
        Create an object to control an Arduino via ModbusControl protocol.

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
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        """

        # just call base class constructor here
        ModbusControl.BaseClient.__init__(self, port=port, baud=baud, timeout=timeout)

    #########
    # read client firmware version
    #########
    def read_version(self, slave=1):
        """Read client software version.
        Is performed as read w/o parameters via READ_INPUT_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier

        Returns
        -------
        dict
            version: {"major": major, "minor": minor}

        Examples
        --------
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        >>> print("firmware: %s" % (str(device.read_version()["version"]))
        """
        address = Client.ADDRESS_VERSION
        num_out = 1
        status = self._read_values(slave=slave, address=address, num_out=num_out)
        logger.info("read_version(): slave %d: read %dB from inputReg address %d -> %s" %
                    (slave, num_out, address, str(status)))
        major = int(status["values"][0]/10)
        minor = status["values"][0] - 10 * major
        return {"version": {"major": major, "minor": minor}}

    #########
    # read client uptime
    #########
    def read_uptime(self, slave=1):
        """Read client uptime [ms]. Corresponds to Arduino millis().
        Is performed as read w/o parameters via READ_INPUT_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier

        Returns
        -------
        dict
            millis: uptime [ms]

        Examples
        --------
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        >>> print(device.read_uptime()["millis"])
        """
        address = Client.ADDRESS_MILLIS
        num_out = 1
        status = self._read_values(slave=slave, address=address, num_out=num_out)
        logger.info("slave %d: read_uptime(): read %dB from inputReg address %d -> %s" %
                    (slave, num_out, address, str(status)))
        return {"millis": status["values"][0]}

    #########
    # set client pin state
    #########
    def set_pin(self, slave=1, pin=None, state=None):
        """Set pin state. Corresponds to Arduino digitalWrite().
        Is performed via WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        pin : int
            pin number
        state : int
            pin state (1=on, 0=off)

        Returns
        -------
        nothing

        Examples
        --------
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        >>> device.set_pin(pin=13, state=1)
        """
        status = self._execute_command(slave=slave, command=Client.COMMAND_SET_PIN, param_in=[pin, state], num_out=0)
        logger.info("slave %d: set_pin(): set pin %d = %d -> %s" % (slave, pin, state, str(status)))
        return

    #########
    # read client pin state (application specific)
    #########
    def read_pin(self, slave=1, pin=None):
        """Read pin state. Corresponds to Arduino digitalRead().
        Is performed via WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        pin : int
            pin number

        Returns
        -------
        dict
            "state": pin state

        Examples
        --------
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        >>> print(device.read_pin(pin=8)["state"])
        """
        status = self._execute_command(slave=slave, command=Client.COMMAND_GET_PIN, param_in=[pin], num_out=2)
        logger.info("slave %d: read_pin(): read pin %d -> %s" % (slave, pin, str(status)))
        return {"state": status["values"][1]}


####################################################################
# MODULE TEST
####################################################################
# only execute this block of code if running this module directly,
# *not* if importing it. See http://effbot.org/pyfaq/tutor-what-is-if-name-main-for.htm
if __name__ == "__main__":

    # import additional libraries
    import sys
    import time
    import argparse

    ########
    # commandline parameters with defaults
    ########
    parser = argparse.ArgumentParser(description="ModbusControl device example")
    parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='/dev/ttyACM0')
    parser.add_argument('-b', '--baud', type=int, help='baud', required=False, default=115200)
    parser.add_argument('-i', '--id', type=int, help='slave ID', required=False, default=1)
    args = parser.parse_args()

    # set logging level
    logging.basicConfig(level=logging.ERROR)
    logger = logging.getLogger(__name__)

    ########
    # connect to the Modbus client
    ########
    client = Client(port=args.port, baud=args.baud, timeout=0.2)

    # for convenience
    id = slave=args.id

    ########
    # read client firmware version
    ########
    version = client.read_version(slave=id)["version"]
    print("client ID=%d SW version v%d.%d\n" % (args.id, version["major"], version["minor"]))
    time.sleep(1.5)

    ########
    # main loop
    ########
    output_state = True
    while True:

        ########
        # get client uptime [ms]
        ########
        millis = client.read_uptime(slave=id)["millis"]
        print("uptime %1.1f s" % (round(millis * 0.001, 1)))

        ########
        # toggle LED pin (=13)
        ########
        output_pin = 13
        client.set_pin(slave=id, pin=output_pin, state=int(output_state))
        print("set pin %d : %d" % (output_pin, output_state))
        output_state = not output_state

        ########
        # read state of pin 8
        ########
        input_pin = 8
        input_state = client.read_pin(slave=id, pin=input_pin)["state"]
        print("read pin %d : %d" % (input_pin, input_state))

        ########
        # wait some time [s]
        ########
        sys.stdout.write("\n")
        sys.stdout.flush()
        time.sleep(1)
