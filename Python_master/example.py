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
    MODBUS_ADDR_VERSION = 1             # Modbus input register address: client firmware version
    MODBUS_ADDR_MILLIS = 2              # Modbus input register address: time [ms]

    # ModbusControl client commands
    # Note: bit 15 must be 1 (=command pending flag), bit 14 must be cleared (=error flag)
    MODBUS_CMD_SET_PIN      = 0x8001    # Remote command: corresponds to digitalWrite(pin, state)
    MODBUS_CMD_GET_PIN      = 0x8002    # Remote command: corresponds to digitalRead(pin)
    MODBUS_CMD_DELAY        = 0x8003    # Remote command: corresponds to digitalRead(pin)
    MODBUS_CMD_DELAY_NO_ISR = 0x8004    # Remote command: wait some time w/ interrupts disabled

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
        address = Client.MODBUS_ADDR_VERSION
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
        address = Client.MODBUS_ADDR_MILLIS
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
        status = self._execute_command(slave=slave, command=Client.MODBUS_CMD_SET_PIN, param_in=[pin, state], num_out=0)
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
        status = self._execute_command(slave=slave, command=Client.MODBUS_CMD_GET_PIN, param_in=[pin], num_out=2)
        logger.info("slave %d: read_pin(): read pin %d -> %s" % (slave, pin, str(status)))
        return {"state": status["values"][1]}


    #########
    # wait time [ms] (application specific)
    #########
    def delay(self, slave=1, time=None):
        """Wait some time [ms]. Corresponds to Arduino delay().
        Is performed via WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        time : int
            time to wait [ms]

        Returns
        -------
        nothing

        Examples
        --------
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        >>> device.delay(time=100)
        """
        status = self._execute_command(slave=slave, command=Client.MODBUS_CMD_DELAY, param_in=[time], num_out=0)
        logger.info("slave %d: delay(): delay %d ms -> %s" % (slave, time, str(status)))
        return


    #########
    # wait some time with interrupts disabled (application specific)
    #########
    def delay_no_interrupts(self, slave=1, cycles=None):
        """Wait some time with interrupts disabled.
        Is performed via WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS.

        Parameters
        ----------
        slave : int
            Modbus slave identifier
        cycles : int
            number of cycles to wait [1000]

        Returns
        -------
        nothing

        Examples
        --------
        >>> import example
        >>> device = example.Client(port="COM6", baud=115200)
        >>> device.delay_no_interrupts(cycles=300)
        """
        status = self._execute_command(slave=slave, command=Client.MODBUS_CMD_DELAY_NO_ISR, param_in=[cycles], num_out=0)
        logger.info("slave %d: delay_no_interrupts(): delay_no_interrupts %g cycles -> %s" % (slave, float(cycles*1000), str(status)))
        return


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
    client = Client(port=args.port, baud=args.baud, timeout=2)

    # avoid USB communication while Arduino is still in bootloader
    time.sleep(2)

    # for convenience
    id = slave=args.id

    ########
    # assert ModbusControl protocol version. Exit on failure
    ########
    client.check_protocol_version(slave=id)

    ########
    # read client firmware version
    ########
    version = client.read_version(slave=id)["version"]
    print("client ID=%d SW version v%d.%d\n" % (id, version["major"], version["minor"]))


    ########
    # main loop
    ########
    output_state = True
    time_start = time.time()    # [s]
    while True:

        ########
        # print PC runtime [s]
        ########
        time_curr = time.time()
        print("runtime %1.1f s" % (time_curr - time_start))

        ########
        # get client uptime (impacted if ISR disabled) [ms]
        ########
        millis = client.read_uptime(slave=id)["millis"]
        print("slave uptime %1.1f s" % (millis * 0.001))

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
        # wait some time (with or w/o interrupts)
        ########
        if True:
            pause = 1000  # ms
            print("delay %1.1fs" % (pause / 1000.0))
            client.delay(slave=id, time=pause)
        else:
            cycles = 1000   # 1000*NOP
            print("halt %g cycles" % (cycles*1000))
            client.delay_no_interrupts(slave=id, cycles=cycles)

        ########
        # indicate new loop
        ########
        sys.stdout.write("\n")
        sys.stdout.flush()
