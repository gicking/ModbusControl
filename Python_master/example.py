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
    MODBUS_CMD_SET_PIN      = 0x8001        # Remote command: corresponds to digitalWrite(pin, state)
    MODBUS_CMD_GET_PIN      = 0x8002        # Remote command: corresponds to digitalRead(pin)
    MODBUS_CMD_DELAY        = 0x8003        # Remote command: corresponds to delay(ms)
    MODBUS_CMD_DELAY_STALL  = 0x8004        # Remote command: corresponds to delay() w/o UART buffering


    #########
    # constructor
    #########
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout_modbus=0.2, timeout_command=0.2):
        """
        Create an object to control an Arduino via ModbusControl protocol.

        Parameters
        ----------
        port : string
            Name of COM port (string). On error lists available ports
        baud : int
            Communication speed [Baud]. Must be supported by driver
        timeout_modbus : float
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
        ModbusControl.BaseClient.__init__(self, port=port, baud=baud, timeout_modbus=timeout_modbus,
                                          timeout_command=timeout_command)


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
        status = self.read_values(slave=slave, address=address, num_out=num_out)
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
        status = self.read_values(slave=slave, address=address, num_out=num_out)
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
        status = self.execute_command(slave=slave, command=Client.MODBUS_CMD_SET_PIN, param_in=[pin, state], num_out=0)
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
        status = self.execute_command(slave=slave, command=Client.MODBUS_CMD_GET_PIN, param_in=[pin], num_out=2)
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
        # set appropriate command timeout
        _timeout_command = time / 1000 + 0.1

        # execute command
        status = self.execute_command(slave=slave, command=Client.MODBUS_CMD_DELAY, timeout_command=_timeout_command,
                                      param_in=[time], num_out=0)
        logger.info("slave %d: delay(): delay %d ms -> %s" % (slave, time, str(status)))
        return


    #########
    # wait time [ms] without Modbus handling (application specific)
    #########
    def delay_no_serial(self, slave=1, time=None):
        """Wait some time [ms] without Modbus handling. Corresponds to Arduino delay(),
        but Modbus is not handled.
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
        >>> device.delay_no_serial(time=100)
        """

        # set appropriate command timeout
        _timeout_command = time/1000+0.1

        # execute command
        status = self.execute_command(slave=slave, command=Client.MODBUS_CMD_DELAY_STALL,
                                      timeout_command=_timeout_command, param_in=[time], num_out=0)
        logger.info("slave %d: delay_no_serial(): delay %d ms w/o UART handling -> %s" % (slave, time, str(status)))
        return


####################################################################
# MODULE TEST
####################################################################
# only execute this block of code if running this module directly,
# *not* if importing it. See http://effbot.org/pyfaq/tutor-what-is-if-name-main-for.htm
if __name__ == "__main__":

    # import additional libraries
    import platform
    import sys
    import time
    import argparse

    ########
    # commandline parameters with defaults
    ########
    parser = argparse.ArgumentParser(description="ModbusControl device example")
    if platform.system() == "Windows":
        parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='COM3')
    else:
        parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='/dev/ttyUSB0')
        # parser.add_argument('-p', '--port', type=str, help='port name', required=False, default='/dev/ttyACM0')
    parser.add_argument('-b', '--baud', type=int, help='baud', required=False, default=115200)
    parser.add_argument('-i', '--id', type=int, help='slave ID', required=False, default=1)
    args = parser.parse_args()

    # set logging level
    logging.basicConfig(level=logging.ERROR)
    logger = logging.getLogger(__name__)

    ########
    # connect to the Modbus client
    ########
    client = Client(port=args.port, baud=args.baud, timeout_modbus=0.1)

    # avoid USB communication while Arduino is still in bootloader
    print("wait for Arduino bootloader ... ", end="", flush=True)
    time.sleep(2.5)
    print("done")

    ########
    # assert ModbusControl protocol version. Exit on failure
    ########
    client.check_protocol_version(slave=args.id)

    ########
    # read client firmware version
    ########
    version = client.read_version(slave=args.id)["version"]
    print("client ID=%d SW version v%d.%d\n" % (args.id, version["major"], version["minor"]))


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
        millis = client.read_uptime(slave=args.id)["millis"]
        print("slave uptime %1.1f s" % (millis * 0.001))

        ########
        # toggle LED pin (=13)
        ########
        output_pin = 13
        client.set_pin(slave=args.id, pin=output_pin, state=int(output_state))
        print("set pin %d : %d" % (output_pin, output_state))
        output_state = not output_state

        ########
        # read state of pin 8
        ########
        input_pin = 8
        input_state = client.read_pin(slave=args.id, pin=input_pin)["state"]
        print("read pin %d : %d" % (input_pin, input_state))

        ########
        # wait some time (with serial interrupts)
        ########
        pause = 1000  # ms
        print("delay %1.1fs with serial interrupts" % (pause / 1000.0))
        client.delay(slave=args.id, time=pause)

        ########
        # wait some time (without serial interrupts)
        ########
        pause = 1000  # ms
        print("delay %1.1fs without serial interrupts" % (pause / 1000.0))
        client.delay_no_serial(slave=args.id, time=pause)

        ########
        # indicate new loop
        ########
        sys.stdout.write("\n")
        sys.stdout.flush()
