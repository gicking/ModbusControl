# ModbusControl

ModbusControl is a high-level protocol based on [Modbus RTU](https://en.wikipedia.org/wiki/Modbus), which facilitates remote controlling clients via serial interfaces like USB or RS485.

General principle:
    - All registers are 16-bit (Modbus standard)
    - Only functions READ_INPUT_REGISTERS, WRITE_MULTIPLE_HOLDING_REGISTERS, and READ_HOLDING_REGISTERS are used
    - Any other Modbus function codes return error code ILLEGAL_FUNCTION

1) Read static values or time-critical reads with low communication overhead:
    - Use Modbus command READ_INPUT_REGISTERS
    - Direct read of input registers via target address
    - No parameters other than address are passed
    - Any valid address within [0,(MODBUS_NUM_INPUT_REG-1)] is allowed
    - Address 0 is reserved for ModbusControl protocol version MODBUSCONTROL_PROTOCOL
    - Measured execution time for read command is ~4.0ms (Debian Linux 18.04 via USB2.0)

2) Trigger command execution on client with optional parameters:
    - Use combination of Modbus commands WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS
    - Write to and read from holding register holdReg[] must start at address 0
    - Command code is written to holdReg[0], parameters to holdReg[1..(N-1)]
    - A pending command is indicated via holdReg[0] bit 15 is set by master
    - After command completion, holdReg[0] bit 15 is cleared by slave
    - After successful command execution, return values are stored in holdReg[1..(N-1)]
    - An error on client side is indicated by setting holdReg[0] bit 14. In this case the error code is stored in holdReg[1]
    - Measured execution time for complete command (=write+read) is ~8.1ms (Debian Linux 18.04 and USB2.0)

This project contains:
  - Python example for ModbusControl master:
    - Tested for Windows and Linux. Should also work for MacOSX etc.
    - Requires Python installation (2.x or 3.x) and libraries serial and modbus_tk
    - Used serial port (e.g. USB) must shows up as device or VCP 
  - Arduino example for ModbusControl slave:
    - Tested for Arduino Mega and Due. Should be compatible with all AVR, SAM and SAMD Arduinos with HW-Serial
    - Requires installed [Modbus RTU slave library](https://github.com/yaacov/ArduinoModbusSlave) 

For bug reports or feature requests please send me a note.

Have fun!
Georg


