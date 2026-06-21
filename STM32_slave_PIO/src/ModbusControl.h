/*************************
  \file ModbusControl.h

  \author G. Icking-Konert

  \brief declaration of ModbusControl protocol and client commands

  Declaration of ModbusControl protocol and client commands.

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
    - Address 0 is reserved for ModbusControl protocol version MODBUSCONTROL_VERSION. Is checked on opening port
    - Content is updated by call-back function
    - Measured execution time for read command is ~4.0ms (Debian Linux 18.04 and USB2.0)

  2) Trigger command execution on client with optional parameters:
    - Use combination of Modbus commands WRITE_MULTIPLE_HOLDING_REGISTERS and READ_HOLDING_REGISTERS
    - Write to and read from holding register holdReg[] must start at address 0
    - Command code is written to holdReg[0], parameters to holdReg[1..(N-1)]
    - A pending command is indicated via holdReg[0].b15=1 (set by master)
    - After command completion, holdReg[0].b15 is cleared by slave
    - After successful command execution, return values are stored in holdReg[1..(N-1)]
    - An error on client side is indicated by setting holdReg[0] bit 14. In this case the error code is stored in holdReg[1]
    - Measured execution time for complete command (=write+read) is ~8.1ms (Debian Linux 18.04 and USB2.0)

*************************/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _MODBUSCONTROL_H_
#define _MODBUSCONTROL_H_


/*-----------------------------------------------------------------------------
    GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize ModbusControl
void init_ModbusControl(void);

/// handle Modbus low-level and ModbusControl high-level commands
void handle_ModbusControl(void);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _MODBUSCONTROL_H_
