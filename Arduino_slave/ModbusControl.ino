/*************************

  Arduino remote control via USB and Modbus RTU protocol 

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
    INCLUDES
-----------------------------------------------------------------------------*/

// download Modbus RTU slave library from https://github.com/yaacov/ArduinoModbusSlave
#include <ModbusSlave.h>


/*-----------------------------------------------------------------------------
    MODULE MACROS
-----------------------------------------------------------------------------*/

// General Modbus parameters
#define MODBUS_SERIAL                 Serial                    //!< Serial interface used for Modbus
#define MODBUS_RS485_CTRL_PIN         MODBUS_CONTROL_PIN_NONE   //!< Pin for RS485 direction control. Use MODBUS_CONTROL_PIN_NONE for no (e.g. USB)
#define MODBUS_BAUDRATE               115200                    //!< Modbus communication speed [Baud]
#define MODBUS_ID                     1                         //!< Modbus slave ID

// ModbusControl protocol version and input register address. Not project specific!
#define MODBUSCONTROL_PROTOCOL        10                        //!< ModbusControl protocol version x.x -> 1.0
#define MODBUSCONTROL_ADDR_PROTOCOL   0                         //!< Modbus input register address: reserved ModbusControl protocol version

// number of Modbus input and holding registers
#define MODBUS_NUM_INPUT_REG          3                         //!< number of Modbus input registers. Only virtual [16b]
#define MODBUS_NUM_HOLD_REG           10                        //!< number of Modbus holding registers. Actual registers [16b]

// Modbus input registers addresses. Note: address 0 is reserved for Modbus protocol version
#define MODBUS_ADDR_VERSION           1                         //!< Modbus input register address: software version
#define MODBUS_ADDR_MILLIS            2                         //!< Modbus input register address: time [ms]

// High-level command codes. Bit 15 must be 1 (=command pending flag), bit 14 must be cleard (=error flag)
#define MODBUS_CMD_SET_PIN            0x8001                    //!< Remote command: corresponds to digitalWrite()
#define MODBUS_CMD_GET_PIN            0x8002                    //!< Remote command: corresponds to digitalRead()
#define MODBUS_CMD_DELAY              0x8003                    //!< Remote command: corresponds to delay()
#define MODBUS_CMD_DELAY_STALL        0x8004                    //!< Remote command: corresponds to delay() w/o UART buffering

// ModbusControl error codes. Stored in holdReg[1] in case of an error
#define MODBUS_ERROR_ILLEGAL_CMD      -1                        //!< Error code: command not supported
#define MODBUS_ERROR_ILLEGAL_PARAM    -2                        //!< Error code: illegal parameter value / range

// misc macros
//#define PIN_DEBUG                     7                         //!< optional pin to indicate command execution. Comment out for none


/*-----------------------------------------------------------------------------
    MODULE VARIABLES
-----------------------------------------------------------------------------*/

// Modbus holding registers
uint16_t  regModbus[MODBUS_NUM_HOLD_REG];

// Modbus RTU slave instance
Modbus    modbus_slave(MODBUS_SERIAL, MODBUS_ID, MODBUS_RS485_CTRL_PIN);


/*-----------------------------------------------------------------------------
    MODBUS HANDLER FUNCTIONS

    The handler functions must return an uint8_t and take the following parameters:
       uint8_t  fc - function code
       uint16_t address - first register/coil address
       uint16_t length/status - length of data / coil status
-----------------------------------------------------------------------------*/

/**********
  handle Modbus low-level command Write Holding Registers (function code=16)

  Don't change!
**********/
uint8_t writeHoldingRegs(uint8_t fc, uint16_t address, uint16_t length)
{
  // avoid compiler warnings
  (void) fc;
  
  // address range check. Starting address must be 0 (=command code)
  if ((address != 0) || ((address + length) > MODBUS_NUM_HOLD_REG))
    return STATUS_ILLEGAL_DATA_ADDRESS;

  // copy data to global array. Command is triggered by high-level handler
  for (uint16_t i=0; i<length; i++)
    regModbus[address+i] = modbus_slave.readRegisterFromBuffer(i);
  
  // return ok
  return STATUS_OK;
  
} // writeHoldingReg()



/**********
  handle Modbus low-level command Read Holding Registers (function code=3)

  Don't change!
**********/
uint8_t readHoldingRegs(uint8_t fc, uint16_t address, uint16_t length)
{
  // avoid compiler warnings
  (void) fc;
  
  // address range check. Starting address must be 0 (=command code)
  if ((address != 0) || ((address + length) > MODBUS_NUM_HOLD_REG))
    return STATUS_ILLEGAL_DATA_ADDRESS;

  // copy data from global array to Modbus buffer.
  for (uint16_t i=0; i<length; i++)
    modbus_slave.writeRegisterToBuffer(i, regModbus[address+i]);
  
  // return ok
  return STATUS_OK;
  
} // readHoldingRegs()


/**********
  handle Modbus low-level command Read Input Registers (function code=4)

  Add project specific reads starting from address 1 (0 is reserved for protocol version)
**********/
uint8_t readInputReg(uint8_t fc, uint16_t address, uint16_t length)
{
  // avoid compiler warnings
  (void) fc;
  
  // address range check. Any address within [0; (MODBUS_NUM_INPUT_REG-1)] is allowed
  if ((address > MODBUS_NUM_INPUT_REG) || ((address + length) > MODBUS_NUM_INPUT_REG))
    return STATUS_ILLEGAL_DATA_ADDRESS;

  // set content of Modbus respective addresses
  for (uint16_t i=0; i<length; i++)
  {
    // ModbusControl protocol version address. Must not be changed!
    if ((address+i) == MODBUSCONTROL_ADDR_PROTOCOL)
      modbus_slave.writeRegisterToBuffer(i, MODBUSCONTROL_PROTOCOL);
    
    ////////////////  add project specific reads here  ////////////////
    
    // firmware version address. May be changed
    else if ((address+i) == MODBUS_ADDR_VERSION)
      modbus_slave.writeRegisterToBuffer(i, SW_VERSION);
    
    // uptime / millis() address. May be changed
    else if ((address+i) == MODBUS_ADDR_MILLIS)
      modbus_slave.writeRegisterToBuffer(i, (uint16_t) (millis()));
    
    // address not mapped -> error
    else
      return STATUS_ILLEGAL_DATA_ADDRESS;
      
  } // loop i over address

  // return ok
  return STATUS_OK;
  
} // readInputReg()



/**********
  initialize ModbusControl
**********/
void init_ModbusControl()
{
  // initialize Modbus holding registers
  for (uint16_t i=0; i<MODBUS_NUM_HOLD_REG; i++)
    regModbus[i] = 0;

  // setup Modbus communication
  MODBUS_SERIAL.begin(MODBUS_BAUDRATE);
  modbus_slave.begin(MODBUS_BAUDRATE);

  // register callback functions when Modbus function code is received
  modbus_slave.cbVector[CB_READ_INPUT_REGISTERS]    = (ModbusCallback) readInputReg;
  modbus_slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = (ModbusCallback) writeHoldingRegs;
  modbus_slave.cbVector[CB_READ_HOLDING_REGISTERS]  = (ModbusCallback) readHoldingRegs;

  // configure debug pin to indicate command execution (optional)
  #if defined(PIN_DEBUG)
    digitalWrite(PIN_DEBUG, LOW);
    pinMode(PIN_DEBUG, OUTPUT); 
  #endif // PIN_DEBUG

} // setup()



/**********
  Handle Modbus low-level and ModbusControl high-level commands. 
  Command codes are in regModbus[0], parameters in regModbus[1..N]

  Modify as required!
**********/
void handle_ModbusControl(void)
{
  uint16_t  *cmd  = &(regModbus[0]);      // for convenience
  
  // handle Modbus low-level protocol
  modbus_slave.poll();

  // command received (bit 15 in regModbus[0] == 1)
  if ((*cmd) & 0x8000) {

    switch (*cmd) {

      //////
      // set pin:
      //  in:  reg[1]=pin, reg[2]=state
      //  out: none
      //////
      case MODBUS_CMD_SET_PIN:

        // range check: allow only pin 13 (=LED) for example
        if (regModbus[1] != 13)
        {
          // set error indication (=bit 14) in regModbus[0]
          (*cmd) |= 0x4000;
          
          // set error code in regModbus[1]
          regModbus[1] = MODBUS_ERROR_ILLEGAL_PARAM;

          // exit switch
          break;
          
        } // parameter check

        // execute command. Note order to avoid glitches
        digitalWrite(regModbus[1], regModbus[2]);
        pinMode(regModbus[1], OUTPUT);
        
        break; // MODBUS_CMD_SET_PIN

      
      //////
      // read pin:
      //  in:  reg[1]=pin
      //  out: reg[2]=state
      //////
      case MODBUS_CMD_GET_PIN:

        // range check: allow als pins except pin 13 (=LED) for example
        if (regModbus[1] == 13)
        {
          // set error indication (=bit 14) in regModbus[0]
          (*cmd) |= 0x4000;
          
          // set error code in regModbus[1]
          regModbus[1] = MODBUS_ERROR_ILLEGAL_PARAM;

          // exit switch
          break;
          
        } // parameter check

        // execute command
        pinMode(regModbus[1], INPUT_PULLUP);
        regModbus[2] = digitalRead(regModbus[1]);
        
        break; // MODBUS_CMD_GET_PIN

      
      //////
      // wait some time
      //  in:  reg[1]=time[ms]
      //  out: none
      //////
      case MODBUS_CMD_DELAY:

        // execute command
        delay(regModbus[1]);
        
        break; // MODBUS_CMD_DELAY

      
      //////
      // wait some time with Serial disabled
      //  in:  reg[1]=time[ms]
      //  out: none
      //////
      case MODBUS_CMD_DELAY_NO_SERIAL:

        // disable Modbus interface
        MODBUS_SERIAL.end();
        
        // execute command
        delay(regModbus[1]);

        // re-enable Modbus interface
        MODBUS_SERIAL.begin(MODBUS_BAUDRATE);
        while(!MODBUS_SERIAL);
        
        break; // MODBUS_CMD_DELAY_NO_SERIAL
      
      
      //////
      // unknown command. Don't change!
      //////
      default:
      
        // set error indication (=bit 14) in regModbus[0]
        (*cmd) |= 0x4000;
        
        // set error code in regModbus[1]
        regModbus[1] = MODBUS_ERROR_ILLEGAL_CMD;
        
    } // switch (cmd)

    // command processed -> clear bit 15 in regModbus[0]. Don't change!
    (*cmd) &= 0x7fff;

    // toggle debug pin to indicate command execution (optional)
    #if defined(PIN_DEBUG)
      digitalWrite(PIN_DEBUG, !digitalRead(PIN_DEBUG));
    #endif // PIN_DEBUG
    
  } // command received

} // handleModbusCmd()

// EOF
