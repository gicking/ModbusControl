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
#define MODBUS_NUM_HOLD_REG           100                       //!< number of Modbus holding registers. Actual registers [16b]

// Modbus input registers addresses. Note: address 0 is reserved for Modbus protocol version
#define MODBUS_ADDR_VERSION           1                         //!< Modbus input register address: software version
#define MODBUS_ADDR_UPTIME            2                         //!< Modbus input register address: time [ms]

// High-level command codes. Bit 15 must be 1 (=command pending flag), bit 14 must be cleard (=error flag)
#define MODBUS_CMD_SET_PIN            0x8001                    //!< Remote command: corresponds to digitalWrite()
#define MODBUS_CMD_GET_PIN            0x8002                    //!< Remote command: corresponds to digitalRead()
#define MODBUS_CMD_DELAY              0x8003                    //!< Remote command: corresponds to delay()
#define MODBUS_CMD_DELAY_NOSERIAL     0x8004                    //!< Remote command: corresponds to delay() w/o UART buffering
#define MODBUS_CMD_TEST               0xBFFF                    //!< Remote command: dummy test command

// ModbusControl error codes. Stored in holdReg[1] in case of an error
#define MODBUS_ERROR_ILLEGAL_CMD      -1                        //!< Error code: command not supported
#define MODBUS_ERROR_ILLEGAL_PARAM    -2                        //!< Error code: illegal parameter value / range


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
  
  // optional debug output
  #if defined(DEBUG_SERIAL) && defined(DEBUG_WRITE_HOLDING_REGISTER)
    DEBUG_SERIAL.println("writeHoldingRegs():");
  #endif

  // loop over data to write
  for (uint16_t i=0; i<length; i++)
  {
    // check address range
    if ((address+i) > (MODBUS_NUM_HOLD_REG-1))
    {
      // optional debug output
      #if defined(DEBUG_SERIAL) && defined(DEBUG_WRITE_HOLDING_REGISTER)
        DEBUG_SERIAL.print("  Error: illegal address ");
        DEBUG_SERIAL.println((int) (address+i));
      #endif

      // return error code
      return STATUS_ILLEGAL_DATA_ADDRESS;

    } // illegal address

    // copy data from Modbus buffer to holding register
    regModbus[address+i] = modbus_slave.readRegisterFromBuffer(i);
    
    // optional debug output
    #if defined(DEBUG_SERIAL) && defined(DEBUG_WRITE_HOLDING_REGISTER)
      DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.print((int) (address+i));
      DEBUG_SERIAL.print("\t0x");
      DEBUG_SERIAL.println(regModbus[address+i], HEX);
    #endif

  } // loop over data

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
  
  // optional debug output
  #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_HOLDING_REGISTER)
    DEBUG_SERIAL.println("readHoldingRegs():");
  #endif

  // loop over data to read
  for (uint16_t i=0; i<length; i++)
  {
    // check address range
    if ((address+i) > (MODBUS_NUM_HOLD_REG-1))
    {
      // optional debug output
      #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_HOLDING_REGISTER)
        DEBUG_SERIAL.print("  Error: illegal address ");
        DEBUG_SERIAL.println((int) (address+i));
      #endif

      // return error code
      return STATUS_ILLEGAL_DATA_ADDRESS;

    } // illegal address

    // copy data from holding register to Modbus buffer
    modbus_slave.writeRegisterToBuffer(i, regModbus[address+i]);
    
    // optional debug output
    #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_HOLDING_REGISTER)
      DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.print((int) (address+i));
      DEBUG_SERIAL.print("\t0x");
      DEBUG_SERIAL.println(regModbus[address+i], HEX);
    #endif

  } // loop over data

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
  
  // optional debug output
  #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_INPUT_REGISTER)
    DEBUG_SERIAL.println("readInputReg():");
  #endif

  // loop over data to read
  for (uint16_t i=0; i<length; i++)
  {
    // ModbusControl protocol version address. Must not be changed!
    if ((address+i) == MODBUSCONTROL_ADDR_PROTOCOL)
    {
      // optional debug output
      #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_INPUT_REGISTER)
        DEBUG_SERIAL.print("  protocol version = ");
        DEBUG_SERIAL.println(MODBUSCONTROL_PROTOCOL);
      #endif

      // copy data to Modbus buffer
      modbus_slave.writeRegisterToBuffer(i, MODBUSCONTROL_PROTOCOL);

    } // address == MODBUSCONTROL_ADDR_PROTOCOL

    ////////////////  add project specific reads here  ////////////////

    // firmware version address. May be changed
    else if ((address+i) == MODBUS_ADDR_VERSION)
    {
      // optional debug output
      #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_INPUT_REGISTER)
        DEBUG_SERIAL.print("  firmware version = ");
        DEBUG_SERIAL.println(SW_VERSION);
      #endif

      // copy data to Modbus buffer
      modbus_slave.writeRegisterToBuffer(i, SW_VERSION);

    } // address == MODBUS_ADDR_VERSION
    

    // uptime / millis() address. May be changed
    else if ((address+i) == MODBUS_ADDR_UPTIME)
    {
      // optional debug output
      #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_INPUT_REGISTER)
        DEBUG_SERIAL.print("  uptime = ");
        DEBUG_SERIAL.print((int) (millis()));
        DEBUG_SERIAL.println("ms");
      #endif

      // copy data to Modbus buffer
      modbus_slave.writeRegisterToBuffer(i, (uint16_t) (millis()));

    } // address == MODBUS_ADDR_UPTIME
    
    
    // address not mapped -> error
    else
    {
      // optional debug output
      #if defined(DEBUG_SERIAL) && defined(DEBUG_READ_INPUT_REGISTER)
        DEBUG_SERIAL.print("  Error: illegal address ");
        DEBUG_SERIAL.println((int) (address+i));
      #endif

      // return error
      return STATUS_ILLEGAL_DATA_ADDRESS;

    } // illegal address
      
  } // loop i over address

  // return ok
  return STATUS_OK;
  
} // readInputReg()



/**********
  initialize ModbusControl
**********/
void init_ModbusControl()
{
  // optional debug output
  #if defined(DEBUG_SERIAL)
    DEBUG_SERIAL.print("initialize ModbusControl ... ");
  #endif

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

  // optional debug output
  #if defined(DEBUG_SERIAL)
    DEBUG_SERIAL.println("done");
  #endif

} // setup()



/**********
  Handle Modbus low-level and ModbusControl high-level commands. 
  Command codes are in regModbus[0], parameters in regModbus[1..N]

  Modify as required!
**********/
void handle_ModbusControl(void)
{
  // for convenience
  uint16_t  *cmd  = &(regModbus[0]);      // pointer to command register
  uint16_t  *data = &(regModbus[1]);      // pointer to data array
  
  // handle Modbus low-level protocol
  modbus_slave.poll();

  // command received (bit 15 in regModbus[0] == 1)
  if ((*cmd) & 0x8000) {

    // execute command
    switch (*cmd) {

      //////
      // set pin:
      //  in:  data[0]=pin, data[1]=state
      //  out: none
      //////
      case MODBUS_CMD_SET_PIN:

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.print("handle_ModbusControl(): set pin ");
          DEBUG_SERIAL.print((int) data[0]);
          DEBUG_SERIAL.print("=");
          DEBUG_SERIAL.print((int) data[1]);
          DEBUG_SERIAL.print(" ... ");
        #endif

        // optional range check: allow only pin 13 (=LED) for example
        if (data[0] != 13)
        {
          // optional debug output
          #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
            DEBUG_SERIAL.println("Error: pin out of range");
          #endif

          // set error indication (=bit 14) in regModbus[0]
          (*cmd) |= 0x4000;
          
          // set error code in regModbus[1]
          data[0] = MODBUS_ERROR_ILLEGAL_PARAM;

          // exit switch
          break;
          
        } // parameter check

        // execute command. Note order to avoid glitches
        digitalWrite(data[0], data[1]);
        pinMode(data[0], OUTPUT);

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.println("done");
        #endif
        
        break; // MODBUS_CMD_SET_PIN

      
      //////
      // read pin:
      //  in:  data[0]=pin
      //  out: data[1]=state
      //////
      case MODBUS_CMD_GET_PIN:

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.print("handle_ModbusControl(): get pin ");
          DEBUG_SERIAL.print((int) data[0]);
          DEBUG_SERIAL.print(" ... ");
        #endif

        // optional range check: allow als pins except pin 13 (=LED) for example
        if (data[0] == 13)
        {
          // optional debug output
          #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
            DEBUG_SERIAL.println("Error: pin out of range");
          #endif

          // set error indication (=bit 14) in regModbus[0]
          (*cmd) |= 0x4000;
          
          // set error code in regModbus[1]
          data[0] = MODBUS_ERROR_ILLEGAL_PARAM;

          // exit switch
          break;
          
        } // parameter check

        // execute command
        pinMode(data[0], INPUT_PULLUP);
        data[1] = digitalRead(data[0]);

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          if (data[1] == 0)
            DEBUG_SERIAL.println("state=LOW");
          else
            DEBUG_SERIAL.println("state=HIGH");
        #endif
        
        break; // MODBUS_CMD_GET_PIN

      
      //////
      // wait some time
      //  in:  reg[1]=time[ms]
      //  out: none
      //////
      case MODBUS_CMD_DELAY:

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.print("handle_ModbusControl(): delay ");
          DEBUG_SERIAL.print((int) data[0]);
          DEBUG_SERIAL.print("ms ... ");
        #endif

        // execute command
        delay(data[0]);

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.println("done");
        #endif
        
        break; // MODBUS_CMD_DELAY

      
      //////
      // wait some time with Serial disabled
      //  in:  reg[1]=time[ms]
      //  out: none
      //////
      case MODBUS_CMD_DELAY_NOSERIAL:

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.print("handle_ModbusControl(): delay w/o serial ");
          DEBUG_SERIAL.print((int) data[0]);
          DEBUG_SERIAL.print("ms ... ");
        #endif

        // disable Modbus interface
        MODBUS_SERIAL.end();
        
        // execute command
        delay(data[0]);

        // re-enable Modbus interface
        MODBUS_SERIAL.begin(MODBUS_BAUDRATE);
        while(!MODBUS_SERIAL);

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.println("done");
        #endif
        
        break; // MODBUS_CMD_DELAY_NO_SERIAL
  
      
      //////
      // dummy test command
      //  in:  reg[1..N]
      //  out: reg[1..N]
      //////
      case MODBUS_CMD_TEST:

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.print("handle_ModbusControl(): test command ... ");
        #endif

        // add test code here

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.println("done");
        #endif
        
        break; // MODBUS_CMD_TEST
    
      
      //////
      // unknown command. Don't change!
      //////
      default:

        // optional debug output
        #if defined(DEBUG_SERIAL) && defined(DEBUG_EXECUTE_COMMAND)
          DEBUG_SERIAL.print("handle_ModbusControl(): Error, illegal command code 0x");
          DEBUG_SERIAL.println((*cmd), HEX);
        #endif
      
        // set error indication (=bit 14) in regModbus[0]
        (*cmd) |= 0x4000;
        
        // set error code in regModbus[1]
        data[0] = MODBUS_ERROR_ILLEGAL_CMD;
        
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
