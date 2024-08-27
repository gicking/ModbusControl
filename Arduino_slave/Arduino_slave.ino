/*************************

  Example for remote controlling an Arduino via USB and ModbusControl from PC.

*************************/

/*-----------------------------------------------------------------------------
    GLOBAL MACROS
-----------------------------------------------------------------------------*/

// misc constants
#define SW_VERSION      12            //!< dummy software version x.x -> 1.2

// debug console. Use either HW or SW serial interface. Note: SoftwareSerial has limited baudrate
//#define DEBUG_SERIAL    Serial1       //!< serial interface for debug output. Comment out for no debug
#define DEBUG_BAUDRATE  115200        //!< baudrate for debug output. Only relevant if DEBUG_SERIAL is defined
//#define DEBUG_PIN_RX    6             //!< SoftwareSerial Rx pin. Only relevant if DEBUG_SERIAL is defined. If defined, overwrite DEBUG_SERIAL with SoftwareSerial
//#define DEBUG_PIN_TX    7             //!< SoftwareSerial Tx pin. Only relevant if DEBUG_SERIAL is defined
#define DEBUG_WRITE_HOLDING_REGISTER  //!< print debug output for writeHoldingRegs()
#define DEBUG_READ_HOLDING_REGISTER   //!< print debug output for readHoldingRegs()
#define DEBUG_READ_INPUT_REGISTER     //!< print debug output for readInputReg()
#define DEBUG_EXECUTE_COMMAND         //!< print debug output for readInputReg()

// debug pin
//#define PIN_DEBUG                   //!< optional pin to indicate command execution. Comment out for none


/*-----------------------------------------------------------------------------
    GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/// if use SoftwareSerial, redefine debug console
#if defined(DEBUG_SERIAL) && defined(DEBUG_PIN_RX)
  SoftwareSerial sw_serial(DEBUG_PIN_RX, DEBUG_PIN_TX);
  #undef  DEBUG_SERIAL
  #define DEBUG_SERIAL sw_serial
#endif


/*-----------------------------------------------------------------------------
    GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**********
  initialize
**********/
void setup()
{
  // initialize optional debug console
  #if defined(DEBUG_SERIAL)
    DEBUG_SERIAL.begin(DEBUG_BAUDRATE);
    while(!DEBUG_SERIAL);
    DEBUG_SERIAL.println("\n");
    DEBUG_SERIAL.println("begin debug output");
  #endif

  // initialize ModbusControl
  init_ModbusControl();

} // setup()



/**********
  main loop
**********/
void loop()
{  
  // ModbusControl protocol handler. Must be called frequently!
  handle_ModbusControl();

} // loop()
