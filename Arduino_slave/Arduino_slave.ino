/*************************

  Example for remote controlling an Arduino via USB and ModbusControl from PC.

*************************/

/*-----------------------------------------------------------------------------
    GLOBAL MACROS
-----------------------------------------------------------------------------*/

// misc constants
#define SW_VERSION                    12                        //!< dummy software version x.x -> 1.2


/*-----------------------------------------------------------------------------
    GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**********
  initialize
**********/
void setup()
{
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
