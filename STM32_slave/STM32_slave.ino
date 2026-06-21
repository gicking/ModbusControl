/*************************

  Example for remote controlling an Nucleo-L432KC via USB/ST-Link and ModbusControl from PC.

*************************/

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
