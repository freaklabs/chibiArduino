/************************************************************/
/*
   Chibi LED Brightness Example
   This is an example of controlling the brighntess of an LED wirelessly. 
   The command line is implemented along with three commands: led, getsaddr, setsaddr.
   "getsaddr" gets the address of the node. "setsaddr" sets the address 
   of the node. Each of the nodes should be set with a unique 16-bit
   address. The "led" command will change the brightness of an LED connected to pin 9
   on the remote node. There is also a printout on each node for the received brightness 
   value. It can be viewed by connecting the node to a serial terminal program.
   
   Directions for use:
   1. Load two nodes with this software.    
   2. Set unique addresses for each node.
   3. Connect LED to pin 9 and GND.
   4. Connect to at least one node via serial terminal program. Ex: Teraterm
   5. Send led command to remote node: 
       led <addr> <brightness>
   Note: the LED brightness value must be a number between 0 and 255
*/
/************************************************************/
#include <chibi.h>

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()  
{ 
  pinMode(9, OUTPUT); // declare pin 9 to be an output
  analogWrite(9, 0); // initialize pin 9 to have a brightness of 0
  
  chibiCmdInit(57600);  // initialize the chibi command line to 57600 bps
  chibiInit();
  
  chibiCmdAdd("led", cmdLed);  // send servo position to remote node
  chibiCmdAdd("getsaddr", cmdGetShortAddr);  // set the short address of the node
  chibiCmdAdd("setsaddr", cmdSetShortAddr);  // get the short address of the node
} 

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()  { 
  chibiCmdPoll();       // poll the command line for any user input from the serial port
  
  if (chibiDataRcvd())
  {
    byte val, buf[CHB_MAX_PAYLOAD];
    chibiGetData(buf); 
    
    // Its assumed that the data will be in the first byte of the data
    val = buf[0];
    analogWrite(9, val);
    
    // Print out the position value that the servo will be moved to 
    Serial.print("LED Brightness = "); Serial.println(val, DEC); 
  }  
}

/**************************************************************************/
// USER FUNCTIONS
/**************************************************************************/

/**************************************************************************/
/*!
   Send data to control the servo.
   Usage: servo <addr> <position>
*/
/**************************************************************************/
void cmdLed(int argc, char **argv)
{
  unsigned int addr;
  byte ledVal, data[1];
  
  // argv[1] is the first argument entered into the command line. This should
  // be the destination address of the remote node we want to send the command to.
  // The address is always a hexadecimal value.
  addr = chibiCmdStr2Num(argv[1], 16);
  
  // argv[2] is the second argument entered into the command line. This should
  // be the servo position. We're going to send this data to the remote node.
  ledVal = chibiCmdStr2Num(argv[2], 10);
  data[0] = ledVal;
  
  // Send out the data
  chibiTx(addr, data, 1);
}

/**************************************************************************/
/*!
    Get short address of device from EEPROM
    Usage: getsaddr
*/
/**************************************************************************/
void cmdGetShortAddr(int arg_cnt, char **args)
{
  int val;
  
  val = chibiGetShortAddr();
  Serial.print("Short Address: "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!
    Write short address of device to EEPROM
    Usage: setsaddr <addr>
*/
/**************************************************************************/
void cmdSetShortAddr(int arg_cnt, char **args)
{
  int val;
  
  val = chibiCmdStr2Num(args[1], 16);
  chibiSetShortAddr(val);
}    
