/************************************************************/
/*
   Chibi Servo Example
   This is an example of controlling a servo wirelessly. The command
   line is implemented along with three commands: servo, getsaddr, setsaddr.
   "getsaddr" gets the address of the node. "setsaddr" sets the address 
   of the node. Each of the nodes should be set with a unique 16-bit
   address. The servo command will move the servo on the remote node. There
   is also a printout on each node for the received servo position. It can
   be viewed by connecting the node to a serial terminal program.
   
   Directions for use:
   1. Load two nodes with this software.    
   2. Set unique addresses for each node.
   3. Connect servo up to 5V, GND, and digital pin 9.
   4. Connect to at least one node via serial terminal program, ie: Teraterm
   5. Send servo command to remote node: 
       servo <addr> <servo position>
   Note: the servo position must be a number between 0 and 180
*/
/************************************************************/

#include <Servo.h> 
#include <chibi.h>
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.write(0);   // set the servo to the 0 degree position

  chibiCmdInit(57600);  // initialize the chibi command line to 57600 bps
  chibiInit();
  
  chibiCmdAdd("servo", cmdServo);  // send servo position to remote node
  chibiCmdAdd("getsaddr", cmdGetShortAddr);  // set the short address of the node
  chibiCmdAdd("setsaddr", cmdSetShortAddr);  // get the short address of the node
} 
 
/**************************************************************************/
// Loop
/**************************************************************************/
void loop() 
{ 
  chibiCmdPoll();       // poll the command line for any user input from the serial port
  
  if (chibiDataRcvd())
  {
    byte len, pos, buf[CHB_MAX_PAYLOAD];
    len = chibiGetData(buf); 
    
    // Its assumed that the data will be in the first byte of the data
    pos = buf[0];
    if ((pos >= 0) && (pos <=180))
    {
       myservo.write(pos);
    }
    
    // Print out the position value that the servo will be moved to 
    Serial.print("Servo = "); Serial.println(pos, DEC); 
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
void cmdServo(int argc, char **argv)
{
  unsigned int addr;
  byte servo_pos, data[1];
  
  // argv[1] is the first argument entered into the command line. This should
  // be the destination address of the remote node we want to send the command to.
  // The address is always a hexadecimal value.
  addr = chibiCmdStr2Num(argv[1], 16);
  
  // argv[2] is the second argument entered into the command line. This should
  // be the servo position. We're going to send this data to the remote node.
  servo_pos = chibiCmdStr2Num(argv[2], 10);
  data[0] = servo_pos;
  
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
