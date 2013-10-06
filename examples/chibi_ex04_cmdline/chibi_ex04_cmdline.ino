/* Chibi for Arduino, Example 4 
This example shows how to use the command line parser integrated into chibi. It's 
very useful for interactive control of a node, especially for testing and experimentation.
You can define your own commands to transmit, read registers, toggle I/O pins, etc. 

There's a function at the bottom called strCat(..) that might look kind of complicated.
It just takes in an array of strings and concatenates (connects) them together. This is 
because the command line parser chops up anything typed into the command line into a string array
with the space as the delimiter. The strCat function just takes the string array and puts them all back 
together.
*/

#include <chibi.h>

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()
{  
  // Initialize the chibi command line and set the speed to 57600 bps
  chibiCmdInit(57600);
  
  // Initialize the chibi wireless stack
  chibiInit();

  // This is where you declare the commands for the command line.
  // The first argument is the alias you type in the command line. The second
  // argument is the name of the function that the command will jump to.
  
  chibiCmdAdd("getsaddr", cmdGetShortAddr);  // set the short address of the node
  chibiCmdAdd("setsaddr", cmdSetShortAddr);  // get the short address of the node
  chibiCmdAdd("send", cmdSend);   // send the string typed into the command line
}

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()
{
  // This function checks the command line to see if anything new was typed.
  chibiCmdPoll();
  
  // Check if any data was received from the radio. If so, then handle it.
  if (chibiDataRcvd() == true)
  { 
    int len, rssi, src_addr;
    byte buf[100];  // this is where we store the received data
    
    // retrieve the data and the signal strength
    len = chibiGetData(buf);

    // discard the data if the length is 0. that means its a duplicate packet
    if (len == 0) return;    

    rssi = chibiGetRSSI();
    src_addr = chibiGetSrcAddr();
    
    // Print out the message and the signal strength
    Serial.print("Message received from node 0x");
    Serial.print(src_addr, HEX);
    Serial.print(": "); 
    Serial.print((char *)buf); 
    Serial.print(", RSSI = 0x"); Serial.println(rssi, HEX);
  }
}

/**************************************************************************/
// USER FUNCTIONS
/**************************************************************************/

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

/**************************************************************************/
/*!
    Transmit data to another node wirelessly using Chibi stack. Currently
    only handles ASCII string payload
    Usage: send <addr> <string...>
*/
/**************************************************************************/
void cmdSend(int arg_cnt, char **args)
{
    byte data[100];
    int addr, len;

    // convert cmd line string to integer with specified base
    addr = chibiCmdStr2Num(args[1], 16);
    
    // concatenate strings typed into the command line and send it to
    // the specified address
    len = strCat((char *)data, 2, arg_cnt, args);    
    chibiTx(addr, data,len);
}

/**************************************************************************/
/*!
    Concatenate multiple strings from the command line starting from the
    given index into one long string separated by spaces.
*/
/**************************************************************************/
int strCat(char *buf, unsigned char index, char arg_cnt, char **args)
{
    uint8_t i, len;
    char *data_ptr;

    data_ptr = buf;
    for (i=0; i<arg_cnt - index; i++)
    {
        len = strlen(args[i+index]);
        strcpy((char *)data_ptr, (char *)args[i+index]);
        data_ptr += len;
        *data_ptr++ = ' ';
    }
    *data_ptr++ = '\0';

    return data_ptr - buf;
}

