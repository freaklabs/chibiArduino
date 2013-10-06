/******************************************************************/
/* 
  Chibi Web Server
  This is a modification of the original web server file that comes
  in the Arduino Ethernet library. Its purpose is to demonstrate how
  to use the Chibi library with the Arduino Ethernet library.

  We're only checking analog channels 0, 1, 4, and 5. Analog channels
  2 and 3 are being used for the wireless radio.  

  In order to run both libraries at the same time, either 
  CHB_RX_POLLING_MODE needs to be set to 1 in chibiUsrCfg.h or the
  Arduino ethernet code needs to be changed to protect the Wiznet
  chip accesses from interrupts. The reason is that there is a hardware
  bug in the Wiznet W5100 chip that doesn't it allow it to share
  the SPI bus properly. For more info, check out this application note from 
  Wiznet:

  http://bit.ly/bENS6J

  A tutorial on how to do this should be available soon on the FreakLabs site. 

  Directions:
  1. Load two nodes with this software. Put an ethernet shield on one of the nodes.    
  2. Set unique chibi addresses for each node using the "setsaddr" command.
  3. Connect networked ethernet cable to ethernet shield.
  4. Get on PC and set browser to connect to node:
     Ex: if node address is 192.168.0.30, type:
     http://192.168.0.30 in URL bar
  5. Connect to non-ethernet node via serial terminal, ie: Teraterm
  6. Use the "send" command to send a message to the ethernet node:
       send <addr> <message>
       ex: send 1234 hello world i love you
  6. Refresh browser. You should see the message in the browser window.
*/
/******************************************************************/

/*
  Web  Server
 
 A simple web server that shows the value of the analog input pins.
 using an Arduino Wiznet Ethernet shield. 
 
 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 
 created 18 Dec 2009
 by David A. Mellis
 modified 4 Sep 2010
 by Tom Igoe
 
 */

#include <SPI.h>
#include <Ethernet.h>
#include <chibi.h>

/**************************************************************************/
// Globals
/**************************************************************************/

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 192,168,0, 30 };

// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
Server server(80);
static int prev;

// These are for the chibi stack. 
byte buf[CHB_MAX_PAYLOAD];  // hold the received data
byte len;                   // hold the length of the received data
byte rssi;                  // the signal strength of the received data

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()
{
  // Initialize the chibi command line and set the speed to 57600 bps
  chibiCmdInit(57600);
  
  // Add commands to the command table
  chibiCmdAdd("getsaddr", cmdGetShortAddr);  // set the short address of the node
  chibiCmdAdd("setsaddr", cmdSetShortAddr);  // get the short address of the node
  chibiCmdAdd("send", cmdSend);   // send the string typed into the command line
  
  // Initialize the chibi wireless stack
  chibiInit();
  
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
}

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()
{
  // listen for incoming clients
  Client client = server.available();
  if (client) {
 
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        
        // We're adding the chibi receive function here as well as in the
	// main loop. Otherwise, when a client (browser) connects, then its possible
	// that a long time can elapse before we exit this loop. This means
	// that there's a high potential to lose data. 
        chibiRcv();
        
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();

          // output the value of each analog input pin
          client.print("Message received: "); client.print((char *)buf);
          client.print(", RSSI = 0x"); client.println(rssi, HEX);
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
      
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
  
  // Poll the command line to check if any new data has arrived via the
  // serial interface. 
  chibiCmdPoll();

  // Handle any received data on the wireless interface. 
  chibiRcv();

}

/**************************************************************************/
// USER FUNCTIONS
/**************************************************************************/

/**************************************************************************/
/*!
    The Chibi receive function has been moved into this function since we're
    using it in two places. That way, it removes the need to duplicate code
    and is also better in terms of flash usage. 
*/
/**************************************************************************/
void chibiRcv()
{
  if (chibiDataRcvd() == true)
  { 
     len = chibiGetData(buf); 
     
     // if length = 0, then discard packet. that means it's a duplicate packet
     if (len == 0) return;

     rssi = chibiGetRSSI();
     
     // Print out the message and the signal strength
     Serial.print("Message received: "); Serial.print((char *)buf);
     Serial.print(", RSSI = 0x"); Serial.println(rssi, HEX);
  }
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


