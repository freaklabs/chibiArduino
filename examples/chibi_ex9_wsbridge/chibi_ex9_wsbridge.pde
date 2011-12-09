/* chibiArduino Wireshark Bridge, Example 9
This sketch enables promiscuous mode on the Freakduino boards and dumps
the raw 802.15.4 frames out to the serial port. It should be used in 
conjunction with the FreakLabs "wsbridge" application to feed the raw
frames into Wireshark.

Before using this sketch, please go into the chibiUsrCfg.h file and 
enable promiscuous mode. To do this, change the definition:

#define CHIBI_PROMISCUOUS 0
to 
#define CHIBI_PROMISCUOUS 1

When not using promiscuous mode, please disable this setting by changing
it back to 0.
*/

#include <chibi.h>

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()
{  
  // Init the chibi stack
  chibiInit();              
  
  // Open the serial port at specified speed. For sniffing, we want to send data
  // out the serial port as quickly as possible. On windows, 250kbps can be used.
  // On linux, it only seems to like standard speeds up to 115200. To make things
  // compatible on both platforms, I'm keeping the speed at 115200. If you want
  // to boost the speed on a Windows system, use 250000 rather than 115200. 
  // Ex: Serial.begin(250000);
  //
  // Remember: If you change the speed here, make sure you change it on the application
  // program as well. 
  Serial.begin(115200);
}

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()
{
  // Check if any data was received from the radio. If so, then handle it.
  if (chibiDataRcvd() == true)
  {  
    int len;
    byte buf[CHB_MAX_PAYLOAD]; 
    
    // send the raw data out the serial port in binary format
    len = chibiGetData(buf);
    Serial.write(buf, len);
  }
}
