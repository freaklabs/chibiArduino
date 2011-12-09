/* Chibi for Arduino, Example 2
This is the same Hello World example except that both 
transmit and receive handling is included. 
*/

#include <chibi.h>

byte msg[] = "Hello World";

void setup()
{
  Serial.begin(57600);
  chibiInit();
}

void loop()
{ 
  // We're going to add 1 to the message length to handle the 
  // terminating character '/0'. We're also sending a broadcast so 
  // any node in listening range will hear the message. 
  chibiTx(BROADCAST_ADDR, msg, 12);

  // if any data is received, then print it to the terminal
  if (chibiDataRcvd() == true)
  {
    byte buf[CHB_MAX_PAYLOAD];  // store the received data in here
    
    chibiGetData(buf);
    
    // The data consists of ASCII characters in byte (unsigned char) format. The print
    // function requires ASCII characters be in char format so we need to inform the function
    // that its okay to convert the format from unsigned char to char. 
    Serial.print((char *)buf);
  }

  // delay half a second between transmission
  delay(500);
}
