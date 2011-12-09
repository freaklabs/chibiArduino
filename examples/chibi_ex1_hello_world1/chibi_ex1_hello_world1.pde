/* Chibi for Arduino, Example 1
This is a very basic sketch that shows how to transmit data. The data being
transmitted is the standard "hello world" message. The destination address will
be a broadcast address so any node within listening range will hear it. There is
also a delay of 500 msec between each transmission. 
*/

#include <chibi.h>

byte msg[] = "Hello World";

void setup()
{
  chibiInit();
}

void loop()
{ 
  // We're going to add 1 to the message length to handle the terminating null 
  // character for a string '/0'.
  chibiTx(BROADCAST_ADDR, msg, 12);

  // delay half a second between transmission
  delay(500);
}
