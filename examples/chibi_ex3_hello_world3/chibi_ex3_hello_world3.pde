/* Chibi for Arduino, Example 3
This example is the same as the previous examples except that it 
changes the channel to one that is not occupied by any 802.11 Wi-Fi
device. Its basically just to show how to change the channel.
*/

#include <chibi.h>

byte msg[] = "Hello World";
byte buf[CHB_MAX_PAYLOAD];

void setup()
{
  Serial.begin(57600);
  
  // Init the chibi wireless stack
  chibiInit();
  
  // set the channel to channel 20. channel 20 is out of band of
  // 802.11 wi-fi channels
  chibiSetChannel(20);
}

void loop()
{ 
  // We're going to automatically calculate the length
  // of the message string using strlen(). We add 1 to the
  // length to handle the terminating character '/0'.
  // We're also sending a broadcast so any node in listening
  // range will hear the message. 
  chibiTx(BROADCAST_ADDR, msg, 12);

  // We're going to print anything we receive in case there are others 
  // transmitting.
  if (chibiDataRcvd() == true)
  {
    chibiGetData(buf);
    Serial.println((char *)buf);
  }

  // delay half a second between transmission
  delay(500);
}
