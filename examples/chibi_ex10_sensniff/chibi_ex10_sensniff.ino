/* chibiArduino Wireshark Bridge Using Sensniff, Example 10
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

#define PROTOCOL_VERSION  0x01
#define CMD_FRAME         0x00
#define CMD_CHANNEL       0x01
#define CMD_GET_CHANNEL   0x81
#define CMD_SET_CHANNEL   0x82

enum
{
  STATE_MAGIC,
  STATE_VERSION,
  STATE_COMMAND,
  STATE_GET_CHANNEL,
  STATE_SET_CHANNEL,
  STATE_RESPONSE
}; 

uint8_t state;
uint8_t cnt = 0;
uint16_t magic[] = {0xC1, 0x1F, 0xFE, 0x72};

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()
{  
  state = STATE_MAGIC;
  
  // Remember: If you change the speed here, make sure you change it on the application
  // program as well. 
  Serial.begin(57600);

  // Init the chibi stack
  chibiInit();
}

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()
{
  uint16_t rem;

  // If we receive any data from the serial port, then it's from the application. At the 
  // time of this writing, it was only used to change the channel. That might change in the future.
  if (Serial.available())
  {
    uint8_t dat = Serial.read();
    
    switch (state)
    {
      case STATE_MAGIC:
        // go through the magic numbers. if we see the magic number,
        // then move to the next state
        if (dat == magic[cnt])
        {
          cnt++;
        }
        else
        {
          cnt = 0;
        }

        // we made it through the magic numbers
        if (cnt > 3)
        {
          state = STATE_VERSION;
          cnt = 0;
        }
      break;

      case STATE_VERSION: 
        if (dat == PROTOCOL_VERSION)
        {
          state = STATE_COMMAND;
        }
      break;

      case STATE_COMMAND: 
        if (dat == CMD_GET_CHANNEL)
        {
          // send channel response
          send_channel_resp();

          // clean up
          state = STATE_MAGIC;
          cnt = 0;          
        }
        else if (dat == CMD_SET_CHANNEL)
        {
          state = STATE_SET_CHANNEL;
        }
        else
        {
          state= STATE_MAGIC;
        }
      break;

      case STATE_SET_CHANNEL:
        
        // get len byte and just verify its correct        
        if (dat == 1) 
        {
          // wait for another piece of data to be available
          while (!Serial.available());
          dat = Serial.read();
        }
        else
        {
          state = STATE_MAGIC;
        }

        // get channel to set to
        if ((dat >= 0) && (dat < 27))
        {
          chibiSetChannel(dat);

          // send response
          send_channel_resp();
        }

        // clean up and go back to idle state
        state = STATE_MAGIC;
        cnt = 0;
      break;
    }
  }
  
  // Check if any data was received from the radio. This is what we dump to the 
  // serial port when we're sniffing.
  if (chibiDataRcvd() == true)
  {  
    int len;
    byte buf[CHB_MAX_PAYLOAD]; 
    
    // send the raw data out the serial port in binary format
    len = chibiGetData(buf);

    // send the header for sensniff
    send_hdr(PROTOCOL_VERSION, CMD_FRAME); 

    // length minus 3 bytes for no FCS and len byte
    Serial.write(len-3);

    // discard the first byte (len) since we need to modify the len byte for the CRC
    for (int i=1; i<len; i++)
    {
      Serial.write(buf[i]);
    }    
  }
}
/**************************************************************************/
// Build header for sensniff
/**************************************************************************/
void send_hdr(uint8_t ver, uint8_t cmd)
{

  uint8_t i;
  for (int i=0; i<4; i++)
  {
    Serial.write(magic[i]);
  }

  // version
  Serial.write(ver);

  // command
  Serial.write(cmd);
}

/**************************************************************************/
// Send channel response to  sensniff
/**************************************************************************/
void send_channel_resp()
{
   uint8_t channel = chibiGetChannel();

  // send the header for sensniff
  send_hdr(PROTOCOL_VERSION, CMD_CHANNEL);
  Serial.write(1); // len
  
  Serial.write(channel);
}
