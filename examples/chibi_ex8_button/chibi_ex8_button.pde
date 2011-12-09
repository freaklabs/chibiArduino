/************************************************************/
/*
   Chibi Wireless Button
   This is an example of controlling an LED wirelessly. A button 
   is connected to one node and an LED is connected to another. 
   Actually this code allows each node to have both so both nodes can control 
   each other's buttons.
  
  Directions for use:
  1. Set the DEST_ADDR definition to the destination address of the node
  you want to control.
  2. Set the LED_PIN definition to the pin that has the LED (if any).
  3. Set the BUTTON_PIN definition to the pin that has the button. Remember
  to use a pullup or pulldown on the button output so that it won't float in
  its idle state. 
  4. Set the BUTTON_DOWN and BUTTON_UP definition to match your button configuration
  5. Upload this sketch to each board.
  
  Good luck!  
*/
/************************************************************/
#include <chibi.h>

// These are general definitions. There are three main things to define
// in this sketch: where the LED is at, where the button is at, and where
// the data will be sent. 
#define DEST_ADDR 3    // the destination address data will be sent to
#define LED_PIN 9      // the pin that will control the LED
#define BUTTON_PIN 8   // the pin that will detect the button

// These two definitions will change based on the user hardware. If
// the idle (UP) position of the button is 0, then set BUTTON_UP to 0 and
// BUTTON_DOWN to 1. And vice-versa
#define BUTTON_DOWN 0  // the value that a button press generates on button pin
#define BUTTON_UP 1    // the value that a button release generates on button pin

// These are the commands. More commands can be added based on the user inputs.
// Then they can be handled in the command handler function. 
#define CMD_LED_OFF 0      // turn the LED off
#define CMD_LED_ON 1       // turn the LED on

// Store the previous state of the button here. We'll use it to compare against
// the current button state to see if any event occurred.
byte button_state;

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()  
{ 
  // init the hardware
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  // init the wireless stack
  chibiInit();
  
  // init the serial port for debug purposes
  Serial.begin(57600);

  // initial state of button is set to be unpushed
  button_state = BUTTON_UP;
} 

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()  
{   
  button_handler();
  command_handler();
}

/**************************************************************************/
/*!
   This function handles any button events. When the button changes state, like
   if it gets pressed or released, then this function will detect it and send
   out a command to change the LED on the remote node.
*/
/**************************************************************************/
void button_handler()
{
  byte button_val, data[1];
  
  // get the button value from the input
  button_val = digitalRead(BUTTON_PIN);  
  
  // check if the button has changed state. compare the current button value
  // with the previous value. if its the same, then ignore. if not, then handle 
  // the event.
  if (button_val != button_state)
  {
    if (button_val == BUTTON_DOWN)
    {
      //the button has been pushed. Send out a command to turn the LED on. then
      // update the button's state for future comparison  
      button_state = BUTTON_DOWN;  
      data[0] = CMD_LED_ON;
      Serial.println("Button is down.");
    }
    else
    {
      // the button has been released. Send out a command to turn the LED off. then
      // update the button's state for future comparison  
      button_state = BUTTON_UP;
      data[0] = CMD_LED_OFF;
      Serial.println("Button is up.");
    }
    chibiTx(DEST_ADDR, data, 1);
  } 
}
/**************************************************************************/
/*!
   This is the command handler. I wrote it semi-generically so it can be extended
   to handle other commands. There are two main parts to it. The first part is to
   retrieve any data that arrived wirelessly and extract the command. The second 
   part is to handle the command.
   
   In a real application, there might be multiple buttons or other input devices 
   that can trigger many different commands. In this case, you just need to add 
   each command to the switch statement and the code to handle it. 
*/
/**************************************************************************/
void command_handler()
{
   byte cmd, buf[CHB_MAX_PAYLOAD];
  
  // check to see if any new data has been received. if so, then we need to handle it.
  if (chibiDataRcvd())
  { 
    // retrieve the data from the chibi stack
    chibiGetData(buf); 
    
    // its assumed that the data will be in the first byte of the buffer since we're only
    // sending one byte.
    cmd = buf[0];
    
    // this is the main command handler. it deals with any commands that arrive through the radio.
    switch (cmd)
    {
      case CMD_LED_OFF: 
        digitalWrite(LED_PIN, 0);
        Serial.println("Command: LED OFF");
        break;
        
      case CMD_LED_ON:
        digitalWrite(LED_PIN, 1);
        Serial.println("Command: LED ON");
        break;
    }
  }      
}
