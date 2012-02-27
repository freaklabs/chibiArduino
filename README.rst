======================================
chibiArduino - Wireless Protocol Stack
======================================

HOWTO: Using the chibiArduino Wireless Protocol Stack v0.5

Document Revision History
=========================

========== ======================
Date
========== ======================
2011-11-01 v0.5 Document creation
2012-02-27 ReST Version
========== ======================


Introduction
============

The chibiArduino protocol stack is a port of the Chibi 802.15.4 wireless stack to the Arduino platform. The stack was designed to be simple, easy to use, and have a very small memory foot- print. As of this writing, the stack footprint is approximately 300 bytes of RAM and 3.8 kB of flash.

Chibi was created as a tool to introduce people to wireless communications and sensor network- ing. There are a number of protocols for wireless communications and sensor networking but most of them are complex with sophisticated features. In contrast, many people in the hobbyist and hacker communities just want something to send and receive small amounts of data without having to understand a lot of protocol complexity.

For simplicity, chibi relies mainly on three functions: init, send, and receive. There are also other functions available for management or configuration, however with just the basic initialization function, the default settings can allow people to start communicating wirelessly.

Traditionally, wireless communication protocol stacks also require an operating system, operating system-like services, or a complex state machine to manage connections. However since the radio hardware in many wireless ICs now include functionality to handle things like automatic retries and timeouts, these can be offloaded to hardware and simple communications can be performed without he need for an operating system or a complex state machine. This greatly simplifies the protocol stack implementation and also decreases the amount of system resources such as flash and RAM used to implement the communications.

The chibiArduino software also integrates a command line which can be optionally used inside an Arduino sketch. User commands can be integrated into the command line to call user functions interactively from a terminal. This is useful for configuring the node such as setting the address or channel. It’s also a useful tool for controlling the data that is being sent and seeing the data that’s being received from the node.

Operation
=========

chibiArduino usage is fairly straightforward and just consists of initializing/configuring the node and setting up when the sending and receiving will take place.

--------------
Initialization
--------------

The first thing that needs to be done is to initialize the node. In a user sketch, it looks something like this:

::

  void setup()
  {
      chbInit();
  }

This would set up the node with the default address, default network ID, and default channel. More sophisticated configuration can also be performed:

::

  void setup()
  {
      chbInit();
      chibiSetShortAddr(0xAAAA);
      chibiSetChannel(15);
  }


This would set the node up, change the network address to 0xAAAA (the address used to uniquely identify the node), and set the channel to 15. Incidentally, chibiArduino uses the IEEE 802.15.4 specification which allocates 16 channels in the 2.4 GHz spectrum. These channels are defined as channels 11 to 26 so the channel needs to be within this range.

------------
Transmitting
------------

Transmitting data requires three things: a destination address to transmit the data to, the actual data, and the length of the data. Performing a simple “hello world” transmission would go some- thing like this:

::

  void loop()
  {
      byte msg[] = “Hello World”;
      // 1 byte added to length for terminating character
      chibiTx(0xAAAA, msg, 12);
  }

You’ll notice that the length of the Hello World message is 11 bytes but an extra byte was added to the transmission length. This is because strings contain a trailing NULL character to terminate the string. It’s just one of those quirky things in programming.

In the example above, the node would transmit “Hello World” to the node whose address is 0xAAAA. The length informs the receiving end of how much data to expect. It’s also important to keep the messages under the maximum payload size for an 802.15.4 frame. For the chibiArduino stack, the maximum payload size is 116 bytes but its set to 100 bytes in the user configuration parameters just to maintain a margin.

Another example of wireless transmission would be something like this:

::

  void loop()
  {
      byte data[6];
      readAccelerometer(data);
      chibiTx(CHIBI_BROADCAST_ADDR, data, 6);
      delay(500);
  }

This code reads data from an accelerometer, broadcasts it to all nodes in the same network, and then waits for 500 msec before repeating the process. CHIBI_BROADCAST_ADDR is defined inside the chibiArduino stack and contains the value of 0xFFFF. This is the reserved address de- fined by the 802.15.4 specification for broadcast transmissions and all nodes on the same network will receive transmissions to this address.

---------
Receiving
---------

Receiving data is a little bit more complicated than transmitting data and that’s because there is little control over when data is received. Data can come in at any time and the main loop needs to check if any new data arrives and handle it. Here’s an example of how to do it:

::

  void loop()
  {
      byte data[100];
      if (chibiDataRcvd() == true)
      {
          chibiGetData(data);
      }
  }

In the above example, a byte array was created to store the received data. The “chibiDataRcvd()” function is used and will return true if new data has arrived. If there is data available, then the chibiGetData() function will retrieve the data and store it in the byte array.

A better way to handle receiving data would be like this:

::

  void loop()
  {
      byte len, data[CHIBI_MAX_PAYLOAD];
      if (chibiDataRcvd() == true)
      {
          len = chibiGetData(data);
      }
  }

In this case, there were two variables created. The “len” variable is used to store the length of the received data in bytes. When the chibiGetData() function is called, it will store the received data in the specified array and also return the length. The length can then be used to loop through the data array, check it for validity, or whatever else the application might require.


Command Line Operation
======================

The cmdArduino library has been integrated into the chibiArduino communications stack because it makes many things very convenient. Having an interactive command line makes things like set- ting different network addresses for each node very simple. It also gives the user control over when to send data and provides visual feedback on what kind of data arrived. Since the command line is configurable to call user functions, anything you might want to do such as reading the radio registers, MCU registers, toggling an I/O, or whatever else might want to be done is also possible interactively.

For a detailed tutorial on how to use the command line, please refer to the cmdArduino tuto- rial on the FreakLabs website. All the functions are the same except they are prefixed with
“chibi”. For example, “cmdInit()” in the tutorial becomes “chibiCmdInit()” when accessed through the chibiArduino stack.

