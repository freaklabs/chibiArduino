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

