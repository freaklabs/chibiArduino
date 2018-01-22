/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.

*******************************************************************/
/*!
    \file 
    \ingroup
 
    This is the user configurable file. It contains parameters that users
    can change to suit their needs. 

*/
/**************************************************************************/
#ifndef CHB_USR_CFG_H
#define CHB_USR_CFG_H

/**************************************************************************/
/*!
    Specify the board type
*/
/**************************************************************************/

// Freakduino Series Boards
#if ((FREAKDUINO == 1) || (FREAKDUINO_LONG_RANGE == 1))
    #warning "ChibiArduino Notification: Freakduino Standard, 5.0V, 8 MHz, ATMega328P board selected"
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINB
    #define CHB_INTP_PIN        6
// FreakUSB Series Boards
#elif ((FREAKUSB == 1) || (FREAKUSBLR == 1))
    #warning "ChibiArduino Notification: FreakUSB, 3.3V, 8 MHz, ATMega328P board selected"
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINB
    #define CHB_INTP_PIN        6
// Illuminado Receiver Boards
#elif (ILLUMINADO_RX == 1)
    #warning "ChibiArduino Notification: Illuminado Receiver, 3.3V, 8 MHz, ATMega328P board selected"
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINB
    #define CHB_INTP_PIN        6
// Illuminado Transmitter Boards
#elif (ILLUMINADO_TX == 1)
    #warning "ChibiArduino Notification: Illuminado Transmitter, 5.0V, 16 MHz, ATMega328P board selected"
    #define USE_PINCHANGE_INTP  0
    #define CHB_INTP_PORT       PIND
    #define CHB_INTP_PIN        3
// Saboten Series Boards
#elif (SABOTEN == 1)
    #warning "ChibiArduino Notification: Saboten, 3.3V, 8 MHz, ATMega1284P board selected" 
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINA
    #define CHB_INTP_PIN        6
// FREAKDUINO1284P Series Boards
#elif ((FREAKDUINO1284P == 1) || (FREAKDUINO1284PLR == 1))
    #warning "ChibiArduino Notification: FREAKDUINO1284P, 5V, 8 MHz, ATMega1284P board selected" 
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINA
    #define CHB_INTP_PIN        6
// FREAKUSB1284P boards
#elif ((FREAKUSB1284P == 1) || (FREAKUSB1284PLR == 1))
    #warning "ChibiArduino Notification: FREAKUSB1284P, 3.3V, 8 MHz, ATMega1284P board selected" 
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINA
    #define CHB_INTP_PIN        6
// Arashi Ethernet Gateway Series Boards
#elif ((ARASHI_ENET_GATEWAY == 1) || (ARASHI_ENET_GATEWAY_LR == 1))
    #warning "ChibiArduino Notification: Arashi Ethernet Gateway, 3.3V, 8 MHz, ATMega1284P selected"
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINA
    #define CHB_INTP_PIN        6
#else
    #warning "ChibiArduino Notification: No board selected. Defaulting to Freakduino Standard."
    #define USE_PINCHANGE_INTP  1
    #define CHB_INTP_PORT       PINB
    #define CHB_INTP_PIN        6
#endif

/**************************************************************************/
/*!
    Enable the chibiArduino stack to run in promiscuous mode. This should
    only be used to analyze raw packet frames, like for wireshark. 
*/
/**************************************************************************/
#define CHIBI_PROMISCUOUS 0
#if (CHIBI_PROMISCUOUS == 1)
    #warning "CHIBI PROMISCUOUS MODE ENABLED"
#endif

/**************************************************************************/
/*!
    Max payload determines the largest size that can be transmitted in a single frame.
    If a frame is transmitted that's greater than the max payload, then it
    will be split up into the max payload size and transmitted over multiple frames.

        Integer, byte(s); Range: 1 - 116; Default: 100
        NOTE values over 100 may not work. Please see the documentation.
    
*/
/**************************************************************************/
#define CHB_MAX_PAYLOAD   100

/**************************************************************************/
/*!
    This is the position in EEPROM where the 64-bit IEEE address is stored.
    It takes up 8 byte positions in the EEPROM. 

        HEX, EEPROM location; Range: 0x00 - 0x01F8; Default: 0x00
*/
/**************************************************************************/
#define CHB_EEPROM_IEEE_ADDR    0x00

/**************************************************************************/
/*!
    This is the position in the EEPROM where the 16-bit short address is
    stored. It takes up 2 bytes in the EEPROM.
    
        HEX, EEPROM location; Range: 0x00 - 0x1FE; Default 0x09
*/
/**************************************************************************/
#define CHB_EEPROM_SHORT_ADDR   0x09

/**************************************************************************/
/*!
    This is where the SLP_TR pin is defined. The PORT, Data Direction register
    and the position is needed to fully define it.
    
        CHB_SLPTR_PORT default PORTC on the chibiduino
        CHB_SLPTR_DDIR default DDRC on the chibiduino
        CHB_SLPTR_PIN default 2 on the chibiduino.
*/
/**************************************************************************/
#if ((FREAKDUINO1284P == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284P == 1) || (FREAKUSB1284PLR == 1) || (SABOTEN == 1))
    #define CHB_SLPTR_PORT       PORTA
    #define CHB_SLPTR_DDIR       DDRA
    #define CHB_SLPTR_PIN        5
#else
    #define CHB_SLPTR_PORT       PORTC
    #define CHB_SLPTR_DDIR       DDRC
    #define CHB_SLPTR_PIN        2
#endif

/**************************************************************************/
/*!
    This is where the SPI Chip Select pin is defined. The PORT, Data Direction register
    and the position is needed to fully define it.
    
        CHB_SPI_CS_PORT default PORTC on the chibiduino
        CHB_SPI_CS_DDIR default DDRC on the chibiduino
        CHB_SPI_CS_PIN default 3 on the chibiduino.
*/
/**************************************************************************/
#if ((FREAKDUINO1284P == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284P == 1) || (FREAKUSB1284PLR == 1) || (SABOTEN == 1))
    #define CHB_SPI_CS_PORT PORTC
    #define CHB_SPI_CS_DDIR DDRC
    #define CHB_SPI_CS_PIN  7                 // PC.3 - SPI Chip Select (SSEL)
#else
    #define CHB_SPI_CS_PORT PORTC
    #define CHB_SPI_CS_DDIR DDRC
    #define CHB_SPI_CS_PIN  3                 // PC.3 - SPI Chip Select (SSEL)
#endif

/**************************************************************************/
/*!
    This is where the IRQ vector is defined. The IRQ vector will be based
    on the interrupt being used for the radio.
    
        CHB_RADIO_IRQ default PCINT0_vect on the chibiduino
*/
/**************************************************************************/
#if (USE_PINCHANGE_INTP == 1)
    #define CHB_RADIO_IRQ       PCINT0_vect
#else
    #define CHB_RADIO_IRQ       INT1_vect
#endif

/**************************************************************************/    
/*!
    This is where the interrupt configuration code is. This may be different
    based on the chip or type of interrupt being used. 
*/
/**************************************************************************/    
// enable rising edge interrupt on IRQ0
#if (USE_PINCHANGE_INTP == 1)
    #define CFG_CHB_INTP() do               \
                {                           \
                    PCMSK0 |= _BV(PCINT6);  \
                    PCICR |= _BV(PCIE0);    \
                }                           \
                while(0)
#else
    #define CFG_CHB_INTP() do                           \
                {                                       \
                    EICRA |= _BV(ISC10) | _BV(ISC11);   \
                    EIMSK |= _BV(INT1);                 \
                }                                       \
                while (0)
#endif

/**************************************************************************/
/*!
    This is the code to enable and disable the interrupts on the MCU side.
    This is only used when we're in RX_POLLING_MODE where the interrupt needs
    to be turned off on the MCU side until the data can be retrieved. It also
    is a workaround for a nasty bug on the Wiznet W5100 chips where the
    SPI interface is not released properly. Hence, the interrupts are turned
    off until the SPI bus is free and the data can be retrieved without collision.
*/
/**************************************************************************/
#if (USE_PINCHANGE_INTP == 1)
    #define CHB_IRQ_DISABLE() do {PCMSK0 &= ~_BV(PCINT6);} while(0)
    #define CHB_IRQ_ENABLE() do {PCMSK0 |= _BV(PCINT6);} while(0)
#else
    #define CHB_IRQ_DISABLE() do {EIMSK &= ~_BV(INT1);} while(0)
    #define CHB_IRQ_ENABLE() do {EIMSK |= _BV(INT1);} while(0)
#endif

/**************************************************************************/
/*!
    The default channel for the radio to start on.
 
    For 802.15.4 in the 868/915 MHz band, the channels go from 0 to 10. Channel
    0 is 868.3 MHz and is the only channel in the 900 MHz band that can be used
    license free in Europe. Channels 1 through 10 are in the 915 MHz band and
    can be used license free in most of North America. These channels require a
    868/915 MHz radio.
 
            Channel     Frequency (MHz)
            0           868.3
            1           906
            2           908
            3           910
            4           912
            5           914
            6           916
            7           918
            8           920
            9           922
            10          924
 
    For 802.15.4 in the 2.4 GHz band, the channels go from 11 to 26. Channels that
    don't conflict with 802.11 (Wi-Fi) are channels 15, 20, and 26. These channels are
    license free worldwide. They also require a 2.4 GHz radio.
 
            Channel     Frequency (MHz)
            11          2405
            12          2410
            13          2415
            14          2420
            15          2425
            16          2430
            17          2435
            18          2440
            19          2445
            20          2450
            21          2455
            22          2460
            23          2465
            24          2470
            25          2475
            26          2480
*/
/**************************************************************************/
#define CHB_2_4GHZ_DEFAULT_CHANNEL    11
#define CHB_900MHZ_DEFAULT_CHANNEL     1


/**************************************************************************/
/*!
    This is the default modulation mode for the 900 MHz boards using the
    AT86RF231. 
                    
    Other modulations that are available
    #define  CHB_2_4GHZ_INIT_MODE OQPSK_2000        // proprietary
    #define  CHB_2_4GHZ_INIT_MODE OQPSK_1000        // proprietary
    #define  CHB_2_4GHZ_INIT_MODE OQPSK_500         // proprietary
    #define  CHB_2_4GHZ_INIT_MODE OQPSK_250         // 802.15.4-2006 compliant

    This is the default modulation mode for the 900 MHz boards using the
    AT86RF212. 
					
	Other modulations that are available
	#define  CHB_900MHZ_INIT_MODE OQPSK_SINRC_100    // 802.15.4-2006 compliant
	#define  CHB_900MHZ_INIT_MODE OQPSK_SIN_250      // 802.15.4-2006 compliant
    #define  CHB_900MHZ_INIT_MODE OQPSK_RC_250       
    #define  CHB_900MHZ_INIT_MODE OQPSK_SIN_500
    #define  CHB_900MHZ_INIT_MODE OQPSK_SIN_1000
    #define  CHB_900MHZ_INIT_MODE BPSK_40
	#define  CHB_900MHZ_INIT_MODE BPSK_20
*/
/**************************************************************************/
#define CHB_2_4GHZ_INIT_MODE OQPSK_250
#define CHB_900MHZ_INIT_MODE OQPSK_SIN_250    

/**************************************************************************/
/*!
    This is the default PAN ID (network ID) of all devices on the network.
    Only devices sharing the same PAN ID can communicate with each other,
    unless the radio is set to promiscuous mode. The PAN ID is a 16-bit value.
    
        HEX; Range: 0x0000 - 0xFFFF; Default: 0x1234
*/
/**************************************************************************/
#define CHB_PAN_ID  0x1234

/**************************************************************************/
/*!
    This is the value that gets written into the radio's register. Each value
    corresponds to a different transmit power in dBm. The mapping is as follows:
 
    2.4 GHz             915 MHz N America   EU1     EU2
    dBm     Value        dBm    Value                   
    3.0     0x0          10     0xE1           
    2.8     0x1          9      0xA1           
    2.3     0x2          8      0x81           
    1.8     0x3          7      0x82           
    1.3     0x4          6      0x83           
    0.7     0x5          5      0x84                0xE8
    0.0     0x6          4      0x85        0x62    0xE9
    -1      0x7          3      0x42        0x63    0xEA
    -2      0x8          2      0x22        0x64    0xEB
    -3      0x9          1      0x23        0x65    0xAB
    -4      0xA          0      0x24        0x66    0xAC
    -5      0xB          -1     0x25        0x47    0xAD
    -7      0xC          -2     0x04        0x48    0x48
    -9      0xD          -3     0x05        0x28    0x28
    -12     0xE          -4     0x06        0x29    0x29
    -17     0xF          -5     0x07        0x2A    0x2A
                         -6     0x08        0x08    0x08
                         -7     0x09        0x09    0x09
                         -8     0x0A        0x0A    0x0A
                         -9     0x0B        0x0B    0x0B
                         -10    0x0C        0x0C    0x0C
                         -11    0x0D        0x0D    0x0D
 
*/
/**************************************************************************/
#define CHB_2_4GHZ_TX_PWR  0x0
#define CHB_900MHZ_TX_PWR  0xE1

/**************************************************************************/
/*!
    This is the number of times the radio will auto-retry a transmission.
    The auto-retry is triggered when an ACK isn't received within the ACK
    timeout period. This is typically ~1 msec. 
    
        Integer, count; Range: 0 - 15; Default: 3
*/
/**************************************************************************/
#define CHB_MAX_FRAME_RETRIES   3

/**************************************************************************/
/*!
    This is the number of times the radio will attempt a transmission when
    the channel is busy. The radio first checks if the channel is clear.
    If it's occupied by another node, then it will back-off for a random
    period and try again. If it exceeds the MAX CSMA RETRIES, it will assume
    the channel is blocked and return an error status.
    
        Integer, count; Range: 0 - 9; Default: 4
*/
/**************************************************************************/
#define CHB_MAX_CSMA_RETRIES    4

/**************************************************************************/
/*!
    This is the minimum backoff exponent. When the channel is busy, the radio
    will wait at least 2^MIN_BE symbol periods before another attempt at
    transmission.
    
        Integer, exponent; Range: 0 - 3; Default: 0
*/
/**************************************************************************/
#define CHB_MIN_BE  0

/**************************************************************************/
/*!
    This is the clear channel assessment mode used to determine whether a
    channel is clear or not. There are generally two ways to assess a busy
    channel. One is to do an energy detection by sampling the energy level
    of the channel. If its below the threshold, then its clear. The other
    way is to check for a 2.4 GHz carrier signal.
 
        Value       Mode
        0           Reserved
        1           Mode 1, Energy above threshold      (Default)
        2           Mode 2, Carrier sense only
        3           Mode 3, Carrier sense with energy above threshold 
*/
/**************************************************************************/
#define CHB_CCA_MODE    0x1

/**************************************************************************/
/*!
    This is the energy detection threshold for the clear channel assessment.
    A channel is considered busy when the energy in the channel is:
    RSSI_BASE_VAL + 2 CCA_ED_THRES [dBm]
 
    where RSSI_BASE_VAL = -91 dBm for the AT86RF230
    
        Integer, byte; Range: 0x01 - 0x0F; Default: 0x07
*/
/**************************************************************************/
#define CHB_CCA_ED_THRES    0x7

#endif
