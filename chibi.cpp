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


*/
/**************************************************************************/
#include <avr/pgmspace.h>
#include "chibi.h"

// Including the actual "c" files rather than the headers. The Arduino lib only
// recognizes one source file in the directory so all the source files need to look like 
// they're in the one file.
#include "src/chb.c"
#include "src/chb_buf.c"
#include "src/chb_drvr.c"
#include "src/chb_spi.c"
#include "src/chb_eeprom.c"
#include "src/chb_cmd.c"
#include "src/chb_aes.c"

#if (CHB_RX_POLLING_MODE)
   #include "src/chb_rx_poll.c"
#else
   #include "src/chb_rx_int.c"
#endif

// used to store info about received data
static chb_rx_data_t rx_data;

/**************************************************************************/
/*!
    Init the chibi stack
*/
/**************************************************************************/
void chibiInit()
{
    chb_init();
}

/**************************************************************************/
/*!
    Set the short address of the wireless node. This is the 16-bit "nickname" 
    of your node and what will be used to identify it. 
*/
/**************************************************************************/
void chibiSetShortAddr(uint16_t addr)
{
    chb_set_short_addr(addr);
}

/**************************************************************************/
/*!
    Retrieve the short address of the node.
*/
/**************************************************************************/
uint16_t chibiGetShortAddr()
{
    return chb_get_short_addr();
}

/**************************************************************************/
/*!
    Sets the 64-bit IEEE address of the node.
*/
/**************************************************************************/
void chibiSetIEEEAddr(uint8_t *ieee_addr)
{
    chb_set_ieee_addr((uint8_t *)ieee_addr);
}

/**************************************************************************/
/*!
    Retrieves the 64-bit IEEE address of the node.
*/
/**************************************************************************/
void chibiGetIEEEAddr(uint8_t *ieee_addr)
{
    chb_get_ieee_addr((uint8_t *)ieee_addr);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t chibiRegRead(uint8_t addr)
{
    return chb_reg_read(addr);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chibiRegWrite(uint8_t addr, uint8_t val)
{
    chb_reg_write(addr, val);
}

/**************************************************************************/
/*!
    Transmit data to another node wirelessly using Chibi stack.
    Usage: send <addr> <char data array...>
*/
/**************************************************************************/
uint8_t chibiTx(uint16_t addr, uint8_t *data, uint8_t len)
{
    return chb_write(addr, (uint8_t *)data, len);
}

/**************************************************************************/
/*!
    This function should be polled in the "loop" portion of the code. It will
    return false if no data has been received or true if data has arrived.
*/
/**************************************************************************/
uint8_t chibiDataRcvd()
{
    pcb_t *pcb = chb_get_pcb();

#if (CHB_RX_POLLING_MODE)
    // need to poll for received data if we're not retrieving the data 
    // in the ISR
    chb_rcv_poll();
#endif

    return pcb->data_rcv;
}

/**************************************************************************/
/*!
    Retrieves the data from the frame. A char array needs to be passed into
    the function and the data will be copied into it. The character array
    should be at least the size of a maximum sized frame for safety. It can
    be smaller to save RAM but this should be done carefully to avoid buffer
    overflows.
*/
/**************************************************************************/
uint8_t chibiGetData(uint8_t *data)
{
    // point the data buffer to the buffer that was passed in from arduino ide
    rx_data.data = (U8 *)data;

    // load the data into the buffer and get the length
    rx_data.len = chb_read(&rx_data);
    return rx_data.len;
}

/**************************************************************************/
/*!
    Returns the signal strength recorded from the last received frame.
*/
/**************************************************************************/
uint8_t chibiGetRSSI()
{
    pcb_t *pcb = chb_get_pcb();
    return pcb->ed;
}

/**************************************************************************/
/*!
    Get the source address of the most recently received frame
*/
/**************************************************************************/
uint16_t chibiGetSrcAddr()
{
    return rx_data.src_addr;
}

/**************************************************************************/
/*!
    Puts the radio to sleep to save power. It cuts approximately 15 mA of
    current from the system.
*/
/**************************************************************************/
void chibiSleepRadio(uint8_t enb)
{
    chb_sleep(enb);
}

/**************************************************************************/
/*!
    Sets the channel for the radio. Available channels are channels 11 to 26.
*/
/**************************************************************************/
uint8_t chibiSetChannel(uint8_t channel)
{
    return chb_set_channel(channel);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t chibiGetPartID()
{
    return chb_get_part_num();
}

/**************************************************************************/
/*!
    Gets the current channel for the radio. 
*/
/**************************************************************************/
uint8_t chibiGetChannel()
{
    return chb_get_channel();
}

/**************************************************************************/
/*!
    Initialize the command line interface
*/
/**************************************************************************/
void chibiCmdInit(uint32_t speed)
{
    chb_cmd_init(speed);
}

/**************************************************************************/
/*!
    Poll the command line interface. This should be in the loop portion of
    the code and will check for any typed characters. If any typed characters
    are found, they will be parsed and checked for a match with commands
    in the command table.
*/
/**************************************************************************/
void chibiCmdPoll()
{
    chb_cmd_poll();
}

/**************************************************************************/
/*!
    Add a command to the command table in the command line interface.
*/
/**************************************************************************/
void chibiCmdAdd(char *name, void (*func)(int argc, char **argv))
{
    chb_cmd_add(name, func);

}

/**************************************************************************/
/*!
    This function performs converts an ASCII number string into the actual number.
    The first argument is the string to be converted, the second argument is
    the base that it should use for the conversion (ie: decimal = 10, hexadecimal = 16).
*/
/**************************************************************************/
uint32_t chibiCmdStr2Num(char *str, uint8_t base)
{
    return chb_cmd_str2num(str, base);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chibiAesInit(uint8_t *key)
{
    chb_aes_init(key);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t chibiAesEncrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext)
{
    return chb_aes_encrypt(AES_ECB, len, plaintext, ciphertext);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t chibiAesDecrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext)
{
    return chb_aes_decrypt(AES_ECB, len, plaintext, ciphertext);
}

/**************************************************************************/
/*!
    Set the data rate of the IC. Here are the values to use:

    AT86RF230: not supported
    AT86RF231, all channels: 0              = 250 kbps
    AT86RF231, all channels: 1              = 500 kbps
    AT86RF231, all channels: 2              = 1000 kbps
    AT86RF231, all channels: 3              = 2000 kbps
    AT86RF212, channel 0 (868 MHz) : 0      = 100 kbps    
    AT86RF212, channel 0 (868 MHz) : 1      = 200 kbps    
    AT86RF212, channel 0 (868 MHz) : 2      = 400 kbps
    AT86RF212, channel 1-10(916 MHz) : 0    = 250 kbps    
    AT86RF212, channel 1-10(916 MHz) : 1    = 500 kbps    
    AT86RF212, channel 1-10(916 MHz) : 2    = 1000 kbps    
*/
/**************************************************************************/
void chibiSetDataRate(uint8_t rate)
{
    chb_set_datarate(rate);
}

/**************************************************************************/
/*!
    Get a true random value from the radio. Only supported by the AT86RF231
    and the AT86RF212.
*/
/**************************************************************************/
uint8_t chibiGetRand()
{
    return chb_get_rand();
}

/**************************************************************************/
/*!
    Set the modulation mode for the radio
*/
/**************************************************************************/
void chibiSetMode(uint8_t mode)
{
    chb_set_mode(mode);
}

/**************************************************************************/
/*!
    Return the length of bytes remaining in the receive buffer
*/
/**************************************************************************/
uint16_t chibiBufGetRemaining()
{
    return chb_buf_get_remaining(); 
}

/**************************************************************************/
/*!
    
*/
/**************************************************************************/
void chibiSetRetries(uint8_t retries)
{
    chb_set_retries(retries);
}

/**************************************************************************/
/*!
    
*/
/**************************************************************************/
void chibiAesTest(uint8_t *key)
{
    chb_aes_test(key);
}

#if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (ARASHI_ENET_GATEWAY_LR == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1)) 
    /**************************************************************************/
    /*!
        
    */
    /**************************************************************************/
    void chibiHighGainModeEnable()
    {
        chb_high_gain_mode_enable();
    }

    /**************************************************************************/
    /*!
        
    */
    /**************************************************************************/
    void chibiHighGainModeDisable()
    {
        chb_high_gain_mode_disable();
    }
#endif
