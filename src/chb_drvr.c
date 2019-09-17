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
#include <avr/pgmspace.h>
#include "chb.h"
#include "chb_drvr.h"
#include "chb_buf.h"
#include "chb_spi.h"
#include "chb_eeprom.h"

// For handling Arduino 1.0 compatibility and backwards compatibility
#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#if (CHB_RX_POLLING_MODE)
   #include "chb_rx_poll.h"
#else
   #include "chb_rx_int.h"
#endif

// do not print out any messages in non-promiscuous mode
#if (CHIBI_PROMISCUOUS)
    const char chb_err_overflow[] PROGMEM = "";
    const char chb_err_init[] PROGMEM = "";
    const char chb_err_not_supported[] PROGMEM = "";
#else
    // store string messages in flash rather than RAM
    const char chb_err_overflow[] PROGMEM = "BUFFER FULL. TOSSING INCOMING DATA\n";
    const char chb_err_init[] PROGMEM = "RADIO NOT INITIALIZED PROPERLY\n";
    const char chb_err_not_supported[] PROGMEM = "FEATURE NOT SUPPORTED";
#endif

// this holds the id of the radio to identify the chip type
static U8 radio_id = 0;

/**************************************************************************/
/*!
    Retrieve the state from the radio's state machine register.
*/
/**************************************************************************/
static U8 chb_get_state()
{
    return chb_reg_read(TRX_STATUS) & 0x1f;
}

/**************************************************************************/
/*!
    Retrieve the status of the last received frame. 
*/
/**************************************************************************/
static U8 chb_get_status()
{
    return chb_reg_read(TRX_STATE) >> CHB_TRAC_STATUS_POS;
}

/**************************************************************************/
/*!
    Perform a delay of the specified amount of microseconds.
*/
/**************************************************************************/
static void chb_delay_us(U16 usec)
{
    delayMicroseconds(usec);
}

/**************************************************************************/
/*!
    Read the radio's registers for the specified address. Interrupts are disabled
    for the duration of the reading to prevent an interrupt from interfering
    with the operation.
*/
/**************************************************************************/
U8 chb_reg_read(U8 addr)
{
    U8 val = 0;

    /* Add the register read command to the register address. */
    addr |= 0x80;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send Register address and read register content.*/
    val = chb_xfer_byte(addr);
    val = chb_xfer_byte(val);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();

    return val;
}

/**************************************************************************/
/*! 
    Read a 16-bit register. This actually does two consecutive 8-bit reads
    of contiguous registers. It's mostly used to retrieve the PAN ID or the
    short address of the node.
*/
/**************************************************************************/
U16 chb_reg_read16(U8 addr)
{
    U8 i;
    U16 val = 0;

    for (i=0; i<2; i++)
    {
        addr |= chb_reg_read(addr + i) << (8 * i);
    }
    return val;
}

/**************************************************************************/
/*!
    Write to the radio's registers at the specified address with the specified
    value.
*/
/**************************************************************************/
void chb_reg_write(U8 addr, U8 val)
{
    /* Add the Register Write command to the address. */
    addr |= 0xC0;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send Register address and write register content.*/
    chb_xfer_byte(addr);
    chb_xfer_byte(val);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!
    Write 16-bits to the radio's registers. This does two consecutive 8-bit
    writes to contiguous memory addresses. 
*/
/**************************************************************************/
void chb_reg_write16(U8 addr, U16 val)
{
    U8 i;

    for (i=0; i<2; i++)
    {
        chb_reg_write(addr + i, val >> (8 * i));
    }
}

/**************************************************************************/
/*!
    Write 64-bits to the radio's registers. This does 8 consecutive 8-bit
    writes to contiguous memory addresses. This is only used to write the
    IEEE address in case it is used. The default for Chibi is just to use
    16-bit addressing.
*/
/**************************************************************************/
void chb_reg_write64(U8 addr, U8 *val)
{
    U8 i;

    for (i=0; i<8; i++)
    {
        chb_reg_write(addr + i, *(val + i));
    }
}

/**************************************************************************/
/*!
    Perform a read/modify/write to a register. This function first reads
    the specified register, then masks off the bits that will be modified.
    It then changes the modified bits and then writes it back to the register.
    It's used to modify registers without changing bits that are outside
    of the mask. 
*/
/**************************************************************************/
void chb_reg_read_mod_write(U8 addr, U8 val, U8 mask)
{
    U8 tmp;

    tmp = chb_reg_read(addr);
    val &= mask;                // mask off stray bits from val
    tmp &= ~mask;               // mask off bits in reg val
    tmp |= val;                 // copy val into reg val
    chb_reg_write(addr, tmp);   // write back to reg
}

/**************************************************************************/
/*!
    Write data to the frame FIFO of the radio. This is where the data to
    be transmitted gets written. The frame data is written in two separate bursts.
    The first burst is to write the frame header and the second burst is
    to write the frame payload. Makes things easier this way.
*/
/**************************************************************************/
void chb_frame_write(U8 *hdr, U8 hdr_len, U8 *data, U8 data_len)
{
    U8 i;

    // dont allow transmission longer than max frame size
    if ((hdr_len + data_len) > 127)
    {
        return;
    }

    // initiate spi transaction
    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE(); 

    // send fifo write command
    chb_xfer_byte(CHB_SPI_CMD_FW);

    // write hdr contents to fifo
    for (i=0; i<hdr_len; i++)
    {
        chb_xfer_byte(*hdr++);
    }

    // write data contents to fifo
    for (i=0; i<data_len; i++)
    {
        chb_xfer_byte(*data++);
    }

    // terminate spi transaction
    CHB_SPI_DISABLE(); 
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!
    Read the data from the frame FIFO. When data arrives at the radio, it will
    get stored in the frame FIFO. The data will get read out from this function
    and stored in the ring buffer. 
*/
/**************************************************************************/
static void chb_frame_read()
{
    U8 i, len, data;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send frame read command and read the length.*/
    chb_xfer_byte(CHB_SPI_CMD_FR);
    len = chb_xfer_byte(0);

    // check the length of the frame to make sure its within the correct size limits
    if ((len >= CHB_MIN_FRAME_LENGTH) && (len <= CHB_MAX_FRAME_LENGTH))
    {
        // check to see if there is room to write the frame in the buffer. if not, then drop it
        if (len < (CHB_BUF_SZ - chb_buf_get_len()))
        {
            chb_buf_write(len);

            for (i=0; i<len; i++)
            {
                data = chb_xfer_byte(0);
                chb_buf_write(data);
            }
        }
        else
        {
            // this frame will overflow the buffer. toss the data and do some housekeeping
            pcb_t *pcb = chb_get_pcb();

            // read out the data and throw it away
            for (i=0; i<len; i++)
            {
                chb_xfer_byte(0);
            }

            // Increment the overflow stat, and print a message.
            pcb->overflow++;
        }
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!
    Get a random number from the on-board physical random number generator 
    from the radio 
*/
/**************************************************************************/
U8 chb_get_rand()
{
    if (radio_id == CHB_AT86RF230)
    {
        // part ID AT86RF230 does not support true random number generation
        char buf[50];
        
        // nothing to be done here. print warning & return.
        strcpy_P(buf, chb_err_not_supported);
        Serial.print(buf);
        return 0;
    }
    else if ((radio_id == CHB_AT86RF212) || (radio_id == CHB_AT86RF231))
    {
        U8 i, rnd, tmp;

        rnd = tmp = 0;
        for (i=0; i<4; i++)
        {
            tmp = chb_reg_read(PHY_RSSI);
            tmp >>= 5;
            tmp &= 0x03;
            rnd |= tmp << (i*2);
            chb_delay_us(5);
        }
        return rnd;
    }
}


/**************************************************************************/
/*!
*/
/**************************************************************************/
uint8_t chb_get_mode()
{
    return chb_reg_read(TRX_CTRL_2);
}

/**************************************************************************/
/*!
    This function is only for the AT86RF212 868/915 MHz chips.
    Set the channel mode, BPSK, OQPSK, etc...
*/
/**************************************************************************/
void chb_set_mode(U8 mode)
{
    if (radio_id == CHB_AT86RF212)
    {
        switch (mode)
        {
        case OQPSK_SINRC_100:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x08, 0x3f);                 // 802.15.4-2006, channel page 2, channel 0 (868 MHz, Europe)
            break;
        case OQPSK_SIN_250:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x0c, 0x3f);                 // 802.15.4-2006, channel page 2, channels 1-10 (915 MHz, US)
            break;
        case OQPSK_RC_250:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x1c, 0x3f);                 // 802.15.4-2006, channel page 5, channel 0-3 (780 MHz, China)
            break;
        case OQPSK_SIN_500:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x0d, 0x3f);                 // proprietary
            break;
        case OQPSK_SIN_1000:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x2e, 0x3f);                 // proprietary
            break;
        case BPSK_40:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x04, 0x3f);                 // 802.15.4-2006, BPSK, 40 kbps
            break;
        case BPSK_20:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x00, 0x3f);                 // 802.15.4-2006, BPSK, 20 kbps
            break;
        }
    }
    else if (radio_id == CHB_AT86RF231)
    {
        switch (mode)
        {
            case OQPSK_2000:            
            chb_reg_read_mod_write(TRX_CTRL_2, 0x03, 0x03);                 // proprietary
            chb_reg_read_mod_write(XAH_CTRL_1, 0x04, 0x04);                 // set auto ack time to be as fast as possible

            break;
            case OQPSK_1000:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x02, 0x03);                 // proprietary
            chb_reg_read_mod_write(XAH_CTRL_1, 0x04, 0x04);                 // set auto ack time to be as fast as possible
            break;
            case OQPSK_500:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x01, 0x03);                 // proprietary
            chb_reg_read_mod_write(XAH_CTRL_1, 0x04, 0x04);                 // set auto ack time to be as fast as possible
            break;
            case OQPSK_250:
            chb_reg_read_mod_write(TRX_CTRL_2, 0x00, 0x03);                 // 802.15.4-2006-compliant
            break;
        }
    }
}

/**************************************************************************/
/*!
    Set the TX/RX state machine state. Some manual manipulation is required 
    for certain operations. Check the datasheet for more details on the state 
    machine and manipulations.
*/
/**************************************************************************/
static U8 chb_set_state(U8 state)
{
    U8 curr_state, delay;

    // if we're sleeping then don't allow transition
    if (CHB_SLPTR_PORT & _BV(CHB_SLPTR_PIN))
    {
        return RADIO_WRONG_STATE;
    }

    // if we're in a transition state, wait for the state to become stable
    curr_state = chb_get_state();
    if ((curr_state == BUSY_TX_ARET) || (curr_state == BUSY_RX_AACK) || (curr_state == BUSY_RX) || (curr_state == BUSY_TX))
    {
        while (chb_get_state() == curr_state);
    }

    // At this point it is clear that the requested new_state is:
    // TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON.
    // we need to handle some special cases before we transition to the new state
    switch (state)
    {
    case TRX_OFF:
        /* Go to TRX_OFF from any state. */
        CHB_SLPTR_DISABLE();
        chb_delay_us(TIME_NOCLK_TO_WAKE);
        chb_reg_read_mod_write(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1f);
        chb_delay_us(TIME_CMD_FORCE_TRX_OFF);
        break;

    case TX_ARET_ON:
        if (curr_state == RX_AACK_ON)
        {
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            chb_reg_read_mod_write(TRX_STATE, CMD_PLL_ON, 0x1f);
            chb_delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
        break;

    case RX_AACK_ON:
        if (curr_state == TX_ARET_ON)
        {
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            chb_reg_read_mod_write(TRX_STATE, CMD_PLL_ON, 0x1f);
            chb_delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
        break;
    }

    /* Now we're okay to transition to any new state. */
    chb_reg_read_mod_write(TRX_STATE, state, 0x1f);

    /* When the PLL is active most states can be reached in 1us. However, from */
    /* TRX_OFF the PLL needs time to activate. */
    delay = (curr_state == TRX_OFF) ? TIME_TRX_OFF_TO_PLL_ACTIVE : TIME_STATE_TRANSITION_PLL_ACTIVE;
    chb_delay_us(delay);

    if (chb_get_state() == state)
    {
        return RADIO_SUCCESS;
    }
    return RADIO_TIMED_OUT;
}

/**************************************************************************/
/*! 
    Sets the IEEE address of the radio.
*/
/**************************************************************************/
void chb_set_ieee_addr(U8 *addr)
{
    chb_eeprom_write(CHB_EEPROM_IEEE_ADDR, addr, 8); 
    chb_reg_write64(IEEE_ADDR_0, addr); 
}

/**************************************************************************/
/*!
    Retrieves the IEEE address of the radio.
*/
/**************************************************************************/
void chb_get_ieee_addr(U8 *addr)
{
    chb_eeprom_read(CHB_EEPROM_IEEE_ADDR, addr, 8);
}

/**************************************************************************/
/*!
    Sets the 16-bit short address of the radio.
*/
/**************************************************************************/
void chb_set_short_addr(U16 addr)
{
    U8 *addr_ptr = (U8 *)&addr;
    pcb_t *pcb = chb_get_pcb();

    chb_eeprom_write(CHB_EEPROM_SHORT_ADDR, addr_ptr, 2);
    chb_reg_write16(SHORT_ADDR_0, addr);
    pcb->src_addr = addr;
}

/**************************************************************************/
/*!
    Retrieves the 16-bit short address of the radio.
*/
/**************************************************************************/
U16 chb_get_short_addr()
{
    U8 addr[2];

    chb_eeprom_read(CHB_EEPROM_SHORT_ADDR, addr, 2);
    return *(U16 *)addr;
}

/**************************************************************************/
/*!
    Set the channel for the radio. 
*/
/**************************************************************************/
U8 chb_set_channel(U8 channel)
{
    U8 state;

    // Let's validate the channel for sanity before we try to set it
    if (radio_id == CHB_AT86RF212)
    {
      // Only the AT86RF212 is 900MHz
      // When set outside this range, default channel (1) seems to be used
      if ((channel < 0) || (channel > 10))
        return RADIO_INVALID_ARGUMENT;
    } else {
      // All other currently defined chips (AT86RF23[01]) are 2.4 GHz
      // When set outside this range, something appears at 2432 MHz, not sure what though
      if ((channel < 11) || (channel > 26))
        return RADIO_INVALID_ARGUMENT;
    }
    
    chb_reg_read_mod_write(PHY_CC_CCA, channel, 0x1f); 

    // check to see if we're running in 868 band
    if (radio_id == CHB_AT86RF212)
    {
        // if we're using the 868 MHz band, we should be running at 100 kbps 
        // according to 802.15.4 spec. set SUB_MODE to 0.
        U8 sub_mode;
        sub_mode = (channel == 0) ? 0 : 1;
        chb_reg_read_mod_write(TRX_CTRL_2, sub_mode << 2, 0x04);
    }

    // add a delay to allow the PLL to lock if in active mode.
    state = chb_get_state();
    if ((state == RX_ON) || (state == PLL_ON))
    {
        chb_delay_us(TIME_PLL_LOCK);
    }

    return ((chb_reg_read(PHY_CC_CCA) & 0x1f) == channel) ? RADIO_SUCCESS : RADIO_TIMED_OUT;
}

/**************************************************************************/
/*!
    Returns the current channel for the radio.
*/
/**************************************************************************/
U8 chb_get_channel()
{
    return (chb_reg_read(PHY_CC_CCA) & 0x1f);
}

/**************************************************************************/
/*!
    Change the default data rate of the radio
*/
/**************************************************************************/
U8 chb_set_datarate(U8 rate)
{
    U8 std_ack;

    // first check if the device is supported
    switch (radio_id)
    {
        case CHB_AT86RF230:
            char buf[50];

            // nothing to be done here. print warning & return.
            strcpy_P(buf, chb_err_not_supported);
            Serial.print(buf);
        return 0;

        case CHB_AT86RF231:
            // if we're in standards compliance mode, then use standard
            // ack turnaround time. if we're using a high speed mode, then
            // we need to speed up the ack turnaround.
            std_ack = (rate == 0) ? 0x00 : 0x04;

            // set the rate and set the ACK behavior for the data rate
            chb_reg_read_mod_write(TRX_CTRL_2, rate, 0x03);
            chb_reg_read_mod_write(XAH_CTRL_1, std_ack, 0x04);
        break;

        case CHB_AT86RF212:
            if (rate != 0)
            {
                // change to OQPSK mode if we're in a proprietary mode                 
                chb_reg_read_mod_write(TRX_CTRL_2, 0x08, 0x08);
            }

            // if we're in standards compliance mode, then use standard
            // ack turnaround time. if we're using a high speed mode, then
            // we need to speed up the ack turnaround.
            std_ack = (rate == 0) ? 0x00 : 0x04;

            // set the rate and set the ACK behavior for the data rate
            chb_reg_read_mod_write(TRX_CTRL_2, rate, 0x03);
            chb_reg_read_mod_write(XAH_CTRL_1, std_ack, 0x04);
        break;

        default:
        break;
    }
    return 1;
}

/**************************************************************************/
/*!
    Transmits a frame via the radio. It loads the data into the fifo,
    initiates a transmission attempt and returns the status of the transmission
    attempt.
*/
/**************************************************************************/
U8 chb_tx(U8 *hdr, U8 *data, U8 len)
{
    U8 state = chb_get_state();
    pcb_t *pcb = chb_get_pcb();

    if ((state == BUSY_TX) || (state == BUSY_TX_ARET))
    {
        return RADIO_WRONG_STATE;
    }

    // transition to the Transmit state
    chb_set_state(TX_ARET_ON);

    // check to make sure we're in the correct state. if not, then continue changing the state
    // until we're in the proper state
    while (chb_get_state() != TX_ARET_ON)
    {
        chb_set_state(TX_ARET_ON);
    }

    // write frame to buffer. first write header into buffer (add 1 for len byte), then data. 
    chb_frame_write(hdr, CHB_HDR_SZ + 1, data, len);

    //Do frame transmission. 
    pcb->trx_end = false;
    chb_reg_read_mod_write(TRX_STATE, CMD_TX_START, 0x1F);


    // wait for the transmission to end, signalled by the TRX END flag
    while (!pcb->trx_end)
    {
        ;
    }
    pcb->trx_end = false;

    // check the status of the transmission
    return chb_get_status();
}

/**************************************************************************/
/*!
    Return the part number of the radio
*/
/**************************************************************************/
U8 chb_get_part_num()
{
    return chb_reg_read(PART_NUM);
}


#if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (ARASHI_ENET_GATEWAY_LR == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1))
/**************************************************************************/
/*!
    Enable the high gain mode pin
*/
/**************************************************************************/        
    void chb_high_gain_mode_enable()
    {
        #if (ARASHI_ENET_GATEWAY_LR == 1)
            PORTC |= (1<<5); 
        #elif ((FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (SABOTEN == 1))
			PORTC |= (1<<6);
		#else
            PORTB |= (1<<7);
        #endif
    }

/**************************************************************************/
/*!
    Disable the high gain mode pin
*/
/**************************************************************************/
    void chb_high_gain_mode_disable()
    {
        #if (ARASHI_ENET_GATEWAY_LR == 1)
            PORTC &= ~(1<<5); 
		#elif ((FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (SABOTEN == 1))
			PORTC &= ~(1<<6);
        #else
            PORTB &= ~(1<<7);
        #endif
    }
#endif


/**************************************************************************/
/*!
    chb_set_retries
*/
/**************************************************************************/
void chb_set_retries(uint8_t retries)
{
    if (retries < MAX_RETRIES)
    {
        chb_reg_read_mod_write(XAH_CTRL_0, retries << CHB_MAX_FRAME_RETRIES_POS, 0xF << CHB_MAX_FRAME_RETRIES_POS);
    }
}

/**************************************************************************/
/*!
    Initialize the radio registers.
*/
/**************************************************************************/
static uint8_t chb_radio_init()
{
    U8 ieee_addr[8];
    U8 rnd, tmp;
    U16 addr;

    // disable intps while we config the radio
    chb_reg_write(IRQ_MASK, 0);

    // force transceiver off while we configure the intps
    chb_reg_read_mod_write(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1F);
    chb_delay_us(TIME_P_ON_TO_TRX_OFF);

    // set radio cfg parameters
    // **note** uncomment if these will be set to something other than default
    chb_reg_read_mod_write(XAH_CTRL_0, CHB_MAX_FRAME_RETRIES << CHB_MAX_FRAME_RETRIES_POS, 0xF << CHB_MAX_FRAME_RETRIES_POS);
    //chb_reg_read_mod_write(XAH_CTRL_0, CHB_MAX_CSMA_RETRIES << CHB_MAX_CSMA_RETRIES_POS, 0x7 << CHB_MAX_CSMA_RETRIES_POS);
    //chb_reg_read_mod_write(CSMA_SEED_1, CHB_MIN_BE << CHB_MIN_BE_POS, 0x3 << CHB_MIN_BE_POS);     
    //chb_reg_read_mod_write(CSMA_SEED_1, CHB_CSMA_SEED1 << CHB_CSMA_SEED1_POS, 0x7 << CHB_CSMA_SEED1_POS);        
    //chb_reg_read_mod_write(PHY_CC_CCA, CHB_CCA_MODE << CHB_CCA_MODE_POS,0x3 << CHB_CCA_MODE_POS);
    //chb_reg_write(CCA_THRES, CHB_CCA_ED_THRES);
    //chb_reg_read_mod_write(PHY_TX_PWR, CHB_TX_PWR, 0xf);

#if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))            
      // enable the rf front end controller
    tmp = chibiRegRead(TRX_CTRL1);
    tmp |= 0x80;
    chibiRegWrite(TRX_CTRL1, tmp);
#endif

    // identify device
    radio_id = chb_get_part_num();

    switch (radio_id)
    {
    case CHB_AT86RF230:
        // set default channel
        chb_set_channel(CHB_2_4GHZ_DEFAULT_CHANNEL);
        //Serial.println("AT86RF230 2.4 GHz radio detected.");

#if (CHIBI_PROMISCUOUS == 0)
    // set autocrc mode
    chb_reg_read_mod_write(PHY_TX_PWR, 1 << CHB_AUTO_CRC_POS, 1 << CHB_AUTO_CRC_POS);
#endif
        break;

    case CHB_AT86RF231:
        // set init mode data rate
        chb_set_mode(CHB_2_4GHZ_INIT_MODE);

        // set default channel
        chb_set_channel(CHB_2_4GHZ_DEFAULT_CHANNEL);
        //Serial.println("AT86RF231 2.4 GHz radio detected.");

#if (CHIBI_PROMISCUOUS == 0)
    // set autocrc mode
    chb_reg_read_mod_write(PHY_TX_PWR, 1 << CHB_AUTO_CRC_POS, 1 << CHB_AUTO_CRC_POS);
#endif
        break;

    case CHB_AT86RF212:
        // set mode to OQPSK or BPSK depending on setting
        chb_set_mode(CHB_900MHZ_INIT_MODE);

        // set default channel and tx power to max
        chb_set_channel(CHB_900MHZ_DEFAULT_CHANNEL);
        chb_reg_write(PHY_TX_PWR, CHB_900MHZ_TX_PWR);
        //Serial.println("AT86RF212 900 MHz radio detected.");

        // set crystal trim to improve signal reception
        // found that a value of 4 works well across all channels &
        // all data rates.
        chb_reg_read_mod_write(XOSC_CTRL, 0x04, 0x0F);

#if (CHIBI_PROMISCUOUS == 0)
        // set autocrc mode
        chb_reg_read_mod_write(TRX_CTRL1, 1 << 5, 1 << 5);
#endif

          // set crystal trim to improve signal reception
          // found that a value of 0xA works well across all channels &
          // all modes w/long range board.
          chb_reg_read_mod_write(XOSC_CTRL, 0x0A, 0x0F);

        #if ((SABOTEN == 1) || (FREAKUSB1284PLR == 1) || (FREAKDUINO1284PLR == 1) || (FREAKDUINO_LONG_RANGE == 1) || (FREAKUSBLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))
            // set the power to 5 dBm, the max for the CC1190 front end
            chb_reg_write(PHY_TX_PWR, 0x84);

                  // enable the high gain mode pin on the rx amp
            #if (ARASHI_ENET_GATEWAY_LR == 1)
                DDRC |= 1<<5;
                PORTC |= 1<<5; 
            #elif ((SABOTEN == 1) || (FREAKUSB1284PLR == 1) || (FREAKDUINO1284PLR == 1))
                DDRC |= 1<<6;
                PORTC |= 1<<6;
            #else
                DDRB |= 1<<7;
                PORTB |= (1<<7);
            #endif
        #endif


        break;

    default:
        //Serial.println("ERROR: Unknown radio detected.");
        break;
    }

    // set transceiver's fsm state
    chb_set_state(RX_STATE);

    // set pan ID
    chb_reg_write16(PAN_ID_0, CHB_PAN_ID);

    // set short addr
    // NOTE: Possibly get this from EEPROM
    chb_reg_write16(SHORT_ADDR_0, chb_get_short_addr());

    // set long addr
    // NOTE: Possibly get this from EEPROM
    chb_get_ieee_addr(ieee_addr);
    chb_reg_write64(IEEE_ADDR_0, ieee_addr);

    // do a read of the interrupt register to clear the interrupt bits
    chb_reg_read(IRQ_STATUS);

    // re-enable intps while we config the radio
    chb_reg_write(IRQ_MASK, 0x08);

    // enable mcu intp pin on INT6 for rising edge
    CFG_CHB_INTP();

    // make sure we're in the right state
    if (chb_get_state() != RX_STATE)
    {
        return 0;
    }

    // init the CSMA random seed value
    if (radio_id == CHB_AT86RF230)
    {
        // set a seed value for the CSMA backoffs
        addr = chb_get_short_addr();
        tmp = (U8)(addr & 0x00FF);
        chb_reg_write(CSMA_SEED_0, tmp); 
    }
    else
    {
        rnd = chb_get_rand();
        chb_reg_write(CSMA_SEED_0, rnd);
    } 
    return 1;
}

/**************************************************************************/
/*!
    Initialize the complete driver.
*/
/**************************************************************************/
void chb_drvr_init()
{
    char buf[50];
    
    // config SPI for at86rf230 access
    chb_spi_init();

    // configure IOs
    CHB_SLPTR_DDIR |= (_BV(CHB_SLPTR_PIN));
    CHB_SLPTR_DISABLE();

    // config radio
    for (int i=0; i<3; i++)
    {
        if (chb_radio_init()) 
        {
            return;
        }
        delay(100);
    }
    // radio could not be initialized. Print out error message.
    strcpy_P(buf, chb_err_init);
    Serial.print(buf);
}

/**************************************************************************/
/*!
    Enable or disable the radio's sleep mode.
*/
/**************************************************************************/
void chb_sleep(U8 enb)
{
    uint8_t tmp;

    if (enb)
    {
        // if we're using a long range device, disable the high gain mode pin
        #if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))     
            // disable RF front end
            tmp = chibiRegRead(TRX_CTRL1);
            tmp &= ~(0x80);
            chibiRegWrite(TRX_CTRL1, tmp);

            chb_high_gain_mode_disable();
        #endif

        // first we need to go to TRX OFF state
        chb_set_state(TRX_OFF);

        // then we need to make SLP_TR pin an input. this enables the external
        // pullup on that pin. we do this so that we won't need to go through
        // a voltage divider which drains current. we want to avoid that when
        // we sleep
        CHB_SLPTR_PORT |= _BV(CHB_SLPTR_PIN);
        CHB_SLPTR_DDIR &= ~(_BV(CHB_SLPTR_PIN));
        CHB_SLPTR_PORT |= _BV(CHB_SLPTR_PIN);
    }
    else
    {
        // if we're using a long range device, enable the high gain mode pin
        #if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))   
            // enable the RF front end
            tmp = chibiRegRead(TRX_CTRL1);
            tmp |= 0x80;
            chibiRegWrite(TRX_CTRL1, tmp);

            chb_high_gain_mode_enable();
        #endif

        // make sure the SLPTR pin is low first
        CHB_SLPTR_PORT &= ~(_BV(CHB_SLPTR_PIN));

        // make the SLPTR pin an output
        CHB_SLPTR_DDIR |= _BV(CHB_SLPTR_PIN);

        // we need to allow some time for the PLL to lock
        chb_delay_us(TIME_SLEEP_TO_TRX_OFF);

        // Turn the transceiver back on
        chb_set_state(RX_STATE);
    }
}
