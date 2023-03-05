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
#include <chibiArduino.h>
#include "chibiUsrCfg.h"

#if (CHIBI_PROMISCUOUS == 1)
    const char chb_err_overflow[] PROGMEM = "";
    const char chb_err_init[] PROGMEM = "";
    const char chb_err_not_supported[] PROGMEM = "";

    #if defined(__AVR_ATmega1284P__) 
        // If the ATMega1284P is used, it  has 16 kB RAM so we can crank up the 
        // radio buffer size
        #define CHB_BUF_SZ 10000
    #else
        // if we're using promiscuous mode, we may end up capturing a lot of frames.
        // crank up the buffer size to handle traffic spikes.
        #define CHB_BUF_SZ 768
    #endif
#else
    // store string messages in flash rather than RAM
    const char chb_err_overflow[] PROGMEM = "BUFFER FULL. TOSSING INCOMING DATA\n";
    const char chb_err_init[] PROGMEM = "RADIO NOT INITIALIZED PROPERLY\n";
    const char chb_err_not_supported[] PROGMEM = "FEATURE NOT SUPPORTED";

    // in normal mode, this is the buffer size to handle incoming frames. if there
    // is a lot of traffic and you're getting buffer issues, then increase this 
    // value so that more frames can be held inside RAM
    #define CHB_BUF_SZ 256
#endif

#define TEMPBUFSZ 100

char tempBuf[TEMPBUFSZ];
uint8_t ringBuf[CHB_BUF_SZ];
uint16_t rdPtr, wrPtr, len;

// these are for the duplicate checking and rejection
uint8_t prev_seq;
uint16_t prev_src_addr;

// this holds the id of the radio to identify the chip type
uint8_t radio_id = 0;
pcb_t pcb;
chb_rx_data_t rx_data;


/**************************************************************************/
// main constructor
/**************************************************************************/
ChibiArduinoTest::ChibiArduinoTest(uint8_t cs, uint8_t slptr)
{
	pinCs = cs;
	pinSlpTr = slptr;

    rdPtr = 0;
    wrPtr = 0;
    len = 0;

    memset(&pcb, 0, sizeof(pcb_t));
    memset(&ringBuf, 0, CHB_BUF_SZ);

    pinMode(pinCs, OUTPUT);
    digitalWrite(pinCs, HIGH);	
    
    pinMode(pinSlpTr, OUTPUT);
    digitalWrite(pinSlpTr, LOW);
}

/**************************************************************************/
// begin
/**************************************************************************/
void ChibiArduinoTest::begin()
{
    pcb.src_addr = getShortAddr();

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    // reset prev_seq and prev_src_addr to default values
    // there's a problem if the radio gets re-initialized since no initializing the prev_seq and 
    // prev_src_addr will result in a bug where the data could get flagged as a dupe
    prev_seq = 0xFF;
    prev_src_addr = 0xFFFE; 
    
    // config radio
    for (int i=0; i<3; i++)
    {
        if (radioInit()) 
        {
            return;
        }
        delay(100);
    }

    // radio could not be initialized. Print out error message.
    strcpy_P(tempBuf, chb_err_init);
    Serial.print(tempBuf);
}

/**************************************************************************/
/*! 
    Requires the dest addr, location to store data, and len of payload.
    Returns the length of the hdr. 
*/
/**************************************************************************/
uint8_t ChibiArduinoTest::genHeader(uint8_t *hdr, uint16_t addr, uint8_t len)
{
    uint8_t *hdr_ptr = hdr;

    // calc frame size and put in 0 position of array
    // frame size = hdr sz + payload len + fcs len
    *hdr_ptr++ = CHB_HDR_SZ + len + CHB_FCS_LEN;

    // use default fcf byte 0 val but test for ack request. we won't request
    // ack if broadcast. all other cases we will.
    *hdr_ptr++ = CHB_FCF_BYTE_0 | ((addr != 0xFFFF) << CHB_ACK_REQ_POS);
    *hdr_ptr++ = CHB_FCF_BYTE_1;

    *hdr_ptr++ = pcb.seq++;

    // fill out dest pan ID, dest addr, src addr
    *(uint16_t *)hdr_ptr = CHB_PAN_ID;
    hdr_ptr += sizeof(uint16_t);
    *(uint16_t *)hdr_ptr = addr;
    hdr_ptr += sizeof(uint16_t);
    *(uint16_t *)hdr_ptr = pcb.src_addr;
    hdr_ptr += sizeof(uint16_t);
    
    // return the len of the header
    return hdr_ptr - hdr;
}

/**************************************************************************/
// setShortAddr
/**************************************************************************/
void ChibiArduinoTest::setShortAddr(uint16_t addr)
{
 	uint8_t *addr_ptr = (uint8_t *)&addr;

    eepromWrite(CHB_EEPROM_SHORT_ADDR, addr_ptr, 2);
    regWrite16(SHORT_ADDR_0, addr);
    pcb.src_addr = addr;
}

/**************************************************************************/
// getShortAddr
/**************************************************************************/
uint16_t ChibiArduinoTest::getShortAddr()
{
    uint8_t addr[2];

    eepromRead(CHB_EEPROM_SHORT_ADDR, addr, 2);
    return *(uint16_t *)addr;
}

/**************************************************************************/
// setIEEEAddr
/**************************************************************************/
void ChibiArduinoTest::setIEEEAddr(uint8_t *addr)
{
    eepromWrite(CHB_EEPROM_IEEE_ADDR, addr, 8); 
    regWrite64(IEEE_ADDR_0, addr); 
}

/**************************************************************************/
// getIEEEAddr
/**************************************************************************/
void ChibiArduinoTest::getIEEEAddr(uint8_t *addr)
{
	eepromRead(CHB_EEPROM_IEEE_ADDR, addr, 8);
}

/**************************************************************************/
/*!
    This is the main function that transmits the data array to the specified
    address.
*/
/**************************************************************************/
uint8_t ChibiArduinoTest::send(uint16_t addr, uint8_t *data, uint8_t len)
{
    uint8_t status, frm_len, hdr[CHB_HDR_SZ + 1];
    
    while (len > 0)
    {
        // calculate which frame len to use. if greater than max payload, split
        // up operation.
        frm_len = (len > CHB_MAX_PAYLOAD) ? CHB_MAX_PAYLOAD : len;

        // gen frame header
        genHeader(hdr, addr, frm_len);

        // send data to chip
        status = tx(hdr, data, frm_len);

        if (status != CHB_SUCCESS)
        {
            switch (status)
            {
            case RADIO_SUCCESS:
                // fall through
            case CHB_SUCCESS_DATA_PENDING:
                pcb.txd_success++;
                break;

            case CHB_NO_ACK:
                pcb.txd_noack++;
                break;

            case CHB_CHANNEL_ACCESS_FAILURE:
                pcb.txd_channel_fail++;
                break;

            default:
                break;
            }
            return status;
        }

        // adjust len and restart
        len = len - frm_len;
    }

    return CHB_SUCCESS;
}

/**************************************************************************/
// tx
/**************************************************************************/
uint8_t ChibiArduinoTest::tx(uint8_t *hdr, uint8_t *data, uint8_t len)
{
    uint8_t state = getState();

    if ((state == BUSY_TX) || (state == BUSY_TX_ARET))
    {
        return RADIO_WRONG_STATE;
    }

    // transition to the Transmit state
    setState(TX_ARET_ON);

    // check to make sure we're in the correct state. if not, then continue changing the state
    // until we're in the proper state
    while (getState() != TX_ARET_ON)
    {
        setState(TX_ARET_ON);
    }

    // write frame to buffer. first write header into buffer (add 1 for len byte), then data. 
    frameWrite(hdr, CHB_HDR_SZ + 1, data, len);

    //Do frame transmission. 
    pcb.trx_end = false;
    regReadModWrite(TRX_STATE, CMD_TX_START, 0x1F);

    // wait for the transmission to end, signalled by the TRX END flag
    while (!pcb.trx_end)
    {
        ;
    }
    pcb.trx_end = false;

    // check the status of the transmission
    return getStatus();
}

/**************************************************************************/
// dataRcvd
/**************************************************************************/
uint8_t ChibiArduinoTest::dataRcvd()
{
    return pcb.data_rcv;
}

/**************************************************************************/
// getData
/**************************************************************************/
uint8_t ChibiArduinoTest::getData(uint8_t *data)
{
    // point the data buffer to the buffer that was passed in from arduino ide
    rx_data.data = (uint8_t *)data;

    // load the data into the buffer and get the length
    rx_data.len = read(&rx_data);
    return rx_data.len;
}

/**************************************************************************/
/*!
    Read data from the buffer. Need to pass in a buffer of at leasts max frame
    size and two 16-bit containers for the src and dest addresses.
 
    The read function will automatically populate the addresses and the data with
    the frm payload. It will then return the len of the payload.
*/
/**************************************************************************/
uint8_t ChibiArduinoTest::read(chb_rx_data_t *rx)
{
    uint8_t i, len, seq, *data_ptr;

    data_ptr = rx->data;

    // first byte is always len. check it to make sure 
    // we have a valid len byte.
    if ((len = bufRead()) > CHB_MAX_FRAME_LENGTH)
    {
        return 0;
    }

// TEST
//    printf("%d, %d\n", len, chb_buf_get_len());

    *data_ptr++ = len;

    // load the rest of the data into buffer
    for (i=0; i<len; i++)
    {
        *data_ptr++ = bufRead();
    }

    // we're using the buffer that's fed in as an argument as a temp
    // buffer as well to save resources.
    // we'll use it as temp storage to parse the frame. then move the frame
    // down so that only the payload will be in the buffer.

    // extract the sequence number
    data_ptr = rx->data + 3;    // location of sequence number
    seq = *data_ptr;

    // parse the buffer and extract the dest and src addresses
    data_ptr = rx->data + 6;                // location of dest addr
    rx->dest_addr = *(uint16_t *)data_ptr;
    data_ptr += sizeof(uint16_t);
    rx->src_addr = *(uint16_t *)data_ptr;
    data_ptr += sizeof(uint16_t);

    // if the data in the rx buf is 0, then clear the rx_flag. otherwise, keep it raised
    if (!bufGetLen())
    {
        pcb.data_rcv = false;
    }

#if (CHIBI_PROMISCUOUS)
    // if we're in promiscuous mode, we don't want to do any duplicate rejection and we don't want to move the payload
    // to the front of the buffer. We want to capture the full frame so just keep the frame intact and return the length.
    return len;
#else
    // duplicate frame check (dupe check). we want to remove frames that have been already been received since they 
    // are just retries. 
    // note: this dupe check only removes duplicate frames from the previous transfer. if another frame from a different
    // node comes in between the dupes, then the dupe will show up as a received frame.
    if ((seq == prev_seq) && (rx->src_addr == prev_src_addr))
    {
        // this is a duplicate frame from a retry. the remote node thinks we didn't receive 
        // it properly. discard.
        return 0;
    }
    else
    {
        prev_seq = seq;
        prev_src_addr = rx->src_addr;
    }

    // move the payload down to the beginning of the data buffer
    memmove(rx->data, data_ptr, len - CHB_HDR_SZ);

    // finally, return the len of the payload
    return len - CHB_HDR_SZ - CHB_FCS_LEN;
#endif

}

/**************************************************************************/
// getRSSI
/**************************************************************************/
uint8_t ChibiArduinoTest::getRSSI()
{
    return pcb.ed;
}

/**************************************************************************/
// getSrcAddr
/**************************************************************************/
uint16_t ChibiArduinoTest::getSrcAddr()
{
    return rx_data.src_addr;
}

/**************************************************************************/
// setChannel
/**************************************************************************/
uint8_t ChibiArduinoTest::setChannel(uint8_t channel)
{
    uint8_t state;

    // Let's validate the channel for sanity before we try to set it
    if (radio_id == CHB_AT86RF212)
    {
      // Only the AT86RF212 is 900MHz
      // When set outside this range, default channel (1) seems to be used
      if (channel > 10)
        return RADIO_INVALID_ARGUMENT;
    } else {
      // All other currently defined chips (AT86RF23[01]) are 2.4 GHz
      // When set outside this range, something appears at 2432 MHz, not sure what though
      if ((channel < 11) || (channel > 26))
        return RADIO_INVALID_ARGUMENT;
    }
    
    regReadModWrite(PHY_CC_CCA, channel, 0x1f); 

    // check to see if we're running in 868 band
    if (radio_id == CHB_AT86RF212)
    {
        // if we're using the 868 MHz band, we should be running at 100 kbps 
        // according to 802.15.4 spec. set SUB_MODE to 0.
        uint8_t sub_mode;
        sub_mode = (channel == 0) ? 0 : 1;
        regReadModWrite(TRX_CTRL_2, sub_mode << 2, 0x04);
    }

    // add a delay to allow the PLL to lock if in active mode.
    state = getState();
    if ((state == RX_ON) || (state == PLL_ON))
    {
        delayUS(TIME_PLL_LOCK);
    }

    return ((regRead(PHY_CC_CCA) & 0x1f) == channel) ? RADIO_SUCCESS : RADIO_TIMED_OUT;
}

/**************************************************************************/
// getChannel
/**************************************************************************/
uint8_t ChibiArduinoTest::getChannel()
{
    return (regRead(PHY_CC_CCA) & 0x1f);
}

/**************************************************************************/
// getPartID
/**************************************************************************/
uint8_t ChibiArduinoTest::getPartID()
{
    return regRead(PART_NUM);
}

/**************************************************************************/
// sleepRadio
/**************************************************************************/
void ChibiArduinoTest::sleepRadio(uint8_t enb)
{
    if (enb)
    {
        // if we're using a long range device, disable the high gain mode pin
        #if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))     
            // disable RF front end
            tmp = chibiRegRead(TRX_CTRL1);
            tmp &= ~(0x80);
            chibiRegWrite(TRX_CTRL1, tmp);

            highGainModeDisable();
        #endif

        // first we need to go to TRX OFF state
        setState(TRX_OFF);

        // then we need to make SLP_TR pin an input. this enables the external
        // pullup on that pin. we do this so that we won't need to go through
        // a voltage divider which drains current. we want to avoid that when
        // we sleep
        pinMode(pinSlpTr,INPUT_PULLUP);
    }
    else
    {
        // if we're using a long range device, enable the high gain mode pin
        #if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))   
            // enable the RF front end
            tmp = chibiRegRead(TRX_CTRL1);
            tmp |= 0x80;
            chibiRegWrite(TRX_CTRL1, tmp);

            highGainModeEnable();
        #endif

        digitalWrite(pinSlpTr, OUTPUT);

        // we need to allow some time for the PLL to lock
        delayUS(TIME_SLEEP_TO_TRX_OFF);

        // Turn the transceiver back on
        setState(RX_STATE);
    }
}

/**************************************************************************/
// setDataRate
/**************************************************************************/
void ChibiArduinoTest::setDataRate(uint8_t rate)
{
    uint8_t std_ack;

    // first check if the device is supported
    switch (radio_id)
    {
        case CHB_AT86RF230:
            // nothing to be done here. print warning & return.
            strcpy_P(tempBuf, chb_err_not_supported);
            Serial.print(tempBuf);
        return;

        case CHB_AT86RF231:
            // if we're in standards compliance mode, then use standard
            // ack turnaround time. if we're using a high speed mode, then
            // we need to speed up the ack turnaround.
            std_ack = (rate == 0) ? 0x00 : 0x04;

            // set the rate and set the ACK behavior for the data rate
            regReadModWrite(TRX_CTRL_2, rate, 0x03);
            regReadModWrite(XAH_CTRL_1, std_ack, 0x04);
        break;

        case CHB_AT86RF212:
            if (rate != 0)
            {
                // change to OQPSK mode if we're in a proprietary mode                 
                regReadModWrite(TRX_CTRL_2, 0x08, 0x08);
            }

            // if we're in standards compliance mode, then use standard
            // ack turnaround time. if we're using a high speed mode, then
            // we need to speed up the ack turnaround.
            std_ack = (rate == 0) ? 0x00 : 0x04;

            // set the rate and set the ACK behavior for the data rate
            regReadModWrite(TRX_CTRL_2, rate, 0x03);
            regReadModWrite(XAH_CTRL_1, std_ack, 0x04);
        break;

        default:
        break;
    }
}

/**************************************************************************/
// getRand
/**************************************************************************/
uint8_t ChibiArduinoTest::getRand()
{
    if (radio_id == CHB_AT86RF230)
    {
        // nothing to be done here. print warning & return.
        strcpy_P(tempBuf, chb_err_not_supported);
        Serial.print(tempBuf);
        return 0;
    }
    else if ((radio_id == CHB_AT86RF212) || (radio_id == CHB_AT86RF231))
    {
        uint8_t i, rnd, tmp;

        rnd = tmp = 0;
        for (i=0; i<4; i++)
        {
            tmp = regRead(PHY_RSSI);
            tmp >>= 5;
            tmp &= 0x03;
            rnd |= tmp << (i*2);
            delayUS(5);
        }
        return rnd;
    }
    return 0;
}

/**************************************************************************/
// setMode
/**************************************************************************/
void ChibiArduinoTest::setMode(uint8_t mode)
{
	if (radio_id == CHB_AT86RF212)
    {
        switch (mode)
        {
        case OQPSK_SINRC_100:
            regReadModWrite(TRX_CTRL_2, 0x08, 0x3f);                 // 802.15.4-2006, channel page 2, channel 0 (868 MHz, Europe)
            break;
        case OQPSK_SIN_250:
            regReadModWrite(TRX_CTRL_2, 0x0c, 0x3f);                 // 802.15.4-2006, channel page 2, channels 1-10 (915 MHz, US)
            break;
        case OQPSK_RC_250:
            regReadModWrite(TRX_CTRL_2, 0x1c, 0x3f);
            break;
        case OQPSK_SIN_500:                                          // 802.15.4-2006, channel page 5, channel 0-3 (780 MHz, China)
            regReadModWrite(TRX_CTRL_2, 0x0d, 0x3f);                 // proprietary
            break;
        case OQPSK_SIN_1000:
            regReadModWrite(TRX_CTRL_2, 0x2e, 0x3f);                 // proprietary
            break;
        case BPSK_40:
            regReadModWrite(TRX_CTRL_2, 0x04, 0x3f);                 // 802.15.4-2006, BPSK, 40 kbps
            break;
        case BPSK_20:
            regReadModWrite(TRX_CTRL_2, 0x00, 0x3f);                 // 802.15.4-2006, BPSK, 20 kbps
            break;
        }
    }
    else if (radio_id == CHB_AT86RF231)
    {
        switch (mode)
        {
            case OQPSK_2000:            
            regReadModWrite(TRX_CTRL_2, 0x03, 0x03);                 // proprietary
            regReadModWrite(XAH_CTRL_1, 0x04, 0x04);                 // set auto ack time to be as fast as possible

            break;
            case OQPSK_1000:
            regReadModWrite(TRX_CTRL_2, 0x02, 0x03);                 // proprietary
            regReadModWrite(XAH_CTRL_1, 0x04, 0x04);                 // set auto ack time to be as fast as possible
            break;
            case OQPSK_500:
            regReadModWrite(TRX_CTRL_2, 0x01, 0x03);                 // proprietary
            regReadModWrite(XAH_CTRL_1, 0x04, 0x04);                 // set auto ack time to be as fast as possible
            break;
            case OQPSK_250:
            regReadModWrite(TRX_CTRL_2, 0x00, 0x03);                 // 802.15.4-2006-compliant
            break;
        }
    }
}

/**************************************************************************/
// getMode
/**************************************************************************/
uint8_t ChibiArduinoTest::getMode()
{
    return regRead(TRX_CTRL_2);
}

/**************************************************************************/
// setRetries
/**************************************************************************/
void ChibiArduinoTest::setRetries(uint8_t retries)
{
    if (retries < MAX_RETRIES)
    {
        regReadModWrite(XAH_CTRL_0, retries << CHB_MAX_FRAME_RETRIES_POS, 0xF << CHB_MAX_FRAME_RETRIES_POS);
    }
}

/**************************************************************************/
// highGainModeEnable
/**************************************************************************/
void ChibiArduinoTest::highGainModeEnable()
{
	// to be implemented
}

/**************************************************************************/
// highGainModeDisable
/**************************************************************************/
void ChibiArduinoTest::highGainModeDisable()
{
	// to be implemented
}

/**************************************************************************/
//    Initialize the radio registers.
/**************************************************************************/
uint8_t ChibiArduinoTest::radioInit()
{
    uint8_t ieee_addr[8];
    uint8_t rnd, tmp;
    uint16_t addr;

    // disable intps while we config the radio
    regWrite(IRQ_MASK, 0);
    
    // force transceiver off while we configure the intps
    regReadModWrite(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1F);
    delayUS(TIME_P_ON_TO_TRX_OFF);

    // set radio cfg parameters
    // **note** uncomment if these will be set to something other than default
    regReadModWrite(XAH_CTRL_0, CHB_MAX_FRAME_RETRIES << CHB_MAX_FRAME_RETRIES_POS, 0xF << CHB_MAX_FRAME_RETRIES_POS);
    //regReadModWrite(XAH_CTRL_0, CHB_MAX_CSMA_RETRIES << CHB_MAX_CSMA_RETRIES_POS, 0x7 << CHB_MAX_CSMA_RETRIES_POS);
    //regReadModWrite(CSMA_SEED_1, CHB_MIN_BE << CHB_MIN_BE_POS, 0x3 << CHB_MIN_BE_POS);     
    //regReadModWrite(CSMA_SEED_1, CHB_CSMA_SEED1 << CHB_CSMA_SEED1_POS, 0x7 << CHB_CSMA_SEED1_POS);        
    //regReadModWrite(PHY_CC_CCA, CHB_CCA_MODE << CHB_CCA_MODE_POS,0x3 << CHB_CCA_MODE_POS);
    //regWrite(CCA_THRES, CHB_CCA_ED_THRES);
    //regReadModWrite(PHY_TX_PWR, CHB_TX_PWR, 0xf);

#if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))            
      // enable the rf front end controller
    tmp = chibiRegRead(TRX_CTRL1);
    tmp |= 0x80;
    chibiRegWrite(TRX_CTRL1, tmp);
#endif

    // identify device
    radio_id = getPartID();

    switch (radio_id)
    {
    case CHB_AT86RF230:
        // set default channel
        setChannel(CHB_2_4GHZ_DEFAULT_CHANNEL);
        //Serial.println("AT86RF230 2.4 GHz radio detected.");

#if (CHIBI_PROMISCUOUS == 0)
    // set autocrc mode
    regReadModWrite(PHY_TX_PWR, 1 << CHB_AUTO_CRC_POS, 1 << CHB_AUTO_CRC_POS);
#endif
        break;

    case CHB_AT86RF231:
        // set init mode data rate
        setMode(CHB_2_4GHZ_INIT_MODE);

        // set default channel
        setChannel(CHB_2_4GHZ_DEFAULT_CHANNEL);
        //Serial.println("AT86RF231 2.4 GHz radio detected.");

#if (CHIBI_PROMISCUOUS == 0)
    // set autocrc mode
    regReadModWrite(PHY_TX_PWR, 1 << CHB_AUTO_CRC_POS, 1 << CHB_AUTO_CRC_POS);
#endif
        break;

    case CHB_AT86RF212:
        // set mode to OQPSK or BPSK depending on setting
        setMode(CHB_900MHZ_INIT_MODE);

        // set default channel and tx power to max
        setChannel(CHB_900MHZ_DEFAULT_CHANNEL);
        regWrite(PHY_TX_PWR, CHB_900MHZ_TX_PWR);
        //Serial.println("AT86RF212 900 MHz radio detected.");

        // set crystal trim to improve signal reception
        // found that a value of 4 works well across all channels &
        // all data rates.
        regReadModWrite(XOSC_CTRL, 0x04, 0x0F);

#if (CHIBI_PROMISCUOUS == 0)
        // set autocrc mode
        regReadModWrite(TRX_CTRL1, 1 << 5, 1 << 5);
#endif

          // set crystal trim to improve signal reception
          // found that a value of 0xA works well across all channels &
          // all modes w/long range board.
          regReadModWrite(XOSC_CTRL, 0x0A, 0x0F);

        #if ((SABOTEN == 1) || (FREAKUSB1284PLR == 1) || (FREAKDUINO1284PLR == 1) || (FREAKDUINO_LONG_RANGE == 1) || (FREAKUSBLR == 1) || (ARASHI_ENET_GATEWAY_LR == 1))
            // set the power to 5 dBm, the max for the CC1190 front end
            regWrite(PHY_TX_PWR, 0x84);

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
    setState(RX_STATE);

    // set pan ID
    regWrite16(PAN_ID_0, CHB_PAN_ID);

    // set short addr
    // NOTE: Possibly get this from EEPROM
    regWrite16(SHORT_ADDR_0, getShortAddr());

    // set long addr
    // NOTE: Possibly get this from EEPROM
    getIEEEAddr(ieee_addr);
    regWrite64(IEEE_ADDR_0, ieee_addr);

    // do a read of the interrupt register to clear the interrupt bits
    regRead(IRQ_STATUS);

    // re-enable intps while we config the radio
    regWrite(IRQ_MASK, 0x08);

    // enable mcu intp pin on INT6 for rising edge
    CFG_CHB_INTP();

    // make sure we're in the right state
    if (getState() != RX_STATE)
    {
        return 0;
    }

    // init the CSMA random seed value
    if (radio_id == CHB_AT86RF230)
    {
        // set a seed value for the CSMA backoffs
        addr = getShortAddr();
        tmp = (uint8_t)(addr & 0x00FF);
        regWrite(CSMA_SEED_0, tmp); 
    }
    else
    {
        rnd = getRand();
        regWrite(CSMA_SEED_0, rnd);
    } 
    return 1;
}

/**************************************************************************/
/*!
    Set the TX/RX state machine state. Some manual manipulation is required 
    for certain operations. Check the datasheet for more details on the state 
    machine and manipulations.
*/
/**************************************************************************/
uint8_t ChibiArduinoTest::setState(uint8_t state)
{
    uint8_t curr_state, delay;

#if (ILLUMINADO_ATMEGA4808 == 0)
    // if we're sleeping then don't allow transition
    if (CHB_SLPTR_PORT & _BV(CHB_SLPTR_PIN))
    {
        return RADIO_WRONG_STATE;
    }
#endif    

    // if we're in a transition state, wait for the state to become stable
    curr_state = getState();
    if ((curr_state == BUSY_TX_ARET) || (curr_state == BUSY_RX_AACK) || (curr_state == BUSY_RX) || (curr_state == BUSY_TX))
    {
        while (getState() == curr_state);
    }

    // At this point it is clear that the requested new_state is:
    // TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON.
    // we need to handle some special cases before we transition to the new state
    switch (state)
    {
    case TRX_OFF:
        /* Go to TRX_OFF from any state. */
        CHB_SLPTR_DISABLE();
        delayUS(TIME_NOCLK_TO_WAKE);
        regReadModWrite(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1f);
        delayUS(TIME_CMD_FORCE_TRX_OFF);
        break;

    case TX_ARET_ON:
        if (curr_state == RX_AACK_ON)
        {
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            regReadModWrite(TRX_STATE, CMD_PLL_ON, 0x1f);
            delayUS(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
        break;

    case RX_AACK_ON:
        if (curr_state == TX_ARET_ON)
        {
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            regReadModWrite(TRX_STATE, CMD_PLL_ON, 0x1f);
            delayUS(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
        break;
    }

    /* Now we're okay to transition to any new state. */
    regReadModWrite(TRX_STATE, state, 0x1f);

    /* When the PLL is active most states can be reached in 1us. However, from */
    /* TRX_OFF the PLL needs time to activate. */
    delay = (curr_state == TRX_OFF) ? TIME_TRX_OFF_TO_PLL_ACTIVE : TIME_STATE_TRANSITION_PLL_ACTIVE;
    delayUS(delay);

    if (getState() == state)
    {
        return RADIO_SUCCESS;
    }
    return RADIO_TIMED_OUT;
}

/**************************************************************************/
/*!
    Retrieve the state from the radio's state machine register.
*/
/**************************************************************************/
uint8_t ChibiArduinoTest::getState()
{
    return regRead(TRX_STATUS) & 0x1f;
}

/**************************************************************************/
/*!
    Retrieve the status of the last received frame. 
*/
/**************************************************************************/
uint8_t ChibiArduinoTest::getStatus()
{
    return regRead(TRX_STATE) >> CHB_TRAC_STATUS_POS;
}

/**************************************************************************/
/*!
    Perform a delay of the specified amount of microseconds.
*/
/**************************************************************************/
void ChibiArduinoTest::delayUS(uint16_t usec)
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
uint8_t ChibiArduinoTest::regRead(uint8_t addr)
{
    uint8_t val = 0;

    /* Add the register read command to the register address. */
    addr |= 0x80;

    cli();
    CHB_SPI_ENABLE();

    /*Send Register address and read register content.*/
    val = SPI.transfer(addr);
    val = SPI.transfer(val);

    CHB_SPI_DISABLE();
    sei();

    return val;
}

/**************************************************************************/
/*! 
    Read a 16-bit register. This actually does two consecutive 8-bit reads
    of contiguous registers. It's mostly used to retrieve the PAN ID or the
    short address of the node.
*/
/**************************************************************************/
uint16_t ChibiArduinoTest::regRead16(uint8_t addr)
{
    uint8_t i;
    uint16_t val = 0;

    for (i=0; i<2; i++)
    {
        addr |= regRead(addr + i) << (8 * i);
    }
    return val;
}

/**************************************************************************/
//    Write to the radio's registers at the specified address with the specified
//    value.
/**************************************************************************/
void ChibiArduinoTest::regWrite(uint8_t addr, uint8_t val)
{
    /* Add the Register Write command to the address. */
    addr |= 0xC0;

    cli();
    CHB_SPI_ENABLE();

    /*Send Register address and write register content.*/
    SPI.transfer(addr);
    SPI.transfer(val);

    CHB_SPI_DISABLE();
    sei();
}

/**************************************************************************/
//    Write 16-bits to the radio's registers. This does two consecutive 8-bit
//    writes to contiguous memory addresses. 
/**************************************************************************/
void ChibiArduinoTest::regWrite16(uint8_t addr, uint16_t val)
{
    uint8_t i;

    for (i=0; i<2; i++)
    {
        regWrite(addr + i, val >> (8 * i));
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
void ChibiArduinoTest::regWrite64(uint8_t addr, uint8_t *val)
{
    uint8_t i;

    for (i=0; i<8; i++)
    {
        regWrite(addr + i, *(val + i));
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
void ChibiArduinoTest::regReadModWrite(uint8_t addr, uint8_t val, uint8_t mask)
{
    uint8_t tmp;

    tmp = regRead(addr);
    val &= mask;                // mask off stray bits from val
    tmp &= ~mask;               // mask off bits in reg val
    tmp |= val;                 // copy val into reg val
    regWrite(addr, tmp);   // write back to reg

}

/**************************************************************************/
/*!
    Write data to the frame FIFO of the radio. This is where the data to
    be transmitted gets written. The frame data is written in two separate bursts.
    The first burst is to write the frame header and the second burst is
    to write the frame payload. Makes things easier this way.
*/
/**************************************************************************/
void ChibiArduinoTest::frameWrite(uint8_t *hdr, uint8_t hdr_len, uint8_t *data, uint8_t data_len)
{
    uint8_t i;

    // dont allow transmission longer than max frame size
    if ((hdr_len + data_len) > 127)
    {
        return;
    }

    // initiate spi transaction
    cli();
    CHB_SPI_ENABLE(); 

    // send fifo write command
    SPI.transfer(CHB_SPI_CMD_FW);

    // write hdr contents to fifo
    for (i=0; i<hdr_len; i++)
    {
        SPI.transfer(*hdr++);
    }

    // write data contents to fifo
    for (i=0; i<data_len; i++)
    {
        SPI.transfer(*data++);
    }

    // terminate spi transaction
    CHB_SPI_DISABLE(); 
    sei();
}

/**************************************************************************/
/*!
    Read the data from the frame FIFO. When data arrives at the radio, it will
    get stored in the frame FIFO. The data will get read out from this function
    and stored in the ring buffer. 
*/
/**************************************************************************/
void ChibiArduinoTest::frameRead()
{
    uint8_t i, len, data;

    cli();
    CHB_SPI_ENABLE();

    /*Send frame read command and read the length.*/
    SPI.transfer(CHB_SPI_CMD_FR);
    len = SPI.transfer(0);

    // check the length of the frame to make sure its within the correct size limits
    if ((len >= CHB_MIN_FRAME_LENGTH) && (len <= CHB_MAX_FRAME_LENGTH))
    {
        // check to see if there is room to write the frame in the buffer. if not, then drop it
        if (len < (CHB_BUF_SZ - bufGetLen()))
        {
            bufWrite(len);

            for (i=0; i<len; i++)
            {
                data = SPI.transfer(0);
                bufWrite(data);
            }
        }
        else
        {
            // read out the data and throw it away
            for (i=0; i<len; i++)
            {
                SPI.transfer(0);
            }

            // Increment the overflow stat, and print a message.
            pcb.overflow++;
        }
    }

    CHB_SPI_DISABLE();
    sei();
}

/**************************************************************************/
/*!
	Chibi buffer management
*/
/**************************************************************************/
// bufWrite
void ChibiArduinoTest::bufWrite(uint8_t data)
{
    if (len < CHB_BUF_SZ)
    {
        ringBuf[wrPtr] = data;
        wrPtr = (wrPtr + 1) % CHB_BUF_SZ;
        len++;
    }
}

// bufRead
uint8_t ChibiArduinoTest::bufRead()
{
    uint8_t data = 0;

    if (len > 0)
    {
        data = ringBuf[rdPtr];
        rdPtr = (rdPtr + 1) % CHB_BUF_SZ;

        // protect the len update. otherwise it could get corrupted if an intp occurs here
        cli();
        len--;
        sei();
    }
    return data;
}

// bufGetLen
uint16_t ChibiArduinoTest::bufGetLen()
{
    return len;
}

// bufGetRemaining
uint16_t ChibiArduinoTest::bufGetRemaining()
{
    return CHB_BUF_SZ - len;
}

/**************************************************************************/
/*!
	EEPROM Handling
*/
/**************************************************************************/
// eepromWrite
void ChibiArduinoTest::eepromWrite(uint16_t addr, uint8_t *buf, uint16_t size)
{
  while(!eeprom_is_ready());
  eeprom_write_block(buf, (uint16_t *)addr, size);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void ChibiArduinoTest::eepromRead(uint16_t addr, uint8_t *buf, uint16_t size)
{
  while(!eeprom_is_ready());
  eeprom_read_block(buf, (uint16_t *)addr, size);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pcb_t *ChibiArduinoTest::getPCB()
{
    return &pcb;
}

/**************************************************************************/
/*!
    This the main ISR for the Chibi radio stack. It doesn't use a standard
    IRQ pin. Instead it uses a general purpose IO configured as an edge-detecting
    interrupt pin. That means that this interrupt needs to qualify the pin
    as a positive rising edge before executing the actual service routine.
*/
/**************************************************************************/
void ChibiArduinoTest::interruptHandler()
{
    uint8_t state, pinval, intp_src = 0;

    // get the pin's value to check whether it was a rising or falling edge.
#if (USE_PINCHANGE_INTP == 1)
    pinval = CHB_INTP_PORT & _BV(CHB_INTP_PIN);
#else
    pinval = 1;
#endif

    // we'll only enter the ISR if the interrupt is a positive edge.  
    if (pinval)
    {
        /*Read Interrupt source.*/
        CHB_SPI_ENABLE();

        /*Send Register address and read register content.*/
        SPI.transfer(IRQ_STATUS | CHB_SPI_CMD_RR);
        intp_src = SPI.transfer(0);

        CHB_SPI_DISABLE();

        while (intp_src)
        {
            /*Handle the incomming interrupt. Prioritized.*/
            if ((intp_src & CHB_IRQ_RX_START_MASK))
            {
                intp_src &= ~CHB_IRQ_RX_START_MASK;
            }
            else if (intp_src & CHB_IRQ_TRX_END_MASK)
            {
                state = getState();

                if ((state == RX_ON) || (state == RX_AACK_ON) || (state == BUSY_RX_AACK))
                {
                    // get the ed measurement
                    pcb.ed = regRead(PHY_ED_LEVEL);

                    // get the crc
                    pcb.crc = (regRead(PHY_RSSI) & (1<<7)) ? 1 : 0;

                    // if the crc is not valid, then do not read the frame and set the rx flag
                    if (pcb.crc)
                    {
                        // get the data
                        frameRead();
                        pcb.rcvd_xfers++;
                        pcb.data_rcv = true;
                    }
                }
                pcb.trx_end = true;
                intp_src &= ~CHB_IRQ_TRX_END_MASK;

                while (getState() != RX_STATE)
                {
                    setState(RX_STATE);
                }
            }
            else if (intp_src & CHB_IRQ_TRX_UR_MASK)
            {
                intp_src &= ~CHB_IRQ_TRX_UR_MASK;
                pcb.underrun++;
            }
            else if (intp_src & CHB_IRQ_PLL_UNLOCK_MASK)
            {
                intp_src &= ~CHB_IRQ_PLL_UNLOCK_MASK;
            }
            else if (intp_src & CHB_IRQ_PLL_LOCK_MASK)
            {
                intp_src &= ~CHB_IRQ_PLL_LOCK_MASK;
            }
            else if (intp_src & CHB_IRQ_BAT_LOW_MASK)
            {
                intp_src &= ~CHB_IRQ_BAT_LOW_MASK;
                pcb.battlow++;
            }
            else
            {
            }
        }
    }
}


/*
// aesInit
void ChibiArduinoTest::aesInit(uint8_t *key)
{

}

// aesEncrypt
uint8_t ChibiArduinoTest::aesEncrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext)
{

}

// aesDecrypt
uint8_t ChibiArduinoTest::aesDecrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext)
{

}

// aesTest
void ChibiArduinoTest::aesTest(uint8_t *key)
{

}	
*/