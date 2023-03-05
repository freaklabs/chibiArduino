#pragma once

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

// For handling Arduino 1.0 compatibility and backwards compatibility
#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <SPI.h>
#include <avr/eeprom.h>
#include "chibiUsrCfg.h"

#define BROADCAST_ADDR 0xFFFF

#define CHB_HDR_SZ        9    // FCF + seq + pan_id + dest_addr + src_addr (2 + 1 + 2 + 2 + 2)
#define CHB_FCS_LEN       2

// frame_type = data
// security enabled = false
// frame pending = false
// ack request = false
// pan ID compression = true
#define CHB_FCF_BYTE_0    0x41    

// dest addr = 16-bit
// frame version = 802.15.4 (not 2003)
// src addr = 16-bit
#define CHB_FCF_BYTE_1    0x98

#define CHB_ACK_REQ_POS   5

#if (ILLUMINADO_ATMEGA4808 == 1)
    #define CHB_SPI_ENABLE()    digitalWriteFast(PIN_CSN, LOW)
    #define CHB_SPI_DISABLE()   digitalWriteFast(PIN_CSN, HIGH)
    #define CHB_SLPTR_ENABLE()  
    #define CHB_SLPTR_DISABLE()
#else
    #define CHB_SPI_ENABLE()    do {CHB_SPI_CS_PORT &= ~(_BV(CHB_SPI_CS_PIN));} while (0)
    #define CHB_SPI_DISABLE()   do {CHB_SPI_CS_PORT |= _BV(CHB_SPI_CS_PIN);} while (0)
    #define CHB_SLPTR_ENABLE()  do {CHB_SLPTR_PORT |= (_BV(CHB_SLPTR_PIN));} while (0)
    #define CHB_SLPTR_DISABLE() do {CHB_SLPTR_PORT &= ~(_BV(CHB_SLPTR_PIN));} while (0)

    #if defined(__AVR_ATmega1284P__)
        #define CHB_SPI_PORT    PORTB
        #define CHB_SPI_DDIR    DDRB
        #define CHB_SCK         7                 // PB.5 - Output: SPI Serial Clock (SCLK)
        #define CHB_MOSI        5                 // PB.3 - Output: SPI Master out - slave in (MOSI)
        #define CHB_MISO        6                 // PB.4 - Input:  SPI Master in - slave out (MISO)
        #define CHB_SPI_SELN    4                 // PB.2 - Input: The dedicated SPI CS pin needs to have internal pullup enabled if an input
    #else
        /* Note: The SPI chip select pin is defined in chibiUsrCfg.h */
        #define CHB_SPI_PORT    PORTB
        #define CHB_SPI_DDIR    DDRB
        #define CHB_SCK         5                 // PB.5 - Output: SPI Serial Clock (SCLK)
        #define CHB_MOSI        3                 // PB.3 - Output: SPI Master out - slave in (MOSI)
        #define CHB_MISO        4                 // PB.4 - Input:  SPI Master in - slave out (MISO)
        #define CHB_SPI_SELN    2                 // PB.2 - Input: The dedicated SPI CS pin needs to have internal pullup enabled if an input
    #endif
#endif

#define CHB_CC1190_PRESENT      0       /// Set to 1 if CC1190 is being used
#define CHB_CHINA               0       /// Support 780 MHz Chinese band        

#define CHB_SPI_CMD_RW      0xC0    /**<  Register Write (short mode). */
#define CHB_SPI_CMD_RR      0x80    /**<  Register Read (short mode). */
#define CHB_SPI_CMD_FW      0x60    /**<  Frame Transmit Mode (long mode). */
#define CHB_SPI_CMD_FR      0x20    /**<  Frame Receive Mode (long mode). */
#define CHB_SPI_CMD_SW      0x40    /**<  SRAM Write. */
#define CHB_SPI_CMD_SR      0x00    /**<  SRAM Read. */
#define CHB_SPI_CMD_RADDRM  0x7F    /**<  Register Address Mask. */

#define CHB_IRQ_BAT_LOW_MASK        0x80  /**< Mask for the BAT_LOW interrupt. */
#define CHB_IRQ_TRX_UR_MASK         0x40  /**< Mask for the TRX_UR interrupt. */
#define CHB_IRQ_TRX_END_MASK        0x08  /**< Mask for the TRX_END interrupt. */
#define CHB_IRQ_RX_START_MASK       0x04  /**< Mask for the RX_START interrupt. */
#define CHB_IRQ_PLL_UNLOCK_MASK     0x02  /**< Mask for the PLL_UNLOCK interrupt. */
#define CHB_IRQ_PLL_LOCK_MASK       0x01  /**< Mask for the PLL_LOCK interrupt. */
    
#define CHB_ENTER_CRIT()    {U8 volatile saved_sreg = SREG; cli()
#define CHB_LEAVE_CRIT()    SREG = saved_sreg;}

#define MAX_RETRIES 10

/**************************************************************************/
/*!
    If promiscuous mode is enabled, then we don't want to use the extended
    hardware features of the chip. We want raw frames that are unacknowledged.
*/
/**************************************************************************/
#if (CHIBI_PROMISCUOUS)
    #define RX_STATE RX_ON
#else
    #define RX_STATE RX_AACK_ON
#endif


enum
{
    CCA_ED                    = 1,    /**< Use energy detection above threshold mode. */
    CCA_CARRIER_SENSE         = 2,    /**< Use carrier sense mode. */
    CCA_CARRIER_SENSE_WITH_ED = 3     /**< Use a combination of both energy detection and carrier sense. */
};

/**************************************************************************/
/*!
    This enumeration contains all the register addresses available in the
    Atmel AT86RF230 radio.
*/
/**************************************************************************/
enum
{
    TRX_STATUS      = 0x01,
    TRX_STATE       = 0x02,
    TRX_CTRL0       = 0x03,
    TRX_CTRL1       = 0x04,
    PHY_TX_PWR      = 0x05,
    PHY_RSSI        = 0x06,
    PHY_ED_LEVEL    = 0x07,
    PHY_CC_CCA      = 0x08,
    CCA_THRES       = 0x09,
    RX_CTRL         = 0x0a,
    SFD_VALUE       = 0x0b,
    TRX_CTRL_2      = 0x0c,
    ANT_DIV         = 0x0d,
    IRQ_MASK        = 0x0e,
    IRQ_STATUS      = 0x0f,
    VREG_CTRL       = 0x10,
    BATMON          = 0x11,
    XOSC_CTRL       = 0x12,
    CC_CTRL_0       = 0x13,
    CC_CTRL_1       = 0x14,
    RX_SYN          = 0x15,
    RF_CTRL_0       = 0x16,
    XAH_CTRL_1      = 0x17,
    FTN_CTRL        = 0x18,
    RF_CTRL_1       = 0x19,
    PLL_CF          = 0x1a,
    PLL_DCU         = 0x1b,
    PART_NUM        = 0x1c,
    VERSION_NUM     = 0x1d,
    MAN_ID_0        = 0x1e,
    MAN_ID_1        = 0x1f,
    SHORT_ADDR_0    = 0x20,
    SHORT_ADDR_1    = 0x21,
    PAN_ID_0        = 0x22,
    PAN_ID_1        = 0x23,
    IEEE_ADDR_0     = 0x24,
    IEEE_ADDR_1     = 0x25,
    IEEE_ADDR_2     = 0x26,
    IEEE_ADDR_3     = 0x27,
    IEEE_ADDR_4     = 0x28,
    IEEE_ADDR_5     = 0x29,
    IEEE_ADDR_6     = 0x2a,
    IEEE_ADDR_7     = 0x2b,
    XAH_CTRL_0      = 0x2c,
    CSMA_SEED_0     = 0x2d,
    CSMA_SEED_1     = 0x2e,
    CSMA_BE         = 0x2f
};

enum
{
    CHB_MAX_FRAME_RETRIES_POS   = 4,
    CHB_MAX_CSMA_RETRIES_POS    = 1,
    CHB_MIN_BE_POS              = 6,
    CHB_CSMA_SEED1_POS          = 0,
    CHB_CCA_MODE_POS            = 5,

    CHB_AUTO_CRC_POS            = 7, // AT86RF23x
//    CHB_AUTO_CRC_POS            = 5, // AT86RF212    
    CHB_RX_SAFE_MODE            = 7,
    CHB_TRX_END_POS             = 3,
    CHB_TRAC_STATUS_POS         = 5,
    CHB_FVN_POS                 = 6,
    CHB_OQPSK_TX_OFFSET         = 2,
    CHB_BPSK_TX_OFFSET          = 3,
    CHB_MIN_FRAME_LENGTH        = 3,
    CHB_MAX_FRAME_LENGTH        = 0x7f,
    CHB_PA_EXT_EN_POS           = 7
};

enum{
    TIME_TO_ENTER_P_ON               = 510,     /**<  Transition time from VCC is applied to P_ON. */
    TIME_P_ON_TO_TRX_OFF             = 510,     /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 880,     /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_RESET                       = 6,       /**<  Time to hold the RST pin low during reset */
    TIME_ED_MEASUREMENT              = 140,     /**<  Time it takes to do a ED measurement. */
    TIME_CCA                         = 140,     /**<  Time it takes to do a CCA. */
    TIME_PLL_LOCK                    = 150,     /**<  Maximum time it should take for the PLL to lock. */
    TIME_FTN_TUNING                  = 25,      /**<  Maximum time it should take to do the filter tuning. */
    TIME_NOCLK_TO_WAKE               = 6,       /**<  Transition time from *_NOCLK to being awake. */
    TIME_CMD_FORCE_TRX_OFF           = 1,       /**<  Time it takes to execute the FORCE_TRX_OFF command. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180,     /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,       /**<  Transition time from PLL active state to another. */
};

// These are the status codes that are used in this stack
enum{
    RADIO_SUCCESS = 0x40,                       /**< The requested service was performed successfully. */
    RADIO_UNSUPPORTED_DEVICE,                   /**< The connected device is not an Atmel AT86RF230. */
    RADIO_INVALID_ARGUMENT,                     /**< One or more of the supplied function arguments are invalid. */
    RADIO_TIMED_OUT,                            /**< The requested service timed out. */
    RADIO_WRONG_STATE,                          /**< The end-user tried to do an invalid state transition. */
    RADIO_BUSY_STATE,                           /**< The radio transceiver is busy receiving or transmitting. */
    RADIO_STATE_TRANSITION_FAILED,              /**< The requested state transition could not be completed. */
    RADIO_CCA_IDLE,                             /**< Channel is clear, available to transmit a new frame. */
    RADIO_CCA_BUSY,                             /**< Channel busy. */
    RADIO_TRX_BUSY,                             /**< Transceiver is busy receiving or transmitting data. */
    RADIO_BAT_LOW,                              /**< Measured battery voltage is lower than voltage threshold. */
    RADIO_BAT_OK,                               /**< Measured battery voltage is above the voltage threshold. */
    RADIO_CRC_FAILED,                           /**< The CRC failed for the actual frame. */
    RADIO_CHANNEL_ACCESS_FAILURE,               /**< The channel access failed during the auto mode. */
    RADIO_NO_ACK,                               /**< No acknowledge frame was received. */
};

enum
{
    CMD_NOP                  = 0,
    CMD_TX_START             = 2,
    CMD_FORCE_TRX_OFF        = 3,
    CMD_FORCE_PLL_ON         = 4,
    CMD_RX_ON                = 6,
    CMD_TRX_OFF              = 8,
    CMD_PLL_ON               = 9,
    CMD_RX_AACK_ON           = 22,
    CMD_TX_ARET_ON           = 25
};

enum
{
    P_ON                = 0,
    BUSY_RX             = 1,
    BUSY_TX             = 2,
    RX_ON               = 6,
    TRX_OFF             = 8,
    PLL_ON              = 9,
    SLEEP               = 15,
    BUSY_RX_AACK        = 17,
    BUSY_TX_ARET        = 18,
    RX_AACK_ON          = 22,
    TX_ARET_ON          = 25,
    RX_ON_NOCLK         = 28,
    RX_AACK_ON_NOCLK    = 29,
    BUSY_RX_AACK_NOCLK  = 30,
    TRANS_IN_PROG       = 31
};

// transceiver modes for AT86RF212
enum
{
    OQPSK_SINRC_100 = 0,
    OQPSK_SIN_250   = 1,
    OQPSK_RC        = 2,
    OQPSK_RC_250    = 3,
    OQPSK_SIN_500   = 4,
    OQPSK_SIN_1000  = 5,
    BPSK_40         = 6,
    BPSK_20         = 7,
    OQPSK_250       = 8,
    OQPSK_500       = 9,
    OQPSK_1000      = 10,
    OQPSK_2000      = 11
};

// part numbers for atmel radios
enum
{
    CHB_AT86RF230   = 2,
    CHB_AT86RF212   = 7,
    CHB_AT86RF231   = 3
};

// Data rate enums
enum
{
    CHB_RATE_250KBPS =  0,
    CHB_RATE_500KBPS =  1,
    CHB_RATE_1000KBPS = 2,
    CHB_RATE_2000KBPS = 3
};

enum
{
    CHB_SUCCESS                 = 0,
    CHB_SUCCESS_DATA_PENDING    = 1,
    CHB_CHANNEL_ACCESS_FAILURE  = 3,
    CHB_NO_ACK                  = 5,
    CHB_INVALID                 = 7
};

typedef struct
{
    uint16_t src_addr;
    uint8_t seq;
    volatile bool data_rcv;
    volatile bool trx_end;

    // stats
    uint16_t rcvd_xfers;
    uint16_t txd_success;
    uint16_t txd_noack;
    uint16_t txd_channel_fail;
    uint16_t overflow;
    uint16_t underrun;
    uint8_t battlow;
    uint8_t status;
    uint8_t ed;
    uint8_t crc;
} pcb_t;

typedef struct
{
    uint8_t len;
    uint16_t src_addr;
    uint16_t dest_addr;
    uint8_t *data;
} chb_rx_data_t;

class ChibiArduinoTest
{
public:
	uint8_t pinCs;
	uint8_t pinSlpTr;

	ChibiArduinoTest(uint8_t cs, uint8_t slptr);
	void begin();
	void setShortAddr(uint16_t addr);
	uint16_t getShortAddr();
	void setIEEEAddr(uint8_t *ieee_addr);
	void getIEEEAddr(uint8_t *ieee_addr);
	uint8_t regRead(uint8_t addr);
    uint8_t read(chb_rx_data_t *rx);
    uint8_t send(uint16_t addr, uint8_t *data, uint8_t len);
	void regWrite(uint8_t addr, uint8_t val);
	uint8_t dataRcvd();
	uint8_t getData(uint8_t *data);
	uint8_t getRSSI();
	uint16_t getSrcAddr();
	uint8_t setChannel(uint8_t channel);
	uint8_t getChannel();
	uint8_t getPartID();
	void sleepRadio(uint8_t enb);
	void setDataRate(uint8_t rate);
	uint8_t getRand();
	void setMode(uint8_t mode);
	uint8_t getMode();
	void setRetries(uint8_t);
    void highGainModeEnable();
    void highGainModeDisable();	
    pcb_t *getPCB();
    void interruptHandler();

	//void aesInit(uint8_t *key);
	//uint8_t aesEncrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext);
	//uint8_t aesDecrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext);
    //void aesTest(uint8_t *key); 

private:
	uint8_t radioInit();
    uint8_t tx(uint8_t *hdr, uint8_t *data, uint8_t len);
	uint8_t setState(uint8_t state);
	uint8_t getState();
	uint8_t getStatus();
	void delayUS(uint16_t usec);
	uint16_t regRead16(uint8_t addr);
	void regWrite16(uint8_t addr, uint16_t val);
	void regWrite64(uint8_t addr, uint8_t *val);
	void regReadModWrite(uint8_t addr, uint8_t val, uint8_t mask);
	void frameWrite(uint8_t *hdr, uint8_t hdr_len, uint8_t *data, uint8_t data_len);
	void frameRead();
	uint8_t genHeader(uint8_t *hdr, uint16_t addr, uint8_t len);
	void bufWrite(uint8_t data);
	uint8_t bufRead();
	uint16_t bufGetLen();
	uint16_t bufGetRemaining();
	void eepromWrite(uint16_t addr, uint8_t *buf, uint16_t size);
	void eepromRead(uint16_t addr, uint8_t *buf, uint16_t size);
    //void spiInit();
    //uint8_t spiTransfer(uint8_t data);
};