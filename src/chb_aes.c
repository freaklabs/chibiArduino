/*******************************************************************
    Copyright (C) 2013 FreakLabs
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
#include "chb_aes.h"
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

static aes_t aes;

const char chb_err_aes_init[] PROGMEM = "AES_ERROR: You must initialize the AES engine first.\n";
const char chb_err_aes_len[] PROGMEM = "AES_ERROR: Length must be multiple of 16 bytes.\n";
const char chb_err_aes_mode[] PROGMEM = "AES_ERROR: Only ECB mode is currently supported.\n";
const char chb_err_aes_partnum[] PROGMEM = "AES_ERROR: This part does not support AES encryption.\n";

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_aes_write_key(uint8_t *key)
{
    // direction doesn't really matter here
    chb_aes_ctrl_set(aes.curr_dir, AES_KEY, AES_NO_REQ);
    chb_aes_write(key);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_aes_ctrl_set(uint8_t dir, uint8_t mode, uint8_t request)
{
    aes.curr_dir = dir;
    aes.cfg = aes.cfg_mirr = (mode << 4) | (dir << 3);
    aes.cfg_mirr |= (request << 7);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_aes_wrrd(uint8_t *rdata, uint8_t *wdata)
{
    uint8_t i;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();
    
    /*send SRAM write command.*/
    chb_xfer_byte(CHB_SPI_CMD_SW);

    /*send address where to start writing to.*/
    chb_xfer_byte(AES_CTRL);

    /*send config for AES module*/
    chb_xfer_byte(aes.cfg);
    
    // write one byte first, then start loop according to fast sram sequence in datasheet
    chb_xfer_byte(*wdata++);
    for (i=0; i<AES_BLOCKSIZE-1; i++)
    {
        *rdata++ = chb_xfer_byte(*wdata++);
    }

    // load the config mirror data. this is where we'd set the request bit
    *rdata = chb_xfer_byte(aes.cfg_mirr);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_aes_write(uint8_t *wdata)
{
    uint8_t i;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM write command.*/
    chb_xfer_byte(CHB_SPI_CMD_SW);

    /*Send address where to start writing to.*/
    chb_xfer_byte(AES_CTRL);

    // load the cfg data
    chb_xfer_byte(aes.cfg);

    for (i=0; i<AES_BLOCKSIZE; i++)
    {
        chb_xfer_byte(*wdata++);
    }

    // load the config mirror data. this is where we'd set the request bit
    chb_xfer_byte(aes.cfg_mirr);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_aes_read(uint8_t *rdata)
{
    uint8_t i;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM read command.*/
    chb_xfer_byte(CHB_SPI_CMD_SR);

    /*Send address where to start writing to.*/
    chb_xfer_byte(AES_CTRL);

    // load the cfg data
    chb_xfer_byte(aes.cfg);

    for (i=0; i<AES_BLOCKSIZE; i++)
    {
        *rdata++ = chb_xfer_byte(0);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_sram_write(uint8_t addr, uint8_t len, uint8_t *wdata)
{
    uint8_t i;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM read command.*/
    chb_xfer_byte(CHB_SPI_CMD_SW);

    /*Send address where to start writing to.*/
    chb_xfer_byte(addr);

    for (i=0; i<len; i++)
    {
        chb_xfer_byte(*wdata++);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_sram_read(uint8_t addr, uint8_t len, uint8_t *rdata)
{
    uint8_t i;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM read command.*/
    chb_xfer_byte(CHB_SPI_CMD_SR);

    /*Send address where to start writing to.*/
    chb_xfer_byte(addr);

    for (i=0; i<len; i++)
    {
        *rdata++ = chb_xfer_byte(0);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
bool chb_aes_err_chk(uint8_t mode, uint8_t len)
{
    char buf[80];

    if (!aes.init)
    {
        // grab the error message from flash & print it out
        strcpy_P(buf, chb_err_aes_init);
        Serial.print(buf);
        return true;
    }
    if (len % 16 != 0)
    {
        // grab the error message from flash & print it out
        strcpy_P(buf, chb_err_aes_len);
        Serial.print(buf);
        return true;
    }
    else if (mode != AES_ECB)
    {
        // grab the error message from flash & print it out
        strcpy_P(buf, chb_err_aes_mode);
        Serial.print(buf);
        return true;
    }
    return false;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_aes_init(uint8_t *key)
{
    uint8_t partnum, dummy[AES_BLOCKSIZE];

    partnum = chb_get_part_num();

    if ((partnum != CHB_AT86RF212) && (partnum != CHB_AT86RF231))
    {
        char buf[60];

        // grab the error message from flash & print it out
        strcpy_P(buf, chb_err_aes_partnum);
        Serial.print(buf);
        return;
    }

    // we've already been initialized
    if (aes.init) return;

    // copy the encryption key and then load it into the engine
    memcpy(aes.enc_key, key, AES_KEYSIZE);
    chb_aes_write_key(key);

    // generate the decryption key
    chb_aes_ctrl_set(AES_ENCRYPT, AES_ECB, AES_START_REQ);
    chb_aes_write(dummy);

    chb_aes_ctrl_set(AES_DECRYPT, AES_KEY, AES_NO_REQ);

    // need to write a block before reading due to pipelined sram access in rf231
    chb_aes_write(dummy);
    chb_aes_read(aes.dec_key);

    // set up the engine for encryption. if we need to decrypt,
    // then we'll take care of that in the decrypt routine
    aes.curr_dir = AES_ENCRYPT;
    chb_aes_write_key(key);
    aes.init = true;
}   

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t chb_aes_encrypt(uint8_t mode, uint8_t len, uint8_t *plaintext, uint8_t *ciphertext)
{
    uint8_t i, iter, *rdata, *wdata;

    // make sure everything's kosher
    if (chb_aes_err_chk(mode, len)) return 1;

    // check to see if we need to load the key
    if (aes.curr_dir != AES_ENCRYPT)
    {
        aes.curr_dir = AES_ENCRYPT;
        chb_aes_write_key(aes.enc_key);
    }

    // init pointers to make housekeeping easier
    wdata = plaintext;      // data we'll be writing
    rdata = ciphertext;     // data we'll be reading
    
    // write data to aes engine
    chb_aes_ctrl_set(aes.curr_dir, AES_ECB, AES_START_REQ);
    chb_aes_write(wdata);

    // move the plaintext pointer forward since we loaded it already
    wdata += AES_BLOCKSIZE;
    len -= 16;

    // loop through remaining data
    iter = len/16;
    for (i=0; i<iter; i++)
    {
        chb_aes_wrrd(rdata, wdata);

        // move the pointers forward
        rdata += AES_BLOCKSIZE;
        wdata += AES_BLOCKSIZE;
    }
    
    // perform the final read for the ciphertext
    chb_aes_read(rdata);
    return 0;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t chb_aes_decrypt(uint8_t mode, uint8_t len, uint8_t *plaintext, uint8_t *ciphertext)
{
    uint8_t i, iter, *rdata, *wdata;

    // make sure everything's kosher
    if (chb_aes_err_chk(mode, len)) return 1;

    // check to see if we need to load the key
    if (aes.curr_dir != AES_DECRYPT)
    {
        aes.curr_dir = AES_DECRYPT;
        chb_aes_write_key(aes.dec_key);
    }

    // init pointers to make housekeeping easier
    wdata = ciphertext;      // data we'll be writing
    rdata = plaintext;     // data we'll be reading
    
    // write data to aes engine
    chb_aes_ctrl_set(AES_DECRYPT, AES_ECB, AES_START_REQ);
    chb_aes_write(wdata);

    // move the plaintext pointer forward since we loaded it already
    wdata += AES_BLOCKSIZE;

    // loop through remaining data
    iter = len / 16;
    for (i=0; i<iter; i++)
    {
        chb_aes_wrrd(rdata, wdata);

        // move the pointers forward
        rdata += AES_BLOCKSIZE;
        wdata += AES_BLOCKSIZE;
    }
    
    // perform the final read for the ciphertext
    chb_aes_read(rdata);
    return 0;
}
