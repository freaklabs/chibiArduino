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
#ifndef CHIBI_H
#define CHIBI_H

// For handling Arduino 1.0 compatibility and backwards compatibility
#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "chibiUsrCfg.h"

#define BROADCAST_ADDR 0xFFFF

void chibiInit();
void chibiSetShortAddr(uint16_t addr);
uint16_t chibiGetShortAddr();
void chibiSetIEEEAddr(uint8_t *ieee_addr);
void chibiGetIEEEAddr(uint8_t *ieee_addr);
uint8_t chibiRegRead(uint8_t addr);
void chibiRegWrite(uint8_t addr, uint8_t val);
uint8_t chibiTx(uint16_t addr, uint8_t *data, uint8_t len);
uint8_t chibiDataRcvd();
uint8_t chibiGetData(uint8_t *data);
uint8_t chibiGetRSSI();
uint16_t chibiGetSrcAddr();
uint8_t chibiSetChannel(uint8_t channel);
uint8_t chibiGetChannel();
uint8_t chibiGetPartID();
void chibiSleepRadio(uint8_t enb);
void chibiCmdInit(uint32_t speed);
void chibiCmdPoll();
void chibiCmdAdd(char *name, void (*func)(int argc, char **argv));
uint32_t chibiCmdStr2Num(char *str, uint8_t base);

void chibiAesInit(uint8_t *key);
uint8_t chibiAesEncrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext);
uint8_t chibiAesDecrypt(uint8_t len, uint8_t *plaintext, uint8_t *ciphertext);
void chibiSetDataRate(uint8_t rate);
uint8_t chibiGetRand();
void chibiSetMode(uint8_t mode);
uint16_t chibiBufGetRemaining();
void chibiSetRetries(uint8_t);

void chibiAesTest(uint8_t *key);

#if ((FREAKDUINO_LONG_RANGE == 1) || (SABOTEN == 1) || (ARASHI_ENET_GATEWAY_LR == 1) || (FREAKDUINO1284PLR == 1) || (FREAKUSB1284PLR == 1))
    void chibiHighGainModeEnable();
    void chibiHighGainModeDisable();
#endif

#endif

