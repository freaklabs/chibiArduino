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
#pragma once

#define AES_BLOCKSIZE   16
#define AES_KEYSIZE     16
#define AES_ENCRYPT     0
#define AES_DECRYPT     1
#define AES_ECB         0
#define AES_KEY         1
#define AES_CBC         2
#define AES_NO_REQ      0
#define AES_START_REQ   1

#define AES_BASE_ADDR       0x80
#define AES_STATUS          0x82
#define AES_CTRL            0x83
#define AES_STATE_KEY_0     0x84

typedef struct 
{
    bool init;
    uint8_t curr_dir;
    uint8_t cfg;
    uint8_t cfg_mirr;
    uint8_t enc_key[AES_KEYSIZE];
    uint8_t dec_key[AES_KEYSIZE];
} aes_t;

void chb_aes_init(uint8_t *key);
void chb_aes_ctrl_set(uint8_t dir, uint8_t mode, uint8_t request);
uint8_t chb_aes_encrypt(uint8_t mode, uint8_t len, uint8_t *plaintext, uint8_t *ciphertext);
uint8_t chb_aes_decrypt(uint8_t mode, uint8_t len, uint8_t *plaintext, uint8_t *ciphertext);
void chb_aes_wrrd(uint8_t *rdata, uint8_t *wdata);
void chb_aes_write(uint8_t *wdata);
void chb_aes_read(uint8_t *rdata);
void chb_sram_write(uint8_t addr, uint8_t len, uint8_t *wdata);
void chb_sram_read(uint8_t addr, uint8_t len, uint8_t *rdata);
bool chb_aes_err_chk(uint8_t mode, uint8_t len);
void chb_aes_write_key(uint8_t *key);

void chb_aes_test(uint8_t *key);

