/******************************************************************************
 *
 *  Copyright(C), 2015, Xradio Technology Co., Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Filename:      cmd.c
 *
 *  Description:   Brom Protocol for message exchange
 *
 ******************************************************************************/

#define LOG_TAG "bt_cmd"
#include <utils/Log.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#include "type.h"
#include "cmd.h"
#include "userial_vendor.h"

#ifndef BTCMD_DBG
#define BTCMD_DBG FALSE
#endif

#if (BTCMD_DBG == TRUE)
#define BTCMDDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTCMDDBG(param, ...) {}
#endif

#define UNUSED(x)                 (void)(x)
#define SYNC_BAUDRATE_RETRY_NUM   3

#define FILL_HEADER_MAGIC(h) do { \
    (h)->magic[0] = 'B'; \
    (h)->magic[1] = 'R'; \
    (h)->magic[2] = 'O'; \
    (h)->magic[3] = 'M'; \
} while(0)

#define HEADER_MAGIC_VALID(h) ( \
    (h)->magic[0] == 'B' && \
    (h)->magic[1] == 'R' && \
    (h)->magic[2] == 'O' && \
    (h)->magic[3] == 'M' )

#define FILL_HEADER_CHECKSUM(h, cs) do { \
    (h)->checksum = cs; \
    } while(0)


#define CMD_WRITEN       0
#define CMD_WRITESEQ     1
#define CMD_SETBAUD      2
#define CMD_SETPC        3
#define DATA_RAW         4

static u16 checksum16(u8 *data, u32 len)
{
    u16 cs = 0;
    u16 *p = (u16*)data;

    while(len > 1) {
        cs += *p++;
        len -= 2;
    }
    if (len) {
        cs += *(u8 *)p;
    }

    return cs;
}

static uint64_t time_gettimeofday_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static uint8_t *memsearch(uint8_t *haystack, uint32_t hlen, uint8_t *needle, uint32_t nlen)
{
    while (hlen-- >= nlen) {
        if (!memcmp(haystack, needle, nlen)) {
            return haystack;
        }
        haystack++;
    }
    return NULL;
}

static s32 xr_raw_write(int type, uint8_t *data, uint32_t len)
{
    uint8_t  buffer[MB_CMD_HEADER_SIZE + 13] = {'B', 'R', 'O', 'M', 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t *psend = data;
    uint32_t lsend = len;
    cmd_header_t *hdr = (cmd_header_t *)buffer;;
    cmd_ack_t    *ack = (cmd_ack_t *)buffer;

    if (type != DATA_RAW) {
        psend = buffer;
        lsend = MB_CMD_HEADER_SIZE + len;
        memcpy(buffer + MB_CMD_HEADER_SIZE, data, len);
        hdr->payload_len = len;
#if ENABLE_DCS
        hdr->flags       = CMD_HFLAG_CHECK;
#endif
        hdr->checksum    = ~checksum16(buffer, MB_CMD_HEADER_SIZE + len);
        hdr->payload_len = SWAP32(hdr->payload_len);
        hdr->checksum    = SWAP16(hdr->checksum);
        switch (type) {
            case CMD_WRITEN:
                {
                    cmd_rw_t *cmd     = (cmd_rw_t *)buffer;
                    cmd->addr         = SWAP32(cmd->addr);
                }
                break;
            case CMD_WRITESEQ:
                {
                    cmd_seq_wr_t *cmd = (cmd_seq_wr_t *)buffer;
                    cmd->addr         = SWAP32(cmd->addr);
                    cmd->dlen         = SWAP32(cmd->dlen);
                    cmd->dcs          = SWAP16(cmd->dcs);
                }
                break;
            case CMD_SETBAUD:
                {
                    cmd_sys_setuart_t *cmd = (cmd_sys_setuart_t*)buffer;
                    cmd->lcr          = SWAP32(cmd->lcr);
                }
                break;
            case CMD_SETPC:
                {
                    cmd_sys_t *cmd    = (cmd_sys_t*)buffer;
                    cmd->val          = SWAP32(cmd->val);
                }
                break;
            default:
                ALOGE("%s: Unsupport type %d", __func__, type);
                return -1;
        }
    }

    ssize_t ret_w, ret_r;
    uint64_t t0, t1, t2, t3;

    userial_vendor_flush(USERIAL_TCIO_FLUSH);

    t0    = time_gettimeofday_us();
    ret_w = userial_vendor_write(psend, lsend);
    t1    = time_gettimeofday_us();

    if ((uint32_t)ret_w != lsend) {
        ALOGE("%s: write error, (%d != %d)", __func__, ret_w, lsend);
        return -1;
    }

    memset(buffer, 0, MB_CMD_HEADER_SIZE + 1);

    t2    = time_gettimeofday_us();
    ret_r = userial_vendor_read(buffer, MB_CMD_HEADER_SIZE, 100000);
    t3    = time_gettimeofday_us();

    if (ret_r != MB_CMD_HEADER_SIZE) {
        ALOGE("%s: read error, (%d != %d)", __func__, ret_r, MB_CMD_HEADER_SIZE);
        return -1;
    }

    ALOGD("%s, type: %d, write len: %5d, time: %6lluus, read len: %2d, time: %6lluus",
               __func__, type, lsend, t1 - t0, MB_CMD_HEADER_SIZE, t3 - t2);

    uint8_t *p = (uint8_t *)memsearch(buffer, MB_CMD_HEADER_SIZE, (uint8_t *)"BROM", 4);
    if (p != buffer) {
        if (p == NULL) {
            ALOGE("%s: invalid response", __func__);
            return -1;
        }
        uint32_t nowread  = buffer + MB_CMD_HEADER_SIZE - p;
        uint32_t needread = p - buffer;
        ALOGW("%s: Index error, re-find header magic", __func__);
        memcpy(buffer, p, nowread);
        memset(buffer + nowread, 0x0, needread);
        userial_vendor_read(buffer + nowread, needread, 100000);
    }

    /* check response */
    if (ack->h.flags & CMD_HFLAG_ERROR) {
        userial_vendor_read(buffer + MB_CMD_HEADER_SIZE, 1, 100000);
        ALOGE("%s: resp error flag, type %d", __func__, ack->err);
        return -ack->err;
    }

    if (ack->h.flags & CMD_HFLAG_ACK) {
        /* convert network byte order to host byte order */
        ack->h.payload_len = SWAP32(ack->h.payload_len);
        ack->h.checksum    = SWAP16(ack->h.checksum);
        if (ack->h.payload_len != 0) {
            ALOGE("%s: data payload len %d != 0", __func__, ack->h.payload_len);
            return -1;
        }
    }

    if (ack->h.flags & CMD_HFLAG_CHECK) {
        if (checksum16(buffer, MB_CMD_HEADER_SIZE) != 0xffff) {
            ALOGE("%s: write data response 0 checksum error", __func__);
            return -1;
        }
    }
    return 0;
}

s32 cmd_sync_uart(void)
{
    uint8_t  sync   = 0x55;
    uint8_t  ack[3] = {0};
    ssize_t  ret = -1;
    uint32_t cnt = 0;
    uint64_t t0, t1, t2;

    t0  = time_gettimeofday_us();
    do {
        BTCMDDBG("uart sync count:%d.", cnt);
        userial_vendor_flush(USERIAL_TCIO_FLUSH);
        userial_vendor_write(&sync, 1);

        ack[0] = ack[1] = 0;

        t1  = time_gettimeofday_us();
        ret = userial_vendor_read(ack, 2, 8000);
        t2  = time_gettimeofday_us();

        if ((ret == 2) && (ack[0] == 'O' && ack[1] == 'K')) {
            BTCMDDBG("Receive \"%s\", time: %5lluus, total: %6lluus, uart sync done.", ack, t2 - t1, t2 - t0);
            return 0;
        }
    } while (cnt++ < 50);

    BTCMDDBG("uart sync fail.");
    return -1;
}

s32 cmd_sync_baud(u32 lcr)
{
    u8  buffer[MB_CMD_HEADER_SIZE + 5];
    cmd_sys_setuart_t *cmd = (cmd_sys_setuart_t*)buffer;

    cmd->cmdid = CMD_ID_SETUART;
    cmd->lcr   = lcr;

    u8  cnt = 0;
    int ret = -1;

    do {
        BTCMDDBG("%s count:%d.", __func__, cnt);
        ret = xr_raw_write(CMD_SETBAUD, buffer + MB_CMD_HEADER_SIZE, 5);
        if (ret == 0) {
            userial_vendor_set_baud(line_speed_to_userial_baud(lcr & 0xffffff));  // low 24 bits
            return cmd_sync_uart();
        }
    } while (cnt++ < SYNC_BAUDRATE_RETRY_NUM);

    BTCMDDBG("cmd_sync_baud fail.");
    return -1;
}

s32 cmd_write_seq(u32 addr, u32 len, u8 *data)
{
    int ret = -1;
    u8 buffer[MB_CMD_HEADER_SIZE + 13];
    cmd_seq_wr_t *cmd = (cmd_seq_wr_t *)buffer;

    cmd->cmdid = CMD_ID_SEQWR;
    cmd->addr  = addr;
    cmd->dlen  = len;
#if ENABLE_DCS
    cmd->dcs   = ~checksum16(data, len);
#endif

    ret = xr_raw_write(CMD_WRITESEQ, buffer + MB_CMD_HEADER_SIZE, 11);
    if (ret == 0) {
        return xr_raw_write(DATA_RAW, data, len);
    }
    return ret;
}

s32 cmd_write_byte(u32 addr, u32 len, u8 *data)
{
    u8 buffer[MB_CMD_HEADER_SIZE + 13];
    cmd_rw_t *cmd = (cmd_rw_t *)buffer;

    cmd->cmdid = len == 1 ? CMD_ID_WRITE1 :
                 len == 2 ? CMD_ID_WRITE2 :
                 len == 4 ? CMD_ID_WRITE4 :
                 len == 8 ? CMD_ID_WRITE8 : CMD_ID_WRITE4;

    cmd->addr  = addr;
    memcpy(buffer + sizeof(cmd_rw_t), data, len);

    return xr_raw_write(CMD_WRITEN, buffer + MB_CMD_HEADER_SIZE, 5 + len);
}

s32 cmd_set_pc(u32 pc)
{
    u8 buffer[MB_CMD_HEADER_SIZE + 5];
    cmd_sys_t *cmd = (cmd_sys_t*)buffer;

    cmd->cmdid = CMD_ID_SETPC;
    cmd->val   = pc;
    BTCMDDBG("set pc %x, val %x", pc, cmd->val);

    return xr_raw_write(CMD_SETPC, buffer + MB_CMD_HEADER_SIZE, 5);
}

