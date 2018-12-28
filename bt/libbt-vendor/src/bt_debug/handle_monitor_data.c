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
 *  Filename:      handle_mointor_data.c
 *
 *  Description:   handle raw data from debugfs
 *
 ******************************************************************************/
#define LOG_TAG "handle_mointor_data"
#include <unistd.h>
#include <sys/inotify.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include "bt_fdi.h"
#include <utils/Log.h>

#define FDIHANMON_DBG FALSE

#if (FDIHANMON_DBG == TRUE)
#define BTHADMON(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTHADMON(param, ...) {}
#endif

void handle_ctl_reg(void *data)
{
    int data_len;
    char *p = data;
    reg_data rdata;
    data_len = strlen(data);
    char raw_addr[RAW_REG_ADDR_LEN + 1];
    char raw_data[RAW_REG_DATA_LEN + 1];
    memset(raw_addr, 0, RAW_REG_ADDR_LEN + 1);
    memset(raw_data, 0, RAW_REG_DATA_LEN + 1);
    unsigned int uaddr;
    unsigned int udata;

    /*read*/
    if (*p == 'r') {
        do {
            p++;
        } while ((*p) == ' ');
        memset(raw_addr, 0, RAW_REG_ADDR_LEN);
        memcpy(raw_addr, p, RAW_REG_ADDR_LEN);
        raw_addr[RAW_REG_ADDR_LEN] = '\0';
        uaddr = strtoul(raw_addr, NULL, 0);
        rdata.addr[0] = (uaddr & 0x000000FF) >> 0;
        rdata.addr[1] = (uaddr & 0x0000FF00) >> 8;
        rdata.addr[2] = (uaddr & 0x00FF0000) >> 16;
        rdata.addr[3] = (uaddr & 0xFF000000) >> 24;
        read_reg_data(&rdata);
    /*write*/
    } else if (*p == 'w') {
        do {
            p++;
        } while ((*p) == ' ');

        memcpy(raw_addr, p, RAW_REG_ADDR_LEN);
        raw_addr[RAW_REG_ADDR_LEN] = '\0';
        uaddr = strtoul(raw_addr, NULL, 0);
        rdata.addr[0] = (uaddr & 0x000000FF) >> 0;
        rdata.addr[1] = (uaddr & 0x0000FF00) >> 8;
        rdata.addr[2] = (uaddr & 0x00FF0000) >> 16;
        rdata.addr[3] = (uaddr & 0xFF000000) >> 24;
        p += RAW_REG_ADDR_LEN;
        do {
            p++;
        } while (*p == ' ');

        rdata.data_len = 4;
        memcpy(raw_data, p, (data_len - (p - (char *)data)));
        raw_data[data_len - (p - (char *)data)] = '\0';
        udata = strtoul(raw_data, NULL, 0);
        rdata.data[0] = (udata & 0x000000FF) >> 0;
        rdata.data[1] = (udata & 0x0000FF00) >> 8;
        rdata.data[2] = (udata & 0x00FF0000) >> 16;
        rdata.data[3] = (udata & 0xFF000000) >> 24;
        BTHADMON("data:0x%08X\n", udata);
        write_reg_data(&rdata);
    } else {
        BTHADMON("command type error\n");
    }
}

void handle_ctl_tag(void *data)
{
    int data_len;
    char *p = data;
    data_len = strlen(data);
    tag_data rdata;
    char raw_id[RAW_TAG_ID_LEN + 1];
    memset(raw_id, 0, RAW_TAG_ID_LEN + 1);
    BTHADMON("type:%c\n", *p);

    /* read */
    if (*p == 'r') {
        do {
            p++;
        } while (*p == ' ');

        memcpy(raw_id, p, RAW_TAG_ID_LEN);
        raw_id[RAW_TAG_ID_LEN] = '\0';
        rdata.id = strtoul(raw_id, NULL, 0);
        if (rdata.id != 0) {
            read_tag_data(&rdata);
        } else {
            BTHADMON("Illeagle tag id!\n");
        }
    /* write */
    } else if (*p == 'w') {
        do {
            p++;
        } while (*p == ' ');

        memcpy(raw_id, p, RAW_TAG_ID_LEN);
        raw_id[RAW_TAG_ID_LEN] = '\0';
        rdata.id = strtoul(raw_id, NULL, 0);
        if (rdata.id != 0) {
            p += RAW_TAG_ID_LEN;
            do {
                p++;
            } while (*p == ' ');

            rdata.data_len = (data_len - (p - (char *)data));
            memcpy(rdata.data, p, rdata.data_len);
            BTHADMON("data:%s\n", rdata.data);
            write_tag_data(&rdata);
        } else {
            BTHADMON("Illeagle tag id!\n");
        }
    /* delete */
    } else if (*p == 'd') {
        do {
            p++;
        } while (*p == ' ');

        memcpy(raw_id, p, RAW_TAG_ID_LEN);
        raw_id[RAW_TAG_ID_LEN] = '\0';
        rdata.id = strtoul(raw_id, NULL, 0);
        if (rdata.id != 0) {
            delete_tag_data(&rdata);
        } else {
            BTHADMON("Illeagle tag id!\n");
        }
    /* lock */
    } else if (*p == 'l') {
        do {
            p++;
        } while (*p == ' ');

        memcpy(raw_id, p, RAW_TAG_ID_LEN);
        raw_id[RAW_TAG_ID_LEN] = '\0';
        rdata.id = strtoul(raw_id, NULL, 0);
        if (rdata.id != 0) {
            p += RAW_TAG_ID_LEN;
            do {
                p++;
            } while (*p == ' ');

            if (*p == '0') {
                rdata.data_len = 0;//data_len should be used as lock value
                BTHADMON("lock:no\n");
                lock_tag_data(&rdata);
            } else if (*p == '1') {
                rdata.data_len = 1;//data_len should be used as lock value
                BTHADMON("lock:yes\n");
                lock_tag_data(&rdata);
            } else {
                BTHADMON("Illeagle lock para, it should be 0 or 1.\n");
            }
        } else {
            BTHADMON("Illeagle tag id!\n");
        }
    } else {
        BTHADMON("Illeagle command, command type error\n");
    }
}

void handle_ctl_ker(void *data)
{
    BTHADMON("ker data:%s\n", (char *)data);
}

void handle_ctl_pkt(void *data)
{
    pkt_data rdata;
    char *p = data;
    char raw_link_id[RAW_PKT_LINK_ID_LEN + 1];
    memset(raw_link_id, 0, RAW_PKT_LINK_ID_LEN + 1);

    /* get */
    if (*p =='r') {
        do {
            p++;
        } while ((*p) == ' ');

        memcpy(raw_link_id, p, RAW_PKT_LINK_ID_LEN);
        raw_link_id[RAW_PKT_LINK_ID_LEN] = '\0';
        rdata.link_id = strtoul(raw_link_id, NULL, 0);
        if ((rdata.link_id == 0xFF) ||
            (rdata.link_id >= 0) ||
            (rdata.link_id <= 6)){
            get_pkt_data(&rdata);
        } else {
            BTHADMON("Illeagle link id! It should be 0-6 or 0xFF.");
        }
    } else {
        BTHADMON("Illeagle command, command type error\n");
    }
}

void handle_ctl_enm(void *data)
{
    enm_data rdata;
    char *p = data;
    char raw_mask[RAW_ENM_MASK_LEN + 1];
    memset(raw_mask, 0, RAW_ENM_MASK_LEN + 1);

    /* get */
    if (*p =='r') {
        get_enm_data(NULL);
    /* set */
    } else if (*p == 'w') {
        do {
            p++;
        } while ((*p) == ' ');

        memcpy(raw_mask, p, RAW_ENM_MASK_LEN);
        raw_mask[RAW_ENM_MASK_LEN] = '\0';
        rdata.mask[0] = strtol(raw_mask, NULL, 0) & 0x0000007F;
        rdata.mask[1] = 0;
        rdata.mask[2] = 0;
        rdata.mask[3] = 0;
        set_enm_data(&rdata);
    } else {
        BTHADMON("Illeagle command, command type error\n");
    }
}

void handle_ctl_tcm(void *data)
{
    char *p = data;
    /* put */
    if (*p =='#') {
        set_tcm_data(data);
    }  else {
        BTHADMON("Illeagle command, command type error, must start with #\n");
    }
}


struct bt_fdi op = {
    handle_ctl_reg,
    handle_ctl_tag,
    handle_ctl_ker,
    handle_ctl_pkt,
    handle_ctl_enm,
    handle_ctl_tcm
};

struct bt_fdi* get_interface()
{
    return &op;
}
