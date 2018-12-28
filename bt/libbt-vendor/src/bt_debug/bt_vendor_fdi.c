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
 *  Filename:      bt_vendor_fdi.c
 *
 *  Description:   firmware debug interface(bt-FDI) implementation
 *
 ******************************************************************************/
#define LOG_TAG "bt_vendor_fdi"
#include <utils/Log.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <sys/time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <string.h>
#include <cutils/properties.h>

#include <stdlib.h>
#include <unistd.h>
#include "bt_hci_bdroid.h"
#include "userial.h"
#include "cmd.h"
#include "userial_vendor.h"
#include "upio.h"
#include "bt_vendor_lib.h"
#include "bt_fdi.h"
#include "const_array.h"


/******************************************************************************
**  Constants & Macros
******************************************************************************/
#define BTFDI_DBG FALSE

#if (BTFDI_DBG == TRUE)
#define BTFDIDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTFDIDBG(param, ...) {}
#endif

static const int INVALID_FD = -1;
static int fdi_firmware_log_fd = INVALID_FD;
static int log_counter;
static int log_counter_max = 90000;
#define TIME_STR_LEN           20
#define FDI_FILE_LEN           50

/* Xradio HCI Debug message type definitions */
#define DEBUG_BUFFER_LEN            512
#define XR_EVT_CMPL                 4
#define XR_DEBUG_COMMAND            0x55
#define XR_DEBUG_EVENT              0xAA

#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}
#define STREAM_TO_UINT32(u32, p)\
    do { \
        u32 = (uint32)*(p) + (uint32)(*((p) + 1) << 8) + (uint32)((*(p) + 2) << 16) + (uint32)(*((p) + 3) << 24); \
        (p) += 4; \
    } while (0)

#define HCI_EVT_CMD_CMPL_OPCODE                 3
#define HCI_CMD_PREAMBLE_SIZE                   3

#define BT_CTL_REG_RSP      "/sys/kernel/debug/xradio_bt_dbg/dbg_ctl_val/reg_val_rsp"
#define BT_CTL_TAG_RSP      "/sys/kernel/debug/xradio_bt_dbg/dbg_ctl_val/tag_val_rsp"
#define BT_CTL_PKT_RSP      "/sys/kernel/debug/xradio_bt_dbg/dbg_ctl_val/pkt_val_rsp"
#define BT_CTL_ENM_RSP      "/sys/kernel/debug/xradio_bt_dbg/dbg_ctl_val/enm_val_rsp"
#define BT_LMP_RSP          "/sys/kernel/debug/xradio_bt_dbg/dbg_lmp_val/lmp_val"

#define LOG_PATH            "/data/vendor/bluetooth/fdi/fdi_events.log"
#define LAST_LOG_PATH       "/data/vendor/bluetooth/fdi/fdi_events.log.last"
/******************************************************************************
*
** Description         xradio farmware debug command
**
*******************************************************************************/
/* All command ogf code */
#define XR_DEBUG_HCI_CMD_MEM        0x11
#define XR_DEBUG_HCI_CMD_TAG        0x12
#define XR_DEBUG_HCI_CMD_KER        0x13
#define XR_DEBUG_HCI_CMD_PKT        0x14
#define XR_DEBUG_HCI_CMD_ENM        0x1F
#define XR_DEBUG_HCI_CMD_TCM        0x20


/* Read write mem opcode */
#define HCI_GRP_MEM_CMDS            (XR_DEBUG_HCI_CMD_MEM << 8)
#define HCI_FDI_READ_MEM            (0x0000 | HCI_GRP_MEM_CMDS)
#define HCI_FDI_WRITE_MEM           (0x0001 | HCI_GRP_MEM_CMDS)

/* Read write tag opcode */
#define HCI_GRP_TAG_CMDS            (XR_DEBUG_HCI_CMD_TAG << 8)
#define HCI_FDI_READ_TAG            (0x0000 | HCI_GRP_TAG_CMDS)
#define HCI_FDI_WRITE_TAG           (0x0001 | HCI_GRP_TAG_CMDS)
#define HCI_FDI_DELETE_TAG          (0x0002 | HCI_GRP_TAG_CMDS)
#define HCI_FDI_LOCK_TAG            (0x0003 | HCI_GRP_TAG_CMDS)

/* Read write ker opcode */
#define HCI_GRP_KER_CMDS            (XR_DEBUG_HCI_CMD_KER << 8)
#define HCI_FDI_STAT_KER            (0x0001 | HCI_GRP_KER_CMDS)

/* Get pkt opcode */
#define HCI_GRP_PKT_CMDS            (XR_DEBUG_HCI_CMD_PKT << 8)
#define HCI_FDI_GET_PKT             (0x0001 | HCI_GRP_PKT_CMDS)

/* Get set enm opcode */
#define HCI_GRP_ENM_CMDS            (XR_DEBUG_HCI_CMD_ENM << 8)
#define HCI_FDI_GET_ENM             (0x0000 | HCI_GRP_ENM_CMDS)
#define HCI_FDI_SET_ENM             (0x0001 | HCI_GRP_ENM_CMDS)

/* Pass Debug Uart Command To Firmware  */
#define HCI_GRP_TCM_CMDS            (XR_DEBUG_HCI_CMD_TCM << 8)
#define HCI_FDI_PUT_TCM             (0x0001 | HCI_GRP_TCM_CMDS)


/******************************************************************************
*
** Description         xradio farmware debug event
**
*******************************************************************************/
/* All event ogf code */
#define XR_DEBUG_EVENT_HCI        0x01
#define XR_DEBUG_EVENT_ACL        0x02
#define XR_DEBUG_EVENT_LMP        0x04
#define XR_DEBUG_EVENT_KER        0x05
#define XR_DEBUG_EVENT_LOG        0x06
#define XR_DEBUG_EVENT_CTL        0x07

/* ctl class Ocf Code (0x07)*/
#define STATUS                  0x00
#define MEM                     0x01
#define TAG                     0x02
#define KER                     0x03
#define PKT                     0x04
#define ENM                     0x05

/******************************************************************************
**
** Function         handle_ctl_event
**
** Description     handle_ctl_event data
**
** Returns          None
**
*******************************************************************************/
enum Ctl_Status{
    SUCCESS,
    LEN_ERR,
    ADDR_ERR,
    TAG_ERR,
    CMD_DISALLOW
};

int write_rsp_file(char *file_path, unsigned char *buf, int len)
{
    int fd;
    fd = open(file_path, O_WRONLY | O_TRUNC);
    if (fd < 0) {
        ALOGI("Can't open file:%s", file_path);
        return -1;
    } else {
        int n = write(fd, buf, len);
        if (n != len) {
            ALOGI("Failed to write! %d bytes to be write,but write %d bytes!", len, n);
        } else {
            BTFDIDBG("Write OK!");
        }
        close(fd);
    }
    return 0;
}

void handle_ctl_event(void *p_data, uint8_t ocf, uint8_t  para_total_len)
{
    uint8_t *p;
    uint8_t *m;
    uint8_t i;
    uint8_t para_data_len;
    uint32 offset;
    uint32 max_static;
    uint32 min_static;
    uint32 current_static;
    uint32 value_temp;
    uint8_t debug_buf[DEBUG_BUFFER_LEN];
    memset(debug_buf, 0, DEBUG_BUFFER_LEN);

    p = (uint8_t *)p_data;
    switch (ocf) {
        /*  CTL STATUS  EVENT*/
        case STATUS:
            BTFDIDBG("command opcode:%02X %02X(ocf-ogf)", p[0], p[1]);
            switch(p[2]) {
                case SUCCESS:
                    ALOGI("handle_ctl_event:SUCCESS");
                    break;
                case LEN_ERR:
                    ALOGI("handle_ctl_event:LEN_ERR");
                    break;
                case ADDR_ERR:
                    ALOGI("handle_ctl_event:ADDR_ERR");
                    break;
                case TAG_ERR:
                    ALOGI("handle_ctl_event:TAG_ERR");
                    break;
                case CMD_DISALLOW:
                    ALOGI("handle_ctl_event:CMD_DISALLOW");
                    break;
                default:
                    ALOGI("handle_ctl_event:Unknow status");
                    break;

                }
            break;

        case MEM:
            offset = 0;
            m = p;
            p += 4;
            para_data_len = *p++;

            if (para_data_len <= 4) {
                offset += sprintf((char *)(debug_buf + offset), "Read register addr:0x%02X%02X%02X%02X\n", m[3], m[2], m[1], m[0]);
                offset += sprintf((char *)(debug_buf + offset), "Read register len:%d\n", para_data_len);
                offset += sprintf((char *)(debug_buf + offset), "Read register data:");
                for (i = 0; i < para_data_len; i++) {
                    offset += sprintf((char *)(debug_buf + offset), "0x%02X ", *p++);
                }
            } else {
                sprintf((char *)(debug_buf + offset), "%s", p);
            }
            ALOGI("%s", debug_buf);
            offset += sprintf((char *)(debug_buf + offset), "\n");
            write_rsp_file(BT_CTL_REG_RSP, debug_buf, offset);
            BTFDIDBG("handle_enm_event:SUCCESS");
            break;

        case TAG:
            offset = 0;
            offset += sprintf((char *)(debug_buf + offset), "tag id:%d\n", *p++);
            para_data_len = *p++;
            offset += sprintf((char *)(debug_buf + offset), "tag data:");
            for (i = 0; i < para_data_len; i++) {
                offset += sprintf((char *)(debug_buf + offset), "0x%02x ", *p++);
            }
            ALOGI("%s", debug_buf);
            offset += sprintf((char *)(debug_buf + offset), "\n");
            write_rsp_file(BT_CTL_TAG_RSP, debug_buf, offset);
            BTFDIDBG("handle_enm_event:SUCCESS");
            break;

        case KER:
            BTFDIDBG("KER TAG:%02X", *p++);
            para_data_len = *p++;
            STREAM_TO_UINT32(max_static, p);
            STREAM_TO_UINT32(min_static, p);
            STREAM_TO_UINT32(current_static, p);
            BTFDIDBG("KER TAG:%d,%d,%d", max_static, min_static, min_static);
            //to do:write the data to file
            break;

        case PKT:
            offset = 0;
            offset += sprintf((char *)(debug_buf + offset), "link id:%d\n", *p++);
            for (i = 0;i < 25;i++) {
                value_temp = 0;
                value_temp |= (uint32)(*p++ << 24);
                value_temp |= (uint32)(*p++ << 16);
                value_temp |= (uint32)(*p++ << 8);
                value_temp |= (uint32)(*p++ << 0);
                offset += sprintf((char *)(debug_buf + offset), "%s:0x%08X\n", pkt_name_array[i], value_temp);
            }
            ALOGI("%s", debug_buf);
            offset += sprintf((char *)(debug_buf + offset), "\n");
            write_rsp_file(BT_CTL_PKT_RSP, debug_buf, offset);
            BTFDIDBG("handle_packet_event:SUCCESS");
            break;

        case ENM:
            offset = 0;
            offset += sprintf((char *)(debug_buf + offset), "Enm data:");
            for (i = 0; i < para_total_len; i++) {
                offset += sprintf((char *)(debug_buf + offset), " 0x%02X", *p++);
            }
            ALOGI("%s", debug_buf);
            offset += sprintf((char *)(debug_buf + offset), "\n");
            write_rsp_file(BT_CTL_ENM_RSP, debug_buf, offset);
            BTFDIDBG("handle_enm_event:SUCCESS");
            break;

        default:
            BTFDIDBG("ctl_event unknow ocf");

    }
}
/******************************************************************************
**
** Function         hci_xr_msg_cback
**
** Description      Callback function for Command Complete Events from HCI
**                  commands sent in hci_send_xr_msg

**event data structure
**      ------------------------------------------
**      | ocf  | ogf  | total_len  | data  |
**      ------------------------------------------
**
** Returns          None
**
*******************************************************************************/
void hci_xr_msg_cback(void *p_mem)
{
    uint8_t *p;
    uint8_t  ocf, ogf, para_total_len;
    uint8_t op_code, op_ext_code;
    p = (uint8_t *)(p_mem);
    ocf = *p++;
    ogf = *p++;
    para_total_len = *p++;
    BTFDIDBG("ocf:0x%02X ogf:0x%02X len:0x%02X", ocf, ogf, para_total_len);

    uint8_t i, offset;
    uint8_t debug_buf[DEBUG_BUFFER_LEN];
    memset(debug_buf, 0, DEBUG_BUFFER_LEN);

    switch (ogf) {
        case XR_DEBUG_EVENT_HCI:

            break;

        case XR_DEBUG_EVENT_ACL:

            break;
        case XR_DEBUG_EVENT_LMP:
            offset = 0;
            offset += sprintf((char *)(debug_buf + offset), "lmp data:");
            for (i = 0; i < para_total_len; i++) {
                offset += sprintf((char *)(debug_buf + offset), " 0x%02X", *p++);
            }
            offset += sprintf((char *)(debug_buf + offset), "\n");

            p = (uint8_t *)(p_mem) + 3;
            op_code = ((*p) & 0xFE) >> 1;
            if ( op_code <= MAX_LMP_CODE) {
                offset += sprintf((char *)(debug_buf + offset), "Lmp op code 0x%02X:%s", op_code, lmp_code[op_code]);
            } else if (op_code == EXTENED_CODE){
                p++;
                op_ext_code = *p;
                if (op_ext_code <= MAX_EXT_CODE) {
                    offset += sprintf((char *)(debug_buf + offset), "Lmp extended op code 0x%02X:%s",
                                      op_ext_code, lmp_extened_code[op_ext_code]);
                } else {
                    offset += sprintf((char *)(debug_buf + offset), "Lmp extended op code 0x%02X:%s",
                                      op_ext_code, lmp_extened_code[0]);
                }
            } else {
                offset += sprintf((char *)(debug_buf + offset), "Lmp op code 0x%02X:%s", op_code, lmp_code[0]);
            }
            ALOGI("%s", debug_buf);
            offset += sprintf((char *)(debug_buf + offset), "\n");
            write_rsp_file(BT_LMP_RSP, debug_buf, offset);
            BTFDIDBG("handle_lmp_event:SUCCESS");
            break;
        case XR_DEBUG_EVENT_KER:

            break;
        case XR_DEBUG_EVENT_LOG:
            snprintf((char *)debug_buf, para_total_len, "%s", p);
            ALOGI("%s", debug_buf);
            fdi_log_firmware_debug_packet(debug_buf);
            break;
        case XR_DEBUG_EVENT_CTL:
            handle_ctl_event(p, ocf, para_total_len);
            break;

        default:
            ALOGI("unknow ogf code!");
            break;
    }
}

/******************************************************************************
**
** Function         build_xr_cmd_buf
**
** Description     build xradio hci cmd_buf
**
*  --------------------------------------------
*  |  HC_BT_HDR  |  HCI command     |
*  --------------------------------------------
*  where
*      HC_BT_HDR.event = 0x2000;
*      HC_BT_HDR.len = Length of HCI command;
*      HC_BT_HDR.offset = 0;
*      HC_BT_HDR.layer_specific = 0;
*
*  For example, a HCI_RESET Command will be formed as
*  ------------------------
*  |  HC_BT_HDR  |03|0c|00|
*  ------------------------
*  with
*      HC_BT_HDR.event = 0x2000;
*      HC_BT_HDR.len = 3;
*      HC_BT_HDR.offset = 0;
*      HC_BT_HDR.layer_specific = 0;
*
** Returns          HC_BT_HDR *buf
*******************************************************************************/

static HC_BT_HDR *build_xr_cmd_buf(uint16 cmd, uint8 len, uint8 *payload)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;
    uint16 cmd_len = HCI_CMD_PREAMBLE_SIZE + len;
    if (bt_vendor_cbacks) {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + cmd_len);
    }

    if (p_buf == NULL) {
        return NULL;
    } else {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->len = cmd_len;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p = (uint8_t *) (p_buf + 1);

        /*opcode */
        UINT16_TO_STREAM(p, cmd);

        /* payload len */
        *p = len;
        ++p;

        /* payload */
        memcpy(p, payload, len);
    }
    return p_buf;
}



/******************************************************************************
*
**
** Function         hci_send_xr_msg
**
** Description      hci_send_xr_msg function for send bt vendor hci command
**
**      ------------------------------------------
**      | ocf  | ogf  | total_len  | data  |
**      ------------------------------------------
**
** Returns          None
**
*******************************************************************************/
void hci_send_xr_msg(HC_BT_HDR *p_msg)
{
    uint8_t type = 0;
    uint16_t bytes_to_send = 0;
    uint8_t *p = ((uint8_t *)(p_msg + 1)) + p_msg->offset;
    uint16_t event = p_msg->event & MSG_EVT_MASK;
    uint16_t bytes_sent;
    uint8_t i, offset;
    uint8_t debug_buf[DEBUG_BUFFER_LEN];
    memset(debug_buf, 0, DEBUG_BUFFER_LEN);
    if (event == MSG_STACK_TO_HC_HCI_CMD)
        type = XR_DEBUG_COMMAND;

    /* put the HCI Transport packet type 1 byte */
    p = ((uint8_t *)(p_msg + 1)) + p_msg->offset - 1;
    *p = type;

    /* message_size + message type */
    bytes_to_send = p_msg->len + 1;

    bytes_sent = userial_vendor_write((uint8_t *)p, bytes_to_send);
    offset = 0;
    if (event == MSG_STACK_TO_HC_HCI_CMD) {
        for (i = 0; i < bytes_to_send - 1; i++) {
            offset += sprintf((char *)(debug_buf + offset), "%02X:", *p++);
        }
        offset += sprintf((char *)(debug_buf + offset), "%02X", *p);
        BTFDIDBG("fw_hci_cmd:%s", debug_buf);
    } else {
        BTFDIDBG("fw_hci_cmd error!");
    }

    if (bt_vendor_cbacks) {
        bt_vendor_cbacks->dealloc(p_msg);
    }
}

/******************************************************************************
**
** Description     Handle Debug Action
**
*******************************************************************************/
void read_reg_data(void *data)
{
    reg_data *rdata = (reg_data *)data;
    rdata->data_len = 4;//mean read 4 byte data from register
    HC_BT_HDR  *p_buf = NULL;
    p_buf = build_xr_cmd_buf(HCI_FDI_READ_MEM, (REG_ADDR_LEN + 1), (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void write_reg_data(void *data)
{
    reg_data *rdata = (reg_data *)data;
    HC_BT_HDR  *p_buf = NULL;
    uint8_t *p = (uint8_t *)rdata;
    uint8_t len = 0;;
    ALOGI("Write register addr:0x%02X%02X%02X%02X", p[0], p[1] ,p[2], p[3]);
    p += 5;
    ALOGI("Write register data: 0x%02X 0x%02X 0x%02X 0x%02X", p[0], p[1] ,p[2], p[3]);
    len = REG_ADDR_LEN + 1 + rdata->data_len;
    p_buf = build_xr_cmd_buf(HCI_FDI_WRITE_MEM, len, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void read_tag_data(void *data)
{
    tag_data *rdata = (tag_data *)data;
    rdata->data_len = 64;//mean read max 64 byte data from tag
    HC_BT_HDR  *p_buf = NULL;
    p_buf = build_xr_cmd_buf(HCI_FDI_READ_TAG, 2, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void write_tag_data(void *data)
{
    tag_data *rdata = (tag_data *)data;
    HC_BT_HDR  *p_buf = NULL;
    uint8 len = 0;
    len = 1 + 1 + rdata->data_len;
    p_buf = build_xr_cmd_buf(HCI_FDI_WRITE_TAG, len, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void delete_tag_data(void *data)
{
    tag_data *rdata = (tag_data *)data;
    rdata->data_len = 0;//mean read no data from tag
    HC_BT_HDR  *p_buf = NULL;
    p_buf = build_xr_cmd_buf(HCI_FDI_DELETE_TAG, 2, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void lock_tag_data(void *data)
{
    tag_data *rdata = (tag_data *)data;
    HC_BT_HDR  *p_buf = NULL;
    uint8 len = 0;
    len = 2;
    p_buf = build_xr_cmd_buf(HCI_FDI_LOCK_TAG, len, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void get_pkt_data(void *data)
{
    pkt_data *rdata = (pkt_data *)data;
    HC_BT_HDR  *p_buf = NULL;
    uint8 len = 0;
    len = 1;
    p_buf = build_xr_cmd_buf(HCI_FDI_GET_PKT, len, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void get_enm_data(void *data)
{
    HC_BT_HDR  *p_buf = NULL;
    p_buf = build_xr_cmd_buf(HCI_FDI_GET_ENM, 0, NULL);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void set_enm_data(void *data)
{
    enm_data *rdata = (enm_data *)data;
    HC_BT_HDR  *p_buf = NULL;
    uint8 len = 0;
    len = 4;
    p_buf = build_xr_cmd_buf(HCI_FDI_SET_ENM, len, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

void set_tcm_data(void *data)
{
    uint8 len = (uint8)strlen(data);
    HC_BT_HDR  *p_buf = NULL;
    p_buf = build_xr_cmd_buf(HCI_FDI_PUT_TCM, len, (uint8 *)data);
    if (p_buf != NULL) {
        hci_send_xr_msg(p_buf);
    }
}

extern struct tm *localtime(const time_t *);

/*******************************************************************************/
uint8_t * timeString()
{
    struct timeval ts;
    gettimeofday(&ts, NULL);
    struct tm *timeinfo = NULL;
    timeinfo = (struct tm *)localtime(&ts.tv_sec);

    uint8_t timeStr[TIME_STR_LEN];
    static uint8_t current_time[TIME_STR_LEN];
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);
    snprintf((char *)current_time, TIME_STR_LEN, "%s.%.3ld",timeStr, ts.tv_usec / 1000);
    return current_time;
}

void fdi_log_write(uint8_t* data, uint8_t len)
{
    if (fdi_firmware_log_fd != INVALID_FD) {
        uint8_t buf[DEBUG_BUFFER_LEN];
        uint8_t *s_time = timeString();
        len = len + strlen((const char *)s_time) + 2;
        snprintf((char *)buf, len, "%s %s", s_time, data);
        buf[len] = '\n';
        if (write(fdi_firmware_log_fd, buf, len + 1) < 0)
          ALOGE("write_fdi_log_file fail");
    }
}

void fdi_log_firmware_debug_packet(uint8_t* data)
{
    if (log_counter++ > log_counter_max) {
        fdi_init_firmware_log_file();
        log_counter = 0;
    }
    uint8_t* p = data;
    uint8_t* p_start = data;
    uint8_t data_len = 0;
    while ((*p++) != '\0') {
      if ((*p) == '\n') {
        data_len = (p++) - p_start;
        if (data_len < DEBUG_BUFFER_LEN - TIME_STR_LEN)
          fdi_log_write(p_start, data_len);
        p_start = p;
      }
    }
    data_len = p - p_start - 1;
    if (data_len < DEBUG_BUFFER_LEN - TIME_STR_LEN)
        fdi_log_write(p_start, data_len);
}

int fdi_open_firmware_log_file()
{
    if (rename(LOG_PATH, LAST_LOG_PATH)) {
      ALOGI("%s unable to rename '%s' to '%s': %s", __func__,
            LOG_PATH, LAST_LOG_PATH, strerror(errno));
    }

    mode_t prevmask = umask(0);
    int logfile_fd = open(LOG_PATH, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);
    umask(prevmask);

    if (logfile_fd == INVALID_FD) {
      ALOGI("unable to open '%s': %s", LOG_PATH, strerror(errno));
    }
    return logfile_fd;
}

void fdi_init_firmware_log_file()
{
    if (log_counter) {
      ALOGI("Default max file size is:(50 MByte)");
      int max_file_size = FDI_FILE_LEN * 1024 *1024;
      struct stat file_size;
      stat(LOG_PATH, &file_size);
      ALOGI("packet_counter top:%d,current file size:(%lldK-%.1fM)",
      log_counter_max, (file_size.st_size / 1024), (file_size.st_size / 1024 / 1024.0));
      if (file_size.st_size < max_file_size) {
        return;
      }
    }

    ALOGI("fdi_init_firmware_log_file");
    fdi_firmware_log_fd = fdi_open_firmware_log_file();
    if (fdi_firmware_log_fd != INVALID_FD) {
        fdi_log_firmware_debug_packet((uint8_t *)"fdi_init_firmware_log_file");
    } else {
        ALOGI("fdi_init_firmware_log_file fail");
    }
}

void fdi_close_firmware_log_file()
{
    if (fdi_firmware_log_fd != INVALID_FD) {
        close(fdi_firmware_log_fd);
    }
}
