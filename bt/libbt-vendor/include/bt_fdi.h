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
 *  Filename:      bt-fdi.h
 *
 *  Description:   firmware debug interface(bt-FDI)
 *
 ******************************************************************************/
#ifndef BT_FDI_H
#define BT_FDI_H

#define DIR_NUM 6

#define BT_FW_HCI_VAL       "/sys/kernel/debug/xradio_bt_dbg/dbg_hci_val"
#define BT_FW_ACL_VAL       "/sys/kernel/debug/xradio_bt_dbg/dbg_acl_val"
#define BT_FW_LMP_VAL       "/sys/kernel/debug/xradio_bt_dbg/dbg_lmp_val"
#define BT_FW_KER_VAL       "/sys/kernel/debug/xradio_bt_dbg/dbg_ker_val"
#define BT_FW_LOG_VAL       "/sys/kernel/debug/xradio_bt_dbg/dbg_log_val"
#define BT_FW_CTL_VAL       "/sys/kernel/debug/xradio_bt_dbg/dbg_ctl_val"
#define RAW_REG_ADDR_LEN    10
#define RAW_REG_DATA_LEN    128
#define REG_ADDR_LEN        4
#define REG_DATA_LEN        64

#define RAW_TAG_ID_LEN      4
#define RAW_TAG_DATA_LEN    128
#define TAG_DATA_LEN        64

#define RAW_PKT_LINK_ID_LEN 4

#define RAW_ENM_MASK_LEN    10
#define ENM_MASK_LEN        4

typedef struct EventMask {
    int     flag;
    char    *name;
}EventMask;

typedef struct FileHandleMask {
    char*           filename;
    char*           path;
    int             handle_num;
}FileHandleMask;

typedef struct reg_data {
    unsigned char addr[REG_ADDR_LEN];
    unsigned char data_len;
    unsigned char data[REG_DATA_LEN];
}reg_data;

typedef struct tag_data {
    unsigned char id;
    unsigned char data_len;
    unsigned char data[TAG_DATA_LEN];
}tag_data;

typedef struct pkt_data {
    unsigned char link_id;
}pkt_data;

typedef struct enm_data {
    unsigned char mask[ENM_MASK_LEN];
}enm_data;

struct bt_fdi {
    void (*handle_ctl_reg)(void *data);
    void (*handle_ctl_tag)(void *data);
    void (*handle_ctl_ker)(void *data);
    void (*handle_ctl_pkt)(void *data);
    void (*handle_ctl_enm)(void *data);
    void (*handle_ctl_tcm)(void *data);
};

void init_pthread(void);
void exit_pthread(void);
void write_nvsram_data(char *data, int datalen);
void write_raw_hci_data(char *data, int datalen);
void write_reg_data(void *data);
void read_reg_data(void *data);
void hci_xr_msg_cback(void *p_mem);

void fdi_init_firmware_log_file();
void fdi_close_firmware_log_file();
void fdi_log_firmware_debug_packet(uint8_t* data);
void read_tag_data(void *data);
void write_tag_data(void *data);
void delete_tag_data(void *data);
void lock_tag_data(void *data);
void get_pkt_data(void *data);
void get_enm_data(void *data);
void set_enm_data(void *data);
void set_tcm_data(void *data);

extern struct bt_fdi *get_interface();

#endif
