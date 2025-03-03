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
 *  Filename:      hardware.c
 *
 *  Description:   Contains controller-specific functions, like
 *                      firmware patch download
 *                      low power mode operations
 *
 ******************************************************************************/

#define LOG_TAG "bt_hwcfg"

#include <utils/Log.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <cutils/properties.h>
#include <stdlib.h>
#include <unistd.h>
#include "bt_hci_bdroid.h"
#include "bt_vendor_xr.h"
#include "userial.h"
#include "cmd.h"
#include "userial_vendor.h"
#include "upio.h"
#include "bt_vendor_lib.h"

#define BT_FW_PATH_NAME             FW_PATCHFILE_LOCATION"fw_xr829_bt.bin"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef BTHW_DBG
#define BTHW_DBG FALSE
#endif

#if (BTHW_DBG == TRUE)
#define BTHWDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTHWDBG(param, ...) {}
#endif


#define HCI_CMD_MAX_LEN             258

#define HCI_RESET                               0x0C03
#define HCI_READ_DEVICE_NAME                    0x0C14
#define HCI_READ_DEVICE_BDADDR                  0x1009
#define HCI_VSC_UPDATE_BAUDRATE                 0xFC18
#define HCI_VSC_WRITE_BDADDR                    0xFC32
#define HCI_VSC_SET_DEBUG_LEVEL                 0xFC04
#define HCI_VSC_SET_SLEEP_MODE                  0xFC19
#define HCI_VSC_GET_FW_VERSION                  0xFC40
#define HCI_VSC_CONTROLLER_SLEEP_CTRL           0xFC20
#define HCI_VSC_WRITE_NVSRAM                    0xFC0A
#define HCI_VSC_WLAN_BT_COEX_CMD                0xFC22

#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING      6
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY     6
#define HCI_EVT_CMD_CMPL_OPCODE                 3

#define HCI_CMD_PREAMBLE_SIZE                   3
#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          4
#define LPM_CMD_PARAM_SIZE                      13
#define CONTROLLER_SLEEP_CTRL_SIZE              1
#define WRITE_NVSRAM_CMD_PARAM_SIZE             9
#define WLAN_BT_COEX_CMD_PARAM_SIZE             2

#define BD_ADDR_LEN                             6
#define LOCAL_NAME_BUFFER_LEN                   32
#define LOCAL_BDADDR_PATH_BUFFER_LEN            256

#define NVRAM_TAG_BDADDR                        0x0002

#define HCI_DEBUG_LEVEL_NONE    0x00
#define HCI_DEBUG_LEVEL_CMD     0x01
#define HCI_DEBUG_LEVEL_ACL     0x02
#define HCI_DEBUG_LEVEL_SCO     0x04
#define HCI_DEBUG_LEVEL_EVT     0x08
#define HCI_DEBUG_LEVEL_FM      0x10
#define HCI_DEBUG_LEVEL_LD      0x20

#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}

#define BT_FW_LOAD_ADDR         0x0000
#define BT_FW_JUMP_ADDR         0x0000
/******************************************************************************
**  Local type definitions
******************************************************************************/

/* Hardware Configuration State */
enum {
    HW_CFG_START = 1,
    HW_CFG_SET_UART_BAUD_1,
    HW_CFG_DL_FW_PATCH,
    HW_CFG_SET_UART_BAUD_2,
    HW_CFG_READ_LOCAL_NAME,
    HW_CFG_SET_BD_ADDR
#if (USE_CONTROLLER_BDADDR == TRUE)
    , HW_CFG_READ_BD_ADDR
#endif
    , HW_CFG_DEBUG_LEVEL
};

/* h/w config control block */
typedef struct{
    uint8_t state;                          /* Hardware configuration state */
    int     fw_fd;                          /* FW patch file fd */
    uint8_t f_set_baud_2;                   /* Baud rate switch state */
    char    local_chip_name[LOCAL_NAME_BUFFER_LEN];
} bt_hw_cfg_cb_t;

typedef struct{
    uint8_t     sleep_mode;
    uint32_t    controller_pwr_sleep_timeout;   //default:19s[controller]
    uint32_t    controller_pwr_active_timeout;  //default:5s[controller]
    uint32_t    resered1;
} xr_bt_lpm_param_t;


/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_cback(void *p_evt_buf);
static int32_t load_btfirmware(void);
extern uint8_t vnd_local_bd_addr[BD_ADDR_LEN];
extern uint8_t bta_av_handle;
extern uint16_t local_l2cap_cid;
extern int hci_comm_bandrate;
/******************************************************************************
**  Static variables
******************************************************************************/

static bt_hw_cfg_cb_t hw_cfg_cb;

static xr_bt_lpm_param_t lpm_param =
{
    LPM_SLEEP_MODE,
    LPM_CONTROLLER_SLEEP_TIMEOUT,
    LPM_CONTROLLER_ACTIVE_TIMEOUT,
    0
};

static const int brom_download_baudrate = 1500000;
static const int hci_startup_baudrate = 115200;

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_set_controller_baudrate(HC_BT_HDR *p_buf, uint32_t baudrate)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *) (p_buf + 1);

    UINT16_TO_STREAM(p, HCI_VSC_UPDATE_BAUDRATE);
    *p++ = 4;
    UINT32_TO_STREAM(p, baudrate);


    p_buf->len = HCI_CMD_PREAMBLE_SIZE + 4;
    hw_cfg_cb.state = HW_CFG_SET_UART_BAUD_1;

    BTHWDBG("Device Init Device Bandrate[%d] Start.",baudrate);
    retval = bt_vendor_cbacks->xmit_cb(HCI_VSC_UPDATE_BAUDRATE, p_buf, \
                                 hw_config_cback);

    return (retval);
}

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_set_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *) (p_buf + 1);

    ALOGI("Setting local bd addr to %02X:%02X:%02X:%02X:%02X:%02X",
        vnd_local_bd_addr[0], vnd_local_bd_addr[1], vnd_local_bd_addr[2],
        vnd_local_bd_addr[3], vnd_local_bd_addr[4], vnd_local_bd_addr[5]);
#if 0
    UINT16_TO_STREAM(p, HCI_VSC_WRITE_BDADDR);
    *p++ = BD_ADDR_LEN; /* parameter length */
    *p++ = vnd_local_bd_addr[5];
    *p++ = vnd_local_bd_addr[4];
    *p++ = vnd_local_bd_addr[3];
    *p++ = vnd_local_bd_addr[2];
    *p++ = vnd_local_bd_addr[1];
    *p = vnd_local_bd_addr[0];

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + BD_ADDR_LEN;
    hw_cfg_cb.state = HW_CFG_SET_BD_ADDR;

    retval = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_BDADDR, p_buf, \
                                 hw_config_cback);
#else
    UINT16_TO_STREAM(p, HCI_VSC_WRITE_NVSRAM);
    *p++ = WRITE_NVSRAM_CMD_PARAM_SIZE; /* parameter length */
    UINT16_TO_STREAM(p, NVRAM_TAG_BDADDR);
    *p++ = BD_ADDR_LEN;
    *p++ = vnd_local_bd_addr[5];
    *p++ = vnd_local_bd_addr[4];
    *p++ = vnd_local_bd_addr[3];
    *p++ = vnd_local_bd_addr[2];
    *p++ = vnd_local_bd_addr[1];
    *p = vnd_local_bd_addr[0];

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + WRITE_NVSRAM_CMD_PARAM_SIZE;
    hw_cfg_cb.state = HW_CFG_SET_BD_ADDR;

    retval = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_NVSRAM, p_buf, \
                                 hw_config_cback);
#endif
    return (retval);
}

#if (USE_CONTROLLER_BDADDR == TRUE)
/*******************************************************************************
**
** Function         hw_config_read_bdaddr
**
** Description      Read controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_read_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *) (p_buf + 1);

    UINT16_TO_STREAM(p, HCI_READ_DEVICE_BDADDR);
    *p = 0; /* parameter length */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
    hw_cfg_cb.state = HW_CFG_READ_BD_ADDR;

    BTHWDBG("read bt bd_addr start.");
    retval = bt_vendor_cbacks->xmit_cb(HCI_READ_DEVICE_BDADDR, p_buf, \
                                 hw_config_cback);

    return (retval);
}
#endif // (USE_CONTROLLER_BDADDR == TRUE)

/*******************************************************************************
**
** Function         hw_config_cback
**
** Description      Callback function for controller configuration
**
** Returns          None
**
*******************************************************************************/
void hw_config_cback(void *p_mem)
{
    HC_BT_HDR   *p_evt_buf = NULL;
    char        *p_name, *p_tmp;
    uint8_t     *p, status = 0;
    uint16_t    opcode;
    HC_BT_HDR   *p_buf=NULL;
    uint8_t     is_proceeding = FALSE;
    int         i;


#if (USE_CONTROLLER_BDADDR == TRUE)
    const uint8_t null_bdaddr[BD_ADDR_LEN] = {0,0,0,0,0,0};
#endif

    if(p_mem != NULL)
    {
        p_evt_buf = (HC_BT_HDR *) p_mem;
        status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
        p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
        STREAM_TO_UINT16(opcode,p);
    }


    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == 0) && bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_MAX_LEN);
    }

    if (p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;

        p = (uint8_t *) (p_buf + 1);

        switch (hw_cfg_cb.state)
        {
        case HW_CFG_START:
            BTHWDBG("hci_reset rsp status:%d.", status);
            hw_cfg_cb.state = HW_CFG_SET_UART_BAUD_1;
            UINT16_TO_STREAM(p, HCI_VSC_UPDATE_BAUDRATE);
            *p++ = UPDATE_BAUDRATE_CMD_PARAM_SIZE;
            UINT32_TO_STREAM(p, hci_comm_bandrate);
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + UPDATE_BAUDRATE_CMD_PARAM_SIZE;

            hw_cfg_cb.state = HW_CFG_SET_UART_BAUD_1;
            is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_UPDATE_BAUDRATE, p_buf, hw_config_cback);
            break;
        case HW_CFG_SET_UART_BAUD_1:
            BTHWDBG("update baudrate:%d, device rsp status:%d.", hci_comm_bandrate, status);
            userial_vendor_set_baud(line_speed_to_userial_baud(hci_comm_bandrate));
            usleep(50000);
        case HW_CFG_SET_UART_BAUD_2:
            UINT16_TO_STREAM(p, HCI_READ_DEVICE_NAME);
            *p = 0;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE;

            hw_cfg_cb.state = HW_CFG_READ_LOCAL_NAME;
            is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_READ_DEVICE_NAME, p_buf, hw_config_cback);
            break;
        case HW_CFG_READ_LOCAL_NAME:

            p_name = (char *) (p_evt_buf + 1) +  HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING;
            for (i=0; (i < LOCAL_NAME_BUFFER_LEN)||(*(p_name+i) != '\0'); i++)
                *(p_name+i) = toupper(*(p_name+i));

            strncpy(hw_cfg_cb.local_chip_name, p_name, i);

            hw_cfg_cb.local_chip_name[i-1] = '\0';
            BTHWDBG("bt device name:%s.", hw_cfg_cb.local_chip_name);

#if (USE_CONTROLLER_BDADDR == TRUE)
            if ((is_proceeding = hw_config_read_bdaddr(p_buf)) == TRUE)
                break;
#else
            if ((is_proceeding = hw_config_set_bdaddr(p_buf)) == TRUE)
                break;
#endif

        case HW_CFG_SET_BD_ADDR:
            BTHWDBG("set bt bd_addr rsp status:%d.", status);

            ALOGI("vendor lib download complete.");
            bt_vendor_cbacks->dealloc(p_buf);
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
            hw_cfg_cb.state = 0;
            is_proceeding = TRUE;
            /*This is an Workaround for device dormancy, Let lpm release the lock*/
            upio_set_btwake(LPM_BT_WAKE_UNLOCK);
            break;

#if (USE_CONTROLLER_BDADDR == TRUE)
        case HW_CFG_READ_BD_ADDR:
            BTHWDBG("read bt bd_addr rsp status:%d.", status);
            p_tmp = (char *) (p_evt_buf + 1) + HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY;
            if (memcmp(p_tmp, null_bdaddr, BD_ADDR_LEN) == 0)
            {
                // Controller does not have a valid OTP BDADDR!
                // Set the BTIF initial BDADDR instead.
                if ((is_proceeding = hw_config_set_bdaddr(p_buf)) == TRUE)
                    break;
            }
            else
            {
                ALOGI("Controller OTP bdaddr %02X:%02X:%02X:%02X:%02X:%02X",
                    *(p_tmp+5), *(p_tmp+4), *(p_tmp+3),
                    *(p_tmp+2), *(p_tmp+1), *p_tmp);
            }

            ALOGI("vendor lib download complete.");
            bt_vendor_cbacks->dealloc(p_buf);
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
            hw_cfg_cb.state = 0;
            is_proceeding = TRUE;
            break;
#endif
        default:
            break;
        }
    }

    /* Free the RX event buffer */
    if ((bt_vendor_cbacks) && (p_mem != NULL))
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (is_proceeding == FALSE)
    {
        ALOGE("vendor lib fwcfg aborted!!!");
        if (bt_vendor_cbacks)
        {
            if (p_buf != NULL)
                bt_vendor_cbacks->dealloc(p_buf);

            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
        hw_cfg_cb.state = 0;
        /*This is an Workaround for device dormancy, Let lpm release the lock*/
        upio_set_btwake(LPM_BT_WAKE_UNLOCK);
    }
}

#define SDD_SAVE_FINAL_FILE (1)

#define BT_SDD_LOAD_ADDR (0x00050000 - 2048)

int sdd_data_create(uint8_t **pp_sdd, uint32_t *p_size);
void sdd_data_destroy(uint8_t *pSdd);

static int32_t load_btsdd(void)
{
    uint8_t *pbuf = NULL;
    uint32_t size = 0;

    ALOGD("BT sdd start");

    if (sdd_data_create(&pbuf, &size) != 0)
        return -1;
    ALOGD("BT sdd buffer created");

#if SDD_SAVE_FINAL_FILE
    FILE *test_file = fopen("/data/vendor/bluetooth/sdd/sdd.test", "wb+");
    if (test_file <= 0)
    {
        ALOGD("sdd test file can't save, return %u: %s", (int)test_file, strerror(errno));
        return 0;
    }
    fwrite(pbuf, 1, size, test_file);
    fclose(test_file);
#endif

    ALOGD("download sdd file, addr: 0x%08X, size: %d", BT_SDD_LOAD_ADDR, size);
    cmd_write_seq(BT_SDD_LOAD_ADDR, size, pbuf);
    sdd_data_destroy(pbuf);

    return 0;
}

static int32_t load_btfirmware(void)
{
    FILE*       fwfile_fd = NULL;
    uint32_t    len;
    uint8_t*    data = NULL;
    uint32_t    addr = BT_FW_LOAD_ADDR;
    uint32_t    section = SZ_16K;

    fwfile_fd = fopen(BT_FW_PATH_NAME, "rb");
    ALOGD("BT firmware: %s", BT_FW_PATH_NAME);
    if(!fwfile_fd) {
        ALOGE("Unable to open BT firmware %s", BT_FW_PATH_NAME);
        return -1;
    }

    data = (uint8_t*)malloc(section);
    if (data == NULL) {
        BTHWDBG("failed to alloc %d byte memory.", section);
        fclose(fwfile_fd);
        return -1;
    }

    ALOGI("load bt firmware starting.");
    while ((len = fread(data, 1, section, fwfile_fd)) > 0) {
        ALOGD("download firmware, addr: 0x%08X, size: %d", addr, len);
        if (cmd_write_seq(addr, len, data) != 0) {
            ALOGE("load firmware data error at addr %08X!", addr);
            return -1;
        }
        addr += len;
    }

    free(data);
    fclose(fwfile_fd);
    ALOGI("load bt firmware done.");

    BTHWDBG("run bt firmware.");
    cmd_set_pc(BT_FW_JUMP_ADDR);

    return addr;
}

/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Kick off controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void)
{
    /*This is an Workaround for device dormancy, Let lpm hold lock to avoid bt start up timeout*/
    upio_set_btwake(LPM_BT_WAKE_LOCK);

    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;

    ALOGI("brom uart sync starting.");
    if (cmd_sync_uart() < 0)
    {
        ALOGE("UART sync fail.");
        goto READY_ERROR;
    }

    ALOGI("brom updata baudrate starting.");
    if (cmd_sync_baud((brom_download_baudrate)|(3<<24)) < 0)
    {
        ALOGE("UART update bandrate fail.");
        goto READY_ERROR;
    }

    if (load_btsdd() < 0)
    {
        ALOGE("Download BT sdd fail.");
    }

    if (load_btfirmware() < 0)
    {
        ALOGE("Download BT firmware fail.");
        goto READY_ERROR;
    }

    userial_vendor_set_baud(line_speed_to_userial_baud(hci_startup_baudrate));
    userial_vendor_set_hw_fctrl(TRUE);
    usleep(20000);
    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;

    /* Start from sending HCI_RESET */
    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0;

        hw_cfg_cb.state = HW_CFG_START;
        BTHWDBG("hci_reset req starting.");
        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
     }
     else
     {
        ALOGE("vendor lib fw conf aborted [no buffer]");
        goto READY_ERROR;
     }

    return;

READY_ERROR:
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
}

uint32_t hw_lpm_get_idle_timeout(void)
{
//  uint32_t timeout_ms = 10000;    //10s, should keep same with kernel xradio_btlpm.ko
    uint32_t timeout_ms = 1500;    //set 1.5s because time is too long to machine enter into dormant slow

    return timeout_ms;
}

void hw_lpm_ctrl_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->lpm_cb(status);
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}
uint8_t hw_lpm_enable(uint8_t turn_on)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     ret = FALSE;
    uint8_t     *p;

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + LPM_CMD_PARAM_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + LPM_CMD_PARAM_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_VSC_SET_SLEEP_MODE);
        *p++ = LPM_CMD_PARAM_SIZE;

        if (turn_on)
        {
            *p++ = LPM_SLEEP_MODE;
            UINT32_TO_STREAM(p, LPM_CONTROLLER_SLEEP_TIMEOUT);
            UINT32_TO_STREAM(p, LPM_CONTROLLER_ACTIVE_TIMEOUT);
            UINT32_TO_STREAM(p, 0);
            upio_set(UPIO_LPM_MODE, UPIO_ASSERT, 0);
        }
        else
        {
            memset(p, 0, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_DEASSERT, 0);
        }

        if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_SET_SLEEP_MODE, p_buf, \
                                        hw_lpm_ctrl_cback)) == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
    }

    if ((ret == FALSE) && bt_vendor_cbacks)
        bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);

    return ret;
}

#if (LPM_SLEEP_MODE == TRUE)
void hw_sleep_ctrl_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    BTHWDBG("bt state: sleep");
    upio_set(UPIO_BT_WAKE, UPIO_DEASSERT, 0);

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

uint8_t hw_lpm_set_wake_state(uint8_t wake_assert)
{
    uint8_t     ret = TRUE;
    HC_BT_HDR   *p_buf = NULL;
    uint8_t     *p;

    if (wake_assert)
    {
        BTHWDBG("bt state: wakeup");
        upio_set(UPIO_BT_WAKE, UPIO_ASSERT, 0);
    }
    else
    {
        BTHWDBG("bt state: sleep request");
        upio_set(UPIO_BT_WAKE, UPIO_DEASSERT, 0);
    }

    return ret;
}

#endif

#if (HW_END_WITH_HCI_RESET == TRUE)
/******************************************************************************
*
**
** Function         hw_epilog_cback
**
** Description      Callback function for Command Complete Events from HCI
**                  commands sent in epilog process.
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_cback(void *p_mem)
{
    HC_BT_HDR   *p_evt_buf = (HC_BT_HDR *) p_mem;
    uint8_t     *p, status;
    uint16_t    opcode;

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

    BTHWDBG("%s Opcode:0x%04X Status: %d", __FUNCTION__, opcode, status);

    if (bt_vendor_cbacks)
    {
        /* Must free the RX event buffer */
        bt_vendor_cbacks->dealloc(p_evt_buf);

        /* Once epilog process is done, must call epilog_cb callback
           to notify caller */
        bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
    }
}

/******************************************************************************
*
**
** Function         hw_epilog_process
**
** Description      Sample implementation of epilog process
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_process(void)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;

    BTHWDBG("hw_epilog_process");

    /* Sending a HCI_RESET */
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_epilog_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib epilog process aborted [no buffer]");
            bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}
#endif


void hw_a2dp_hci_cmp_cback(void *p_mem)
{
    HC_BT_HDR   *p_evt_buf = NULL;
    uint8_t     *p, hci_result = 0;
    uint16_t    opcode = 0;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_SUCCESS;

    if (p_mem != NULL)
    {
        p_evt_buf = (HC_BT_HDR *) p_mem;
        hci_result = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
        p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
        STREAM_TO_UINT16(opcode, p);

        if (hci_result != 0)
        {
            status = BT_VND_OP_RESULT_FAIL;
        }

        if (opcode == HCI_VSC_WLAN_BT_COEX_CMD)
        {
            BTHWDBG("a2dp coexitence opt hci complete status:%d", hci_result);

            if (bt_vendor_cbacks)
            {
                bt_vendor_cbacks->dealloc(p_evt_buf);
            }
        }
    }

}

void hw_a2dp_send_hci_vsc(uint16_t cmd, uint8_t *payload, uint8_t len, tINT_CMD_CBACK cback)
{
    HC_BT_HDR   *p_buf;
    uint8_t     *p;

    if (bt_vendor_cbacks)
    {

        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                        HCI_CMD_PREAMBLE_SIZE + len);

        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + len;
            p = (uint8_t *)(p_buf + 1);

            UINT16_TO_STREAM(p, cmd);
            *p++ = len;
            memcpy(p, payload, len);

            bt_vendor_cbacks->xmit_cb(cmd, p_buf, cback);
        }
    }
}

void hw_a2dp_coex_start(uint8_t ctrl_cmd)
{
    uint8_t msg_payload[WLAN_BT_COEX_CMD_PARAM_SIZE];

    msg_payload[0] = 0x01;
    if (ctrl_cmd == 1)
    {
        msg_payload[1] = 0x01;
        hw_a2dp_send_hci_vsc(HCI_VSC_WLAN_BT_COEX_CMD, msg_payload, WLAN_BT_COEX_CMD_PARAM_SIZE, hw_a2dp_hci_cmp_cback);
        BTHWDBG("a2dp coexitence opt has started.");
    }
    else
    {
        msg_payload[1] = 0x02;
        hw_a2dp_send_hci_vsc(HCI_VSC_WLAN_BT_COEX_CMD, msg_payload, WLAN_BT_COEX_CMD_PARAM_SIZE, hw_a2dp_hci_cmp_cback);
        BTHWDBG("a2dp coexitence opt has stopped.");
    }

}
