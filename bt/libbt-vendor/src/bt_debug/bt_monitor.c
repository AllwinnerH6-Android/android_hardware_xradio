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
 *  Filename:      bt_mointor.c
 *
 *  Description:   mointor bt debugfs file
 *
 ******************************************************************************/
#define LOG_TAG "bt_monitor"
#include <unistd.h>
#include <sys/inotify.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <fcntl.h>
#include <pthread.h>
#include <utils/Log.h>
#include <sys/time.h>
#include "bt_fdi.h"

#define FDIMON_DBG FALSE

#if (FDIMON_DBG == TRUE)
#define BTMON(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTMON(param, ...) {}
#endif

volatile int running = 1;
static int maxfd;
struct bt_fdi *fdi_op = NULL;
pthread_t id;

struct EventMask event_masks[] = {
    {IN_ACCESS          , "IN_ACCESS"}          ,//read
    {IN_ATTRIB          , "IN_ATTRIB"}          ,
    {IN_CLOSE_WRITE     , "IN_CLOSE_WRITE"}     ,//write
    {IN_CLOSE_NOWRITE   , "IN_CLOSE_NOWRITE"}   ,
    {IN_CREATE          , "IN_CREATE"}          ,
    {IN_DELETE          , "IN_DELETE"}          ,
    {IN_DELETE_SELF     , "IN_DELETE_SELF"}     ,
    {IN_MODIFY          , "IN_MODIFY"}          ,
    {IN_MOVE_SELF       , "IN_MOVE_SELF"}       ,
    {IN_MOVED_FROM      , "IN_MOVED_FROM"}      ,
    {IN_MOVED_TO        , "IN_MOVED_TO"}        ,
    {IN_OPEN            , "IN_OPEN"}            ,
    {IN_ONLYDIR         , "IN_ONLYDIR"}         ,
    {IN_IGNORED         , "IN_IGNORED"}         ,
    {IN_ISDIR           , "IN_ISDIR"}           ,
};

enum file_mask {
    CTL_REG,
    CTL_TAG,
    CTL_KER,
    CTL_PKT,
    CTL_ENM,
    CTL_TCM
};

struct FileHandleMask file_handle_masks[] = {
    /* File_name        File_path       File_mask */
    { "reg_val",    BT_FW_CTL_VAL,  CTL_REG},
    { "tag_val",    BT_FW_CTL_VAL,  CTL_TAG},
    { "ker_val",    BT_FW_CTL_VAL,  CTL_KER},
    { "pkt_val",    BT_FW_CTL_VAL,  CTL_PKT},
    { "enm_val",    BT_FW_CTL_VAL,  CTL_ENM},
    { "hci_val",    BT_FW_HCI_VAL,  CTL_TCM},
};

int fread_data(void *dest, size_t remain, FILE *file)
{
    char *offset = (char*)dest;
    while (remain) {
        int n = fread(offset, 1, remain, file);
        if (n == 0) {
            return -1;
        }
        remain -= n;
        offset += n;
    }
    return 0;
}

int handle_file_data(int mask, char *data)
{
    int retval = 0;
    switch (mask) {
        case CTL_REG:
            fdi_op->handle_ctl_reg(data);
            break;
        case CTL_TAG:
            fdi_op->handle_ctl_tag(data);
            break;
        case CTL_KER:
            fdi_op->handle_ctl_ker(data);
            break;
        case CTL_PKT:
            fdi_op->handle_ctl_pkt(data);
            break;
        case CTL_ENM:
            fdi_op->handle_ctl_enm(data);
            break;
        case CTL_TCM:
            fdi_op->handle_ctl_tcm(data);
            break;
        default:
            BTMON("unknow file mask\n");
            retval = -1;
            break;
    }
    return retval;
}

void handle_file(int mask)
{
    int fd;
    char file[64];
    memset(file,0,64);
    char databuf[128];
    memset(databuf,0,128);

    strcat(file, file_handle_masks[mask].path);
    strcat(file, "/");
    strcat(file, file_handle_masks[mask].filename);

    fd = open(file, O_RDONLY);
    if (fd < 0) {
        ALOGI("can't open file\n");
    } else {
        int n = read(fd, databuf, sizeof(databuf));//3
        if (n != 0) {
                if (!handle_file_data(mask, databuf)) {
                /* if the data is the send data,we clear the send file data*/
                ftruncate(fd, 0);
                lseek(fd, 0, SEEK_SET);
                }
        } else {
            BTMON("data is none\n");
        }
        close(fd);
    }
}


void debug_event(char *name, uint32_t mask)
{
    ALOGI("debug_event name:%s ", name);
    int i = 0;
    for(i=0; i < 15; i++) {
        if (mask == (uint32_t)event_masks[i].flag)
            ALOGI(" %s ", event_masks[i].name);
    }
}

void handle_event(char *name, uint32_t mask)
{
    int filemask;
    /* watch write only*/
    if (mask == IN_CLOSE_WRITE) {
        for (filemask = 0; filemask < (int)(sizeof(file_handle_masks) / sizeof(FileHandleMask)); filemask++) {
            if (strcmp(name, file_handle_masks[filemask].filename) == 0) {
                handle_file(filemask);
            }
        }
    }
}

void event_read(int monitor_file)
{
    char name[128];
    char buffer[1024];
    struct inotify_event *event;
    char *p;
    int i, buffer_len;
    size_t event_size;
    buffer_len = 0;
    i = read (monitor_file, buffer, 1024);
    if (i <= 0) {
        BTMON("fread_data none\n");
        return;
    }

    while (buffer_len < i) {
        event = (struct inotify_event *)(&buffer[buffer_len]);
        event_size = sizeof(struct inotify_event) + event->len;
        p = (char *)&buffer[sizeof(struct inotify_event) + buffer_len];
        memcpy(name, p, event->len);
        handle_event(name, event->mask);
        buffer_len += event_size;
    }
}

int event_check(int fd)
{
    int retval;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    maxfd = fd + 1;
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    retval = select(maxfd, &rfds, NULL, NULL, &tv);
    if (retval < 0) {
        BTMON("select error, process will exit\n");
        running = 0;
    }
    return retval;
}

void process_inotify_event(int fd)
{
    running = 1;
    while (running > 0) {
        if (event_check(fd) > 0) {
            event_read(fd);
        }
    }
    BTMON("process will exit\n");
}

int watch_dir(int fd, const char *dirname, unsigned long mask)
{
    int wd;
    wd = inotify_add_watch (fd, dirname, mask);
    if (wd < 0) {
        ALOGI("can't watch %s, %s\n", dirname, strerror(errno));
    } else {
        ALOGI("watch %s\n", dirname);
    }
    return wd;
}

void monitor_function()
{
    BTMON("enter monitor_function");
    int wd = 0;
    int monitor;
    int index;
    char *dir[DIR_NUM] = {BT_FW_HCI_VAL, BT_FW_ACL_VAL, BT_FW_LMP_VAL, BT_FW_KER_VAL, BT_FW_LOG_VAL, BT_FW_CTL_VAL};
    fdi_op = get_interface();
    if (fdi_op == NULL) {
        BTMON("can't get monitor interface\n");
        return;
    }

    monitor = inotify_init();
    if (-1 == monitor) {
        BTMON("monitor error");
        return;
    }

    for (index = 0; (index < DIR_NUM) && (wd >= 0); index++) {
        wd = watch_dir(monitor, dir[index], IN_ALL_EVENTS);
    }

    if (wd > 0) {
        process_inotify_event(monitor);
    }

    BTMON("monitor end\n");
}

void init_pthread()
{
    int ret;
    ret = pthread_create(&id, NULL, (void*)monitor_function, NULL);
    if (ret != 0) {
        ALOGI("init monitor pthread error!\n");
    } else {
        ALOGI("init monitor pthread\n");
    }
}

void exit_pthread()
{
    running = 0;
    pthread_join(id, NULL);
    ALOGI("mointor pthread end\n");
}
