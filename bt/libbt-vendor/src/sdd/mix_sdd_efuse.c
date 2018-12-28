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


#define LOG_TAG "bt_vendor_efuse_sdd"

#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <utils/Log.h>
#include "sdd_if.h"
#include "sdd_defs.h"
#include "efuse.h"
#include "bt_vendor_xr.h"


#define SDD_DIR SDD_FILE_LOCATION"bt_sdd.bin"
#if 0
#ifdef NO_POWERDOWN_PROTECTION
#define MIXED_SDD_DIR SDD_DIR
#else
#define MIXED_SDD_DIR ""
#endif
#endif

#define EXPAND_BUFFER_SIZE (4 * 1024)
#define EXPAND_IES_MAX_SIZE (EXPAND_BUFFER_SIZE - sizeof(SDD_SECTION_HEADER_ELT) - 4)


#define MIX_SDD_EFUSE_VERBOSE(fmt, arg...)    ALOGV("<%s:%u>: " fmt "\n", __FUNCTION__, __LINE__, ##arg)
#define MIX_SDD_EFUSE_DBG(fmt, arg...)    ALOGD("<%s:%u>: " fmt "\n", __FUNCTION__, __LINE__, ##arg)
#define MIX_SDD_EFUSE_ERR(fmt, arg...)    ALOGE("<%s:%u>: " fmt "\n", __FUNCTION__, __LINE__, ##arg)

static uint8_t *efuse_to_sdd(uint8_t *pIE, int type, uint32_t start_bit, uint32_t bit_num)
{
#define EFUSE_DATA_BUFFER_SIZE 17
    uint8_t data[EFUSE_DATA_BUFFER_SIZE] = {0};
    uint32_t len = (bit_num + 7) >> 3;

    efuse_read(start_bit, bit_num, data);
    MIX_SDD_EFUSE_VERBOSE("efuse startbit: %d, bitnum: %d, data32:0x%x", start_bit, bit_num, *((uint32_t *)data));
    return SDD_WriteSectIe(pIE, type, len, data);
}

static uint8_t *ram_to_sdd(uint8_t *pIE, int type, uint8_t *data, uint32_t bit_num)
{
    uint32_t len = (bit_num + 7) >> 3;
    return SDD_WriteSectIe(pIE, type, len, data);
}

static uint8_t *write_efuse_ies(uint8_t *pIE)
{
    uint8_t *p_max = pIE + EXPAND_IES_MAX_SIZE;
    uint8_t *p = pIE;

    if (efuse_init() < 0) {
        MIX_SDD_EFUSE_ERR("efuse file exist but can't read");
        return p;
    }

    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_TX_PAPOWER_TRIM,    EFUSE_BT_TX_PAPOWER_TRIM_STARTBIT,     EFUSE_BT_TX_PAPOWER_TRIM_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_TX_PADRIVER_TRIM,     EFUSE_BT_TX_PADRIVER_TRIM_STARTBIT, EFUSE_BT_TX_PADRIVER_TRIM_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_TX_ISM_TRIM,         EFUSE_BT_TX_ISM_TRIM_STARTBIT,         EFUSE_BT_TX_ISM_TRIM_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_TX_IPM_TRIM,         EFUSE_BT_TX_IPM_TRIM_STARTBIT,         EFUSE_BT_TX_IPM_TRIM_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_ADC_OFFSET,             EFUSE_ADC_OFFSET_STARTBIT,             EFUSE_ADC_OFFSET_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_ADC_LSB,                 EFUSE_ADC_LSB_STARTBIT,             EFUSE_ADC_LSB_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_TNOM,                    EFUSE_TNOM_STARTBIT,                 EFUSE_TNOM_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_TOFFSET,                 EFUSE_TOFFSET_STARTBIT,             EFUSE_TOFFSET_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_PNOM_2441,             EFUSE_BT_PNOM_2441_STARTBIT,         EFUSE_BT_PNOM_2441_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_PNOM_2402,             EFUSE_BT_PNOM_2402_STARTBIT,         EFUSE_BT_PNOM_2402_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_BT_PNOM_2480,             EFUSE_BT_PNOM_2480_STARTBIT,         EFUSE_BT_PNOM_2480_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_SMPS_VSEL,             EFUSE_SMPS_VSEL_STARTBIT,             EFUSE_SMPS_VSEL_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_SMPS_BGTR,             EFUSE_SMPS_BGTR_STARTBIT,             EFUSE_SMPS_BGTR_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_SMPS_FTR,                 EFUSE_SMPS_FTR_STARTBIT,             EFUSE_SMPS_FTR_WIDTH);
    p = efuse_to_sdd(p, SDD_EFUSE_BT_SMPS_OCP,                 EFUSE_SMPS_OCP_STARTBIT,             EFUSE_SMPS_OCP_WIDTH);

    uint8_t data[128 / 8] = {0};
     if (efuse_read_field(EFUSE_FIELD_MAC, data) == 0)
        p = ram_to_sdd(p, SDD_EFUSE_MAC_ADDRESS,             data,                                 EFUSE_MAC_1_WIDTH);
    else
        MIX_SDD_EFUSE_DBG("efuse has no MAC address");

    if (efuse_read_field(EFUSE_FIELD_CHIPID, data) == 0)
        p = ram_to_sdd(p, SDD_EFUSE_CHIP_ID,                 data,                                 EFUSE_CHIP_ID1_WIDTH);

    efuse_deinit();

    if (p > p_max)
        MIX_SDD_EFUSE_ERR("writing EXPAND_BUFFER overflow");
    return p;
}

static int __sdd_data_create(uint8_t **pp_sdd, uint32_t *p_size, bool mix_efuse)
{
    uint8_t *pSdd;
    uint32_t sdd_file_size;
    FILE *sdd_file = fopen(SDD_DIR, "rb");
    uint32_t expand_size = mix_efuse ? EXPAND_BUFFER_SIZE : 0;

    if (sdd_file == NULL)
    {
        MIX_SDD_EFUSE_ERR("fopen %s failed, return %u: %s", SDD_DIR, (int)sdd_file, strerror(errno));
        return -1;
    }

    MIX_SDD_EFUSE_DBG("fopen %s success, return %u", SDD_DIR, (int)sdd_file);
    /* get sdd size */
    fseek(sdd_file, 0, SEEK_END);
    sdd_file_size = ftell(sdd_file);

    MIX_SDD_EFUSE_DBG("sdd file size: %d", sdd_file_size);

    pSdd = malloc(sdd_file_size + expand_size);
    if (pSdd == NULL)
    {
        MIX_SDD_EFUSE_ERR("no memory to malloc!");
        goto failed;
    }

    /* load sdd data */
    fseek(sdd_file, 0, SEEK_SET);
    fread(pSdd, 1, sdd_file_size, sdd_file);
    fclose(sdd_file);

    if (mix_efuse)
    {
        /* construct a efuse section */
        uint8_t *p = SDD_WriteSection(pSdd + sdd_file_size - SDD_LAST_SECTION_SIZE, SDD_BT_EFUSE_SECT_ID, write_efuse_ies);
        p = SDD_RegenerateLastSection(p);

        *pp_sdd = pSdd;
        *p_size = p - pSdd;
        return 0;
    }
    else
    {
        *pp_sdd = pSdd;
        *p_size = sdd_file_size;
        return 0;
    }

failed:
    if (pSdd != NULL)
        free(pSdd);
    if (sdd_file > 0)
        fclose(sdd_file);
    return -1;
}

int sdd_data_create(uint8_t **pp_sdd, uint32_t *p_size)
{
    bool efuse_exist = is_efuse_ready();
    return __sdd_data_create(pp_sdd, p_size, efuse_exist);
}

void sdd_data_destroy(uint8_t *pSdd)
{
    free(pSdd);
}


#if 0
static bool has_mixed_sdd()
{
#ifdef NO_POWERDOWN_PROTECTION
    uint8_t buffer[2048] = {0};

    FILE *sdd_file = fopen(SDD_DIR, "r+");

    fseek(sdd_file, 0, SEEK_END);
    uint32_t size = ftell(sdd_file);
    fseek(sdd_file, 0, SEEK_SET);

    fread(buffer, 1, size, sdd_file);
    fclose(sdd_file);

    if (SDD_FindSection(buffer, SDD_BT_EFUSE_SECT_ID))
        return 1;
    else
        return 0;
#else
    uint8_t standard_last_section[SDD_LAST_SECTION_SIZE], read_last_section[SDD_LAST_SECTION_SIZE] = {0};
    FILE *mixed_sdd_file = fopen(MIXED_SDD_DIR, "r+");
    if (mixed_sdd_file <= 0)
        return 0;

    fseek(mixed_sdd_file, -SDD_LAST_SECTION_SIZE, SEEK_END);
    fread(read_last_section, 1, SDD_LAST_SECTION_SIZE, mixed_sdd_file);
    fclose(mixed_sdd_file);

    SDD_RegenerateLastSection(standard_last_section);

    /* we believed mixed sdd is correct if the last section is correct */
    if (!memcmp(standard_last_section, read_last_section, SDD_LAST_SECTION_SIZE))
        return 1;
    else
        return 0;
#endif
}

static int mix_sdd_efuse_file()
{
    uint8_t expand[EXPAND_BUFFER_SIZE] = {0};

    /* 1. construct a efuse section */
    uint8_t *p = SDD_WriteSection(expand, SDD_BT_EFUSE_SECT_ID, write_efuse_ies);
    uint32_t expand_size = p - expand;
    p = SDD_RegenerateLastSection(p);
    uint32_t sdd_last_section_size = p - expand - expand_size; /* same as SDD_LAST_SECTION_SIZE */

    /* 2. update sdd */
    FILE *sdd_file = fopen(SDD_DIR, "r+");

    /* get sdd size */
    fseek(sdd_file, 0, SEEK_END);
    uint32_t size = ftell(sdd_file);

#ifndef NO_POWERDOWN_PROTECTION
    uint32_t cp_size = size - sdd_last_section_size;
    FILE *mixed_sdd_file = fopen(MIXED_SDD_DIR, "w+");

    /* copy a sdd file, but not copy all, for reason that we need
     * the last section to recognize the file written finished */
    fseek(sdd_file, 0, SEEK_SET);
    while (cp_size--)
        fputc(fgetc(sdd_file), mixed_sdd_file);
    fclose(sdd_file);
    sdd_file = mixed_sdd_file;
#endif

    /* expand first */
    fseek(sdd_file, size, SEEK_SET);
    fseek(sdd_file, expand_size, SEEK_CUR);
    fsync(sdd_file);

    /* write from last section */
    fseek(sdd_file, size - sdd_last_section_size, SEEK_SET);
    if (fwrite(expand, 1, expand_size + sdd_last_section_size, sdd_file) != expand_size + sdd_last_section_size)
        MIX_SDD_EFUSE_ERR("write faile!");
    fclose(sdd_file);

    return size + expand_size;
}

const char *get_sdd_dir()
{
    if (has_mixed_sdd())
        return MIXED_SDD_DIR;
    if (is_efuse_ready())
    {
        mix_sdd_efuse_file();
        return MIXED_SDD_DIR;
    }
    else
        return SDD_DIR;
}
#endif

