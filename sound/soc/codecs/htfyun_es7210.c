/*
 * ALSA SoC ES7210 adc driver
 *
 * Author:      David Yang, <yangxiaohua@everest-semi.com>
 * Copyright:   (C) 2018 Everest Semiconductor Co Ltd.,
 *
 * Based on sound/soc/codecs/es7243.c by David Yang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  ES7210 is a 4-ch ADC of Everest
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <linux/regmap.h>

#include "htfyun_es7210.h"

#define ENABLE      1
#define DISABLE     0

#define MIC_CHN_16  16
#define MIC_CHN_14  14
#define MIC_CHN_12  12
#define MIC_CHN_10  10
#define MIC_CHN_8   8
#define MIC_CHN_6   6
#define MIC_CHN_4   4
#define MIC_CHN_2   2

#define ES7210_TDM_ENABLE   DISABLE
#define ES7210_CHANNELS_MAX MIC_CHN_2

#if ES7210_CHANNELS_MAX == MIC_CHN_2
#define ADC_DEV_MAXNUM  1
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_4
#define ADC_DEV_MAXNUM  1
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_6
#define ADC_DEV_MAXNUM  2
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_8
#define ADC_DEV_MAXNUM  2
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_10
#define ADC_DEV_MAXNUM  3
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_12
#define ADC_DEV_MAXNUM  3
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_14
#define ADC_DEV_MAXNUM  4
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_16
#define ADC_DEV_MAXNUM  4
#endif

#define ES7210_TDM_1LRCK_DSPA                 0
#define ES7210_TDM_1LRCK_DSPB                 1
#define ES7210_TDM_1LRCK_I2S                  2
#define ES7210_TDM_1LRCK_LJ                   3
#define ES7210_TDM_NLRCK_DSPA                 4
#define ES7210_TDM_NLRCK_DSPB                 5
#define ES7210_TDM_NLRCK_I2S                  6
#define ES7210_TDM_NLRCK_LJ                   7
#define ES7210_NORMAL_I2S                     8

#define ES7210_WORK_MODE    ES7210_NORMAL_I2S

#define ES7210_I2C_BUS_NUM          1
#define ES7210_CODEC_RW_TEST_EN     0
#define ES7210_IDLE_RESET_EN        1   //reset ES7210 when in idle time

#define ES7210_MIC_GAIN 0x1A  // need check hw design and channel
#define ES7210_AEC_GAIN 0x13  // need check hw design and channel

/*  to set internal mclk and adclrclk ratio   */

#define RATIO_768  0xC3
#define RATIO_256  0xC1
#define RATIO_128  0x01
#define RATIO_64   0x41 /* mclk from bclk pin */

#define ES7210_MCLK_LRCK_RATIO   RATIO_256

/* codec private data */
struct es7210_priv {
    struct regmap *regmap;
    struct i2c_client *i2c_client;
    unsigned int dmic_enable;
    unsigned int tdm_mode;
    struct delayed_work pcm_pop_work;
};

struct es7210_reg_config {
    unsigned char reg_addr;
    unsigned char reg_v;
};

static struct i2c_client *i2c_clt1[ADC_DEV_MAXNUM];
static struct es7210_priv *resume_es7210 = NULL;
static int es7210_init_reg = 0;
static int es7210_codec_num = 0;

static const struct regmap_config es7210_regmap_config = {
    .reg_bits = 8,  //Number of bits in a register address
    .val_bits = 8,  //Number of bits in a register value
};
/*
* ES7210 register cache
*/
static const u8 es7210_reg[] = {
    0x32, 0x40, 0x02, 0x04, 0x01, 0x00, 0x00, 0x20, /* 0 - 7 */
    0x10, 0x40, 0x40, 0x00, 0x00, 0x09, 0x00, 0x00, /* 8 - F */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 10 - 17 */
    0xf7, 0xf7, 0x00, 0xbf, 0xbf, 0xbf, 0xbf, 0x00, /* 18 - 1f */
    0x26, 0x26, 0x06, 0x26, 0x00, 0x00, 0x00, 0x00, /* 20 - 27 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 28 - 2f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 30 - 37 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x10, 0x00, /* 38 - 3f */
    0x80, 0x71, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, /* 40 - 47 */
    0x00, 0x00, 0x00, 0xff, 0xff,                   /* 48 - 4c */
};

static const struct es7210_reg_config es7210_tdm_reg_common_cfg1[] = {
    { 0x00, 0xFF },
    { 0x00, 0x32 },
    { 0x09, 0x30 },
    { 0x0A, 0x30 },
    { 0x23, 0x2a },
    { 0x22, 0x0a },
    { 0x21, 0x2a },
    { 0x20, 0x0a },
};

static const struct es7210_reg_config es7210_tdm_reg_fmt_cfg[] = {
    { 0x11, 0x63 },
    { 0x12, 0x01 },
};

static const struct es7210_reg_config es7210_tdm_reg_common_cfg2[] = {
    { 0x40, 0xC3 },
    { 0x41, 0x70 },
    { 0x42, 0x70 },
    { 0x43, 0x1E },
    { 0x44, 0x1E },
    { 0x45, 0x1E },
    { 0x46, 0x1E },
    { 0x47, 0x08 },
    { 0x48, 0x08 },
    { 0x49, 0x08 },
    { 0x4A, 0x08 },
    { 0x07, 0x20 },
};

static const struct es7210_reg_config es7210_tdm_reg_mclk_cfg[] = {
    { 0x02, 0xC1 },
};

static const struct es7210_reg_config es7210_tdm_reg_common_cfg3[] = {
    { 0x06, 0x00 },
    { 0x4B, 0x0F },
    { 0x4C, 0x0F },
    { 0x00, 0x71 },
    { 0x00, 0x41 },
};

static int es7210_read(u8 reg, u8 *rt_value, struct i2c_client *client)
{
    int ret;
    u8 read_cmd[3] = {0};
    u8 cmd_len = 0;

    read_cmd[0] = reg;
    cmd_len = 1;

    if (client->adapter == NULL) {
        pr_err("es7210_read client->adapter==NULL\n");
    }

    ret = i2c_master_send(client, read_cmd, cmd_len);
    if (ret != cmd_len) {
        pr_err("es7210_read error1\n");
        return -1;
    }

    ret = i2c_master_recv(client, rt_value, 1);
    if (ret != 1) {
        pr_err("es7210_read error2, ret = %d.\n", ret);
        return -1;
    }

    return 0;
}

static int es7210_write(u8 reg, unsigned char value, struct i2c_client *client)
{
    int ret;
    u8 write_cmd[2] = {0};
    write_cmd[0] = reg;
    write_cmd[1] = value;

    ret = i2c_master_send(client, write_cmd, 2);
    if (ret != 2) {
        pr_err("es7210_write error->[REG-0x%02x,val-0x%02x]\n", reg, value);
        return -1;
    }
    return 0;
}

static int es7210_update_bits(u8 reg, u8 mask, u8 value, struct i2c_client *client)
{
    u8 val_old = 0, val_new = 0;

    es7210_read(reg, &val_old, client);
    val_new = (val_old & ~mask) | (value & mask);
    if (val_new != val_old) {
        es7210_write(reg, val_new, client);
    }

    return 0;
}

/*
static int es7210_multi_chips_read(u8 reg, unsigned char *rt_value)
{
    u8 i;

    for(i=0; i< ADC_DEV_MAXNUM; i++){
        es7210_read(reg, rt_value++, i2c_clt1[i]);
    }

    return 0;
}
*/

static int es7210_multi_chips_write(u8 reg, unsigned char value)
{
    u8 i;
    for (i = 0; i < ADC_DEV_MAXNUM; i++) {
        es7210_write(reg, value, i2c_clt1[i]);
    }

    return 0;
}

static int es7210_multi_chips_update_bits(u8 reg, u8 mask, u8 value)
{
    u8 i;

    for (i = 0; i < ADC_DEV_MAXNUM; i++) {
        es7210_update_bits(reg, mask, value, i2c_clt1[i]);
    }

    return 0;
}

/*
* to initialize es7210 for tdm mode
*/
static void es7210_tdm_init_codec(u8 mode)
{
    int cnt, channel, i;
    printk("begin->>>>>>>>>>%s!\n", __func__);
    for (cnt = 0; cnt < sizeof(es7210_tdm_reg_common_cfg1) / sizeof(es7210_tdm_reg_common_cfg1[0]); cnt++) {
        es7210_multi_chips_write(es7210_tdm_reg_common_cfg1[cnt].reg_addr, es7210_tdm_reg_common_cfg1[cnt].reg_v);
    }
    switch (mode) {
        case ES7210_TDM_1LRCK_DSPA:
            /*
            * Here to set TDM format for DSP-A mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x63, i2c_clt1[cnt]);
                es7210_write(ES7210_SDP_CFG2_REG12, 0x01, i2c_clt1[cnt]);
            }
            break;
        case ES7210_TDM_1LRCK_DSPB:
            /*
            * Here to set TDM format for DSP-B mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x73, i2c_clt1[cnt]);
                es7210_write(ES7210_SDP_CFG2_REG12, 0x01, i2c_clt1[cnt]);
            }
            break;
        case ES7210_TDM_1LRCK_I2S:
            /*
            * Here to set TDM format for I2S mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x60, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG2_REG12, 0x02, i2c_clt1[cnt]);
            }
            break;
        case ES7210_TDM_1LRCK_LJ:
            /*
            * Here to set TDM format for Left Justified mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x61, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG2_REG12, 0x02, i2c_clt1[cnt]);
            }
            break;
        case ES7210_TDM_NLRCK_DSPA:
            /*
            * set format, dsp-a with multiple LRCK tdm mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x63, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                if (cnt == 0) {
                    /*
                    * set tdm flag in the interface
                    * chip
                    */
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x07, i2c_clt1[cnt]);
                } else {
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x03, i2c_clt1[cnt]);
                }
            }
            break;
        case ES7210_TDM_NLRCK_DSPB:
            /*
            * set format, dsp-b with multiple LRCK tdm mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x73, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                if (cnt == 0) {
                    /*
                    * set tdm flag in the interface
                    * chip
                    */
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x07, i2c_clt1[cnt]);
                } else {
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x03, i2c_clt1[cnt]);
                }
            }
            break;
        case ES7210_TDM_NLRCK_I2S:
            /*
            * set format, I2S with multiple LRCK tdm mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x60, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                if (cnt == 0) {
                    /*
                    * set tdm flag in the interface
                    * chip
                    */
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x07, i2c_clt1[cnt]);
                } else {
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x03, i2c_clt1[cnt]);
                }
            }
            break;
        case ES7210_TDM_NLRCK_LJ:
            /*
            * set format, left justified with multiple LRCK
            * tdm mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x61, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                if (cnt == 0) {
                    /*
                    * set tdm flag in the interface
                    * chip
                    */
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x07, i2c_clt1[cnt]);
                } else {
                    es7210_write(ES7210_SDP_CFG2_REG12, 0x03, i2c_clt1[cnt]);
                }
            }
            break;
        case ES7210_NORMAL_I2S:
            /*
            * Here to set Normal  i2s mode format for SD1/SD2 output
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x60, i2c_clt1[cnt]);
            }
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG2_REG12, 0x00, i2c_clt1[cnt]);
            }
            break;
        default:
            /*
            * here to disable tdm and set i2s-16bit for
            * normal mode
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_SDP_CFG1_REG11, 0x60, i2c_clt1[cnt]);
                es7210_write(ES7210_SDP_CFG2_REG12, 0x00, i2c_clt1[cnt]);
            }
            break;
    }
    for (cnt = 0; cnt < sizeof(es7210_tdm_reg_common_cfg2) / sizeof(es7210_tdm_reg_common_cfg2[0]); cnt++) {
        es7210_multi_chips_write(es7210_tdm_reg_common_cfg2[cnt].reg_addr, es7210_tdm_reg_common_cfg2[cnt].reg_v);
    }
    switch (mode) {
        case ES7210_TDM_1LRCK_DSPA:
        case ES7210_TDM_1LRCK_DSPB:
        case ES7210_TDM_1LRCK_I2S:
        case ES7210_TDM_1LRCK_LJ:
        case ES7210_NORMAL_I2S:
            /*
            * to set internal mclk
            * here, we assume that cpu/soc always provides
            * 256FS i2s clock
            * to es7210.
            * dll bypassed, use clock doubler to get double
            * frequency for
            * internal modem which need
            * 512FS clock. the clk divider ratio is 1.
            * user must modify the setting of register0x02
            * according to FS
            * ratio provided by CPU/SOC.
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_MCLK_CTL_REG02, ES7210_MCLK_LRCK_RATIO, i2c_clt1[cnt]);
            }
            break;
        case ES7210_TDM_NLRCK_DSPA:
        case ES7210_TDM_NLRCK_DSPB:
        case ES7210_TDM_NLRCK_I2S:
        case ES7210_TDM_NLRCK_LJ:
            /*
            * Here to set TDM format for DSP-A with
            * multiple LRCK TDM mode
            */
            channel = ES7210_CHANNELS_MAX;
            /*
            * Set the microphone numbers in array
            */
            switch (channel) {
                case 2:
                    /* ES7210_CHANNELS_MAX=2 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x10);
                    break;
                case 4:
                    /* ES7210_CHANNELS_MAX=4 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x20);
                    break;
                case 6:
                    /* ES7210_CHANNELS_MAX=6 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x30);
                    break;
                case 8:
                    /* ES7210_CHANNELS_MAX=8 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x40);
                    break;
                case 10:
                    /* ES7210_CHANNELS_MAX=10 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x50);
                    break;
                case 12:
                    /* ES7210_CHANNELS_MAX=12 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x60);
                    break;
                case 14:
                    /* ES7210_CHANNELS_MAX=14 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x70);
                    break;
                case 16:
                    /* ES7210_CHANNELS_MAX=16 */
                    es7210_multi_chips_write(ES7210_MODE_CFG_REG08, 0x80);
                    break;
                default:
                    break;
            }
            /*
            * to set internal mclk
            * here, we assume that cpu/soc always provides
            * 256FS i2s clock
            * to es7210 and there is four
                * es7210 devices in tdm link. so the
            * internal FS in es7210
            * is only FS/4;
            * dll bypassed, clock doubler bypassed. the clk
            * divider ratio is
            * 2. so the clock of internal
                * modem equals to (256FS / (FS/4) / 2) * FS
            * = 512FS
            * user must modify the setting of register0x02
            * according to FS
            * ratio provided by CPU/SOC.
            */
            es7210_multi_chips_write(ES7210_MCLK_CTL_REG02, ES7210_MCLK_LRCK_RATIO);// NFS MODE:RATIO_768 ,12.288M/48K(16K 6 CH),12.288M/64K(16K 8 CH)
            break;
        default:
            /*
            * to set internal mclk for normal mode
            * here, we assume that cpu/soc always provides
            * 256FS i2s clock
            * to es7210.
            * dll bypassed, use clock doubler to get double
            * frequency for
            * internal modem which need
            * 512FS clock. the clk divider ratio is 1.
            * user must modify the setting of register0x02
            * according to FS
            * ratio provided by CPU/SOC.
            */
            for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
                es7210_write(ES7210_MCLK_CTL_REG02, ES7210_MCLK_LRCK_RATIO, i2c_clt1[cnt]);
            }
            break;
    }
    for (cnt = 0; cnt < sizeof(es7210_tdm_reg_common_cfg3) / sizeof(es7210_tdm_reg_common_cfg3[0]); cnt++) {
        es7210_multi_chips_write(es7210_tdm_reg_common_cfg3[cnt].reg_addr, es7210_tdm_reg_common_cfg3[cnt].reg_v);
    }
    /*
    * Mute All ADC
    */
    printk("enter es7210_multi_chips_update_bits  %s\n", __func__);
    for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
        es7210_write(ES7210_ADC34_MUTE_REG14, 0x03, i2c_clt1[cnt]);
        es7210_write(ES7210_ADC12_MUTE_REG15, 0x03, i2c_clt1[cnt]);
    }
    for (i = 0; i < ADC_DEV_MAXNUM; i++) {
        if (i == 0) {
            /* set first es7210 PGA GAIN */
            es7210_write(ES7210_MIC1_GAIN_REG43, ES7210_MIC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC2_GAIN_REG44, ES7210_MIC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC3_GAIN_REG45, ES7210_MIC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC4_GAIN_REG46, ES7210_MIC_GAIN, i2c_clt1[i]);
        }
        if (i == 1) {
            /* set second es7210 PGA GAIN */
            es7210_write(ES7210_MIC1_GAIN_REG43, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC2_GAIN_REG44, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC3_GAIN_REG45, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC4_GAIN_REG46, ES7210_AEC_GAIN, i2c_clt1[i]);
        }
        if (i == 2) {
            /* set third es7210 PGA GAIN */
            es7210_write(ES7210_MIC1_GAIN_REG43, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC2_GAIN_REG44, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC3_GAIN_REG45, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC4_GAIN_REG46, ES7210_AEC_GAIN, i2c_clt1[i]);
        }

        if (i == 3) {
            /* set third es7210 PGA GAIN */
            es7210_write(ES7210_MIC1_GAIN_REG43, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC2_GAIN_REG44, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC3_GAIN_REG45, ES7210_AEC_GAIN, i2c_clt1[i]);
            es7210_write(ES7210_MIC4_GAIN_REG46, ES7210_AEC_GAIN, i2c_clt1[i]);
        }
    }
    printk("exit->>>>>>>>>>%s!\n", __func__);
}

static void es7210_unmute(void)
{
    printk("enter into %s\n", __func__);
    es7210_multi_chips_update_bits(ES7210_ADC34_MUTE_REG14, 0x03, 0x00);
    es7210_multi_chips_update_bits(ES7210_ADC12_MUTE_REG15, 0x03, 0x00);
}

static void pcm_pop_work_events(struct work_struct *work)
{
    printk("enter into %s\n", __func__);
    es7210_unmute();
    es7210_init_reg = 1;
}

static int es7210_mute(int mute)
{
    printk("enter into %s, mute = %d\n", __func__, mute);
    if (mute) {
        es7210_multi_chips_update_bits(ES7210_ADC34_MUTE_REG14, 0x03, 0x03);
        es7210_multi_chips_update_bits(ES7210_ADC12_MUTE_REG15, 0x03, 0x03);
    } else {
        es7210_multi_chips_update_bits(ES7210_ADC34_MUTE_REG14, 0x03, 0x00);
        es7210_multi_chips_update_bits(ES7210_ADC12_MUTE_REG15, 0x03, 0x00);
    }

    return 0;
}

static int es7210_pcm_startup()
{
    if (resume_es7210 && es7210_init_reg == 0) {
        schedule_delayed_work(&resume_es7210->pcm_pop_work, msecs_to_jiffies(100));
    }

    return 0;
}

int es7210_pcm_hw_params(struct snd_pcm_hw_params *params)
{
    int i;
    es7210_multi_chips_update_bits(ES7210_RESET_CTL_REG00, 0x30, 0x30);
    for (i = 0; i < ADC_DEV_MAXNUM; i++) {
        /* set es7210 bit size */
        switch (params_format(params)) {
            case SNDRV_PCM_FORMAT_S16_LE:
                es7210_update_bits(0x11, 0xE0, 0x60, i2c_clt1[i]);
                break;
            case SNDRV_PCM_FORMAT_S20_3LE:
                es7210_update_bits(0x11, 0xE0, 0x20, i2c_clt1[i]);
                break;
            case SNDRV_PCM_FORMAT_S24_3LE:
                es7210_update_bits(0x11, 0xE0, 0x00, i2c_clt1[i]);
                break;
            case SNDRV_PCM_FORMAT_S24_LE:
            case SNDRV_PCM_FORMAT_S32_LE:
                es7210_update_bits(0x11, 0xE0, 0x80, i2c_clt1[i]);
                break;
        }
    }
    es7210_multi_chips_update_bits(ES7210_RESET_CTL_REG00, 0x30, 0x00);
    switch (params_rate(params)) {
        case 16000:
            break;
        case 32000:
            break;
        case 48000:
            break;
        case 64000:
            break;
        default:
            break;
    }
    return 0;
}

int es7210_suspend()
{
    int i = 0;

    printk("jsl es7210_suspend enter\n");

    for (i = 0; i < ADC_DEV_MAXNUM; i++) {
        if (i2c_clt1[i] && i == 1) {
            es7210_write(0x06, 0x05, i2c_clt1[i]);
            es7210_write(0x05, 0x1B, i2c_clt1[i]);
            es7210_write(0x06, 0x5C, i2c_clt1[i]);
            es7210_write(0x07, 0x3F, i2c_clt1[i]);
            es7210_write(0x08, 0x4B, i2c_clt1[i]);
            es7210_write(0x09, 0x9F, i2c_clt1[i]);
        } else if (i2c_clt1[i]) {
            /*turn off mic 1/2/3/4 */
            es7210_write(0x4B, 0xFF, i2c_clt1[i]);
            es7210_write(0x4C, 0xFF, i2c_clt1[i]);
            es7210_write(0x0B, 0xD0, i2c_clt1[i]);
            es7210_write(0x40, 0x80, i2c_clt1[i]);
            es7210_write(0x01, 0x7F, i2c_clt1[i]);
            es7210_write(0x06, 0x07, i2c_clt1[i]);
        }
    }

    es7210_init_reg = 0;
    return 0;
}

int es7210_resume()
{
    int i = 0;

    if (resume_es7210) {
        es7210_tdm_init_codec(resume_es7210->tdm_mode);
        es7210_pcm_startup();
    }
    return 0;
}

static ssize_t es7210_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val = 0, flag = 0;
    u8 i = 0, reg, num, value_w, value_r;

    struct es7210_priv *es7210 = dev_get_drvdata(dev);
    val = simple_strtol(buf, NULL, 16);
    flag = (val >> 16) & 0xFF;

    if (flag) {
        reg = (val >> 8) & 0xFF;
        value_w = val & 0xFF;
        printk("\nWrite: start REG:0x%02x,val:0x%02x,count:0x%02x\n", reg, value_w, flag);
        while (flag--) {
            es7210_write(reg, value_w, es7210->i2c_client);
            printk("Write 0x%02x to REG:0x%02x\n", value_w, reg);
            reg++;
        }
    } else {
        reg = (val >> 8) & 0xFF;
        num = val & 0xff;
        printk("\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);
        do {
            value_r = 0;
            es7210_read(reg, &value_r, es7210->i2c_client);
            printk("REG[0x%02x]: 0x%02x;  ", reg, value_r);
            reg++;
            i++;
            if ((i == num) || (i % 4 == 0)) {
                printk("\n");
            }
        } while (i < num);
    }

    return count;
}

static ssize_t es7210_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("echo flag|reg|val > es7210\n");
    printk("eg read star address=0x06,count 0x10:echo 0610 >es7210\n");
    printk("eg write star address=0x90,value=0x3c,count=4:echo 4903c >es7210\n");
    return 0;
}

static DEVICE_ATTR(es7210, 0644, es7210_show, es7210_store);

static struct attribute *es7210_debug_attrs[] = {
    &dev_attr_es7210.attr,
    NULL,
};

static struct attribute_group es7210_debug_attr_group = {
    .name   = "es7210_debug",
    .attrs  = es7210_debug_attrs,
};

/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
static int es7210_i2c_probe(struct i2c_client *i2c_client, const struct i2c_device_id *i2c_id)
{
    struct es7210_priv *es7210;
    int ret;

    printk("begin->>>>>>>>>>%s!\n", __func__);

    es7210 = devm_kzalloc(&i2c_client->dev, sizeof(struct es7210_priv), GFP_KERNEL);
    if (es7210 == NULL) {
        return -ENOMEM;
    }
    es7210->i2c_client = i2c_client;
    es7210->tdm_mode =  ES7210_WORK_MODE;  //to set tdm mode or normal mode
    i2c_set_clientdata(i2c_client, es7210);
    if (i2c_id->driver_data < ADC_DEV_MAXNUM) {
        i2c_clt1[i2c_id->driver_data] = i2c_client;
    }
    INIT_DELAYED_WORK(&es7210->pcm_pop_work, pcm_pop_work_events);
    es7210_tdm_init_codec(es7210->tdm_mode);
    resume_es7210 = es7210;
    ret = sysfs_create_group(&i2c_client->dev.kobj, &es7210_debug_attr_group);
    if (ret) {
        pr_err("failed to create attr group\n");
    }
    printk("%s success!\n", __func__);
    return ret;
}

static  int es7210_i2c_remove(struct i2c_client *client)
{
    sysfs_remove_group(&client->dev.kobj, &es7210_debug_attr_group);

    return 0;
}

static const struct i2c_device_id es7210_i2c_id[] = {
#if ES7210_CHANNELS_MAX > 0
    { "ES7210_MicArray_0", 0 },//es7210_0
#endif

#if ES7210_CHANNELS_MAX > 4
    { "ES7210_MicArray_1", 1 },//es7210_1
#endif

#if ES7210_CHANNELS_MAX > 8
    { "ES7210_MicArray_2", 2 },//es7210_2
#endif

#if ES7210_CHANNELS_MAX > 12
    { "ES7210_MicArray_3", 3 },//es7210_3
#endif
    { }
};
MODULE_DEVICE_TABLE(i2c_client, es7210_i2c_id);

static const struct of_device_id es7210_dt_ids[] = {
#if ES7210_CHANNELS_MAX > 0
    { .compatible = "ES7210_MicArray_0", },//es7210_0
#endif

#if ES7210_CHANNELS_MAX > 4
    { .compatible = "ES7210_MicArray_1", },//es7210_1
#endif

#if ES7210_CHANNELS_MAX > 8
    { .compatible = "ES7210_MicArray_2", },//es7210_2
#endif

#if ES7210_CHANNELS_MAX > 12
    { .compatible = "ES7210_MicArray_3", },//es7210_3
#endif
    { }
};
MODULE_DEVICE_TABLE(of, es7210_dt_ids);

static struct i2c_driver es7210_i2c_driver = {
    .driver = {
        .name = "es7210",
        .owner = THIS_MODULE,
        .of_match_table = es7210_dt_ids,
    },
    .probe = es7210_i2c_probe,
    .remove   = es7210_i2c_remove,
    .id_table = es7210_i2c_id,
};

module_i2c_driver(es7210_i2c_driver);

MODULE_DESCRIPTION("ASoC ES7210 audio adc driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com> / info@everest-semi.com");
MODULE_LICENSE("GPL v2");
