/*****************************************************************
 *Copyright (C) 2021 Seekwave Tech Inc.
 *Filename : skw_sdio.h
 *Authors:seekwave platform
 *
 * This software is licensed under the terms of the the GNU
 * General Public License version 2, as published by the Free
 * Software Foundation, and may be copied, distributed, and
 * modified under those terms.
 *
 * This program is distributed in the hope that it will be usefull,
 * but without any warranty;without even the implied warranty of
 * merchantability or fitness for a partcular purpose. See the
 * GUN General Public License for more details.
 * **************************************************************/
#ifndef __SKW_BOOT_H__
#define __SKW_BOOT_H__

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/workqueue.h>
#if  KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
#include <uapi/linux/sched/types.h>
#else
#include <linux/sched.h>
#endif

#ifdef SKW_EXT_INC
#include "skw_platform_data.h"
#else
#include <linux/platform_data/skw_platform_data.h>
#endif

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#else
#include <linux/pm_wakeup.h>
#endif

#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
        #define skw_read_file   kernel_read
        #define skw_write_file  kernel_write
#else
        #define skw_read_file   vfs_read
        #define skw_write_file  __kernel_write
#endif

#if KERNEL_VERSION(5, 5, 0) <= LINUX_VERSION_CODE
        #define skw_wakeup_source_register(x, y)        wakeup_source_register(x,y)
#else
        #define skw_wakeup_source_register(x, y)        wakeup_source_register(y)
#endif

/****************************************************************
 *Description:the skwsdio log define and the skwsdio data debug,
 *Func: skwsdio_log, skwsdio_err, skwsdio_data_pr;
 *Calls:
 *Call By:
 *Input: skwsdio log debug informations
 *Output:
 *Return：
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-08-25
 * **************************************************************/
#define skwboot_log(fmt, args...) \
    pr_info("[SKWBOOT]:" fmt, ## args)

#define skwboot_err(fmt, args...) \
    pr_err("[SKWBOOT_ERR]:" fmt, ## args)

#define skwsdio_data_pr(level, prefix_str, prefix_type, rowsize,\
        groupsize, buf, len, asscii)\
        do{if(loglevel) \
            print_hex_dump(level, prefix_str, prefix_type, rowsize,\
                    groupsize, buf, len, asscii);\
        }while(0)

/**********************sdio boot interface start******************/

#define SKW_BOOT_START_ADDR             0x100000
#define SKW_CHIP_ID						0x40000000  //SV6160 chip id
/*add the 32bit*4 128bit */
struct img_head_data_t
{
	 unsigned int index;
	 unsigned int dl_addr;
	 unsigned int data_size;
	 unsigned int write_addr;
};
/*add the 32bit*4 128bit */
struct img_dl_data
{
	 unsigned int dl_addr;
	 unsigned int dl_info; /*type and the size*/
	 unsigned int write_addr;
};

#define CRC_16_L_SEED   0x80
#define CRC_16_L_POLYNOMIAL  0x8000
#define CRC_16_POLYNOMIAL  0x1021
#define IRAM_CRC_OFFSET     0
#define DRAM_CRC_OFFSET     0

/*slp reg add the ap send the irq to cp reg*/
#define SKW_SDIO_PD_DL_AP2CP_BSP        0x160 //download done  or first boot setup addrn
#define SDIOHAL_PD_DL_AP2CP_WIFI		0x161
#define SDIOHAL_PD_DL_AP2CP_BT          0x162
#define SDIOHAL_PD_DL_ALL               0x163
#define SKW_SDIO_DL_POWERON_MODULE		0x164 //Poweron CP Moudle  1 WIFI 2:BT
#define SKW_SDIO_PLD_DMA_TYPE			0x165
//#define SDIOHAL_PD_DL_AP2CP_SIG5        0x165
#define SDIOHAL_CPLOG_TO_AP_SWITCH      0x166
#define SDIOHAL_PD_DL_AP2CP_SIG7        0x167
#define SKW_SDIO_CREDIT_TO_CP           0x168

// CP signal 3
#define SKW_SDIO_RX_CHANNEL_FTL0        0x16C
#define SKW_SDIO_RX_CHANNEL_FTL1        0x16D

/*slp reg get the cp dl state reg*/
#define SKW_SDIO_DL_CP2AP_BSP         0x180 //poweron OK ? 1: WIFI 2:BT
#define SKW_SDIO_CP2AP_FIFO_IND       0x181 //CP_RX FIFO Empty Indiacation.
#define SDIOHAL_PD_DL_CP2AP_BT          0x182
#define SDIOHAL_PD_DL_CP2AP_ALL         0x183
#define SDIOHAL_PD_DL_CP2AP_SIG4        0x184
#define SDIOHAL_PD_DL_CP2AP_SIG5        0x185
#define SDIOHAL_PD_DL_CP2AP_SIG6        0x186
#define SDIOHAL_PD_DL_CP2AP_SIG7        0x187

#define SKWSDIO_AP2CP_IRQ               0x1b0  //AP to CP interrupt and used BIT4 set 1 :fifth bit
#if 0
/****************************************************************
 *Description:
 *Func:
 *Calls:
 *Call By:
 *Input:
 *Output:
 *Return：
 *Others:
 *Author：
 *Date:
 * **************************************************************/
int sdio_dloader(unsigned int subsys_num);
int skw_sdio_dl_bspbin(void);
int skw_sdio_dl_btbin(void);
int skw_sdio_dl_wifibin(void);
#endif
/********************sdio boot interface end*********************/

/********************usb download infterface*********************/

enum dma_type_en{
	ADMA=1,
	SDMA,
};

enum skw_service_ops {
	SKW_NO_SERVICE =0,
	SKW_WIFI_START,
	SKW_WIFI_STOP,
	SKW_BT_START,
	SKW_BT_STOP,
};

enum serivce_state{
	STOP=1,
	START,
};
struct seekwave_device {
	char *file_path;
	char *iram_file_path;
	char *dram_file_path;
	char *img_data;
	int  img_size;
	int host_gpio;/*GPIO0_A3*/
	int chip_gpio;/*GPIO2_D2*/
	int chip_en;/*GPIO0_B1*/
	char *iram_img_data;
	char *dram_img_data;

    void *dl_bin;//load the img
	unsigned int iram_dl_addr;
	unsigned int iram_dl_size;
	unsigned int dram_dl_addr;
	unsigned int dram_dl_size;
	unsigned int setup_addr;//setup address
	unsigned int save_setup_addr;//send the setup address register
	unsigned int dl_addr;//
	unsigned int dl_acount_addr;
	unsigned int dl_size;
	unsigned int first_dl_flag;
	unsigned int first_bootwifi_flag;//01:first boot BSP 10:firstbootwifi 11:firstbootBT
	unsigned int first_bootbt_flag;//01:first boot BSP 10:firstbootwifi 11:firstbootBT
	unsigned int dl_module;
	unsigned int dma_type_addr;//1:ADMA,2:SDMA
	unsigned int dma_type;//1:ADMA,2:SDMA
	unsigned int head_addr;
	unsigned int tail_addr;
	unsigned int bsp_head_addr;
	unsigned int bsp_tail_addr;
	unsigned int wifi_head_addr;
	unsigned int wifi_tail_addr;
	unsigned int bt_head_addr;
	unsigned int bt_tail_addr;
	unsigned int service_req_flag;
	int poweron_flag;
	int poweron_wifi_flag;
	int poweron_bt_flag;
	int bt_service_state;
	int wifi_service_state;
	int service_ops;
	int dl_done_signal;
	int gpio_out;//host wakeup gpio0:/*GPIO0_A3*/,chip_wakeup gpio2:/*GPIO2_D2*/
	int gpio_in;//host wakeup gpio 0
	int gpio_val;
	int gpio_next_val;
	//int (*skw_dloader)(unsigned int subsys);
	unsigned int chip_id;
	unsigned int fpga_debug;
	unsigned int cp_boot_config;
	unsigned int iram_crc_offset;
	unsigned int dram_crc_offset;
	unsigned short iram_crc_val;
	unsigned short dram_crc_val;

	unsigned int iram_crc_en;
	unsigned int dram_crc_en;

};

extern int dloader_cpbin(int size, char *data);
int skw_ucom_init(void);
void skw_ucom_exit(void);
int seekwave_boot_start(u32 subsys);
int seekwave_boot_stop(u32 subsys);
#endif
