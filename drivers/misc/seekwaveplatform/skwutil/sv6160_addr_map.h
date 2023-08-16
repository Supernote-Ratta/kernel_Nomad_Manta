/*
 * SeekWave - Platform data for sv6160 platform.
 *
 * This software is distributed under the terms of the GNU General Public
 * License ("GPL") version 2, as published by the Free Software Foundation.
 *
 */

#ifndef _SV6160_ADDR_MAP_H
#define _SV6160_ADDR_MAP_H

#include <linux/kernel.h>
#include <linux/version.h>

/*****************************************************************************
Copyright: 2020-2021, Seekwave Tech. Co., Ltd.
File name: SV6160_ADDR_MAP.H
Description:
Author: JUNWEI.JIANG
Version: V1.00
Date: 2021-07-29
History: NONE
 *****************************************************************************/

#define IRAM_BASE_ADDRESS		0x100000
#define DRAM_BASE_ADDRESS		0x20200000
/*--------------------------------------------------------------------------*/
//AON IRAM 1:BSP AON IRAM SIZE 0x8000--32K,2:WIFI IRAM　AON　SIZE：32K 0x8000
/*--------------------------------------------------------------------------*/
#define PLATFROM_IRAM_AON_SIZE		0x10000 	//64K
/*--------------------------------------------------------------------------*/
//SV6160 IRAM 64K listing
//Total memory size 512K
/*--------------------------------------------------------------------------*/
/*--------------------------PLATFORM IRAM-----------------------------------*/
//platform iram pd addr:0x110000
//psb_dl_addr : 0x110000
#define PLATFROM_IRAM_PD_ADDR		(IRAM_BASE_ADDRESS + PLATFROM_IRAM_AON_SIZE)
#define PLATFROM_IRAM_PD_SIZE		(0x2800) 	//10K
/*---------------------------WIFI IRAM--------------------------------------*/
//wifi iram pd addr: 0x112800
//wifi dload_iram_addr 0x112800
#define WIFI_IRAM_PD_ADDR		(PLATFROM_IRAM_PD_ADDR + PLATFROM_IRAM_PD_SIZE)
#define WIFI_IRAM_PD_SIZE		(0x30800) 	//194K
/*---------------------------BT IRAM----------------------------------------*/
//bt iram pd addr :0x143000
#define BT_IRAM_PD_ADDR			(WIFI_IRAM_PD_ADDR+WIFI_IRAM_PD_SIZE)
#define BT_IRAM_PD_SIZE			(0x37000)	//220K


/*--------------------------------------------------------------------------*/
//SV6160 DRAM WR
//Total memory size 256K
//dram need read back to AP
/*--------------------------------------------------------------------------*/
/*--------------------------PLATFORM DRAM-----------------------------------*/
//platform dram pd addr:0x20200000
#define PLATFROM_DRAM_PD_ADDR		(DRAM_BASE_ADDRESS)
#define PLATFROM_DRAM_PD_SIZE		(0xAC00) 	//43K
/*---------------------------WIFI DRAM--------------------------------------*/
//wifi dram pd addr: 0x2020AC00
#define WIFI_DRAM_PD_ADDR		(PLATFROM_DRAM_PD_ADDR + PLATFROM_DRAM_PD_SIZE)
#define WIFI_DRAM_PD_SIZE		(0x25400) 	//149K
/*---------------------------BT DRAM----------------------------------------*/
//bt dram pd addr :0x20230000
#define BT_DRAM_PD_ADDR			(WIFI_DRAM_PD_ADDR+WIFI_DRAM_PD_SIZE)
#define BT_DRAM_PD_SIZE			(0x100000)	//64K

/*---------------------------------END--------------------------------------*/

#endif //_SV6160_ADDR_MAP_H

