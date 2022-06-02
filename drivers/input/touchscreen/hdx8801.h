/*++
 
 Copyright (c) 2019-2022 HDX Technology (ShenZhen) Co., Ltd. All Rights Reserved.
 This PROPRIETARY SOFTWARE is the property of HDX Technology (ShenZhen) Co., Ltd. 
 and may contains trade secrets and/or other confidential information of HDX 
 Technology (ShenZhen) Co., Ltd. This file shall not be disclosed to any third party,
 in whole or in part, without prior written consent of HDX.  
 THIS PROPRIETARY SOFTWARE & ANY RELATED DOCUMENTATION ARE PROVIDED AS IS, 
 WITH ALL FAULTS, & WITHOUT WARRANTY OF ANY KIND. CHIPONE DISCLAIMS ALL EXPRESS OR 
 IMPLIED WARRANTIES.  
 
 File Name:    hdx8801.h
 Abstract:
               input driver.
 Author:       Walter Lee
 Date :        03,07,2019
 Version:      1.0
 History :
     2015,03,07, V0.1 first version  
 --*/



#ifndef __LINUX_HDX8801_H__
	#define __LINUX_HDX8801_H__
	
	/******* Include File section************/
	
	/******* Macro Define Section************/
	#define REPORT_MAX_X		0x37FF//0x4200//0x37FF
	#define REPORT_MAX_Y		0x1FFF//0x3200//0x1FFF
	#define DIG_MAX_P			0x03FF
	#define RD_COMMAND_BIT	8
	#define COMMAND_COUSE		10
	#define COMMAND_BYTE		5
	#define ACK_BIT					5
	/******* DATA  Define Section************/
	
	static u_int8_t command_list[COMMAND_COUSE][COMMAND_BYTE] = 
	{
		{0x01, 0x02, 0x04 , 0x03 , 0x10 },	// 0 Turn to Bootloader
		{0x01, 0x02, 0x04 , 0x03 , 0x00 },	// 1 Turn to App
		{0x01, 0x02, 0x07 , 0x01 , 0xAA },	// 2 Bootloader action
		{0x01, 0x02, 0x06 , 0x03 , 0x02 },	// 3 read firmware version
		{0x01, 0x02, 0x05 , 0x03 , 0x01 },	// 4 read product ID
		{0x01, 0x02, 0x03 , 0x55 , 0xAA },	// 5 Check State
		{0x01, 0x02, 0x02 , 0xA1 , 0xA0 },	// 6 read flash
		{0x01, 0x02, 0x01 , 0xA1 , 0xA0 },	// 7 write flash
		{0xDE, 0x55, 0x00 , 0x00 , 0x00 },	// 8 sleep
		{0xDE, 0x5A, 0x00 , 0x00 , 0x00 },  // 9 wake up
	};
	
	enum hdx_dig_report 
	{
		HDX_DIG_LEN = 0x0,
		HDX_DIG_MODE,
		HDX_DIG_STATUS,
		HDX_DIG_X_LOW,
		HDX_DIG_X_HI,
		HDX_DIG_Y_LOW,
		HDX_DIG_Y_HI,
		/* Z represents pressure value */
		HDX_DIG_Z_LOW,
		HDX_DIG_Z_HI,
		HDX_DIG_X_TILE,
		HDX_DIG_Y_TILE,
	};
	enum hdx_dig_state 
	{
		HDX_OUT_RANG = 0,
		HDX_IN_RANG = 0x10,
		HDX_TIP_SWITCH = 0x11,
		HDX_CUSTOM_BTN = 0x12,
		HDX_RIGHT_BTN = 0x13,
	};

#endif 

