/* drivers/input/touchscreen/huiontablet.h
 * 
 * 2010 - 2013 HUION Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the HUION's TP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */

#ifndef _HUION_XX_H_
#define _HUION_XX_H_

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
//#include "../tp_suspend.h"
#include "tp_suspend.h"


//#include <mach/gpio.h>
//#include <linux/earlysuspend.h>

#define CONFIG_8_9  0
#define DEBUG_SWITCH  1

//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        0
#if CONFIG_8_9
#else
#define GTP_CHANGE_X2Y        1
#define GTP_X_REVERSE_ENABLE	1
#define GTP_Y_REVERSE_ENABLE	0
#endif
#define GTP_DRIVER_SEND_CFG   0
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  0

#if defined(CONFIG_CHROME_PLATFORMS)
#define GTP_ICS_SLOT_REPORT   1
#else
#define GTP_ICS_SLOT_REPORT   0
#endif

#define GTP_AUTO_UPDATE       0    // auto update fw by .bin file as default
#define GTP_HEADER_FW_UPDATE  0    // auto update fw by huion_default_FW in gt9xx_firmware.h, function together with GTP_AUTO_UPDATE
#define GTP_AUTO_UPDATE_CFG   0    // auto update config by .cfg file, function together with GTP_AUTO_UPDATE


#define GTP_WITH_PEN          0
#define GTP_PEN_HAVE_BUTTON   0    // active pen has buttons, function together with GTP_WITH_PEN


#define GTP_DEBUG_ON          1
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     1

/* init use fixed clk num */
/* if open, u8 p_main_clk[6] = {69,69,69,69,69,167}; */
#define GTP_USE_FIXED_CLK     1

#define PEN_DOWN 1
#define PEN_RELEASE 0
#define PEN_DOWN_UP 2 //fjp

#define TOUCH_SCREEN_MAX_X			(41932)
#define TOUCH_SCREEN_MAX_Y			(31450)


#define SCREEN_MAP_SWITCH    1
#define DISPLAY_SCREEN_MAX_X			(1872)//(1404)
#define DISPLAY_SCREEN_MAX_Y			(1404)//(1872)
#define TOUCH_X_AXIS_MAPPING (TOUCH_SCREEN_MAX_X/DISPLAY_SCREEN_MAX_X)     //(39800/SCREEN_MAX_X) //31=(39800/1280) 
#define TOUCH_Y_AXIS_MAPPING (TOUCH_SCREEN_MAX_Y/DISPLAY_SCREEN_MAX_Y)  //17=(24800/800)
#define TOUCHE_MAX_PRESS_VALUE   2047


struct huion_ts_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
    //struct early_suspend early_suspend;
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  int_trigger_type;
    u8  enter_update;
    u8  huion_is_suspend;
    u8  huion_rawdiff_mode;
    u8  fw_error;
 	u8  cfg_file_num;
    //add struct tp_device by Sam
    struct  tp_device  tp;

    //add by Daniel(yc)
    int irq;
    int irq_pin;
    int pwr_pin;
    int rst_pin;
    int tp_select_pin;
    int rst_val;
	  u8 pendown;
    unsigned long irq_flags;

    struct regulator *tp_regulator;

};

extern u16 show_len;
extern u16 total_len;

// STEP_2(REQUIRED): Customize your I/O ports & I/O operations
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define GTP_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)              gpio_free(pin)
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

// STEP_3(optional): Specify your special config info if needed
#if GTP_CUSTOM_CFG
  #define GTP_MAX_HEIGHT   800
  #define GTP_MAX_WIDTH    480
  #define GTP_INT_TRIGGER  0            // 0: Rising 1: Falling
#else
  #define GTP_MAX_HEIGHT   4096
  #define GTP_MAX_WIDTH    4096
  #define GTP_INT_TRIGGER  1
#endif


// STEP_4(optional): If keys are available and reported as keys, config your key info here                             
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB  {KEY_MENU, KEY_HOME, KEY_BACK}
#endif

//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION          "V2.2<2021/05/10>"
#define GTP_I2C_NAME                "Huion-Ts"
#define GT91XX_CONFIG_PROC_FILE     "hgtxx_config"
#define GTP_POLL_TIME         10    
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MIN_LENGTH 186
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL                  0
#define SUCCESS               1
#define SWITCH_OFF            0
#define SWITCH_ON             1

//******************** For GT9XXF Start **********************//

#define GTP_FL_FW_BURN              0x00
#define GTP_FL_ESD_RECOVERY         0x01
#define GTP_FL_READ_REPAIR          0x02

#define GTP_CHK_FW_MAX                  40
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH                "/data/huion_ref.bin"
#define GTP_MAIN_CLK_PATH               "/data/huion_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

//******************** For GT9XXF End **********************//


// Log define
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#if DEBUG_SWITCH
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("     <<-GTP-FUNC->>       Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

#else
#define GTP_INFO(fmt,arg...)
#define GTP_DEBUG(fmt,arg...)
#define GTP_DEBUG_ARRAY(array, num)
#define GTP_DEBUG_FUNC()
#endif
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)

//*****************************End of Part III********************************
#define TRUE    1
#define FALSE   0

#endif /* _HUION_XX_H_ */
