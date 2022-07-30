/*++
 
 Copyright (c) 2019-2022 HDX Technology (ShenZhen) Co., Ltd. All Rights Reserved.
 This PROPRIETARY SOFTWARE is the property of HDX Technology (ShenZhen) Co., Ltd. 
 and may contains trade secrets and/or other confidential information of HDX 
 Technology (ShenZhen) Co., Ltd. This file shall not be disclosed to any third party,
 in whole or in part, without prior written consent of HDX.  
 THIS PROPRIETARY SOFTWARE & ANY RELATED DOCUMENTATION ARE PROVIDED AS IS, 
 WITH ALL FAULTS, & WITHOUT WARRANTY OF ANY KIND. CHIPONE DISCLAIMS ALL EXPRESS OR 
 IMPLIED WARRANTIES.  
 
 File Name:    hdx8801.c
 Abstract:
               input driver.
 Author:       Walter Lee
 Date :        04,15,2015
 Version:      1.0
 History :
     2019,04,15 V0.1 first version  
 --*/

/******* Include File section************/
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
//#include <linux/earlysuspend.h>
//#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/ctype.h>

#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "hdx8801.h"
#include "update.h"

/******* Macro  Define Section************/
#define EnableDebug				0
#define HDX_DRIVER_NAME 	"hdx8801_touch"
#define DIG_BUF_SIZE 			9//8
#define BTN_TOOL_PEN_HOVER 	238

#define UPDATEFW					0
#define HDX_DEBUG					1
#define FIRMWARE_UPDATE_LENGTH	13


#define COORD_INTERPRET(MSB_BYTE, LSB_BYTE)  (MSB_BYTE << 8 | LSB_BYTE)

/******* Static Define Section************/
struct hdx_data
{
	__u16 	x, y, w, p, id;
	struct i2c_client *client;
	/* digitizer */
	struct input_dev *dig_dev;
	struct mutex lock;
	struct work_struct work;
	//struct early_suspend early_suspend;
	struct workqueue_struct *hdx_wq;
	struct regulator *supply;
    int revert_x;
    int revert_y;
    int swap_xy;
	int irq;
    int irq_level;
	int reset_gpio;
    int reset_level;
	int (*power)(int on);
	int intr_gpio;
	struct miscdevice firmware;
};
static struct workqueue_struct *Hdx_wq;
static int command_flag= 0;
static struct hdx_data *private_ts;
static u8 read_buf[DIG_BUF_SIZE]={0};
static u8 ver_buf[RD_COMMAND_BIT]={0};
int ver_flag = 0;
int id_flag = 0;
static int exchange_x_y_flag 	= 0;//1
static int revert_x_flag 		= 0;//1
static int revert_y_flag 		= 0;

/******* Data Define Section************/
//static void hdx_early_suspend(struct early_suspend *h);
//static void hdx_late_resume(struct early_suspend *h);
static struct i2c_driver hdx_driver;


/******* Function Define Section************/
	static int boot_flag = 0;
	/**
	* @brief       ReadFirmwareVersion
	* @param       None
	* @return      None
	* @details     int
	*/
//------------------------------以下是设备更新相关部分-----------------
	#define UPDATE_ENABLE		1

	#define DEV_VID  			0x0ed1
	#define DFU_PID  			0xabcd
	#define APP_PID  			0x7810
	
	#define EMR_PRINTK_ENABLE	0
	#define DFU_MODE  			0xabcd
	#define APP_MODE  			0x7800
	#define INIT_MODE		 	0x0
	#define DEV_CONNECTED 		1
	#define DEV_DISCONNECTED	0
	
	#define CMD_VPID_INDEX		0x02
	#define CMD_VER_INDEX		0x03
	#define CMD_TODFU_INDEX		0x04
	#define CMD_TOAPP_INDEX		0x05
	
	#define FLASH_PAGE			128
	struct hdx_dev
	{
		__u16 	vid;
		__u16 	pid;
		__u16 	ver;
		__u16 dfu_app_flag;
		int dev_connect;
	};
	static struct hdx_dev hdx_device;
	/**
	* @brief       check_update
	* @param       None
	* @return      None
	* @details     int
	*/
	static int get_hdx_devinfo(void)
	{//获取hdx设备信息 vid pid ver
		int ret;
		char cmd[6] = {0x06, 0xa1, 0x00, 0x70, 0x46, 0x02};//获取VID PID
		char info[6]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		//获取设备VID PID
		cmd[5] = CMD_VPID_INDEX;
		struct i2c_msg msgs[] = {
			{
				.addr = private_ts->client->addr,
				.flags = !I2C_M_RD,
				.len = sizeof(cmd),
				.buf = cmd,
			},
			{
				.addr = private_ts->client->addr,
				.flags = I2C_M_RD,
				.len = sizeof(info),
				.buf = info,
			},
		};
		ret = i2c_transfer(private_ts->client->adapter, msgs, ARRAY_SIZE(msgs));
		
		#if EMR_PRINTK_ENABLE
			printk(" Read Ack: %4x,%4x,%4x,%4x,%4x\n", info[1], info[2], info[3], info[4], info[5]);
		#endif
		
		if (ret < 0)
			return -1;
		if (ret != ARRAY_SIZE(msgs))
			return -EIO;
		hdx_device.vid =  COORD_INTERPRET(info[2], info[3]);
		hdx_device.pid =  COORD_INTERPRET(info[4], info[5]);
		#if EMR_PRINTK_ENABLE
			printk(" hdx_device info1--> vid:%4x,pid:%4x\n", hdx_device.vid, hdx_device.pid);
		#endif
		//获取Ver
		mdelay(20);
		cmd[5] = CMD_VER_INDEX;
		ret = i2c_transfer(private_ts->client->adapter, msgs, ARRAY_SIZE(msgs));
		#if EMR_PRINTK_ENABLE
			printk(" Read Ack: %4x,%4x,%4x,%4x,%4x\n", info[1], info[2], info[3], info[4], info[5]);
		#endif
		if (ret < 0)
			return -1;
		if (ret != ARRAY_SIZE(msgs))
			return -EIO;
		hdx_device.ver =  COORD_INTERPRET(info[4], info[5]);
		
		#if EMR_PRINTK_ENABLE
			printk(" hdx_device info2--> ver:%4x\n", hdx_device.ver);
		#endif
		//
		if(hdx_device.vid == DEV_VID)
		{//华鼎星设备
			#if EMR_PRINTK_ENABLE
				printk("\n THE Device is HDX EMR Device!!\n");
			#endif
			hdx_device.dev_connect = DEV_CONNECTED;
			if(hdx_device.pid == DFU_PID)
			{
				#if EMR_PRINTK_ENABLE
					printk("\n Device in DFU mode!!\n");
				#endif
				hdx_device.dfu_app_flag = DFU_MODE;
			}
			else if(hdx_device.pid == APP_PID)
			{
				#if EMR_PRINTK_ENABLE
					printk("\n Device is BOE10.3 Inch EMR!!\n");
				#endif
				hdx_device.dfu_app_flag = APP_MODE;
			}
			return 0;
		}
		else
		{
			#if EMR_PRINTK_ENABLE
				printk("\n Unfind HDX Device!!\n");
			#endif
			hdx_device.dev_connect = DEV_DISCONNECTED;
			return -1;
		}
	}
	/**
	* @brief       check_update
	* @param       None
	* @return      None
	* @details     int
	*/
	int check_update(void)
	{
		unsigned char *p_data = NULL;
		u16 current_ver, fw_ver;
		if( get_hdx_devinfo() )
		{//获取设备信息失败
			#if EMR_PRINTK_ENABLE
				printk("HDX Driver:get_hdx_devinfo error!!!!!\n");
			#endif
			return -1;
		}
		p_data = hdx_fw;
		fw_ver = hdx_fw_ver;
		current_ver = hdx_device.ver;
		
		if(hdx_device.dfu_app_flag == DFU_MODE)
		{//需要更新
			return 0;
		}
		else if(hdx_device.dfu_app_flag == APP_MODE)
		{
			if( hdx_fw_ver != current_ver )
			{//需要更新
				return 0;
			}
			else
			{
				return 1;
			}
		}
        return -1;
	}
	/**
	* @brief       check_update
	* @param       None
	* @return      None
	* @details     int
	*/
	int update_fw(void)
	{
		int i, j,ret = 0;
		int kk= 10;
		int PkgCnt = 0;
		u8 test[5] = {0};
		int DataLen = 0;
		u8 flash_data[134];
		__u16 write_adstart=0;
		__u16 write_adend=0;
		unsigned char *p_data = NULL;
		DataLen = sizeof(hdx_fw);
		PkgCnt = (DataLen)/FLASH_PAGE;
		p_data = hdx_fw;
		printk("HDXFWLen = %d\n", DataLen);
		
		char cmd[6] = {0x06, 0xa1, 0x00, 0x70, 0x46, 0x02};//获取VID PID
		char info[6]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		//获取设备VID PID
		struct i2c_msg msgs[] = {
			{
				.addr = private_ts->client->addr,
				.flags = !I2C_M_RD,
				.len = sizeof(cmd),
				.buf = cmd,
			},
			{
				.addr = private_ts->client->addr,
				.flags = I2C_M_RD,
				.len = sizeof(info),
				.buf = info,
			},
		};
		
		struct i2c_msg flash_msgs[] = {
			{
				.addr = private_ts->client->addr,
				.flags = !I2C_M_RD,
				.len = sizeof(flash_data),
				.buf = flash_data,
			},
		};
		
		if(hdx_device.dfu_app_flag != DFU_MODE)
		{//需要先切换到dfu
			printk("\n hdx device need switch to dfu mode!!\n");	
			for(i = 0;i<200 ;i++)
			{
				cmd[5] = CMD_TODFU_INDEX;
				ret = i2c_transfer(private_ts->client->adapter, msgs, ARRAY_SIZE(msgs));
				
				mdelay(50);
				
				cmd[5] = CMD_VPID_INDEX;
				ret = i2c_transfer(private_ts->client->adapter, msgs, ARRAY_SIZE(msgs));
				hdx_device.vid =  COORD_INTERPRET(info[2], info[3]);
				hdx_device.pid =  COORD_INTERPRET(info[4], info[5]);
				#if EMR_PRINTK_ENABLE
					printk(" hdx_device info1--> pid:%4x\n", hdx_device.pid);
				#endif	
				if( hdx_device.pid == DFU_PID)
				{
					hdx_device.dfu_app_flag = DFU_MODE;
					break;
				}
			}
		}
		write_adstart = 0x0000;
		write_adend = 0x0000;
		printk("================hdx emr start update=================\n");
		for(i = 0; i < PkgCnt+1; i++)
		{
			printk(" [Driver DFU Info] WriteFlash Page: %4x\n",i);
			flash_data[0] = 134;
			flash_data[1] = 0xAF;
			write_adstart = 0 + 128*i;
			write_adend = write_adstart + 127;
			flash_data[2] = (write_adstart&0xff00)>>8;
           flash_data[3] = (write_adstart&0x00ff);
			flash_data[4] = (write_adend&0xff00)>>8;
			flash_data[5] = (write_adend&0x00ff);
			for(j = 0;j < 128 ;j++)
			{
				flash_data[6+j] = p_data[128*i + j];
			}
			#if EMR_PRINTK_ENABLE
				ret = i2c_transfer(private_ts->client->adapter, flash_msgs, ARRAY_SIZE(flash_msgs));
			#endif		
			if(ret < 0)
			{
				#if EMR_PRINTK_ENABLE
					printk("[Driver ]i2c_smbus_write_i2c_block_data fail, Update Failed!!!!!\n");
				#endif
				return -1;
			}
			mdelay(50);
		}
		mdelay(400);
		#if EMR_PRINTK_ENABLE
			printk("================hdx emr end update=================\n");
		#endif
		for(i = 0;i<200;i++)
		{
			cmd[5] = CMD_TOAPP_INDEX;
			ret = i2c_transfer(private_ts->client->adapter, msgs, ARRAY_SIZE(msgs));
			mdelay(50);
			cmd[5] = CMD_VPID_INDEX;
			ret = i2c_transfer(private_ts->client->adapter, msgs, ARRAY_SIZE(msgs));
			hdx_device.vid =  COORD_INTERPRET(info[2], info[3]);
			hdx_device.pid =  COORD_INTERPRET(info[4], info[5]);
			#if EMR_PRINTK_ENABLE
				printk(" hdx_device info1--> pid:%4x\n", hdx_device.pid);
			#endif
			if(  (hdx_device.pid == APP_PID) )
			{
				hdx_device.dfu_app_flag = APP_MODE;
				break;
			}
			mdelay(30);
		}
		if( get_hdx_devinfo() )
		{//获取设备信息失败
			#if EMR_PRINTK_ENABLE
				printk("HDX Driver:get_hdx_devinfo error!!!!!\n");
			#endif
			return -1;
		}
		if(hdx_device.dfu_app_flag == APP_MODE)
		{
			#if EMR_PRINTK_ENABLE
				printk("[Driver]Turn to App sucess!!!!\n");	
			#endif
		}
		else
		{
			#if EMR_PRINTK_ENABLE
				printk("[Driver]Turn to App Fail!!!!!\n");
			#endif
			return -1;
		}
		return 0;
	}
//------------------------------以上是设备更新相关部分-----------------
/**
 * @brief       hdx8801_reset
 * @param       None
 * @return      None
 * @details     void
 */
//void hdx8801_reset(void)
//{
//    gpio_set_value(HDX_RST, GPIO_LOW);
//    mdelay(10);
//    gpio_set_value(HDX_RST, GPIO_HIGH);
//}
/**
 * @brief       mg_irq
 * @param       None
 * @return      None
 * @details     irqreturn_t
 */
static irqreturn_t hdx_irq(int irq, void *_hdx)
{
	struct hdx_data *hdx = _hdx;

	queue_work(Hdx_wq, &hdx->work);

	return IRQ_HANDLED;
}

/**
 * @brief       mg_report
 * @param       None
 * @return      None
 * @details     inline
 */
static inline void hdx_report(struct hdx_data *hdx)
{	
	__u16 x,y;
	x = hdx->x;
	y = hdx->y;
	if(1 == hdx->swap_xy){
		x = hdx->y;
		y = hdx->x;
		if(1 == hdx->revert_x){
			x = REPORT_MAX_Y-hdx->y;
		}
		if(1 == hdx->revert_y){
			y = REPORT_MAX_X-hdx->x;
		}
	}else{
		if(1 == hdx->revert_x){
			x = REPORT_MAX_X-hdx->x;
		}
		if(1 == hdx->revert_y){
			y = REPORT_MAX_Y-hdx->y;
		}
	}
	

	input_report_key(hdx->dig_dev, BTN_TOOL_PEN, ABS_MT_TOOL_TYPE);
	input_report_abs(hdx->dig_dev,ABS_MT_POSITION_X,(x));
	input_report_abs(hdx->dig_dev,ABS_MT_POSITION_Y,(y));
	input_report_abs(hdx->dig_dev,ABS_MT_PRESSURE,hdx->p);
	input_sync(hdx->dig_dev);
}

/**
 * @brief       mg_irq
 * @param       None
 * @return      None
 * @details     irqreturn_t
 */
static void hdx_msg_process(u8 *data_in,struct hdx_data *hdx)
{
	u8 keyCode = read_buf[2];
	__u16 pressure = (COORD_INTERPRET(read_buf[8], read_buf[7]));
	bool hover_enter = 0;

	//last pressure is saved in hdx->p,and last key code is saved in hdx->w
	if(pressure==0)
	{
		if(0 != hdx->p)
			input_event(hdx->dig_dev,EV_KEY,BTN_TOUCH,0);
		switch(keyCode)
		{
			case HDX_OUT_RANG:
				if(HDX_IN_RANG == hdx->w){
					input_event(hdx->dig_dev,EV_KEY,BTN_TOOL_PEN,0);
					//printk("input report EV_KEY,code is pen hover out rang\n");
				}
				break;
			case HDX_IN_RANG:
				if(HDX_OUT_RANG == hdx->w){
					input_event(hdx->dig_dev,EV_KEY,BTN_TOOL_PEN_HOVER,1);
					hover_enter = 1;
					//printk("input report EV_KEY,code is pen hover\n");
				}
				else if(HDX_CUSTOM_BTN == hdx->w||HDX_RIGHT_BTN == hdx->w)
					input_event(hdx->dig_dev,EV_KEY,hdx->w == HDX_CUSTOM_BTN?BTN_STYLUS:BTN_STYLUS2,0);
				break;
			case HDX_CUSTOM_BTN:
				if(HDX_IN_RANG == hdx->w)
					input_event(hdx->dig_dev,EV_KEY,BTN_STYLUS,1);
				break;
			case HDX_RIGHT_BTN:
				if(HDX_IN_RANG == hdx->w)
					input_event(hdx->dig_dev,EV_KEY,BTN_STYLUS2,1);
				break;
			default:
				break;
		}
	}
	else if(0 == hdx->p)
	{
		input_event(hdx->dig_dev,EV_KEY,BTN_TOUCH,1);
	}
	input_sync(hdx->dig_dev);
	hdx->x =  COORD_INTERPRET(read_buf[4], read_buf[3]);
	hdx->y =  COORD_INTERPRET(read_buf[6], read_buf[5]);
	hdx->w = read_buf[2];
	hdx->p = pressure;
	hdx_report(hdx);
	if(hover_enter){
		input_event(hdx->dig_dev,EV_KEY,BTN_TOOL_PEN,1);
		input_event(hdx->dig_dev,EV_KEY,BTN_TOOL_PEN_HOVER,0);
	}
}


/**
 * @brief       mg_irq
 * @param       None
 * @return      None
 * @details     irqreturn_t
 */
static void hdx_i2c_work(struct work_struct *work)
{
	int i = 0;
	struct hdx_data *hdx = container_of(work, struct hdx_data, work);
	u_int8_t ret = 0;

	memset( read_buf, 0, sizeof(read_buf) );
	/*	Check if I/O control command 	*/
	if(command_flag == 1)
	{
		ret = i2c_smbus_read_i2c_block_data(hdx->client, 0x0, RD_COMMAND_BIT, read_buf);
		if(ret < 0)
		{
			for(i = 1; i< 11; i++)
			{
				if(i == 11)
				{
					printk("Read 10 Times error!!!!!\n");
					command_flag = 0;
					return;
				}
				printk("Read %4d Times Error!!!!!\n", i);
				ret = i2c_smbus_read_i2c_block_data(hdx->client, 0x0, RD_COMMAND_BIT, read_buf);
				if(ret >= 0)
				{
					printk("Read OK!!!!!\n");
					break;
				}
			}
		}

		if(ver_flag == 1 || id_flag == 1)
		{
			for(i = 0; i < RD_COMMAND_BIT; i ++)
				ver_buf[i] = read_buf[i];
			ver_flag = 0;
			id_flag = 0;
		}

		command_flag = 0;
		return;
	}

	ret = i2c_smbus_read_i2c_block_data(hdx->client, 0x0, DIG_BUF_SIZE, read_buf);
	if(ret < 0)
	{
		printk("Read error!!!!!\n");
		return;
	}
#if (HDX_DEBUG)
	printk("\n EMR Data: 0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  \n",read_buf[1],read_buf[2],read_buf[3],read_buf[4],read_buf[5],read_buf[6],read_buf[7],read_buf[8]);
//	printk("\n");
//	printk("READ BUF: ");
//	for(i = 0; i < read_buf[0]; i++)
//	{
//		printk("%02x", read_buf[i]);
//	}
//	printk("\n");
#endif

	if (read_buf[HDX_DIG_MODE] == 1) 
	{
		hdx_msg_process(read_buf, hdx);
	}	
}

/**
 * @brief       mg_irq
 * @param       None
 * @return      None
 * @details     irqreturn_t
 */
static int hdx_probe(struct i2c_client *client, const struct i2c_device_id *ids)	
{
	printk("\n=========%s=======\n", __func__);
	struct hdx_data *hdx;
	struct input_dev *input_dig;
	struct device_node *hdx_np;
	enum of_gpio_flags flags;
	int err = 0;
	hdx = kzalloc(sizeof(struct hdx_data), GFP_KERNEL);
	if (!hdx)
		return -ENOMEM;

	hdx->client = client;
	dev_info(&hdx->client->dev, "device probing\n");
	i2c_set_clientdata(client, hdx);
	hdx_np = client->dev.of_node;
	if (!hdx_np) {
		dev_err(&client->dev, "get device node error!!!\n");
		return -ENODEV;
	}
    of_property_read_u32(hdx_np, "revert_x", &hdx->revert_x);
    of_property_read_u32(hdx_np, "revert_y", &hdx->revert_y);
    of_property_read_u32(hdx_np, "xy_exchange", &hdx->swap_xy);
    hdx->supply = devm_regulator_get(&client->dev, "pwr");
    if (hdx->supply) {
        printk("hdx power supply = %dmv\n", regulator_get_voltage(hdx->supply));
        err = regulator_enable(hdx->supply);
        if (err < 0) {
            dev_err(&client->dev, "failed to enable hdx power supply!!!\n");
            return -ENODEV;
        }
    }

    hdx->reset_gpio = of_get_named_gpio_flags(hdx_np, "gpio_rst", 0, &flags);
    if (!gpio_is_valid(hdx->reset_gpio)) {
        dev_err(&client->dev, "no gpio_rst pin available\n");
        return -ENODEV;
    }
    hdx->reset_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
    printk("hdx reset level %d\n", hdx->reset_level);
    err = devm_gpio_request_one(&client->dev, hdx->reset_gpio, !hdx->reset_level ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW, "hdx-rst");
    if (err < 0) {
        dev_err(&client->dev, "request gpio_rst pin failed!!!\n");
        return -ENODEV;
    }

    hdx->irq = of_get_named_gpio_flags(hdx_np, "gpio_intr", 0, &flags);
    if (!gpio_is_valid(hdx->irq)) {
        dev_err(&client->dev, "no gpio_intr pin available\n");
        goto err_free_reset_gpio;
    }
    hdx->irq_level = (flags & OF_GPIO_ACTIVE_LOW) ?  0 : 1;
    err = devm_gpio_request_one(&client->dev, hdx->irq, GPIOF_IN, "hdx_intr");
    if (err < 0) {
        goto err_free_irq_gpio;
    }

	mutex_init(&hdx->lock);

	/* allocate input device for digitizer */
	input_dig = input_allocate_device();
	input_dig->name = "hdx-digitizer";
	input_dig->id.bustype = BUS_I2C;
	hdx->dig_dev = input_dig;

	__set_bit(INPUT_PROP_DIRECT, input_dig->propbit);
	__set_bit(EV_ABS, input_dig->evbit);
	__set_bit(EV_KEY, input_dig->evbit);

	__set_bit(BTN_TOUCH, input_dig->keybit);
	__set_bit(BTN_TOOL_PEN, input_dig->keybit);
	__set_bit(BTN_TOOL_PEN_HOVER, input_dig->keybit);
	__set_bit(BTN_STYLUS, input_dig->keybit);
	__set_bit(BTN_STYLUS2, input_dig->keybit);
	if(hdx->swap_xy){
		input_set_abs_params(input_dig, ABS_MT_POSITION_X, 0, REPORT_MAX_X, 0, 0);
		input_set_abs_params(input_dig, ABS_MT_POSITION_Y, 0, REPORT_MAX_Y, 0, 0);
	}else{
		input_set_abs_params(input_dig, ABS_MT_POSITION_X, 0, REPORT_MAX_Y, 0, 0);
		input_set_abs_params(input_dig, ABS_MT_POSITION_Y, 0, REPORT_MAX_X, 0, 0);
	}
	input_set_abs_params(input_dig, ABS_MT_TOUCH_MAJOR, 0, DIG_MAX_P, 0, 0);
	input_set_abs_params(input_dig, ABS_MT_PRESSURE, 0, DIG_MAX_P, 0, 0);


	err = input_register_device(input_dig);
	if (err)
		goto exit_input;

	INIT_WORK(&hdx->work, hdx_i2c_work);

	private_ts = hdx;
	//disable_irq(client->irq);

	//hdx->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	//hdx->early_suspend.suspend = hdx_early_suspend;
	//hdx->early_suspend.resume = hdx_late_resume;
	//register_early_suspend(&hdx->early_suspend);
	/*
	err = ReadFirmwareVersion();
	if(err==0)
	{
	  printk("[Driver ]ReadFirmwareVersion error!!!!!\n");
	  goto exit_input;
	}
	*/
	if( get_hdx_devinfo() )
	{
		#if EMR_PRINTK_ENABLE
			printk("HDX Driver:get_hdx_devinfo error!!!!!\n");
		#endif
		goto exit_input;
	}
	client->irq = gpio_to_irq(hdx->irq);

	//client->irq = hdx->irq;
	//hdx->irq = client->irq;

	err = request_irq(client->irq, hdx_irq,IRQF_TRIGGER_FALLING, HDX_DRIVER_NAME, hdx);

	#if UPDATE_ENABLE
	  if(check_update())
			printk( "-------HDX EMR Device‘s FW is latest version!!!!\n" );
	  else
	  {
		  #if EMR_PRINTK_ENABLE
			printk( "-------HDX EMR Device‘s FW need to update!\n");
		  #endif
	      mdelay(50);
	      update_fw();
	      mdelay(50);
	  }
	#endif
	
	printk( "====hdx8801 after reset!!!!\n" );
    //enable_irq(client->irq);
	return 0;

exit_input:
	input_unregister_device(hdx->dig_dev);
err_free_irq_gpio:
	devm_gpio_free(&client->dev, hdx->irq);
err_free_reset_gpio:
	devm_gpio_free(&client->dev, hdx->reset_gpio);
	kfree(hdx);


	return -ENODEV;


}

/**
 * @brief       hdx_remove
 * @param       None
 * @return      None
 * @details     __devexit
 */
static int hdx_remove(struct i2c_client *client)
{
	struct hdx_data *hdx = i2c_get_clientdata(client);

	free_irq(client->irq, hdx);
	input_unregister_device(hdx->dig_dev);
	kfree(hdx);
	return 0;
}

/**
 * @brief       mg_resume
 * @param       None
 * @return      None
 * @details     static
 */
static int hdx_suspend(struct i2c_client *client)
{

    struct hdx_data *ts = i2c_get_clientdata(client);
    int ret;
    if (client->irq)
        disable_irq(client->irq);

    ret = cancel_work_sync(&ts->work);
    if (ret && client->irq) // if work was pending disable-count is now 2 
        enable_irq(client->irq);
    return 0;
}

/**
 * @brief       mg_resume
 * @param       None
 * @return      None
 * @details     static
 */
static int hdx_resume(struct i2c_client *client)
{ 
    struct hdx_data *ts = i2c_get_clientdata(client);
    if (client->irq)
        enable_irq(client->irq);
    gpio_set_value(ts->reset_gpio, ts->reset_level);
    mdelay(10);
    gpio_set_value(ts->reset_gpio, !ts->reset_level);
    printk("hdx8801_resume success\n");    
    return 0;

}

/**
 * @brief       hdx_early_suspend
 * @param       None
 * @return      None
 * @details     static
 */
static int hdx_early_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hdx_data *ts = i2c_get_clientdata(client);
	hdx_suspend(ts->client);
	return 0;
}

/**
 * @brief       mg_resume
 * @param       None
 * @return      None
 * @details     static
 */
static int hdx_late_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hdx_data *ts = i2c_get_clientdata(client);

	hdx_resume(ts->client);
	return 0;
}

static const struct dev_pm_ops hdx_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(hdx_early_suspend, hdx_late_resume)
};

/***/
static struct i2c_device_id hdx_id_table[] =
{
	/* the slave address is passed by i2c_boardinfo */
	{HDX_DRIVER_NAME},
	{/* end of list */}
};

/****/
static struct i2c_driver hdx_driver = {
	.driver = 
	{
		.name	 = HDX_DRIVER_NAME,
		.pm		 = &hdx_pm,
	},
	.id_table 	= hdx_id_table,
	.probe 		= hdx_probe,
	.remove 	= hdx_remove,
};

/**
 * @brief       hdx_init
 * @param       None
 * @return      None
 * @details     __init
 */
static int __init hdx_init(void)
{
	Hdx_wq = create_singlethread_workqueue("hdx_wq");	
	if (!Hdx_wq) 
	{
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
	}
	return i2c_add_driver(&hdx_driver);
}

/**
 * @brief       hdx_exit
 * @param       None
 * @return      None
 * @details     static
 */
static void hdx_exit(void)
{
	i2c_del_driver(&hdx_driver);
}
//module_init(hdx_init); //__define_initcall ( " 6s " , fn , 6 s )
fs_initcall(hdx_init);//must early than pmu sy7636a||tps65185  __define_initcall ( " 5 " , fn , 5 )
module_exit(hdx_exit);

MODULE_AUTHOR("<jinshui.li@sz-hdx.cn>");
MODULE_DESCRIPTION("HDX8801 Digitizer Driver"); 
