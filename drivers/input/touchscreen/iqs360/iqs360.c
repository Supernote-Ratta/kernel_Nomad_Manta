/* -------------------------------------------------------------------------
 * Copyright (C) 2014-2015, Intel Corporation
 *
 * Derived from:
 *  gslX68X.c
 *  Copyright (C) 2010-2015, Shanghai Sileadinc Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * -------------------------------------------------------------------------
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
//#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
//#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
//#include <linux/of_irq.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/completion.h>



#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "iqs360.h"
#include "iqs360_handler.h"
#include "iqs360_Init.h"

#include <asm/unaligned.h>

#define IQS360_TS_NAME		"iqs360_ts"

#define IQS360_REG_RESET	0xE0
#define IQS360_REG_DATA		0x80
#define IQS360_REG_TOUCH_NR	0x80
#define IQS360_REG_POWER	0xBC
#define IQS360_REG_CLOCK	0xE4
#define IQS360_REG_STATUS	0xB0
#define IQS360_REG_ID		0xFC
#define IQS360_REG_MEM_CHECK	0xB0

#define IQS360_STATUS_OK	0x5A5A5A5A
#define IQS360_TS_DATA_LEN	44
#define IQS360_CLOCK		0x04

#define IQS360_CMD_RESET	0x88
#define IQS360_CMD_START	0x00

#define IQS360_POINT_DATA_LEN	0x04
#define IQS360_POINT_Y_OFF      0x00
#define IQS360_POINT_Y_MSB_OFF	0x01
#define IQS360_POINT_X_OFF	0x02
#define IQS360_POINT_X_MSB_OFF	0x03
#define IQS360_EXTRA_DATA_MASK	0xF0

#define IQS360_CMD_SLEEP_MIN	10000
#define IQS360_CMD_SLEEP_MAX	20000
#define IQS360_POWER_SLEEP	20
#define IQS360_STARTUP_SLEEP	30

#define IQS360_MAX_FINGERS	10
// Private defines
#define PWM_ON      0x6F
#define PWM_OFF     0x60

#define KEY_DIR_UP   0x01
#define KEY_DIR_DOWN 0x02
#define KEY_START_X 600
#define KEY_END_X 200
#define KEY_STEP_OFFSET 20
#define KEY_OFFSET 400
#define IQS360_SLIDE_DOWN		KEY_F23
#define IQS360_SLIDE_UP		KEY_REFRESH


IQS360_t IQS360 = {0};
//X_t X_prev = {0};
//Y_t Y_prev = {0};

enum iqs360_ts_power {
	IQS360_POWER_ON  = 1,
	IQS360_POWER_OFF = 0
};

struct iqs360_ts_data {
	struct i2c_client *client;
	unsigned int irq_gpio;
    int irq;
    unsigned int irq_flags;
	struct input_dev *input;
	//struct regulator_bulk_data regulators[2];
	struct touchscreen_properties prop;
	u32 max_fingers;
	u16 chip_id;
	struct input_mt_pos pos[IQS360_MAX_FINGERS];
	int slots[IQS360_MAX_FINGERS];
	int id[IQS360_MAX_FINGERS];
};

struct iqs360_fw_data {
	u32 offset;
	u32 val;
};

struct iqs360_data {
	/* start x and y position */
	int x;
	int y;
	unsigned long jiffs;
	bool done;
	int offset;
	int dir;
};
struct iqs360_data pre_key ;
struct iqs360_data now_key ;

static int iqs360_ts_request_input_dev(struct iqs360_ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	data->input = devm_input_allocate_device(dev);
	if (!data->input) {
		dev_err(dev,
			"Failed to allocate input device\n");
		return -ENOMEM;
	}

	//input_set_abs_params(data->input, ABS_MT_POSITION_X, 0, 4095, 0, 0);
	//input_set_abs_params(data->input, ABS_MT_POSITION_Y, 0, 4095, 0, 0);
	//touchscreen_parse_properties(data->input, true, &data->prop);

	//input_mt_init_slots(data->input, data->max_fingers,
	//		    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED |
	//		    INPUT_MT_TRACK);

	//if (device_property_read_bool(dev, "iqs360,home-button"))
	//	input_set_capability(data->input, EV_KEY, KEY_LEFTMETA);

	data->input->name = IQS360_TS_NAME;
	data->input->phys = "input/ts";
	data->input->id.bustype = BUS_I2C;
	__set_bit(EV_KEY, data->input->evbit);
	__set_bit(IQS360_SLIDE_UP, data->input->keybit);
	__set_bit(IQS360_SLIDE_DOWN, data->input->keybit);
	//__set_bit(RATTA_SLIDE_DROP, data->input->keybit);

	error = input_register_device(data->input);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static void iqs360_ts_set_power(struct i2c_client *client,
				enum iqs360_ts_power state)
{
	//struct iqs360_ts_data *data = i2c_get_clientdata(client);

	//if (data->gpio_power) {
	//	gpiod_set_value_cansleep(data->gpio_power, state);
	//	msleep(IQS360_POWER_SLEEP);
	//}
}
void IQS360_event_handler(struct i2c_client *client)
{
	/**
	 * It is possible to build a buffer here or threshold, so that a write
	 * only occurs after a certain delta of movement was made on the trackpad
	 */
	struct iqs360_ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input = data->input;
	int i,j=0;
	int touch_point = 0;
	touch_point = IQS360.Touch_Channels.Touch_Channels+((*(&IQS360.Touch_Channels.Touch_Channels+1))<<8);
	for(i=0;i<12;i++){
		if((touch_point & (1<<i))!=0){
			j++;
		}
	}
	
	printk("IQS360_event_handler pre_key.x:%d X_prev.Y:%d IQS360.X.X=%d IQS360.Y.Y=%d Touch_Channels:0x%x \n",
	 			pre_key.x,pre_key.y,IQS360.X.X,IQS360.Y.Y,touch_point);
	printk("IQS360_event_handler now_key.done:%d now_key.offset:%d now_key.dir=%d jiffies=%d pre_key.jiffs:%d \n",
	 			now_key.done,now_key.offset,now_key.dir,jiffies,pre_key.jiffs);
	if(j>3){
		now_key.done = false;
		now_key.offset = 0;
		now_key.dir = 0;
		printk("IQS360_event_handler more than one key down:%d\n",(j-2));
		return;
	}
	if((jiffies_to_msecs(jiffies - pre_key.jiffs)>150)){
		now_key.done = true;
		now_key.offset = 0;
		now_key.dir = 0;
		return;
	}
	if(now_key.done){
		if(now_key.dir == 0){
			if((pre_key.x > KEY_START_X)&&(pre_key.x > now_key.x)){
				now_key.dir = KEY_DIR_DOWN;
				now_key.offset = pre_key.x - now_key.x;
			}else if((pre_key.x < KEY_END_X)&&(now_key.x > pre_key.x)){
				now_key.dir = KEY_DIR_UP;
				now_key.offset = now_key.x - pre_key.x;
			}
		}else if(now_key.dir == KEY_DIR_UP){
			if(now_key.x >= pre_key.x){
				now_key.offset += now_key.x - pre_key.x;
			}else{
				now_key.done = false;
				now_key.offset = 0;
				now_key.dir = 0;
			}
		}else if(now_key.dir == KEY_DIR_DOWN){
			if(pre_key.x >= now_key.x){
				now_key.offset += pre_key.x - now_key.x;
			}else{
				now_key.done = false;
				now_key.offset = 0;
				now_key.dir = 0;
			}
		}
		if(now_key.offset > KEY_OFFSET){
			if(now_key.dir == KEY_DIR_UP){
				input_report_key(input,
					 		IQS360_SLIDE_UP, 1);
				input_report_key(input,
							 IQS360_SLIDE_UP, 0);
				input_sync(input);
				printk("IQS360_event_handler key up:\n");
				now_key.done = false;
			}else if(now_key.dir == KEY_DIR_DOWN){
				input_report_key(input,
					 		IQS360_SLIDE_DOWN, 1);
				input_report_key(input,
							 IQS360_SLIDE_DOWN, 0);
				input_sync(input);
				now_key.done = false;
				printk("IQS360_event_handler key down:\n");
			}
		}
	}

	/*	Set the Prev IQS360 equal to this one	*/
	//pre_key.x = IQS360.X.X;
	//pre_key.y = IQS360.Y.Y;
}

static void iqs360_ts_read_data(struct i2c_client *client)
{
	struct iqs360_ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input = data->input;
	struct device *dev = &client->dev;
	u8 *bufp, buf[IQS360_TS_DATA_LEN];
	int touch_nr, softbutton, error, i;
	bool softbutton_pressed = false;
	int rev_y = 0;

	error = i2c_smbus_read_i2c_block_data(client, COUNTS,
					      26, &IQS360.Counts_CH0.Counts_Low);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}
	error = i2c_smbus_read_i2c_block_data(client, LTA,
					      26, &IQS360.LTA_CH0.LTA_Low);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}
	error = i2c_smbus_read_i2c_block_data(client, SYSTEM_FLAGS,
					      2, &IQS360.SystemFlags.SystemFlags);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}
	/*  Read XY-Data */
	error = i2c_smbus_read_i2c_block_data(client, XY_DATA,
						  4, &IQS360.X.X_Low);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}
	error = i2c_smbus_read_i2c_block_data(client, STATUS,
						  4, &IQS360.Touch_Channels.Touch_Channels_0);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}
	
	if(IQS360.X.X>600){
		rev_y = 500-IQS360.Y.Y;
	}else if(IQS360.X.X>400){
		rev_y = 500+IQS360.Y.Y;
	}else if(IQS360.X.X>150){
		rev_y = 1500-IQS360.Y.Y;
	}else if(IQS360.X.X>=0){
		rev_y = 1500+IQS360.Y.Y;
	}
	IQS360.Y.Y = rev_y;
	now_key.x = IQS360.X.X;
	now_key.y = IQS360.Y.Y;
	IQS360_event_handler(client);
	//if (pre_key.x != IQS360.X.X || pre_key.y != IQS360.Y.Y & !IQS360.Snap_Channels.Snap_Channels){
	//	input_report_key(input, BTN_TOUCH, 1);
	//} else{
	//	input_report_key(input, BTN_TOUCH, 0);
	//}
	/*	Set the Prev IQS360 equal to this one	*/
	pre_key.x = IQS360.X.X;
	pre_key.y = IQS360.Y.Y;
	pre_key.jiffs = jiffies;

	//input_mt_assign_slots(input, data->slots, data->pos, touch_nr, 0);

	//for (i = 0; i < touch_nr; i++) {
		//input_mt_slot(input, 1);
		//input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		//input_report_abs(input, ABS_MT_POSITION_X, IQS360.X.X);
		//input_report_abs(input, ABS_MT_POSITION_Y, IQS360.Y.Y);

		//dev_dbg(dev, "x=%d y=%d \n", IQS360.X.X,IQS360.Y.Y);
	//}

	//input_mt_sync_frame(input);
	//input_report_key(input, KEY_LEFTMETA, softbutton_pressed);
	//input_sync(input);
}

static int iqs360_ts_init(struct i2c_client *client)
{
	//struct iqs360_ts_data *data = i2c_get_clientdata(client);
	int error;

	error = i2c_smbus_write_i2c_block_data(client, MULTIPLIERS, 13,
					&IQS360.Multi_CH0.Multi);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

#if 1
	error = i2c_smbus_write_i2c_block_data(client, SETTINGS, 6,
					&IQS360.Settings0.Settings0);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

#endif

	error = i2c_smbus_write_i2c_block_data(client, THRESHOLDS, 13,
					&IQS360.Threshold_CH0);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	error = i2c_smbus_write_i2c_block_data(client, TIMINGS, 3,
					&IQS360.Filter_Halt);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	error = i2c_smbus_write_i2c_block_data(client, TARGETS, 2,
					&IQS360.ATI_Target_CH0);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	
	error = i2c_smbus_write_i2c_block_data(client, EVENTS, 1,
					&IQS360.EventsMask);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	error = i2c_smbus_write_i2c_block_data(client, PWM_LIM_SPEED, 2,
					&IQS360.PWM_Limit);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	
	error = i2c_smbus_write_i2c_block_data(client, ACTIVE_CHANNELS, 4,
					&IQS360.Active_Channels.Touch_Channels_0);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	do 
	{
		error = i2c_smbus_read_i2c_block_data(client, SYSTEM_FLAGS, 2, &IQS360.SystemFlags.SystemFlags);
		//if (error < 0)
		//	return error;
		msleep(1);
		printk("iqs360 read system_flags=0x%x \n",IQS360.SystemFlags.SystemFlags);
	}
	while(IQS360.SystemFlags.ATI_Busy);
	if (IQS360.Events.ATI_Error)
	{
		//IQS360_setup();
		
		printk("iqs360 need resetup\n");
	}

return 0;

i2c_write_err:
	dev_err(&client->dev, "Registers clear error %d\n", error);
	return error;
}

static int iqs360_ts_reset(struct i2c_client *client)
{
#if 0
	int error;
	struct iqs360_ts_data *data = i2c_get_clientdata(client);
	//gpio_direction_output(data->irq_gpio, 0);
	//udelay(5000);
	//gpio_direction_output(data->irq_gpio, 1);
	//msleep(100);

	error = i2c_smbus_write_byte_data(client, IQS360_REG_RESET,
					  IQS360_CMD_RESET);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, IQS360_REG_CLOCK,
					  IQS360_CLOCK);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, IQS360_REG_POWER,
					  IQS360_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(IQS360_CMD_SLEEP_MIN, IQS360_CMD_SLEEP_MAX);

	return 0;

i2c_write_err:
	dev_err(&client->dev, "Chip reset error %d\n", error);
	return error;
	#endif
	return 0;
}

static int iqs360_ts_startup(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, IQS360_REG_RESET, 0x00);
	if (error) {
		dev_err(&client->dev, "Startup error %d\n", error);
		return error;
	}

	msleep(IQS360_STARTUP_SLEEP);

	return 0;
}

static u32 iqs360_ts_get_status(struct i2c_client *client)
{
	int error;
	__le32 status;

	error = i2c_smbus_read_i2c_block_data(client, IQS360_REG_STATUS,
					      sizeof(status), (u8 *)&status);
	if (error < 0) {
		dev_err(&client->dev, "Status read error %d\n", error);
		return error;
	}

	return le32_to_cpu(status);
}

static int iqs360_ts_get_id(struct i2c_client *client)
{
	struct iqs360_ts_data *data = i2c_get_clientdata(client);
	u16 chip_id;
	int error;

	error = i2c_smbus_read_i2c_block_data(client, DEVICE_INFORMATION,
					      2, (u8 *)&chip_id);
	if (error < 0)
		return error;

	data->chip_id = chip_id;
	printk("iqs360 chip ID: 0x%4x \n", data->chip_id);

	return 0;
}

static int iqs360_ts_setup(struct i2c_client *client)
{
	int error;
	u32 status;

	iqs360_ts_set_power(client, IQS360_POWER_OFF);
	iqs360_ts_set_power(client, IQS360_POWER_ON);

    /*  Multipliers (BASE)  */
    IQS360.Multi_CH0.Multi = MULTIPLIERS_CH0;
    IQS360.Multi_CH1.Multi = MULTIPLIERS_CH1;
    IQS360.Multi_CH2.Multi = MULTIPLIERS_CH2;
    IQS360.Multi_CH3.Multi = MULTIPLIERS_CH3;
    IQS360.Multi_CH4.Multi = MULTIPLIERS_CH4;
    IQS360.Multi_CH5.Multi = MULTIPLIERS_CH5;
    IQS360.Multi_CH6.Multi = MULTIPLIERS_CH6;
    IQS360.Multi_CH7.Multi = MULTIPLIERS_CH7;
    IQS360.Multi_CH8.Multi = MULTIPLIERS_CH8;
    IQS360.Multi_CH9.Multi = MULTIPLIERS_CH9;
    IQS360.Multi_CH10.Multi = MULTIPLIERS_CH10;
    IQS360.Multi_CH11.Multi = MULTIPLIERS_CH11;
    IQS360.Multi_CH12.Multi = MULTIPLIERS_CH12;

    /*  Please Note - No need to write Compensation */

    /*  Settings    */
    IQS360.Settings0.Settings0 = PROXSETTINGS0_VAL|REDO_ATI|ATI_OFF;
    IQS360.Settings1.Settings1 = PROXSETTINGS1_VAL|ACK_RESET|TURBO_MODE;
    IQS360.Settings2.Settings2 = PROXSETTINGS2_VAL|EVENT_MODE;
    IQS360.Settings3.Settings3 = PROXSETTINGS3_VAL;
    IQS360.Settings4.Settings4 = PROXSETTINGS4_VAL;
    IQS360.Settings5 = PROXSETTINGS5_VAL;

    /*  Thresholds  */
    IQS360.Threshold_CH0 = PROX_THRESHOLD;
    IQS360.Threshold_CH1 = TOUCH_THRESHOLD_CH1;
    IQS360.Threshold_CH2 = TOUCH_THRESHOLD_CH2;
    IQS360.Threshold_CH3 = TOUCH_THRESHOLD_CH3;
    IQS360.Threshold_CH4 = TOUCH_THRESHOLD_CH4;
    IQS360.Threshold_CH5 = TOUCH_THRESHOLD_CH5;
    IQS360.Threshold_CH6 = TOUCH_THRESHOLD_CH6;
    IQS360.Threshold_CH7 = TOUCH_THRESHOLD_CH7;
    IQS360.Threshold_CH8 = TOUCH_THRESHOLD_CH8;
    IQS360.Threshold_CH9 = TOUCH_THRESHOLD_CH9;
    IQS360.Threshold_CH10 = TOUCH_THRESHOLD_CH10;
    IQS360.Threshold_CH11 = TOUCH_THRESHOLD_CH11;
    IQS360.Threshold_CH12 = TOUCH_THRESHOLD_CH12;

    /*  Timings */
    IQS360.Filter_Halt = FILTER_HALT;
    IQS360.Power_Mode = POWER_MODE;
    IQS360.Timeout_Period = TIMEOUT_PERIOD;

    /*  ATI Target  */
    IQS360.ATI_Target_CH0 = ATI_TARGET_CH0;
    IQS360.ATI_Target_CH1_12 = ATI_TARGET_CH0_9;
	IQS360.EventsMask = 0xFF;

    /*  PWM */
    IQS360.PWM0.PWM = PWM_0;
    IQS360.PWM1.PWM = PWM_1;
    IQS360.PWM2.PWM = PWM_2;
    IQS360.PWM3.PWM = PWM_3;
    IQS360.PWM4.PWM = PWM_4;
    IQS360.PWM5.PWM = PWM_5;
    IQS360.PWM6.PWM = PWM_6;
    IQS360.PWM7.PWM = PWM_7;

    /*  PWM Limits and Speed    */
    IQS360.PWM_Limit = PWM_LIMITS;
    IQS360.PWM_Speed = PWM_SPEED;

    /*  Active Channels */
    IQS360.Active_Channels.Touch_Channels_0 = ACTIVE_CH0;
    IQS360.Active_Channels.Touch_Channels_1 = ACTIVE_CH1;

    /*  Trackpad Active Channels    */
    IQS360.Trackpad_Active_Channels.Touch_Channels_0 = TRACKPAD_CH0;
    IQS360.Trackpad_Active_Channels.Touch_Channels_1 = TRACKPAD_CH1;

    /*  Buzzer  */
    IQS360.Buzzer.Buzzer = BUZZER;

	error = iqs360_ts_get_id(client);
	if (error) {
		dev_err(&client->dev, "Chip ID read error %d\n", error);
		return error;
	}

	error = iqs360_ts_init(client);
	if (error)
		return error;

	error = iqs360_ts_reset(client);
	if (error)
		return error;

	//error = iqs360_ts_startup(client);
	//if (error)
	//	return error;

	//status = iqs360_ts_get_status(client);
	//if (status != IQS360_STATUS_OK) {
	//	dev_err(&client->dev,
	//		"Initialization error, status: 0x%X\n", status);
	//	return -ENODEV;
	//}

	return 0;
}

static irqreturn_t iqs360_ts_threaded_irq_handler(int irq, void *id)
{
	struct iqs360_ts_data *data = id;
	struct i2c_client *client = data->client;
printk("iqs360_ts_threaded_irq_handler \n");
	iqs360_ts_read_data(client);

	return IRQ_HANDLED;
}

static void iqs360_ts_read_props(struct i2c_client *client)
{
	struct iqs360_ts_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	const char *str;
	int error;

	error = device_property_read_u32(dev, "iqs360,max-fingers",
					 &data->max_fingers);
	if (error) {
		dev_dbg(dev, "Max fingers read error %d\n", error);
		data->max_fingers = 5; /* Most devices handle up-to 5 fingers */
	}

}

//static void iqs360_disable_regulator(void *arg)
//{
//	struct iqs360_ts_data *data = arg;

//	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
//}

static int iqs360_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct iqs360_ts_data *data;
	struct device *dev = &client->dev;
	int error;
	int irq_gpio= -1;
	struct device_node *iqs360_np;
	iqs360_np = client->dev.of_node;
	printk("iqs360_ts_probe \n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				     I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		dev_err(dev, "I2C functionality check failed\n");
		return -ENXIO;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;

	/* We must have the IRQ provided by DT or ACPI subsytem */
	//if (client->irq <= 0)
	//	return -ENODEV;

	//data->regulators[0].supply = "vddio";
	//data->regulators[1].supply = "avdd";
	//error = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regulators),
	//				data->regulators);
	//if (error)
	//	return error;

	/*
	 * Enable regulators at probe and disable them at remove, we need
	 * to keep the chip powered otherwise it forgets its firmware.
	 */
	//error = regulator_bulk_enable(ARRAY_SIZE(data->regulators),
	//			      data->regulators);
	//if (error)
	//	return error;

	//error = devm_add_action_or_reset(dev, iqs360_disable_regulator, data);
	//if (error)
	//	return error;

	/* Power GPIO pin */

	irq_gpio = of_get_named_gpio(iqs360_np, "iqs360,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(&client->dev, "no iqs360,irq-gpio pin available\n");
		return -EINVAL;
		//goto err_free_pen_detect_gpio;
	}

	error = devm_gpio_request_one(&client->dev, irq_gpio, GPIOF_IN, "slide_intr");
	if (error < 0) {
		dev_err(&client->dev, "request intr gpio failed,%d\n", error);
		//goto err_free_pen_detect_gpio;
		return -EINVAL;
	}

	client->irq = gpio_to_irq(irq_gpio);
	//printk("wacom_i2c_probe irq=%d, irq_gpio=%d\n",client->irq, irq_gpio);
	if (client->irq < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", irq_gpio, client->irq);
		//goto err_free_irq_gpio;
		return -EINVAL;
	}

	//data->gpio_power = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	//if (IS_ERR(data->gpio_power)) {
	//	if (PTR_ERR(data->gpio_power) != -EPROBE_DEFER)
	//		dev_err(dev, "Shutdown GPIO request failed\n");
	//	return PTR_ERR(data->gpio_power);
	//}

	error = iqs360_ts_setup(client);
	if (error)
		return error;

	error = iqs360_ts_request_input_dev(data);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, client->irq,
					  NULL, iqs360_ts_threaded_irq_handler,
					  IRQF_ONESHOT, client->name, data);
	if (error) {
		if (error != -EPROBE_DEFER)
			dev_err(dev, "IRQ request failed %d\n", error);
		return error;
	}

	return 0;
}

static int __maybe_unused iqs360_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);
	iqs360_ts_set_power(client, IQS360_POWER_OFF);
	return 0;
}

static int __maybe_unused iqs360_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	bool second_try = false;
	int error, status;

	iqs360_ts_set_power(client, IQS360_POWER_ON);

 retry:
	error = iqs360_ts_reset(client);
	if (error)
		return error;

	//error = iqs360_ts_startup(client);
	//if (error)
	//	return error;
	error = iqs360_ts_setup(client);
	if (error)
		return error;

	//status = iqs360_ts_get_status(client);
	//if (status != IQS360_STATUS_OK) {
	//	if (!second_try) {
	//		second_try = true;
	//		dev_dbg(dev, "Reloading firmware after unsuccessful resume\n");
	//		goto retry;
	//	}
	//	dev_err(dev, "Resume error, status: 0x%02x\n", status);
	//	return -ENODEV;
	//}

	enable_irq(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(iqs360_ts_pm, iqs360_ts_suspend, iqs360_ts_resume);

static const struct i2c_device_id iqs360_ts_id[] = {
	{ "iqs360", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, iqs360_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id iqs360_ts_of_match[] = {
	{ .compatible = "iqs360,gsl1680" },
	{ .compatible = "iqs360,gsl1688" },
	{ .compatible = "iqs360,gsl3670" },
	{ .compatible = "iqs360,gsl3675" },
	{ .compatible = "iqs360,gsl3692" },
	{ },
};
MODULE_DEVICE_TABLE(of, iqs360_ts_of_match);
#endif

static struct i2c_driver iqs360_ts_driver = {
	.probe = iqs360_ts_probe,
	.id_table = iqs360_ts_id,
	.driver = {
		.name = IQS360_TS_NAME,
		.of_match_table = of_match_ptr(iqs360_ts_of_match),
		.pm = &iqs360_ts_pm,
	},
};
module_i2c_driver(iqs360_ts_driver);

MODULE_AUTHOR("Robert Dolca <robert.dolca@intel.com>");
MODULE_DESCRIPTION("Silead I2C touchscreen driver");
MODULE_LICENSE("GPL");
