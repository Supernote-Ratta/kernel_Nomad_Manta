/*
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include "ls_ltr578.h"
#include <linux/cdev.h>

static int als_gainrange = ALS_DEF_GAIN;
static struct i2c_client *client_test = NULL;
static int cal_factor = 1000;
static struct class *lsensor_class;

int sensor_als_read(struct i2c_client *client)
{
    int alsval_lo, alsval_mi, alsval_hi;
    int alsval = 0, clearval = 0;
    int luxdata_int;

    alsval_lo = sensor_read_reg(client, APS_RO_ALS_DATA_0);
    alsval_mi = sensor_read_reg(client, APS_RO_ALS_DATA_1);
    alsval_hi = sensor_read_reg(client, APS_RO_ALS_DATA_2);
    alsval = (alsval_hi << 16) + (alsval_mi << 8) + alsval_lo;

    alsval_lo = sensor_read_reg(client, APS_RO_CLEAR_DATA_0);
    alsval_mi = sensor_read_reg(client, APS_RO_CLEAR_DATA_1);
    alsval_hi = sensor_read_reg(client, APS_RO_CLEAR_DATA_2);
    clearval = (alsval_hi << 16) + (alsval_mi << 8) + alsval_lo;

    if (alsval == 0) {
        luxdata_int = 0;
        return luxdata_int;
    }

    if (ALS_USE_CLEAR_DATA == 1) {
        // ALS_Lux = ALS_DATA * 0.8 * WINFAC1 * (1 ?WINFAC2 * CLEAR_DATA / ALS_DATA) / ALS_GAIN / ALS_INT
#if 0
        luxdata_int = alsval * 8 * ALS_WIN_FACTOR * (1 - ALS_WIN_FACTOR2 * clearval / alsval / 1000) / als_gainrange / 10;
#else
        luxdata_int = alsval * 8 * ALS_WIN_FACTOR * (1 - ALS_WIN_FACTOR2 * clearval / alsval / 1000) / 10;
#endif
    } else {
#if 0
        luxdata_int = alsval * 8 * ALS_WIN_FACTOR / als_gainrange / 10;//formula: ALS counts * 0.8/gain/int , int=1
#else
        luxdata_int = alsval * 8 * ALS_WIN_FACTOR / 10;
#endif
    }

    printk("[ltr578als] %s,als_value_lux = %d\n", __func__, luxdata_int);
    return luxdata_int;
}

static int sensor_als_enable(struct i2c_client *client)
{
    int error = 0;
    int regdata = 0;

    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);

    regdata = sensor_read_reg(client, sensor->ops->ctrl_reg);
    regdata &= 0xEF;    // Clear reset bit
    regdata |= 0x02;

    error = sensor_write_reg(client, sensor->ops->ctrl_reg, regdata);

    msleep(WAKEUP_DELAY);

    return error;
}

// Put ALS into Standby mode
static int sensor_als_disable(struct i2c_client *client)
{
    int error = 0;
    int regdata = 0;

    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);

    regdata = sensor_read_reg(client, sensor->ops->ctrl_reg);
    regdata &= 0xEF;    // Clear reset bit
    regdata &= 0xFD;

    error = sensor_write_reg(client, sensor->ops->ctrl_reg, regdata);
    return error;
}

/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
    int result = 0;

    client_test = client;

    printk("[ltr578als] %s,enable = %d\n", __func__, enable);
    if (enable) {
        result = sensor_als_enable(client);
    } else {
        result = sensor_als_disable(client);
    }

    return result;
}

// class node
// cat /sys/class/ls_ltr578/sensor_value
// cat /sys/class/ls_ltr578/sensor_ctrl
static ssize_t sensor_value_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client_test);
    int result = 0;
    char value = 0;
	ssize_t len = 0;
    sensor_active(client_test, 1, 9600);

    if (sensor->pdata->irq_enable) {
        if (sensor->ops->int_status_reg) {
            value = sensor_read_reg(client_test, sensor->ops->int_status_reg);
        }
    }

    //result = sensor->ops->active(client,1,0);
    result = sensor_als_read(client_test);
    result = (cal_factor * result) / 1000;

    len += sprintf(_buf, "%d:\n",result);
	return len;
}


static ssize_t sensor_value_store(struct class *cls,struct class_attribute *attr, const char *buf, size_t _count)
{

	return 0;
}

static ssize_t sensor_ctrl_show(struct class *cls,struct class_attribute *attr, char *_buf)
{

	ssize_t len = 0;

	return len;
}


static ssize_t sensor_ctrl_store(struct class *cls,struct class_attribute *attr, const char *buf, size_t _count)
{

	return 0;
}

static CLASS_ATTR_RW(sensor_value);
static CLASS_ATTR_RW(sensor_ctrl);

static int sensor_init(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0;
    //int regdata;

    printk("[ltr578als] %s:line=%d\n", __func__, __LINE__);

#if 0
    /* Reset the devices */
    regdata = sensor_read_reg(client, sensor->ops->ctrl_reg);
    if ((regdata & 0x03) == 0) {
        result = sensor_write_reg(client, sensor->ops->ctrl_reg, 0x10);
        if (result < 0) {
            DBG("[ltr578als] %s: ALS reset fail...\n", __func__);
        }
    }
    msleep(5);
#endif

    if (als_gainrange == 1) {
        result = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range1);
    } else if (als_gainrange == 3) {
        result = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range3);
    } else if (als_gainrange == 6) {
        result = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range6);
    } else if (als_gainrange == 9) {
        result = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range9);
    } else if (als_gainrange == 18) {
        result = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range18);
    } else {
        result = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range3);
    }

    sensor_write_reg(client, APS_RW_ALS_MEAS_RATE, ALS_RESO_MEAS);
    sensor_write_reg(client, APS_RW_ALS_THRES_UP_0, 0);
    sensor_write_reg(client, APS_RW_ALS_THRES_UP_1, 0);
    sensor_write_reg(client, APS_RW_ALS_THRES_UP_2, 0);
    sensor_write_reg(client, APS_RW_ALS_THRES_LOW_0, 0xff);
    sensor_write_reg(client, APS_RW_ALS_THRES_LOW_1, 0xff);
    sensor_write_reg(client, APS_RW_ALS_THRES_LOW_2, 0xff);

    result = sensor->ops->active(client, 0, 0);
    if (result) {
        printk("[ltr578als] %s:line=%d,error\n", __func__, __LINE__);
        return result;
    }

    sensor->status_cur = SENSOR_OFF;
	lsensor_class = class_create(THIS_MODULE, client->name);
	result = class_create_file(lsensor_class, &class_attr_sensor_value);
	if (result)
		printk("[ltr578als] Fail to create class sensor_class_value.\n");
	result = class_create_file(lsensor_class, &class_attr_sensor_ctrl);
	if (result)
		printk("[ltr578als] Fail to create class sensor_class_ctrl.\n");

    return result;
}

static int light_report_value(struct input_dev *input, int data)
{
    unsigned char index = 0;

    if (data < 0) {
        printk("[ltr578als] light val err\n");
        data = 0; // no light
    }

    if (data <= 10) {
        index = 0; goto report;
    } else if (data <= 160) {
        index = 1; goto report;
    } else if (data <= 225) {
        index = 2; goto report;
    } else if (data <= 320) {
        index = 3; goto report;
    } else if (data <= 640) {
        index = 4; goto report;
    } else if (data <= 1280) {
        index = 5; goto report;
    } else if (data <= 2600) {
        index = 6; goto report;
    } else {
        index = 7; goto report;
    }

report:
    input_report_abs(input, ABS_MISC, index);
    input_sync(input);
    return index;
}

static int sensor_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0;
    char value = 0;
    char index = 0;

    if (sensor->pdata->irq_enable) {
        if (sensor->ops->int_status_reg) {
            value = sensor_read_reg(client, sensor->ops->int_status_reg);
        }
    }

    //result = sensor->ops->active(client,1,0);
    result = sensor_als_read(client);
    result = (cal_factor * result) / 1000;
    index = light_report_value(sensor->input_dev, result);

    printk("[ltr578als] %s:%s result=0x%x,index=%d\n", __func__, sensor->ops->name, result, index);

    return result;
}

static int sensor_report_value1(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0;
    char value = 0;
    char index = 0;

    if (sensor->pdata->irq_enable) {
        if (sensor->ops->int_status_reg) {
            value = sensor_read_reg(client, sensor->ops->int_status_reg);
        }
    }

    //result = sensor->ops->active(client,1,0);
    result = sensor_als_read(client);
    index = light_report_value(sensor->input_dev, result);

    printk("[ltr578als] %s:%s result=0x%x,index=%d\n", __func__, sensor->ops->name, result, index);

    return result;
}

int light_ltr578_get_value(void)
{
    int result = 0;

    sensor_active(client_test, 1, 9600);
    result = sensor_report_value(client_test);
    mdelay(100);
    sensor_active(client_test, 1, 9600);
    result = sensor_report_value(client_test);
    printk("%s: result=%4d\n", __func__, result);

    return result;
}
EXPORT_SYMBOL(light_ltr578_get_value);

void light_ltr578_set_cal_factor(int level)
{
    int result = 0;

    sensor_active(client_test, 1, 9600);
    result = sensor_report_value1(client_test);
    mdelay(100);
    sensor_active(client_test, 1, 9600);
    result = sensor_report_value1(client_test);
    cal_factor = (level * 1000) / result;
    printk("%s: level=%4d, result=%4d, cal_factor=%4d\n", __func__, level, result, cal_factor);

}
EXPORT_SYMBOL(light_ltr578_set_cal_factor);

int light_ltr578_get_cal_factor(void)
{
    int result = 0;

    result = cal_factor;
    printk("%s: result=%4d\n", __func__, result);

    return result;

}
EXPORT_SYMBOL(light_ltr578_get_cal_factor);

struct sensor_operate light_ltr578_ops = {
    .name               = "ls_ltr578",
    .type               = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c             = LIGHT_ID_LTR578_ALS,  //i2c id number
    .read_reg           = SENSOR_UNKNOW_DATA,   //read data
    .read_len           = 3,                    //data length
    .id_reg             = APS_RO_PART_ID,       //read device id from this register
    .id_data            = PARTID,               //device id
    .precision          = 18,                   //18 bits
    .ctrl_reg           = APS_RW_MAIN_CTRL,     //enable or disable
    .int_status_reg     = APS_RO_MAIN_STATUS,   //intterupt status register
    .range              = {100, 65535},         //range
    .brightness         = {10, 255},            // brightness
    .trig               = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
    .active             = sensor_active,
    .init               = sensor_init,
    .report             = sensor_report_value,
};

/****************operate according to sensor chip:end************/

static int light_ltr578_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    return sensor_register_device(client, NULL, devid, &light_ltr578_ops);
}

static int light_ltr578_remove(struct i2c_client *client)
{
    return sensor_unregister_device(client, NULL, &light_ltr578_ops);
}

static const struct i2c_device_id light_ltr578_id[] = {
    {"ls_ltr578", LIGHT_ID_LTR578_ALS},
    {}
};

static struct i2c_driver light_ltr578_driver = {
    .probe = light_ltr578_probe,
    .remove = light_ltr578_remove,
    .shutdown = sensor_shutdown,
    .id_table = light_ltr578_id,
    .driver = {
        .name = "light_ltr578",
#ifdef CONFIG_PM
        .pm = &sensor_pm_ops,
#endif
    },
};

module_i2c_driver(light_ltr578_driver);

MODULE_AUTHOR("Bin Yang <yangbin@rock-chips.com>");
MODULE_DESCRIPTION("ltr578 light driver");
MODULE_LICENSE("GPL");
