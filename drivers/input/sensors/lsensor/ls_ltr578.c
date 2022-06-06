/* drivers/input/sensors/access/kxtik.c
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
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include "ls_ltr578.h"

struct ltr578_data {
    struct i2c_client *client;
    struct class *lsensor_class;
    int als_gainrange;
    uint16_t calibration_value;
	int als;
	int ir_code;
};

static struct ltr578_data *ls_data;

static uint16_t light578_calibration_data_read(uint16_t *value)
{
    int ret;

    ret = rk_vendor_read(LIGHT578_CALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static int sensor_als_read(struct i2c_client *client)
{
    int alsval_lo, alsval_mi, alsval_hi;
    int alsval = 0, clearval = 0;
    int ratio;
    int cal_factor;
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

    ratio = clearval * 100 / (alsval + 1);
    if (ratio <= 45) {
        cal_factor = 8;
    } else if (ratio <= 180) {
        cal_factor = 4;
    } else {
        cal_factor = 5;
    }

    luxdata_int = alsval * cal_factor * ALS_WIN_FACTOR / ls_data->als_gainrange;
	ls_data->als = alsval;
	ls_data->ir_code = clearval;
    printk("%s als_value: %d , clearval_value: %d , luxdata_int = %d\n", __func__, alsval, clearval, luxdata_int);
    return luxdata_int;
}

static int sensor_als_enable(struct i2c_client *client)
{
    int error, regdata;
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);

    regdata = sensor_read_reg(client, sensor->ops->ctrl_reg);
    regdata &= 0xEF;    // Clear reset bit
    regdata |= 0x02;

    error = sensor_write_reg(client, sensor->ops->ctrl_reg, regdata);

    msleep(WAKEUP_DELAY);

    return error;
}

static int sensor_als_disable(struct i2c_client *client)
{
    int error, regdata;
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

    printk(KERN_INFO "%s,enable = %d\n", __func__, enable);
    if (enable) {
        result = sensor_als_enable(client);
    } else {
        result = sensor_als_disable(client);
    }

    return result;
}

static ssize_t lux_value_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(ls_data->client);
    int result = 0, value = 0;
    ssize_t len = 0;

    if (!ls_data) {
        printk(KERN_ERR "%s ls data is null!\n", __func__);
        return 0;
    }
    sensor_active(ls_data->client, 1, 9600);
    if (sensor->pdata->irq_enable) {
        if (sensor->ops->int_status_reg) {
            value = sensor_read_reg(ls_data->client, sensor->ops->int_status_reg);
        }
    }
    value = sensor_als_read(ls_data->client);
    result = value * ls_data->calibration_value / 1000;
    //len += sprintf(_buf, "origin lux: %d, after calibration: %d\n", value, result);
	len += sprintf(_buf, "x: %d, y: %d, z: %d\n", ls_data->als, ls_data->ir_code, result);
    return len;
}
static CLASS_ATTR_RO(lux_value);

static int sensor_init(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int ret = 0, regdata;

    printk("%s:line=%d\n", __func__, __LINE__);

    /* Reset the devices */
    regdata = sensor_read_reg(client, sensor->ops->ctrl_reg);
    if ((regdata & 0x03) == 0) {
        ret = sensor_write_reg(client, sensor->ops->ctrl_reg, 0x10);
        if (ret < 0) {
            printk("%s: ALS reset fail...\n", __func__);
        }
    }
    msleep(5);

    ls_data->als_gainrange = ALS_DEF_GAIN;
    if (ls_data->als_gainrange == 1) {
        ret = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range1);
    } else if (ls_data->als_gainrange == 3) {
        ret = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range3);
    } else if (ls_data->als_gainrange == 6) {
        ret = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range6);
    } else if (ls_data->als_gainrange == 9) {
        ret = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range9);
    } else if (ls_data->als_gainrange == 18) {
        ret = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range18);
    } else {
        ret = sensor_write_reg(client, APS_RW_ALS_GAIN, MODE_ALS_Range3);
    }

    sensor_write_reg(client, APS_RW_ALS_MEAS_RATE, ALS_RESO_MEAS);
    sensor_write_reg(client, APS_RW_ALS_THRES_UP_0, 0);
    sensor_write_reg(client, APS_RW_ALS_THRES_UP_1, 0);
    sensor_write_reg(client, APS_RW_ALS_THRES_UP_2, 0);
    sensor_write_reg(client, APS_RW_ALS_THRES_LOW_0, 0xff);
    sensor_write_reg(client, APS_RW_ALS_THRES_LOW_1, 0xff);
    sensor_write_reg(client, APS_RW_ALS_THRES_LOW_2, 0xff);

    ret = sensor->ops->active(client, 0, 0);
    if (ret) {
        printk("%s:line=%d,error!\n", __func__, __LINE__);
        return ret;
    }
    sensor->status_cur = SENSOR_OFF;

    ret = light578_calibration_data_read(&ls_data->calibration_value);
    if (ret) {
        ls_data->calibration_value = 1000;
        dev_err(&client->dev, "Fail to get light3x1x calibration data!use default: %d\n", ls_data->calibration_value);
    }

    ls_data->client = client;

    return ret;
}

static int light_report_value(struct input_dev *input, int data)
{
    unsigned char index = 0;

    if (data < 0) {
        printk(KERN_ERR "%s light val err!\n", __func__);
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

    result = sensor_als_read(client);
    result = ls_data->calibration_value * result / 1000;
    index = light_report_value(sensor->input_dev, result);

    printk("%s:%s result=0x%x, index=%d\n", __func__, sensor->ops->name, result, index);

    return result;
}

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
    int ret = 0;

    ls_data = kzalloc(sizeof(struct ltr578_data), GFP_KERNEL);
    if (!ls_data) {
        dev_err(&client->dev, "%s: failed to allocate ltr578_data!\n", __func__);
        return -ENOMEM;
    }
    ls_data->lsensor_class = class_create(THIS_MODULE, client->name);
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_value);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light578 class value!\n");
    }
    ret = sensor_register_device(client, NULL, devid, &light_ltr578_ops);
    return ret;
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

MODULE_AUTHOR("tower");
MODULE_DESCRIPTION("ltr578 Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
