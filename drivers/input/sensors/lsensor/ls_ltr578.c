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
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include "ls_ltr578.h"

struct ltr578_data {
    struct i2c_client *client;
    struct class *lsensor_class;
    int als_gainrange;
    uint16_t lightcalibration_value;
    uint16_t darkcalibration_value;
    uint16_t calibration_reference;
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

static uint16_t light578_calibration_data_write(uint16_t *value)
{
    int ret;

    ret = rk_vendor_write(LIGHT578_CALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static uint16_t light578_darkcalibration_data_read(uint16_t *value)
{
    int ret;

    ret = rk_vendor_read(LIGHT578_DCALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static uint16_t light578_darkcalibration_data_write(uint16_t *value)
{
    int ret;

    ret = rk_vendor_write(LIGHT578_DCALIBRATION_ID, (void *)value, 2);
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
    if (ratio <= 240) {
        cal_factor = 9;
    } else if (ratio <= 2000) {
        cal_factor = 7;
    } else {
        cal_factor = 7;
    }

    alsval = ((alsval - ls_data->darkcalibration_value) < 0) ? 0 : (alsval - ls_data->darkcalibration_value);
    luxdata_int = alsval * cal_factor * ALS_WIN_FACTOR / ls_data->als_gainrange;
    ls_data->als = alsval;
    ls_data->ir_code = clearval;
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

    if (enable) {
        result = sensor_als_enable(client);
    } else {
        result = sensor_als_disable(client);
    }

    dev_info(&client->dev, "%s enable: %d ret: %d\n", __func__, enable, result);
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
    } else {
        msleep(100);
    }
    value = sensor_als_read(ls_data->client);
    result = value * ls_data->lightcalibration_value / 100;
    sensor_active(ls_data->client, 0, 9600);
    printk("ltr578 value: ----ret:%d alsraw:%d value:%d ir:%d cail:%d darkcail:%d lightcailref:%d----\n", result, ls_data->als, value, ls_data->ir_code, ls_data->lightcalibration_value, ls_data->darkcalibration_value, ls_data->calibration_reference);
    len += sprintf(_buf, "%d\n", result);
    return len;
}
static CLASS_ATTR_RO(lux_value);

static ssize_t lux_rawdata_show(struct class *cls, struct class_attribute *attr, char *_buf)
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
    } else {
        msleep(100);
    }
    value = sensor_als_read(ls_data->client);
    result = value * ls_data->lightcalibration_value / 100;
    sensor_active(ls_data->client, 0, 9600);
    printk("ltr578 raw: ----ret:%d alsraw:%d value:%d ir:%d cail:%d darkcail:%d lightcailref:%d----\n", result, ls_data->als, value, ls_data->ir_code, ls_data->lightcalibration_value, ls_data->darkcalibration_value, ls_data->calibration_reference);
    len += sprintf(_buf, "x: %d, y: %d, z: %d\n", result, ls_data->als, ls_data->ir_code);
    return len;
}
static CLASS_ATTR_RO(lux_rawdata);

static int do_calibration(struct sensor_private_data *sensor, int dark)
{
    int ret = -1, i = 0;
    int count = 10, adjvalue = 0;

    sensor_active(ls_data->client, 1, 9600);

    if (sensor->ops->read_len < 2) {
        dev_err(&ls_data->client->dev, "%s:length is error, len=%d\n", __func__, sensor->ops->read_len);
        return ret;
    }

    for (i = 0; i < count; i++) {
        msleep(80);
        sensor_als_read(ls_data->client);
        adjvalue += ls_data->als;
        printk("ltr578 cal: ----cout %d adjvalue: %d, value: %d\n", i, adjvalue, ls_data->als);
    }

    if (!dark) {
        adjvalue = (adjvalue / i) ? (adjvalue / i) : ls_data->calibration_reference;
        ls_data->lightcalibration_value = (ls_data->calibration_reference * 100) / adjvalue;
        printk("ltr578 cal: ----light calibration count: %d value: %d\n", i, ls_data->lightcalibration_value);
        ret = light578_calibration_data_write(&ls_data->lightcalibration_value);
        if (ret) {
            printk(KERN_ERR "%s wirte calibration fail!\n", __func__);
            return ret;
        }
    } else {
        adjvalue = adjvalue / i;
        ls_data->darkcalibration_value = adjvalue;
        printk("ltr578 cal: ----dark calibration count: %d, value: %d\n", i, ls_data->darkcalibration_value);
        ret = light578_darkcalibration_data_write(&ls_data->darkcalibration_value);
        if (ret) {
            printk(KERN_ERR "%s wirte dark calibration fail!\n", __func__);
            return ret;
        }
    }
    sensor_active(ls_data->client, 0, 9600);

    return 0;
}

static ssize_t lux_calibration_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(ls_data->client);
    uint16_t value = 0, dvalue = 0;
    int len = 0;

    if (!ls_data) {
        printk(KERN_ERR "%s ls data is null!\n", __func__);
        return len;
    }
    if (light578_calibration_data_read(&value)) {
        printk(KERN_ERR "read light ltr578 calibration error!\n");
        value = ls_data->lightcalibration_value;
    }
    if (light578_darkcalibration_data_read(&dvalue)) {
        printk(KERN_ERR "read light ltr578 darkcalibration error!\n");
        dvalue = ls_data->darkcalibration_value;
    }
    len += sprintf(_buf, "%d %d\n", dvalue, value);
    return len;
}

static ssize_t lux_calibration_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(ls_data->client);
    int value = 0, dark = 0;
    int ret = -1, pre_status;
    uint16_t zero = 0, defcali = 100;

    if (!ls_data) {
        printk(KERN_ERR "%s ls data is null!\n", __func__);
        return ret;
    }

    ret = kstrtoint(buf, 10, &value);
    if (ret) {
        printk(KERN_ERR "%s: kstrtoint error return %d\n", __func__, ret);
        return ret;
    }
    if (0 == value) {
        light578_calibration_data_write(&defcali);
        ls_data->lightcalibration_value = defcali;
        light578_darkcalibration_data_write(&zero);
        ls_data->darkcalibration_value = 0;
        return count;
    } else if (1 == value) {
        dark = 0;
    } else if (2 == value) {
        dark = 1;
    } else {
        printk(KERN_ERR "%s: error value\n", __func__);
        return -1;
    }

    atomic_set(&sensor->is_factory, 1);

    pre_status = sensor->status_cur;
    if (pre_status == SENSOR_OFF) {
        mutex_lock(&sensor->operation_mutex);
        sensor->ops->active(sensor->client, SENSOR_ON, sensor->pdata->poll_delay_ms);
        mutex_unlock(&sensor->operation_mutex);
    } else {
        sensor->stop_work = 1;
        if (sensor->pdata->irq_enable) {
            disable_irq_nosync(sensor->client->irq);
        } else {
            cancel_delayed_work_sync(&sensor->delaywork);
        }
    }
    ret = do_calibration(sensor, dark);
    if (ret) {
        printk(KERN_ERR "%s: calibration fail!\n", __func__);
    }

    if (pre_status == SENSOR_ON) {
        sensor->stop_work = 0;
        if (sensor->pdata->irq_enable) {
            enable_irq(sensor->client->irq);
        } else {
            schedule_delayed_work(&sensor->delaywork, msecs_to_jiffies(sensor->pdata->poll_delay_ms));
        }
    } else {
        mutex_lock(&sensor->operation_mutex);
        sensor->ops->active(sensor->client, SENSOR_OFF, sensor->pdata->poll_delay_ms);
        mutex_unlock(&sensor->operation_mutex);
    }

    atomic_set(&sensor->is_factory, 0);
    wake_up(&sensor->is_factory_ok);

    return ret ? -1 : count;
}
static CLASS_ATTR_RW(lux_calibration);

static int sensor_init(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int ret = 0, regdata;

    printk("ltr578 %s init ...\n", __func__);

#if 0
    /* Reset the devices */
    regdata = sensor_read_reg(client, sensor->ops->ctrl_reg);
    if ((regdata & 0x03) == 0) {
        ret = sensor_write_reg(client, sensor->ops->ctrl_reg, 0x10);
        if (ret < 0) {
            printk("%s: ALS reset fail...\n", __func__);
        }
    }
    msleep(5);
#endif
    ret = sensor->ops->active(client, 0, 0);
    if (ret) {
        printk("%s:line=%d,error!\n", __func__, __LINE__);
        return ret;
    }
    sensor->status_cur = SENSOR_OFF;

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

    ret = light578_calibration_data_read(&ls_data->lightcalibration_value);
    if (ret) {
        ls_data->lightcalibration_value = 100;
        dev_err(&client->dev, "Fail to get lightltr578 calibration data!use default: %d\n", ls_data->lightcalibration_value);
    }
    ret = light578_darkcalibration_data_read(&ls_data->darkcalibration_value);
    if (ret) {
        ls_data->darkcalibration_value = 0;
        dev_err(&client->dev, "Fail to get lightltr578 calibration data!use dark default: %d\n", ls_data->darkcalibration_value);
    }

    ls_data->client = client;
    printk("ltr578 %s init successful \n", __func__);
    return ret;
}

/*
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
*/

static int sensor_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0;
    char value = 0;
    //char index = 0;

    if (sensor->pdata->irq_enable) {
        if (sensor->ops->int_status_reg) {
            value = sensor_read_reg(client, sensor->ops->int_status_reg);
        }
    }

    result = sensor_als_read(client);
    result = ls_data->lightcalibration_value * result / 100;

    // 20220627: do this at sensor-dev.c for two lsensor.
    //index = light_report_value(sensor->input_dev, result);

    pr_debug("%s:%s report value 0x%x\n", __func__, sensor->ops->name, result);
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
    .range              = {0, 65535},           //range -- ABS_MISC  {0, 10}, //
    .brightness         = {0, 255},             // brightness -- ABS_TOOL_WIDTH
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
    ls_data->calibration_reference = 2350;
    ls_data->lsensor_class = class_create(THIS_MODULE, client->name);
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_value);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light578 class value!\n");
    }
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_calibration);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light578 class calibration!\n");
    }
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_rawdata);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light578 class raw!\n");
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

static int __init light_ltr578_init(void)
{
    return i2c_add_driver(&light_ltr578_driver);;
}
late_initcall(light_ltr578_init);

static void __exit light_ltr578_exit(void)
{
    i2c_del_driver(&light_ltr578_driver);
}
module_exit(light_ltr578_exit);

MODULE_AUTHOR("tower");
MODULE_DESCRIPTION("ltr578 Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
