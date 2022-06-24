/*
 *  ls_stk3x1x.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x
 *  proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2015 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sensor-dev.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif
#include "ls_stk3x1x.h"

#define DRIVER_VERSION                "3.22.0_0601"

/* Driver Settings */
#define STK_POLL_ALS                  /* ALS interrupt is valid only when STK_PS_INT_MODE = 1 or 4*/
#define STK_IRS
#define STK_DEBUG_PRINTF

#define PROXIMITY_ID_I2C               2

/*****************************************************************************/
/* Define Register Map */
#define STK_STATE_REG                  0x00
#define STK_PSCTRL_REG                 0x01
#define STK_ALSCTRL_REG                0x02
#define STK_LEDCTRL_REG                0x03
#define STK_INT_REG                    0x04
#define STK_WAIT_REG                   0x05
#define STK_THDH1_PS_REG               0x06
#define STK_THDH2_PS_REG               0x07
#define STK_THDL1_PS_REG               0x08
#define STK_THDL2_PS_REG               0x09
#define STK_THDH1_ALS_REG              0x0A
#define STK_THDH2_ALS_REG              0x0B
#define STK_THDL1_ALS_REG              0x0C
#define STK_THDL2_ALS_REG              0x0D
#define STK_FLAG_REG                   0x10
#define STK_DATA1_PS_REG               0x11
#define STK_DATA2_PS_REG               0x12
#define STK_DATA1_ALS_REG              0x13
#define STK_DATA2_ALS_REG              0x14
#define STK_DATA1_OFFSET_REG           0x15
#define STK_DATA2_OFFSET_REG           0x16
#define STK_DATA1_IR_REG               0x17
#define STK_DATA2_IR_REG               0x18
#define STK_PDT_ID_REG                 0x3E
#define STK_RSRVD_REG                  0x3F
#define STK_SW_RESET_REG               0x80

#define STK_STATE_EN_IRS_MASK          0x80
#define STK_STATE_EN_AK_MASK           0x40
#define STK_STATE_EN_ASO_MASK          0x20
#define STK_STATE_EN_IRO_MASK          0x10
#define STK_STATE_EN_WAIT_MASK         0x04
#define STK_STATE_EN_ALS_MASK          0x02
#define STK_STATE_EN_PS_MASK           0x01

#define STK_FLG_ALSDR_MASK             0x80
#define STK_FLG_PSDR_MASK              0x40
#define STK_FLG_ALSINT_MASK            0x20
#define STK_FLG_PSINT_MASK             0x10
#define STK_FLG_OUI_MASK               0x04
#define STK_FLG_IR_RDY_MASK            0x02
#define STK_FLG_NF_MASK                0x01

#define STK_INT_ALS                    0x08

#define STK_IRC_MAX_ALS_CODE           20000
#define STK_IRC_MIN_ALS_CODE           25
#define STK_IRC_MIN_IR_CODE            50
#define STK_IRC_ALS_DENOMI             2
#define STK_IRC_ALS_NUMERA             5
#define STK_IRC_ALS_CORREC             850

#define STK_IRS_IT_REDUCE              2
#define STK_ALS_READ_IRS_IT_REDUCE     5
#define STK_ALS_THRESHOLD              30

#define LIGHT_SLOPE_CWF                1000
#define LIGHT_SLOPE_D65                860
#define LIGHT_SLOPE_A                  540
#define LIGHT_RATIO_D                  350
#define LIGHT_RATIO_A                  24800

/*****************************************************************************/
#define STK3310SA_PID                  0x17
#define STK3311SA_PID                  0x1E
#define STK3311WV_PID                  0x1D
#define STK3321WV_PID                  0x15
/*****************************************************************************/
/* FLAG 0x10 */
#define STK_FLAG_NF                    (1 << 0)
#define STK_FLAG_IR_RDY                (1 << 1)
#define STK_FLAG_OUI                   (1 << 2)
#define STK_FLAG_PSINT                 (1 << 4)
#define STK_FLAG_ALSINT                (1 << 5)
#define STK_FLAG_PSDR                  (1 << 6)
#define STK_FLAG_ALSDR                 (1 << 7)

struct stk3x1x_data {
    struct i2c_client *client;
    struct class *lsensor_class;
    uint16_t ir_code;
    uint16_t als_correct_factor;
    uint16_t lightcalibration_value;
    uint16_t darkcalibration_value;
    uint16_t calibration_reference;
    uint8_t alsctrl_reg;
    uint8_t psctrl_reg;
    uint8_t ledctrl_reg;
    uint8_t state_reg;
    int int_pin;
    uint8_t wait_reg;
    uint8_t int_reg;
    uint16_t ps_thd_h;
    uint16_t ps_thd_l;
    struct mutex io_lock;
    struct input_dev *ps_input_dev;
    int32_t ps_distance_last;
    bool ps_enabled;
    bool re_enable_ps;
    struct wake_lock ps_wakelock;
    struct input_dev *als_input_dev;
    int32_t als_lux_last;
    uint32_t als_transmittance;
    bool als_enabled;
    bool re_enable_als;
    ktime_t ps_poll_delay;
    ktime_t als_poll_delay;
#ifdef STK_POLL_ALS
    struct work_struct stk_als_work;
    struct hrtimer als_timer;
    struct workqueue_struct *stk_als_wq;
#endif
    bool first_boot;
#ifdef STK_ALS_FIR
    struct data_filter fir;
    atomic_t firlength;
#endif
    atomic_t recv_reg;
    int als_data_index;
    uint8_t pid;
    uint8_t p_wv_r_bd_with_co;
    uint32_t als_code_last;
};

static struct stk3x1x_data *ls_data;
const int ALS_LEVEL[] = { 100, 1600, 2250, 3200, 6400, 12800, 26000 };

static uint16_t light3x1x_calibration_data_read(uint16_t *value)
{
    int ret;

    ret = rk_vendor_read(LIGHT3X1X_CALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static uint16_t light3x1x_calibration_data_write(uint16_t *value)
{
    int ret;

    ret = rk_vendor_write(LIGHT3X1X_CALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static uint16_t light3x1x_darkcalibration_data_read(uint16_t *value)
{
    int ret;

    ret = rk_vendor_read(LIGHT3X1X_DCALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static uint16_t light3x1x_darkcalibration_data_write(uint16_t *value)
{
    int ret;

    ret = rk_vendor_write(LIGHT3X1X_DCALIBRATION_ID, (void *)value, 2);
    if (ret < 0) {
        printk(KERN_ERR "%s failed!\n", __func__);
        return ret;
    }

    return 0;
}

static struct stk3x1x_platform_data stk3x1x_pfdata = {
    .state_reg = 0x0,      /* disable all */
    .psctrl_reg = 0x31,    /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */
    .alsctrl_reg = 0x3A,   /* als_persistance=1, als_gain=64X, ALS_IT=100ms */
    .ledctrl_reg = 0xFF,   /* 100mA IRDR, 64/64 LED duty */
    .wait_reg = 0x07,      /* 50 ms */
    .ps_thd_h = 800,
    .ps_thd_l = 600,
    //.int_pin = sprd_3rdparty_gpio_pls_irq,
    .transmittance = 500,
    .stk_max_min_diff = 200,
    .stk_lt_n_ct = 60,
    .stk_ht_n_ct = 80,
};

/*****************************************************************************/
#ifndef STK_POLL_ALS
static int32_t stk3x1x_set_als_thd_l(struct i2c_client *client, uint16_t thd_l)
{
    unsigned char val[3];
    int ret;

    val[0] = STK_THDL1_ALS_REG;
    val[1] = (thd_l & 0xFF00) >> 8;
    val[2] = thd_l & 0x00FF;
    ret = sensor_tx_data(client, val, 3);

    // ret = sensor_write_reg(client, STK_THDL1_ALS_REG, );
    // if(ret)
    // printk("%s:fail to active sensor\n",__func__);

    return ret;
}

static int32_t stk3x1x_set_als_thd_h(struct i2c_client *client, uint16_t thd_h)
{
    unsigned char val[2];
    int ret;

    val[0] = STK_THDH1_ALS_REG;
    val[1] = (thd_h & 0xFF00) >> 8;
    val[2] = thd_h & 0x00FF;
    ret = sensor_tx_data(client, val, 3);
    // ret = sensor_write_reg(client, STK_THDL1_ALS_REG, );
    // if(ret)
    // printk("%s:fail to active sensor\n",__func__);
    return ret;
}
#endif

static int32_t stk3x1x_check_pid(struct i2c_client *client)
{
    char value = 0;
    int result;

    ls_data->p_wv_r_bd_with_co = 0;
    value = sensor_read_reg(client, STK_PDT_ID_REG);

    printk("%s: PID=0x%x\n", __func__, value);
    ls_data->pid = value;
    if (ls_data->pid == STK3321WV_PID) {
        ls_data->p_wv_r_bd_with_co |= 0b100;
    }

    printk("%s: p_wv_r_bd_with_co = 0x%x\n", __func__, ls_data->p_wv_r_bd_with_co);
    return 0;
}

static int light_report_abs_value(struct input_dev *input, int data)
{
    unsigned char index = 0;

    if (data <= ALS_LEVEL[0]) {
        index = 0; goto report;
    } else if (data <= ALS_LEVEL[1]) {
        index = 1; goto report;
    } else if (data <= ALS_LEVEL[2]) {
        index = 2; goto report;
    } else if (data <= ALS_LEVEL[3]) {
        index = 3; goto report;
    } else if (data <= ALS_LEVEL[4]) {
        index = 4; goto report;
    } else if (data <= ALS_LEVEL[5]) {
        index = 5; goto report;
    } else if (data <= ALS_LEVEL[6]) {
        index = 6; goto report;
    } else {
        index = 7; goto report;
    }

report:
    input_report_abs(input, ABS_MISC, index);
    input_sync(input);

    return index;
}

static int32_t stk3x1x_set_irs_it_slp(struct i2c_client *client, uint16_t *slp_time, int32_t ials_it_reduce)
{
    int irs_alsctrl;
    int32_t ret;

    irs_alsctrl = (stk3x1x_pfdata.alsctrl_reg & 0x0F) - 4;
    switch (irs_alsctrl) {
        case 2:
            *slp_time = 1;
            break;
        case 3:
            *slp_time = 2;
            break;
        case 4:
            *slp_time = 3;
            break;
        case 5:
            *slp_time = 6;
            break;
        case 6:
            *slp_time = 12;
            break;
        case 7:
            *slp_time = 24;
            break;
        case 8:
            *slp_time = 48;
            break;
        case 9:
            *slp_time = 96;
            break;
        case 10:
            *slp_time = 192;
            break;
        default:
            printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
            ret = -EINVAL;
            return ret;
    }
    irs_alsctrl |= (stk3x1x_pfdata.alsctrl_reg & 0xF0);
    ret = sensor_write_reg(client, STK_ALSCTRL_REG, irs_alsctrl);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static uint16_t stk3x1x_get_ir_reading(struct i2c_client *client, int32_t als_it_reduce)
{
    int ret = 0;
    uint16_t ir_data = 0;
    int w_reg, retry = 0;
    uint16_t irs_slp_time = 100;
    char buffer[2] = {0};

    ret = stk3x1x_set_irs_it_slp(client, &irs_slp_time, als_it_reduce);
    if (ret) {
        printk(KERN_ERR "%s set irs it slp error!\n", __func__);
        return 0;
    }

    w_reg = sensor_read_reg(client, STK_STATE_REG);
    if (w_reg <= 0) {
        printk("stk %s i2c error(%d)\n", __func__, w_reg);
        return 0;
    }

    w_reg |= STK_STATE_EN_IRS_MASK;
    ret = sensor_write_reg(client, STK_STATE_REG, w_reg);
    if (ret) {
        printk("stk %s en ir error(%d)\n", __func__, w_reg);
        return 0;
    }
    msleep(irs_slp_time);
    do {
        w_reg = sensor_read_reg(client, STK_FLAG_REG);
        if (w_reg & STK_FLG_IR_RDY_MASK) {
            break;
        }
        usleep_range(3000, 4000);
        retry++;
    } while (retry < 10);

    if (retry == 10) {
        printk(KERN_ERR "%s: ir data is not ready for a long time\n", __func__);
        return ir_data;
    }

    buffer[0] = STK_DATA1_IR_REG;
    ret = sensor_rx_data(client, buffer, 2);
    if (ret) {
        printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
        return 0;
    }
    ir_data = ((buffer[0] << 8) | buffer[1]);

    w_reg &= (~STK_FLG_IR_RDY_MASK);
    ret = sensor_write_reg(client, STK_FLAG_REG, w_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s write STK_FLAG_REG error!!\n", __func__);
        return 0;
    }

    ret = sensor_write_reg(client, STK_ALSCTRL_REG, stk3x1x_pfdata.alsctrl_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s write STK_ALSCTRL_REG error!!\n", __func__);
        return 0;
    }

    return ir_data;
}

static int stk_als_ir_skip_als(struct i2c_client *client, struct sensor_private_data *sensor)
{
    int ret;
    unsigned char buffer[2] = {0};

    if (ls_data->als_data_index < 60000) {
        ls_data->als_data_index++;
    } else {
        ls_data->als_data_index = 0;
    }

    if (ls_data->als_data_index % 10 == 1) {
        buffer[0] = STK_DATA1_ALS_REG;
        ret = sensor_rx_data(client, buffer, 2);
        if (ret) {
            printk(KERN_ERR "%s:line=%d,error=%d\n", __func__, __LINE__, ret);
            return ret;
        }
        return 1;
    }
    return 0;
}

static void stk_als_ir_get_corr(int32_t als)
{
    int32_t als_comperator;
    int32_t ir_ratio;

    if (ls_data->ir_code) {
        ls_data->als_correct_factor = 1000;
        ir_ratio = (ls_data->ir_code * 100) / (als + 1);

        /*
        if (ls_data->ir_code > 32000) {
            ls_data->als_correct_factor = LIGHT_SLOPE_A;
            printk("stk3x1x_als_compensation: light type = A");
        }
        */
        if (ls_data->ir_code > LIGHT_RATIO_A) {
            ls_data->als_correct_factor = LIGHT_SLOPE_A * (10000 - (ir_ratio * 3 - 5061)) / 10000;
            printk(KERN_DEBUG "%s: stk als factor A=%d", __func__, ls_data->als_correct_factor);
        } else if (ls_data->ir_code > LIGHT_RATIO_D) {
            ls_data->als_correct_factor = LIGHT_SLOPE_D65 * (10000 - (ir_ratio * 2 - 1355)) / 10000;
            printk(KERN_DEBUG "%s: stk als factor D=%d", __func__, ls_data->als_correct_factor);
        } else {
            ls_data->als_correct_factor = LIGHT_SLOPE_CWF;
            printk(KERN_DEBUG "%s: stk als factor C=%d", __func__, ls_data->als_correct_factor);
        }
    }

    ls_data->ir_code = 0;
}

/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0;

    sensor->ops->ctrl_data = 0;
    if (enable) {
        sensor->ops->ctrl_data |= (STK_STATE_EN_ALS_MASK);
    }
    result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
    if (result) {
        printk(KERN_ERR "%s:fail to active sensor\n", __func__);
    }

    if (enable) {
        ls_data->als_data_index = 0;
    }
    ls_data->als_enabled = enable ? true : false;
    return result;
}

static ssize_t lux_value_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(ls_data->client);
    int result = 0, value = 0, rawvalue = 0, ircode = 0;
    ssize_t len = 0, retry = 0;
    char index = 0;
    char buffer[2] = {0};

    if (!ls_data) {
        printk(KERN_ERR "%s ls data is null!\n", __func__);
        return 0;
    }

    sensor_active(ls_data->client, 1, 9600);
    if (sensor->ops->read_len < 2) {
        printk(KERN_ERR "%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
        return 0;
    }
    do {
        value = sensor_read_reg(ls_data->client, STK_FLAG_REG);
        if (value < 0) {
            printk("stk %s read als data flag error, ret=%d\n", __func__, value);
            return value;
        }
        if (value & STK_FLG_ALSDR_MASK) {
            break;
        }
        usleep_range(3000, 4000);
        retry++;
    } while (retry < 1000);
    buffer[0] = sensor->ops->read_reg;
    result = sensor_rx_data(ls_data->client, buffer, sensor->ops->read_len);
    if (result) {
        printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
        return 0;
    }
    rawvalue = (buffer[0] << 8) | buffer[1];
    value = ((rawvalue - ls_data->darkcalibration_value) < 0) ? 0 : (rawvalue - ls_data->darkcalibration_value);
    ls_data->ir_code = stk3x1x_get_ir_reading(ls_data->client, STK_IRS_IT_REDUCE);
    ircode = ls_data->ir_code;
    stk_als_ir_get_corr(value);
    result = (value * ls_data->als_correct_factor * ls_data->lightcalibration_value) / (1000 * 100);
    sensor_active(ls_data->client, 0, 0);
    printk("stk3x1x value: ----ret:%d factor:%d value:%d ir:%d lightcali:%d darkcali:%d lightcaliref:%d----\n", result, ls_data->als_correct_factor, value, ircode, ls_data->lightcalibration_value, ls_data->darkcalibration_value, ls_data->calibration_reference);
    len += sprintf(_buf, "%d\n", result);
    return len;
}
static CLASS_ATTR_RO(lux_value);

static ssize_t lux_rawdata_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(ls_data->client);
    int result = 0, rawvalue = 0, value = 0, ircode = 0;
    ssize_t len = 0, retry = 0;
    char index = 0;
    char buffer[2] = {0};

    if (!ls_data) {
        printk(KERN_ERR "%s ls data is null!\n", __func__);
        return 0;
    }

    sensor_active(ls_data->client, 1, 9600);
    if (sensor->ops->read_len < 2) {
        printk(KERN_ERR "%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
        return 0;
    }
    do {
        value = sensor_read_reg(ls_data->client, STK_FLAG_REG);
        if (value < 0) {
            printk("stk %s read als data flag error, ret=%d\n", __func__, value);
            return value;
        }
        if (value & STK_FLG_ALSDR_MASK) {
            break;
        }
        usleep_range(3000, 4000);
        retry++;
    } while (retry < 1000);
    buffer[0] = sensor->ops->read_reg;
    result = sensor_rx_data(ls_data->client, buffer, sensor->ops->read_len);
    if (result) {
        printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
        return 0;
    }
    rawvalue = (buffer[0] << 8) | buffer[1];
    value = ((rawvalue - ls_data->darkcalibration_value) < 0) ? 0 : (rawvalue - ls_data->darkcalibration_value);
    ls_data->ir_code = stk3x1x_get_ir_reading(ls_data->client, STK_IRS_IT_REDUCE);
    ircode = ls_data->ir_code;
    stk_als_ir_get_corr(value);
    result = (value * ls_data->als_correct_factor * ls_data->lightcalibration_value) / (1000 * 100);
    sensor_active(ls_data->client, 0, 0);

    printk("stk3x1x: ----ret:%d factor:%d value:%d ir:%d lightcali:%d darkcali:%d lightcaliref:%d----\n", result, ls_data->als_correct_factor, value, ircode, ls_data->lightcalibration_value, ls_data->darkcalibration_value, ls_data->calibration_reference);
    len += sprintf(_buf, "x: %d, y: %d, z: %d\n", result, value, ircode);
    return len;
}
static CLASS_ATTR_RO(lux_rawdata);

static int do_calibration(struct sensor_private_data *sensor, int dark)
{
    int ret = -1, i = 0;
    int count = 100, adjvalue = 0, retry = 0;
    uint16_t value = 0;
    char buffer[2] = {0}, oktimes = 0;

    sensor_active(ls_data->client, 1, 9600);
    if (sensor->ops->read_len < 2) {
        dev_err(&ls_data->client->dev, "%s:length is error, len=%d\n", __func__, sensor->ops->read_len);
        return ret;
    }

    for (i = 0; i < count; i++) {
        usleep_range(8000, 10000);
        do {
            value = sensor_read_reg(ls_data->client, STK_FLAG_REG);
            if (value < 0) {
                printk("stk %s read als data flag error, ret=%d\n", __func__, value);
                return value;
            }
            if (value & STK_FLG_ALSDR_MASK) {
                break;
            }
            usleep_range(3000, 4000);
            retry++;
        } while (retry < 1000);
        buffer[0] = sensor->ops->read_reg;
        ret = sensor_rx_data(ls_data->client, buffer, sensor->ops->read_len);
        if (ret) {
            printk(KERN_ERR "%s:sensor read data fail times: %d\n", __func__, i);
        } else {
            value = (buffer[0] << 8) | buffer[1];
            adjvalue += value;
            printk("----adjvalue: %d, value: %d\n", adjvalue, value);
            value = 0;
            oktimes++;
        }
    }

    if (!oktimes) {
        printk(KERN_ERR "%s:can not read sensor read data when calibration\n", __func__);
        return ret;
    }

    if (!dark) {
        adjvalue = (adjvalue / oktimes) ? (adjvalue / oktimes) : ls_data->calibration_reference;
        ls_data->lightcalibration_value = (ls_data->calibration_reference * 100) / adjvalue;
        printk("times: %d, adjvalue: %d, lightcalibration value: %d\n", oktimes, adjvalue, ls_data->lightcalibration_value);

        ret = light3x1x_calibration_data_write(&ls_data->lightcalibration_value);
        if (ret) {
            printk(KERN_ERR "%s wirte calibration fail!\n", __func__);
            return ret;
        }
    } else {
        adjvalue = adjvalue / oktimes;
        ls_data->darkcalibration_value = adjvalue;
        printk("times: %d, adjvalue: %d, darkcalibration value: %d\n", oktimes, adjvalue, ls_data->darkcalibration_value);

        ret = light3x1x_darkcalibration_data_write(&ls_data->darkcalibration_value);
        if (ret) {
            printk(KERN_ERR "%s wirte dark calibration fail!\n", __func__);
            return ret;
        }
    }
    sensor_active(ls_data->client, 0, 0);

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
    if (light3x1x_calibration_data_read(&value)) {
        printk(KERN_ERR "read light stk3x1x calibration error!\n");
        value = ls_data->lightcalibration_value;
    }
    if (light3x1x_darkcalibration_data_read(&dvalue)) {
        printk(KERN_ERR "read light stk3x1x dark calibration error!\n");
        dvalue = ls_data->darkcalibration_value;
    }
    len += sprintf(_buf, "%d  %d\n", dvalue, value);
    return len;
}

static ssize_t lux_calibration_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(ls_data->client);
    int value = 0, dark = 0;
    uint16_t zero = 0, defcali = 100;
    int ret = 1, pre_status;

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
        light3x1x_calibration_data_write(&defcali);
        ls_data->lightcalibration_value = defcali;
        light3x1x_darkcalibration_data_write(&zero);
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
    struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);
    int ret = 0;

    printk("stk %s init ...\n", __func__);
    ret = sensor->ops->active(client, 0, 0);
    if (ret) {
        dev_err(&client->dev, "%s:sensor active fail\n", __func__);
        return ret;
    }
    sensor->status_cur = SENSOR_OFF;

    ret = sensor_write_reg(client, STK_SW_RESET_REG, 0x0);
    if (ret < 0) {
        printk(KERN_ERR "stk %s i2c error line:%d\n", __func__, __LINE__);
        goto EXIT_ERR;
    }
    usleep_range(13000, 15000);
    ret = stk3x1x_check_pid(client);
    if (ret < 0) {
        printk(KERN_ERR "stk %s i2c error line:%d\n", __func__, __LINE__);
        goto EXIT_ERR;
    }

    ret = sensor_write_reg(client, STK_STATE_REG, stk3x1x_pfdata.state_reg);
    if (ret < 0) {
        printk(KERN_ERR "stk %s i2c error line:%d\n", __func__, __LINE__);
        goto EXIT_ERR;
    }

    ret = sensor_write_reg(client, STK_ALSCTRL_REG, stk3x1x_pfdata.alsctrl_reg);
    if (ret < 0) {
        printk(KERN_ERR "stk %s i2c error line:%d\n", __func__, __LINE__);
        goto EXIT_ERR;
    }

#ifndef STK_POLL_ALS
    value = STK_INT_REG;
    ret = sensor_rx_data(client, value, 1);
    if (ret) {
        printk(KERN_ERR "%s:line=%d,error=%d\n", __func__, __LINE__, ret);
        return ret;
    }

    value |= STK_INT_ALS;
    ret = sensor_write_reg(client, STK_INT_REG, value);
    if (ret < 0) {
        printk(KERN_ERR "stk %s i2c error line:%d\n", __func__, __LINE__);
        goto EXIT_ERR;
    }
#endif
    ret = light3x1x_calibration_data_read(&ls_data->lightcalibration_value);
    if (ret) {
        ls_data->lightcalibration_value = 100;
        dev_err(&client->dev, "Fail to get light3x1x calibration data!!! use default: %d\n", ls_data->lightcalibration_value);
    }

    ret = light3x1x_darkcalibration_data_read(&ls_data->darkcalibration_value);
    if (ret) {
        ls_data->darkcalibration_value = 0;
        dev_err(&client->dev, "Fail to get light3x1x darkcalibration data!!! use default: %d\n", ls_data->darkcalibration_value);
    }

    ls_data->als_code_last = 0;
    ls_data->client = client;
    printk("stk %s init successful \n", __func__);
    return 0;

EXIT_ERR:
    printk(KERN_ERR "stk init fail dev: %d\n", ret);
    return ret;
}

static int sensor_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
    int result = 0, rawvalue = 0, ircode = 0;
    ssize_t retry = 0;
    uint32_t value = 0;
    unsigned char buffer[2] = {0};
    char index = 0;

    if (sensor->ops->read_len < 2) {
        printk(KERN_ERR "%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
        return -1;
    }

    do {
        value = sensor_read_reg(ls_data->client, STK_FLAG_REG);
        if (value < 0) {
            printk("stk %s read als data flag error, ret=%d\n", __func__, value);
            return value;
        }
        if (value & STK_FLG_ALSDR_MASK) {
            break;
        }
        usleep_range(3000, 4000);
        retry++;
    } while (retry < 1000);
    buffer[0] = sensor->ops->read_reg;
    result = sensor_rx_data(ls_data->client, buffer, sensor->ops->read_len);
    if (result) {
        printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
        return 0;
    }
    rawvalue = (buffer[0] << 8) | buffer[1];
    value = ((rawvalue - ls_data->darkcalibration_value) < 0) ? 0 : (rawvalue - ls_data->darkcalibration_value);
    ls_data->ir_code = stk3x1x_get_ir_reading(ls_data->client, STK_IRS_IT_REDUCE);
    ircode = ls_data->ir_code;
    stk_als_ir_get_corr(value);
    result = (value * ls_data->als_correct_factor * ls_data->lightcalibration_value) / (1000 * 100);
    index = light_report_abs_value(sensor->input_dev, result);

    if (sensor->pdata->irq_enable && sensor->ops->int_status_reg) {
        value = sensor_read_reg(client, sensor->ops->int_status_reg);
        if (value & STK_FLAG_ALSINT) {
            value &= ~STK_FLAG_ALSINT;
            result = sensor_write_reg(client, sensor->ops->int_status_reg, value);
            if (result) {
                dev_err(&client->dev, "%s:write status reg error\n", __func__);
                return result;
            }
        }
    }

    return result;
}

struct sensor_operate light_stk3x1x_ops = {
    .name               = "ls_stk3x1x",
    .type               = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c             = LIGHT_ID_STK3x1x,     //i2c id number
    .read_reg           = STK_DATA1_ALS_REG,    //read data
    .read_len           = 2,                    //data length
    .id_reg             = SENSOR_UNKNOW_DATA,   //read device id from this register
    .id_data            = SENSOR_UNKNOW_DATA,   //device id
    .precision          = 16,                   //16 bits
    .ctrl_reg           = STK_STATE_REG,        //enable or disable
    .int_status_reg     = SENSOR_UNKNOW_DATA,   //intterupt status register
    .range              = {100, 65535},         //range
    .brightness         = {10, 255},            //brightness
    .trig               = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
    .active             = sensor_active,
    .init               = sensor_init,
    .report             = sensor_report_value,
};
/****************operate according to sensor chip:end************/

static int light_stk3x1x_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    int ret = 0;

    ls_data = kzalloc(sizeof(struct stk3x1x_data), GFP_KERNEL);
    if (!ls_data) {
        printk(KERN_ERR "%s: failed to allocate stk3x1x_data\n", __func__);
        return -ENOMEM;
    }
    ls_data->calibration_reference = 500;
    ls_data->lsensor_class = class_create(THIS_MODULE, client->name);
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_value);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light3x1x class value.\n");
    }
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_calibration);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light3x1x class calibration!\n");
    }
    ret = class_create_file(ls_data->lsensor_class, &class_attr_lux_rawdata);
    if (ret) {
        dev_err(&client->dev, "Fail to create class light3x1x class raw!\n");
    }
    ret = sensor_register_device(client, NULL, devid, &light_stk3x1x_ops);
    return ret;
}

static int light_stk3x1x_remove(struct i2c_client *client)
{
    return sensor_unregister_device(client, NULL, &light_stk3x1x_ops);
}

static const struct i2c_device_id light_stk3x1x_id[] = {
    {"ls_stk3x1x", LIGHT_ID_STK3x1x},
    {}
};

static struct i2c_driver light_stk3x1x_driver = {
    .probe = light_stk3x1x_probe,
    .remove = light_stk3x1x_remove,
    .shutdown = sensor_shutdown,
    .id_table = light_stk3x1x_id,
    .driver = {
        .name = "light_stk3x1x",
#ifdef CONFIG_PM
        .pm = &sensor_pm_ops,
#endif
    },
};

static int __init light_stk3x1x_init(void)
{
    return i2c_add_driver(&light_stk3x1x_driver);;
}
late_initcall(light_stk3x1x_init);

static void __exit light_stk3x1x_exit(void)
{
    i2c_del_driver(&light_stk3x1x_driver);
}
module_exit(light_stk3x1x_exit);

MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
