/*
 * Papyrus epaper power control HAL
 *
 *  Copyright (C) 2009 Dimitar Dimitrov, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 * TPS6518x power control is facilitated using I2C control and WAKEUP GPIO
 * pin control. The other VCC GPIO Papyrus' signals must be tied to ground.
 *
 * TODO:
 *  - Instead of polling, use interrupts to signal power up/down
 *    acknowledge.
 */
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/suspend.h>
#include <linux/htfy_dbg.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
#include <asm/gpio.h>
#else
#include <linux/gpio.h>
#endif

#include "htfy_pmic.h"

#define SY7636A_I2C_NAME                      "sy7636a"
#define INVALID_GPIO                          -1
#define PAPYRUS_VCOM_MAX_MV                   0
#define PAPYRUS_VCOM_MIN_MV                   -5000
#define PAPYRUS_TEMP_READ_TIME_MS             10
#define PAPYRUS_EEPROM_DELAY_MS               50
#define Operation_Mode_Control                0x0
#define OMC_OnOff                             0x80
#define OMC_VcomCtl                           0x40
#define OMC_VDDH_En                           0x10
#define OMC_VPOS_En                           0x04
#define OMC_VNEG_En                           0x02
#define OMC_VCOM_En                           0x01
#define VCOM_Adjustment_Control_1             0x1
#define VCOM_Adjustment_Control_2             0x2
#define VLDO_Voltage_Adjustment_Control       0x3
#define Power_On_Delay_Time                   0x6
#define Fault_Flag                            0x7
#define Thermistor_Readout                    0x8

#define PAPYRUS_MV_TO_VCOMREG_SY7636A(MV)     (MV/10)
#define PAPYRUS_V3P3OFF_DELAY_MS              10  //100

#if 0
#define sy7636a_printk(fmt, args...)          printk(KERN_INFO "[sy7636a] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define sy7636a_printk(fmt, args...)
#endif

struct papyrus_sess {
    struct i2c_adapter *adap;
    struct i2c_client *client;
    uint8_t vadj;
    uint8_t vcom1;
    uint8_t vcom2;

    /* Custom power up/down sequence settings */
    unsigned int v3p3off_time_ms;
    int epd_type;
    int power_fault_cnt;
    struct gpio_desc *pwr_up_pin;
    struct gpio_desc *error_pin;
    struct gpio_desc *vcom_ctl_pin;
    struct gpio_desc *vdd_pin;
	struct gpio_desc *pmicon;
    int poweron_active_high;
    int vcomctl_active_high;
    int vdd_active_high;
    bool need_init;
};

struct papyrus_hw_state {
    uint8_t tmst_value;
};

struct sy7636a_t {
    uint8_t data[3];
    int16_t temp;
};

extern int pmic_id;
static struct class *pmu_class;
static struct kobject *sy7636a_kobj;
struct pmic_sess pmic_sess_data;

extern bool fb_is_power_off(void);

int sy7636a_vcom_get(void);

static int papyrus_hw_setreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t val)
{
    int stat;
    uint8_t txbuf[2] = { regaddr, val };
    struct i2c_msg msgs[] = {
        {
            .addr = sess->client->addr,
            .flags = 0,
            .len = 2,
            .buf = txbuf,
        }
    };

    stat = i2c_transfer(sess->adap, msgs, ARRAY_SIZE(msgs));

    if (stat < 0) {
        pr_err("papyrus: I2C send error: %d\n", stat);
    } else if (stat != ARRAY_SIZE(msgs)) {
        pr_err("papyrus: I2C send N mismatch: %d\n", stat);
        stat = -EIO;
    } else {
        stat = 0;
    }

    return stat;
}

static int papyrus_hw_getreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t *val)
{
    int stat;
    struct i2c_msg msgs[] = {
        {
            .addr = sess->client->addr,
            .flags = 0,
            .len = 1,
            .buf = &regaddr,
        },
        {
            .addr = sess->client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = val,
        }
    };

    stat = i2c_transfer(sess->adap, msgs, ARRAY_SIZE(msgs));
    if (stat < 0) {
        pr_err("papyrus: I2C read error: %d\n", stat);
        pr_err("Papyrus i2c addr %x %s\n", sess->client->addr, __FILE__);
    } else if (stat != ARRAY_SIZE(msgs)) {
        pr_err("papyrus: I2C read N mismatch: %d\n", stat);
        stat = -EIO;
    } else {
        stat = 0;
    }

    return stat;
}

static int papyrus_hw_reset(struct papyrus_sess *sess)
{
    uint8_t reg_addr = Operation_Mode_Control;
    uint8_t reg_data;

    sess->need_init = true;
	gpiod_direction_output(sess->vdd_pin, 0);
	gpiod_direction_output(sess->vcom_ctl_pin, 0);
    papyrus_hw_getreg(sess, reg_addr, &reg_data);
    reg_data |= OMC_OnOff;
    reg_data |= OMC_VcomCtl;
    papyrus_hw_setreg(sess, reg_addr, reg_data);

    return 0;
}

static ssize_t store_turnon_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint8_t reg_addr = Operation_Mode_Control;
    uint8_t reg_data;
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;
    struct papyrus_sess *sess = psess->drvpar;
    papyrus_hw_reset(sess);
    papyrus_hw_getreg(sess, reg_addr, &reg_data);
    reg_data |= OMC_OnOff;
    papyrus_hw_setreg(sess, reg_addr, reg_data);

    return count;
}

static ssize_t store_turnoff_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint8_t reg_addr = Operation_Mode_Control;
    uint8_t reg_data;
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;
    struct papyrus_sess *sess = psess->drvpar;
    papyrus_hw_getreg(sess, reg_addr, &reg_data);
    reg_data &= 0xff - OMC_OnOff;
    papyrus_hw_setreg(sess, reg_addr, reg_data);

    return count;
}

static ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    int i = 0;
    int count = 0;
    u8 reg_data = 0;
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;
    struct papyrus_sess *sess = psess->drvpar;
    for (i = 0; i < 9; i++) {
        ret = papyrus_hw_getreg(sess, i, &reg_data);
        if (ret < 0) {
            break;
        } else {
            count += sprintf(buf + count, "[%x] = (%x)\n", 0x00 + i, reg_data);
        }
    }

    return count;
}
static ssize_t store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int addr, cmd, ret;
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;
    struct papyrus_sess *sess = psess->drvpar;
    if (2 != sscanf(buf, "%x %x", &addr, &cmd)) {
        printk(KERN_EMERG "SY7636A Store reg fail\n");
        return count;
    }

    ret = papyrus_hw_setreg(sess, addr, cmd);

    return count;
}
static DEVICE_ATTR(da_xe_turnon_power, S_IWUSR, NULL, store_turnon_power);
static DEVICE_ATTR(da_xe_turnoff_power, S_IWUSR, NULL, store_turnoff_power);
static DEVICE_ATTR(da_ce_reg, S_IWUSR | S_IRUGO, show_reg, store_reg);

static struct attribute *sy7636a_attributes[] = {
    &dev_attr_da_xe_turnon_power.attr,
    &dev_attr_da_xe_turnoff_power.attr,
    &dev_attr_da_ce_reg.attr,
    NULL
};

static const struct attribute_group sy7636a_attr_group = {
    .attrs = sy7636a_attributes,
};

static int papyrus_hw_arg_init(struct papyrus_sess *sess)
{
    uint8_t reg_addr = Operation_Mode_Control;
    uint8_t reg_data;

    sy7636a_printk("papyrus_hw_arg_init addr:0x%2x\n", sess->client->addr);

	gpiod_direction_output(sess->vdd_pin, 1);
    papyrus_hw_getreg(sess, reg_addr, &reg_data);
    reg_data |= OMC_OnOff;
    reg_data |= OMC_VcomCtl;
    papyrus_hw_setreg(sess, reg_addr, reg_data);
    reg_addr = VLDO_Voltage_Adjustment_Control;
    reg_data = 0x60;
    papyrus_hw_setreg(sess, reg_addr, reg_data);
    reg_addr = Power_On_Delay_Time;
    reg_data = 0xaa;
    papyrus_hw_setreg(sess, reg_addr, reg_data);
    papyrus_hw_setreg(sess, VCOM_Adjustment_Control_1, sess->vcom1);
    papyrus_hw_setreg(sess, VCOM_Adjustment_Control_2, (sess->vcom2&0x01)<<7);

    sy7636a_printk("sess->vcom1 = 0x%x,sess->vcom2 = 0x%x\n", sess->vcom1, sess->vcom2);

    sess->need_init = false;
    return 0;
}

static void papyrus_hw_send_powerup(struct papyrus_sess *sess)
{
    uint8_t reg_addr = Operation_Mode_Control;
    uint8_t reg_data;
    int stat;
    sy7636a_printk("papyrus_hw_send_powerup need_init:%d\n", sess->need_init);

    if (sess->need_init) {
        papyrus_hw_arg_init(sess);
    } else {
		gpiod_direction_output(sess->vdd_pin, 1);
        reg_data |= OMC_OnOff;
        reg_data |= OMC_VcomCtl;
        stat = papyrus_hw_setreg(sess, reg_addr, reg_data);
        if (stat) {
            pr_err("papyrus: I2C error at pwr down: %d\n", stat);
        }
    }

	gpiod_direction_output(sess->vcom_ctl_pin, 1);
}

static void papyrus_hw_send_powerdown(struct papyrus_sess *sess)
{
    uint8_t reg_addr = Operation_Mode_Control;
    uint8_t reg_data;
    int stat;

    sy7636a_printk("papyrus_hw_send_powerdown\n");

    reg_data = 0x00;
    stat = papyrus_hw_setreg(sess, reg_addr, reg_data);
    if (stat) {
        pr_err("papyrus: I2C error at pwr down: %d\n", stat);
    }

	gpiod_direction_output(sess->vcom_ctl_pin, 0);
    msleep(sess->v3p3off_time_ms);
	gpiod_direction_output(sess->vdd_pin, 0);
}

static int papyrus_hw_read_temperature(struct pmic_sess *pmsess, int *t)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    int stat;
    int ntries = 50;
    uint8_t tb;
    int temp;
    uint8_t reg_data;

    stat = papyrus_hw_getreg(sess, Operation_Mode_Control, &reg_data);
    //msleep(PAPYRUS_TEMP_READ_TIME_MS);
    stat = papyrus_hw_setreg(sess, Operation_Mode_Control, (reg_data | OMC_OnOff));
    do {
        stat = papyrus_hw_getreg(sess, Thermistor_Readout, &tb);
    } while (!stat && ntries-- && ((tb  == 0x61) || (tb  == 0xef)));

    stat = papyrus_hw_setreg(sess, Operation_Mode_Control, reg_data);
    temp = (tb > 127) ? ((int)tb - 256) : (int)tb;

    if (stat) {
        return stat;
    }
    msleep(PAPYRUS_TEMP_READ_TIME_MS);
    *t = (int)(int8_t)temp;

    printk("current temperature is %d\n", *t);

    return stat;
}
extern int sy7636a_temperature;
static ssize_t sy7636a_temperature_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;
    //temperature = 25;
    ssize_t len = 0;

    papyrus_hw_read_temperature(psess, &sy7636a_temperature);
    len += sprintf(_buf, "%d:\n", sy7636a_temperature);
    return len;
}

static ssize_t sy7636a_temperature_store(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
	int ret;

	//sscanf(buf, "%d", &sy7636a_temperature);
	 ret = kstrtoint(buf, 10, &sy7636a_temperature);
	 if (ret) {
		 pr_err("%s: kstrtoint error return %d\n", __func__, ret);
		 return ret;
	 }
	return _count;
}
static CLASS_ATTR_RW(sy7636a_temperature);

static int papyrus_hw_init(struct papyrus_sess *sess, const char *chip_id)
{
	gpiod_direction_input(sess->error_pin);
	gpiod_direction_output(sess->vcom_ctl_pin, 0);
    gpiod_direction_output(sess->pwr_up_pin, 1);
    msleep(PAPYRUS_EEPROM_DELAY_MS);
    gpiod_direction_output(sess->vdd_pin, 1);
	msleep(10);
    gpiod_direction_output(sess->pmicon, 1);

    pmu_class = class_create(THIS_MODULE, "pmu");
    if (class_create_file(pmu_class, &class_attr_sy7636a_temperature)) {
        pr_err("sy7636a: Fail to create class temperature!\n");
    }

    return 0;
}

static void papyrus_hw_cleanup(struct papyrus_sess *sess)
{
    if (!IS_ERR_OR_NULL(sess->vcom_ctl_pin)) {
        gpiod_put(sess->vcom_ctl_pin);
    }
    if (!IS_ERR_OR_NULL(sess->vdd_pin)) {
        gpiod_put(sess->vdd_pin);
    }
    if (!IS_ERR_OR_NULL(sess->pwr_up_pin)) {
        gpiod_put(sess->pwr_up_pin);
    }
    if (!IS_ERR_OR_NULL(sess->error_pin)) {
        gpiod_put(sess->error_pin);
    }

    i2c_put_adapter(sess->adap);
}

static void papyrus_hw_power_req(struct pmic_sess *pmsess, bool up)
{
    struct papyrus_sess *sess = pmsess->drvpar;

    if (up) {
        papyrus_hw_send_powerup(sess);
    } else {
        papyrus_hw_send_powerdown(sess);
    }
    return;
}

static bool papyrus_hw_power_ack(struct pmic_sess *pmsess)
{
    return true;
}

static int papyrus_set_enable(struct pmic_sess *pmsess, int enable)
{
    return 0;
}

static int papyrus_set_vcom_voltage(struct pmic_sess *pmsess, int vcom_mv)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG_SY7636A(vcom_mv) & 0x00FF);
    sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG_SY7636A(vcom_mv) & 0x0100) >> 8);
    return 0;
}

static int papyrus_set_vcom1(struct pmic_sess *pmsess, uint8_t vcom1)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->vcom1 = vcom1;
    return 0;
}

static int papyrus_set_vcom2(struct pmic_sess *pmsess, uint8_t vcom2)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->vcom2 = vcom2;
    return 0;
}

static int papyrus_set_vadj(struct pmic_sess *pmsess, uint8_t vadj)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->vadj = vadj;
    return 0;
}

static int papyrus_set_int_en1(struct pmic_sess *pmsess, uint8_t int_en1)
{
    return 0;
}

static int papyrus_set_int_en2(struct pmic_sess *pmsess, uint8_t int_en2)
{
    return 0;
}

static int papyrus_set_upseq0(struct pmic_sess *pmsess, uint8_t upseq0)
{
    return 0;
}

static int papyrus_set_upseq1(struct pmic_sess *pmsess, uint8_t upseq1)
{
    return 0;
}

static int papyrus_set_dwnseq0(struct pmic_sess *pmsess, uint8_t dwnseq0)
{
    return 0;
}

static int papyrus_set_dwnseq1(struct pmic_sess *pmsess, uint8_t dwnseq1)
{
    return 0;
}

static int papyrus_set_tmst1(struct pmic_sess *pmsess, uint8_t tmst1)
{
    return 0;
}

static int papyrus_set_tmst2(struct pmic_sess *pmsess, uint8_t tmst2)
{
    return 0;
}

static int papyrus_vcom_switch(struct pmic_sess *pmsess, bool state)
{
    sy7636a_printk(" papyrus_vcom_switch%d\n", state);

    return 0;
}

int sy7636a_reinit(void *priv, int epd_type)
{
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    sess->epd_type = epd_type;
    sy7636a_printk(" sy7636a_reinit\n");

    return papyrus_hw_arg_init(sess);
}

static int papyrus_pm_sleep(struct pmic_sess *sess)
{
    struct papyrus_sess *s = sess->drvpar;

    sy7636a_printk(" enter sleep,fb off=%d\n", fb_is_power_off());
	gpiod_direction_output(s->vcom_ctl_pin, 0);
    s->need_init = fb_is_power_off();

    return 0;
}

static int papyrus_pm_resume(struct pmic_sess *sess)
{
    struct papyrus_sess *s = sess->drvpar;

    sy7636a_printk("%s re-init=%d!\n", __func__, s->need_init);

    return 0;
}

static int papyrus_probe(struct pmic_sess *pmsess, struct i2c_client *client)
{
    struct papyrus_sess *sess;
    int stat;

    if (pmic_id == 0x6518) {
        return 0;
    }
    printk("papyrus_probe_sy7636a \n");

    sess = kzalloc(sizeof(*sess), GFP_KERNEL);
    if (!sess) {
        pr_err("%s:%d: kzalloc failed\n", __func__, __LINE__);
        return -ENOMEM;
    }
    sess->client = client;
    sess->adap = client->adapter;
    sess->v3p3off_time_ms = PAPYRUS_V3P3OFF_DELAY_MS;

    sess->vcom_ctl_pin = devm_gpiod_get_optional(&client->dev, "vcomctl", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(sess->vcom_ctl_pin)) {
        pr_err("sy7636a: failed to find vcomctl pin!\n");
		kfree(sess);
		return -ENOMEM;
	}
    sess->vdd_pin = devm_gpiod_get_optional(&client->dev, "vdd", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(sess->vdd_pin)) {
        pr_err("sy7636a: failed to find vdd pin!\n");
		kfree(sess);
		return -ENOMEM;
    }
    sess->pwr_up_pin = devm_gpiod_get_optional(&client->dev, "powerup", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(sess->pwr_up_pin)) {
        pr_err("sy7636a: failed to find powerup pin!\n");
		kfree(sess);
		return -ENOMEM;
    }
    sess->error_pin = devm_gpiod_get_optional(&client->dev, "error", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(sess->error_pin)) {
        pr_err("sy7636a: failed to find error pin!\n");
		kfree(sess);
		return -ENOMEM;
    }
	sess->pmicon = devm_gpiod_get_optional(&client->dev, "pmicon", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(sess->pmicon)) {
        printk("sy7636a: failed to find pmicon pin!\n");
		//kfree(sess);
		//return -ENOMEM;
    }

    stat = papyrus_hw_init(sess, 0);
    if (stat) {
		pr_err("sy7636a: failed to init!\n");
		kfree(sess);
        return stat;
    }
	printk("papyrus_probe sy7636a:%d \n", pmsess->revision);
    pmsess->drvpar = sess;

    return 0;
}

static void papyrus_remove(struct pmic_sess *pmsess)
{
    struct papyrus_sess *sess = pmsess->drvpar;

    papyrus_hw_cleanup(sess);

    kfree(sess);
    pmsess->drvpar = 0;
}

const struct pmic_driver pmic_driver_sy7636a_i2c = {
    .id = "sy7636a-i2c",
    .vcom_min = PAPYRUS_VCOM_MIN_MV,
    .vcom_max = PAPYRUS_VCOM_MAX_MV,
    .vcom_step = 10,
    .hw_read_temperature = papyrus_hw_read_temperature,
    .hw_power_ack = papyrus_hw_power_ack,
    .hw_power_req = papyrus_hw_power_req,
    .set_enable = papyrus_set_enable,
    .set_vcom_voltage = papyrus_set_vcom_voltage,
    .set_vcom1 = papyrus_set_vcom1,
    .set_vcom2 = papyrus_set_vcom2,
    .set_vadj = papyrus_set_vadj,
    .set_int_en1 = papyrus_set_int_en1,
    .set_int_en2 = papyrus_set_int_en2,
    .set_upseq0 = papyrus_set_upseq0,
    .set_upseq1 = papyrus_set_upseq1,
    .set_dwnseq0 = papyrus_set_dwnseq0,
    .set_dwnseq1 = papyrus_set_dwnseq1,
    .set_tmst1 = papyrus_set_tmst1,
    .set_tmst2 = papyrus_set_tmst2,
    .hw_vcom_switch = papyrus_vcom_switch,
    .hw_init = papyrus_probe,
    .hw_cleanup = papyrus_remove,
    .hw_pm_sleep = papyrus_pm_sleep,
    .hw_pm_resume = papyrus_pm_resume,
};

/*--------------------------------------------------------------------------------------------------------------
=========>HTFYUN pmic opts
--------------------------------------------------------------------------------------------------------------*/
int sy7636a_vcom_get(void)
{
    struct pmic_sess tpmic_sess_data = pmic_sess_data;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    uint8_t rev_val1 = 0, rev_val2 = 0;
    int stat = 0;
    int read_vcom_mv = 0;

    sy7636a_printk("sy7636a_vcom_get enter.\n");
    if (!tpmic_sess_data.is_inited) {
        return -1;
    }

    read_vcom_mv = 0;
    stat |= papyrus_hw_getreg(sess, VCOM_Adjustment_Control_1, &rev_val1);
    read_vcom_mv = rev_val1 * 10;
    stat |= papyrus_hw_getreg(sess, VCOM_Adjustment_Control_2, &rev_val2);
    read_vcom_mv += ((rev_val2 & 0x0080)<<1)*10;
    sy7636a_printk("read_vcom_mv = %d\n", read_vcom_mv);

    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
    }

    return read_vcom_mv;
}

int sy7636a_vcom_set(int vcom_mv)
{
    struct pmic_sess tpmic_sess_data = pmic_sess_data;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    //uint8_t rev_val1 = 0, rev_val2 = 0;
    int stat = 0;
    int read_vcom_mv = 0;

    sy7636a_printk("sy7636a_vcom_set enter.\n");
    if (!tpmic_sess_data.is_inited) {
        sy7636a_printk("sy7636a_vcom_set enter,vcom_mv=%d,but Not Init!!\n", vcom_mv);
        return -1;
    }
    msleep(10);
    read_vcom_mv = sy7636a_vcom_get();
    // Set vcom voltage
    pmic_driver_sy7636a_i2c.set_vcom_voltage((struct pmic_sess *)&tpmic_sess_data, vcom_mv);
    stat |= papyrus_hw_setreg(sess, VCOM_Adjustment_Control_1, sess->vcom1);
    stat |= papyrus_hw_setreg(sess, VCOM_Adjustment_Control_2, (sess->vcom2&0x01)<<7);
    sy7636a_printk("sess->vcom1 = 0x%x\n", sess->vcom1);

#if 0
    read_vcom_mv = 0;
    stat |= papyrus_hw_getreg(sess, VCOM_Adjustment_Control_1, &rev_val1);
    read_vcom_mv = rev_val1 * 10;
    stat |= papyrus_hw_getreg(sess, VCOM_Adjustment_Control_2, &rev_val2);
	read_vcom_mv += ((rev_val2 & 0x0080)<<1)*10;
    //sy7636a_printk("read_vcom_mv = %d\n", read_vcom_mv);
    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
    }
#else
    printk("%s:set vcom=%d,read vcom0=%d, vcom1=%d\n", __func__,
        vcom_mv, read_vcom_mv, sy7636a_vcom_get());
#endif
    return 0;
}

int sy7636a_set_vcom_voltage(void *priv, int vcom_mv)
{
    if (vcom_mv < 0 || vcom_mv > 5002) {
        printk("tps: err vcom value, vcom value shoule be 0~5110\n");
        return -1;
    }

    return sy7636a_vcom_set(vcom_mv);
}

int sy7636a_power_on(void *priv)
{
    if (pmic_sess_data.is_inited) {
        pmic_driver_sy7636a_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data, 1);
    }
    return 0;
}

int sy7636a_power_down(void *priv)
{
    if (pmic_sess_data.is_inited) {
        pmic_driver_sy7636a_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data, 0);
    }
    return 0;
}

int sy7636a_power_check(void *priv, int timeout)
{
    int stat;
    uint8_t ints1, pwrg;
    int err_pin_value = -1;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;

    err_pin_value = gpiod_get_value(sess->error_pin);
    if (err_pin_value == 1) {
        return 1;
    }
    if (!timeout) {
        return 0;
    }

    stat |= papyrus_hw_getreg(sess, Fault_Flag, &pwrg);
    if (stat) {
        pr_err("papyrus: power_check I2C error: %d,reset pmic!\n", stat);

        // 20220719: sometime reboot will happen,need to reset papyrus.
        gpiod_direction_output(sess->pwr_up_pin, 0);
        msleep(20);
        gpiod_direction_output(sess->pwr_up_pin, 1);
        msleep(PAPYRUS_EEPROM_DELAY_MS);

        papyrus_hw_reset(sess);
        papyrus_hw_send_powerup(sess);
        return -1;
    } else {
        if ((pwrg & 0X00000001) == 0X00000001) {
            return 1;
        }
        if (timeout) {
            ints1 = pwrg & 0X1E;
            pr_err("papyrus: Power_Fault,fcnt=%d,PG=0X%02X\n", sess->power_fault_cnt, pwrg);
            sess->power_fault_cnt++;

            if (ints1) {
                papyrus_hw_reset(sess);
                papyrus_hw_send_powerup(sess);
            }
        }
    }
    return 0;
}

int sy7636a_temperature_get(void *priv, int *temp)
{
    if (pmic_sess_data.is_inited) {
        return pmic_driver_sy7636a_i2c.hw_read_temperature((struct pmic_sess *)&pmic_sess_data, temp);
    } else {
        return 0;
    }
}
/*--------------------------------------------------------------------------------------------------------------*/

static int sy7636a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    sy7636a_printk("I2C addr:%x\n", client->addr);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("I2C check functionality failed.");
        return -ENODEV;
    }

    if (0 != pmic_driver_sy7636a_i2c.hw_init((struct pmic_sess *)&pmic_sess_data, client)) {
        printk("pmic_driver_sy7636a_i2c hw_init failed.");
        return -ENODEV;
    }

    pmic_sess_data.is_inited = 1;
    sy7636a_kobj = kobject_create_and_add("sy7636a", NULL);
    if (sy7636a_kobj) {
        ret = sysfs_create_group(sy7636a_kobj, &sy7636a_attr_group);
    }

    sy7636a_printk("sy7636a_probe ok.\n");

    return 0;
}

static int sy7636a_remove(struct i2c_client *client)
{
    pmic_driver_sy7636a_i2c.hw_cleanup((struct pmic_sess *)&pmic_sess_data);
    memset(&pmic_sess_data, 0, sizeof(struct pmic_sess));
    return 0;
}

static int sy7636a_suspend(struct device *dev/*struct i2c_client *client*/)
{
    return pmic_driver_sy7636a_i2c.hw_pm_sleep((struct pmic_sess *)&pmic_sess_data);
}

static int sy7636a_resume(struct device *dev/*struct i2c_client *client*/)
{
    return pmic_driver_sy7636a_i2c.hw_pm_resume((struct pmic_sess *)&pmic_sess_data);
}

static const struct i2c_device_id sy7636a_id[] = {
    { SY7636A_I2C_NAME, 0 },
    { }
};

static const struct of_device_id sy7636a_dt_ids[] = {
    { .compatible = "sy7636a", },
    { /* sentinel */ }
};

static const struct dev_pm_ops sy7636a_pm_ops = {
    .resume = sy7636a_resume,
    .suspend = sy7636a_suspend,
};
MODULE_DEVICE_TABLE(of, sy7636a_dt_ids);

static struct i2c_driver sy7636a_driver = {
    .probe = sy7636a_probe,
    .remove = sy7636a_remove,
    .id_table = sy7636a_id,
    .driver = {
        .of_match_table = sy7636a_dt_ids,
        .name = SY7636A_I2C_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_PM
        .pm = &sy7636a_pm_ops,
#endif
    },
};

static int __init sy7636a_init(void)
{
    int ret;

    ret = i2c_add_driver(&sy7636a_driver);
    if (ret) {
        printk("Register sy7636a driver failed.\n");
    }

    return ret;
}

static void __exit sy7636a_exit(void)
{
    return i2c_del_driver(&sy7636a_driver);
}

fs_initcall(sy7636a_init);
module_exit(sy7636a_exit);

MODULE_DESCRIPTION("ti sy7636a pmic");
MODULE_LICENSE("GPL");
