// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2022 Rockchip Co.,Ltd.
 * Author: Wang Jie <dave.wang@rock-chips.com>
 *
 * Richtek ET7303 Type-C Chip Driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/tcpm.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include "tcpci.h"

#define ET7303_VID         0x6DCF
#define ET7303_PID         0x1711

#define RT1711H_VID        0x29CF
#define RT1711H_PID        0x1711

#define ET7303_RTCTRL8     0x9B

/* Autoidle timeout = (tout * 2 + 1) * 6.4ms */
#define ET7303_RTCTRL8_SET(ck300, ship_off, auto_idle, tout) (((ck300) << 7) | ((ship_off) << 5) | ((auto_idle) << 3) | ((tout) & 0x07))

#define ET7303_RTCTRL11    0x9E

/* I2C timeout = (tout + 1) * 12.5ms */
#define ET7303_RTCTRL11_SET(en, tout) (((en) << 7) | ((tout) & 0x0F))

#define ET7303_RTCTRL13    0xA0
#define ET7303_RTCTRL14    0xA1
#define ET7303_RTCTRL15    0xA2
#define ET7303_RTCTRL16    0xA3

struct et7303_chip {
    struct tcpci_data data;
    struct tcpci *tcpci;
    struct device *dev;
    struct kobject kobj;
};

static int et7303_read16(struct et7303_chip *chip, unsigned int reg, u16 *val)
{
    return regmap_raw_read(chip->data.regmap, reg, val, sizeof(u16));
}

static int et7303_write16(struct et7303_chip *chip, unsigned int reg, u16 val)
{
    return regmap_raw_write(chip->data.regmap, reg, &val, sizeof(u16));
}

static int et7303_read8(struct et7303_chip *chip, unsigned int reg, u8 *val)
{
    return regmap_raw_read(chip->data.regmap, reg, val, sizeof(u8));
}

static int et7303_write8(struct et7303_chip *chip, unsigned int reg, u8 val)
{
    return regmap_raw_write(chip->data.regmap, reg, &val, sizeof(u8));
}

static const struct regmap_config et7303_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xFF, /* 0x80 .. 0xFF are vendor defined */
};

static struct et7303_chip *tdata_to_et7303(struct tcpci_data *tdata)
{
    return container_of(tdata, struct et7303_chip, data);
}

static int et7303_init(struct tcpci *tcpci, struct tcpci_data *tdata)
{
    int ret;
    struct et7303_chip *chip = tdata_to_et7303(tdata);
	printk("ljc:---->%s\n",__func__);

    /* CK 300K from 320K, shipping off, auto_idle enable, tout = 32ms */
    ret = et7303_write8(chip, ET7303_RTCTRL8, ET7303_RTCTRL8_SET(0, 1, 1, 2));
    if (ret < 0) {
        return ret;
    }

    /* I2C reset : (val + 1) * 12.5ms */
    ret = et7303_write8(chip, ET7303_RTCTRL11, ET7303_RTCTRL11_SET(1, 0x0F));
    if (ret < 0) {
        return ret;
    }

    /* tTCPCfilter : (26.7 * val) us */
    ret = et7303_write8(chip, ET7303_RTCTRL14, 0x0F);
    if (ret < 0) {
        return ret;
    }

    /*  tDRP : (51.2 + 6.4 * val) ms */
    ret = et7303_write8(chip, ET7303_RTCTRL15, 0x04);
    if (ret < 0) {
        return ret;
    }

    /* dcSRC.DRP : 33% */
    return et7303_write16(chip, ET7303_RTCTRL16, 330);
}

static int et7303_set_vconn(struct tcpci *tcpci, struct tcpci_data *tdata, bool enable)
{
    struct et7303_chip *chip = tdata_to_et7303(tdata);

    return et7303_write8(chip, ET7303_RTCTRL8, ET7303_RTCTRL8_SET(0, 1, !enable, 2));
}

static int et7303_start_drp_toggling(struct tcpci *tcpci, struct tcpci_data *tdata, enum typec_cc_status cc)
{
    struct et7303_chip *chip = tdata_to_et7303(tdata);
    int ret;
    unsigned int reg = 0;

    switch (cc) {
        default:
        case TYPEC_CC_RP_DEF:
            reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
            break;
        case TYPEC_CC_RP_1_5:
            reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
            break;
        case TYPEC_CC_RP_3_0:
            reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
            break;
    }

    if (cc == TYPEC_CC_RD) {
        reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) | (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
    } else {
        reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) | (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);
    }

    ret = et7303_write8(chip, TCPC_ROLE_CTRL, reg);
    if (ret < 0) {
        return ret;
    }
    usleep_range(500, 1000);

    return 0;
}

static irqreturn_t et7303_irq(int irq, void *dev_id)
{
    int ret;
    u16 alert;
    u8 status;
    struct et7303_chip *chip = dev_id;

    ret = et7303_read16(chip, TCPC_ALERT, &alert);
	printk("ljc:---->%s :%d\n",__func__,ret);
    if (ret < 0) {
        goto out;
    }

    if (alert & TCPC_ALERT_CC_STATUS) {
        ret = et7303_read8(chip, TCPC_CC_STATUS, &status);
		printk("ljc:---->%sret:%d alert:0x%x status:0x%x\n",__func__,ret,alert,status);
        if (ret < 0) {
            goto out;
        }
        /* Clear cc change event triggered by starting toggling */
        if (status & TCPC_CC_STATUS_TOGGLING) {
            et7303_write8(chip, TCPC_ALERT, TCPC_ALERT_CC_STATUS);
        }
    }

out:
    return tcpci_irq(chip->tcpci);
}

static int et7303_sw_reset(struct et7303_chip *chip)
{
    int ret;

    ret = et7303_write8(chip, ET7303_RTCTRL13, 0x01);
    if (ret < 0) {
        return ret;
    }

    usleep_range(1000, 2000);
	//printk("ljc:---->%s\n",__func__);
    return 0;
}

static int et7303_check_revision(struct i2c_client *i2c)
{
    int ret;

    ret = i2c_smbus_read_word_data(i2c, TCPC_VENDOR_ID);
    if (ret < 0) {
        return ret;
    }
    if (ret != ET7303_VID && ret != RT1711H_VID) {
        dev_err(&i2c->dev, "vid is not correct, 0x%04x\n", ret);
        return -ENODEV;
    }
    ret = i2c_smbus_read_word_data(i2c, TCPC_PRODUCT_ID);
    if (ret < 0) {
        return ret;
    }
    if (ret != ET7303_PID && ret != RT1711H_PID) {
        dev_err(&i2c->dev, "pid is not correct, 0x%04x\n", ret);
        return -ENODEV;
    }
    return 0;
}

/* changed tower: create debug sys node. */
struct typec_sysfs_entry {
    struct attribute attr;
    ssize_t (*show)(struct et7303_chip *, char *);
    ssize_t (*store)(struct et7303_chip *, const char *, size_t);
};

static ssize_t typec_direction_show(struct et7303_chip *chip, char *buf)
{
    int ret;
    u8 status;

    ret = et7303_read8(chip, TCPC_CC_STATUS, &status);
    if (ret < 0) {
        printk("read typec state error!\n");
        return sprintf(buf, "%s\n", "err");
    } else {
        if (0x11 == status) {
            printk("CC dir status: %x positive.\n");
            return sprintf(buf, "%s\n", "positive");
        } else if (0x14 == status) {
            printk("CC dir status: %x negative.\n");
            return sprintf(buf, "%s\n", "negative");
        }
    }
    return sprintf(buf, "%s\n", "not support");
}

static ssize_t htdebug_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
    struct et7303_chip *chip = container_of(kobj, struct et7303_chip, kobj);
    struct typec_sysfs_entry *entry = container_of(attr, struct typec_sysfs_entry, attr);
    ssize_t ret = 0;

    if (entry->show) {
        ret = entry->show(chip, buf);
    }

    return ret;
}

static ssize_t htdebug_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
    struct et7303_chip *chip = container_of(kobj, struct et7303_chip, kobj);
    struct typec_sysfs_entry *entry = container_of(attr, struct typec_sysfs_entry, attr);
    ssize_t ret = 0;

    if (entry->store) {
        ret = entry->store(chip, buf, count);
    }

    return ret;
}

static struct typec_sysfs_entry typecdir_entry = {
    .attr = { .name = "typecdir", .mode = 0444 },
    .show = typec_direction_show,
};

static struct attribute *htdebug_attrs[] = {
    &typecdir_entry.attr,
    NULL,
};

static const struct sysfs_ops htdebug_ops = {
    .show = &htdebug_show,
    .store = &htdebug_store,
};

static struct kobj_type htdebug_ktype = {
    .default_attrs = htdebug_attrs,
    .sysfs_ops = &htdebug_ops,
};
/* changed end. */

static int et7303_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
    int ret;
    struct et7303_chip *chip;

    ret = et7303_check_revision(client);
    if (ret < 0) {
        dev_err(&client->dev, "check vid/pid fail\n");
        return ret;
    }

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        return -ENOMEM;
    }

    chip->data.regmap = devm_regmap_init_i2c(client, &et7303_regmap_config);
    if (IS_ERR(chip->data.regmap)) {
        return PTR_ERR(chip->data.regmap);
    }

    chip->dev = &client->dev;
    i2c_set_clientdata(client, chip);

    ret = et7303_sw_reset(chip);
    if (ret < 0) {
        return ret;
    }

    /* Disable chip interrupts before requesting irq */
    ret = et7303_write16(chip, TCPC_ALERT_MASK, 0);
    if (ret < 0) {
        return ret;
    }

    chip->data.init = et7303_init;
    chip->data.set_vconn = et7303_set_vconn;
    chip->data.start_drp_toggling = et7303_start_drp_toggling;
    chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
    if (IS_ERR(chip->tcpci)) {
        return PTR_ERR(chip->tcpci);
    }

    ret = devm_request_threaded_irq(chip->dev, client->irq, NULL, et7303_irq, IRQF_ONESHOT | IRQF_TRIGGER_LOW, client->name, chip);
    if (ret < 0) {
        return ret;
    }
    enable_irq_wake(client->irq);

    /* changed tower: create debug sys node. */
    ret = kobject_init_and_add(&chip->kobj, &htdebug_ktype, NULL, "htdebug");
    if (ret) {
        printk("Create htdebug dir sysfs node error!\n");
        return ret;
    }
    /* changed end. */

    return 0;
}

static int et7303_remove(struct i2c_client *client)
{
    struct et7303_chip *chip = i2c_get_clientdata(client);

    tcpci_unregister_port(chip->tcpci);
    /* changed tower: del debug sys node. */
    kobject_del(&chip->kobj);
	kobject_put(&chip->kobj);
    /* changed end. */
    return 0;
}

static const struct i2c_device_id et7303_id[] = {
    { "et7303", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, et7303_id);

static int et7303_pm_suspend(struct device *dev)
{
    return 0;
}

static int et7303_pm_resume(struct device *dev)
{
    struct et7303_chip *chip = dev->driver_data;
    if (mem_sleep_current == PM_SUSPEND_MEM_ULTRA) {
        //printk("ljc:---->%s:PM_SUSPEND_MEM_ULTRA\n",__func__);
        et7303_sw_reset(chip);

        /*----from et7303_init----*/
        /* CK 300K from 320K, shipping off, auto_idle enable, tout = 32ms */
        et7303_write8(chip, ET7303_RTCTRL8, ET7303_RTCTRL8_SET(0, 1, 1, 2));

        /* I2C reset : (val + 1) * 12.5ms */
        et7303_write8(chip, ET7303_RTCTRL11, ET7303_RTCTRL11_SET(1, 0x0F));

        /* tTCPCfilter : (26.7 * val) us */
        et7303_write8(chip, ET7303_RTCTRL14, 0x0F);

        /*  tDRP : (51.2 + 6.4 * val) ms */
        et7303_write8(chip, ET7303_RTCTRL15, 0x04);

        /* dcSRC.DRP : 33% */
        et7303_write16(chip, ET7303_RTCTRL16, 330);
    }
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id et7303_of_match[] = {
    { .compatible = "etek,et7303", },
    {},
};
MODULE_DEVICE_TABLE(of, et7303_of_match);
#endif

static const struct dev_pm_ops et7303_pm_ops = {
    .suspend = et7303_pm_suspend,
    .resume = et7303_pm_resume,
};

static struct i2c_driver et7303_i2c_driver = {
    .driver = {
        .name = "et7303",
        .pm = &et7303_pm_ops,
        .of_match_table = of_match_ptr(et7303_of_match),
    },
    .probe = et7303_probe,
    .remove = et7303_remove,
    .id_table = et7303_id,
};
module_i2c_driver(et7303_i2c_driver);

MODULE_AUTHOR("Wang Jie <dave.wang@rock-chips.com>");
MODULE_DESCRIPTION("ET7303 USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
