/*
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/mfd/rk808.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/rk_usbbc.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#define MAX_FIELDS 40

enum charger_t {
    USB_TYPE_UNKNOWN_CHARGER,
    USB_TYPE_NONE_CHARGER,
    USB_TYPE_USB_CHARGER,
    USB_TYPE_AC_CHARGER,
    USB_TYPE_AC_OLD_CHARGER,
    USB_TYPE_CDP_CHARGER,
    DC_TYPE_DC_CHARGER,
    DC_TYPE_NONE_CHARGER,
};

struct charger_platform_data {
    u32 max_input_current;
    u32 min_input_voltage;
    u32 max_chrg_current;
    u32 max_chrg_voltage;
    u32 chrg_finish_cur;
    u32 chrg_term_mode;
    u32 power_dc2otg;
    u32 dc_det_level;
    int dc_det_pin;
    bool support_dc_det;
    int virtual_power;
    int sample_res;
    int otg5v_suspend_enable;
    bool extcon;
    int gate_function_disable;
};

struct rk817_charger {
    struct i2c_client *client;
    struct platform_device *pdev;
    struct device *dev;
    struct rk808 *rk817;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[MAX_FIELDS];
    struct power_supply *ac_psy;
    struct power_supply *usb_psy;
    struct extcon_dev *cable_edev;
    struct charger_platform_data *pdata;
    struct workqueue_struct *usb_charger_wq;
    struct workqueue_struct *dc_charger_wq;
    struct delayed_work dc_work;
    struct delayed_work usb_work;
    struct delayed_work host_work;
    struct delayed_work discnt_work;
    struct delayed_work irq_work;
    struct notifier_block bc_nb;
    struct notifier_block cable_cg_nb;
    struct notifier_block cable_host_nb;
    struct notifier_block cable_discnt_nb;
    unsigned int bc_event;
    enum charger_t usb_charger;
    enum charger_t dc_charger;
    struct regulator *otg5v_rdev;
    u8 ac_in;
    u8 usb_in;
    u8 otg_in;
    u8 dc_in;
    u8 prop_status;
    u32 max_input_current;
    u32 min_input_voltage;
    u32 max_chrg_current;
    u32 max_chrg_voltage;
    u32 chrg_finish_cur;
    u32 chrg_term_mode;
    u8 res_div;
    u8 otg_slp_state;
    u8 plugin_trigger;
    u8 plugout_trigger;
    int plugin_irq;
    int plugout_irq;
	struct wake_lock		suspend_lock;
};
