/*
* drivers/input/sensors/hall/blepower.c
*
* Copyright (C) 2012-2016 Rockchip Co.,Ltd.
* Author: Bin Yang <yangbin@rock-chips.com>
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

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#include <linux/sensor-dev.h>
//#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/rk_keys.h>
#include <linux/suspend.h>
#include <linux/htfy_dbg.h>  // 20220720,hsl add.

struct blepower_para {
    struct device *dev;
    struct notifier_block fb_notif;
    struct mutex ops_lock;
    //int is_suspend;
    int det_pin;
    int irq;
    bool irq_disabled;
    int power_pin;
    int last_gpio_value;
	bool enable;
	struct wake_lock wake_lock; //tanlq add 221028
};

#define BLE_DBG          1

static struct blepower_para *ble = NULL;
static struct class *ble_class;

static struct input_dev *sinput_dev;

// 20220723: 只判断 POWER 的 IRQ会有问题。deep-sleep 的情况下，WIFI/BT的唤醒
// 也会导致系统闪屏。此处需要修改。
#define BLE_FG_IRQ         59
static ssize_t ble_enable_show(struct class *cls,
		struct class_attribute *attr, char *buf)
{
	//struct blepower_para *ble = dev_get_drvdata(dev);
	ssize_t ret;
	int retval;
	//mutex_lock(&cd->system_lock);
	retval = ble->enable;
	ret = snprintf(buf, 10,
		" %x\n", retval);

	//mutex_unlock(&cd->system_lock);

	return ret;
}
		
static ssize_t ble_enable_store(struct class *cls,
		struct class_attribute *attr, const char *buf, size_t size)
{
	//struct blepower_para *ble = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;
    int gpio_value = 0;
    int out_value = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		printk("%s: Invalid value\n", __func__);
		return size;
	}

	//mutex_lock(&(ble->wake_lock);
	switch (value) {
	case 0:
		ble->enable = false;
		printk("%s: enable now %d\n",__func__,ble->enable);

		break;

	case 1:
		ble->enable = true;
		printk("%s: enable now %d\n",__func__,ble->enable);
		break;
		
	default:
		ble->enable = false;
		printk("%s: Invalid value\n", __func__);
	}
	//mutex_unlock(&(ble->wake_lock));
	if(ble->enable == false){
		gpio_set_value(ble->power_pin,0);
	}else{
		gpio_value = gpio_get_value(ble->det_pin);
		if(BLE_DBG) printk("%s gpio_value=0x%x,last value=%d\n", __func__, 
			gpio_value, ble->last_gpio_value);
			ble->last_gpio_value = gpio_value;
			if(gpio_value==1)
				gpio_set_value(ble->power_pin,0);
			else
				gpio_set_value(ble->power_pin,1);
			out_value = gpio_get_value(ble->power_pin);
			if(BLE_DBG)
				printk("%s out_value=0x%x\n", __func__, out_value);
	}
	return size;
}

#if 1
static int ble_fb_notifier_callback(struct notifier_block *self, unsigned long action, void *data)
{
    struct blepower_para *blepower;
    //struct fb_event *event = data;
    int gpio_value = 0;
    int out_value = 0;
    blepower = container_of(self, struct blepower_para, fb_notif);
	if(blepower->enable == false)
		return 0 ;
    mutex_lock(&blepower->ops_lock);

    gpio_value = gpio_get_value(blepower->det_pin);
    if (action == EINK_NOTIFY_EVENT_SCREEN_OFF) {
        if (!blepower->irq_disabled) {
            //disable_irq(blepower->irq);
            blepower->irq_disabled = 1;
            disable_irq_wake(blepower->irq);
            printk("%s :disable irq \n ", __func__);
        }
		gpio_set_value(blepower->power_pin, 0);
    } else if (action == EINK_NOTIFY_EVENT_SCREEN_ON) {
        if (blepower->irq_disabled) {
            blepower->irq_disabled = false;
            //enable_irq(blepower->irq);
            enable_irq_wake(blepower->irq);
            printk("%s :enable irq \n ", __func__);
        }
        if(gpio_value==1)
            gpio_set_value(blepower->power_pin,0);
        else
            gpio_set_value(blepower->power_pin,1);
    }
    out_value = gpio_get_value(blepower->power_pin);
    mutex_unlock(&blepower->ops_lock);
    if(BLE_DBG)
        printk("%s :action=:%d gpio_value=%d power:%d\n", __func__,action,gpio_value,out_value);

    return NOTIFY_OK;
}
#endif 

//static void blepower_power_control(struct blepower_para *blepower, int gpio_value)
//{

//    gpio_set_value(blepower->power_pin,gpio_value);
//}

static irqreturn_t blepower_interrupt(int irq, void *dev_id)
{
    struct blepower_para *blepower = (struct blepower_para *)dev_id;
    int gpio_value = 0;
    int out_value = 0;

	if(blepower->enable == false){
		gpio_set_value(blepower->power_pin,0);
		return IRQ_HANDLED;
	}
    gpio_value = gpio_get_value(blepower->det_pin);
    if(BLE_DBG) printk("%s gpio_value=0x%x,last value=%d\n", __func__, 
        gpio_value, blepower->last_gpio_value);
    if (true/*blepower->last_gpio_value != gpio_value*/) {        
        blepower->last_gpio_value = gpio_value;
        if(gpio_value==1)
            gpio_set_value(blepower->power_pin,0);
        else
            gpio_set_value(blepower->power_pin,1);
        out_value = gpio_get_value(blepower->power_pin);
        if(BLE_DBG)
            printk("%s out_value=0x%x\n", __func__, out_value);
    }

    return IRQ_HANDLED;
}

static int blepower_suspend(struct platform_device *dev, pm_message_t state)
{
#if 1
    struct blepower_para *blepower = (struct blepower_para *)platform_get_drvdata(dev);
    int out_value = 0;
    blepower->last_gpio_value = gpio_get_value(blepower->det_pin);
    out_value = gpio_get_value(blepower->power_pin);
    if(BLE_DBG)
        printk("entering %s,det_gpio_value=%d out_value=%d\n", __func__, blepower->last_gpio_value,out_value);
#endif
    return 0;
}

static int blepower_resume(struct platform_device *dev)
{
#if 1
    struct blepower_para *blepower = (struct blepower_para *)platform_get_drvdata(dev);
    int gpio_value = 0;
    int out_value = 0;
    gpio_value = gpio_get_value(blepower->det_pin);
    out_value = gpio_get_value(blepower->power_pin);
    if(BLE_DBG) printk("entering %s,gpio_value=%d/%d,wake_irq=%d,wkirq=%d,fb_off=%d\n", 
        __func__, blepower->last_gpio_value, gpio_value,  pm_wakeup_irq, 
        blepower->irq, fb_is_power_off());
    if(BLE_DBG)
        printk("entering %s,det_gpio_value=%d out_value=%d\n", __func__, gpio_value,out_value);

    //if(/*fb_is_power_off() && gpio_value == blepower->active_value &&*/
    //    !blepower->irq_handled 
    //    && (gpio_value != blepower->last_gpio_value || // rk817_hall_irq_wakeup
     //       HALL_FG_IRQ == pm_wakeup_irq || blepower->irq == pm_wakeup_irq)){
     //   blepower_report_key(blepower, gpio_value);
    //}
    #endif
    return 0;
}

//static DEVICE_ATTR(enable, S_IRUSR | S_IWUSR, ble_enable_show, ble_enable_store);
//static CLASS_ATTR_RW(ble_enable);
static struct class_attribute class_attr_ble_enable={
		.attr={.name="ble_enable",.mode=0666},
		.show=ble_enable_show,
		.store=ble_enable_store,
};
static int blepower_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct blepower_para *blepower;
    enum of_gpio_flags irq_flags,power_flags;
    int hallactive = 0;
    int gpio_value = 0;
    int ret = 0;
    struct input_dev *hall_key;

    //printk("hall %s init ...\n", __func__);

    blepower = devm_kzalloc(&pdev->dev, sizeof(*blepower), GFP_KERNEL);
    if (!blepower) {
        return -ENOMEM;
    }

    blepower->dev = &pdev->dev;

    blepower->det_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, &irq_flags);
    if (!gpio_is_valid(blepower->det_pin)) {
        printk("Can not read property irq-gpio\n");
        return blepower->det_pin;
    }
    blepower->irq = gpio_to_irq(blepower->det_pin);

    blepower->power_pin = of_get_named_gpio_flags(np, "power-gpio", 0, &power_flags);
    if (!gpio_is_valid(blepower->power_pin)) {
        printk("Can not read property power-gpio\n");
        return blepower->power_pin;
    }else{
        gpio_request(blepower->power_pin, NULL);
        gpio_direction_output(blepower->power_pin, 1);
    }

    mutex_init(&blepower->ops_lock);

    ret = devm_gpio_request_one(blepower->dev, blepower->det_pin, GPIOF_DIR_IN, "bledet");
    if (ret < 0) {
        printk("fail to request gpio:%d\n", blepower->det_pin);
        return ret;
    }
    gpio_value = gpio_get_value(blepower->det_pin);
    blepower->last_gpio_value = gpio_value;

    ret = devm_request_threaded_irq(blepower->dev, blepower->irq, NULL, blepower_interrupt, 
        irq_flags | IRQF_NO_SUSPEND | IRQF_ONESHOT, "bledet", blepower);
    if (ret < 0) {
        printk("request irq(%d) failed, ret=%d\n", blepower->irq, ret);
        return ret;
    }
    if(gpio_value==1)
        gpio_set_value(blepower->power_pin,0);
    else
        gpio_set_value(blepower->power_pin,1);
    enable_irq_wake(blepower->irq);
    blepower->fb_notif.notifier_call = ble_fb_notifier_callback;
    htfy_ebc_register_notifier(&blepower->fb_notif);
    blepower->irq_disabled = 0;
    ble = blepower;

    platform_set_drvdata(pdev, blepower);
	//device_create_file(&pdev->dev, &dev_attr_enable);
	ble_class = class_create(THIS_MODULE, "ble_power");
	ret =  class_create_file(ble_class, &class_attr_ble_enable);
	if (ret)
		printk("Fail to create class ble_enable.\n");
	//wake_lock_init(&blepower->wake_lock, WAKE_LOCK_SUSPEND,
	//		   "blepower_lock"); //tanlq add 221028
    printk("blepower_probe success.\n");

    return 0;
}

int is_ble_in(void)
{
    int in_value,out_value;

    in_value = gpio_get_value(ble->det_pin);
	out_value = gpio_get_value(ble->power_pin);
    if(BLE_DBG) printk("[is_ble_in]: in_value=%d,out_value=%d\n", in_value, out_value);
    if (in_value != 0){
        return true;
    }
    return false;
}
EXPORT_SYMBOL(is_ble_in);

static const struct of_device_id blepower_match[] = {
    { .compatible = "ble-power" },
    { /* Sentinel */ }
};

static struct platform_driver blepower_driver = {
    .probe = blepower_probe,
    .suspend = blepower_suspend,
    .resume = blepower_resume,
    .driver = {
        .name = "blepower",
        .owner = THIS_MODULE,
        .of_match_table = blepower_match,
    },
};

module_platform_driver(blepower_driver);

MODULE_ALIAS("platform:blepower");
MODULE_AUTHOR("Tanluqiang <tanluqiang@htfyun.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("blepower driver");
