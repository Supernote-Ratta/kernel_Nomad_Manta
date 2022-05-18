/*
* drivers/input/sensors/hall/wh2506d.c
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
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#include <linux/sensor-dev.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/rk_keys.h>

struct wh2506d_para {
    struct device *dev;
    struct notifier_block fb_notif;
    struct mutex ops_lock;
    int is_suspend;
    int gpio_pin;
    int irq;
    int active_value;
};

static struct wh2506d_para *hall = NULL;
static struct input_dev *sinput_dev;
static int hall_gpio_value = 0xff;

static int hall_fb_notifier_callback(struct notifier_block *self, unsigned long action, void *data)
{
    struct wh2506d_para *wh2506d;
    struct fb_event *event = data;

    wh2506d = container_of(self, struct wh2506d_para, fb_notif);

    mutex_lock(&wh2506d->ops_lock);
    if (action == FB_EARLY_EVENT_BLANK) {
        switch (*((int *)event->data)) {
            case FB_BLANK_UNBLANK:
                break;
            default:
                wh2506d->is_suspend = 1;
                break;
        }
    } else if (action == FB_EVENT_BLANK) {
        switch (*((int *)event->data)) {
            case FB_BLANK_UNBLANK:
                wh2506d->is_suspend = 0;
                break;
            default:
                break;
        }
    }
    mutex_unlock(&wh2506d->ops_lock);

    return NOTIFY_OK;
}

static irqreturn_t hall_wh2506d_interrupt(int irq, void *dev_id)
{
    struct wh2506d_para *wh2506d = (struct wh2506d_para *)dev_id;
    int gpio_value = 0;

    gpio_value = gpio_get_value(wh2506d->gpio_pin);
    if (hall_gpio_value != gpio_value) {
        printk("hall %s gpio_value=%x,wh2506d->is_suspend=%d ...\n", __func__, gpio_value, wh2506d->is_suspend);
        hall_gpio_value = gpio_value;

        if (gpio_value != wh2506d->active_value) {
            input_report_key(sinput_dev, KEY_SLEEP, 1);
            input_sync(sinput_dev);
            input_report_key(sinput_dev, KEY_SLEEP, 0);
            input_sync(sinput_dev);
            printk("hall %s suspend ...\n", __func__);
        } else if (gpio_value == wh2506d->active_value) {
            input_report_key(sinput_dev, KEY_WAKEUP, 1);
            input_sync(sinput_dev);
            input_report_key(sinput_dev, KEY_WAKEUP, 0);
            input_sync(sinput_dev);
            printk("hall %s wakeup ...\n", __func__);
        }
    }

    return IRQ_HANDLED;
}

static int hall_wh2506d_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct wh2506d_para *wh2506d;
    enum of_gpio_flags irq_flags;
    int hallactive = 0;
    int gpio_value = 0;
    int ret = 0;
    struct input_dev *hall_key;

    printk("hall %s init ...\n", __func__);

    wh2506d = devm_kzalloc(&pdev->dev, sizeof(*wh2506d), GFP_KERNEL);
    if (!wh2506d) {
        return -ENOMEM;
    }

    wh2506d->dev = &pdev->dev;

    wh2506d->gpio_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, &irq_flags);
    if (!gpio_is_valid(wh2506d->gpio_pin)) {
        printk("Can not read property irq-gpio\n");
        return wh2506d->gpio_pin;
    }
    wh2506d->irq = gpio_to_irq(wh2506d->gpio_pin);

    of_property_read_u32(np, "hall-active", &hallactive);
    wh2506d->active_value = hallactive;
    wh2506d->is_suspend = 0;
    mutex_init(&wh2506d->ops_lock);

    ret = devm_gpio_request_one(wh2506d->dev, wh2506d->gpio_pin, GPIOF_DIR_IN, "hall_wh2506d");
    if (ret < 0) {
        printk("fail to request gpio:%d\n", wh2506d->gpio_pin);
        return ret;
    }
    gpio_value = gpio_get_value(wh2506d->gpio_pin);

    ret = devm_request_threaded_irq(wh2506d->dev, wh2506d->irq, NULL, hall_wh2506d_interrupt, irq_flags | IRQF_NO_SUSPEND | IRQF_ONESHOT, "hall_wh2506d", wh2506d);
    if (ret < 0) {
        printk("request irq(%d) failed, ret=%d\n", wh2506d->irq, ret);
        return ret;
    }

    enable_irq_wake(wh2506d->irq);
    wh2506d->fb_notif.notifier_call = hall_fb_notifier_callback;
    fb_register_client(&wh2506d->fb_notif);

    hall = wh2506d;

    hall_key = devm_input_allocate_device(&pdev->dev);
    if (!hall_key) {
        printk("Can't allocate hall key input\n");
        return -ENOMEM;
    }
    hall_key->name = "hall-key";
    hall_key->phys = "hall_key/input0";
    hall_key->id.bustype = BUS_HOST;
    input_set_capability(hall_key, EV_KEY, KEY_SLEEP);
    input_set_capability(hall_key, EV_KEY, KEY_WAKEUP);
    ret = input_register_device(hall_key);
    if (ret) {
        printk("Unable to register input device, error: %d\n", ret);
        return ret;
    }
    sinput_dev = hall_key;
    printk("hall_wh2506d_probe success.\n");

    return 0;
}

int hall_get_value(void)
{
    int result = 0;

    result = gpio_get_value(hall->gpio_pin);
    printk("[hall_get_value] %s: result=%d\n", __func__, result);

    return result;
}
EXPORT_SYMBOL(hall_get_value);

static const struct of_device_id hall_wh2506d_match[] = {
    { .compatible = "hall-wh2506d" },
    { /* Sentinel */ }
};

static struct platform_driver hall_wh2506d_driver = {
    .probe = hall_wh2506d_probe,
    .driver = {
        .name = "wh2506d",
        .owner = THIS_MODULE,
        .of_match_table = hall_wh2506d_match,
    },
};

module_platform_driver(hall_wh2506d_driver);

MODULE_ALIAS("platform:wh2506d");
MODULE_AUTHOR("Ye Hongfeng <yehongfeng@boe.com.cn>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hall Sensor WH2506D driver");
