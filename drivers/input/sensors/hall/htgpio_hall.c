/*
* Copyright (C) 2023 HTFyun.,Ltd.
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
#include <linux/htfy_dbg.h>

struct htgpiohall_para {
    struct device *dev;
    struct input_dev *input;
#ifdef CONFIG_FB
    struct notifier_block fb_notif;
#endif
    struct mutex ops_lock;
    int is_suspend;
    int irqgpio;
    int irq;
    int active_value;
};

#ifdef CONFIG_FB
static int hall_fb_notifier_callback(struct notifier_block *self, unsigned long action, void *data)
{
    struct htgpiohall_para *halldata;

    halldata = container_of(self, struct htgpiohall_para, fb_notif);

    dev_info(halldata->dev, "entering %s, action=%d\n", __func__, action);

    mutex_lock(&halldata->ops_lock);
    if (action == EINK_NOTIFY_EVENT_SCREEN_OFF) {
        halldata->is_suspend = 1;
    } else if (action == EINK_NOTIFY_EVENT_SCREEN_ON) {
        halldata->is_suspend = 0;
    }
    mutex_unlock(&halldata->ops_lock);

    return NOTIFY_OK;
}
#endif

static void send_power_key(struct htgpiohall_para *data, int state)
{
    if (!data || !data->input) {
        return;
    }
    if (state) {
        input_report_key(data->input, KEY_POWER, 1);
        input_sync(data->input);
    } else {
        input_report_key(data->input, KEY_POWER, 0);
        input_sync(data->input);
    }
}

static void send_wakeup_key(struct htgpiohall_para *data)
{
    if (!data || !data->input) {
        return;
    }

    input_report_key(data->input, KEY_WAKEUP, 1);
    input_sync(data->input);
    input_report_key(data->input, KEY_WAKEUP, 0);
    input_sync(data->input);
}

static irqreturn_t htgpiohall_interrupt(int irq, void *dev_id)
{
    struct htgpiohall_para *halldata = (struct htgpiohall_para *)dev_id;
    int gpio_value = 0;

    gpio_value = gpio_get_value(halldata->irqgpio);
    dev_info(halldata->dev, "hall gpio value=%x, suspend=%d ...\n", gpio_value, halldata->is_suspend);
    if ((gpio_value != halldata->active_value) && (halldata->is_suspend == 0)) {
        send_power_key(halldata, 1);
        send_power_key(halldata, 0);
        dev_info(halldata->dev, "hall suspend ...\n");
    } else if ((gpio_value == halldata->active_value) && (halldata->is_suspend == 1)) {
        send_wakeup_key(halldata);
        dev_info(halldata->dev, "hall wakeup ...\n");
    }

    return IRQ_HANDLED;
}

static int htgpiohall_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct htgpiohall_para *halldata = NULL;
    struct input_dev *input = NULL;
    enum of_gpio_flags irq_flags;
    int hallactive = 0;
    int ret = 0;

    halldata = devm_kzalloc(&pdev->dev, sizeof(*halldata), GFP_KERNEL);
    if (!halldata) {
        dev_err(&pdev->dev, "alloc halldata failed.\n");
        return -ENOMEM;
    }

    halldata->dev = &pdev->dev;

    halldata->irqgpio = of_get_named_gpio_flags(np, "irq-gpio", 0, &irq_flags);
    if (!gpio_is_valid(halldata->irqgpio)) {
        dev_err(&pdev->dev, "cat not get irq gpio!!!\n");
        return -EINVAL;
    }
    halldata->irq = gpio_to_irq(halldata->irqgpio);

    of_property_read_u32(np, "hall-active", &hallactive);
    halldata->active_value = hallactive;
    halldata->is_suspend = 0;
    mutex_init(&halldata->ops_lock);

    ret = devm_gpio_request_one(&pdev->dev, halldata->irqgpio, GPIOF_DIR_IN, "htgpiohall");
    if (ret < 0) {
        dev_err(&pdev->dev, "fail to request gpio:%d!!!\n", halldata->irqgpio);
        return -ENODEV;
    }

    ret = devm_request_threaded_irq(&pdev->dev, halldata->irq, NULL, htgpiohall_interrupt, irq_flags | IRQF_NO_SUSPEND | IRQF_ONESHOT, "htgpiohall", halldata);
    if (ret < 0) {
        dev_err(&pdev->dev, "request irq(%d) failed, ret=%d!!!\n", halldata->irq, ret);
        return ret;
    }

    enable_irq_wake(halldata->irq);
#ifdef CONFIG_FB
    memset(&halldata->fb_notif, 0, sizeof(struct notifier_block));
    halldata->fb_notif.notifier_call = hall_fb_notifier_callback;
    htfy_ebc_register_notifier(&halldata->fb_notif);
#endif

    input = devm_input_allocate_device(&pdev->dev);
    if (!input) {
        dev_err(&pdev->dev, "alloc hall input device failed!!!\n");
        return -ENOMEM;
    }

    input->name = "hall-key";
    input->phys = "hall-key/input0";
    input->dev.parent = &pdev->dev;
    input->id.bustype = BUS_HOST;
    input->id.vendor = 0x0001;
    input->id.product = 0x0001;
    input->id.version = 0x0100;
    input_set_capability(input, EV_KEY, KEY_POWER);
    input_set_capability(input, EV_KEY, KEY_WAKEUP);

    ret = input_register_device(input);
    if (ret) {
        dev_err(&pdev->dev, "Unable to register input device, error: %d\n", ret);
        return -ENODEV;
    }
    halldata->input = input;
    platform_set_drvdata(pdev, halldata);

    dev_info(&pdev->dev, "%s success.\n", __func__);

    return 0;
}

static const struct of_device_id htgpiohall_match[] = {
    {.compatible = "hthall-gpio"},
    {},
};

static struct platform_driver htgpiohall_driver = {
    .probe = htgpiohall_probe,
    .driver = {
        .name = "hall",
        .owner = THIS_MODULE,
        .of_match_table = htgpiohall_match,
    },
};

module_platform_driver(htgpiohall_driver);

MODULE_ALIAS("HT Giop Hall");
MODULE_AUTHOR("Tower");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HTFyun GpioHall driver");
