#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/i2c-hid.h>
#include <asm/unaligned.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include "../../gpu/drm/rockchip/ebc-dev/ebc_dev.h"

#include "../hid-ids.h"
#include "i2c-hid.h"

#define WACOM_CMD_QUERY0        0x04
#define WACOM_CMD_QUERY1        0x00
#define WACOM_CMD_QUERY2        0x33
#define WACOM_CMD_QUERY3        0x02
#define WACOM_CMD_THROW0        0x05
#define WACOM_CMD_THROW1        0x00
#define WACOM_QUERY_SIZE        19

struct hid_desc {
    __le16 wHIDDescLength;
    __le16 bcdVersion;
    __le16 wReportDescLength;
    __le16 wReportDescRegister;
    __le16 wInputRegister;
    __le16 wMaxInputLength;
    __le16 wOutputRegister;
    __le16 wMaxOutputLength;
    __le16 wCommandRegister;
    __le16 wDataRegister;
    __le16 wVendorID;
    __le16 wProductID;
    __le16 wVersionID;
    __le32 reserved;
} __packed;

struct wacom_features {
    int x_max;
    int y_max;
    int pressure_max;
	int tx;
	int ty;
    char fw_version;
};

/* The main device structure */
struct wacom_pencil {
    struct i2c_client *client;              /* i2c client */
    struct hid_device *hid;                 /* pointer to corresponding HID dev */
    struct input_dev *input;
    struct hid_desc hdesc;                  /* the HID Descriptor */
    struct notifier_block fb_notif;
    int(*pen_suspend)(struct wacom_pencil *);
    int(*pen_resume)(struct wacom_pencil *);
    struct mutex fb_lock;
    u8 data[WACOM_QUERY_SIZE];
    __le16 wHIDDescRegister;                /* location of the i2c register of the HID descriptor. */
    struct wacom_features features;
    struct regulator *supply;
    int revert_x;
    int revert_y;
    int swap_xy;
    int irq_gpio;
    int irq_level;
    int detect_gpio;
    int detect_level;
    int reset_gpio;
    int reset_level;
    bool prox;
    int tool;
    int pendet_irq;
};

/* debug option */
static bool debug = true;

module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

static u8 greport_desc[] = {
    0x05, 0x0d, 0x09, 0x02, 0xa1, 0x01, 0x85, 0x02, 0x09, 0x20, 0xa1, 0x00,
    0x09, 0x42, 0x09, 0x44, 0x09, 0x45, 0x09, 0x3c, 0x09, 0x5a, 0x09, 0x32,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x06, 0x81, 0x02, 0x95, 0x01,
    0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x26, 0x60, 0x4f, 0x46, 0x60, 0x4f,
    0x75, 0x10, 0x95, 0x01, 0x55, 0x0d, 0x65, 0x11, 0x81, 0x02, 0x09, 0x31,
    0x26, 0x88, 0x3b, 0x46, 0x88, 0x3b, 0x81, 0x02, 0x05, 0x0d, 0x09, 0x30,
    0x26, 0xff, 0x0f, 0x81, 0x02, 0x75, 0x10, 0x09, 0x3d, 0x16, 0xd8, 0xdc,
    0x26, 0x28, 0x23, 0x36, 0xd8, 0xdc, 0x46, 0x28, 0x23, 0x65, 0x14, 0x81,
    0x02, 0x09, 0x3e, 0x81, 0x02, 0xc0, 0xc0
};

#define wacom_dbg(fmt, arg...) \
    do { \
        if (debug) { \
            printk(KERN_INFO fmt, ##arg); \
        } \
    } while (0)

static int wacom_chip_power(struct i2c_client *client)
{
    struct wacom_pencil *wpen = i2c_get_clientdata(client);

    if (!gpio_is_valid(wpen->reset_gpio)) {
        dev_err(&client->dev, "gpio_rst pin available\n");
        return -ENODEV;
    }
    //gpio_direction_output(wpen->reset_gpio, wpen->reset_level);
    //msleep(100);
    gpio_direction_output(wpen->reset_gpio, !wpen->reset_level);
    //msleep(100);

    return 0;
}

static irqreturn_t wacom_report_irq(int irq, void *dev_id)
{
    struct wacom_pencil *wpen = dev_id;
    struct input_dev *input = wpen->input;
    u8 *data = wpen->data;
    unsigned int x, y, pressure;
	int tx,ty;
    unsigned char tsw, f1, f2, ers;
    int error;

    //wacom_dbg("entering %s\n", __func__);
	if(input==NULL){
		return IRQ_HANDLED;
	}

    if (device_can_wakeup(&wpen->client->dev)) {
        pm_stay_awake(&wpen->client->dev);
    }
    error = i2c_master_recv(wpen->client, wpen->data, sizeof(wpen->data));
    if (error < 0) {
        goto out;
    }

    tsw = data[3] & 0x01;
    ers = data[3] & 0x04;
    f1 = data[3] & 0x02;
    f2 = data[3] & 0x10;
    x = le16_to_cpup((__le16 *)&data[4]);
    y = le16_to_cpup((__le16 *)&data[6]);
    pressure = le16_to_cpup((__le16 *)&data[8]);
	tx = le16_to_cpup((__le16 *)&data[11]);
	ty = le16_to_cpup((__le16 *)&data[13]);

    if (!wpen->prox) {
        wpen->tool = (data[3] & 0x0c) ? BTN_TOOL_RUBBER : BTN_TOOL_PEN;
    }

    wpen->prox = data[3] & 0x20;

    if (1 == wpen->swap_xy) {
        swap(x, y);
		//swap(tx, ty);
    }
    if (1 == wpen->revert_x) {
        x = wpen->features.x_max - x;
		//tx = -tx;
    }
    if (1 == wpen->revert_y) {
        y =  wpen->features.y_max - y;
		ty = 0x10000-ty;
    }

    input_report_key(input, BTN_TOUCH, tsw || ers);
    input_report_key(input, wpen->tool, wpen->prox);
    input_report_key(input, BTN_STYLUS, f1);
    input_report_key(input, BTN_STYLUS2, f2);
    input_report_abs(input, ABS_X, x);
    input_report_abs(input, ABS_Y, y);
    input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_abs(input, ABS_TILT_X, tx);
    input_report_abs(input, ABS_TILT_Y, ty);
    input_sync(input);

out:
    if (device_can_wakeup(&wpen->client->dev)) {
        pm_relax(&wpen->client->dev);
    }

    return IRQ_HANDLED;
}

static irqreturn_t wacom_pendet_irq(int irq, void *dev_id)
{
    struct wacom_pencil *wpen = dev_id;
    //int pendet_gpio_value = gpio_get_value(wpen->detect_gpio);

    wacom_dbg("entering %s\n", __func__);
    // if (pendet_gpio_value) {
    //     irq_set_irq_type(wpen->pendet_irq, IRQ_TYPE_LEVEL_LOW);
    // } else {
    //     irq_set_irq_type(wpen->pendet_irq, IRQ_TYPE_LEVEL_HIGH);
    // }

    return IRQ_HANDLED;
}

static int wacom_hid_output_report(struct hid_device *hid, __u8 *buf, size_t count)
{
    wacom_dbg("entering %s\n", __func__);
    return 0;
}

static int wacom_hid_raw_request(struct hid_device *hid, unsigned char reportnum, __u8 *buf, size_t len, unsigned char rtype, int reqtype)
{
    wacom_dbg("entering %s\n", __func__);
    return 0;
}

static int wacom_hid_parse(struct hid_device *hid)
{
    int ret;

    wacom_dbg("entering %s\n", __func__);

    ret = hid_parse_report(hid, (char *)greport_desc, 103);
    if (ret) {
        wacom_dbg("parsing report descriptor failed\n");
        return ret;
    }

    return 0;
}

static int wacom_hid_start(struct hid_device *hid)
{
    wacom_dbg("entering %s\n", __func__);
    return 0;
}

static void wacom_hid_stop(struct hid_device *hid)
{
    wacom_dbg("entering %s\n", __func__);
}

static int wacom_hid_open(struct hid_device *hid)
{
    wacom_dbg("entering %s\n", __func__);
    return 0;
}

static void wacom_hid_close(struct hid_device *hid)
{
    wacom_dbg("entering %s\n", __func__);
}

static int wacom_hid_power(struct hid_device *hid, int lvl)
{
    wacom_dbg("%s lvl:%d\n", __func__, lvl);
    return 0;
}

struct hid_ll_driver wacom_hid_ll_driver = {
    .parse = wacom_hid_parse,
    .start = wacom_hid_start,
    .stop = wacom_hid_stop,
    .open = wacom_hid_open,
    .close = wacom_hid_close,
    .power = wacom_hid_power,
    .output_report = wacom_hid_output_report,
    .raw_request = wacom_hid_raw_request,
};
EXPORT_SYMBOL_GPL(wacom_hid_ll_driver);

static int wacom_input_open(struct input_dev *dev)
{
    struct wacom_pencil *wpen = input_get_drvdata(dev);
    struct i2c_client *client = wpen->client;

	enable_irq(client->irq);

    return 0;
}

static void wacom_input_close(struct input_dev *dev)
{
    struct wacom_pencil *wpen = input_get_drvdata(dev);
    struct i2c_client *client = wpen->client;

    disable_irq(client->irq);
}

static int wacom_query_device(struct wacom_pencil *wpen)
{
    int ret;
    u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1, WACOM_CMD_QUERY2, WACOM_CMD_QUERY3};
    u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
    u8 data[WACOM_QUERY_SIZE];
    struct i2c_client *client = wpen->client;
    struct wacom_features *features = &wpen->features;
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = sizeof(cmd1),
            .buf = cmd1,
        },
        {
            .addr = client->addr,
            .flags = 0,
            .len = sizeof(cmd2),
            .buf = cmd2,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = sizeof(data),
            .buf = data,
        },
    };

    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret < 0) {
        wacom_dbg("wacom query device error:%d\n", ret);
        return ret;
    }
    if (ret != ARRAY_SIZE(msgs)) {
        return -EIO;
    }

    features->x_max = get_unaligned_le16(&data[3]);
    features->y_max = get_unaligned_le16(&data[5]);
    features->pressure_max = get_unaligned_le16(&data[11]);
    features->fw_version = get_unaligned_le16(&data[13]);
    wacom_dbg("Wacom source screen x_max:%d, y_max:%d, pressure:%d, fw:%x\n", features->x_max, features->y_max, features->pressure_max, features->fw_version);

    if (wpen->swap_xy) {
        swap(features->x_max, features->y_max);
    }

    wacom_dbg("Wacom desc screen x_max:%d, y_max:%d\n", features->x_max, features->y_max);

    return 0;
}

static int get_hid_desc(struct wacom_pencil *wpen)
{
    struct i2c_client *client = wpen->client;
    struct hid_desc *hiddesc = &wpen->hdesc;
    int ret = -1;
    short int cmd = wpen->wHIDDescRegister;
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 2,
            .buf = (__u8 *) &cmd,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = sizeof(*hiddesc),
            .buf = (char *)hiddesc,
        },
    };

    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret < 0) {
        return ret;
    }
    if (ret != ARRAY_SIZE(msgs)) {
        return -EIO;
    }

    wacom_dbg("******************************\n");
    wacom_dbg("wacom firmware vesrsion:0x%x\n", hiddesc->wVersionID);
    wacom_dbg("******************************\n");

    return 0;
}

static int wacom_pen_init_detirq(struct i2c_client *client)
{
    int ret = -1;
    struct wacom_pencil *wpen = i2c_get_clientdata(client);

    wpen->pendet_irq = gpio_to_irq(wpen->detect_gpio);
    if (wpen->pendet_irq < 0) {
        dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", wpen->detect_gpio, wpen->pendet_irq);
        return ret;
    }

    ret = devm_request_threaded_irq(&client->dev, wpen->pendet_irq, NULL, wacom_pendet_irq, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "wacom-pendet", wpen);
    if (ret < 0) {
        dev_err(&client->dev, "Could not register for %s interrupt, irq=%d, ret=%d\n", "wacom-pendet", wpen->pendet_irq, ret);
        return ret;
    }
    disable_irq(wpen->pendet_irq);

    return 0;
}

static int wacom_pen_init_reportirq(struct i2c_client *client)
{
    struct wacom_pencil *wpen = i2c_get_clientdata(client);
    int ret = -1;

    client->irq = gpio_to_irq(wpen->irq_gpio);
    if (client->irq < 0) {
        dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", wpen->irq_gpio, client->irq);
        return ret;
    }

    ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, wacom_report_irq, IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, wpen);
    if (ret < 0) {
        dev_err(&client->dev, "Could not register for %s interrupt, irq = %d, ret = %d\n", client->name, client->irq, ret);
        return ret;
    }
    disable_irq(client->irq);

    return 0;
}

static int wacom_pen_init_irq(struct i2c_client *client)
{
    int ret;
    struct wacom_pencil *wpen = i2c_get_clientdata(client);

    ret = wacom_pen_init_reportirq(client);
    if (ret) {
        return ret;
    }
    ret = wacom_pen_init_detirq(client);
    if (ret) {
        goto error_irq;
    }

    return 0;

error_irq:
    free_irq(client->irq, wpen);
    return ret;
}

static int wacom_pen_of_probe(struct wacom_pencil *wpen)
{
    struct i2c_client *client = wpen->client;
    struct device_node *wac_np;
    int ret = 0, hidRegister = 0;
    enum of_gpio_flags flags;

    wac_np = client->dev.of_node;
    if (!wac_np) {
        dev_err(&client->dev, "get device node error!!!\n");
        return -ENODEV;
    }
    of_property_read_u32(wac_np, "revert_x", &wpen->revert_x);
    of_property_read_u32(wac_np, "revert_y", &wpen->revert_y);
    of_property_read_u32(wac_np, "xy_exchange", &wpen->swap_xy);

    ret = of_property_read_u32(wac_np, "hid-descr-addr", &hidRegister);
    if (ret) {
        dev_err(&client->dev, "HID register address not provided\n");
        return -ENODEV;
    }
    if (hidRegister >> 16) {
        dev_err(&client->dev, "Bad HID register address: 0x%08x\n", hidRegister);
        return -ENODEV;
    }
    wpen->wHIDDescRegister = cpu_to_le16(hidRegister);
    wpen->supply = devm_regulator_get(&client->dev, "pwr");
    if (wpen->supply) {
        wacom_dbg("wacom power supply = %dmv\n", regulator_get_voltage(wpen->supply));
        ret = regulator_enable(wpen->supply);
        if (ret < 0) {
            dev_err(&client->dev, "failed to enable wacom power supply!!!\n");
            return -ENODEV;
        }
    }

    wpen->reset_gpio = of_get_named_gpio_flags(wac_np, "gpio_rst", 0, &flags);
    if (!gpio_is_valid(wpen->reset_gpio)) {
        dev_err(&client->dev, "no gpio_rst pin available\n");
        return -ENODEV;
    }
    wpen->reset_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
    wacom_dbg("wacom reset level %d\n", wpen->reset_level);
    ret = devm_gpio_request_one(&client->dev, wpen->reset_gpio, !wpen->reset_level ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW, "wacom-rst");
    if (ret < 0) {
        dev_err(&client->dev, "request gpio_rst pin failed!!!\n");
        return -ENODEV;
    }
    wpen->detect_gpio = of_get_named_gpio_flags(wac_np, "gpio_detect", 0, &flags);
    if (!gpio_is_valid(wpen->detect_gpio)) {
        dev_err(&client->dev, "no detect_gpio pin available\n");
        goto err_free_reset_gpio;
    }
    wpen->detect_level = (flags & OF_GPIO_ACTIVE_LOW) ?  0 : 1;
    ret = devm_gpio_request_one(&client->dev, wpen->detect_gpio, GPIOF_IN, "wacom_detect");
    if (ret < 0) {
        goto err_free_reset_gpio;
    }
    wpen->irq_gpio = of_get_named_gpio_flags(wac_np, "gpio_intr", 0, &flags);
    if (!gpio_is_valid(wpen->irq_gpio)) {
        dev_err(&client->dev, "no gpio_intr pin available\n");
        goto err_free_detect_gpio;
    }
    wpen->irq_level = (flags & OF_GPIO_ACTIVE_LOW) ?  0 : 1;
    ret = devm_gpio_request_one(&client->dev, wpen->irq_gpio, GPIOF_IN, "wacom_intr");
    if (ret < 0) {
        goto err_free_detect_gpio;
    }

    return 0;

err_free_detect_gpio:
    devm_gpio_free(&client->dev, wpen->detect_gpio);
err_free_reset_gpio:
    devm_gpio_free(&client->dev, wpen->reset_gpio);
    return -ENODEV;
}

static int hid_init(struct wacom_pencil *wpen)
{
    int ret = -1;
    struct i2c_client *client = wpen->client;

    wpen->hid = hid_allocate_device();
    if (IS_ERR(wpen->hid)) {
        PTR_ERR(wpen->hid);
        return -ENOMEM;
    }

    wpen->hid->driver_data = client;
    wpen->hid->ll_driver = &wacom_hid_ll_driver;
    wpen->hid->dev.parent = &client->dev;
    wpen->hid->bus = BUS_I2C;
    wpen->hid->version = le16_to_cpu(wpen->hdesc.bcdVersion);
    wpen->hid->vendor = le16_to_cpu(wpen->hdesc.wVendorID);
    wpen->hid->product = le16_to_cpu(wpen->hdesc.wProductID);

    snprintf(wpen->hid->name, sizeof(wpen->hid->name), "%s %04hX:%04hX", client->name, wpen->hid->vendor, wpen->hid->product);
    strlcpy(wpen->hid->phys, dev_name(&client->dev), sizeof(wpen->hid->phys));

    ret = hid_add_device(wpen->hid);
    if (ret) {
        if (ret != -ENODEV) {
            wacom_dbg("can't add hid device: %d\n", ret);
        }
        return ret;
    }

    return 0;
}

static int input_init(struct wacom_pencil *wpen)
{
    int ret = 0;
    struct i2c_client *client = wpen->client;

    wpen->input = input_allocate_device();
    if (!wpen->input) {
        wacom_dbg("input allocate failed!!!\n");
        return -ENOMEM;
    }

    wpen->input->name = "Wacom Pencil";
    wpen->input->id.bustype = BUS_I2C;
    wpen->input->id.vendor = 0x56a;
    wpen->input->id.version = wpen->hdesc.wVersionID;
    wpen->input->dev.parent = &client->dev;
    wpen->input->open = wacom_input_open;
    wpen->input->close = wacom_input_close;
    wpen->input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

    __set_bit(BTN_TOOL_PEN, wpen->input->keybit);
    __set_bit(BTN_TOOL_RUBBER, wpen->input->keybit);
    __set_bit(BTN_STYLUS, wpen->input->keybit);
    __set_bit(BTN_STYLUS2, wpen->input->keybit);
    __set_bit(BTN_TOUCH, wpen->input->keybit);
    __set_bit(INPUT_PROP_DIRECT, wpen->input->propbit);

    input_set_abs_params(wpen->input, ABS_X, 0, wpen->features.x_max, 0, 0);
    input_set_abs_params(wpen->input, ABS_Y, 0, wpen->features.y_max, 0, 0);
    input_set_abs_params(wpen->input, ABS_PRESSURE, 0, wpen->features.pressure_max, 0, 0);
	input_set_abs_params(wpen->input, ABS_TILT_X, -9000, 9000, 0, 0);
	input_set_abs_params(wpen->input, ABS_TILT_Y, -9000, 9000, 0, 0);
    input_set_drvdata(wpen->input, wpen);
    ret = input_register_device(wpen->input);
    if (ret) {
        wacom_dbg("Failed to register input device, error: %d\n", ret);
        input_free_device(wpen->input);
        wpen->input = NULL;
        return ret;
    }

    return 0;
}

static int wacom_pen_early_suspend(struct wacom_pencil *wpen)
{
    struct i2c_client *client = wpen->client;

    wacom_dbg("entering %s\n", __func__);

    disable_irq(client->irq);

    return 0;
}

static int wacom_pen_late_resume(struct wacom_pencil *wpen)
{
    struct i2c_client *client = wpen->client;

    wacom_dbg("entering %s\n", __func__);

    wacom_chip_power(client);
    enable_irq(client->irq);

    return 0;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long action, void *data)
{
    struct wacom_pencil *pen;

    pen = container_of(self, struct wacom_pencil, fb_notif);

    wacom_dbg("entering %s\n", __func__);

    mutex_lock(&pen->fb_lock);
    if (action == EBC_FB_BLANK) {
        pen->pen_suspend(pen);
    } else if (action == EBC_FB_UNBLANK) {
        pen->pen_resume(pen);
    }
    mutex_unlock(&pen->fb_lock);

    return NOTIFY_OK;
}

static int wacom_pen_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
    int ret = 0;
    struct wacom_pencil *wpen;

    wacom_dbg("Pen probe called for i2c 0x%02x\n", client->addr);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "check wacom i2c functionality error!!!\n");
        return -EIO;
    }

    wpen = kzalloc(sizeof(*wpen), GFP_KERNEL);
    if (!wpen) {
        return -ENOMEM;
    }
    wpen->client = client;

    ret = wacom_pen_of_probe(wpen);
    if (ret) {
        goto error_free_mem;
    }

    i2c_set_clientdata(client, wpen);

    ret = wacom_query_device(wpen);
    if (ret) {
        goto error_of_parase;
    }

    wacom_chip_power(client);
    ret = get_hid_desc(wpen);
    if (ret) {
        goto error_of_parase;
    }

    ret = wacom_pen_init_irq(client);
    if (ret) {
        goto error_of_parase;
    }

    ret = of_property_read_bool(client->dev.of_node, "wacom-support-wakeup");
    if (ret) {
        device_init_wakeup(&client->dev, true);
        wpen->pen_resume = wacom_pen_late_resume;
        wpen->pen_suspend = wacom_pen_early_suspend;
        memset(&wpen->fb_notif, 0, sizeof(struct notifier_block));
        wpen->fb_notif.notifier_call = fb_notifier_callback;
        mutex_init(&wpen->fb_lock);
        ebc_register_notifier(&wpen->fb_notif);
    }

    ret = input_init(wpen);
    if (ret) {
        goto error_irq_init;
    }

    // ret = hid_init(wpen);
    // if (ret) {
    //     goto error_input;
    // }

    return 0;

error_input:
    if (wpen->input) {
        input_free_device(wpen->input);
    }
error_irq_init:
    free_irq(client->irq, wpen);
    free_irq(wpen->pendet_irq, wpen);
error_of_parase:
    devm_gpio_free(&client->dev, wpen->irq_gpio);
    devm_gpio_free(&client->dev, wpen->detect_gpio);
    devm_gpio_free(&client->dev, wpen->reset_gpio);
error_free_mem:
    if (wpen) {
        kfree(wpen);
    }
    return -ENODEV;
}

static int wacom_pen_remove(struct i2c_client *client)
{
    struct wacom_pencil *wpen = i2c_get_clientdata(client);

    wacom_dbg("entering %s\n", __func__);

    //hid_destroy_device(wpen->hid);
    free_irq(client->irq, wpen);

    return 0;
}

static void wacom_pen_shutdown(struct i2c_client *client)
{
    struct wacom_pencil *wpen = i2c_get_clientdata(client);
    wacom_dbg("entering %s\n", __func__);

    free_irq(client->irq, wpen);
}

#ifdef CONFIG_PM
static int wacom_pen_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct wacom_pencil *wpen = i2c_get_clientdata(client);

    wacom_dbg("entering %s\n", __func__);

    disable_irq(client->irq);
    if (device_may_wakeup(dev)) {
        enable_irq_wake(wpen->pendet_irq);
    }

    return 0;
}

static int wacom_pen_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct wacom_pencil *wpen = i2c_get_clientdata(client);

    wacom_dbg("entering %s\n", __func__);

    enable_irq(client->irq);
    if (device_may_wakeup(dev)) {
        disable_irq_wake(wpen->pendet_irq);
    }

    return 0;
}
#endif

static const struct dev_pm_ops wacom_pen_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(wacom_pen_suspend, wacom_pen_resume)
};

static const struct i2c_device_id wacom_i2c_id[] = {
    { "wacom", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static const struct of_device_id wacom_dt_ids[] = {
    {
        .compatible = "wacom,10S12MI",
        .data = (void *) &wacom_i2c_id[0],
    }, {
        /* sentinel */
    }
};
MODULE_DEVICE_TABLE(of, wacom_dt_ids);

static struct i2c_driver wacom_pen_driver = {
    .driver = {
        .name = "wacom_pencil",
        .owner = THIS_MODULE,
        .pm = &wacom_pen_pm,
        .of_match_table = of_match_ptr(wacom_dt_ids),
    },

    .probe = wacom_pen_probe,
    .remove = wacom_pen_remove,
    .shutdown = wacom_pen_shutdown,
    .id_table = wacom_i2c_id,
};

static int __init wacom_pen_init(void)
{
    wacom_dbg("wacom pen init.\n");
    return i2c_add_driver(&wacom_pen_driver);
}
module_init(wacom_pen_init);

static void __exit wacom_pen_exit(void)
{
    wacom_dbg("wacom pen exit.\n");
    i2c_del_driver(&wacom_pen_driver);
}
module_exit(wacom_pen_exit);

MODULE_DESCRIPTION("wacom HID over I2C pen driver");
MODULE_AUTHOR("tower");
MODULE_LICENSE("GPL");
