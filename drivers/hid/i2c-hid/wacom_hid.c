/*
 * Wacom HID over I2C protocol implementation
 *
 * Copyright (c) 2023 HTFY, Inc
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

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
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/i2c-hid.h>
#include <linux/htfy_dbg.h> 
#include "../hid-ids.h"
#include "i2c-hid.h"

/* quirks to control the device */
#define WACOM_HID_QUIRK_SET_PWR_WAKEUP_DEV BIT(0)
#define WACOM_HID_QUIRK_NO_IRQ_AFTER_RESET BIT(1)
#define WACOM_HID_QUIRK_NO_RUNTIME_PM      BIT(2)
#define WACOM_HID_QUIRK_DELAY_AFTER_SLEEP  BIT(3)
#define WACOM_HID_QUIRK_BOGUS_IRQ          BIT(4)
#define WACOM_HID_QUIRK_RESET_ON_RESUME    BIT(5)
#define WACOM_HID_QUIRK_BAD_INPUT_SIZE     BIT(6)

/* flags */
#define WACOM_HID_STARTED       0
#define WACOM_HID_RESET_PENDING 1
#define WACOM_HID_READ_PENDING  2

#define WACOM_HID_PWR_ON    0x00
#define WACOM_HID_PWR_SLEEP 0x01

/* filter report id */
#define PEN_REPORT_ID              0x2
#define SHINONOME_REFILL_REPORT_ID 0x1A

/* debug option */
static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define wacom_hid_dbg(ihid, fmt, arg...) \
    do { \
        if (debug) \
            dev_printk(KERN_INFO, &(ihid)->client->dev, fmt, ##arg); \
    } while (0)

struct wacom_hid_desc {
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

struct wacom_hid_cmd {
    unsigned int registerIndex;
    __u8 opcode;
    unsigned int length;
    bool wait;
};

union command {
    u8 data[0];
    struct cmd {
        __le16 reg;
        __u8 reportTypeID;
        __u8 opcode;
    } __packed c;
};

#define WACOM_HID_CMD(opcode_) \
    .opcode = opcode_, \
    .length = 4, \
    .registerIndex = offsetof(struct wacom_hid_desc, wCommandRegister)

/* fetch HID descriptor */
static const struct wacom_hid_cmd hid_descr_cmd = {.length = 2};
/* fetch report descriptors */
static const struct wacom_hid_cmd hid_report_descr_cmd = {
    .registerIndex = offsetof(struct wacom_hid_desc, wReportDescRegister),
    .opcode = 0x00,
    .length = 2,
};
/* commands */
static const struct wacom_hid_cmd hid_reset_cmd = {WACOM_HID_CMD(0x01), .wait = true};
static const struct wacom_hid_cmd hid_get_report_cmd = {WACOM_HID_CMD(0x02)};
static const struct wacom_hid_cmd hid_set_report_cmd = {WACOM_HID_CMD(0x03)};
static const struct wacom_hid_cmd hid_set_power_cmd = {WACOM_HID_CMD(0x08)};
static const struct wacom_hid_cmd hid_no_cmd = {.length = 0};

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct wacom_hid_cmd hid_get_idle_cmd = { WACOM_HID_CMD(0x04) };
 * static const struct wacom_hid_cmd hid_set_idle_cmd = { WACOM_HID_CMD(0x05) };
 * static const struct wacom_hid_cmd hid_get_protocol_cmd = { WACOM_HID_CMD(0x06) };
 * static const struct wacom_hid_cmd hid_set_protocol_cmd = { WACOM_HID_CMD(0x07) };
 */

/* The main device structure */
struct wacom_hid {
    struct i2c_client *client; /* i2c client */
    struct hid_device *hid;    /* pointer to corresponding HID dev */
    union {
        __u8 hdesc_buffer[sizeof(struct wacom_hid_desc)];
        struct wacom_hid_desc hdesc; /* the HID Descriptor */
    };
    __le16 wHIDDescRegister;         /* location of the i2c register of the HID descriptor. */
    unsigned int bufsize;            /* i2c buffer size */
    u8 *inbuf;                       /* Input buffer */
    u8 *rawbuf;                      /* Raw Input buffer */
    u8 *cmdbuf;                      /* Command buffer */
    u8 *argsbuf;                     /* Command arguments buffer */
    unsigned long flags;             /* device flags */
    unsigned long quirks;            /* Various quirks */
    wait_queue_head_t wait;          /* For waiting the interrupt */
    struct i2c_hid_platform_data pdata;
    bool screen_off;
    struct mutex reset_lock;
    unsigned long sleep_delay;
    struct gpio_desc *gpiod_detect;
    int detect_irq;
    int detect_direction_in;
    struct gpio_desc *gpiod_rst;
#ifdef CONFIG_FB
    struct notifier_block fb_notifier;
    struct mutex fb_lock;
#endif
};

static const struct wacom_hid_quirks {
    __u16 idVendor;
    __u16 idProduct;
    __u32 quirks;
} wacom_hid_quirks[] = {
    {USB_VENDOR_ID_WEIDA, USB_DEVICE_ID_WEIDA_8752, WACOM_HID_QUIRK_SET_PWR_WAKEUP_DEV},
    {USB_VENDOR_ID_WEIDA, USB_DEVICE_ID_WEIDA_8755, WACOM_HID_QUIRK_SET_PWR_WAKEUP_DEV},
    {I2C_VENDOR_ID_HANTICK, I2C_PRODUCT_ID_HANTICK_5288, WACOM_HID_QUIRK_NO_IRQ_AFTER_RESET | WACOM_HID_QUIRK_NO_RUNTIME_PM},
    {I2C_VENDOR_ID_RAYDIUM, I2C_PRODUCT_ID_RAYDIUM_4B33, WACOM_HID_QUIRK_DELAY_AFTER_SLEEP},
    {USB_VENDOR_ID_LG, I2C_DEVICE_ID_LG_8001, WACOM_HID_QUIRK_NO_RUNTIME_PM},
    {USB_VENDOR_ID_ELAN, HID_ANY_ID, WACOM_HID_QUIRK_BOGUS_IRQ},
    {USB_VENDOR_ID_ALPS_JP, HID_ANY_ID, WACOM_HID_QUIRK_RESET_ON_RESUME},
    {I2C_VENDOR_ID_SYNAPTICS, I2C_PRODUCT_ID_SYNAPTICS_SYNA2393, WACOM_HID_QUIRK_RESET_ON_RESUME},
    {USB_VENDOR_ID_ITE, I2C_DEVICE_ID_ITE_LENOVO_LEGION_Y720, WACOM_HID_QUIRK_BAD_INPUT_SIZE},
    {0, 0},
};

/*
 * wacom_hid_lookup_quirk: return any quirks associated with a I2C HID device
 * @idVendor: the 16-bit vendor ID
 * @idProduct: the 16-bit product ID
 *
 * Returns: a u32 quirks value.
 */
static u32 wacom_hid_lookup_quirk(const u16 idVendor, const u16 idProduct)
{
    u32 quirks = 0;
    int n;

    for (n = 0; wacom_hid_quirks[n].idVendor; n++) {
        if (wacom_hid_quirks[n].idVendor == idVendor && (wacom_hid_quirks[n].idProduct == (__u16)HID_ANY_ID || wacom_hid_quirks[n].idProduct == idProduct)) {
            quirks = wacom_hid_quirks[n].quirks;
        }
    }
    return quirks;
}

static int __wacom_hid_command(struct i2c_client *client, const struct wacom_hid_cmd *command, u8 reportID, u8 reportType, u8 *args, int args_len, unsigned char *buf_recv, int data_len)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    union command *cmd = (union command *)ihid->cmdbuf;
    int ret;
    struct i2c_msg msg[2];
    int msg_num = 1;

    int length = command->length;
    bool wait = command->wait;
    unsigned int registerIndex = command->registerIndex;

    /* special case for hid_descr_cmd */
    if (command == &hid_descr_cmd) {
        cmd->c.reg = ihid->wHIDDescRegister;
    } else {
        cmd->data[0] = ihid->hdesc_buffer[registerIndex];
        cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
    }

    if (length > 2) {
        cmd->c.opcode = command->opcode;
        cmd->c.reportTypeID = reportID | reportType << 4;
    }

    memcpy(cmd->data + length, args, args_len);
    length += args_len;

    wacom_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

    msg[0].addr = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len = length;
    msg[0].buf = cmd->data;
    if (data_len > 0) {
        msg[1].addr = client->addr;
        msg[1].flags = client->flags & I2C_M_TEN;
        msg[1].flags |= I2C_M_RD;
        msg[1].len = data_len;
        msg[1].buf = buf_recv;
        msg_num = 2;
        set_bit(WACOM_HID_READ_PENDING, &ihid->flags);
    }

    if (wait) {
        set_bit(WACOM_HID_RESET_PENDING, &ihid->flags);
    }

    ret = i2c_transfer(client->adapter, msg, msg_num);

    if (data_len > 0) {
        clear_bit(WACOM_HID_READ_PENDING, &ihid->flags);
    }

    if (ret != msg_num) {
        return ret < 0 ? ret : -EIO;
    }

    ret = 0;

    if (wait && (ihid->quirks & WACOM_HID_QUIRK_NO_IRQ_AFTER_RESET)) {
        msleep(100);
    } else if (wait) {
        wacom_hid_dbg(ihid, "%s: waiting...\n", __func__);
        if (!wait_event_timeout(ihid->wait, !test_bit(WACOM_HID_RESET_PENDING, &ihid->flags), msecs_to_jiffies(5000))) {
            ret = -ENODATA;
        }
        wacom_hid_dbg(ihid, "%s: finished.\n", __func__);
    }

    return ret;
}

static int wacom_hid_command(struct i2c_client *client, const struct wacom_hid_cmd *command, unsigned char *buf_recv, int data_len)
{
    return __wacom_hid_command(client, command, 0, 0, NULL, 0, buf_recv, data_len);
}

static int wacom_hid_get_report(struct i2c_client *client, u8 reportType, u8 reportID, unsigned char *buf_recv, int data_len)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    u8 args[3];
    int ret;
    int args_len = 0;
    u16 readRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

    if (reportID >= 0x0F) {
        args[args_len++] = reportID;
        reportID = 0x0F;
    }

    args[args_len++] = readRegister & 0xFF;
    args[args_len++] = readRegister >> 8;

    ret = __wacom_hid_command(client, &hid_get_report_cmd, reportID, reportType, args, args_len, buf_recv, data_len);
    if (ret) {
        dev_err(&client->dev, "failed to retrieve report from device.\n");
        return ret;
    }

    return 0;
}

/**
 * wacom_hid_set_or_send_report: forward an incoming report to the device
 * @client: the i2c_client of the device
 * @reportType: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @reportID: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @len: size of buf
 * @use_data: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
static int wacom_hid_set_or_send_report(struct i2c_client *client, u8 reportType, u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    u8 *args = ihid->argsbuf;
    const struct wacom_hid_cmd *hidcmd;
    int ret;
    u16 dataRegister = le16_to_cpu(ihid->hdesc.wDataRegister);
    u16 outputRegister = le16_to_cpu(ihid->hdesc.wOutputRegister);
    u16 maxOutputLength = le16_to_cpu(ihid->hdesc.wMaxOutputLength);
    u16 size;
    int args_len;
    int index = 0;

    wacom_hid_dbg(ihid, "%s\n", __func__);

    if (data_len > ihid->bufsize) {
        return -EINVAL;
    }

    size = 2 /* size */ + (reportID ? 1 : 0) /* reportID */ + data_len /* buf */;
    args_len = (reportID >= 0x0F ? 1 : 0) /* optional third byte */ + 2 /* dataRegister */ + size /* args */;

    if (!use_data && maxOutputLength == 0) {
        return -ENOSYS;
    }

    if (reportID >= 0x0F) {
        args[index++] = reportID;
        reportID = 0x0F;
    }

    /*
     * use the data register for feature reports or if the device does not
     * support the output register
     */
    if (use_data) {
        args[index++] = dataRegister & 0xFF;
        args[index++] = dataRegister >> 8;
        hidcmd = &hid_set_report_cmd;
    } else {
        args[index++] = outputRegister & 0xFF;
        args[index++] = outputRegister >> 8;
        hidcmd = &hid_no_cmd;
    }

    args[index++] = size & 0xFF;
    args[index++] = size >> 8;

    if (reportID) {
        args[index++] = reportID;
    }

    memcpy(&args[index], buf, data_len);

    ret = __wacom_hid_command(client, hidcmd, reportID, reportType, args, args_len, NULL, 0);
    if (ret) {
        dev_err(&client->dev, "failed to set a report to device.\n");
        return ret;
    }

    return data_len;
}

static int wacom_hid_set_power(struct i2c_client *client, int power_state)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    int ret;
    unsigned long now, delay;

    /*
     * Some devices require to send a command to wakeup before power on.
     * The call will get a return value (EREMOTEIO) but device will be
     * triggered and activated. After that, it goes like a normal device.
     */
    if (power_state == WACOM_HID_PWR_ON && ihid->quirks & WACOM_HID_QUIRK_SET_PWR_WAKEUP_DEV) {
        ret = wacom_hid_command(client, &hid_set_power_cmd, NULL, 0);

        /* Device was already activated */
        if (!ret) {
            goto set_pwr_exit;
        }
    }

    if (ihid->quirks & WACOM_HID_QUIRK_DELAY_AFTER_SLEEP && power_state == WACOM_HID_PWR_ON) {
        now = jiffies;
        if (time_after(ihid->sleep_delay, now)) {
            delay = jiffies_to_usecs(ihid->sleep_delay - now);
            usleep_range(delay, delay + 1);
        }
    }

    ret = __wacom_hid_command(client, &hid_set_power_cmd, power_state, 0, NULL, 0, NULL, 0);

    if (ihid->quirks & WACOM_HID_QUIRK_DELAY_AFTER_SLEEP && power_state == WACOM_HID_PWR_SLEEP) {
        ihid->sleep_delay = jiffies + msecs_to_jiffies(20);
    }

    if (ret) {
        dev_err(&client->dev, "failed to change power setting.\n");
    }

set_pwr_exit:
    /*
     * The HID over I2C specification states that if a DEVICE needs time
     * after the PWR_ON request, it should utilise CLOCK stretching.
     * However, it has been observered that the Windows driver provides a
     * 1ms sleep between the PWR_ON and RESET requests.
     * According to Goodix Windows even waits 60 ms after (other?)
     * PWR_ON requests. Testing has confirmed that several devices
     * will not work properly without a delay after a PWR_ON request.
     */
    if (!ret && power_state == WACOM_HID_PWR_ON) {
        msleep(60);
    }

    return ret;
}

static int wacom_hid_hwreset(struct i2c_client *client)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    int ret;

    /*
     * This prevents sending feature reports while the device is
     * being reset. Otherwise we may lose the reset complete
     * interrupt.
     */
    mutex_lock(&ihid->reset_lock);

    ret = wacom_hid_set_power(client, WACOM_HID_PWR_ON);
    if (ret) {
        goto out_unlock;
    }

    wacom_hid_dbg(ihid, "resetting...\n");

    ret = wacom_hid_command(client, &hid_reset_cmd, NULL, 0);
    if (ret) {
        dev_err(&client->dev, "failed to reset device.\n");
        wacom_hid_set_power(client, WACOM_HID_PWR_SLEEP);
    }

out_unlock:
    mutex_unlock(&ihid->reset_lock);
    return ret;
}

static void wacom_hid_get_input(struct wacom_hid *ihid)
{
    int ret;
    u32 ret_size;
    int size = le16_to_cpu(ihid->hdesc.wMaxInputLength);

    if (size > ihid->bufsize) {
        size = ihid->bufsize;
    }

    ret = i2c_master_recv(ihid->client, ihid->inbuf, size);
    if (ret != size) {
        if (ret < 0) {
            return;
        }

        dev_err(&ihid->client->dev, "%s: got %d data instead of %d\n", __func__, ret, size);
        return;
    }

    ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;
    if (!ret_size) {
        /* host or device initiated RESET completed */
        if (test_and_clear_bit(WACOM_HID_RESET_PENDING, &ihid->flags)) {
            wake_up(&ihid->wait);
        }
        return;
    }

    if (ihid->quirks & WACOM_HID_QUIRK_BOGUS_IRQ && ret_size == 0xffff) {
        dev_warn_once(&ihid->client->dev, "%s: IRQ triggered but there's no data\n", __func__);
        return;
    }

    if ((ret_size > size) || (ret_size < 2)) {
        if (ihid->quirks & WACOM_HID_QUIRK_BAD_INPUT_SIZE) {
            ihid->inbuf[0] = size & 0xff;
            ihid->inbuf[1] = size >> 8;
            ret_size = size;
        } else {
            dev_err(&ihid->client->dev, "%s: incomplete report (%d/%d)\n", __func__, size, ret_size);
            return;
        }
    }

    //wacom_hid_dbg(ihid, "input: %*ph\n", ret_size, ihid->inbuf);

    if (test_bit(WACOM_HID_STARTED, &ihid->flags)) {
        if(ihid->inbuf[2] == 0x1a) ihid->inbuf[2] = 02;    // change 1a to 02. TEST ok.
        hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2, ret_size - 2, 1);
    }

    return;
}

static irqreturn_t wacom_hid_pendet_irq(int irq, void *dev_id)
{
    struct wacom_hid *ihid = dev_id;
    int pendet_gpio_value = gpiod_get_value(ihid->gpiod_detect);

    //wacom_hid_dbg(ihid, "detect irq value: %d\n", pendet_gpio_value);
    ebc_set_tp_power(pendet_gpio_value == ihid->detect_direction_in, 50);

    return IRQ_HANDLED;
}

static irqreturn_t wacom_hid_irq(int irq, void *dev_id)
{
    struct wacom_hid *ihid = dev_id;

    if (test_bit(WACOM_HID_READ_PENDING, &ihid->flags)) {
        return IRQ_HANDLED;
    }

    wacom_hid_get_input(ihid);

    return IRQ_HANDLED;
}

static int wacom_hid_get_report_length(struct hid_report *report)
{
    return ((report->size - 1) >> 3) + 1 + report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void wacom_hid_find_max_report(struct hid_device *hid, unsigned int type, unsigned int *max)
{
    struct hid_report *report;
    unsigned int size;

    /* We should not rely on wMaxInputLength, as some devices may set it to
     * a wrong length.
     */
    list_for_each_entry(report, &hid->report_enum[type].report_list, list)
    {
        size = wacom_hid_get_report_length(report);
        if (*max < size) {
            *max = size;
        }
    }
}

static void wacom_hid_free_buffers(struct wacom_hid *ihid)
{
    kfree(ihid->inbuf);
    kfree(ihid->rawbuf);
    kfree(ihid->argsbuf);
    kfree(ihid->cmdbuf);
    ihid->inbuf = NULL;
    ihid->rawbuf = NULL;
    ihid->cmdbuf = NULL;
    ihid->argsbuf = NULL;
    ihid->bufsize = 0;
}

static int wacom_hid_alloc_buffers(struct wacom_hid *ihid, size_t report_size)
{
    /* the worst case is computed from the set_report command with a
     * reportID > 15 and the maximum report length
     */
    int args_len = sizeof(__u8) + /* ReportID */ sizeof(__u8) + /* optional ReportID byte */ sizeof(__u16) + /* data register */ sizeof(__u16) + /* size of the report */ report_size; /* report */

    ihid->inbuf = kzalloc(report_size, GFP_KERNEL);
    ihid->rawbuf = kzalloc(report_size, GFP_KERNEL);
    ihid->argsbuf = kzalloc(args_len, GFP_KERNEL);
    ihid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

    if (!ihid->inbuf || !ihid->rawbuf || !ihid->argsbuf || !ihid->cmdbuf) {
        wacom_hid_free_buffers(ihid);
        return -ENOMEM;
    }

    ihid->bufsize = report_size;

    return 0;
}

static int wacom_hid_get_raw_report(struct hid_device *hid, unsigned char report_number, __u8 *buf, size_t count, unsigned char report_type)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    size_t ret_count, ask_count;
    int ret;

    if (report_type == HID_OUTPUT_REPORT) {
        return -EINVAL;
    }

    /* +2 bytes to include the size of the reply in the query buffer */
    ask_count = min(count + 2, (size_t)ihid->bufsize);

    ret = wacom_hid_get_report(client, report_type == HID_FEATURE_REPORT ? 0x03 : 0x01, report_number, ihid->rawbuf, ask_count);
    if (ret < 0) {
        return ret;
    }

    ret_count = ihid->rawbuf[0] | (ihid->rawbuf[1] << 8);
    if (ret_count <= 2) {
        return 0;
    }

    ret_count = min(ret_count, ask_count);
    /* The query buffer contains the size, dropping it in the reply */
    count = min(count, ret_count - 2);
    memcpy(buf, ihid->rawbuf + 2, count);

    return count;
}

static int wacom_hid_output_raw_report(struct hid_device *hid, __u8 *buf, size_t count, unsigned char report_type, bool use_data)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    int report_id = buf[0];
    int ret;

    if (report_type == HID_INPUT_REPORT) {
        return -EINVAL;
    }

    mutex_lock(&ihid->reset_lock);

    if (report_id) {
        buf++;
        count--;
    }

    ret = wacom_hid_set_or_send_report(client, report_type == HID_FEATURE_REPORT ? 0x03 : 0x02, report_id, buf, count, use_data);
    if (report_id && ret >= 0) {
        ret++; /* add report_id to the number of transfered bytes */
    }

    mutex_unlock(&ihid->reset_lock);

    return ret;
}

static int wacom_hid_output_report(struct hid_device *hid, __u8 *buf, size_t count)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "entering %s\n", __func__);

    return wacom_hid_output_raw_report(hid, buf, count, HID_OUTPUT_REPORT, false);
}

static int wacom_hid_raw_request(struct hid_device *hid, unsigned char reportnum, __u8 *buf, size_t len, unsigned char rtype, int reqtype)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "entering %s\n", __func__);

    switch (reqtype) {
        case HID_REQ_GET_REPORT:
            return wacom_hid_get_raw_report(hid, reportnum, buf, len, rtype);
        case HID_REQ_SET_REPORT:
            if (buf[0] != reportnum) {
                return -EINVAL;
            }
            return wacom_hid_output_raw_report(hid, buf, len, rtype, true);
        default:
            return -EIO;
    }
}

static int wacom_hid_parse(struct hid_device *hid)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    struct wacom_hid_desc *hdesc = &ihid->hdesc;
    unsigned int rsize;
    char *rdesc;
    int ret;
    int tries = 3;
    char *use_override;

    wacom_hid_dbg(ihid, "entering %s\n", __func__);

    rsize = le16_to_cpu(hdesc->wReportDescLength);
    if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
        dev_err(&(ihid)->client->dev, "weird size of report descriptor (%u)\n", rsize);
        return -EINVAL;
    }

    do {
        ret = wacom_hid_hwreset(client);
        if (ret) {
            msleep(1000);
        }
    } while (tries-- > 0 && ret);

    if (ret) {
        return ret;
    }

    use_override = i2c_hid_get_dmi_hid_report_desc_override(client->name, &rsize);
    if (use_override) {
        rdesc = use_override;
        wacom_hid_dbg(ihid, "Using a HID report descriptor override\n");
    } else {
        rdesc = kzalloc(rsize, GFP_KERNEL);
        if (!rdesc) {
            dev_err(&(ihid)->client->dev, "couldn't allocate rdesc memory\n");
            return -ENOMEM;
        }

        wacom_hid_dbg(ihid, "asking HID report descriptor\n");

        ret = wacom_hid_command(client, &hid_report_descr_cmd, rdesc, rsize);
        if (ret) {
            dev_err(&(ihid)->client->dev, "reading report descriptor failed\n");
            kfree(rdesc);
            return -EIO;
        }
    }

    wacom_hid_dbg(ihid, "Report Descriptor: %*ph\n", rsize, rdesc);

    ret = hid_parse_report(hid, rdesc, rsize);
    if (!use_override) {
        kfree(rdesc);
    }
    if (ret) {
        dev_err(&(ihid)->client->dev, "parsing report descriptor failed\n");
        return ret;
    }

    return 0;
}

static int wacom_hid_start(struct hid_device *hid)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    int ret;
    unsigned int bufsize = HID_MIN_BUFFER_SIZE;

    wacom_hid_dbg(ihid, "entering %s\n", __func__);

    wacom_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
    wacom_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
    wacom_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

    if (bufsize > ihid->bufsize) {
        disable_irq(client->irq);
        wacom_hid_free_buffers(ihid);

        ret = wacom_hid_alloc_buffers(ihid, bufsize);
        enable_irq(client->irq);

        if (ret) {
            return ret;
        }
    }

    return 0;
}

static void wacom_hid_stop(struct hid_device *hid)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "entering %s\n", __func__);
    hid->claimed = 0;
}

static int wacom_hid_open(struct hid_device *hid)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    int ret = 0;

    wacom_hid_dbg(ihid, "entering %s\n", __func__);
    ret = pm_runtime_get_sync(&client->dev);
    if (ret < 0) {
        return ret;
    }

    set_bit(WACOM_HID_STARTED, &ihid->flags);
    return 0;
}

static void wacom_hid_close(struct hid_device *hid)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "entering %s\n", __func__);
    clear_bit(WACOM_HID_STARTED, &ihid->flags);

    /* Save some power */
    pm_runtime_put(&client->dev);
}

static int wacom_hid_power(struct hid_device *hid, int lvl)
{
    struct i2c_client *client = hid->driver_data;
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "entering %s lvl:%d\n", __func__, lvl);

    switch (lvl) {
        case PM_HINT_FULLON:
            pm_runtime_get_sync(&client->dev);
            break;
        case PM_HINT_NORMAL:
            pm_runtime_put(&client->dev);
            break;
    }
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

#ifdef CONFIG_FB
static int wacom_hid_early_suspend(struct wacom_hid *ihid)
{
    struct i2c_client *client = ihid->client;
    struct hid_device *hid = ihid->hid;
    int ret;

    wacom_hid_dbg(ihid, "**entering %s\n", __func__);

    if (hid->driver && hid->driver->suspend) {
        /*
		 * Wake up the device so that IO issues in
		 * HID driver's suspend code can succeed.
		 */
        ret = pm_runtime_resume(&client->dev);
        if (ret < 0) {
            return ret;
        }

        ret = hid->driver->suspend(hid, PMSG_SUSPEND);
        if (ret < 0) {
            return ret;
        }
    }

    if (!pm_runtime_suspended(&client->dev)) {
        /* Save some power */
        wacom_hid_set_power(client, WACOM_HID_PWR_SLEEP);
        disable_irq(client->irq);
    }

    if (!device_may_wakeup(&client->dev)) {
        regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);
    }

    return 0;
}

static int wacom_hid_late_resume(struct wacom_hid *ihid)
{
    struct i2c_client *client = ihid->client;
    struct hid_device *hid = ihid->hid;
    int ret;

    wacom_hid_dbg(ihid, "**entering %s\n", __func__);

    if (!device_may_wakeup(&client->dev)) {
        ret = regulator_bulk_enable(ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);
        if (ret) {
            hid_warn(hid, "Failed to enable supplies: %d\n", ret);
        }

        if (ihid->pdata.post_power_delay_ms) {
            msleep(ihid->pdata.post_power_delay_ms);
        }
    }

    /* We'll resume to full power */
    pm_runtime_disable(&client->dev);
    pm_runtime_set_active(&client->dev);
    pm_runtime_enable(&client->dev);

    enable_irq(client->irq);

    /* Instead of resetting device, simply powers the device on. This
     * solves "incomplete reports" on Raydium devices 2386:3118 and
     * 2386:4B33 and fixes various SIS touchscreens no longer sending
     * data after a suspend/resume.
     *
     * However some ALPS touchpads generate IRQ storm without reset, so
     * let's still reset them here.
     */
    if (ihid->quirks & WACOM_HID_QUIRK_RESET_ON_RESUME) {
        ret = wacom_hid_hwreset(client);
    } else {
        ret = wacom_hid_set_power(client, WACOM_HID_PWR_ON);
    }
    if (ret) {
        return ret;
    }

    if (hid->driver && hid->driver->reset_resume) {
        ret = hid->driver->reset_resume(hid);
        return ret;
    }

    return 0;
}

static int wacom_hid_fb_notifier_callback(struct notifier_block *self, unsigned long action, void *data)
{
    struct wacom_hid *ihid;

    ihid = container_of(self, struct wacom_hid, fb_notifier);

    wacom_hid_dbg(ihid, "entering %s, action=%d\n", __func__, action);

    mutex_lock(&ihid->fb_lock);
    if (action == EINK_NOTIFY_EVENT_SCREEN_OFF) {
        if (!ihid->screen_off) {
            wacom_hid_early_suspend(ihid);
            ihid->screen_off = true;
        }
    } else if (action == EINK_NOTIFY_EVENT_SCREEN_ON) {
        if (ihid->screen_off) {
            ihid->screen_off = false;
            wacom_hid_late_resume(ihid);
        }
    }
    mutex_unlock(&ihid->fb_lock);

    return NOTIFY_OK;
}
#endif

static int wacom_hid_init_dev(struct i2c_client *client)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    unsigned long irqflags = 0;
    int ret;

    wacom_hid_dbg(ihid, "init hid dev.");

    // detect irq
    ihid->gpiod_detect = devm_gpiod_get_optional(&client->dev, "detect", GPIOD_IN);
    if (IS_ERR(ihid->gpiod_detect)) {
        dev_err(&client->dev, "Bad HID detect pin.\n");
        return PTR_ERR(ihid->gpiod_detect);
    }
    ihid->detect_direction_in = 0;
    ihid->detect_irq = gpiod_to_irq(ihid->gpiod_detect);
    if (ihid->detect_irq < 0) {
        dev_err(&client->dev, "HID unable to request IRQ for detect gpio!\n");
        return -EINVAL;
    }

    ret = request_threaded_irq(ihid->detect_irq, NULL, wacom_hid_pendet_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "wacom-pendet", ihid);
    if (ret < 0) {
        dev_err(&client->dev, "Could not register for %s interrupt, irq=%d, ret=%d\n", "wacom-pendet", ihid->detect_irq, ret);
        return ret;
    }

    // rst 设置默认电平
    ihid->gpiod_rst = devm_gpiod_get_optional(&client->dev, "rst", GPIOD_OUT_LOW);
    if (IS_ERR(ihid->gpiod_rst)) {
        dev_err(&client->dev, "Bad HID rst pin.\n");
        return PTR_ERR(ihid->gpiod_rst);
    }

    // data irq
    if (!irq_get_trigger_type(client->irq)) {
        irqflags = IRQF_TRIGGER_LOW;
    }

    ret = request_threaded_irq(client->irq, NULL, wacom_hid_irq, irqflags | IRQF_ONESHOT, client->name, ihid);
    if (ret < 0) {
        dev_err(&client->dev, "Could not register for %s interrupt, irq = %d, ret = %d\n", client->name, client->irq, ret);
        return ret;
    }

    // wakeup
    ret = of_property_read_bool(client->dev.of_node, "wacom-support-wakeup");
    if (ret) {
        device_init_wakeup(&client->dev, true);
    }

    return 0;
}

static int wacom_hid_fetch_hid_descriptor(struct wacom_hid *ihid)
{
    struct i2c_client *client = ihid->client;
    struct wacom_hid_desc *hdesc = &ihid->hdesc;
    unsigned int dsize;
    int ret;

    /* i2c hid fetch using a fixed descriptor size (30 bytes) */
    if (i2c_hid_get_dmi_i2c_hid_desc_override(client->name)) {
        wacom_hid_dbg(ihid, "Using a HID descriptor override\n");
        ihid->hdesc = *((struct wacom_hid_desc *)i2c_hid_get_dmi_i2c_hid_desc_override(client->name));
    } else {
        wacom_hid_dbg(ihid, "Fetching the HID descriptor\n");
        ret = wacom_hid_command(client, &hid_descr_cmd, ihid->hdesc_buffer, sizeof(struct wacom_hid_desc));
        if (ret) {
            dev_err(&client->dev, "hid_descr_cmd failed\n");
            return -ENODEV;
        }
    }

    /* Validate the length of HID descriptor, the 4 first bytes:
     * bytes 0-1 -> length
     * bytes 2-3 -> bcdVersion (has to be 1.00)
     */
    /* check bcdVersion == 1.0 */
    if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
        dev_err(&client->dev, "unexpected HID descriptor bcdVersion (0x%04hx)\n", le16_to_cpu(hdesc->bcdVersion));
        return -ENODEV;
    }

    /* Descriptor length should be 30 bytes as per the specification */
    dsize = le16_to_cpu(hdesc->wHIDDescLength);
    if (dsize != sizeof(struct wacom_hid_desc)) {
        dev_err(&client->dev, "weird size of HID descriptor (%u)\n", dsize);
        return -ENODEV;
    }
    wacom_hid_dbg(ihid, "HID Descriptor: %*ph\n", dsize, ihid->hdesc_buffer);
    return 0;
}

#ifdef CONFIG_OF
static int wacom_hid_of_probe(struct i2c_client *client, struct i2c_hid_platform_data *pdata)
{
    struct device *dev = &client->dev;
    u32 val;
    int ret;
    struct gpio_desc *gpiodx;

    ret = of_property_read_u32(dev->of_node, "hid-descr-addr", &val);
    if (ret) {
        dev_err(&client->dev, "HID register address not provided\n");
        return -ENODEV;
    }
    if (val >> 16) {
        dev_err(&client->dev, "Bad HID register address: 0x%08x\n", val);
        return -EINVAL;
    }
    pdata->hid_descriptor_address = val;

    gpiodx = devm_gpiod_get_optional(&client->dev, "int", GPIOD_IN);
    if (IS_ERR(gpiodx)) {
        dev_err(&client->dev, "Bad HID int pin.\n");
        return PTR_ERR(gpiodx);
    }
    client->irq = gpiod_to_irq(gpiodx);
    if (client->irq < 0) {
        dev_err(&client->dev, "HID unable to request IRQ for INT gpio!\n");
        return -EINVAL;
    }

    return 0;
}

static const struct of_device_id wacom_hid_of_match[] = {
    {.compatible = "wacom,hidx"},
    {},
};
MODULE_DEVICE_TABLE(of, wacom_hid_of_match);
#else
static inline int wacom_hid_of_probe(struct i2c_client *client, struct i2c_hid_platform_data *pdata)
{
    return -ENODEV;
}
#endif

static void wacom_hid_fwnode_probe(struct i2c_client *client, struct i2c_hid_platform_data *pdata)
{
    u32 val;

    if (!device_property_read_u32(&client->dev, "post-power-on-delay-ms", &val)) {
        pdata->post_power_delay_ms = val;
    }
}

static int wacom_hid_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
    int ret;
    struct wacom_hid *ihid;
    struct hid_device *hid;
    __u16 hidRegister;
    struct i2c_hid_platform_data *platform_data = client->dev.platform_data;

    dev_info(&client->dev, "HID probe called for i2c 0x%02x\n", client->addr);

    ihid = devm_kzalloc(&client->dev, sizeof(*ihid), GFP_KERNEL);
    if (!ihid) {
        return -ENOMEM;
    }

    if (client->dev.of_node) {
        ret = wacom_hid_of_probe(client, &ihid->pdata);
        if (ret) {
            return ret;
        }
    } else if (platform_data) {
        ihid->pdata = *platform_data;
    } else {
        dev_err(&client->dev, "HID can not parase of note get device info.\n");
        return -EINVAL;
    }

    /* Parse platform agnostic common properties from ACPI / device tree */
    wacom_hid_fwnode_probe(client, &ihid->pdata);

    ihid->pdata.supplies[0].supply = "vdd";
    ihid->pdata.supplies[1].supply = "vddl";

    ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);
    if (ret) {
        return ret;
    }

    ret = regulator_bulk_enable(ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);
    if (ret < 0) {
        return ret;
    }

    if (ihid->pdata.post_power_delay_ms) {
        msleep(ihid->pdata.post_power_delay_ms);
    }

    i2c_set_clientdata(client, ihid);

    ihid->client = client;

    hidRegister = ihid->pdata.hid_descriptor_address;
    ihid->wHIDDescRegister = cpu_to_le16(hidRegister);

    init_waitqueue_head(&ihid->wait);
    mutex_init(&ihid->reset_lock);

    /* we need to allocate the command buffer without knowing the maximum
     * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
     * real computation later.
     */
    ret = wacom_hid_alloc_buffers(ihid, HID_MIN_BUFFER_SIZE);
    if (ret < 0) {
        goto err_regulator;
    }

    pm_runtime_get_noresume(&client->dev);
    pm_runtime_set_active(&client->dev);
    pm_runtime_enable(&client->dev);

    /* Make sure there is something at this address */
    ret = i2c_smbus_read_byte(client);
    if (ret < 0) {
        dev_dbg(&client->dev, "nothing at this address: %d\n", ret);
        ret = -ENXIO;
        goto err_pm;
    }

    ret = wacom_hid_fetch_hid_descriptor(ihid);
    if (ret < 0) {
        goto err_pm;
    }

    ret = wacom_hid_init_dev(client);
    if (ret < 0) {
        goto err_pm;
    }

    hid = hid_allocate_device();
    if (IS_ERR(hid)) {
        ret = PTR_ERR(hid);
        goto err_irq;
    }

#ifdef CONFIG_FB
    memset(&ihid->fb_notifier, 0, sizeof(struct notifier_block));
    ihid->fb_notifier.notifier_call = wacom_hid_fb_notifier_callback;
    mutex_init(&ihid->fb_lock);
    htfy_ebc_register_notifier(&ihid->fb_notifier);
#endif

    ihid->hid = hid;

    hid->driver_data = client;
    hid->ll_driver = &wacom_hid_ll_driver;
    hid->dev.parent = &client->dev;
    hid->bus = BUS_I2C;
    hid->version = le16_to_cpu(ihid->hdesc.wVersionID);
    hid->vendor = le16_to_cpu(ihid->hdesc.wVendorID);
    hid->product = le16_to_cpu(ihid->hdesc.wProductID);

    snprintf(hid->name, sizeof(hid->name), "%s", "Wacom-pen");
    strlcpy(hid->phys, dev_name(&client->dev), sizeof(hid->phys));

    ihid->quirks = wacom_hid_lookup_quirk(hid->vendor, hid->product);

    ret = hid_add_device(hid);
    if (ret) {
        if (ret != -ENODEV) {
            hid_err(client, "can't add hid device: %d\n", ret);
        }
        goto err_mem_free;
    }

    if (!(ihid->quirks & WACOM_HID_QUIRK_NO_RUNTIME_PM)) {
        pm_runtime_put(&client->dev);
    }

    return 0;

err_mem_free:
    hid_destroy_device(hid);

err_irq:
    free_irq(client->irq, ihid);
    free_irq(ihid->detect_irq, ihid);

err_pm:
    pm_runtime_put_noidle(&client->dev);
    pm_runtime_disable(&client->dev);

err_regulator:
    regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);
    wacom_hid_free_buffers(ihid);
    return ret;
}

static int wacom_hid_remove(struct i2c_client *client)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);
    struct hid_device *hid;

    if (!(ihid->quirks & WACOM_HID_QUIRK_NO_RUNTIME_PM)) {
        pm_runtime_get_sync(&client->dev);
    }
    pm_runtime_disable(&client->dev);
    pm_runtime_set_suspended(&client->dev);
    pm_runtime_put_noidle(&client->dev);

    hid = ihid->hid;
    hid_destroy_device(hid);

    free_irq(client->irq, ihid);
    free_irq(ihid->detect_irq, ihid);

    if (ihid->bufsize) {
        wacom_hid_free_buffers(ihid);
    }

    regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);

    return 0;
}

static void wacom_hid_shutdown(struct i2c_client *client)
{
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_set_power(client, WACOM_HID_PWR_SLEEP);
    free_irq(client->irq, ihid);
    free_irq(ihid->detect_irq, ihid);
}

#ifdef CONFIG_PM_SLEEP
static int wacom_hid_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "%s\n", __func__);

    if (!ihid->screen_off) {
        if (device_may_wakeup(dev)) {
            enable_irq_wake(ihid->detect_irq);
        }
    }

    return 0;
}

static int wacom_hid_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct wacom_hid *ihid = i2c_get_clientdata(client);

    wacom_hid_dbg(ihid, "%s\n", __func__);

    if (!ihid->screen_off) {
        if (device_may_wakeup(dev)) {
            disable_irq_wake(ihid->detect_irq);
        }
    }

    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(wacom_hid_pm, wacom_hid_suspend, wacom_hid_resume);

static const struct i2c_device_id wacom_hid_id_table[] = {
    {"wacom,hidx", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, wacom_hid_id_table);

static struct i2c_driver wacom_hid_driver = {
    .driver = {
        .name = "wacom_hidx",
        .pm = &wacom_hid_pm,
        .of_match_table = of_match_ptr(wacom_hid_of_match),
    },

    .probe = wacom_hid_probe,
    .remove = wacom_hid_remove,
    .shutdown = wacom_hid_shutdown,
    .id_table = wacom_hid_id_table,
};

module_i2c_driver(wacom_hid_driver);

MODULE_DESCRIPTION("Wacom HID driver");
MODULE_AUTHOR("Tower");
MODULE_LICENSE("GPL");
