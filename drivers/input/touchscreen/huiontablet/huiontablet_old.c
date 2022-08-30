/* 
 * 
 * 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version: 2.2
 * Authors: 
 * Release Date: 2014/01/14
 * Revision record:
 *      V1.0:   
 *          first Release. By Andrew, 2012/08/31 
 *      V1.2:
 *          modify huion_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6: 
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup 
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5) 
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification 
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 *      V2.2:
 *          1. gt9xx_config for debug
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *                  By Meta, 2014/01/14
 */

#include <linux/irq.h>
#include "huiontablet.h"
#include "huiontablet_cfg.h"
#include <linux/regulator/consumer.h>

static u8 huion_change_x2y = TRUE;
static u8 huion_x_reverse = TRUE;
static u8 huion_y_reverse = FALSE;

static const char *huion_ts_name = "huion-ts";
static struct workqueue_struct *huion_wq;
struct i2c_client *i2c_connect_client_hn = NULL;

void huion_reset_guitar(struct i2c_client *client, s32 ms);
void huion_int_sync(s32 ms, struct huion_ts_data *ts);

#if GTP_AUTO_UPDATE
extern u8 hup_init_update_proc(struct huion_ts_data *);
#endif

int ghuion_i2c_rxdata(struct i2c_client *client, char *rxdata, int length, u8 cmd)
{
    int ret;
    //u8 read_cmd[1] = {cmd};

    struct i2c_msg msgs[1] = {
        /*{
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = read_cmd,
        },*/
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxdata,
        },
    };

    //printk("IIC addr = %x\n",this_client->addr);
    ret = i2c_transfer(client->adapter, msgs, 1);
    if (ret < 0)
        printk("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

int huion_i2c_rxdata_cmd(struct i2c_client *client, char *rxdata, int length, u8 cmd)
{
    int ret;
    u8 read_cmd[1] = {cmd};

    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = read_cmd,
        },
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = rxdata,
        },
    };

    //printk("IIC addr = %x\n",this_client->addr);
    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0)
        printk("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

int ghuion_i2c_rxdata01(struct i2c_client *client, char *rxdata, int length, u8 cmd)
{
    int ret;
  
    struct i2c_msg msgs[1] = {
        {
            .addr = client->addr,
            .flags = I2C_M_RD, //|I2C_M_NO_RD_ACK,//|I2C_M_IGNORE_NAK,
            .len = length,
            .buf = rxdata,
        },
    };

    //printk("IIC addr = %x\n",client->addr);
    ret = i2c_transfer(client->adapter, msgs, 1);
    if (ret < 0)
        printk("msg %s i2c read error ret:%d\n", __func__, ret);

    return ret;
}

int ghuion_i2c_wxdata(struct i2c_client *client, char *txdata, int length)
{
    int ret;
    // txdata[0], cmd;
    // txdata[1],  txdata;

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    ret = i2c_transfer(client->adapter, msg, 1);
    if (ret < 0)
        //printk("%s i2c write error: %d\n", __func__, ret);
        printk("%s i2c write error: %d client->addr: %x\n", __func__, ret, client->addr);

    return ret;
}


/*******************************************************
Function:
    Disable irq function
Input:
    ts:  i2c_client private data
Output:
    None.
*********************************************************/
void huion_irq_disable(struct huion_ts_data *ts)
{
    unsigned long irqflags;

    //GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1;
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts:  i2c_client private data
Output:
    None.
*********************************************************/
void huion_irq_enable(struct huion_ts_data *ts)
{
    unsigned long irqflags = 0;

    //GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable)
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Report touch point event 
Input:
    ts:  i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void huion_touch_down(struct huion_ts_data *ts, s32 id, s32 x, s32 y, s32 w)
{
    printk("000 x_max:%d, y_max:%d, x:%d, y:%d w:%d\n", ts->abs_x_max,ts->abs_y_max, x, y,w);

    if (huion_change_x2y)
        GTP_SWAP(x, y);

    printk(" x:%d, y:%d w:%d\n", ts->abs_x_max,ts->abs_y_max, x, y,w);

    if (huion_x_reverse)
        x = ts->abs_x_max - x;

    if (huion_y_reverse)
        y = ts->abs_y_max - y;

   
    x = 10;
	y = 10;
	
    x = 1872-100;
	y = 1404-100;

	y = 1872/2-20;
	x = 1404/2-20;

	x = 733;
	y = 899;
	
	printk("ID:%d, X:%d, Y:%d, W:%d\n", id, x, y, w);

    //input_report_abs(ts->input_dev, ABS_X, x);
    //input_report_abs(ts->input_dev, ABS_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    
    input_report_abs(ts->input_dev, ABS_PRESSURE, w); // w is not 0, mouse move;
    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
    input_sync(ts->input_dev);

    //GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts:  i2c_client private data
Output:
    None.
*********************************************************/
static void huion_touch_up(struct huion_ts_data *ts, s32 id)
{
    input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
    input_sync(ts->input_dev);
}

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of huion_workqueue
Output:
    None.
*********************************************************/
static void huion_ts_work_func(struct work_struct *work)
{
    #define HUION_EMR_DATA_LEN 12
    u8 touch_num = 0;
    u8 finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
    u8 key_value = 0;
    //u8 *coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
	long org_x,org_y;
    s32 input_w = 0;
    s32 id = 0;
    s32 i = 0;
    //s32 ret = -1;
    struct huion_ts_data *ts = NULL;
    char huibuf[HUION_EMR_DATA_LEN];
    //char versionbuf[8];

    //printk("huion inter work\n");
    //GTP_DEBUG_FUNC();
    ts = container_of(work, struct huion_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }

    ghuion_i2c_rxdata(ts->client, huibuf, HUION_EMR_DATA_LEN, 0x0);
    printk("huion_ts_work_func R %x %x %x %x %x %x %x %x \n", huibuf[0], huibuf[1], huibuf[2], huibuf[3], huibuf[4], huibuf[5], huibuf[6], huibuf[7]);

    //ghuion_i2c_wxdata
    //char readversion[8] = {0x57,0xde,0x00,0x00,0x00,0x00,0x00,0x00};
    //ghuion_i2c_wxdata(ts->client, readversion, 2);
    //ghuion_i2c_rxdata(ts->client, versionbuf, 8, 0x0);
    //printk("huion_ts_work_func Read version %x %x %x %x %x %x %x %x \n",versionbuf[0],versionbuf[1],versionbuf[2],versionbuf[3],versionbuf[4],versionbuf[5],versionbuf[6],versionbuf[7]);

    //ghuion_i2c_wxdata(ts->client, readversion, 8);
    //ghuion_i2c_rxdata(ts->client, versionbuf, 8, 0x0);
    //printk("huion_ts_work_func Read version %x %x %x %x %x %x %x %x \n",versionbuf[0],versionbuf[1],versionbuf[2],versionbuf[3],versionbuf[4],versionbuf[5],versionbuf[6],versionbuf[7]);

    if ((huibuf[1] == 0x11) || (huibuf[1] == 0x10)) // 0x00, 0x10, 0x11
        finger = 1;
    else
        finger = 0;

    touch_num = finger & 0x0f;
    //printk("touch num %d\n",touch_num);

    pre_key = key_value;

    //printk("huion report touch num %d \n",touch_num);
    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {

            id = huibuf[0];
            input_x = huibuf[2] | (huibuf[3] << 8);
            input_y = huibuf[4] | (huibuf[5] << 8);
            input_w = huibuf[6] | (huibuf[7] << 8);
//GTP_DEBUG("x,y,w %d %d %d\n", input_x, input_y, input_w);
			printk("org: x:%d y:%d d:%d\n", input_x, input_y, input_w);
			printk("xmap:%d ymap:%d \n", TOUCH_X_AXIS_MAPPING, TOUCH_Y_AXIS_MAPPING);
			

#if SCREEN_MAP_SWITCH
            //input_x = (input_x / (TOUCH_X_AXIS_MAPPING));
            //input_y = (input_y / (TOUCH_Y_AXIS_MAPPING));
		//input_x = ((input_x * DISPLAY_SCREEN_MAX_X)/ TOUCH_SCREEN_MAX_X);
		//input_y = ((input_y * DISPLAY_SCREEN_MAX_Y)/ TOUCH_SCREEN_MAX_Y);

		//input_x = ((input_x * DISPLAY_SCREEN_MAX_X*10)/ TOUCH_SCREEN_MAX_X);
		//input_y = ((input_y * DISPLAY_SCREEN_MAX_Y*10)/ TOUCH_SCREEN_MAX_Y);

		org_x = input_x*100;
		org_x = (org_x*DISPLAY_SCREEN_MAX_X)/TOUCH_SCREEN_MAX_X;
		input_x = org_x/100;

		org_y = input_y*100;
		org_y = (org_y*DISPLAY_SCREEN_MAX_Y)/TOUCH_SCREEN_MAX_Y;

		input_y = org_y/100;
		
		
		printk("after: x:%d y:%d d:%d\n", input_x, input_y, input_w);
		
		
			
#endif
            {
                huion_touch_down(ts, id, input_x, input_y, input_w);
            }
        }
    }
    else if (pre_touch)
    {
        {
            //GTP_DEBUG("Touch Release!");
            huion_touch_up(ts, 0);
        }
    }

    pre_touch = touch_num;

    {
        input_sync(ts->input_dev);
    }

//exit_work_func:
    if (!ts->huion_rawdiff_mode)
    {
    }
    if (ts->use_irq)
    {
        huion_irq_enable(ts);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode. 
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart huion_ts_timer_handler(struct hrtimer *timer)
{
    struct huion_ts_data *ts = container_of(timer, struct huion_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(huion_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t huion_ts_irq_handler(int irq, void *dev_id)
{
    struct huion_ts_data *ts = dev_id;

    //GTP_DEBUG_FUNC();

    huion_irq_disable(ts);

    queue_work(huion_wq, &ts->work);

    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void huion_int_sync(s32 ms, struct huion_ts_data *ts)
{
    GTP_GPIO_OUTPUT(ts->irq_pin, 0);
    msleep(ms);
    gpio_direction_input(ts->irq_pin);
}

/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void huion_reset_guitar(struct i2c_client *client, s32 ms)
{

    struct huion_ts_data *ts = i2c_get_clientdata(client);
    printk("huion reset\n");
    //return;
    GTP_DEBUG_FUNC();
    GTP_INFO("Guitar reset");
    GTP_GPIO_OUTPUT(ts->rst_pin, 0); // begin select I2C slave addr
    msleep(ms);                      // T2: > 10ms
    GTP_GPIO_OUTPUT(ts->irq_pin, client->addr == 0x08);

    msleep(2); // T3: > 100us
    GTP_GPIO_OUTPUT(ts->rst_pin, 1);

    msleep(6); // T4: > 5ms

    gpio_direction_input(ts->rst_pin);

    huion_int_sync(50, ts);
}

void ghuion_reset_fw(struct i2c_client *client, s32 ms)
{
    struct huion_ts_data *ts = i2c_get_clientdata(client);

    printk("huion reset fw\n");
    //return;

    GTP_GPIO_OUTPUT(ts->rst_pin, 0); // begin select I2C slave addr
    msleep(ms);                      // T2: > 10ms
    GTP_GPIO_OUTPUT(ts->rst_pin, 1);

    /*while(1)
    {
        GTP_GPIO_OUTPUT(ts->rst_pin, 0);
        //GTP_GPIO_OUTPUT(ts->irq_pin, 0);
        msleep(60);
        GTP_GPIO_OUTPUT(ts->rst_pin, 1);
        //GTP_GPIO_OUTPUT(ts->irq_pin, 1);
        msleep(60);
    }*/
}
unsigned int gget_fw_status(struct i2c_client *client)
{
#define INT_PIN_GPIO_4_C_0 0x01 //the int pin is PORT4_C_0
    int pinstatus = 0;
    struct huion_ts_data *ts = i2c_get_clientdata(client);

    //printk("get_int_status\n");

    pinstatus = GTP_GPIO_GET_VALUE(ts->irq_pin); //

    //printk("get_int_status %08X\n",pinstatus);

    return pinstatus & INT_PIN_GPIO_4_C_0;

    /*while(1)
    {
        GTP_GPIO_OUTPUT(ts->rst_pin, 0);
        //GTP_GPIO_OUTPUT(ts->irq_pin, 0);
        msleep(60);
        GTP_GPIO_OUTPUT(ts->rst_pin, 1);
        //GTP_GPIO_OUTPUT(ts->irq_pin, 1);
        msleep(60);
    }*/
}

/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
s8 huion_fw_enter_sleep(struct huion_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {0x55, 0xDE, 0x55};

    GTP_DEBUG_FUNC();

    GTP_GPIO_OUTPUT(ts->irq_pin, 0);
    msleep(5);

    while (retry++ < 5)
    {
        ret = ghuion_i2c_wxdata(ts->client, i2c_control_buf, 3);

        if (ret > 0)
        {
            printk("GTP enter sleep!");

            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}

s8 huion_fw_wakeup_sleep(struct huion_ts_data *ts)
{
    u8 retry = 0;
    s8 ret = -1;

    u8 i2c_control_buf[3] = {0x5A, 0xDE, 0x5A};
    GTP_DEBUG_FUNC();

    while (retry++ < 10)
    {

        GTP_GPIO_OUTPUT(ts->irq_pin, 1);
        msleep(5);
        ret = ghuion_i2c_wxdata(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            printk("wake up sleep\n");
            return ret;
        }
        huion_reset_guitar(ts->client, 20);
    }

    GTP_ERROR("GTP wakeup sleep failed.");

    return ret;
}

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
s8 huion_wakeup_sleep(struct huion_ts_data *ts)
{
    s8 ret = 0;
    return ret;
}


/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 huion_request_io_port(struct huion_ts_data *ts)
{
    s32 ret = 0;

    GTP_DEBUG_FUNC();
    //ret = GTP_GPIO_REQUEST(ts->rst_pin, "HN_RST_PORT");
    //printk("ts->rst_pin:%d",ts->rst_pin);
    ret = gpio_request(ts->rst_pin, "HN_RST_PORT");
    //printk("ts->rst_pin:%d",ts->rst_pin);
    
    if (ret < 0)
    {
        GTP_ERROR("2Failed to request GPIO:%d, ERRNO:%d", (s32)ts->rst_pin, ret);
        GTP_GPIO_FREE(ts->rst_pin);
        return -ENODEV;
    }

    //printk("ts->rst_pin 000: %d",ts->rst_pin);
    
    ret = GTP_GPIO_REQUEST(ts->irq_pin, "HN_INT_IRQ");
    if (ret < 0)
    {
        GTP_ERROR("3Failed to request GPIO:%d, ERRNO:%d", (s32)ts->irq_pin, ret);
        GTP_GPIO_FREE(ts->irq_pin);
        return -ENODEV;
    }
    else
    {
        //printk("ts->rst_pin222:%d",ts->rst_pin);
    
        //GTP_GPIO_AS_INT(GTP_INT_PORT);
        gpio_direction_input(ts->irq_pin);

        //ts->client->irq = ts->irq_pin;
    }

    //printk("ts->rst_pin111:%d",ts->rst_pin);
    

    //GTP_GPIO_AS_INPUT(ts->rst_pin);
    gpio_direction_input(ts->rst_pin);
    //s3c_gpio_setpull(pin, S3C_GPIO_PULL_NONE);
    //printk("ts->rst_pin333:%d",ts->rst_pin);
    //huion_reset_guitar(ts->client, 20);

    return ret;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 huion_request_irq(struct huion_ts_data *ts)
{
    s32 ret = -1;

    GTP_DEBUG_FUNC();
    GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

    ts->irq = gpio_to_irq(ts->irq_pin); //If not defined in client
    if (ts->irq)
    {
        ts->client->irq = ts->irq;
        printk("ts->irq is %d\n", ts->irq);
        printk("ts->irq_flags is %d\n", (int)ts->irq_flags);

        ret = devm_request_threaded_irq(&(ts->client->dev), ts->irq, NULL,
                                        huion_ts_irq_handler, ts->irq_flags | IRQF_ONESHOT /*irq_table[ts->int_trigger_type]*/,
                                        ts->client->name, ts);
        if (ret != 0)
        {
            GTP_ERROR("Cannot allocate ts INT!ERRNO:%d\n", ret);
            goto test_pit;
        }
        //huion_irq_disable(ts->irq);

        GTP_INFO("  <%s>_%d     ts->irq=%d   ret = %d\n", __func__, __LINE__, ts->irq, ret);
    }
    else
    {
        GTP_ERROR("   ts->irq  error \n");
        ret = 1;
        goto test_pit;
    }
/*
    ret  = request_irq(ts->client->irq, 
                       huion_ts_irq_handler,
                       irq_table[ts->int_trigger_type],
                       ts->client->name,
                       ts);
*/
test_pit:
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        //GTP_GPIO_AS_INPUT(GTP_INT_PORT);
        gpio_direction_input(ts->irq_pin);
        //s3c_gpio_setpull(pin, S3C_GPIO_PULL_NONE);

        GTP_GPIO_FREE(ts->irq_pin);

        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = huion_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else
    {
        GTP_INFO("  <%s>_%d     ts->irq=%d   ret = %d\n", __func__, __LINE__, ts->irq, ret);
        huion_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
}

/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int huion_ts_early_suspend(struct tp_device *tp_d)
{
    struct huion_ts_data *ts;
    //s8 ret = -1;
    int reg = 0;

    ts = container_of(tp_d, struct huion_ts_data, tp);
    GTP_DEBUG_FUNC();

    GTP_INFO("System suspend.");

    ts->huion_is_suspend = 1;

    if (ts->use_irq)
    {
        huion_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }

    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability
    msleep(58);

    reg = regulator_disable(ts->tp_regulator);
    if (reg < 0)
        GTP_ERROR("failed to disable tp regulator\n");
    msleep(20);
    return 0;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int huion_ts_early_resume(struct tp_device *tp_d)
{
    struct huion_ts_data *ts;
    s8 ret = -1;
    int reg = 0;
    ts = container_of(tp_d, struct huion_ts_data, tp);
    GTP_DEBUG_FUNC();

    GTP_INFO("System resume.");

    reg = regulator_enable(ts->tp_regulator);
    if (reg < 0)
        GTP_ERROR("failed to enable tp regulator\n");
    msleep(10);

    //ret = huion_wakeup_sleep(ts);
    //ret = huion_huion_wakeup_sleep(ts);

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

    if (ts->use_irq)
    {
        huion_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->huion_is_suspend = 0;

    return 0;
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 huion_request_input_dev(struct i2c_client *client,
                                  struct huion_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif
    GTP_DEBUG_FUNC();

    ts->input_dev = devm_input_allocate_device(&client->dev);
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    //if (huion_change_x2y)
    //    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);

    set_bit(INPUT_PROP_POINTER, ts->input_dev->propbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(BTN_TOUCH, ts->input_dev->keybit);
    set_bit(BTN_TOOL_PEN, ts->input_dev->keybit);
    set_bit(ABS_X, ts->input_dev->absbit);
    set_bit(ABS_Y, ts->input_dev->absbit);
    set_bit(ABS_PRESSURE, ts->input_dev->absbit);


	input_set_abs_params(ts->input_dev, ABS_X, 0, DISPLAY_SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, DISPLAY_SCREEN_MAX_X, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, DISPLAY_SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, DISPLAY_SCREEN_MAX_X, 0, 0);

	{
		int x,y;
		x = DISPLAY_SCREEN_MAX_Y;
		y = DISPLAY_SCREEN_MAX_X;
		printk("=====ABS_x:%d ========ABS_y:%d",x,y);
	}
    //input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
    //input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
 
    input_set_abs_params(ts->input_dev,
		                         ABS_PRESSURE, 0, TOUCHE_MAX_PRESS_VALUE, 0, 0);

    sprintf(phys, "input/ts");
    ts->input_dev->name = huion_ts_name;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;

    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }

    ts->tp.tp_resume = huion_ts_early_resume;
    ts->tp.tp_suspend = huion_ts_early_suspend;
    tp_register_fb(&ts->tp);

    return 0;
}

int huion_ts_get_version(struct i2c_client *client,char*ver)
{
    int ret  = 0;
    int i;
    int trycnt = 0;
    char huibuf[22];
    char huicmd[3];
 
    //huicmd[0] = 0x00;   
    huicmd[0] = 0x5A;
    huicmd[1] = 0xC9;
    huicmd[2] = 0xA5;

    memset(huibuf, 0x00, 8);
      
    for(trycnt =0; trycnt<1; trycnt++)
    {
        ghuion_i2c_wxdata(client,huicmd,sizeof(huicmd));
        udelay(100);
        ghuion_i2c_rxdata(client, huibuf, sizeof(huibuf), 0x0);
        printk("fw version is: ");
        for(i=0; i< sizeof(huibuf); i++)
        {
            printk(KERN_CONT "%02x ",huibuf[i]);
        }
       
    }
    memcpy(ver,huibuf,sizeof(huibuf));
    if(huibuf[0]==1)
        ret = 1;
    return ret;    
}

int huion_ts_get_parameter(struct i2c_client *client,char*ver)
{
    int ret  = 0;
    int i;
    int trycnt = 0;
    char huibuf[22];
    char huicmd[3];
 
    //huicmd[0] = 0x00;   
    huicmd[0] = 0x5A;
    huicmd[1] = 0xC8;
    huicmd[2] = 0xA5;

    memset(huibuf, 0x00, sizeof(huibuf));
      
    for(trycnt =0; trycnt<1; trycnt++)
    {
        ghuion_i2c_wxdata(client,huicmd,sizeof(huicmd));
        udelay(100);
        ghuion_i2c_rxdata(client, huibuf, sizeof(huibuf), 0x0);
        printk("fw parameter is: ");
        for(i=0; i< sizeof(huibuf); i++)
        {
            printk(KERN_CONT "%02x ",huibuf[i]);
        }
    }
    memcpy(ver,huibuf,sizeof(huibuf));
    if(huibuf[0]==1)
        ret = 1;
    return ret;    
}
/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes. 
        0: succeed.
*******************************************************/
static int huion_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct huion_ts_data *ts;
    //u16 version_info;
    char huibuf[22];
    char parabuf[19];
    
    
    struct device_node *np = client->dev.of_node;
    enum of_gpio_flags rst_flags, pwr_flags;
    u32 val;
    printk("%s() start ####################\n", __func__);

    GTP_DEBUG_FUNC();

    //do NOT remove these logs
    GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    client->addr = (0x11 >> 1);
    //client->addr = (0x70>>1);

    GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

    i2c_connect_client_hn = client;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }

    memset(ts, 0, sizeof(*ts));

    if (!np)
    {
        dev_err(&client->dev, "no device tree\n");
        return -EINVAL;
    }
#if 1
    if (of_property_read_u32(np, "huion,tp-size", &val))
    {
        dev_err(&client->dev, "no tp-size defined\n");
        return -EINVAL;
    }
#endif

    ts->tp_regulator = devm_regulator_get(&client->dev, "tp");
    if (IS_ERR(ts->tp_regulator))
    {
        dev_err(&client->dev, "failed to get regulator, %ld\n",
                PTR_ERR(ts->tp_regulator));
        return PTR_ERR(ts->tp_regulator);
    }

    ret = regulator_enable(ts->tp_regulator);
    if (ret < 0)
        GTP_ERROR("failed to enable tp regulator\n");
    msleep(20);

    ts->irq_pin = of_get_named_gpio_flags(np, "touch-gpio", 0, (enum of_gpio_flags *)(&ts->irq_flags));
    ts->rst_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);
    ts->pwr_pin = of_get_named_gpio_flags(np, "power-gpio", 0, &pwr_flags);
    //ts->tp_select_pin = of_get_named_gpio_flags(np, "tp-select-gpio", 0, &tp_select_flags);

    printk("parse property of tree,%d %d\n", ts->irq_pin, ts->rst_pin);
    if (of_property_read_u32(np, "huion,max-x", &val))
    {
        dev_err(&client->dev, "no max-x defined\n");
        return -EINVAL;
    }
    printk("max-x %d\n", val);
    ts->abs_x_max = val;
    if (of_property_read_u32(np, "huion,max-y", &val))
    {
        dev_err(&client->dev, "no max-y defined\n");
        return -EINVAL;
    }

    printk("max-y %d\n", val);
    ts->abs_y_max = val;
    if (of_property_read_u32(np, "huion,configfile-num", &val))
    {
        ts->cfg_file_num = 0;
    }
    else
    {
        ts->cfg_file_num = val;
    }

    printk("val %d configfile-num %d \n", val, ts->cfg_file_num);

    ts->pendown = PEN_RELEASE;
    ts->client = client;

    INIT_WORK(&ts->work, huion_ts_work_func);
    ts->client = client;
    spin_lock_init(&ts->irq_lock); // 2.6.39 later
    // ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before

    i2c_set_clientdata(client, ts);

    ts->huion_rawdiff_mode = 0;

    ret = huion_request_io_port(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.");
        //return ret;
        goto probe_init_error_requireio;
    }

    huion_ts_get_version(client,huibuf);
    huion_ts_get_parameter(client,parabuf);
    {
        extern int DFU_proc(char *version);        
        
        ret = DFU_proc(&huibuf[2]); //前面的两个字节和版本号无关
        printk("DFU_proc ret: %d",ret);
        client->addr = (0x11 >> 1);   //

        if(ret){
            msleep(100);
            printk("need to read the version again ");
            huion_ts_get_version(client,huibuf);
            huion_ts_get_parameter(client,parabuf);
  
        }
    }
    client->addr = (0x11 >> 1);

    
   
   
#if GTP_AUTO_UPDATE
    ret = hup_init_update_proc(ts);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif

    ret = huion_request_input_dev(client, ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
    }
    ret = huion_request_irq(ts);
    if (ret < 0)
    {
        GTP_INFO("GTP works in polling mode.");
    }
    else
    {
        GTP_INFO("GTP works in interrupt mode.");
    }

    if (ts->use_irq)
    {
        huion_irq_enable(ts);
    }

    
    return 0;

//probe_init_error:
    printk("   <%s>_%d  prob error !!!!!!!!!!!!!!!\n", __func__, __LINE__);
    tp_unregister_fb(&ts->tp);
    GTP_GPIO_FREE(ts->rst_pin);
    GTP_GPIO_FREE(ts->irq_pin);
probe_init_error_requireio:
    tp_unregister_fb(&ts->tp);
    kfree(ts);
    return ret;
}

/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int huion_ts_remove(struct i2c_client *client)
{
    struct huion_ts_data *ts = i2c_get_clientdata(client);

    tp_unregister_fb(&ts->tp);

    GTP_DEBUG_FUNC();

    if (ts)
    {
        if (ts->use_irq)
        {
            //GTP_GPIO_AS_INPUT(GTP_INT_PORT);

            gpio_direction_input(ts->irq_pin);
            //s3c_gpio_setpull(pin, S3C_GPIO_PULL_NONE);

            GTP_GPIO_FREE(ts->irq_pin);

            //free_irq(client->irq, ts);
            printk("devm_free_irq ts->irq is: %d\n", ts->irq);
            devm_free_irq(&(ts->client->dev), ts->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
        GTP_INFO("GTP driver removing...");
        i2c_set_clientdata(client, NULL);
        input_unregister_device(ts->input_dev);
        kfree(ts);
    }

    return 0;
}

static const struct i2c_device_id huion_ts_id[] = {
    {GTP_I2C_NAME, 0},
    {}};

static struct of_device_id huion_ts_dt_ids[] = {
    {.compatible = "huion,hgtxx"},
    {}};

static struct i2c_driver huion_ts_driver = {
    .probe = huion_ts_probe,
    .remove = huion_ts_remove,
    .id_table = huion_ts_id,
    .driver = {
        .name = GTP_I2C_NAME,
        .of_match_table = of_match_ptr(huion_ts_dt_ids),
    },
};

/*******************************************************    
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int huion_ts_init(void)
{
    s32 ret;

    GTP_DEBUG_FUNC();
    printk("Huion driver installing...\n");
    huion_wq = create_singlethread_workqueue("huion_wq");
    if (!huion_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
    ret = i2c_add_driver(&huion_ts_driver);
    return ret;
}

/*******************************************************    
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void huion_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&huion_ts_driver);
    if (huion_wq)
    {
        destroy_workqueue(huion_wq);
    }
}
//late_initcall(huion_ts_init);
module_init(huion_ts_init);

module_exit(huion_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
