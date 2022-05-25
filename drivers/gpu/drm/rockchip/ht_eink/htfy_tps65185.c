/*
 * Papyrus epaper power control HAL
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, MM Solutions
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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31))
#include <linux/i2c/pmic-tps65185-i2c.h>
#else
#define PAPYRUS2_1P0_I2C_ADDRESS           0x48
#define PAPYRUS2_1P1_I2C_ADDRESS           0x68
extern void papyrus_set_i2c_address(int address);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
#include <asm/gpio.h>
#else
#include <linux/gpio.h>
#endif

#include "htfy_pmic.h"

// BITS OF power-good-satus
#define VNEG_PG                            (1<<1)
#define VEE_PG                             (1<<3)
#define VPOS_PG                            (1<<4)
#define VN_PG                              (1<<5)
#define VDDH_PG                            (1<<6)
#define VB_PG                              (1<<7)
#define ALL_POWER_GOOD                     (VNEG_PG|VEE_PG|VPOS_PG|VN_PG|VDDH_PG|VB_PG)
// BITS OF INT1-STATUS
#define INTS1_TSD                          (1<<6)      // Thermal shutdown interrupt
#define INTS1_UVLO                         (1<<2)      // VIN under voltage detect interrupt
// BITS OF INT2-STATUS
#define INTS2_VB_UV                        (1<<7)      // under-voltage on DCDC1 detected
#define INTS2_VDDH_UV                      (1<<6)      // under-voltage on VDDH charge pump detected
#define INTS2_VN_UV                        (1<<5)      // under-voltage on DCDC2 detected
#define INTS2_VPOS_UV                      (1<<4)      // under-voltage on LDO1(VPOS) detected
#define INTS2_VEE_UV                       (1<<3)      // under-voltage on VEE charge pump detected
#define INTS2_VCOMF                        (1<<2)      // fault on VCOM detected (VCOM is outside normal operating range)
#define INTS2_VNEG_UV                      (1<<1)      // under-voltage on LDO2(VNEG) detected
#define ANY_PWR_FAULT                      (INTS2_VB_UV|INTS2_VDDH_UV|INTS2_VN_UV|INTS2_VPOS_UV|INTS2_VEE_UV|INTS2_VCOMF|INTS2_VNEG_UV)

#define TPS65185_I2C_NAME                  "tps65185"
#define INVALID_GPIO                       -1
#define PAPYRUS_VCOM_MAX_MV                0
#define PAPYRUS_VCOM_MIN_MV                -5110
#define PAPYRUS_ADDR_TMST_VALUE            0x00
#define PAPYRUS_ADDR_ENABLE                0x01
#define PAPYRUS_ADDR_VADJ                  0x02
#define PAPYRUS_ADDR_VCOM1_ADJUST          0x03
#define PAPYRUS_ADDR_VCOM2_ADJUST          0x04
#define PAPYRUS_ADDR_INT_ENABLE1           0x05
#define PAPYRUS_ADDR_INT_ENABLE2           0x06
#define PAPYRUS_ADDR_INT_STATUS1           0x07
#define PAPYRUS_ADDR_INT_STATUS2           0x08
#define PAPYRUS_ADDR_UPSEQ0                0x09
#define PAPYRUS_ADDR_UPSEQ1                0x0a
#define PAPYRUS_ADDR_DWNSEQ0               0x0b
#define PAPYRUS_ADDR_DWNSEQ1               0x0c
#define PAPYRUS_ADDR_TMST1                 0x0d
#define PAPYRUS_ADDR_TMST2                 0x0e
#define PAPYRUS_ADDR_PG_STATUS             0x0f
#define PAPYRUS_ADDR_REVID                 0x10
// INT_ENABLE1
#define PAPYRUS_INT_ENABLE1_ACQC_EN        1
#define PAPYRUS_INT_ENABLE1_PRGC_EN        0
// INT_STATUS1
#define PAPYRUS_INT_STATUS1_ACQC           1
#define PAPYRUS_INT_STATUS1_PRGC           0
// VCOM2_ADJUST
#define PAPYRUS_VCOM2_ACQ                  7
#define PAPYRUS_VCOM2_PROG                 6
#define PAPYRUS_VCOM2_HIZ                  5
#define PAPYRUS_MV_TO_VCOMREG(MV)          ((MV) / 10)
#define PAPYRUS_V3P3OFF_DELAY_MS           10
#define V3P3_EN_MASK                       0x20
/*
   After waking up from sleep, Papyrus waits for VN to be discharged and all
   voltage ref to startup before loading the default EEPROM settings. So accessing
   registers too early after WAKEUP could cause the register to be overridden by default values
*/
#define PAPYRUS_EEPROM_DELAY_MS            50
/*
   Papyrus WAKEUP pin must stay low for a minimum time
*/
#define PAPYRUS_SLEEP_MINIMUM_MS           110
/*
   Temp sensor might take a little time to settle eventhough the status bit in TMST1
   state conversion is done - if read too early 0C will be returned instead of the right temp
*/
#define PAPYRUS_TEMP_READ_TIME_MS          10
/*
   Powerup sequence takes at least 24 ms - no need to poll too frequently
*/
#define HW_GET_STATE_INTERVAL_MS           24

#define BGB_PRINT                          1
#if BGB_PRINT
#define tps65185_printk(fmt, args...)      printk(KERN_INFO "[tps] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define tps65185_printk(fmt, args...)
#endif

struct papyrus_sess {
    struct i2c_adapter *adap;
    struct i2c_client *client;
    uint8_t enable_reg_shadow;
    uint8_t enable_reg;
    uint8_t vadj;
    uint8_t vcom1;
    uint8_t vcom2;
    uint8_t vcom2off;
    uint8_t int_en1;
    uint8_t int_en2;
    uint8_t upseq0;
    uint8_t upseq1;
    uint8_t dwnseq0;
    uint8_t dwnseq1;
    uint8_t tmst1;
    uint8_t tmst2;

    /* Custom power up/down sequence settings */
    struct {
        /* If options are not valid we will rely on HW defaults. */
        bool valid;
        unsigned int dly[8];
    } seq;

    unsigned int v3p3off_time_ms;
    int epd_type;
    int power_fault_cnt;
    int pwr_up_pin;
    int error_pin;
    int vcom_ctl_pin;
    int wake_up_pin;
    /* True if a high WAKEUP brings Papyrus out of reset. */
    int poweron_active_high;
    int wakeup_active_high;
    int vcomctl_active_high;
    bool need_init;
};

struct papyrus_hw_state {
    uint8_t tmst_value;
    uint8_t int_status1;
    uint8_t int_status2;
    uint8_t pg_status;
};

static uint8_t papyrus2_i2c_addr = PAPYRUS2_1P1_I2C_ADDRESS;
int pmic_id = 0;
static struct class *tps_pmu_class;

extern struct pmic_sess pmic_sess_data;
extern bool fb_is_power_off(void);
extern int sy7636a_power_on(void *priv);
extern int sy7636a_power_down(void *priv);
extern int sy7636a_set_vcom_voltage(void *priv, int vcom_mv);
extern int sy7636a_power_check(void *priv, int dump);
extern int sy7636a_reinit(void *priv, int epd_type);
extern int sy7636a_temperature_get(void *priv, int *temp);

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


static void papyrus_hw_get_pg(struct papyrus_sess *sess, struct papyrus_hw_state *hwst)
{
    int stat;

    stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
    }
}

static int papyrus_hw_reset(struct papyrus_sess *sess)
{

  // 20210817: 如果出错了，我们需要重新初始化 power-up 寄存器，修改延迟。所以此处需要设置
  // need_init 标志。
  sess->need_init = true;
  
  if (gpio_is_valid(sess->vcom_ctl_pin))
	  gpio_set_value(sess->vcom_ctl_pin, !sess->vcomctl_active_high);
  if (gpio_is_valid(sess->pwr_up_pin))
	  gpio_set_value(sess->pwr_up_pin, !sess->poweron_active_high);
	  
  if (gpio_is_valid(sess->wake_up_pin)) {
	  gpio_set_value(sess->wake_up_pin, !sess->wakeup_active_high);
	  msleep(PAPYRUS_SLEEP_MINIMUM_MS);

	  gpio_set_value(sess->wake_up_pin, sess->wakeup_active_high);
	  msleep(PAPYRUS_EEPROM_DELAY_MS);
  }

  return 0;
}

static void papyrus_hw_dump_reg(const char *stage, struct papyrus_sess *sess)
{
    int stat = 0;
    uint8_t upseq0, upseq1, dwnseq0, dwnseq1, enx;
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_UPSEQ0, &upseq0);
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_UPSEQ1, &upseq1);
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_DWNSEQ0, &dwnseq0);
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_DWNSEQ1, &dwnseq1);
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_ENABLE, &enx);

    tps65185_printk("%s:upseq0=0x%x/0x%x,upseq1=0x%x/0x%x,dwseq0=0x%x/0x%x,dwseq1=0x%x/0x%x,epd=0x%x,enx=0x%x/0x%x,failed=%d\n",
                    stage, upseq0, sess->upseq0, upseq1, sess->upseq1,
                    dwnseq0, sess->dwnseq0, dwnseq1, sess->dwnseq1, sess->epd_type,
                    enx, sess->enable_reg_shadow, sess->power_fault_cnt);
}

static int papyrus_hw_arg_init(struct papyrus_sess *sess)
{
    int stat = 0;

    sess->vadj = 0x03;
    sess->upseq0 = SEQ_VEE(0) | SEQ_VNEG(1) | SEQ_VPOS(2) | SEQ_VDD(3);
	sess->upseq1 =  0x00;
    sess->dwnseq0 = SEQ_VDD(0) | SEQ_VPOS(1)  | SEQ_VNEG(2) | SEQ_VEE(3);
    sess->dwnseq1 = DDLY_6ms(0) | DDLY_6ms(1) | DDLY_6ms(2) | DDLY_6ms(3);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);
    if (stat) {
        pr_err("papyrus: set UPSEQ-DOWNSEQ failed,i2c error!\n");
    }

    if (BGB_PRINT) {
        papyrus_hw_dump_reg("hwinit1", sess);
    }

    return stat;
}

static void papyrus_hw_send_powerup(struct papyrus_sess *sess)
{
    int stat = 0;

    if (sess->need_init) {
        papyrus_hw_arg_init(sess);
    }

    if (sess->vcom_ctl_pin != INVALID_GPIO) {
        gpio_direction_output(sess->vcom_ctl_pin, sess->vcomctl_active_high);
    }

    /* switch to active mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive,
     * don't enable vcom buffer
     */
    sess->enable_reg_shadow = (0x80 | 0x30 | 0x0F);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
    if (stat) {
        pr_err("papyrus: I2C error at pwr up: %d\n", stat);
    }
}

static void papyrus_hw_send_powerdown(struct papyrus_sess *sess)
{
    int stat;

    /* switch to standby mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive,
     * don't enable vcom buffer
     */
    sess->enable_reg_shadow = (0x40 | 0x00 | 0x0F);  // 20191005,update from RK source code.
    stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
    if (stat) {
        pr_err("papyrus: I2C error at pwr down: %d\n", stat);
    }

    if (sess->vcom_ctl_pin != INVALID_GPIO) {
        gpio_direction_output(sess->vcom_ctl_pin, !sess->vcomctl_active_high);
    }
}

static int papyrus_hw_read_temperature(struct pmic_sess *pmsess, int *t)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    int stat;
    int ntries = 50;
    uint8_t tb;

    stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_TMST1, 0x80);

    do {
        stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_TMST1, &tb);
    } while (!stat && ntries-- && (((tb & 0x20) == 0) || (tb & 0x80)));

    if (stat) {
        return stat;
    }

    msleep(PAPYRUS_TEMP_READ_TIME_MS);
    stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_TMST_VALUE, &tb);
    *t = (int)(int8_t)tb;

    //tps65185_printk("current temperature is %d\n",*t);

    return stat;
}
static int papyrus_hw_get_revid(struct papyrus_sess *sess)
{
    int stat;
    uint8_t revid;

    stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_REVID, &revid);
    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
        return stat;
    } else {
        return revid;
    }
}

// class node
// cat /sys/class/pmu/temperature
static ssize_t papyrus_temperature_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;

    int temperature = 25;
	ssize_t len = 0;
	//u8 val = 0;
	papyrus_hw_read_temperature(psess,&temperature);
    len += sprintf(_buf, "%d:\n",temperature);
	return len;
}


/*static ssize_t papyrus_temperature_store(struct class *cls,struct class_attribute *attr, const char *buf, size_t _count)
{

	return 0;
}*/
//static CLASS_ATTR(temperature, 0644, papyrus_class_temperature_show, papyrus_class_temperature_store);
static CLASS_ATTR_RO(papyrus_temperature);

void papyrus_set_i2c_address(int address)
{
    if (address == PAPYRUS2_1P0_I2C_ADDRESS) {
        papyrus2_i2c_addr = PAPYRUS2_1P0_I2C_ADDRESS;
    } else if (address == PAPYRUS2_1P1_I2C_ADDRESS) {
        papyrus2_i2c_addr = PAPYRUS2_1P1_I2C_ADDRESS;
    } else {
        pr_err("papyrus: Invalid i2c address: %d\n", address);
    }
    printk("papyrus i2c addr set to %x\n", papyrus2_i2c_addr);
}


static int papyrus_hw_init(struct papyrus_sess *sess, const char *chip_id)
{
    int stat = 0;
    int ret;

    if ((sess->pwr_up_pin != INVALID_GPIO)) {
        stat |= gpio_request(sess->pwr_up_pin, "papyrus-power-on");
    }

    if ((sess->vcom_ctl_pin != INVALID_GPIO)) {
        stat |= gpio_request(sess->vcom_ctl_pin, "papyrus-vcom-ctl");
    }
    if ((sess->error_pin != INVALID_GPIO)) {
        stat |= gpio_request(sess->error_pin, "papyrus-error");
    }

    if ((sess->wake_up_pin != INVALID_GPIO)) {
        stat |= gpio_request(sess->wake_up_pin, "papyrus-wake_up");
    }

    if (stat) {
        pr_err("papyrus: cannot reserve GPIOs\n");
        stat = -ENODEV;
        return stat;
    }
    sess->poweron_active_high = 1;
    sess->vcomctl_active_high = 1;
    sess->wakeup_active_high = 1;

    if ((sess->wake_up_pin != INVALID_GPIO)) {
        gpio_direction_output(sess->wake_up_pin, sess->wakeup_active_high);
        sess->need_init = true;
    }

    gpio_direction_output(sess->vcom_ctl_pin, !sess->vcomctl_active_high);

    if (sess->error_pin != INVALID_GPIO) {
        gpio_direction_input(sess->error_pin);
    }

    msleep(PAPYRUS_EEPROM_DELAY_MS);

    if (sess->pwr_up_pin != INVALID_GPIO) {
        gpio_direction_output(sess->pwr_up_pin, !sess->poweron_active_high);
    }

    stat = papyrus_hw_get_revid(sess);
    if (stat < 0) {
        goto cleanup_i2c_adapter;
    }
    pr_info("papyrus: detected device with ID=%02x (TPS6518%dr%dp%d)\n", stat, stat & 0xF, (stat & 0xC0) >> 6, (stat & 0x30) >> 4);
    if ((stat == 0x65) || (stat == 0x66)) {
        pmic_id = 0x6518;
    }
    stat = 0;
	
	tps_pmu_class = class_create(THIS_MODULE, "pmu");
	ret =  class_create_file(tps_pmu_class, &class_attr_papyrus_temperature);
	if (ret) {
		pr_info("Fail to create class pmu_temperature.\n");
    }
    
    return stat;

cleanup_i2c_adapter:
    i2c_put_adapter(sess->adap);
    if (sess->wake_up_pin != INVALID_GPIO) {
        gpio_free(sess->wake_up_pin);
    }
    gpio_free(sess->vcom_ctl_pin);
    if (sess->pwr_up_pin != INVALID_GPIO) {
        gpio_free(sess->pwr_up_pin);
    }
	if (sess->error_pin != INVALID_GPIO)
		gpio_free(sess->error_pin);
    pr_err("papyrus: ERROR: could not initialize I2C papyrus!\n");

    return stat;
}

static void papyrus_hw_power_req(struct pmic_sess *pmsess, bool up)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    // tps65185_printk("powerup=%d\n", up);
    if (up) {
        papyrus_hw_send_powerup(sess);
    } else {
        papyrus_hw_send_powerdown(sess);
    }
    return;
}

static bool papyrus_hw_power_ack(struct pmic_sess *pmsess)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    struct papyrus_hw_state hwst;
    int st;
    int retries_left = 10;

    do {
        papyrus_hw_get_pg(sess, &hwst);

        pr_debug("hwst: tmst_val=%d, ist1=%02x, ist2=%02x, pg=%02x\n",
                 hwst.tmst_value, hwst.int_status1, hwst.int_status2, hwst.pg_status);
        hwst.pg_status &= 0xfa;
        if (hwst.pg_status == 0xfa) {
            st = 1;
        } else if (hwst.pg_status == 0x00) {
            st = 0;
        } else {
            st = -1;    /* not settled yet */
            msleep(HW_GET_STATE_INTERVAL_MS);
        }
        retries_left--;
    } while ((st == -1) && retries_left);

    if ((st == -1) && !retries_left) {
        pr_err("papyrus: power up/down settle error (PG = %02x)\n", hwst.pg_status);
    }

    return !!st;
}

static void papyrus_hw_cleanup(struct papyrus_sess *sess)
{
	if (sess->vcom_ctl_pin != INVALID_GPIO)
	    gpio_free(sess->vcom_ctl_pin);
    if (sess->pwr_up_pin != INVALID_GPIO) {
        gpio_free(sess->pwr_up_pin);
    }

    if (sess->error_pin != INVALID_GPIO) {
        gpio_free(sess->error_pin);
    }

    if (sess->wake_up_pin != INVALID_GPIO) {
        gpio_free(sess->wake_up_pin);
    }

    i2c_put_adapter(sess->adap);
}

static int papyrus_set_enable(struct pmic_sess *pmsess, int enable)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->enable_reg = enable;
    return 0;
}

static int papyrus_set_vcom_voltage(struct pmic_sess *pmsess, int vcom_mv)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(vcom_mv) & 0x00FF);
    sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(vcom_mv) & 0x0100) >> 8);
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
    // TODO; Remove this temporary solution to set custom vcom-off mode
    //       Add PMIC setting when this is to be a permanent feature
    pr_debug("papyrus_set_vcom2 vcom2off 0x%02x\n", vcom2);
    sess->vcom2off = vcom2;
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
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->int_en1 = int_en1;
    return 0;
}

static int papyrus_set_int_en2(struct pmic_sess *pmsess, uint8_t int_en2)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->int_en2 = int_en2;
    return 0;
}

static int papyrus_set_upseq0(struct pmic_sess *pmsess, uint8_t upseq0)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->upseq0 = upseq0;
    return 0;
}

static int papyrus_set_upseq1(struct pmic_sess *pmsess, uint8_t upseq1)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->upseq1 = upseq1;
    return 0;
}

static int papyrus_set_dwnseq0(struct pmic_sess *pmsess, uint8_t dwnseq0)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->dwnseq0 = dwnseq0;
    return 0;
}

static int papyrus_set_dwnseq1(struct pmic_sess *pmsess, uint8_t dwnseq1)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->dwnseq1 = dwnseq1;
    return 0;
}

static int papyrus_set_tmst1(struct pmic_sess *pmsess, uint8_t tmst1)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->tmst1 = tmst1;
    return 0;
}

static int papyrus_set_tmst2(struct pmic_sess *pmsess, uint8_t tmst2)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    sess->tmst2 = tmst2;
    return 0;
}

static int papyrus_vcom_switch(struct pmic_sess *pmsess, bool state)
{
    struct papyrus_sess *sess = pmsess->drvpar;
    int stat;

    sess->enable_reg_shadow &= ~((1u << 4) | (1u << 6) | (1u << 7));
    sess->enable_reg_shadow |= (state ? 1u : 0) << 4;

    stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);

    /* set VCOM off output */
    if (!state && sess->vcom2off != 0) {
        stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, sess->vcom2off);
    }

    return stat;
}

static int tps65185_reinit(void *priv, int epd_type)
{
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    tps65185_printk("tps65185_reinit: sess=%p\n", sess);
    if (sess == NULL) {
        return -EINVAL;
    }

    sess->epd_type = epd_type;
    return papyrus_hw_arg_init(sess);
}

static int papyrus_pm_sleep(struct pmic_sess *sess)
{
    struct papyrus_sess *s = sess->drvpar;

    tps65185_printk(" enter sleep,wakeup gpio=%d,fb off=%d\n", s->wake_up_pin, fb_is_power_off());

    gpio_direction_output(s->vcom_ctl_pin, !s->vcomctl_active_high);

    if (s->wake_up_pin != INVALID_GPIO) {
        gpio_direction_output(s->wake_up_pin, !s->wakeup_active_high);
        s->need_init = true;
    } else {
        s->need_init = fb_is_power_off();
    }

    return 0;
}

static int papyrus_pm_resume(struct pmic_sess *sess)
{
    struct papyrus_sess *s = sess->drvpar;

    tps65185_printk(" exit sleep,re-init=%d!\n", s->need_init);

    if (s->wake_up_pin != INVALID_GPIO) {
        gpio_direction_output(s->wake_up_pin, s->wakeup_active_high);
    }

    return 0;
}

static int papyrus_probe(struct pmic_sess *pmsess, struct i2c_client *client)
{
    struct papyrus_sess *sess;
    int stat;
    enum of_gpio_flags flags;
    struct device_node *node = client->dev.of_node;

    sess = kzalloc(sizeof(*sess), GFP_KERNEL);
    if (!sess) {
        pr_err("%s:%d: kzalloc failed\n", __func__, __LINE__);
        return -ENOMEM;
    }
    sess->client = client;
    sess->adap = client->adapter;
    sess->v3p3off_time_ms = PAPYRUS_V3P3OFF_DELAY_MS;
    sess->vcom_ctl_pin = of_get_named_gpio_flags(node, "vcomctl-gpios", 0, &flags);
    if (!gpio_is_valid(sess->vcom_ctl_pin)) {
        pr_err("tsp65185: failed to find vcom_ctl pin\n");
        goto free_sess;
    }

    /*sess->error_pin = of_get_named_gpio_flags(node, "int-gpios", 0, &flags); 
    if (!gpio_is_valid(sess->error_pin)) {
        sess->error_pin = INVALID_GPIO;
        pr_err("tsp65185: failed to find error_pin\n");
        goto free_sess;
    }*/

    sess->pwr_up_pin = of_get_named_gpio_flags(node, "powerup-gpios", 0, &flags);
    if (!gpio_is_valid(sess->pwr_up_pin)) {
        sess->pwr_up_pin = INVALID_GPIO;
        pr_err("tsp65185: failed to find pwr_up pin\n");
    }

    sess->wake_up_pin = of_get_named_gpio_flags(node, "wakeup-gpios", 0, &flags);
    if (!gpio_is_valid(sess->wake_up_pin)) {
        sess->wake_up_pin = INVALID_GPIO;
        pr_err("tsp65185: failed to find wakeup pin\n");
    }


    sess->error_pin = of_get_named_gpio_flags(node, "error-gpios", 0, &flags);
    if (!gpio_is_valid(sess->error_pin)) {
        sess->error_pin = INVALID_GPIO;
        pr_err("tsp65185: failed to find error gpios\n");
    }
    stat = papyrus_hw_init(sess, pmsess->drv->id);
    if (stat) {
        goto free_sess;
    }

    sess->enable_reg_shadow = 0x40;
    stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
    if (stat) {
        goto free_sess;
    }

    pmsess->drvpar = sess;
    pmsess->revision = papyrus_hw_get_revid(sess);
    printk("papyrus_probe 65185:%d \n", pmsess->revision);

    return 0;

free_sess:
    if (gpio_is_valid(sess->vcom_ctl_pin)) {
        sess->vcom_ctl_pin = INVALID_GPIO;
    }
    if (gpio_is_valid(sess->pwr_up_pin)) {
        sess->pwr_up_pin = INVALID_GPIO;
    }
    if (gpio_is_valid(sess->wake_up_pin)) {
        sess->wake_up_pin = INVALID_GPIO;
    }
    if (gpio_is_valid(sess->error_pin)) {
        sess->error_pin = INVALID_GPIO;
    }
    kfree(sess);

    return stat;
}

static void papyrus_remove(struct pmic_sess *pmsess)
{
    struct papyrus_sess *sess = pmsess->drvpar;

    papyrus_hw_cleanup(sess);
    kfree(sess);
    pmsess->drvpar = 0;
}

const struct pmic_driver pmic_driver_tps65185_i2c = {
    .id = "tps65185-i2c",
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
int tps65185_vcom_get(void)
{
    struct pmic_sess tpmic_sess_data = pmic_sess_data;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    uint8_t rev_val = 0;
    int stat = 0;
    int read_vcom_mv = 0;

    tps65185_printk("tps65185_vcom_get enter.\n");
    if (!tpmic_sess_data.is_inited) {
        return -1;
    }

    read_vcom_mv = 0;
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, &rev_val);
    tps65185_printk("rev_val = 0x%x\n", rev_val);
    read_vcom_mv += rev_val;
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, &rev_val);
    tps65185_printk("rev_val = 0x%x\n", rev_val);
    read_vcom_mv += ((rev_val & 0x0001) << 8);
    tps65185_printk("read_vcom_mv = %d\n", read_vcom_mv);

    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
    }

    return read_vcom_mv * 10;
}

static int tps65185_vcom_read(struct papyrus_sess *sess)
{
    int stat = 0;
    uint8_t rev_val1 = 0, rev_val2 = 0;
    int read_vcom_mv;

    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, &rev_val1);
    read_vcom_mv = rev_val1;
    stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, &rev_val2);
    read_vcom_mv += ((rev_val2 & 0x0001) << 8);
    read_vcom_mv *= 10;
    tps65185_printk("read_vcom_mv = %d(vcom1=0x%x,vcom2=0x%x)\n", read_vcom_mv, rev_val1, rev_val2);

    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
    }
    return read_vcom_mv;
}

int tps65185_vcom_set(int vcom_mv)
{
    struct pmic_sess tpmic_sess_data = pmic_sess_data;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    uint8_t rev_val = 0;
    int stat = 0;
    int read_vcom_mv;

    if (!tpmic_sess_data.is_inited) {
        tps65185_printk("tps65185_vcom_set enter,vcom_mv=%d,but Not Init!!\n", vcom_mv);
        return -1;
    }

    read_vcom_mv = tps65185_vcom_read(sess);

    tps65185_printk("tps65185_vcom_set,new vcom_mv=%d,read vcom=%d!!\n", vcom_mv, read_vcom_mv);
    if (read_vcom_mv == vcom_mv) {
        return 0;
    }

    // Set vcom voltage
    pmic_driver_tps65185_i2c.set_vcom_voltage((struct pmic_sess *)&tpmic_sess_data, vcom_mv);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, sess->vcom1);
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, sess->vcom2);
    tps65185_printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n", sess->vcom1, sess->vcom2);

    sess->vcom2 |= 1 << PAPYRUS_VCOM2_PROG;
    stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, sess->vcom2);
    rev_val = 0;
    while (!(rev_val & (1 << PAPYRUS_INT_STATUS1_PRGC))) {
        stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_INT_STATUS1, &rev_val);
        tps65185_printk("PAPYRUS_ADDR_INT_STATUS1 = 0x%x\n", rev_val);
        msleep(50);
    }

    read_vcom_mv = tps65185_vcom_read(sess);
    tps65185_printk("set vcomd done,read_vcom_mv = %d,set vcom=%d\n", read_vcom_mv, vcom_mv);

    if (stat) {
        pr_err("papyrus: I2C error: %d\n", stat);
    }

    return 0;
}

static int tps65185_set_vcom_voltage(void *priv, int vcom_mv)
{
    if (vcom_mv < 0 || vcom_mv > 5110) {
        printk("tps: err vcom value, vcom value shoule be 0~5110\n");
        return -1;
    }

    return tps65185_vcom_set(vcom_mv);
}

static int tps65185_power_on(void *priv)
{
    if (pmic_sess_data.is_inited) {
        pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data, 1);
    }
    return 0;
}
static int tps65185_power_down(void *priv)
{
    if (pmic_sess_data.is_inited) {
        pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data, 0);
    }
    return 0;
}

// return 1: check POWER GOOD; 0: check power fail but Not-Fault.
// -1: power Fault, needs reset. only check after power-on.
static int tps65185_power_check(void *priv, int timeout)
{
    int stat;
    uint8_t ints1, ints2, pwrg;
    int err_pin_value = -1;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;

    // Once a fault is detected,the PWR_GOOD and nINT pins are pulled low and the corresponding 
    // interrupt bit is set in the interrupt register.
    if (sess->error_pin != INVALID_GPIO){
       err_pin_value = gpio_get_value(sess->error_pin);
       tps65185_printk("power_check: power-good(gpio%d)=%d,timeout=%d\n", sess->error_pin, err_pin_value, timeout);
	   if(err_pin_value == 1) { // if high, means power-good.
	        return 1;
	   }

	   // 20211227：我们在 SNX A6X 上面发现上电异常，原因是在 65185上电的时候，不能通过I2C访问
	   // 里面的寄存器（尤其是 INT1/INT2),否则容易造成上电异常(FAULT)。所以此处如果有GPIO，直接返回，除非TIMEOUT
	   // 了才进行访问,判断是否是真正的异常。
	   if(!timeout) return 0;
	}
	
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &pwrg); // power-good-status.
	if (stat) {
		pr_err("papyrus: I2C error: stat = %d\n", stat);
		return -1;
	} else {
		if((pwrg & 0X000000FA) == 0X000000FA) {
			return 1;
		}
		// 20210724: 如果已经出现 falt了，则不需要再等待了。
	    if(timeout) {
	    	// 20210512: 如果电源异常，我们通过读取 INT_STATUS1 和 INT_STATUS2 来清除异常。
			stat |= papyrus_hw_getreg(sess,
						PAPYRUS_ADDR_INT_STATUS1, &ints1);
			stat |= papyrus_hw_getreg(sess,
						PAPYRUS_ADDR_INT_STATUS2, &ints2);
	    	pr_err("papyrus: Power_Fault,fcnt=%d,PG=0X%02X,INT1=0X%02X,INT2=0X%02X\n", 
	    		sess->power_fault_cnt, pwrg, ints1, ints2);
	    	sess->power_fault_cnt++;
	    	
			 if(ints2 & ANY_PWR_FAULT) {
		    	// 20211227: don't reset,just power-up again!!
		    	papyrus_hw_reset(sess);
		    	//msleep(PAPYRUS_EEPROM_DELAY_MS);

		    	papyrus_hw_send_powerup(sess);
	    	}
	    } 
	}
	return 0;
}

static int tps65185_temperature_get(void *priv, int *temp)
{
    if (pmic_sess_data.is_inited) {
        return pmic_driver_tps65185_i2c.hw_read_temperature((struct pmic_sess *)&pmic_sess_data, temp);
    } else {
        return 0;
    }
}

__weak int htfy_register_ebc_pwr_ops(struct htfy_pwr_ops *ops)
{
    printk("===============htfy_register_ebc_pwr_ops:%d\n",pmic_id);
	if(pmic_id != 0x6518){
	    ops->priv = &pmic_sess_data;
		ops->power_on = sy7636a_power_on;
		ops->power_down = sy7636a_power_down;
		ops->vcom_set = sy7636a_set_vcom_voltage;
		ops->power_check = sy7636a_power_check;
		ops->reinit = sy7636a_reinit;
	}else{
		ops->priv = &pmic_sess_data;
    	ops->power_on = tps65185_power_on;
    	ops->power_down = tps65185_power_down;
    	ops->vcom_set = tps65185_set_vcom_voltage;
    	ops->power_check = tps65185_power_check;
    	ops->reinit = tps65185_reinit;
	}
	return 0;
}

__weak int htfy_register_ebc_temp_ops(struct htfy_gettemp_ops *ops)
{
    ops->priv = &pmic_sess_data;
    if (pmic_id != 0x6518) {
        ops->priv = &pmic_sess_data;
        ops->temperature_get = sy7636a_temperature_get;
    } else {
        ops->priv = &pmic_sess_data;
        ops->temperature_get = tps65185_temperature_get;
    }
    return 0;
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
    u32 value;

    if (pmic_id != 0x6518) {
        sy7636a_temperature_get(&pmic_sess_data, &value);
    } else {
        tps65185_temperature_get(&pmic_sess_data, &value);
    }
    seq_printf(s, "%d\n", value);

    return 0;
}

static int proc_lm_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_lm_show, NULL);
}

static const struct file_operations proc_lm_fops = {
    .open = proc_lm_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static int __init lm_proc_init(void)
{
    proc_create("epdsensor", 0, NULL, &proc_lm_fops);

    return 0;
}
late_initcall(lm_proc_init);
#endif
/*--------------------------------------------------------------------------------------------------------------*/

static int tps65185_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    //tps65185_printk("tps65185_probe: I2C addr:%x\n", client->addr);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("I2C check functionality failed.");
        return -ENODEV;
    }

    if (0 != pmic_driver_tps65185_i2c.hw_init((struct pmic_sess *)&pmic_sess_data, client)) {
        pmic_id = 0;
        printk("pmic_driver_tps65185_i2c hw_init failed.");
        return -ENODEV;
    }
    pmic_sess_data.is_inited = 1;

    return 0;
}

static int tps65185_remove(struct i2c_client *client)
{
    pmic_driver_tps65185_i2c.hw_cleanup((struct pmic_sess *)&pmic_sess_data);
    memset(&pmic_sess_data, 0, sizeof(struct pmic_sess));
    return 0;
}

static int tps65185_suspend(struct device *dev)
{
    return pmic_driver_tps65185_i2c.hw_pm_sleep((struct pmic_sess *)&pmic_sess_data);
}
static int tps65185_resume(struct device *dev)
{
    return pmic_driver_tps65185_i2c.hw_pm_resume((struct pmic_sess *)&pmic_sess_data);
}

static const struct i2c_device_id tps65185_id[] = {
    { TPS65185_I2C_NAME, 0 },
    { }
};

static const struct of_device_id tps65185_dt_ids[] = {
    { .compatible = "ti,tps65185", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tps65185_dt_ids);

static const struct dev_pm_ops tps65185_pm_ops = {
    .resume = tps65185_resume,
    .suspend = tps65185_suspend,
};

static struct i2c_driver tps65185_driver = {
    .probe = tps65185_probe,
    .remove = tps65185_remove,
    .id_table = tps65185_id,
    .driver = {
        .of_match_table = tps65185_dt_ids,
        .name = TPS65185_I2C_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_PM
        .pm = &tps65185_pm_ops,
#endif
    },
};

static int __init tps65185_init(void)
{
    int ret;

    ret = i2c_add_driver(&tps65185_driver);
    if (ret) {
        printk("Register tps65185 driver failed.\n");
    }

    return ret;
}

static void __exit tps65185_exit(void)
{
    return i2c_del_driver(&tps65185_driver);
}

fs_initcall(tps65185_init);
module_exit(tps65185_exit);

MODULE_DESCRIPTION("ti tps65185 pmic");
MODULE_LICENSE("GPL");