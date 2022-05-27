/*
 * mynode driver
 * yehongfeng <yehongfeng@boe.com.cn>
 *
 * This is a general node driver for Linux. 
 * Its main function is to provide android with the required driver parameters
 *
 */
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/rk_keys.h>

#define DEBUG

//extern void set_test_gpio(int level);
//extern int get_test_gpio(void);

extern int light_ltr578_get_value(void);
extern int light_stk3x1x_get_value(void);

extern void light_ltr578_set_cal_factor(int level);
extern void light_stk3x1x_set_cal_factor(int level);
extern int light_ltr578_get_cal_factor(void);
extern int light_stk3x1x_get_cal_factor(void);

extern void set_ltr578_cali_value(int value);
extern int get_ltr578_cali_value(void);
extern void set_stk3x1x_cali_value(int value);
extern int get_stk3x1x_cali_value(void);

extern int hall_get_value(void);

static int major;
static struct class *cls;
static struct input_dev *mynode_dev;


#define LSENSOR_IOCTL_SET_LTR578_CALIBRATION			1001
#define LSENSOR_IOCTL_GET_LTR578_CALIBRATION			1002
#define LSENSOR_IOCTL_SET_STK3X1X_CALIBRATION			1003
#define LSENSOR_IOCTL_GET_STK3X1X_CALIBRATION			1004
#define LSENSOR_IOCTL_GET_LTR578_LUX				1005
#define LSENSOR_IOCTL_GET_STK3X1X_LUX				1006

/*
static ssize_t
gpio0a7_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = get_test_gpio();

    return sprintf(buf, "%d\n", count);
}

static ssize_t gpio0a7_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int n;
    if (count <= 0)
        return 0;

    n = simple_strtoul(buf, NULL, 0);

    set_test_gpio(n);

    return count;
}

static DEVICE_ATTR_RW(gpio0a7);
*/

static ssize_t
ltr578_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int count = light_ltr578_get_value();

    	return sprintf(buf, "%d\n", count);
}

static ssize_t ltr578_value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    	int n;
    	if (count <= 0)
        	return 0;

    	n = simple_strtoul(buf, NULL, 0);

    	//set_test_gpio(n);

    	return count;
}

static DEVICE_ATTR_RW(ltr578_value);

static ssize_t
stk3x1x_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int count = light_stk3x1x_get_value();

    	return sprintf(buf, "%d\n", count);
}

static ssize_t stk3x1x_value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int n;
    	if (count <= 0)
        	return 0;

    	n = simple_strtoul(buf, NULL, 0);

    	//set_test_gpio(n);

    	return count;
}

static DEVICE_ATTR_RW(stk3x1x_value);

static ssize_t
ltr578_cal_factor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int count = light_ltr578_get_cal_factor();
    	if(count > 50 && count < 150)
		count = 1;
    	else
		count = 0;

    	return sprintf(buf, "%d\n", count);
}

static ssize_t ltr578_cal_factor_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    	int n;
    	if (count <= 0)
        	return 0;

    	n = simple_strtoul(buf, NULL, 0);

    	light_ltr578_set_cal_factor(n);
	
	input_report_key(mynode_dev, KEY_F7, 1);
	input_sync(mynode_dev);
	input_report_key(mynode_dev, KEY_F7, 0);
	input_sync(mynode_dev);

    	return count;
}

static DEVICE_ATTR_RW(ltr578_cal_factor);

static ssize_t
stk3x1x_cal_factor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int count = light_stk3x1x_get_cal_factor();
    	if(count > 50 && count < 150)
		count = 1;
    	else
		count = 0;

    	return sprintf(buf, "%d\n", count);
}

static ssize_t stk3x1x_cal_factor_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    	int n;
    	if (count <= 0)
        	return 0;

    	n = simple_strtoul(buf, NULL, 0);

    	light_stk3x1x_set_cal_factor(n);
	
	input_report_key(mynode_dev, KEY_F8, 1);
	input_sync(mynode_dev);
	input_report_key(mynode_dev, KEY_F8, 0);
	input_sync(mynode_dev);

    	return count;
}

static DEVICE_ATTR_RW(stk3x1x_cal_factor);

static ssize_t
hall_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int count = hall_get_value();

    	return sprintf(buf, "%d\n", count);
}

static ssize_t hall_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    	int n;
    	if (count <= 0)
        	return 0;

    	n = simple_strtoul(buf, NULL, 0);

    	//set_test_gpio(n);

    	return count;
}

static DEVICE_ATTR_RW(hall);


static int mynode_open(struct inode* i_node,struct file* filp)    
{    
    	printk("===>YHF[%s]\n",__func__);    
    	return 0;    
}  

static long mynode_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int result = 0;
	void __user *argp = (void __user *)arg;
	short cali_value;

 
	printk("func = %s, line = %d, ----->>cmd = %d",__func__,__LINE__,cmd); 
			
	switch(cmd)
	{ 
		case LSENSOR_IOCTL_SET_LTR578_CALIBRATION:	
			if (copy_from_user(&cali_value, argp, sizeof(cali_value))) {
				printk("%s:failed to copy light sensor ltr578 cali_value from user space.\n", __func__);
				return -EFAULT;
			}			
			set_ltr578_cali_value(cali_value);
			printk("Set ltr578 calibration value = %d\n", cali_value);
			break;

		case LSENSOR_IOCTL_SET_STK3X1X_CALIBRATION:	
			if (copy_from_user(&cali_value, argp, sizeof(cali_value))) {
				printk("%s:failed to copy light sensor stk3x1x cali_value from user space.\n", __func__);
				return -EFAULT;
			}			
			set_stk3x1x_cali_value(cali_value);
			printk("Set stk3x1x calibration value = %d\n", cali_value);
			break;

		case LSENSOR_IOCTL_GET_LTR578_CALIBRATION:	
			result = get_ltr578_cali_value();
			printk("Get ltr578 calibration value = %d\n", result);
			break;

		case LSENSOR_IOCTL_GET_STK3X1X_CALIBRATION:	
			result = get_stk3x1x_cali_value();
			printk("Get stk3x1x calibration value = %d\n", result);
			break;

		case LSENSOR_IOCTL_GET_LTR578_LUX:	
			result = light_ltr578_get_value();
			printk("Get ltr578 lux value = %d\n", result);
			break;

		case LSENSOR_IOCTL_GET_STK3X1X_LUX:	
			result = light_stk3x1x_get_value();
			printk("Get stk3x1x lux value = %d\n", result);
			break;
		
		default:						
			printk("no cmd....");
			return -EINVAL;
	}
	
	return result;
}  

static struct file_operations mynode_ops={
        .owner  = THIS_MODULE,
        .open   = mynode_open, 
	.unlocked_ioctl    	=    mynode_ioctl,   
};

static int __init mynode_init(void)
{
	struct device *mydev;   
	int ret = 0;

	major=register_chrdev(0,"mynode", &mynode_ops);
	cls=class_create(THIS_MODULE, "mynode_class");
	mydev = device_create(cls, 0, MKDEV(major,0),NULL,"mynode_device");
	
	printk("===>YHF[%s] enter!\n",__func__);

   

    	/*if(sysfs_create_file(&(mydev->kobj), &dev_attr_gpio0a7.attr))
    	{
        	printk("===>YHF[%s] create node for gpio0a7 error!\n",__func__);
        	return -1;
    	}*/

	if(sysfs_create_file(&(mydev->kobj), &dev_attr_ltr578_value.attr))
    	{
        	printk("===>YHF[%s] create node for ltr578 error!\n",__func__);
        	return -1;
    	}

	if(sysfs_create_file(&(mydev->kobj), &dev_attr_stk3x1x_value.attr))
    	{
        	printk("===>YHF[%s] create node for stk3x1x error!\n",__func__);
        	return -1;
    	}

	if(sysfs_create_file(&(mydev->kobj), &dev_attr_ltr578_cal_factor.attr))
    	{
        	printk("===>YHF[%s] create node for ltr578 error!\n",__func__);
        	return -1;
    	}

	if(sysfs_create_file(&(mydev->kobj), &dev_attr_stk3x1x_cal_factor.attr))
    	{
        	printk("===>YHF[%s] create node for stk3x1x error!\n",__func__);
        	return -1;
    	}

	if(sysfs_create_file(&(mydev->kobj), &dev_attr_hall.attr))
    	{
        	printk("===>YHF[%s] create node for stk3x1x error!\n",__func__);
        	return -1;
    	}

	mynode_dev = devm_input_allocate_device(mydev);
	if (!mynode_dev) {
		printk("Can't allocate lightsensor calibration key input\n");
		return -ENOMEM;
	}
	mynode_dev->name = "ls-calibration";
	mynode_dev->phys = "ls_calibration/input0";
	mynode_dev->id.bustype = BUS_HOST;
	input_set_capability(mynode_dev, EV_KEY, KEY_F7);
	input_set_capability(mynode_dev, EV_KEY, KEY_F8);
	ret = input_register_device(mynode_dev);
	if (ret) {
		printk("Unable to register lightsensor calibration input device, error: %d\n",
			ret);
		return ret;
	}

	printk("===>YHF[%s] end!\n",__func__);

	return 0;
}

static void __exit mynode_exit(void)
{
	printk("===>YHF[%s] \n",__func__);

    device_destroy(cls, MKDEV(major,0));
    class_destroy(cls);
    unregister_chrdev(major, "mynode");
}

late_initcall_sync(mynode_init);
//late_initcall_sync(mynode_exit);

MODULE_AUTHOR("yehongfeng@boe.com.cn");
MODULE_DESCRIPTION("platform filenode driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mynode");
