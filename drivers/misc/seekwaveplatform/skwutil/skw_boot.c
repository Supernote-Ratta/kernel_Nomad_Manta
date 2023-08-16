/*****************************************************************
 *Copyright (C) 2021 Seekwave Tech Inc.
 *Filename : skw_boot.c
 *Authors:seekwave platform
 *
 * This software is licensed under the terms of the the GNU
 * General Public License version 2, as published by the Free
 * Software Foundation, and may be copied, distributed, and
 * modified under those terms.
 *
 * This program is distributed in the hope that it will be usefull,
 * but without any warranty;without even the implied warranty of
 * merchantability or fitness for a partcular purpose. See the
 * GUN General Public License for more details.
 * **************************************************************/

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/completion.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/mmc/sdio_func.h>
#include <linux/dma-mapping.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include "sv6160_addr_map.h"
#include "skw_boot.h"
#include "boot_config.h"
/**************************sdio boot start******************************/
//#define  __FIRSTBOOT_BT_DEBUG
#define  __SKWBOOT_DEBUG_LOG
#define __SKW_DOUBLE_IMG_BOOT_MODE

int g_firstboot_bt = 0;

int fpga_debug = 0;
module_param(fpga_debug, int, S_IRUGO);
int is_dloader_flag =0;
int image_size=0;
unsigned char dl_signal_acount=0;
unsigned int start_addr_reg = 0x20202100;
struct platform_device *btboot_pdev;
static u64 port_dmamask = DMA_BIT_MASK(32);
//#define SDIO_BUFFER_SIZE	 (16*1024)
enum skw_sub_sys {
	SKW_BSP =1,
	SKW_WIFI,
	SKW_BLUETOOTH,
	SKW_ALL,
};

/***************************************************************************
 * Description:Create the FPGA debug mode set
 *Seekwave tech LTD
 *Author:junwei.jiang
 *Date:2022-01-19
 *Modify:
 * ************************************************************************/
enum skw_fpga_debug_mode {
	NORMAL_BOOT_MODE,		//[:0] first boot and start or close cp service
	FPGA_DEBUG_COMMON_MODE, //[:1] fpga_debug common debug code
	FPGA_FIRST_BOOT_MODE,	//[:2] only ap dl cp and running cp bin
	FPGA_TRACE_BOOT_MODE,	//[:3] AP start or close service CP start trace shell
	FPGA_AP2CP_BOOT_MODE,	//[:4]
	FPGA_AUTO_TEST_MODE,	//[:5] autotest first boot and download.
	FPGA_FIRST_BOOTBT_MODE, //[:6] first boot bt and start bt service
	FPGA_FIRST_BOOTWIFI_MODE,//[:7] first boot WIFI and start wifi service
};

/***************************************************************************
 * Description:
 *Seekwave tech LTD
 *Author:junwei.jiang
 *Date:
 *Modify:
 * ************************************************************************/
/*the bsp bt wifi style*
 * bsp :01
 * wifi:02
 * bt: 03 */
#define BSP_IMG_STYLE	1
#define WIFI_IMG_STYLE	2
#define BT_IMG_SYTLE	3

/*
 *add the little endian
 * */
#define _LITTLE_ENDIAN  1

#define CP_IMG_HEAD0	"kees"		 //"6B656573"
#define CP_IMG_HEAD1	"0616"		//"30363136"
#define CP_IMG_TAIL0	"evaw"		//"65766177"
#define CP_IMG_TAIL1	"0616"		//"30363136" //ASCII code 36 31 36 30

#define IMG_HEAD_OPS_LEN	4
#define RAM_ADDR_OPS_LEN	8
#define MODULE_INFO_LEN		12
//#define IMG_HEAD_INFOR_RANGE	0x2800  //10K Byte

#define IMG_HEAD_INFOR_RANGE	0x200  //10K Byte

int skw_sdio_dl_img(void);
int skw_sdio_img_read(void);
int skw_sdio_dl_btbin(void);
unsigned int EndianConv_32(unsigned int value);
int sdio_dloader(unsigned int subsys);
int skw_bind_boot_driver(struct device *dev);
/***********sdio drv extern interface **************/
/* driect mode,reg access.etc */
extern int skw_get_chipid(unsigned int system_addr, void *buf, unsigned int len);
extern int skw_boot_loader(struct seekwave_device *boot_data);
extern void *skw_get_bus_dev(void);
extern int skw_reset_bus_dev(void);
int skw_dloader(unsigned int subsys);
int skw_first_boot(struct seekwave_device *boot_data);
int skw_doubleimg_first_boot(struct seekwave_device *boot_data);
int skw_boot_bt(struct seekwave_device *boot_data ,int service_state);
static int skw_boot_wifi(struct seekwave_device *boot_data);
int skw_bootimg_analysis(struct seekwave_device *boot_data);
int skw_bootimg_analysis_new(struct seekwave_device *boot_data);
void skw_boot_debug_log(int value);
int skw_boot_init(struct seekwave_device *boot_data);
int skw_start_wifi_service(void);
int skw_cp_exception_reboot(void);
int skw_start_bt_service(void);
int skw_stop_wifi_service(void);
int skw_stop_bt_service(void);
/**************************sdio boot end********************************/
struct seekwave_device *boot_data;
/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/

static unsigned int crc_16_l_calc(char *buf_ptr,unsigned int len)
{
	unsigned int i;
	unsigned short crc=0;

	while(len--!=0)
	{
		for(i= CRC_16_L_SEED;i!=0;i=i>>1)
		{
			if((crc &CRC_16_L_POLYNOMIAL)!=0)
			{
				crc= crc<<1;
				crc= crc ^ CRC_16_POLYNOMIAL;
			}else{
				crc = crc <<1;
			}

			if((*buf_ptr &i)!=0)
			{
				crc = crc ^ CRC_16_POLYNOMIAL;
			}
		}
		buf_ptr++;
	}
	return (crc);
}


static int skw_request_firmwares(struct seekwave_device *boot_data,
	const char *dram_image_name, const char *iram_image_name)
{
	int ret;
	const struct firmware *fw;

	ret = request_firmware(&fw, dram_image_name, NULL);
	if (ret) {
		pr_err("request_firmware %s fail\n", dram_image_name);
		goto ret;
	}

	if (fw->size <= 0) {
		ret = -EINVAL;
		goto relese_fw;
	}

	boot_data->dram_img_data = (char *)kzalloc(fw->size, GFP_KERNEL);
	if (boot_data->dram_img_data == NULL) {
		pr_err("alloc memory %ld failed\n", fw->size);
		ret = -ENOMEM;
		goto relese_fw;
	}
	skwboot_log("boot data dram_img_data %p\n",boot_data->dram_img_data);
	memcpy(boot_data->dram_img_data, fw->data, fw->size);
	boot_data->dram_dl_size = fw->size;
	release_firmware(fw);
	//dram crc16
	boot_data->dram_crc_en = 1;
	boot_data->dram_crc_offset=0;
	boot_data->dram_crc_val = crc_16_l_calc(boot_data->dram_img_data + boot_data->dram_crc_offset, boot_data->dram_dl_size);

	ret = request_firmware(&fw, iram_image_name, NULL);
	if (ret) {
		pr_err("request_firmware %s fail\n", iram_image_name);
	}

	if (fw->size <= 0) {
		ret = -EINVAL;
		goto relese_fw;
	}

	boot_data->iram_img_data = (char *)kzalloc(fw->size, GFP_KERNEL);
	if (boot_data->iram_img_data == NULL) {
		pr_err("alloc memory %ld failed\n", fw->size);
		ret = -ENOMEM;
		goto relese_fw;
	}
	memcpy(boot_data->iram_img_data, fw->data, fw->size);
	boot_data->iram_dl_size = fw->size;
	ret = 0;
	//iram crc16
	boot_data->iram_crc_en = 1;
	boot_data->iram_crc_offset=0;
	boot_data->iram_crc_val = crc_16_l_calc(boot_data->iram_img_data + boot_data->iram_crc_offset, boot_data->iram_dl_size);

relese_fw:
	release_firmware(fw);
ret:
	return ret;
}

static int seekwave_boot_parse_dt(struct platform_device *pdev, struct seekwave_device *boot_data)
{
	int ret = 0;
	enum of_gpio_flags flags;
	struct device_node *np = pdev->dev.of_node;
	/*add the dma type dts config*/
	if (of_property_read_u32(np, "dma_type", &(boot_data->dma_type))){
		boot_data->dma_type = ADMA;
		boot_data->chip_en = MODEM_ENABLE_GPIO; 
		boot_data->host_gpio =  HOST_WAKEUP_GPIO_IN;
		boot_data->chip_gpio =  MODEM_WAKEUP_GPIO_OUT;
		skwboot_log("no DTS setting\n");
	} else {
		boot_data->host_gpio = of_get_named_gpio_flags(np, "gpio_host_wake", 0, &flags);
		boot_data->chip_gpio = of_get_named_gpio_flags(np, "gpio_chip_wake",0, &flags);
		boot_data->chip_en = of_get_named_gpio_flags(np, "gpio_chip_en",0, &flags);
	}
	if (boot_data->host_gpio >= 0) {
		ret = devm_gpio_request_one(&pdev->dev, boot_data->host_gpio, GPIOF_IN, "HOST_WAKE" );
		if(ret < 0){
			skwboot_err("%s:gpio_host request fail ret=%d\n",__func__, ret);
		}
		if (boot_data->chip_gpio >= 0)
			ret = devm_gpio_request_one(&pdev->dev, boot_data->chip_gpio, GPIOF_OUT_INIT_HIGH,"CHIP_WAKE");

		skwboot_log("%s, gpio_out:%d gpio_in:%d state = %d\n", __func__, boot_data->chip_gpio,
			boot_data->host_gpio, gpio_get_value(boot_data->host_gpio));
	}
	if (boot_data->chip_en >= 0)
		ret = devm_gpio_request_one(&pdev->dev, boot_data->chip_en, GPIOF_OUT_INIT_HIGH,"CHIP_EN");
	return ret;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int seekwave_check_cp_ready(void)
{

	return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
int seekwave_boot_stop(u32 subsys)
{
	skwboot_log("seekwave boot stop done:%s\n",__func__);
	return 0;
}
EXPORT_SYMBOL_GPL(seekwave_boot_stop);

/************************************************************************/
//Description: BT start service
//Func: BT start service
//Call：
//Author:junwei.jiang
//Date:2021-11-1
//Modify:
/************************************************************************/
extern void kernel_restart(char *cmd);

static int bt_start_service(int id, void *callback, void *data)
{
	int ret=0;
	skwboot_log("%s line:%d Enter \n", __func__, __LINE__);
	if(fpga_debug == FPGA_FIRST_BOOTBT_MODE ||fpga_debug==FPGA_AUTO_TEST_MODE){
		if(!g_firstboot_bt){
			g_firstboot_bt = 1;
			ret = skw_doubleimg_first_boot(boot_data);
		}
	}else {
		ret = skw_start_bt_service();
	}
	if(ret < 0){
		skwboot_err("%s boot bt fail \n", __func__);
		return -1;
	}
	skwboot_log("%s line:%d  boot sucessfuly\n", __func__, __LINE__);
	if(fpga_debug == FPGA_AUTO_TEST_MODE){
		kernel_restart(0);
	}
	return 0;
}

/************************************************************************/
//Description: BT stop service
//Func: BT stop service
//Call：
//Author:junwei.jiang
//Date:2021-11-1
//Modify:
/************************************************************************/
static int bt_stop_service(int id)
{
	int ret=0;

	skwboot_log("%s line:%d Enter \n", __func__, __LINE__);
	if(fpga_debug== FPGA_FIRST_BOOTBT_MODE){
		return 0;
	}
	ret = skw_stop_bt_service();
	if(ret < 0){
		skwboot_err("%s boot bt fail \n", __func__);
		return -1;
	}
	skwboot_log("bt_stop_service OK\n");
	return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int seekwave_boot_probe(struct  platform_device *pdev)
{
	int ret;
	int time_count=0;
	void *io_bus;

	boot_data = devm_kzalloc(&pdev->dev, sizeof(struct seekwave_device), GFP_KERNEL);
	if (!boot_data) {
		skwboot_err("%s :kzalloc error !\n", __func__);
		return -ENOMEM;
	}
	seekwave_boot_parse_dt(pdev, boot_data);
	skw_boot_init(boot_data);
	io_bus = skw_get_bus_dev();
	if (!io_bus) {
		if(boot_data->iram_dl_size>0) {
			skwboot_log("%s :CHIP_RESET AGAIN!\n", __func__);
			gpio_set_value(boot_data->chip_en,0);
			msleep(20);
			gpio_set_value(boot_data->chip_en, 1);
		}
		do {
			msleep(10);
			io_bus = skw_get_bus_dev();
		} while(!io_bus && time_count++ < 50);
	}
	if (!io_bus) {
		skwboot_err("%s get bus dev fail !\n",__func__);
		return -ENODEV;
	}
	skw_bind_boot_driver(io_bus);
	ret = skw_doubleimg_first_boot(boot_data);
	return ret;
}
/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int seekwave_boot_remove(struct  platform_device *pdev)
{
	skwboot_log("%s the Enter \n", __func__);

	if (boot_data->host_gpio) {
		int irq = gpio_to_irq(boot_data->host_gpio);
		free_irq(irq, NULL);
	}
	if (btboot_pdev) {
		platform_device_unregister(btboot_pdev);
		btboot_pdev = NULL;
	}
	if(boot_data){
		if(boot_data->iram_img_data){
			kfree(boot_data->iram_img_data);
			boot_data->iram_img_data = NULL;
		}
		if(boot_data->dram_img_data){
			kfree(boot_data->dram_img_data);
			boot_data->dram_img_data = NULL;
		}
		if(boot_data->dl_bin){
			kfree(boot_data->dl_bin);
			boot_data->dl_bin = NULL;
		}
		if(boot_data->img_data){
			kfree(boot_data->img_data);
			boot_data->img_data = NULL;
		}
		boot_data->iram_file_path = NULL;
		boot_data->dram_file_path = NULL;
		devm_kfree(&pdev->dev, boot_data);
		boot_data=NULL;
	}
	return 0;
}
extern void skw_sdio_log_stop_rec(void);
static void seekwave_boot_shutdown(struct platform_device *pdev)
{
	printk("%s enter ...\n", __func__);
	skw_sdio_log_stop_rec();
	skw_reset_bus_dev();
}
static const struct of_device_id seekwave_match_table[] ={

	{ .compatible = "seekwave,sv6160"},
	{ },
};

static struct platform_driver seekwave_driver ={

	.driver = {
		.owner = THIS_MODULE,
		.name  = "sv6160",
		.of_match_table = seekwave_match_table,
	},
	.probe = seekwave_boot_probe,
	.remove = seekwave_boot_remove,
	.shutdown = seekwave_boot_shutdown,
};

/***********************************************************************
 *Description:BT download boot pdata
 *Seekwave tech LTD
 *Author:junwei.jiang
 *Date:2021-11-3
 *Modify:
 ***********************************************************************/
struct sv6160_platform_data boot_pdata = {
	.data_port = 8,
	.bus_type = SDIO_LINK,
	.max_buffer_size = 0x800,
	.align_value = 4,
	.open_port = bt_start_service,
	.close_port = bt_stop_service,
};

/***************************************************************
 *Description:BT bind boot driver
 *Seekwave tech LTD
 *Author:junwei.jiang
 *Date:2021-11-3
 *Modify:
***************************************************************/
int skw_bind_boot_driver(struct device *dev)
{
	struct platform_device *pdev;
	char	pdev_name[32];
	int ret = 0;
	sprintf(pdev_name, "skw_ucom");
/*
 *	creaete BT DATA device
 */
	if(!dev){
		skwboot_err("%s the dev fail \n", __func__);
		return -1;
	}
	pdev = platform_device_alloc(pdev_name, PLATFORM_DEVID_AUTO);
	if(!pdev)
		return -ENOMEM;
	pdev->dev.parent = dev;
	pdev->dev.dma_mask = &port_dmamask;
	pdev->dev.coherent_dma_mask = port_dmamask;
	boot_pdata.port_name = "BTBOOT";
	boot_pdata.data_port = 8;
	ret = platform_device_add_data(pdev, &boot_pdata, sizeof(boot_pdata));
	if(ret) {
		dev_err(dev, "failed to add boot data \n");
		platform_device_put(pdev);;
		return ret;
	}
	ret = platform_device_add(pdev);
	if(ret) {
		platform_device_put(pdev);
		skwboot_err("%s,line:%d the device add fail \n",__func__,__LINE__);
		return ret;
	}
	btboot_pdev = pdev;
	return ret;
}
#ifndef CONFIG_OF
static void seekwave_release(struct device *dev)
{
}
static struct platform_device seekwave_device ={
	.name = "sv6160",
	.dev = {
		.release = seekwave_release,
	}
};
#endif
static int seekwave_boot_init(void)
{
	btboot_pdev = NULL;
	skw_ucom_init();
#ifndef CONFIG_OF
	platform_device_register(&seekwave_device);
#endif
	return platform_driver_register(&seekwave_driver);
}

static void seekwave_boot_exit(void)
{
	skw_ucom_exit();
#ifndef CONFIG_OF
	platform_device_unregister(&seekwave_device);
#endif
	platform_driver_unregister(&seekwave_driver);

}

/****************************************************************
 *Description:the data Little Endian process interface
 *Func:EndianConv_32
 *Calls:None
 *Call By:The img data process
 *Input:value
 *Output:the Endian data
 *Return：value
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-08-26
 * **************************************************************/
unsigned int EndianConv_32(unsigned int value)
{
#ifdef _LITTLE_ENDIAN
	unsigned int nTmp = (value >>24 | value <<24);
	nTmp |= ((value >> 8) & 0x0000FF00);
	nTmp |= ((value << 8) & 0x00FF0000);
	return nTmp;
#else
	return value;
#endif
}

/****************************************************************
 *Description:dram read the double img file
 *Func:
 *Calls:
 *Call By:sdio_dloader
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
int skw_download_signal_ops(void)
{
	unsigned int tmp_signal = 0;
	//download done flag ++
	dl_signal_acount ++;
	tmp_signal = dl_signal_acount;
	boot_data->dl_done_signal = 0xff&tmp_signal;
	boot_data->dl_acount_addr = SKW_SDIO_PD_DL_AP2CP_BSP;

	//gpio need set high or low power interrupt to cp wakeup
	boot_data->gpio_out = boot_data->chip_gpio;
	if(boot_data->gpio_val)
		boot_data->gpio_val =0;
	else
		boot_data->gpio_val =1;
	skwboot_log("%s line:%d download data ops done \n", __func__, __LINE__);
	return 0;
}

/****************************************************************
 *Description:analysis the double img dram iram
 *Func:
 *Calls:
 *Call By:sdio_dloader
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
int skw_boot_init(struct seekwave_device *boot_data)
{
	int i =0;
	unsigned int head_offset=0;
	unsigned int tail_offset=0;
	int ret = 0;
	struct img_head_data_t dl_data_info;
	unsigned int *data=NULL;
	unsigned int *dl_addr_data=NULL;
	
	ret = skw_request_firmwares(boot_data, "RAM_RW_KERNEL_DRAM", "ROM_EXEC_KERNEL_IRAM");
	skwboot_log("image_size=%d,%d, ret=%d\n", boot_data->iram_dl_size, boot_data->dram_dl_size, ret);
	if (ret < 0)
		return ret;

	boot_data->head_addr = 0;
	boot_data->tail_addr = 0;
	boot_data->bsp_head_addr = 0;
	boot_data->bsp_tail_addr = 0;
	boot_data->wifi_head_addr =0;
	boot_data->wifi_tail_addr = 0;
	boot_data->bt_head_addr = 0;
	boot_data->bt_tail_addr = 0;
	if(boot_data->iram_img_data!=NULL){
		/*analysis the img*/
		for(i=0; i*IMG_HEAD_OPS_LEN<IMG_HEAD_INFOR_RANGE; i++)
		{
			if(!head_offset)
			{
				if((0==memcmp(CP_IMG_HEAD0, boot_data->iram_img_data+i*IMG_HEAD_OPS_LEN,IMG_HEAD_OPS_LEN))&&
						(0==memcmp(CP_IMG_HEAD1,boot_data->iram_img_data+(i+1)*IMG_HEAD_OPS_LEN,IMG_HEAD_OPS_LEN)))
					head_offset = (i+1)*IMG_HEAD_OPS_LEN;
			}else if(!tail_offset){
				if((0==memcmp(CP_IMG_TAIL0, boot_data->iram_img_data+i*IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN))&&
						(0==memcmp(CP_IMG_TAIL1, boot_data->iram_img_data+(i+1)*IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN))){
					tail_offset = (i-1)*IMG_HEAD_OPS_LEN;
					break;
				}
			}
		}
		if(!tail_offset){
			skwboot_err("%s,%d,the iram_img not need analysis!!! or Fail!! \n",__func__,__LINE__);
			//boot_data->iram_img_data = NULL;
			return -1;
		}else{
			//get the iram img addr and dram img addr
			dl_addr_data = (unsigned int *)(boot_data->iram_img_data+head_offset+IMG_HEAD_OPS_LEN);
			boot_data->iram_dl_addr = dl_addr_data[0];
			boot_data->dram_dl_addr = dl_addr_data[1];
			head_offset = head_offset+RAM_ADDR_OPS_LEN;//jump the ram addr data;

			/*get the img head tail offset*/
			boot_data->head_addr = head_offset;
			boot_data->tail_addr = tail_offset;
			skwboot_log("%s line:%d,the tail_offset ---0x%x, the head_offset --0x%x ,iram_addr=0x%x,dram_addr=0x%x,\n",
					__func__, __LINE__,tail_offset, head_offset,boot_data->iram_dl_addr,boot_data->dram_dl_addr);
		}
		/*need download the img bin for WIFI or BT service dl_module >0*/
		head_offset = head_offset +IMG_HEAD_OPS_LEN;
		skwboot_log("%s line:%d analysis the img module\n", __func__, __LINE__);
		for(i=0; i*MODULE_INFO_LEN<=(tail_offset-head_offset); i++)
		{
			data = (unsigned int *)(boot_data->iram_img_data +head_offset+i*MODULE_INFO_LEN);
			dl_data_info.dl_addr=data[0];
			dl_data_info.write_addr =data[2];
			dl_data_info.index = 0x000000FF&EndianConv_32(data[1]);
			dl_data_info.data_size = 0x00FFFFFF&data[1];
			skwboot_log("%s line:%d dl_addr=0x%x, write_addr=0x%x, index=0x%x,data_size=0x%x\n", __func__,
					__LINE__, dl_data_info.dl_addr,dl_data_info.write_addr,dl_data_info.index,dl_data_info.data_size);

		}
	}
	//get the debug boot ko setting value
	if(fpga_debug)
		boot_data->fpga_debug=1;
	else
		boot_data->fpga_debug=0;

	return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:JUNWEI.JIANG
 *Date:
 *Modify:
 **************************************************************************/
int skw_cp_exception_reboot(void)
{
	int ret =0;
	boot_data->dl_module = SKW_ALL;
	//download done flag ++
	skwboot_log("%s Enter!! \n", __func__);
	skw_download_signal_ops();
	boot_data->first_dl_flag = 1;
	boot_data->cp_boot_config = 1;
	//end the update process
	ret = skw_boot_loader(boot_data);
	if(ret !=0){
		skwboot_err("%s, reboot fail \n", __func__);
		return -1;
	}
	skwboot_log("%s CP reboot pass \n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(skw_cp_exception_reboot);
/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
int skw_start_wifi_service(void)
{
	int ret =0;
	boot_data->service_ops = SKW_WIFI_START;
	boot_data->dl_module = SKW_WIFI;
	boot_data->wifi_service_state = START;
	boot_data->service_req_flag = START;
	boot_data->poweron_flag = SKW_WIFI; //WIFI 2 BT：3
	boot_data->poweron_wifi_flag = 1;
	boot_data->first_dl_flag = 1;
	//download done flag ++
	skw_download_signal_ops();
	ret = skw_boot_loader(boot_data);
	if(ret !=0){
		skwboot_err("%s,line:%d boot fail \n", __func__,__LINE__);
		return -1;
	}
	//first_boot wifi flag:
	if(!boot_data->first_bootwifi_flag)
		boot_data->first_bootwifi_flag =1;

	seekwave_check_cp_ready();
	skwboot_log("%s wifi boot sucessfull\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(skw_start_wifi_service);

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
int skw_stop_wifi_service(void)
{
	int ret =0;
	boot_data->service_ops = SKW_WIFI_STOP;
	boot_data->dl_module = 0;
	boot_data->poweron_wifi_flag = 0; //WIFI 2 BT：3
	boot_data->wifi_service_state = STOP;
	boot_data->service_req_flag = STOP;
	boot_data->poweron_flag = SKW_WIFI; //WIFI 2 BT：3
	//gpio need set high or low power interrupt to cp wakeup
	boot_data->gpio_out = boot_data->chip_gpio;
	if(boot_data->gpio_val)
		boot_data->gpio_val =0;
	else
		boot_data->gpio_val =1;
	ret = skw_boot_loader(boot_data);
	if(ret !=0)
	{
		skwboot_err("dload the img fail \n");
		return -1;
	}
	seekwave_check_cp_ready();
	skwboot_log("seekwave boot stop done:%s\n",__func__);
	return 0;
}
EXPORT_SYMBOL_GPL(skw_stop_wifi_service);


/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
int skw_start_bt_service(void)
{
	int ret=0;
	boot_data->service_ops = SKW_BT_START;
	boot_data->first_dl_flag = 1;
	boot_data->dl_module = SKW_BLUETOOTH;
	boot_data->bt_service_state = START;
	boot_data->service_req_flag = START;
	boot_data->poweron_bt_flag = 1; //WIFI 2 BT：3
	//download done flag ++
	skw_download_signal_ops();
	ret = skw_boot_loader(boot_data);
	if(ret !=0){
		skwboot_err("%s boot fail \n", __func__);
		return -1;
	}
	//first_boot bt flag:
	if(!boot_data->first_bootbt_flag)
		boot_data->first_bootbt_flag =1;

	seekwave_check_cp_ready();
	skwboot_log("%s line:%d , boot bt sucessfully!\n", __func__,__LINE__);
	return 0;
}
EXPORT_SYMBOL_GPL(skw_start_bt_service);

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/

int skw_stop_bt_service(void)
{
	int ret =0;
	boot_data->service_ops = SKW_BT_STOP;
	boot_data->dl_module = 0;
	boot_data->poweron_bt_flag = 0; //WIFI 2 BT：3
	boot_data->dl_module = SKW_BLUETOOTH;
	boot_data->bt_service_state = STOP;
	boot_data->service_req_flag = STOP;
	//gpio need set high or low power interrupt to cp wakeup
	boot_data->gpio_out = boot_data->chip_gpio;
	if(boot_data->gpio_val)
		boot_data->gpio_val =0;
	else
		boot_data->gpio_val =1;
	ret = skw_boot_loader(boot_data);
	if(ret < 0)
	{
		skwboot_err("dload the img fail \n");
		return -1;
	}
	//seekwave_check_cp_ready();
	skwboot_log("seekwave boot stop done:%s\n",__func__);
	return 0;
}
EXPORT_SYMBOL_GPL(skw_stop_bt_service);

/****************************************************************
 *Description:skw download bin interface for WIFI BT OR ALL
 *Func:skw_dloader
 *Calls:
 *Call By:the wifi host
 *Input: subsys BT WIFI OR BSP ALL
 *Output: download the img of the subsys bin
 *Return：0:pass ,others fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-09-08
 * **************************************************************/
int skw_dloader(unsigned int subsys)
{
	int ret = 0;
	switch(subsys)
	{
		case SKW_WIFI:
			ret = skw_boot_wifi(boot_data);
			if(ret !=0)
			{
				skwboot_err("%s new sdio dloader error \n", __func__);
			}
			break;
	case SKW_BLUETOOTH:
			ret = skw_boot_bt(boot_data,START);
			if(ret !=0)
			{
				skwboot_err("%s new sdio dloader error \n", __func__);
			}
			break;
		case SKW_BSP:
			ret = skw_doubleimg_first_boot(boot_data);
			break;
		default:
			skwboot_err("%s have not subsys request download img\n", __func__);
			break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(skw_dloader);

int skw_download_bin_ops(void *bin, unsigned int addr, unsigned int size)
{
	int ret=0;

	boot_data->dl_bin = bin;
	boot_data->dl_addr =addr;
	boot_data->dl_size = size;
	ret = skw_boot_loader(boot_data);
	if(ret < 0)
	{
		skwboot_err("dload the img fail \n");
		return -1;
	}
	skwboot_log("%s the download bin sucessfully\n", __func__);
	return 0;
}
/****************************************************************
 *Description:the seekwave bsp cp bin img analysis
 *Func:img_data_analysis and download
 *Calls:seekwave_read_image()
 *Call By:skw_dloader()
 *Input:subsys
 *Output:0:pass;other:fail
 *Return：
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-09-08
 * **************************************************************/
int skw_bootimg_analysis_new(struct seekwave_device *boot_data)
{
	struct img_head_data_t dl_data_info;
	int i =0;
	unsigned int head_offset=0;
	unsigned int tail_offset=0;
	unsigned int *data;
	char *img_data =boot_data->img_data;

	/*need download the img bin for WIFI or BT service dl_module >0*/
	head_offset = boot_data->head_addr;
	tail_offset = boot_data->tail_addr;
	head_offset = head_offset +IMG_HEAD_OPS_LEN;
	skwboot_log("%s line:%d the start analysis img\n", __func__, __LINE__);
	for(i=0; i*MODULE_INFO_LEN<=(tail_offset-head_offset); i++)
	{
		data = (unsigned int *)(img_data +head_offset+i*MODULE_INFO_LEN);
		dl_data_info.dl_addr=data[0];
		dl_data_info.write_addr =data[2];
		dl_data_info.index = 0x000000FF&EndianConv_32(data[1]);
		dl_data_info.data_size = 0x00FFFFFF&data[1];

		//WIFI pd or sleep  BSP bin WIFI bin need download
		if(fpga_debug ==FPGA_DEBUG_COMMON_MODE){
			/*dl_module== SKW_WIFI*/
			if(boot_data->dl_module== SKW_WIFI){
				if(boot_data->bt_service_state==START){
					if(dl_data_info.index == SKW_WIFI){
						if(dl_data_info.dl_addr == boot_data->wifi_tail_addr){
							boot_data->wifi_service_state =START;
							skwboot_log("%s line:%d START WIFI service \n", __func__, __LINE__);
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
							break;
						}else{
							if(dl_data_info.dl_addr== boot_data->wifi_head_addr && !boot_data->wifi_tail_addr){
								boot_data->wifi_service_state = START;
								skwboot_log("%s line:%d START WIFI service \n", __func__, __LINE__);
							}else{
								boot_data->wifi_service_state = 0;
								skwboot_log("%s line:%d download wifi service bin \n", __func__, __LINE__);
							}
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
						}
						/*just download the WIFI bin*/
						//skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
						//		dl_data_info.write_addr,dl_data_info.data_size);
					}
					/*dl_module== SKW_WIFI*/
				}else if(boot_data->bt_service_state==STOP || (!boot_data->first_bootbt_flag)){
					if(dl_data_info.index == SKW_BSP || dl_data_info.index == SKW_WIFI){
						/*need download the WIFI BSP bin*/
						/*just download the BT bin*/
						if(dl_data_info.dl_addr == boot_data->wifi_tail_addr){
							boot_data->wifi_service_state =START;
							skwboot_log("%s line:%d START WIFI service \n", __func__, __LINE__);
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
							break;
						}else{
							if(dl_data_info.dl_addr == boot_data->wifi_head_addr && !boot_data->wifi_tail_addr){
								boot_data->wifi_service_state = START;
								skwboot_log("%s line:%d START WIFI service \n", __func__, __LINE__);
							}else{
								boot_data->wifi_service_state = 0;
								skwboot_log("%s line:%d download wifi service bin \n", __func__, __LINE__);
							}
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
						}
						//skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
						//		dl_data_info.write_addr,dl_data_info.data_size);
					}
				}
			} else if(boot_data->dl_module== SKW_BLUETOOTH){
				/*dl_module == SKW_BLUETOOTH*/
				if(boot_data->wifi_service_state == START){
					if(dl_data_info.index == SKW_BLUETOOTH){
						/*just download the BT bin*/
						if(dl_data_info.dl_addr == boot_data->bt_tail_addr){
							boot_data->bt_service_state =START;
							skwboot_log("%s line:%d start bt service \n", __func__, __LINE__);
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
							break;
						}else{
							if(dl_data_info.dl_addr == boot_data->bt_head_addr && !boot_data->bt_tail_addr){
								boot_data->bt_service_state = START;
								skwboot_log("%s line:%d START BT service \n", __func__, __LINE__);
							}else{
								boot_data->bt_service_state = 0;
								skwboot_log("%s line:%d download bt service bin \n", __func__, __LINE__);
							}
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
						}
					}
					/*dl_module == SKW_BLUETOOTH*/
				}else if(boot_data->wifi_service_state == STOP || !boot_data->first_bootwifi_flag){
					if(dl_data_info.index == SKW_BSP || dl_data_info.index == SKW_BLUETOOTH){
						/*need download the WIFI BSP bin*/
						if(dl_data_info.dl_addr == boot_data->bt_tail_addr){
							boot_data->bt_service_state =START;
							skwboot_log("%s line:%d START BT  SERVICE---- \n", __func__, __LINE__);
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
							break;
						}else{
							if(dl_data_info.dl_addr == boot_data->bt_head_addr && !boot_data->bt_tail_addr){
								boot_data->bt_service_state = START;
								skwboot_log("%s line:%d START BT SERVICE two \n", __func__, __LINE__);
							}else{
								boot_data->bt_service_state = 0;
								skwboot_log("%s line:%d DOWNLOAD BT BIN\n", __func__, __LINE__);
							}
							skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
									dl_data_info.write_addr,dl_data_info.data_size);
						}
					}
				}else{
					skwboot_log("%s line:%d BT not need the analysis img \n", __func__, __LINE__);
				}
			}
		//	if(dl_data_info.index == boot_data->dl_module){
		//		skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
		//				dl_data_info.write_addr,dl_data_info.data_size);
		//	}
		} else {
			/*dl_module== SKW_WIFI*/
			if(boot_data->bt_service_state==START){
				if(dl_data_info.index == SKW_WIFI){
					/*just download the WIFI bin*/
					skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
							dl_data_info.write_addr,dl_data_info.data_size);
				}
			/*dl_module== SKW_WIFI*/
			}else if(boot_data->bt_service_state==STOP){
				if(dl_data_info.index == SKW_BSP || dl_data_info.index == SKW_WIFI){
					/*need download the WIFI BSP bin*/
					skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
							dl_data_info.write_addr,dl_data_info.data_size);
				}
			/*dl_module == SKW_BLUETOOTH*/
			}else if(boot_data->wifi_service_state == START){
				if(dl_data_info.index == SKW_BLUETOOTH){
					/*just download the BT bin*/
					skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
							dl_data_info.write_addr,dl_data_info.data_size);
				}
			/*dl_module == SKW_BLUETOOTH*/
			}else if(boot_data->wifi_service_state == STOP){
				if(dl_data_info.index == SKW_BSP || dl_data_info.index == SKW_BLUETOOTH){
					/*need download the WIFI BSP bin*/
					skw_download_bin_ops((void *)(img_data+dl_data_info.dl_addr),
							dl_data_info.write_addr,dl_data_info.data_size);
				}
			}else{
				skwboot_log("%s,line:%d, noting need dl\n",__func__, __LINE__);
				break;
			}
		}
	}
	return 0;
}

/****************************************************************
 *Description:double img analysis and download
 *Func:
 *Calls:skw_doubleimg_first_boot
 *Call By:skw_dloader()
 *Input:subsys
 *Output:0:pass;other:fail
 *Return：
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
int skw_doubleimg_analysis(struct seekwave_device *boot_data)
{
	struct img_head_data_t dl_data_info;
	int i =0;
	unsigned int head_offset=0;
	unsigned int tail_offset=0;
	unsigned int *data;
	char *iram_img_data =boot_data->iram_img_data;
	char *dram_img_data= boot_data->dram_img_data;

	/*need download the img bin for WIFI or BT service dl_module >0*/
	head_offset = boot_data->head_addr;
	tail_offset = boot_data->tail_addr;
	head_offset = head_offset +IMG_HEAD_OPS_LEN;
	skwboot_log("%s line:%d the start analysis img\n", __func__, __LINE__);
	for(i=0; i*MODULE_INFO_LEN<=(tail_offset-head_offset); i++)
	{
		data = (unsigned int *)(iram_img_data +head_offset+i*MODULE_INFO_LEN);
		dl_data_info.dl_addr=data[0];
		dl_data_info.write_addr =data[2];
		dl_data_info.index = 0x000000FF&EndianConv_32(data[1]);
		dl_data_info.data_size = 0x00FFFFFF&data[1];
		skwboot_log("%s line:%d the dl_addr=0x%x , write_addr= 0x%x ,index=%d, data_size=0x%x \n ",
				__func__, __LINE__, dl_data_info.dl_addr,dl_data_info.write_addr,dl_data_info.index,
				dl_data_info.data_size);
		if((dl_data_info.dl_addr&0xFFFF0000)== boot_data->iram_dl_addr){
			skw_download_bin_ops((void*)(iram_img_data+dl_data_info.dl_addr),
					dl_data_info.write_addr,dl_data_info.data_size);
		}else if((dl_data_info.dl_addr&0xFFFF0000)== boot_data->dram_dl_addr){
				skw_download_bin_ops((void*)(dram_img_data+dl_data_info.dl_addr),
					dl_data_info.write_addr,dl_data_info.data_size);	
		}else{
			skwboot_log("have nothing download !!!!\n");
		}
	}
	return 0;
}

/****************************************************************
 *Description:the seekwave bsp cp bin img analysis
 *Func:img_data_analysis and download
 *Calls:seekwave_read_image()
 *Call By:skw_dloader()
 *Input:subsys
 *Output:0:pass;other:fail
 *Return：
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-09-08
 * **************************************************************/
int skw_bootimg_analysis(struct seekwave_device *boot_data)
{
	int ret =0;
	struct img_head_data_t dl_data_info;
	int i =0;
	unsigned int head_offset=0;
	unsigned int tail_offset=0;
	unsigned int *data;
	char *img_data =boot_data->img_data;

#if 0//def DEBUG
	print_hex_dump(KERN_ERR, "boot data ", 0, 16, 1,
			boot_data->img_data, boot_data->img_size, 1);
#endif
	for(i=0; i*IMG_HEAD_OPS_LEN<IMG_HEAD_INFOR_RANGE; i++)
	{
		if(!head_offset)
		{
			if((0==memcmp(CP_IMG_HEAD0, img_data+i*IMG_HEAD_OPS_LEN,IMG_HEAD_OPS_LEN))&&
					(0==memcmp(CP_IMG_HEAD1,img_data+(i+1)*IMG_HEAD_OPS_LEN,IMG_HEAD_OPS_LEN)))
				head_offset = (i+1)*IMG_HEAD_OPS_LEN;
		}
		else if(!tail_offset)
		{
			if(0==memcmp(CP_IMG_TAIL0, img_data+i*IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)){
				if(0==memcmp(CP_IMG_TAIL1, img_data+(i+1)*IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)){
					tail_offset = i*IMG_HEAD_OPS_LEN;
					break;
				}
			}
		}
	}
	skwboot_log("the tail_offset ---0x%x, the head_offset --0x%x \n", tail_offset, head_offset);
	if(!tail_offset){
		skwboot_err("%s,%d,the get head or tail data fail \n",__func__,__LINE__);
		img_data = NULL;
		return -1;
	}
	/*kzalloc the data buf size */
	head_offset = head_offset +IMG_HEAD_OPS_LEN;
	for(i=0; i*MODULE_INFO_LEN<=(tail_offset-head_offset); i++)
	{
		data = (unsigned int *)(img_data +head_offset+i*MODULE_INFO_LEN);
		dl_data_info.dl_addr=data[0];
		dl_data_info.write_addr =data[2];
		dl_data_info.index = 0x000000FF&EndianConv_32(data[1]);
		dl_data_info.data_size = 0x00FFFFFF&data[1];
		//WIFI pd or sleep  BSP bin WIFI bin need download
		if(fpga_debug ==FPGA_DEBUG_COMMON_MODE){
			if(dl_data_info.index == boot_data->dl_module)//if(SKW_BLUETOOTH == boot_data->dl_module)
			{
				boot_data->dl_bin = (void *)(img_data +dl_data_info.dl_addr);
				boot_data->dl_addr = dl_data_info.write_addr;
				boot_data->dl_size = dl_data_info.data_size;
				ret = skw_boot_loader(boot_data);
				if(ret !=0)
				{
					skwboot_err("dload the img fail \n");
					img_data =NULL;
					return -1;
				}
				skwboot_log("%s the download bin sucessfully\n", __func__);
			}
		} else{
			if(dl_data_info.index == SKW_BSP || dl_data_info.index == boot_data->dl_module)
			//if(dl_data_info.index == boot_data->dl_module)
			{
				boot_data->dl_bin = (void *)(img_data +dl_data_info.dl_addr);
				boot_data->dl_addr = dl_data_info.write_addr;
				boot_data->dl_size = dl_data_info.data_size;
				ret = skw_boot_loader(boot_data);
				if(ret !=0)
				{
					skwboot_err("dload the img fail \n");
					img_data =NULL;
					return -1;
				}
				skwboot_log("%s the download bin sucessfully\n", __func__);
			}
		}
	}
	return 0;
}

/****************************************************************
 *Description:double iram dram img first boot cp
 *Func:
 *Calls:
 *Call By:skw_doubleimg_first_boot
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
int skw_doubleimg_first_boot(struct seekwave_device *boot_data)
{
	int ret =0;
	//get the img data
#ifdef DEBUG_SKWBOOT_TIME
	ktime_t cur_time,last_time;
	cur_time = ktime_get();
#endif
	//set download the value;
	boot_data->service_ops = SKW_NO_SERVICE;
	boot_data->save_setup_addr = SKW_SDIO_PD_DL_AP2CP_BSP; //160
	boot_data->poweron_flag = 1;
	boot_data->gpio_out = boot_data->chip_gpio;
	boot_data->gpio_val = 0;
	boot_data->dl_module = 0;
	boot_data->first_dl_flag =0;
	boot_data->gpio_in  = boot_data->host_gpio;
	boot_data->dma_type_addr = SKW_SDIO_PLD_DMA_TYPE;
	boot_data->service_req_flag = 0;
	boot_data->wifi_service_state = 0;
	boot_data->bt_service_state = 0;
#ifdef __FIRSTBOOT_BT_DEBUG
	//boot_data->dl_addr = SKW_BOOT_BT_START_ADDR;
	boot_data->dl_module = SKW_BLUETOOTH;
	boot_data->bt_service_state = START;
	boot_data->service_req_flag = START;
	boot_data->poweron_bt_flag = 1; //WIFI 2 BT：3
#endif
	if(fpga_debug >0)
		boot_data->first_dl_flag =1;
	ret = skw_boot_loader(boot_data);
	if(ret < 0){
		skwboot_err("%s firt boot cp fail \n", __func__);
		return -1;
	}
	//download done set the download flag;
	boot_data->first_dl_flag =1;
	boot_data->poweron_wifi_flag = 0;
	boot_data->first_bootwifi_flag = 0;
	boot_data->first_bootbt_flag =0;

	//download done tall cp acount;
	boot_data->dl_done_signal &= 0xFF;
	boot_data->dl_done_signal +=1;
	skwboot_log("%s first boot pass\n", __func__);
#ifdef DEBUG_SKWBOOT_TIME
	last_time = ktime_get();
	skwboot_log("%s,the download time start time %llu and the over time %llu \n",
			__func__, cur_time, last_time);
#endif
	return ret;
}

/****************************************************************
 *Description:first boot cp
 *Func:
 *Calls:
 *Call By:skw_first_boot
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-08-26
 * **************************************************************/
int skw_first_boot(struct seekwave_device *boot_data)
{
	int ret =0;
	//get the img data
#ifdef DEBUG_SKWBOOT_TIME
	ktime_t cur_time,last_time;
	cur_time = ktime_get();
#endif
	//set download the value;
	boot_data->dl_bin =boot_data->img_data;
	boot_data->dl_addr = SKW_BOOT_START_ADDR;
	boot_data->setup_addr = SKW_BOOT_START_ADDR;
	boot_data->dl_size = boot_data->img_size;
	boot_data->save_setup_addr = SKW_SDIO_PD_DL_AP2CP_BSP; //160
	boot_data->poweron_flag = 1;
	boot_data->gpio_out = boot_data->chip_gpio;
	boot_data->gpio_val = 0;
	boot_data->dl_module = 0;
	boot_data->first_dl_flag =0;
	boot_data->gpio_in  = boot_data->host_gpio;
	boot_data->dma_type_addr = SKW_SDIO_PLD_DMA_TYPE;
	boot_data->dma_type = boot_data->dma_type;
	boot_data->service_req_flag = 0;
	boot_data->wifi_service_state = 0;
	boot_data->bt_service_state = 0;
//#ifdef __FIRSTBOOT_BT_DEBUG
	if(fpga_debug== FPGA_FIRST_BOOTBT_MODE){
		//boot_data->dl_addr = SKW_BOOT_BT_START_ADDR;
		boot_data->dl_module = SKW_BLUETOOTH;
		boot_data->bt_service_state = START;
		boot_data->service_req_flag = START;
		boot_data->poweron_bt_flag = 1; //WIFI 2 BT：3
		boot_data->fpga_debug=0;
	}
//#endif
	//first boot wifi debug
	if(fpga_debug==FPGA_FIRST_BOOTWIFI_MODE)
	{
		boot_data->dl_module = SKW_WIFI;
		boot_data->wifi_service_state = START;
		boot_data->service_req_flag = START;
		boot_data->poweron_wifi_flag = 1; //WIFI 2 BT：3
		boot_data->fpga_debug=0;
		ret = skw_boot_loader(boot_data);
		if(ret !=0){
			skwboot_err("%s firt boot cp fail \n", __func__);
			return -1;
		}
	}
	if(!fpga_debug){
		skwboot_log("%s,line:%d comming download-----start!", __func__, __LINE__);
		ret = skw_boot_loader(boot_data);
		if(ret < 0){
			skwboot_err("%s firt boot cp fail \n", __func__);
			return -1;
		}
	}
	//download done set the download flag;
	boot_data->first_dl_flag =1;
	boot_data->poweron_wifi_flag = 0;
	boot_data->first_bootwifi_flag = 0;
	boot_data->first_bootbt_flag =0;
	//download done tall cp acount;
	boot_data->dl_done_signal &= 0xFF;
	boot_data->dl_done_signal +=1;
	skwboot_log("%s first boot pass\n", __func__);
#ifdef DEBUG_SKWBOOT_TIME
	last_time = ktime_get();
	skwboot_log("%s,the download time start time %llu and the over time %llu \n",
			__func__, cur_time, last_time);
#endif
	return ret;
}

/****************************************************************
 *Description:analysis the img file interface
 *Func:
 *Calls:
 *Call By:sdio_dloader
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-08-26
 * **************************************************************/
int skw_boot_bt(struct seekwave_device *boot_data,int service_state)
{
	int ret;
	unsigned int tmp_signal = 0;
	//skw_BT power on
	boot_data->dl_module = SKW_BLUETOOTH;
	//start bt service: 1 stop :0
	boot_data->bt_service_state = service_state;
	//sdio send irq cmd to cp bsp poweron subsys to poweron the mem
	boot_data->poweron_flag = SKW_BLUETOOTH; //WIFI 2 BT：3
	//download done flag ++
	dl_signal_acount ++;
	tmp_signal = dl_signal_acount;
	boot_data->dl_done_signal = 0xff&tmp_signal;
	boot_data->dl_acount_addr = SKW_SDIO_PD_DL_AP2CP_BSP;
	//gpio need set high or low power interrupt to cp wakeup
	boot_data->gpio_out = boot_data->chip_gpio;
	if(boot_data->gpio_val)
		boot_data->gpio_val =0;
	else
		boot_data->gpio_val =1;

	if(!boot_data->poweron_bt_flag){
		//pweron the BT set 1 have been poweron BT
		boot_data->poweron_bt_flag = 1; //WIFI 1 BT：2
		//Second Boot BT download BT BIN
	}
	ret = skw_bootimg_analysis(boot_data);
	if(ret !=0){
		skwboot_err("%s boot fail \n", __func__);
		return -1;
	}
	//first_boot bt flag:
	if(!boot_data->first_bootbt_flag)
		boot_data->first_bootbt_flag =1;

	skwboot_log("%s bt boot sucessfull\n", __func__);
	return 0;
}

/****************************************************************
 *Description:analysis the img file interface statrt service or
 *		sleep wakeup wifi.
 *Func:skw_boot_wifi
 *Calls:
 *Call By:sdio_dloader
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others: 1,first boot WIFI,(BSP runing or not)
 *Author：JUNWEI.JIANG
 *Date:2021-09-14
 * **************************************************************/
static int skw_boot_wifi(struct seekwave_device *boot_data)
{
	int ret =0;
	unsigned int tmp_signal = 0;
	//not the first boot CP But first boot WIFI
	//wether is the first boot WIFI : download the WIFI BSP
	boot_data->dl_module = SKW_WIFI;
	//sdio send irq cmd to cp bsp poweron subsys to poweron the mem
	boot_data->poweron_flag = SKW_WIFI; //WIFI 2 BT：3
	//download done flag ++
	dl_signal_acount ++;
	tmp_signal = dl_signal_acount;
	boot_data->dl_done_signal = 0xff&tmp_signal;
	boot_data->dl_acount_addr = SKW_SDIO_PD_DL_AP2CP_BSP;
	//gpio need set high or low power interrupt to cp wakeup
	boot_data->gpio_out = boot_data->chip_gpio;
	if(boot_data->gpio_val)
		boot_data->gpio_val =0;
	else
		boot_data->gpio_val =1;

	if(!boot_data->poweron_wifi_flag){
		//pweron the wifi set 1 have been poweron wifi
		boot_data->poweron_wifi_flag = 1; //WIFI 1 BT：2
		//Second Boot WIFI download WIFI BIN
	}
	ret = skw_bootimg_analysis(boot_data);
	if(ret !=0){
		skwboot_err("%s boot fail \n", __func__);
		return -1;
	}
	//first_boot wifi flag:
	if(!boot_data->first_bootwifi_flag)
		boot_data->first_bootwifi_flag =1;
	skwboot_log("%s wifi boot sucessfull\n", __func__);
	return 0;
}
module_init(seekwave_boot_init);
module_exit(seekwave_boot_exit);
MODULE_LICENSE("GPL");
