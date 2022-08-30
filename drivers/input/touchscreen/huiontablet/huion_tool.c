/* drivers/input/touchscreen/huion_tool.c
 * 
 * 2010 - 2012 Huion Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the Huion's TP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:2.2
 *        V1.0:2012/05/01,create file.
 *        V1.2:2012/06/08,modify some warning.
 *        V1.4:2012/08/28,modified to support 
 *        V1.6:new proc name
 *        V2.2: compatible with Linux 3.10, 2014/01/14
 */

#include <asm/memory.h>
#include <linux/init.h>
#include<linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fdtable.h>
#include <linux/irq.h>
#include "huiontablet.h"
#include "huiontablet_cfg.h"


#include "HUIONDFU_API_IF.h"
u16 gshow_len;
u16 gtotal_len;
 

#define DATA_LENGTH_UINT    512
#define CMD_HEAD_LENGTH     (sizeof(st_cmd_head) - sizeof(u8*))
static char procname[20] = {0};

#define UPDATE_FUNCTIONS

#ifdef UPDATE_FUNCTIONS
extern s32 hup_enter_update_mode(struct i2c_client *client);
extern void hup_leave_update_mode(void);
//extern s32 hup_update_proc(void *dir);
#endif

extern void huion_irq_disable(struct huion_ts_data *);
extern void huion_irq_enable(struct huion_ts_data *);

#pragma pack(1)
typedef struct{
    u8  wr;         //write read flag 0:R  1:W  2:PID 3:
    u8  flag;       //0:no need flag 1: need flag  2:need int
    u8 flag_addr[2];  //flag address
    u8  flag_val;   //flag val
    u8  flag_relation;  //flag_val:flag 0:not equal 1:equal 2:> 3:<
    u16 circle;     //polling cycle 
    u8  times;      //polling times
    u8  retry;      //I2C retry times
    u16 delay;      //delay befor read or after write
    u16 data_len;   //data length
    u8  addr_len;   //address length
    u8  addr[2];    //address
    u8  res[3];     //reserved
    u32 flashaddr;
    u8* data;       //data pointer

}st_cmd_head;
#pragma pack()
static st_cmd_head cmd_head;

static struct i2c_client *gt_client = NULL;

static struct proc_dir_entry *huion_proc_entry;

static ssize_t huion_tool_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t huion_tool_write(struct file *, const char __user *, size_t, loff_t *);
extern struct i2c_client * i2c_connect_client;   

static const struct file_operations tool_ops = {
    .owner = THIS_MODULE,
    .read = huion_tool_read,
    .write = huion_tool_write,
};

static int (*tool_i2c_read)(char *, int);
static int (*tool_i2c_write)(char *, int);

static s32 DATA_LENGTH = 0;

static void tool_set_proc_name(char * procname)
{
    sprintf(procname, "huion_fw_update");  //do not need the date;
}


extern int  gsend_data(char *txdata, int length);
extern int  greceive_data(char *rxdata, int length);

static void register_i2c_func(void)
{
    tool_i2c_read = greceive_data;
    tool_i2c_write = gsend_data;
}

static void unregister_i2c_func(void)
{
    tool_i2c_read = NULL;
    tool_i2c_write = NULL;
    GTP_INFO("I2C function: unregister i2c transfer function!");
}

s32 huion_init_wr_node(struct i2c_client *client)
{
    s32 i;

    gt_client = client;
    
    memset(&cmd_head, 0, sizeof(cmd_head));
    cmd_head.data = NULL;

    i = 5;
    while ((!cmd_head.data) && i)
    {
        cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);
        if (NULL != cmd_head.data)
        {
            break;
        }
        i--;
    }
    if (i)
    {
        DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
        GTP_INFO("Applied memory size:%d.", DATA_LENGTH);
    }
    else
    {
        GTP_ERROR("Apply for memory failed.");
        return FAIL;
    }

    cmd_head.addr_len = 2;
    cmd_head.retry = 5;

    register_i2c_func();

    tool_set_proc_name(procname);
    //huion_proc_entry = create_proc_entry(procname, 0666, NULL);
    huion_proc_entry = proc_create(procname, 0666, NULL, &tool_ops);
    if (huion_proc_entry == NULL)
    {
        GTP_ERROR("Couldn't create proc entry!");
        return FAIL;
    }
    else
    {
        GTP_INFO("Create proc entry success!");
    }

    while(1)
    {
        msleep(10);
    }
    return SUCCESS;
}

void huion_uninit_wr_node(void)
{
    if(cmd_head.data != NULL)
    kfree(cmd_head.data);
    cmd_head.data = NULL;
    unregister_i2c_func();
    remove_proc_entry(procname, NULL);
}

//   u8  wr;         //write read flag 0:R  1:W  2:PID 3:
//     u8  flag;       //0:no need flag 1: need flag  2:need int
//     u8 flag_addr[2];  //flag address
//     u8  flag_val;   //flag val
//     u8  flag_relation;  //flag_val:flag 0:not equal 1:equal 2:> 3:<
//     u16 circle;     //polling cycle 
//     u8  times;      //polling times
//     u8  retry;      //I2C retry times
//     u16 delay;      //delay befor read or after write
//     u16 data_len;   //data length
//     u8  addr_len;   //address length
//     u8  addr[2];    //address
//     u8  res[3];     //reserved
//     u8* data;   
static void dump_cmd_head(void)
{
    printk("wr:%d\n",cmd_head.wr);
    printk("dl:%d\n",cmd_head.data_len);
    printk("al:%d\n",cmd_head.addr_len);

    printk("addr[0]:%2x\n",cmd_head.addr[0]);
    printk("addr[1]:%2x\n",cmd_head.addr[1]);
    
    printk("flag:%d\n",cmd_head.flag);
    printk("flag_addr[0]:%2x\n",cmd_head.flag_addr[0]);
    printk("flag_addr[1]:%2x\n",cmd_head.flag_addr[1]);
    printk("cycle:%d\n",cmd_head.circle);
    printk("times:%d\n",cmd_head.times);
    //printk("data:%4x\n",cmd_head.data);
    printk("delay:%4x\n",cmd_head.delay);
    printk("retry:%4x\n",cmd_head.retry);
    printk("flashaddr:%08x\n",cmd_head.flashaddr);
    
}

static void dump_array(unsigned char*data, int data_len)
{
     int i=0;
     printk("%0x \n",data_len);
     for(i=0; i< data_len; i++)
     {
         printk("%02x ",*data);
         if(i % 8 == 0 && i !=0)
         {
             printk("\n");
         }
         data++;
     }
     printk("\n");
     
}  
       
/*******************************************************    
Function:
    Huion tool write function.
Input:
  standard proc write function param.
Output:
    Return write length.
********************************************************/
ssize_t huion_tool_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
    s32 ret = 0;
    //uint32_t writeaddr = 0;
    char *databuf = NULL;
    //GTP_DEBUG_FUNC();
    //GTP_DEBUG_ARRAY((u8*)buff, len);
    
    ret = copy_from_user(&cmd_head, buff, CMD_HEAD_LENGTH); //copy the command

    //printk("huion_tool_write len %d\n",len);
    
    if(ret)
    {
        GTP_ERROR("copy_from_user cmd_head failed.\n");
        return -EPERM;
    }

    dump_cmd_head();

    
    if (1 == cmd_head.wr) //write 
    {
        extern int gI2C_Command_Write_Proc(uint8_t *buf, int startaddr, int length );

        printk("write the to %08x len: %d \n",cmd_head.flashaddr,cmd_head.data_len);

        databuf = kmalloc(cmd_head.data_len,GFP_KERNEL);
        ret = copy_from_user(databuf, &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
        
        if(ret)
        {
            GTP_ERROR("copy_from_user data failed.\n");
            return -EPERM;
        }
        //dump_array(databuf, cmd_head.data_len);  
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Write_Proc(databuf,cmd_head.flashaddr,cmd_head.data_len );
        
        if(databuf)
            kfree(databuf);
        if(ret<0)
        {
            GTP_ERROR("[WRITE]Write data failed!");
            return ret;
        }
       
    }
    else if (2 == cmd_head.wr) //erase
    {
        extern int gI2C_Command_Erase_Page(int whichpage);

        printk("erase the page %02x\n",cmd_head.flashaddr);
         
        dump_array(databuf, cmd_head.data_len);  
      
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Erase_Page(cmd_head.flashaddr);
        if(ret< 0)
        {
            printk("Erase error\n");
            return ret;
        }
        else
        {
            printk("Erase ok\n");
            return ret;
        }

    }
    else if (3 == cmd_head.wr)//run application program
    {
        extern int gI2C_Command_Go_Proc(void);
        
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Go_Proc();
        if(databuf)
           kfree(databuf);
        if(ret< 0)
        {
            printk("run application error");
            return ret;
        }
        else
        {
            printk("run applicatiaon ok");
            return ret;
        }

    }
    else if (4 == cmd_head.wr)//disable irq!
    {
        huion_irq_disable(i2c_get_clientdata(gt_client));
        
    }
    else if (5 == cmd_head.wr) //enable irq!
    {
        huion_irq_enable(i2c_get_clientdata(gt_client));
    }  
    else if (6 == cmd_head.wr) //Update firmware! the cmd_head.data is the dir of the firmware file 
    {
        //gshow_len = 0;
        //gtotal_len = 0;
        //memset(cmd_head.data, 0, cmd_head.data_len + 1);
        //memcpy(cmd_head.data, &buff[CMD_HEAD_LENGTH], cmd_head.data_len);

        //if (FAIL == hup_update_proc((void*)cmd_head.data))
        //{
        //    return -EPERM;
        //}
    }
    else if (7 == cmd_head.wr) //reset the firmware board
    {
        s32 delayms;
        extern void ghuion_reset_fw(struct i2c_client *client, s32 ms);
        printk("Reset the board and sleep %d ms\n",cmd_head.delay);
        delayms = cmd_head.delay;
        ghuion_reset_fw( i2c_connect_client, 50);

        msleep(delayms);
        //return 1;
    }  
    else if (8 == cmd_head.wr) //read the sector board
    {
        
        extern int gI2C_Command_Read_Proc(uint8_t *buf, int startaddr, int length );

        printk("read data from %08x len:%d\n",cmd_head.flashaddr,cmd_head.data_len);
        databuf = kmalloc(cmd_head.data_len,GFP_KERNEL);
        
        if(ret)
        {
            GTP_ERROR("copy_from_user data failed.\n");
            return -EPERM;
        }
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Read_Proc(databuf,cmd_head.flashaddr,cmd_head.data_len );
        //dump_array(databuf, cmd_head.data_len);  
        
        ret = copy_to_user( (void *)&buff[CMD_HEAD_LENGTH], (const char*)databuf, cmd_head.data_len);

        if(databuf)
            kfree(databuf);
        if(ret<0)
        {
            GTP_ERROR("[WRITE]Read data failed!");
            return ret;
        }
        
    }
    else if (9 == cmd_head.wr) //get the dfu command list
    {
        int gI2C_Command_Get_PID(uint8_t *buf);
        int gI2C_Command_Get_Proc(uint8_t *buf);
        int gI2C_Command_Dis_Read_Pro_Proc(void);

        //extern uint8_t gI2C_Cmd_list[];

        printk("get the com list from %08x len:%d\n",cmd_head.flashaddr,cmd_head.data_len);
        #define  I2C_CMD_CNT 12 //(sizeof(gI2C_Cmd_list)/sizeof(gI2C_Cmd_list[0]))

        databuf = kmalloc(I2C_CMD_CNT+2,GFP_KERNEL);  //com list(12 bytes) + number(1bytes) + version(1bytes)
        
        if(ret)
        {
            GTP_ERROR("copy_from_user data failed.\n");
            return -EPERM;
        }
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Get_Proc(databuf );
        //dump_array(databuf, cmd_head.data_len);  
        
        ret = copy_to_user( (void *)&buff[CMD_HEAD_LENGTH], (const char*)databuf, I2C_CMD_CNT+2);

        if(databuf)
            kfree(databuf);
        if(ret<0)
        {
            GTP_ERROR("[WRITE]Read cmd list failed!");
            return ret;
        }
        
    }  
    else if (10 == cmd_head.wr) //get the PID
    {
        extern int gI2C_Command_Get_PID(uint8_t *buf);
        extern int gI2C_Command_Dis_Read_Pro_Proc(void);

        //extern uint8_t gI2C_Cmd_list[];

        printk("get the PID  from %08x len:%d\n",cmd_head.flashaddr,cmd_head.data_len);
        #define  PID_CNT 4  //4 BYTES

        databuf = kmalloc(PID_CNT,GFP_KERNEL);
        
        if(ret)
        {
            GTP_ERROR("copy_from_user data failed.\n");
            return -EPERM;
        }
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Get_PID(databuf );
        //dump_array(databuf, cmd_head.data_len);  
        
        ret = copy_to_user( (void*)&buff[CMD_HEAD_LENGTH], (const char*)databuf, PID_CNT);

        if(databuf)
            kfree(databuf);
        if(ret<0)
        {
            GTP_ERROR("[WRITE]Read PID failed!");
            return ret;
        }
        
    }
    else if (11 == cmd_head.wr) //disable the read protect, this command will erase all the chip
    {
       
        extern int gI2C_Command_Dis_Read_Pro_Proc(void);

        //extern uint8_t gI2C_Cmd_list[];

        printk("disable the read protect, this com will erase the whole chips\n");
        
        i2c_connect_client->addr = (unsigned short)(cmd_head.addr[0]>>1);

        ret = gI2C_Command_Dis_Read_Pro_Proc( );       
       
        if(ret<0)
        {
            GTP_ERROR("[WRITE]Read PID failed!");
            return ret;
        }
        
    }
    
    return len;
}

/*******************************************************    
Function:
    huion tool read function.
Input:
    standard proc read function param.
Output:
    Return read length.
********************************************************/
ssize_t huion_tool_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    s32 ret = 0;
    GTP_DEBUG_FUNC();    
    if (*ppos)      // ADB call again
    {
        return 0;
    }
    
    if (cmd_head.wr % 2) //only 0 or 1
    {
        return -EPERM;
    }
    else if (!cmd_head.wr) //this is read proc
    {
        u16 len = 0;
        s16 data_len = 0;
        u16 loc = 0;
        memcpy(cmd_head.data, cmd_head.addr, cmd_head.addr_len);
 
        if (cmd_head.delay)
        {
            msleep(cmd_head.delay);
        }
        
        data_len = cmd_head.data_len;

        while(data_len > 0)
        {
            if (data_len > DATA_LENGTH)
            {
                len = DATA_LENGTH;
            }
            else
            {
                len = data_len;
            }
            data_len -= len;

            if (tool_i2c_read(cmd_head.data, len) <= 0)
            {
                GTP_ERROR("[READ]Read data failed!");
                return -EPERM;
            }

            ret = simple_read_from_buffer(&page[loc], size, ppos, &cmd_head.data[GTP_ADDR_LENGTH], len);
            if (ret < 0)
            {
                return ret;
            }
            loc += len;

            GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH], len);
            GTP_DEBUG_ARRAY(page, len);
        }
        return cmd_head.data_len; 
    }
   
    else if (4 == cmd_head.wr)
    {
        u8 progress_buf[4];
        progress_buf[0] = gshow_len >> 8;
        progress_buf[1] = gshow_len & 0xff;
        progress_buf[2] = gtotal_len >> 8;
        progress_buf[3] = gtotal_len & 0xff;
        
        ret = simple_read_from_buffer(page, size, ppos, progress_buf, 4);
        return ret;
    }
    return -EPERM;
}
