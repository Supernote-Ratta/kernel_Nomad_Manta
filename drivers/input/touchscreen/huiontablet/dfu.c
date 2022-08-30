
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
#include "huiontablet_firmware.h"

//#define UPDATE_FILE_PATH_1          "/sdcard/_huion_update_.bin"
//#define UPDATE_FILE_PATH_1          "/sdcard/demo20211111.bin"
#define UPDATE_FILE_PATH_1          "/data/huiwang/_huion_firmware_.bin"
#define UPDATE_FILE_PATH_2          "/sdcard/_huion_firmware_.bin"
#define UPDATE_FILE_PATH_3          "/data/data/com.huion.huiondfu/files/_huion_firmware_.bin"
#define UPDATE_FILE_PATH_6          "/sdcard/_demofw_.bin"
#define UPDATE_FILE_PATH_5          "/sdcard/P30_APP_FW_DEBUG.bin"
#define UPDATE_FILE_PATH_7          "/sdcard/GM001_M21V_211015_517.bin.encrypt"
#define UPDATE_FILE_PATH_8          "/sdcard/gendata.bin"
//#define UPDATE_FILE_PATH_9          "/sdcard/OEM01_T225_220402_515.bin.encrypt"
#define UPDATE_FILE_PATH_9          "/sdcard/_huion_firmware_2022060e.bin"


int gHUION_DFU_ErasePage(int page);
int gHUION_DFU_WriteData(uint8_t *databuf, int desaddress, int length);
extern int ghuion_i2c_wxdata(struct i2c_client *client, char *txdata, int length);
extern int ghuion_i2c_rxdata(struct i2c_client *client,char *rxdata, int length, u8 cmd);
extern int ghuion_i2c_rxdata01(struct i2c_client *client,char *rxdata, int length, u8 cmd);
extern unsigned int gget_fw_status(struct i2c_client *client);


extern struct i2c_client * i2c_connect_client_hn;   
int dfu_main(void);
uint8_t gI2C_Cmd_list[] ={
                     I2C_CMD_GET,           // Get command                        
                     I2C_CMD_GR,            // Get version command                        
                     I2C_CMD_GET_ID,        // Get ID command                                    
                     I2C_CMD_READ,          // 从 RAM & Flash 指定位置读取256个字节            
                     I2C_CMD_GO,            // 跳转到用户区代码                                
                     I2C_CMD_WRITE,         // 将 256个字节写到 RAM & Flash 指定的位置                                        
                     I2C_CMD_Ex_EARSE,      // 使用两字节地址模式擦除一页或多页（V3.0以后支持）
                     I2C_CMD_WRP_EN,        // 使能某些簇的写保护                              
                     I2C_CMD_WRP_DIS,       // 禁用所有簇的写保护                              
                     I2C_CMD_RDP_EN,        // 使能读保护                                      
                     I2C_CMD_RDP_DIS,       // 取消读保护
                     I2C_CMD_GET_PID,
                    };
#define  I2C_CMD_CNT (sizeof(gI2C_Cmd_list)/sizeof(gI2C_Cmd_list[0]))


int gI2C_Command_Read_Proc(uint8_t *buf, int startaddr, int length );
int gI2C_Command_Write_Proc(uint8_t *buf, int startaddr, int length );

void gI2C_Command_GR_Proc(uint8_t *buf);
int gI2C_Command_Go_Proc(void);
int gI2C_Command_Get_PID(uint8_t *buf);
int gI2C_Command_Get_Proc(uint8_t *buf);
int gI2C_Command_Erase_Page(int page);
int gI2C_Command_Dis_Read_Pro_Proc(void);


uint8_t gRead_Write_Address_Validation(uint32_t address);
int  gsend_data(char *txdata, int length);
int  greceive_data(char *rxdata, int length);
uint8_t gData_XOR( uint8_t* data , uint16_t length );

uint8_t *gFileBuf =NULL;

#define FLASH_PAGE_SIZE 1024
//#define I2C_TR_SIZE 256 //256 // is 256 BYTES   //only 16 bytes for i2c 
#define I2C_TR_SIZE 128 // is 16 BYTES //only 16 bytes for i2c 

   

void ghuion_reset_guitar(struct i2c_client *client, s32 ms);
void ghuion_reset_fw(struct i2c_client *client, s32 ms);
   
int gbinchecksum(unsigned short *gFileBuf, int FileLength)
{
    int i=0;
    unsigned short checksum = 0;
    
    for(i=0; i< FileLength/2; i++)
    {
        checksum += gFileBuf[i];
    }
    if(checksum == 0xffff)
    {    
        return 1;
    }
    else
    { 
        return -1;
    }
}
char* GetBinVersion_PreTag(char* full_data, int full_data_len, char* substr)
{
    int sublen;
    int i;
    char* cur;
    int last_possible;

    if (full_data == NULL || full_data_len <= 0 || substr == NULL) {
        return NULL;
    }
    if (*substr == '\0') {
        return NULL;
    }
    sublen = strlen(substr);
    
    cur = full_data;
    last_possible = full_data_len - sublen + 1;
    for (i = 0; i < last_possible; i++) {
        if (*cur == *substr) {
            //assert(full_data_len - i >= sublen);
            if (memcmp(cur, substr, sublen) == 0) {
                //found
                return cur;
            }
        }
        cur++;
    }
    return NULL;
}



char* gGetBinVersion_PostTag(char* full_data, int full_data_len, char* substr)
{
    int sublen;
    int i;
    char* cur; 
    int last_possible;
    if (full_data == NULL || full_data_len <= 0 || substr == NULL) {
        return NULL;
    }
 
    if (*substr == '\0') {
        return NULL;
    }
    sublen = strlen(substr);
    
    cur = full_data;
    last_possible = full_data_len - sublen + 1;
    for (i = 0; i < last_possible; i++) {
        if (*cur == *substr) {
            //assert(full_data_len - i >= sublen);
            if (memcmp(cur, substr, sublen) == 0) {
                //found
                return cur-3;
            }
        }
        cur++;
    }
    return NULL;
}
            
void gI2c_Test(void)
{
    int ret;
    int wrongcn=0;
    int okcn=0;
    u_int8_t TxBuffer[8];
    u_int8_t RxBuffer[8];
    
    u_int8_t sdata=0;
    while(1)
    {
        TxBuffer[0] =  0xaa;//sdata;
        ret = gsend_data(TxBuffer,1); 
        //msleep(1);
         udelay(100);
        ret = greceive_data(RxBuffer,2);
        //msleep(1);
        udelay(100);
        sdata++;
        if(RxBuffer[0]!= 0x79)//TxBuffer[0])
        {
            wrongcn++;
            printk("wrongcn: %d %02x %02x %02x \r\n",wrongcn,TxBuffer[0],RxBuffer[0],RxBuffer[1]);
        }
        else
        {
            okcn++;
            printk("okcn: %d %02x \r\n",okcn, RxBuffer[0]);
        }   
        
    }
}
int ghuion_dfu(char *ver)
{
    char *version; 
    #define FIRMWARE_VERSION_LENGTH 20
    #define DFU_I2C_ADDRESS 0x70

    long FileLength;
    int BlockCount;
    int ret;
    DWORD i;
    DWORD j;
    
    DWORD DownloadAddress;
    LPBYTE DataPointer;
    unsigned char versiontag[]= { 'H','U','I','O','N','_','V','e','r',0x0};

    //return;
   
    gFileBuf = huion_fw;    
        
    FileLength = sizeof(huion_fw);
     
    version = GetBinVersion_PreTag(gFileBuf, FileLength, (char* )versiontag);
    if(version)
    {
        printk("new fw version is: ");
        for(i=0; i< FIRMWARE_VERSION_LENGTH; i++)
        {
            printk(KERN_CONT "%02x ",version[i]);
        }
        printk("old fw version is: ");
        for(i=0; i< FIRMWARE_VERSION_LENGTH; i++)
        {
            printk(KERN_CONT "%02x ",ver[i]);
        }
    }
    else
    {
        printk("There no version tag:%s, do not update!",versiontag);
    }
    if(!memcmp(version,ver,FIRMWARE_VERSION_LENGTH))
    {
        printk("Both the fw version are same,do not need update! ");    

        return 0; //goto update now
    }

    printk("The firmware version is diff, now updateing  %d \n",(int)FileLength);
  
  
    //printk("total is %d \n",(int)FileLength);
    //printk("file first 4 bytes are: %x %x %x %x\n",gFileBuf[0],gFileBuf[1],gFileBuf[2],gFileBuf[3]);
                             
   
    i2c_connect_client_hn->addr = (DFU_I2C_ADDRESS>>1);  //dfu i2c address    

	
    ghuion_reset_fw( i2c_connect_client_hn, 50);
    msleep(80);

	//ghuion_reset_fw( i2c_connect_client_hn, 50);
    //msleep(100);
	
	//printk("test the reset pin  %d \n",(int)FileLength);		
	/*while(1)
	{
		ghuion_reset_fw( i2c_connect_client_hn, 50);
		msleep(50);
	}*/

    /*----------------3.获取Option Bytes-----------------------------*/
    // BYTE OptionBytes[16];
    // if (!HUION_DFU_GetDeviceOptionBytes(OptionBytes))
    // {
    //     printk("获取Option Bytes失败!");
    //     goto error_handler;
    // }

    // if (OptionBytes[0] != 0xA5)
    // {
    //     printk("读保护状态需要先解除读保护!");
    //     goto error_handler;
    // }

    /*---------------4.擦除page-----------------------------------*/
    #define APP_ADDR  0x08004000
    #define FLASH_ADDR  0x08000000
              

    DownloadAddress = APP_ADDR;
    i = 0;    
    for (i=DownloadAddress; i<DownloadAddress+FileLength; )
    {
        int whichpage = 0;
    
        //Erase,huion例 
        if (i < 0x8080000)
        {   //BANK1页面大小为1K 
            
            whichpage = (i- FLASH_ADDR)/FLASH_PAGE_SIZE;
            printk("Erase the page total is :%x filelenis: %d, whichpage: %x\n",i,(int)FileLength,whichpage);
            ret = gHUION_DFU_ErasePage(whichpage);
            if (ret > 0)
            {
                i += FLASH_PAGE_SIZE; 
            } 
            else
            {
                 printk("擦除页面失败!");
                 goto error_handler;
            }                           
        }
        else
        {   //BANK1页面大小为1K
            whichpage = (i- FLASH_ADDR)/FLASH_PAGE_SIZE;
            if (gHUION_DFU_ErasePage(whichpage))
            {
                //更新进度
                i += FLASH_PAGE_SIZE; 
            } 
            else
            {
                 printk("擦除页面失败!");
                goto error_handler;
            } 
        }       
    }
    printk("erase is finished \n");
   

   /*--------------5.烧录----------------------------------*/
    //按照256bytes 对齐进行烧录
    BlockCount = FileLength / I2C_TR_SIZE;
    DataPointer = (LPBYTE)gFileBuf;
    DownloadAddress = APP_ADDR;
    
   
    for (j=0; j < BlockCount; j++) 
    {
       
        ret = gHUION_DFU_WriteData(DataPointer, DownloadAddress,I2C_TR_SIZE);
        printk("ret:%d Addr is :%x totalSectors: %d, whichsector: %d\n",ret,DownloadAddress,BlockCount,j);
        if(ret > 0)
        {
            DataPointer += I2C_TR_SIZE;
            DownloadAddress += I2C_TR_SIZE;
        }
        else
        {
            printk("烧录固件失败!");
            goto error_handler;
        }
    }
    if (FileLength % I2C_TR_SIZE != 0)
    {
        //发送剩余不足整页的数据
        if (!gHUION_DFU_WriteData(DataPointer, DownloadAddress,FileLength%I2C_TR_SIZE))
        {
            printk("烧录文件失败!");
            goto error_handler;
        }
    }

    printk("烧录成功!");    
    printk("run the application\n");
    
    gI2C_Command_Go_Proc();


error_handler:
    
    return 1;
}

int ghuion_dfu_file(char *ver)
{
    char *version; 
    #define FIRMWARE_VERSION_LENGTH 20
    #define DFU_I2C_ADDRESS 0x70

    char *DownloadFileName = UPDATE_FILE_PATH_9;
    
    struct file *fp=NULL; 
    long FileLength;
    int fileops;
    int BlockCount;
    int ret=0;
    mm_segment_t old_fs;
    DWORD i=0;
    DWORD j=0;
    
    DWORD DownloadAddress;
    LPBYTE DataPointer;
    unsigned char versiontag[]= { 'H','U','I','O','N','_','V','e','r',0x0};
  
    //return;

   
    //gI2c_Test();

    DownloadFileName = UPDATE_FILE_PATH_9;
    fp = filp_open(DownloadFileName,O_RDWR,0); //O_RDONLY;

    //fp = filp_open(DownloadFileName,O_RDONLY,0);
    
    if (IS_ERR(fp))
    {        
        printk("file %s open eror! \n",DownloadFileName);
        return 0;
    }
    else
    {
        printk("file %s open ok\n",DownloadFileName);
    }
    

    old_fs = get_fs();
    set_fs(KERNEL_DS);
 
    //获取文件长度
    FileLength = fp->f_op->llseek(fp, 0, SEEK_END);
    
    printk("bin file size is %ld\n",FileLength);
    
    fileops = fp->f_op->llseek(fp, 0, SEEK_SET);
      

    if ( fp )
    {                
        //
        //gFileBuf = (unsigned char *)kmalloc(FileLength * sizeof(BYTE),GFP_KERNEL);
        gFileBuf = (unsigned char *)kmalloc(FileLength * sizeof(BYTE),GFP_USER);

        
        if (gFileBuf != NULL)
        {
    
            j = fp->f_op->read(fp, (char*)gFileBuf, FileLength, &fp->f_pos);      
            filp_close(fp,NULL);
           
            printk("file first 4 bytes are: %x %x %x %x\n",gFileBuf[0],gFileBuf[1],gFileBuf[2],gFileBuf[3]);                     
           
        } 
           
    }

    set_fs(old_fs);
   
    version = GetBinVersion_PreTag(gFileBuf, FileLength, (char* )versiontag);
    if(version)
    {
        printk("new fw version is: ");
        for(i=0; i< FIRMWARE_VERSION_LENGTH; i++)
        {
            printk(KERN_CONT "%02x ",version[i]);
        }
        printk("old fw version is: ");
        for(i=0; i< FIRMWARE_VERSION_LENGTH; i++)
        {
            printk(KERN_CONT "%02x ",ver[i]);
        }
    }
    else
    {
        printk("There no version tag:%s, do not update!",versiontag);
    }
   
    if(!memcmp(version,ver,FIRMWARE_VERSION_LENGTH))
    {
        printk("Both the fw version are same,do not need update! ");      
        return 0;
    }

    printk("The firmware version is diff, now updateing  %d \n",(int)FileLength);
   
    i2c_connect_client_hn->addr = (DFU_I2C_ADDRESS>>1);  //dfu address

    ghuion_reset_fw( i2c_connect_client_hn, 50);
    msleep(50);

    //while(1);
    //msleep(20);
   
    /*----------------3.获取Option Bytes-----------------------------*/
    // BYTE OptionBytes[16];
    // if (!HUION_DFU_GetDeviceOptionBytes(OptionBytes))
    // {
    //     printk("获取Option Bytes失败!");
    //     goto error_handler;
    // }

    // if (OptionBytes[0] != 0xA5)
    // {
    //     printk("读保护状态需要先解除读保护!");
    //     goto error_handler;
    // }

    /*---------------4.擦除page-----------------------------------*/
    #define APP_ADDR  0x08004000
    #define FLASH_ADDR  0x08000000
           
   
    DownloadAddress = APP_ADDR;
    i = 0;    
    for (i=DownloadAddress; i<DownloadAddress+FileLength; )
    {
        int whichpage = 0;
    
        //Erase,huion例 
        if (i < 0x8080000)
        {   //BANK1页面大小为1K 
            //    
            whichpage = (i- FLASH_ADDR)/FLASH_PAGE_SIZE;
            printk("Erase the page total is :%x filelenis: %d, whichpage: %x\n",i,(int)FileLength,whichpage);
            ret = gHUION_DFU_ErasePage(whichpage);
            if (ret > 0)
            {
                //更新进度
                i += FLASH_PAGE_SIZE; 
            } 
            else
            {
                 printk("擦除页面失败!");
                 goto error_handler;
            }                           
        }
        else
        {//BANK1页面大小为1K
            whichpage = (i- FLASH_ADDR)/FLASH_PAGE_SIZE;

            if (gHUION_DFU_ErasePage(whichpage))
            {
                //更新进度
                i += FLASH_PAGE_SIZE; 
            } 
            else
            {
                 printk("擦除页面失败!");
                goto error_handler;
            } 
        }       
    }
    printk("erase is finished \n");
    //while(1)
    //msleep(20);

    //FileLength = 64;//32*1024;//64;//1024;


   /*--------------5.烧录----------------------------------*/
    //按照256bytes 对齐进行烧录
    BlockCount = FileLength / I2C_TR_SIZE;
    DataPointer = (LPBYTE)gFileBuf;
    DownloadAddress = APP_ADDR;
    
   
    for (j=0; j < BlockCount; j++) 
    {
       
        if(j==0)
        {
            printk("The first sector data is\n"); 
            printk("%02x %02x %02x %02x \n",*DataPointer,*(DataPointer+1),*(DataPointer+2),*(DataPointer+3));
            printk("%02x %02x %02x %02x \n",*(DataPointer+4),*(DataPointer+5),*(DataPointer+6),*(DataPointer+7));
            
        }
        ret = gHUION_DFU_WriteData(DataPointer, DownloadAddress,I2C_TR_SIZE);
        printk("ret:%d Addr is :%x totalSectors: %d, whichsector: %d\n",ret,DownloadAddress,BlockCount,j);
   
        if(ret > 0)
        {
            DataPointer += I2C_TR_SIZE;
            DownloadAddress += I2C_TR_SIZE;
        }
        else
        {
            printk("烧录文件失败!");
            goto error_handler;
        }
    }
    //while(1)
    //msleep(20);
    if (FileLength % I2C_TR_SIZE != 0)
    {
        //发送剩余不足整页的数据
        if (!gHUION_DFU_WriteData(DataPointer, DownloadAddress,FileLength%I2C_TR_SIZE))
        {
            printk("烧录文件失败!");
            goto error_handler;
        }
    }

    printk("烧录成功!");    
    printk("run the application\n");
    
    gI2C_Command_Go_Proc();


error_handler:
    if (gFileBuf)
    {
        kfree(gFileBuf);
    }
    return 1;
}

    
s32 huion_init_wr_node(struct i2c_client *client);

int DFU_proc(char *ver)
{
    //huion_init_wr_node(i2c_connect_client_hn);
    int ret;
    ret = ghuion_dfu(ver);
    return ret;
    //dfu_main();
 
}

int gHUION_DFU_ErasePage(int page)
{
    int ret = 0;
    ret = gI2C_Command_Erase_Page(page);
    return ret;
    
}
int gHUION_DFU_WriteData(uint8_t *databuf, int desaddress,int length)
{
    int ret = 0;
    ret = gI2C_Command_Write_Proc(databuf,desaddress,length);
    return ret;
}



int gI2C_Master_Send_Command(int cmd)
{
    int i=0;
    uint8_t cmdbuf[2];
    int ret = 0;
   
    for(i=0; i< I2C_CMD_CNT; i++)
    {
        if(gI2C_Cmd_list[i]== cmd)
            break;
    }   
    if(i== I2C_CMD_CNT)
        ret = -2;

    cmdbuf[0] =  cmd;
    cmdbuf[1] =  0xff - cmd;    
    ret = gsend_data(cmdbuf,2);
   
    return ret;    
}



int  gsend_data(char *txdata, int length)
{
   
    return ghuion_i2c_wxdata(i2c_connect_client_hn, txdata, length);
}
int  greceive_data(char *rxdata, int length)
{
    
    return ghuion_i2c_rxdata01(i2c_connect_client_hn, rxdata, length, 0);
}
 
int gI2C_Send_Address(uint32_t addr)
{
    //addr = 0x12345678;
    int ret = 0;
    uint8_t data[5];
    data[0] = addr>>24;
    data[1] = addr>>16;
    data[2] = addr>>8;
    data[3] = addr>>0;    
    data[4] = gData_XOR(data,4);

    //printk("gI2C_Send_Address addr:%0x\n",addr);
    //printk("gI2C_Send_Address:%02x %02x %02x %02x %02x\n",data[0],data[1],data[2],data[3],data[4]);
    ret = gsend_data(data,5);
    return ret;  
}


/****************************************************************************
* 名    称：
* 功    能：计算输入数据补码
* 入口参数：
*           data    数据指针
*           length	数据个数
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
uint8_t gData_XOR( uint8_t* data , uint16_t length )
{
   int i;
   uint8_t temp = data[ 0 ] ;

   for( i = 1 ; i < length ; i++ )
   {
       temp  ^= data[ i ]  ;
   }   
    
   return temp ;
}

int gI2C_Command_Dis_Read_Pro_Proc(void)
{
    uint8_t RxBuffer[1];
    int ret = 0;

    gI2C_Master_Send_Command(I2C_CMD_RDP_DIS);
   
    ret = greceive_data(RxBuffer,1);
    if(ret <0)
    {
        ret = -1;
        return ret;
    }

    if(RxBuffer[0] == ACK_FRAME)
    {
        //
        ret = greceive_data(RxBuffer,1);
        if(ret <0)
        {
            ret = -2;
            return ret;
        }
    
        if(RxBuffer[0] == ACK_FRAME)
        {
            printk("program is ready\n");
        }
        else if(RxBuffer[0] == NACK_FRAME)
        {
            printk("program is not ready\n");
            ret = -3;
        }
    }
    else
    {
        ret = -4;
        printk("gI2C_Command_Dis_Read_Pro_Proc error\n ");
    }
    return ret;

}

int gI2C_Command_Go_Proc(void)
{
    int ret = 0;
    uint8_t RxBuffer[2];
    gI2C_Master_Send_Command(I2C_CMD_GO);
   
    ret = greceive_data(RxBuffer,2);
    if(ret < 0)
        return ret;
    printk("gI2C_Command_Go_Proc cmd ret %02x %02x",RxBuffer[0], RxBuffer[1]);

    //if(RxBuffer[1] == ACK_FRAME)
    {
        printk("gI2C_Command_Go_Proc is finshed\n");
    }
    /*else
    {
        ret = -1;
        printk("gI2C_Command_Go_Proc error\n ");
    }*/
    return ret;

}

int gI2C_Command_Get_Proc(uint8_t *buf)
{
    int ret = 0;
    int Num;
    uint8_t RxBuffer[1] = {};
        
    ret = gI2C_Master_Send_Command(I2C_CMD_GET);
    if(ret < 0)
    {
        ret = -1;
        printk("gI2C_Master_Send_Command ret == %d\n",ret);
        return ret; 
    }
        
    Num = sizeof(gI2C_Cmd_list)-1;//获取数组大小, do not include this command itself
    
    
    ret = greceive_data(RxBuffer,1);
    if(ret < 0)
    {
        ret = -2;
        printk("gI2C_Master_Send_Command ret == %d\n",ret);
        return ret; 
    }
    printk("00000 RxBuffer[0] == %x\n",RxBuffer[0]);
    
    if(RxBuffer[0] != ACK_FRAME)
    {
        ret = -3;
    }
    ret = greceive_data(buf,Num+2);  //the first two bytes is command list number and version 
    if(ret < 0)
    {
        ret = -4;
        return ret; 
    }
    ret = greceive_data(RxBuffer,1);
    printk("11111 RxBuffer[0] == %x\n",RxBuffer[0]);
   
    if(ret < 0)
    {
        ret = -5;
        return ret; 
    }
    if(RxBuffer[0] != ACK_FRAME)
    {
        ret = -6;
        return ret;
    }   
    ret = 1;
    return ret;
}

void gI2C_Command_GR_Proc(uint8_t *buf)
{
    uint8_t RxBuffer[1] = {};
    int ret = 0;

    gI2C_Master_Send_Command(I2C_CMD_GR);
   
    ret = greceive_data(RxBuffer,1);

    printk("gI2C_Command_GR_Proc ACK ret:%d, RxBuffer[0]:%x \n",ret,RxBuffer[0]);
    
    if(RxBuffer[0] == ACK_FRAME)
    {
        printk("gI2C_Command_GR_Proc ACK ok\n");
    }
    ret = greceive_data(buf,1);

    printk("gI2C_Command_GR_Proc Version ret:%d, version: %x \n",ret,buf[0]);
  
   
    ret = greceive_data(RxBuffer,1);

    printk("gI2C_Command_GR_Proc next ack ret:%d, RxBuffer[0]:%x \n",ret,RxBuffer[0]);

    if(RxBuffer[0] == ACK_FRAME)
    {
        printk("gI2C_Command_GR_Proc ack ok\n");
    }

}

int gI2C_Command_Get_PID(uint8_t *buf)
{
    int ret =0;
    uint8_t RxBuffer[1];

    gI2C_Master_Send_Command(I2C_CMD_GET_PID);
   
    ret = greceive_data(RxBuffer,1);
    if( ret <0 )
        return ret;

    if(RxBuffer[0] == ACK_FRAME)
    {
        printk("ack ok\n");
    }
    
    ret = greceive_data(buf,4);
    return ret;

}

int gI2C_Command_Erase_Page(int whichpage)
{
    int ret = 0;
    int fmstatus =0;
    //int ack_cnt = 0;
    uint8_t RxBuffer[4];
    uint8_t TxBuffer[4];
   
    uint16_t numofpages;
    
    numofpages = whichpage; 
    
    printk("erase the page %0x\n",whichpage);

    ret = gI2C_Master_Send_Command(I2C_CMD_Ex_EARSE);
    if(ret< 0)
    {
        return ret;
    }
   
    ret = greceive_data(RxBuffer,2);
    
    printk("Erase command ret %d ack is %0x\n",ret,RxBuffer[1]); // the FLASH program is busying
    
    if((numofpages&0xfff0) == 0xfff0)//f
    {
        TxBuffer[0] =  numofpages&0xff;
        TxBuffer[1] =  (numofpages>>8);
        TxBuffer[2] =  ~numofpages; //
        ret = gsend_data(TxBuffer,3); //send the numofpages

        //printk("Number of page send ret %0x\n",ret); // 
        greceive_data(RxBuffer,1);
        //printk("0xFFFF Erase command ack is %0x\n",RxBuffer[0]); //         
    }
    else
    {
        TxBuffer[0] =  0;
        TxBuffer[1] =  0;
        TxBuffer[2] =  0; //
        ret = gsend_data(TxBuffer,3); //send the 3 bytes zero        
        //printk("Number of page first 3 bytes send ret %0x\n",ret); // 
        greceive_data(RxBuffer,2);
        printk("other pages Erase command ack is %0x\n",RxBuffer[1]); //            
        
        //TxBuffer[0] =  (numofpages>>8)& 0xff;
        //TxBuffer[1] =  (numofpages>>0)& 0xff;
        TxBuffer[0] =  0; //only one page will be erased; 
        TxBuffer[1] =  numofpages&0xff;   //which page will be erased
        TxBuffer[2] =  gData_XOR(TxBuffer,2);
        ret = gsend_data(TxBuffer,3); //send the numofpages
        //printk("send 3 bytes %02x %02x %02x\n",TxBuffer[0],TxBuffer[1],TxBuffer[2]); //
        printk("Number of page next 3 bytes send ret %0x\n",ret); // 
              
    }
    fmstatus = gget_fw_status(i2c_connect_client_hn);
    printk("pre erase fmstatus %0x\n",fmstatus); // 
    
    while(1) //擦除需要时间14ms，等待20毫秒
    {
        //msleep(1);
        udelay(200);
        fmstatus = gget_fw_status(i2c_connect_client_hn);
        if(fmstatus)
            break;
        
    }
    
    ret = greceive_data(RxBuffer,2); //
   
    fmstatus = gget_fw_status(i2c_connect_client_hn);
    printk("after erase fmstatus %0x\n",fmstatus); // 
    
    printk("Erase command final ret %d ack %02x\n",ret, RxBuffer[0]); // the FLASH program is busying
    
    //return 1;
    if(RxBuffer[1] !=ACK_FRAME )
    {
        ret = -1;
        printk("Erase cmd failed\n");
    }
    return ret;
   
}


int gI2C_Command_Write_Proc(uint8_t *buf, int startaddr, int length )
{
    uint8_t RxBuffer[2];
    int ret = 0;
    int write_time;
    uint8_t *tmpbuf = NULL;
    
    printk("gI2C_Command_Write_Proc startaddr: %08x %d\n",startaddr,length);
    
    
    //printk("send the command I2C_CMD_WRITE\n");
    ret = gI2C_Master_Send_Command(I2C_CMD_WRITE);  
    //printk("I2C_CMD_WRITE cmd ret %d \n",ret);
    if(ret < 0)
    {
        printk("the FLASH is protected\n");
        //ret = ret;
        return ret;
    }
    
   
    ret = greceive_data(RxBuffer,2);
    
    printk("I2C_CMD_WRITE ret %d  %02x %02x \n",ret,RxBuffer[0],RxBuffer[1]);
    
    if(RxBuffer[1] == NACK_FRAME)
    {
        //printk("the FLASH is protected\n");
        ret = -1;
        return ret;
    }
    
    if(RxBuffer[1] != ACK_FRAME)
    {
        //printk("the FLASH is protected\n");
        ret = -2;
        return ret;
    }
    

    printk("Write Start addr: %0x length:%d\n",startaddr,length);
    //printk("send the write address \n");
    ret = gI2C_Send_Address(startaddr); //send the start address and checksum
    printk("gI2C_Send_Address ret %d\n",ret);
    if(ret < 0)
    {
        printk("Address is not ok \n");
        ret = -4;
        return ret;
    }
    
    ret = greceive_data(RxBuffer,1); //
    printk("Received Address ret %d ACK  %x \n",ret, RxBuffer[0]);
   
    if(RxBuffer[0] != ACK_FRAME )//|| ret < 0)
    {
        printk("Address is not ok \n");
        ret = -2;
        return ret;
    }
    
    tmpbuf = kmalloc(length+2,GFP_KERNEL);
    tmpbuf[0] = length-1;                      //write (number -1)
    memcpy(tmpbuf+1,buf,length);   
    tmpbuf[length+1] = gData_XOR(tmpbuf,length+1); //checksum

    /*for(write_time=0; write_time<length+2; write_time++)
    {
        printk("%02x ",tmpbuf[write_time]);
        if(write_time !=0  && ((write_time % 0x8) ==0) )
        printk("\n");
    }
    */
    ret = gsend_data(tmpbuf,length+2);    //send the write number + content + checksum

    kfree(tmpbuf);

    if(ret <0)
    {
        printk("Send write number + content+ checksum error\n");        
        return ret; 
    }
        
    //printk("Send %0d bytes\n",length); 
    write_time = 0;
    ret = gget_fw_status(i2c_connect_client_hn);
    printk("before write gget_fw_status %0d \n",ret); 
    
    
    while(1) //等低
    {
        udelay(2);
        ret = gget_fw_status(i2c_connect_client_hn);
        if(!ret)
            break;
       
    }
  
    while(1) //等高
    {
        //msleep(1);
        udelay(5);
        ret = gget_fw_status(i2c_connect_client_hn);
        if(ret)
            break;
       
    }
    ret = gget_fw_status(i2c_connect_client_hn);
    printk("after write gget_fw_status %0d \n",ret); 
    ret = greceive_data(RxBuffer,1); //
    
    
    printk("Write finished ret %d ACK  %x \n",ret,RxBuffer[0]);
    
    if(RxBuffer[0] != ACK_FRAME)
    {
        ret = -3;
    }
   
    return ret;
   
    
}

int gI2C_Command_Read_Proc(uint8_t *buf, int startaddr, int length )
{
    int ret = -1;
    uint8_t RxBuffer[1];
    uint8_t datasize[2]; /* +1 */
  
    gI2C_Master_Send_Command(I2C_CMD_READ);   
    
    greceive_data(RxBuffer,1);

    if(RxBuffer[0] == NACK_FRAME)
    {
        printk("FLASH is protected \n");
        ret = -1;
        return ret;
    }

    gI2C_Send_Address(startaddr);

    greceive_data(RxBuffer,1);
    if(RxBuffer[0] != ACK_FRAME)
    {
        ret = -2;
        return ret;
    }
    //printk("The receive address's ACK  %x\n",RxBuffer[0]);

    datasize[0] = length-1; //need 
    datasize[1] = ~datasize[0] & 0xff;
    //printk("00send the data size %2x size compliment %2x\n",datasize[0],datasize[1]);

    ret = gsend_data(datasize,1); //send the data size
    if(ret < 0)
    {
        ret = -3;
        return ret;
    }
   
    ret = gsend_data(&datasize[1],1); //send the data size's complement 
    if(ret < 0)
    {
        ret = -4;
        return ret;
    }
    
    greceive_data(RxBuffer,1);

    //printk("data size and comlement of size's ACK  %x\n",RxBuffer[0]);

    if(RxBuffer[0] == NACK_FRAME)
    {
        printk("datasize is error  \n");
        ret = -5;
        return ret;
    }
    ret = greceive_data(buf,length);

    //printk("receive the data ret:%d len:%x \n",ret,length);
    return ret;
}

uint8_t gRead_Write_Address_Validation(uint32_t address)
{
    uint8_t address_status = 0x0;
    
    if((address>=ram_start)&&(address<RAM_END))
    {
         address_status = 0x1;
    }
    else if((address>=flash_start)&&(address<FLASH_END))
    {
         address_status = 0x2;
    }
    else if((address>=opt_start)&&(address<=opt_end))
    {
         address_status = 0x4;
    }
    return address_status;
}


