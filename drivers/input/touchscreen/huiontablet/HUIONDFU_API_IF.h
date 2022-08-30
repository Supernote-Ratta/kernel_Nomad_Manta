#ifndef __HUION_DFU_API_IF
#define __HUION_DFU_API_IF

typedef char BOOL; 
typedef unsigned char BYTE;
typedef  unsigned short USHORT;
typedef unsigned short UINT;
typedef unsigned int DWORD;
typedef unsigned char* LPBYTE;


typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef void (*pfunction)(void);

#define TRUE 1
#define FALSE 0



//请求结构体
typedef struct _REQUEST_DATA
{
    BYTE    Direction;  //通信方向,主机到从机/从机到主机
    BYTE    Request;    //请求类型,标准请求码(bRequest的值)
    union Value
    {
        struct
        {
            BYTE low;
            BYTE high;            
        }m_BYTE;  
        USHORT m_value; 
    }union_value;       //控制传输的value值,根据DFU协议,读操作该值大于1(The Read memory operation is selected when wValue > 1)
    union Index
    {
        struct
        {
            BYTE low;
            BYTE high;
        }m_BYTE;  
        USHORT m_index;//控制传输的index值,根据DFU文档设置该值
    }union_index;
    union Length
    {
        struct
        {
            BYTE low;
            BYTE high;
        }m_BYTE;  
        USHORT m_length;    //控制传输长度，表示数据长度
    }union_length;
    UINT   pData;
}REQUEST_DATA,*PREQUEST_DATA;


// DFU States
//////////////////////////////////////////////////////////////////////

#define HN_STATE_IDLE							0x00
#define HN_STATE_DETACH						    0x01
#define HN_STATE_DFU_IDLE						0x02
#define HN_STATE_DFU_DOWNLOAD_SYNC				0x03
#define HN_STATE_DFU_DOWNLOAD_BUSY				0x04
#define HN_STATE_DFU_DOWNLOAD_IDLE				0x05
#define HN_STATE_DFU_MANIFEST_SYNC				0x06
#define HN_STATE_DFU_MANIFEST					0x07
#define HN_STATE_DFU_MANIFEST_WAIT_RESET		0x08
#define HN_STATE_DFU_UPLOAD_IDLE				0x09
#define HN_STATE_DFU_ERROR						0x0A

//Dfu Transfer
#define DFU_SEND                                0x21
#define DFU_RECEIVE                             0xA1

//Dfu Request Code
#define DFU_DETACH                              0x00
#define DFU_DNLOAD                              0x01
#define DFU_UPLOAD                              0x02
#define DFU_GETSTATUS                           0x03
#define DFU_CLRSTATUS                           0x04
#define DFU_GETSTATE                            0x05
#define DFU_ABORT                               0x06

//Dfu Command
#define DFU_ERASE                               0x41
#define DFU_SET_ADDRESS                         0x21
#define DFU_READ_UNPROTECT                      0x92

#define WAITTIME  50


    //打开指定符号链接路径的DFU设备
    int HUION_DeviceOpen(char* DevPath);

    //关闭DFU设备
    int HUION_DeviceClose(void);

    //向DFU设备发送命令/数据
    int HUION_Download(PREQUEST_DATA pRequestData);

    //从DFU设备获取命令/数据
    DWORD HUION_Upload(PREQUEST_DATA pRequestData);

    //获取DFU设备Status,6字节
    DWORD HUION_GetStatus(LPBYTE pStatus);

    //清除DFU设备Status
    DWORD HUION_ClearStatus(void);

    //获取DFU设备State,1字节
    DWORD HUION_GetState(LPBYTE pState);

    //向DFU设备发送终止命令
    DWORD HUION_Abort(void);

    //等待DFU设备到某一个state，重试5次
    BOOL HUION_WaitState(BYTE state);

    //在指定地址获取Option Bytes,16字节
    BOOL HUION_GetSIFData(DWORD Address, LPBYTE data);

    //等待DFU设备到某一个status，重试5次
    BOOL HUION_WaitStatus(BYTE state);

    //获取描述符
    DWORD HUION_GetDescriptor(PREQUEST_DATA pRequestData);


//


//bootloader version
#define Version                     0x21
#define START_FRAME                 0x7F  // one start bit, 0x7F data bits, even parity bit and one stop bit. 
#define ACK_FRAME                   0x79  // 应答开始帧
#define NACK_FRAME                  0x1F  // 非法应答开始帧
/*USART bootloader commands	*/
#define I2C_CMD_GET                 0x00  // 获取版本和所支持的命令  
#define I2C_CMD_GR                  0x01  // 获取版本和读保护的状态  
#define I2C_CMD_GET_ID              0x02  // 获得芯片ID  
#define I2C_CMD_READ                0x11  // 从 RAM & Flash 指定位置读取最多256个字节  
#define I2C_CMD_GO                  0x21  // 跳转到用户区代码（FLASH OR SRAM）  
#define I2C_CMD_WRITE               0x31  // 将 256个字节写到 RAM & Flash 指定的位置    
//#define I2C_CMD_ERASE	              0x43  // 擦除一页或多页  
#define I2C_CMD_Ex_EARSE            0x44  // 使用两字节地址模式擦除一页或多页（V3.0以后支持）  
#define I2C_CMD_WRP_EN              0x63  // 使能某些簇的写保护
#define I2C_CMD_WRP_DIS             0x73  // 禁用flash所有簇的写保护  
#define I2C_CMD_RDP_EN              0x82  // 使能读保护  
#define I2C_CMD_RDP_DIS             0x92  // 取消读保护
#define I2C_No_Stretch_CMD_WRITE            0x32  // 将 256个字节写到 RAM & Flash 指定的位置    
#define I2C_No_Stretch_CMD_Ex_EARSE         0x45  // 使用两字节地址模式擦除一页或多页（V3.0以后支持）  
#define I2C_No_Stretch_CMD_WRP_EN           0x64  // 使能某些簇的写保护
#define I2C_No_Stretch_CMD_WRP_DIS          0x74  // 禁用flash所有簇的写保护  
#define I2C_No_Stretch_CMD_RDP_EN           0x83  // 使能读保护  
#define I2C_No_Stretch_CMD_RDP_DIS          0x93  // 取消读保护

#define I2C_CMD_GET_PID                     0x06

#define I2C_SLAVE_BYTE_RECEIVED ((uint32_t)0x00020040)  /* BUSY and RXNE flags */

#define ram_start           0x20000800
#define RAM_END             0x20001fff      //8K
#define flash_start         0x08000000
#define FLASH_END           0x0800ffff    //max 63K
#define system_start        0x1fffec00
#define system_end          0x1ffff7ff    //3k
#define opt_start           0x1FFFF800
#define opt_end             0x1FFFF80F

void i2c_slave_transmit_interrupt(uint8_t *p_buffer, uint32_t length);

void i2c_slave_receive_interrupt(uint8_t *p_buffer, uint32_t length);

void I2C_Slave_Receive_Command(uint8_t* pBuffer, uint8_t NumByteToReceive);

void i2c_slave_transmit_poll(uint8_t *p_buffer, uint32_t length);

void i2c_slave_receive_poll(uint8_t *p_buffer, uint32_t length);

void i2c_slave_buffer(uint8_t device_id,uint8_t offset,uint8_t *p_buffer, uint32_t length);

void I2C_EventIRQ_Handler(void);

void I2C_ErrorIRQ_Handler(void);


#endif