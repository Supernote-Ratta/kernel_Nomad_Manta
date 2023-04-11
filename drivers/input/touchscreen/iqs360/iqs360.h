/******************************************************************************
*                                                                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              Azoteq (Pty) Ltd                               *
*                          Republic of South Africa                           *
*                                                                             *
*                           Tel: +27(0)21 863 0033                            *
*                          E-mail: info@azoteq.com                            *
*                                                                             *
*=============================================================================*
* @file 	IQS360.h						      *
* @brief 	IQS330 Object                                                 *
* @author 	AJ van der Merwe - Azoteq PTY Ltd                             *
* @version 	V1.0.0                                                        *
* @date 	19/9/2014                                                     *
*******************************************************************************/

/** @file
 *
 * @defgroup 	IQS360 Object Description
 * @{
 * @ingroup 	IQS360 Code
 * @brief       The IQS360, make up IQS60 as an object with registers and bit
 *		defines
 *
 * @details 	This module defines the IQS360 and makes it into an object. In
 *              order to use this object, include it into the project and
 *              create a variable of type IQS360_t. This will contain all
 *              the member fields of the IQS360 object.
 *
 * @note 
 *
 */

#ifndef IQS360_H__
#define IQS360_H__

// Private includes
// Private defines

/*      IQS360 Address  */
#define	IQS360_ADDR                     0x64

/*      Device Info */
typedef union
{
    struct
    {
        u8 ProductNumber;
        u8 VersionNumber;
    };
    u8 DeviceInfo;
}DeviceInfo_t;

/*      System Flags    */
typedef union
{
    struct
    {
        u8 Zoom			:1;
        u8 Noise			:1;
        u8 ATI_Busy			:1;
        u8 LP_Active			:1;
        u8 Is_CH0			:1;
        u8 Charge_8M_4M      	:1;
        u8 Filter_Halted		:1;
        u8 Show_Reset		:1;
    };
    u8 SystemFlags;
}SystemFlags_t;

/*      Events (System Flags Byte 2)   */
typedef union
{
    struct
    {
        u8 ATI_Event			:1;
        u8 Prox_Event		:1;
        u8 Touch_Event		:1;
        u8 Track_Event		:1;
        u8 None			:1;
        u8 Snap_Event          	:1;
        u8 Block_Event		:1;
        u8 ATI_Error 		:1;
    };
    u8 Events;
}Events_t;

/*      X-Data	*/
typedef union
{
    struct
    {
        u8 X_Low                     ;
        u8 X_High                    ;
    };
    u16 	X;  // 10-bit number
}X_t;

/*      Y-Data	*/
typedef union
{
    struct
    {
        u8 Y_Low                     :8;
        u8 Y_High                    :2;
    };
    u16 	Y;  // 10 -bit number
}Y_t;

/*      Status  */
/*	Touch Channels	*/
typedef union
{
    struct
    {
        u8 Touch_Channels_0;
        u8 Touch_Channels_1;
    };
    u16 Touch_Channels;
}TouchChannels_t;

/*	Snap Channels	*/
typedef union
{
    struct
    {
        u8 Snap_Channels_0;
        u8 Snap_Channels_1;
    };
    u16 Snap_Channels;
}SnapChannels_t;

/*	Counts	*/
typedef union
{
    struct
    {
        u8 Counts_Low;
        u8 Counts_High;
    };
    u16 Counts;
}Counts_t;

/*	LTA	*/
typedef union
{
    struct
    {
        u8 LTA_Low;
        u8 LTA_High;
    };
    u16 LTA;
}LTA_t;

/*	Multipliers	*/
typedef union
{
    struct
    {
        u8 Sens_Multi		:4;
        u8 Comp_Multi		:2;
        u8 Base_Value		:2;
    };
    u8 Multi;
}Multi_t;

/*	Settings	*/
typedef union
{
    struct
    {
        u8 Proj_Bias			:2;
        u8 Cs_Size			:1;
        u8 Reseed			:1;
        u8 Redo_ATI			:1;
        u8 Snap_Enable		:1;
        u8 ATI_Partial		:1;
        u8 ATI_Off			:1;
    };
    u8 Settings0;
}Settings0_t;

typedef union
{
    struct
    {
        u8 ATI_Band			:1;
        u8 Force_Sleep               :1;
        u8 Prox_Proj			:1;
        u8 None			:1;
        u8 Halt_Charge		:1;
        u8 Turbo_Mode		:1;
        u8 Xfer              	:1;
        u8 ACK_Reset              	:1;
    };
    u8 Settings1;
}Settings1_t;

typedef union
{
    struct
    {
        u8 Halt			:2;
        u8 Event_Mode		:1;
        u8 Timeout                   :1;
        u8 ACF_Disable		:1;
        u8 Force_Halt		:1;
        u8 WDT_Off			:1;
        u8 Soft_Reset		:1;
    };
    u8 Settings2;
}Settings2_t;

typedef union
{
    struct
    {
        u8 Coord_Filter		:1;
        u8 Relative_Coord		:1;
        u8 RX_On_Multiple		:1;
        u8 Reject_Touch      	:1;
        u8 Beta      		:2;
        u8 Border_Correct		:1;
        u8 Block_7_5 		:1;
    };
    u8 Settings3;
}Settings3_t;

typedef union
{
    struct
    {
        u8 Pass			:3;
        u8 Up_Enable                 :1;
        u8 UP			:3;
        u8 None                      :1;
    };
    u8 Settings4;
}Settings4_t;

/*	PWM	*/
typedef union
{
    struct
    {
        u8 Compare                   :5;
        u8 Mode			:3;
    };
    u8 PWM;
}PWM_t;

/*      Buzzer  */
typedef union
{
    struct
    {
        u8 Burst			:1;
        u8 Perm			:1;
        u8 DC			:1;
        u8 Buz_None                  :4;
        u8 Buz_Enable		:1;
    };
    u8 Buzzer;
}Buzzer_t;

/*      IQS360 Object   */
typedef struct
{
    /*  Device Info */
    DeviceInfo_t DeviceInfo;

    /*  System Flags    */
    SystemFlags_t SystemFlags;

    /*  Events  */
    Events_t Events;

    /*  X Coordinates   */
    X_t X;

    /*  Y Coordinates   */
    Y_t Y;

    /*  Touch Channels (Events) */
    /**
     * @brief Read which channels had an event
     */
    TouchChannels_t Touch_Channels;

    /*  Snap Channels (Events) */
    /**
     * @brief Read which channels had an event
     */
    SnapChannels_t Snap_Channels;

    /*  Channel Counts  */
    Counts_t Counts_CH0;
    Counts_t Counts_CH1;
    Counts_t Counts_CH2;
    Counts_t Counts_CH3;
    Counts_t Counts_CH4;
    Counts_t Counts_CH5;
    Counts_t Counts_CH6;
    Counts_t Counts_CH7;
    Counts_t Counts_CH8;
    Counts_t Counts_CH9;
    Counts_t Counts_CH10;
    Counts_t Counts_CH11;
    Counts_t Counts_CH12;

    /*  LTA */
    LTA_t LTA_CH0;
    LTA_t LTA_CH1;
    LTA_t LTA_CH2;
    LTA_t LTA_CH3;
    LTA_t LTA_CH4;
    LTA_t LTA_CH5;
    LTA_t LTA_CH6;
    LTA_t LTA_CH7;
    LTA_t LTA_CH8;
    LTA_t LTA_CH9;
    LTA_t LTA_CH10;
    LTA_t LTA_CH11;
    LTA_t LTA_CH12;

    /*  Multipliers (BASE)  */
    Multi_t Multi_CH0;
    Multi_t Multi_CH1;
    Multi_t Multi_CH2;
    Multi_t Multi_CH3;
    Multi_t Multi_CH4;
    Multi_t Multi_CH5;
    Multi_t Multi_CH6;
    Multi_t Multi_CH7;
    Multi_t Multi_CH8;
    Multi_t Multi_CH9;
    Multi_t Multi_CH10;
    Multi_t Multi_CH11;
    Multi_t Multi_CH12;

    /*  Settings    */
    Settings0_t Settings0;
    Settings1_t Settings1;
    Settings2_t Settings2;
    Settings3_t Settings3;
    Settings4_t Settings4;
    u8 Settings5;

    /*	Thresholds	*/
    u8 Threshold_CH0;
    u8 Threshold_CH1;
    u8 Threshold_CH2;
    u8 Threshold_CH3;
    u8 Threshold_CH4;
    u8 Threshold_CH5;
    u8 Threshold_CH6;
    u8 Threshold_CH7;
    u8 Threshold_CH8;
    u8 Threshold_CH9;
    u8 Threshold_CH10;
    u8 Threshold_CH11;
    u8 Threshold_CH12;

    /*	Timings	*/
    u8 Filter_Halt;
    u8 Power_Mode;
    u8 Timeout_Period;

    /*	Ati Targets	*/
    u8 ATI_Target_CH0;
    u8 ATI_Target_CH1_12;
	u8 EventsMask;

    /*  PWM */
    PWM_t PWM0;
    PWM_t PWM1;
    PWM_t PWM2;
    PWM_t PWM3;
    PWM_t PWM4;
    PWM_t PWM5;
    PWM_t PWM6;
    PWM_t PWM7;

    /*  PWM Settings    */
    u8 PWM_Limit;
    u8 PWM_Speed;

    /*  Active Channels */
    TouchChannels_t Active_Channels;

    /*  Trackpad Active Channels */
    TouchChannels_t Trackpad_Active_Channels;

    /*	Snap Thresholds	*/
    u8 Snap_Threshold_CH0;
    u8 Snap_Threshold_CH1;
    u8 Snap_Threshold_CH2;
    u8 Snap_Threshold_CH3;
    u8 Snap_Threshold_CH4;
    u8 Snap_Threshold_CH5;
    u8 Snap_Threshold_CH6;
    u8 Snap_Threshold_CH7;
    u8 Snap_Threshold_CH8;
    u8 Snap_Threshold_CH9;
    u8 Snap_Threshold_CH10;
    u8 Snap_Threshold_CH11;
    u8 Snap_Threshold_CH12;

    /*  Correction Constant (Trackpad Edge Correction   */
    u8 Correction_Constant;

    /*  Buzzer  */
    Buzzer_t Buzzer;
	
} IQS360_t;

/*	Registers	*/
#define DEVICE_INFORMATION                  0x00    /*    R  -  2 bytes  */
#define SYSTEM_FLAGS                        0x01    /*    R  -  2 bytes  */
#define XY_DATA                             0x02    /*    R  -  4 bytes  */
#define STATUS                              0x03    /*    R  -  4 bytes  */
#define COUNTS                              0x04    /*    R  - 26 bytes  */
#define LTA                                 0x05    /*    R  - 26 bytes  */
#define MULTIPLIERS                         0x06    /*  R/W  - 26 bytes  */
#define COMPENSATION                        0x07    /*  R/W  - 26 bytes  */
#define SETTINGS                            0x08    /*  R/W  -  6 bytes  */
#define THRESHOLDS                          0x09    /*  R/W  - 26 bytes  */
#define TIMINGS                             0x0A    /*  R/W  -  3 bytes  */
#define TARGETS                             0x0B    /*  R/W  -  2 bytes  */
#define EVENTS                              0x0C    /*  R/W  -  8 bytes  */
#define PWM_LIM_SPEED                       0x0D    /*  R/W  -  2 bytes  */
#define ACTIVE_CHANNELS                     0x0E    /*  R/W  -  4 bytes  */
#define SNAP_THRESHOLDS                     0x0F    /*  R/W  - 12 bytes  */
#define CORRECTION_CONSTANT                 0x10    /*  R/W  -  1 bytes  */
#define BUZZER                              0x11    /*  R/W  -  2 bytes  */

/*	Bit Definitions	*/

/*	Channels	*/
#define CH0                                 0x0001
#define CH1                                 0x0002
#define CH2                                 0x0004
#define CH3                                 0x0008
#define CH4                                 0x0010
#define CH5                                 0x0020
#define CH6                                 0x0040
#define CH7                                 0x0080
#define CH8                                 0x0100	// Remeber High Byte
#define CH9                                 0x0200	// Remeber High Byte
#define CH10                                0x0400	// Remeber High Byte
#define CH11                                0x0800	// Remeber High Byte
#define CH12                                0x1000	// Remeber High Byte

/*	System Flags	*/
#define ZOOM                                0x01
#define NOISE                               0x02
#define ATI_BUSY                            0x04
#define LP_ACTIVE                           0x08
#define IS_CH0                              0x10
#define CHARGE_8M_4M                        0x20
#define FILTER_HALTED                       0x40
#define SHOW_RESET                          0x80

/*      Events  */
#define ATI_EVENT                           0x01
#define PROX_EVENT                          0x02
#define TOUCH_EVENT                         0x04
#define TRACK_EVENT                         0x08
#define SNAP_EVENT                          0x20
#define BLOCK_EVENT                         0x40
#define ATI_ERROR                           0x80

/*	Multipliers	*/
#define MUTLIPLIERS                         0x3F
#define BASE_VALUE                          0xC0

/*	Settings	*/
/*	Settings 0	*/
#define PROJ_BIAS                           0x03
#define CS_SIZE                             0x04
#define RESEED                              0x08
#define REDO_ATI                            0x10
#define SNAP_ENABLE                         0x20
#define ATI_PARTIAL                         0x40
#define ATI_OFF                             0x80

/*	Settings 1	*/
#define ATI_BAND                           0x01
#define FORCE_SLEEP                        0x02
#define PROX_PROJ                          0x04
#define HALT_CHARGE                        0x10
#define TURBO_MODE                         0x20
#define XFER                               0x40
#define ACK_RESET                          0x80

/*      Settings 2	*/
#define HALT                               0x03
#define EVENT_MODE                         0x04
#define TIMEOUT_DISABLE                    0x08
#define ACF_DISABLE                        0x10
#define FORCE_HALT                         0x20
#define WDT_OFF                            0x40
#define SOFT_RESET                         0x80

/*	Settings 3	*/
#define COORD_FILTER                       0x01
#define RELATIVE_COORD                     0x02
#define RX_ON_MULTIPLE                     0x04
#define REJECT_TOUCH                       0x08
#define BETA                               0x30
#define BORDER_CORRECT                     0x40
#define BLOCK_7_5                          0x80

/*	PWM	*/
#define COMPARE                            0x1F
#define MODE                               0xE0
#define PWM_LIMIT_BITS                     0x1F
#define PWM_SPEED_BITS                     0x0F

/*	Buzzer	*/
#define BURST                              0x01
#define PERM                               0x02
#define DC                                 0x04
#define PWM_ENABLE                         0x80

#endif /*	IQS360_H__	*/

/** @} */
