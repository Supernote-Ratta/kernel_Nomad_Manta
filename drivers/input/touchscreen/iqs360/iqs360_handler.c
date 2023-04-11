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
* @file 	IQS360_handler  					      *
* @brief 	IQS360 handler - handles the IQS360 setup and read            *
* @author 	AJ van der Merwe - Azoteq PTY Ltd                             *
* @version 	V1.0.0                                                        *
* @date 	22/9/2014                                                     *
*******************************************************************************/

// Private includes
//#include "Includes.h"
#include "IQS360.h"
#include "IQS360_handler.h"
#include "IQS360_Init.h"


// Private defines
#define PWM_ON      0x6F
#define PWM_OFF     0x60

/*  Create an IQS360 Object */
IQS360_t IQS360 = {0};
// Keep the previous values for X and Y
X_t X_prev = {0};
Y_t Y_prev = {0};
/**
 * @brief   Handles the IQS360 Setup of the Object, as well as physical
 * @param   None
 * @retval  None
 */
void IQS360_setup(void)
{
    //LIGHTS_ON;

    /*  First setup the IQS360 Object   */

    /*  Multipliers (BASE)  */
    IQS360.Multi_CH0.Multi = MULTIPLIERS_CH0;
    IQS360.Multi_CH1.Multi = MULTIPLIERS_CH1;
    IQS360.Multi_CH2.Multi = MULTIPLIERS_CH2;
    IQS360.Multi_CH3.Multi = MULTIPLIERS_CH3;
    IQS360.Multi_CH4.Multi = MULTIPLIERS_CH4;
    IQS360.Multi_CH5.Multi = MULTIPLIERS_CH5;
    IQS360.Multi_CH6.Multi = MULTIPLIERS_CH6;
    IQS360.Multi_CH7.Multi = MULTIPLIERS_CH7;
    IQS360.Multi_CH8.Multi = MULTIPLIERS_CH8;
    IQS360.Multi_CH9.Multi = MULTIPLIERS_CH9;
    IQS360.Multi_CH10.Multi = MULTIPLIERS_CH10;
    IQS360.Multi_CH11.Multi = MULTIPLIERS_CH11;
    IQS360.Multi_CH12.Multi = MULTIPLIERS_CH12;

    /*  Please Note - No need to write Compensation */

    /*  Settings    */
    IQS360.Settings0.Settings0 = PROXSETTINGS0_VAL|REDO_ATI|ATI_OFF;
    IQS360.Settings1.Settings1 = PROXSETTINGS1_VAL|ACK_RESET|TURBO_MODE;
    IQS360.Settings2.Settings2 = PROXSETTINGS2_VAL|EVENT_MODE;
    IQS360.Settings3.Settings3 = PROXSETTINGS3_VAL;
    IQS360.Settings4.Settings4 = PROXSETTINGS4_VAL;
    IQS360.Settings5 = PROXSETTINGS5_VAL;

    /*  Thresholds  */
    IQS360.Threshold_CH0 = PROX_THRESHOLD;
    IQS360.Threshold_CH1 = TOUCH_THRESHOLD_CH1;
    IQS360.Threshold_CH2 = TOUCH_THRESHOLD_CH2;
    IQS360.Threshold_CH3 = TOUCH_THRESHOLD_CH3;
    IQS360.Threshold_CH4 = TOUCH_THRESHOLD_CH4;
    IQS360.Threshold_CH5 = TOUCH_THRESHOLD_CH5;
    IQS360.Threshold_CH6 = TOUCH_THRESHOLD_CH6;
    IQS360.Threshold_CH7 = TOUCH_THRESHOLD_CH7;
    IQS360.Threshold_CH8 = TOUCH_THRESHOLD_CH8;
    IQS360.Threshold_CH9 = TOUCH_THRESHOLD_CH9;
    IQS360.Threshold_CH10 = TOUCH_THRESHOLD_CH10;
    IQS360.Threshold_CH11 = TOUCH_THRESHOLD_CH11;
    IQS360.Threshold_CH12 = TOUCH_THRESHOLD_CH12;

    /*  Timings */
    IQS360.Filter_Halt = FILTER_HALT;
    IQS360.Power_Mode = POWER_MODE;
    IQS360.Timeout_Period = TIMEOUT_PERIOD;

    /*  ATI Target  */
    IQS360.ATI_Target_CH0 = ATI_TARGET_CH0;
    IQS360.ATI_Target_CH1_12 = ATI_TARGET_CH0_9;

    /*  PWM */
    IQS360.PWM0.PWM = PWM_0;
    IQS360.PWM1.PWM = PWM_1;
    IQS360.PWM2.PWM = PWM_2;
    IQS360.PWM3.PWM = PWM_3;
    IQS360.PWM4.PWM = PWM_4;
    IQS360.PWM5.PWM = PWM_5;
    IQS360.PWM6.PWM = PWM_6;
    IQS360.PWM7.PWM = PWM_7;

    /*  PWM Limits and Speed    */
    IQS360.PWM_Limit = PWM_LIMITS;
    IQS360.PWM_Speed = PWM_SPEED;

    /*  Active Channels */
    IQS360.Active_Channels.Touch_Channels_0 = ACTIVE_CH0;
    IQS360.Active_Channels.Touch_Channels_1 = ACTIVE_CH1;

    /*  Trackpad Active Channels    */
    IQS360.Trackpad_Active_Channels.Touch_Channels_0 = TRACKPAD_CH0;
    IQS360.Trackpad_Active_Channels.Touch_Channels_1 = TRACKPAD_CH1;

    /*  Buzzer  */
    IQS360.Buzzer.Buzzer = BUZZER;

    /*  Object Setup Done   */

    /*  Now do the IQS360 Physical Setup    */
    CommsIQS_start();   // Start I2C Transfer
    CommsIQS_Read(IQS360_ADDR, DEVICE_INFORMATION, &IQS360.DeviceInfo.DeviceInfo, 2);    // check for correct version
    // do something if incorrect version was detected
    CommsIQS_repeat_start();

    // start writing data to IQS36

    /*  Write Multipliers */
    CommsIQS_Write(IQS360_ADDR, MULTIPLIERS, &IQS360.Multi_CH0.Multi, 13);
    CommsIQS_repeat_start();

    /*  Write Settings    */
    CommsIQS_Write(IQS360_ADDR, SETTINGS, &IQS360.Settings0.Settings0, 6);
    CommsIQS_repeat_start();

    /*  Write Thresholds    */
    CommsIQS_Write(IQS360_ADDR, THRESHOLDS, &IQS360.Threshold_CH0, 13);
    CommsIQS_repeat_start();

    /*  Write Timings    */
    CommsIQS_Write(IQS360_ADDR, TIMINGS, &IQS360.Filter_Halt, 3);
    CommsIQS_repeat_start();

    /*  Write ATI Targets    */
    CommsIQS_Write(IQS360_ADDR, TARGETS, &IQS360.ATI_Target_CH0, 2);
    CommsIQS_repeat_start();

    /*  Write PWM    */
    CommsIQS_Write(IQS360_ADDR, PWM_REG, &IQS360.PWM0.PWM, 8);
    CommsIQS_repeat_start();

    /*  Write PWM Limits and Speed    */
    CommsIQS_Write(IQS360_ADDR, PWM_LIM_SPEED, &IQS360.PWM_Limit, 2);
    CommsIQS_repeat_start();

    /*  Write Active Channels and Trackpad Channels   */
    CommsIQS_Write(IQS360_ADDR, ACTIVE_CHANNELS, &IQS360.Active_Channels.Touch_Channels_0, 4);
    CommsIQS_stop();    // End Setup

    /*  End Of Setup   */

    /*  Wait For ATI Done   */
    do 
    {
        CommsIQS_start();
        CommsIQS_Read(IQS360_ADDR, SYSTEM_FLAGS, &IQS360.SystemFlags.SystemFlags, 2);
        CommsIQS_stop();
    }
    while(IQS360.SystemFlags.ATI_Busy);

    LIGHTS_OFF;

    /*
     * An ATI error Occurred - Try and setup chip again
     */
    if (IQS360.Events.ATI_Error)
    {
        LIGHTS_ON;
        IQS360_setup();
    }
}

/**
 * @brief   Reads from the IQS360 and refreshes the Object with lates data -
 *          this data is stored in the IQS360 object
 * @param   None
 * @retval  None
 */
void IQS360_refresh_data(void)
{
    /*  Read the latest data from the IQS360    */

    /*  First Read System Flags and the Events that occured */
    CommsIQS_start();
    CommsIQS_Read(IQS360_ADDR, SYSTEM_FLAGS, &IQS360.SystemFlags.SystemFlags, 2);

    /*  Read XY-Data */
    CommsIQS_repeat_start();
    CommsIQS_Read(IQS360_ADDR, XY_DATA, &IQS360.X.X_Low, 4);   // get XY-Data

    /*  Read Status */
    CommsIQS_repeat_start();
    CommsIQS_Read(IQS360_ADDR, STATUS, &IQS360.Touch_Channels.Touch_Channels_0, 4);   // get status

    /**
     * Only do this if you are sure that you will not keep the MCU in a stuck
     * condition. For this, the IQS360 I2C Timeout is set at 12*1.28ms
     * timeout to ensure that a stuck condition does not occur on the
     * I2C bus
     */
    //CommsIQS_stop();
}


/**
 * @brief   Handles the IQS360 events, based on the latest object
 * @param   None
 * @retval  None
 */
void IQS360_event_handler(void)
{
    /**
     * It is possible to build a buffer here or threshold, so that a write
     * only occurs after a certain delta of movement was made on the trackpad
     */
    if (X_prev.X != IQS360.X.X || Y_prev.Y != IQS360.Y.Y & !IQS360.Snap_Channels.Snap_Channels)
    {
         /*  X-Coordinates   */
        if (IQS360.X.X > 50 && IQS360.X.X < 192)
        {
            IQS360.PWM0.PWM = PWM_ON;
            IQS360.PWM1.PWM = PWM_OFF;
            IQS360.PWM2.PWM = PWM_OFF;
            IQS360.PWM3.PWM = PWM_OFF;
        }
        else if (IQS360.X.X > 192 && IQS360.X.X < 384)
        {
            IQS360.PWM0.PWM = PWM_ON;
            IQS360.PWM1.PWM = PWM_ON;
            IQS360.PWM2.PWM = PWM_OFF;
            IQS360.PWM3.PWM = PWM_OFF;
        }
        else if (IQS360.X.X > 384 && IQS360.X.X < 576)
        {
            IQS360.PWM0.PWM = PWM_ON;
            IQS360.PWM1.PWM = PWM_ON;
            IQS360.PWM2.PWM = PWM_ON;
            IQS360.PWM3.PWM = PWM_OFF;
        }
        else if (IQS360.X.X > 576 && IQS360.X.X <= 768)
        {
            IQS360.PWM0.PWM = PWM_ON;
            IQS360.PWM1.PWM = PWM_ON;
            IQS360.PWM2.PWM = PWM_ON;
            IQS360.PWM3.PWM = PWM_ON;
        }
        else
        {
            IQS360.PWM0.PWM = PWM_OFF;
            IQS360.PWM1.PWM = PWM_OFF;
            IQS360.PWM2.PWM = PWM_OFF;
            IQS360.PWM3.PWM = PWM_OFF;
        }

        /*  Y-Coordinate    */
        if (IQS360.Y.Y > 50 && IQS360.Y.Y < 128)
        {
            IQS360.PWM4.PWM = PWM_ON;
            IQS360.PWM5.PWM = PWM_OFF;
            IQS360.PWM6.PWM = PWM_OFF;
            IQS360.PWM7.PWM = PWM_OFF;
        }
        else if (IQS360.Y.Y > 128 && IQS360.Y.Y < 256)
        {
            IQS360.PWM4.PWM = PWM_ON;
            IQS360.PWM5.PWM = PWM_ON;
            IQS360.PWM6.PWM = PWM_OFF;
            IQS360.PWM7.PWM = PWM_OFF;
        }
        else if (IQS360.Y.Y > 256 && IQS360.Y.Y < 384)
        {
            IQS360.PWM4.PWM = PWM_ON;
            IQS360.PWM5.PWM = PWM_ON;
            IQS360.PWM6.PWM = PWM_ON;
            IQS360.PWM7.PWM = PWM_OFF;
        }
        else if (IQS360.Y.Y > 384 && IQS360.Y.Y <= 512)
        {
            IQS360.PWM4.PWM = PWM_ON;
            IQS360.PWM5.PWM = PWM_ON;
            IQS360.PWM6.PWM = PWM_ON;
            IQS360.PWM7.PWM = PWM_ON;
        }
        else
        {
            IQS360.PWM4.PWM = PWM_OFF;
            IQS360.PWM5.PWM = PWM_OFF;
            IQS360.PWM6.PWM = PWM_OFF;
            IQS360.PWM7.PWM = PWM_OFF;
        }

        //  Write to PWM
        CommsIQS_start();
        CommsIQS_Write(IQS360_ADDR, PWM_REG, &IQS360.PWM0.PWM, 8);
        CommsIQS_stop();
    }
    /*  Snaps occured   */
    else if (IQS360.Snap_Channels.Snap_Channels)
    {
        IQS360.PWM0.PWM = PWM_OFF;
        IQS360.PWM1.PWM = PWM_OFF;
        IQS360.PWM2.PWM = PWM_OFF;
        IQS360.PWM3.PWM = PWM_OFF;
        IQS360.PWM4.PWM = PWM_OFF;
        IQS360.PWM5.PWM = PWM_OFF;
        IQS360.PWM6.PWM = PWM_OFF;
        IQS360.PWM7.PWM = PWM_OFF;

        if (IQS360.Snap_Channels.Snap_Channels&CH1)
            IQS360.PWM0.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH2)
            IQS360.PWM1.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH3)
            IQS360.PWM2.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH4)
            IQS360.PWM3.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH5)
            IQS360.PWM4.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH6)
            IQS360.PWM5.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH7)
            IQS360.PWM6.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH8)
            IQS360.PWM7.PWM = PWM_ON;

        else if (IQS360.Snap_Channels.Snap_Channels&CH9)
        {
            IQS360.PWM0.PWM = PWM_ON;
            IQS360.PWM7.PWM = PWM_ON;
        }
        else if (IQS360.Snap_Channels.Snap_Channels&CH10)
        {
            IQS360.PWM1.PWM = PWM_ON;
            IQS360.PWM7.PWM = PWM_ON;
        }
        else if (IQS360.Snap_Channels.Snap_Channels&CH11)
        {
            IQS360.PWM2.PWM = PWM_ON;
            IQS360.PWM7.PWM = PWM_ON;
        }

        else if (IQS360.Snap_Channels.Snap_Channels&CH12)
        {
            IQS360.PWM3.PWM = PWM_ON;
            IQS360.PWM7.PWM = PWM_ON;
        }
        else
        {
            IQS360.PWM0.PWM = PWM_OFF;
            IQS360.PWM1.PWM = PWM_OFF;
            IQS360.PWM2.PWM = PWM_OFF;
            IQS360.PWM3.PWM = PWM_OFF;
            IQS360.PWM7.PWM = PWM_OFF;
        }

        //  Write to PWM
        CommsIQS_start();
        CommsIQS_Write(IQS360_ADDR, PWM_REG, &IQS360.PWM0.PWM, 8);
        CommsIQS_stop();
    }
    // nothing to write, close window - Warning: Ensure no stuck condition occurs
    else
        CommsIQS_stop();

    /*  Set the Prev IQS360 equal to this one   */
    X_prev.X = IQS360.X.X;
    Y_prev.Y = IQS360.Y.Y;
}