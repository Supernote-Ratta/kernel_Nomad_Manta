/*
* This file contains all the necessary settings for the IQS360 and this file can
* be changed from the GUI or edited here
* File:   IQS360_init.h
* Author: Azoteq
*/

#ifndef IQS360_INIT_H
#define IQS360_INIT_H

/* Change the Multipliers (BASE value) of each channel  */
#define MULTIPLIERS_CH0						0x22
#define MULTIPLIERS_CH1						0x23
#define MULTIPLIERS_CH2						0x22
#define MULTIPLIERS_CH3						0x21
#define MULTIPLIERS_CH4						0x21
#define MULTIPLIERS_CH5						0x24
#define MULTIPLIERS_CH6						0x22
#define MULTIPLIERS_CH7						0x21
#define MULTIPLIERS_CH8						0x22
#define MULTIPLIERS_CH9						0x24
#define MULTIPLIERS_CH10					0x22
#define MULTIPLIERS_CH11					0x22
#define MULTIPLIERS_CH12					0x23

/* Change the Compensation for each channel  */
#define COMPENSATION_CH0					0x90
#define COMPENSATION_CH1					0x8C
#define COMPENSATION_CH2					0x81
#define COMPENSATION_CH3					0x86
#define COMPENSATION_CH4					0x90
#define COMPENSATION_CH5					0x84
#define COMPENSATION_CH6					0x68
#define COMPENSATION_CH7					0x70
#define COMPENSATION_CH8					0x82
#define COMPENSATION_CH9					0x85
#define COMPENSATION_CH10					0x88
#define COMPENSATION_CH11					0x6C
#define COMPENSATION_CH12					0x70

/* Change the Prox Settings or setup of the IQS360 */
#define PROXSETTINGS0_VAL					0x26
#define PROXSETTINGS1_VAL					0x00
#define PROXSETTINGS2_VAL					0x08
#define PROXSETTINGS3_VAL					0x00
#define PROXSETTINGS4_VAL					0x07
#define PROXSETTINGS5_VAL					0x7F

/* Change the Thresholds for each channel */
#define PROX_THRESHOLD						0x04
#define TOUCH_THRESHOLD_CH1					0x23
#define TOUCH_THRESHOLD_CH2					0x23
#define TOUCH_THRESHOLD_CH3					0x23
#define TOUCH_THRESHOLD_CH4					0x23
#define TOUCH_THRESHOLD_CH5					0x23
#define TOUCH_THRESHOLD_CH6					0x23
#define TOUCH_THRESHOLD_CH7					0x23
#define TOUCH_THRESHOLD_CH8					0x23
#define TOUCH_THRESHOLD_CH9					0x23
#define TOUCH_THRESHOLD_CH10				0x23
#define TOUCH_THRESHOLD_CH11				0x23
#define TOUCH_THRESHOLD_CH12				0x23

/* Change the Snap Thresholds for each channel */
#define SNAP_THRESHOLD_CH1					0x14
#define SNAP_THRESHOLD_CH2					0x14
#define SNAP_THRESHOLD_CH3					0x14
#define SNAP_THRESHOLD_CH4					0x14
#define SNAP_THRESHOLD_CH5					0x14
#define SNAP_THRESHOLD_CH6					0x14
#define SNAP_THRESHOLD_CH7					0x14
#define SNAP_THRESHOLD_CH8					0x14
#define SNAP_THRESHOLD_CH9					0x14
#define SNAP_THRESHOLD_CH11					0x14
#define SNAP_THRESHOLD_CH11					0x14
#define SNAP_THRESHOLD_CH12					0x14

/* Change the Timing settings */
#define FILTER_HALT							0x4F
#define POWER_MODE							0x00
#define TIMEOUT_PERIOD						0x0C//0x10

/* Change ATI Target values  */
#define ATI_TARGET_CH0						0x40
#define ATI_TARGET_CH0_9					0x20

/* Change PWM settings  */
#define PWM_0								0x00
#define PWM_1								0x00
#define PWM_2								0x00
#define PWM_3								0x00
#define PWM_4								0x00
#define PWM_5								0x00
#define PWM_6								0x00
#define PWM_7								0x00

/* Change PWM limits and Speed */
#define PWM_LIMITS							0x01
#define PWM_SPEED							0x01

/* Set Active Channels */
#define ACTIVE_CH0							0xFF
#define ACTIVE_CH1							0x1F
#define TRACKPAD_CH0						0xFE
#define TRACKPAD_CH1						0x1F

/* Set Buzzer Output */
#define BUZZER_VAL							0x00

#endif	/* IQS360_INIT_H */
