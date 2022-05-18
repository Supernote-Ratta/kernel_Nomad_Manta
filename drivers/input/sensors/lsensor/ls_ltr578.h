/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for LTR578 als/ps sensor chip.
 */
#ifndef _LTR578_H_
#define _LTR578_H_

#define LTR578_I2C_SLAVE_ADDR   0x53//0xA6 // 0x53 << 1
#define DRIVER_VERSION          "2.0"

/* LTR-578 Registers */
#define APS_RW_MAIN_CTRL        0x00
#define APS_RW_PS_LED           0x01
#define APS_RW_PS_N_PULSES      0x02
#define APS_RW_PS_MEAS_RATE     0x03
#define APS_RW_ALS_MEAS_RATE    0x04
#define APS_RW_ALS_GAIN         0x05

#define APS_RW_INTERRUPT        0x19
#define APS_RW_INT_PST          0x1A
#define APS_RW_PS_THRES_UP_0    0x1B
#define APS_RW_PS_THRES_UP_1    0x1C
#define APS_RW_PS_THRES_LOW_0   0x1D
#define APS_RW_PS_THRES_LOW_1   0x1E
#define APS_RW_PS_CAN_0         0x1F
#define APS_RW_PS_CAN_1         0x20
#define APS_RW_ALS_THRES_UP_0   0x21
#define APS_RW_ALS_THRES_UP_1   0x22
#define APS_RW_ALS_THRES_UP_2   0x23
#define APS_RW_ALS_THRES_LOW_0  0x24
#define APS_RW_ALS_THRES_LOW_1  0x25
#define APS_RW_ALS_THRES_LOW_2  0x26

/* 578's Read Only Registers */
#define APS_RO_PART_ID          0x06
#define APS_RO_MAIN_STATUS      0x07
#define APS_RO_PS_DATA_0        0x08
#define APS_RO_PS_DATA_1        0x09
#define APS_RO_CLEAR_DATA_0     0x0A
#define APS_RO_CLEAR_DATA_1     0x0B
#define APS_RO_CLEAR_DATA_2     0x0C
#define APS_RO_ALS_DATA_0       0x0D
#define APS_RO_ALS_DATA_1       0x0E
#define APS_RO_ALS_DATA_2       0x0F

/* Basic Operating Modes */
#define MODE_ALS_Range1         0x00  ///for als gain x1
#define MODE_ALS_Range3         0x01  ///for als gain x3
#define MODE_ALS_Range6         0x02  ///for als gain x6
#define MODE_ALS_Range9         0x03  ///for als gain x9
#define MODE_ALS_Range18        0x04  ///for als gain x18

#define ALS_RANGE_1             1
#define ALS_RANGE_3             3
#define ALS_RANGE_6             6
#define ALS_RANGE_9             9
#define ALS_RANGE_18            18

#define ALS_RESO_MEAS           0x22
#define ALS_DEF_GAIN            3

#define ALS_WIN_FACTOR          1
#define ALS_WIN_FACTOR2         5
#define ALS_USE_CLEAR_DATA      0

/* Power On response time in ms */
#define PON_DELAY               600
#define WAKEUP_DELAY            10

#define PARTID                  0xB1
#endif
