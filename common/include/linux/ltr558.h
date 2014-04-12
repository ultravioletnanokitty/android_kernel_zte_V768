/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
* 	@file	include/linux/ltr558.h
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

#ifndef LINUX_LTR558_MODULE_H
#define LINUX_LTR558_MODULE_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#include <plat/bcm_i2c.h>

#ifdef __KERNEL__

struct ltr558_platform_data {
	struct i2c_slave_platform_data i2c_pdata;
	int (*init) (void);
	//int (*init) (struct device *);
	void (*exit) (struct device *);
};

/* LTR-558 Registers */
#define LTR558_ALS_CONTR	0x80
#define LTR558_PS_CONTR		0x81
#define LTR558_PS_LED		0x82
#define LTR558_PS_N_PULSES	0x83
#define LTR558_PS_MEAS_RATE	0x84
#define LTR558_ALS_MEAS_RATE	0x85
#define LTR558_MANUFACTURER_ID	0x87

#define LTR558_INTERRUPT	0x8F
#define LTR558_PS_THRES_UP_0	0x90
#define LTR558_PS_THRES_UP_1	0x91
#define LTR558_PS_THRES_LOW_0	0x92
#define LTR558_PS_THRES_LOW_1	0x93

#define LTR558_ALS_THRES_UP_0	0x97
#define LTR558_ALS_THRES_UP_1	0x98
#define LTR558_ALS_THRES_LOW_0	0x99
#define LTR558_ALS_THRES_LOW_1	0x9A

#define LTR558_INTERRUPT_PERSIST 0x9E

/* 558's Read Only Registers */
#define LTR558_ALS_DATA_CH1_0	0x88
#define LTR558_ALS_DATA_CH1_1	0x89
#define LTR558_ALS_DATA_CH0_0	0x8A
#define LTR558_ALS_DATA_CH0_1	0x8B
#define LTR558_ALS_PS_STATUS	0x8C
#define LTR558_PS_DATA_0	0x8D
#define LTR558_PS_DATA_1	0x8E


/* Basic Operating Modes */
#define MODE_ALS_ON_Range1	0x3B
#define MODE_ALS_ON_Range2	0x33
#define MODE_ALS_StdBy		0x00

#define MODE_PS_ON_Gain1	0x03
#define MODE_PS_ON_Gain2	0x07
#define MODE_PS_ON_Gain4	0x0B
#define MODE_PS_ON_Gain8	0x0C
#define MODE_PS_StdBy		0x00

#define PS_RANGE1 	1
#define PS_RANGE2	2
#define PS_RANGE4 	4
#define PS_RANGE8	8

#define ALS_RANGE1_320	1
#define ALS_RANGE2_64K 	2

/* 
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR558_IOCTL_MAGIC      'c'

/* IOCTLs for ltr558 device */
#define LTR558_IOCTL_PS_ENABLE		_IOW(LTR558_IOCTL_MAGIC, 0, char *)
#define LTR558_IOCTL_ALS_ENABLE		_IOW(LTR558_IOCTL_MAGIC, 1, char *)
#define LTR558_IOCTL_READ_PS_DATA	_IOR(LTR558_IOCTL_MAGIC, 2, char *)
#define LTR558_IOCTL_READ_PS_INT	_IOR(LTR558_IOCTL_MAGIC, 3, char *)
#define LTR558_IOCTL_READ_ALS_DATA	_IOR(LTR558_IOCTL_MAGIC, 4, char *)
#define LTR558_IOCTL_READ_ALS_INT	_IOR(LTR558_IOCTL_MAGIC, 5, char *)
#define LTR558_IOCTL_GET_ENABLED    _IOR(LTR558_IOCTL_MAGIC, 6, char *) 
#define LTR558_IOCTL_GET_ALS_ENABLED    _IOR(LTR558_IOCTL_MAGIC, 7, char *)

/* Power On response time in ms */
#define PON_DELAY	600
#define WAKEUP_DELAY	10

/* Interrupt vector number to use when probing IRQ number.
 * User changeable depending on sys interrupt.
 * For IRQ numbers used, see /proc/interrupts.
 */
#define GPIO_INT_NO	14


#endif

#endif /* LINUX_LTR558_MODULE_H */
