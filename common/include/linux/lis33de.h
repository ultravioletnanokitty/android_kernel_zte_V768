/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	include/linux/bma150.h
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

#ifndef LINUX_BMA150_MODULE_H
#define LINUX_BMA150_MODULE_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#include <plat/bcm_i2c.h>

#ifdef __KERNEL__

/* enum to indicate the direction
 * in which the bma sensor has been
 * placed with respect to phone
 */

enum bma_orientation {
	ST_NO_ROT = 0,
	ST_ROT_90,
	ST_ROT_180,
	ST_ROT_270,
};

struct lis33de_accl_platform_data {
	struct i2c_slave_platform_data i2c_pdata;
	int orientation;
	bool invert;
	int (*init) (struct device *);
	void (*exit) (struct device *);
};

#endif /* __KERNEL__ */


#define LIS33DE_WHO_AM_I 0x0F
#define LIS33DE_CTRL_RERG1 0x20
#define LIS33DE_CTRL_RERG2 0x21
#define LIS33DE_CTRL_RERG3 0x22
#define LIS33DE_HP_FILTER_RESET 0x23
#define LIS33DE_STATUS_REG 0x27
#define LIS33DE_OUTX  0x29
#define LIS33DE_OUTY  0x2B
#define LIS33DE_OUTZ  0x2D
#define LIS33DE_FF_WU_CFG_1 0x30
#define LIS33DE_FF_WU_SRC_1 0x31
#define LIS33DE_FF_WU_THS_1 0x32
#define LIS33DE_FF_WU_DURATION_1 0x33
#define LIS33DE_FF_WU_CFG_2 0x34
#define LIS33DE_FF_WU_SRC_2 0x35
#define LIS33DE_FF_WU_THS_2 0x36 
#define LIS33DE_FF_WU_DURATION_2	0x37
#define LIS33DE_CLICK_CFG		0x38
#define LIS33DE_CLICK_SRC		0x39
#define LIS33DE_CLICK_THSY_X		0x3B
#define LIS33DE_CLICK_THSZ		0x3C
#define LIS33DE_CLICK_TIMELIMIT		0x3D
#define LIS33DE_CLICK_LATENCY		0x3E
#define LIS33DE_CLICK_WINDOW		0x3F



#define LIS33DE_DATA_SIZE	3
/* IOCTL MACROS */
#define LIS33DE_ACCL_IOCTL_GET_DELAY		_IOR(0x1, 0x00, int)
#define LIS33DE_ACCL_IOCTL_SET_DELAY		_IOW(0x1, 0x01, int)
#define LIS33DE_ACCL_IOCTL_SET_FLAG		_IOW(0x1, 0x02, int)
#define LIS33DE_ACCL_IOCTL_GET_DATA		_IOR(0x1, 0x03, short[LIS33DE_DATA_SIZE])

#endif /* LINUX_BMA150_MODULE_H */


