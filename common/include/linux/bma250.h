/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	include/linux/bma250.h
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

#ifndef LINUX_BMA250_MODULE_H
#define LINUX_BMA250_MODULE_H

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
	BMA_NO_ROT = 0,
	BMA_ROT_90,
	BMA_ROT_180,
	BMA_ROT_270,
};

struct bma250_accl_platform_data {
	struct i2c_slave_platform_data i2c_pdata;
	int orientation;
	bool invert;
	int (*init) (struct device *);
	void (*exit) (struct device *);
};

#define BMA250_CHIP_ID          3

#define BMA250_CHIP_ID_REG                      0x00
//#define BMA250_X_AXIS_REG                       0x03
//#define BMA250_Y_AXIS_REG                       0x05
//#define BMA250_Z_AXIS_REG                       0x07
#define BMA250_MODE_CTRL_REG                    0x11

#endif /* __KERNEL__ */

#define BMA_DATA_SIZE	3
/* IOCTL MACROS */
#define BMA250_ACCL_IOCTL_GET_DELAY		_IOR(0x1, 0x00, int)
#define BMA250_ACCL_IOCTL_SET_DELAY		_IOW(0x1, 0x01, int)
#define BMA250_ACCL_IOCTL_SET_FLAG		_IOW(0x1, 0x02, int)
#define BMA250_ACCL_IOCTL_GET_DATA		_IOR(0x1, 0x03, short[BMA_DATA_SIZE])

#endif /* LINUX_BMA250_MODULE_H */


