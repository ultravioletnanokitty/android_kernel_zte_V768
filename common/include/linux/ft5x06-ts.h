/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	include/linux/ft5x06-ts.h
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

#ifndef _LINUX_FT5X06_I2C_H
#define _LINUX_FT5X06_I2C_H
#include <plat/bcm_i2c.h>


struct ft5x06_platform_data {
	struct i2c_slave_platform_data i2c_pdata;

	/* Screen area definition */
	int            scr_x_min;
	int            scr_x_max;
	int            scr_y_min;
	int            scr_y_max;

	/* Virtual key definition */
	int            *virtual_keys;

	int            (*init_platform_hw)(void);
	void           (*exit_platform_hw)(void);
	/* xuhuashan@gmail.com 2011.05.17 begin */
	/* Add power management support for FT5x06 driver */
	int            (*powerdown_chip)(void);  /* power down the chip */
	int            (*wakeup_chip)(void);     /* wakeup from power down mode */
	/* xuhuashan@gmail.com 2011.05.17 end */
};

#endif	/* _LINUX_FT5X06_I2C_H */

