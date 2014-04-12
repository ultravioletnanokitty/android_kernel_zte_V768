/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _SYNAPTICS_RMI4_GENERIC_H_
#define _SYNAPTICS_RMI4_GENERIC_H_
#include <plat/bcm_i2c.h>

/**
 * struct synaptics_rmi4_platform_data - contains the rmi4 platform data
 * @irq_number: irq number
 * @irq_type: irq type
 * @x flip: x flip flag
 * @y flip: y flip flag
 * @regulator_en: regulator enable flag
 * @gpio_config: callback for gpio set up
 *
 * This structure gives platform data for rmi4.
 */
struct synaptics_rmi4_platform_data {
	bool x_flip;
	bool y_flip;
	bool regulator_en;
	int irq_type;
    unsigned gpio_irq;

	int (*gpio_config)(unsigned interrupt_gpio, bool configure);

	
	struct i2c_slave_platform_data i2c_pdata;

	/* Screen area definition */
	int            scr_x_min;
	int            scr_x_max;
	int            scr_y_min;
	int            scr_y_max;

};

#endif
