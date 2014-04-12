/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_HX8357B.h
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

/****************************************************************************
*
*  lcd.c
*
*  PURPOSE:
*    This is the LCD-specific code for a HX8357B module.
*
*****************************************************************************/

#ifndef __BCM_LCD_HX8357B__
#define __BCM_LCD_HX8357B__

#define RESET_SEQ {WR_CMND, 0x2A, 0},\
		{WR_DATA, 0x00, (dev->col_start) >> 8},\
		{WR_DATA, 0x00, dev->col_start & 0xFF},\
		{WR_DATA, 0x01, (dev->col_end) >> 8},\
		{WR_DATA, 0x3F, dev->col_end & 0xFF},\
		{WR_CMND, 0x2B, 0},\
		{WR_DATA, 0x00, (dev->row_start) >> 8},\
		{WR_DATA, 0x00, dev->row_start & 0xFF},\
		{WR_DATA, 0x01, (dev->row_end) >> 8},\
		{WR_DATA, 0xE0, dev->row_end & 0xFF},\
		{WR_CMND, 0x2C, 0}

#define LCD_CMD(x) (x)
#define LCD_DATA(x) (x)

#define LCD_HEIGHT              480
#define LCD_WIDTH               320

#define PANEL_HEIGHT              480
#define PANEL_WIDTH               320

#define LCD_BITS_PER_PIXEL	16


#define TEAR_SCANLINE	480

const char *LCD_panel_name = "HVGA HX8357B Controller";

int LCD_num_panels = 1;
LCD_Intf_t LCD_Intf = LCD_Z80;
LCD_Bus_t LCD_Bus = LCD_18BIT;
/* yaogangxiang@wind-mobi.com 2011.11.09 begin */
//review by liubin
//modify LCD can not display normally
//CSL_LCDC_PAR_SPEED_t timingReg_ns = {24, 25, 0, 3, 6, 1};
//CSL_LCDC_PAR_SPEED_t timingMem_ns = {24, 25, 0, 3, 6, 1};
/* yaogangxiang@wind-mobi.com 2011.11.09 begin */
//review by liubin
//modify LCD can not display normally
//CSL_LCDC_PAR_SPEED_t timingReg_ns = {180, 188, 0, 36, 50, 14};
//CSL_LCDC_PAR_SPEED_t timingMem_ns = {180, 188, 0, 36, 50, 14};

//xiongbiao@wind-mobi.com 2011-11-24 begin
//fix lcd blur displuy issue. 
//reviewed by yuanlan@wind-mobi.com
//CSL_LCDC_PAR_SPEED_t timingReg_ns = {180, 188, 0, 36, 50, 14};
//CSL_LCDC_PAR_SPEED_t timingMem_ns = {180, 188, 0, 36, 50, 14};
CSL_LCDC_PAR_SPEED_t timingReg_ns = {180, 188, 0, 44, 58, 22};
CSL_LCDC_PAR_SPEED_t timingMem_ns = {180, 188, 0, 44, 58, 22};
//xiongbiao@wind-mobi.com 2011-11-24 end


/* yaogangxiang@wind-mobi.com 2011.11.09 end */

LCD_dev_info_t LCD_device[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= LCD_HEIGHT,
	 .width		= LCD_WIDTH,
	 .bits_per_pixel = LCD_BITS_PER_PIXEL,
	 .te_supported	= false    //.te_supported	= false
	 }
};


Lcd_init_t power_on_seq[] = {
	
	{WR_CMND, 0x11, 0}, 
	{SLEEP_MS,0, 200},	//Exit sleep

	{WR_CMND, 0xB4, 0}, 	//Set RM  DM
	{WR_DATA, 0x00, 0x00},
			
	{WR_CMND, 0xC0, 0}, 	//Set PANEL
	{WR_DATA, 0x00, 0x14},
	{WR_DATA, 0x00, 0x3B},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x11},
				
	{WR_CMND, 0xC5, 0}, 	
	{WR_DATA, 0x00, 0x0d},
			
	{WR_CMND, 0xC8, 0}, 	//set Gamma2.5
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x24},
	{WR_DATA, 0x00, 0x10},
	{WR_DATA, 0x00, 0x10},
	{WR_DATA, 0x00, 0x01},
	{WR_DATA, 0x00, 0x08},
	{WR_DATA, 0x00, 0x76},
	{WR_DATA, 0x00, 0x35},
	{WR_DATA, 0x00, 0x77},
	{WR_DATA, 0x00, 0x01},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x02},
		
	{WR_CMND, 0xD0, 0}, 	//Set Power
	{WR_DATA, 0x00, 0x44},	//DDVDH
	{WR_DATA, 0x00, 0x41},
	{WR_DATA, 0x00, 0x04},	//VREG1 		
	{WR_DATA, 0x00, 0xC4},	//XDK
		
	{WR_CMND, 0xD1, 0}, 	//Set VCOM
	{WR_DATA, 0x00, 0x70},	//VCOMH    70
//xiongbiao@wind-mobi.com 2011-11-23 begin
//improve display effect.
//reviewed by yuanlan@wind-mobi.com
//	 {WR_DATA, 0x00, 0x19},  //VCOML	
	{WR_DATA, 0x00, 0x11},  //VCOML 
//xiongbiao@wind-mobi.com 2011-11-23 end

		
	{WR_CMND, 0xD2, 0}, 	//Set NOROW
	{WR_DATA, 0x00, 0x05},	//SAP
	{WR_DATA, 0x00, 0x12},	//DC10/00
		
	//{WR_CMND, 0xD3, 0},	//Set NOROW
	//{WR_DATA, 0x00, 0x04},	//SAP
	//{WR_DATA, 0x00, 0x12},	//DC10/00
			
	//{WR_CMND, 0xD4, 0},	//Set NOROW
	//{WR_DATA, 0x00, 0x07},	//SAP
	//{WR_DATA, 0x00, 0x12},	//DC10/00

	{WR_CMND, 0xEe, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x2c},

	{WR_CMND, 0xEd, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x9a},
	{WR_DATA, 0x00, 0x9a},
	{WR_DATA, 0x00, 0x9b},
	{WR_DATA, 0x00, 0x9b},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0xAE},
	{WR_DATA, 0x00, 0xAE},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x9B},
	{WR_DATA, 0x00, 0x2c},
		
	{WR_CMND, 0xE9, 0}, 	//Set Color Mode 65K
	{WR_DATA, 0x00, 0x01},	//

	{WR_CMND, 0x35, 0}, 	
	{WR_DATA, 0x00, 0x00},

	{WR_CMND, 0xEA, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x03},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
			
	{WR_CMND, 0x3A, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x06},	//0x06

	{SLEEP_MS, 0, 5},
	{WR_CMND, 0x29, 0},    //Display on
	{SLEEP_MS, 0, 25},			

	{CTRL_END, 0, 0},

};

Lcd_init_t power_off_seq[] = {
	{WR_CMND, 0x28, 0},
	{WR_CMND, 0x10, 0},
	{SLEEP_MS, 0, 120},
	{CTRL_END, 0, 0},
};
#endif

