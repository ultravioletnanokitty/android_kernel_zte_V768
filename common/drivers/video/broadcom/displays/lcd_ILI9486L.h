/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_ILI9481.h
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
*    This is the LCD-specific code for a ILI9486L module.
*
*****************************************************************************/

#ifndef __BCM_LCD_ILI9486L__
#define __BCM_LCD_ILI9486L__

#define RESET_SEQ {WR_CMND, 0x2A, 0},\
		{WR_DATA, 0x00, (dev->col_start) >> 8},\
		{WR_DATA, 0x00, dev->col_start & 0xFF},\
		{WR_DATA, 0x00, (dev->col_end) >> 8},\
		{WR_DATA, 0x00, dev->col_end & 0xFF},\
		{WR_CMND, 0x2B, 0},\
		{WR_DATA, 0x00, (dev->row_start) >> 8},\
		{WR_DATA, 0x00, dev->row_start & 0xFF},\
		{WR_DATA, 0x00, (dev->row_end) >> 8},\
		{WR_DATA, 0x00, dev->row_end & 0xFF},\
		{WR_CMND, 0x2C, 0}

#define LCD_CMD(x) (x)
#define LCD_DATA(x) (x)

#define LCD_HEIGHT              480
#define LCD_WIDTH               320

#define PANEL_HEIGHT              480
#define PANEL_WIDTH               320

#define LCD_BITS_PER_PIXEL	16


#define TEAR_SCANLINE	480

const char *LCD_panel_name = "HVGA ILI 9486L Controller";

int LCD_num_panels = 1;
LCD_Intf_t LCD_Intf = LCD_Z80;
LCD_Bus_t LCD_Bus = LCD_16BIT;

/* yaogangxiang@wind-mobi.com 2011.11.09 begin */
//review by liubin
//modify LCD can not display normally
//CSL_LCDC_PAR_SPEED_t timingReg_ns = {24, 25, 0, 3, 6, 1};
//CSL_LCDC_PAR_SPEED_t timingMem_ns = {24, 25, 0, 3, 6, 1};
CSL_LCDC_PAR_SPEED_t timingReg_ns = {180, 188, 0, 36, 50, 14};
CSL_LCDC_PAR_SPEED_t timingMem_ns = {180, 188, 0, 36, 50, 14};
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
	{SLEEP_MS,0, 200},      //Exit sleep
	
	{WR_CMND, 0x36, 0},
//	{WR_DATA, 0x00, 0x48},
	
	{WR_DATA, 0x00, 0x88},
	{WR_CMND, 0x3A, 0},     //set pixel format  
	{WR_DATA, 0x00, 0x55},  //16bpp 0x66=>18bpp
//xiongbiao@wind-mobi.com 2011.11.29 begin
//reviewed by yuanlan@wind-mobi.com
//modify LCD effect in internet	
/*	
	{WR_CMND, 0xE0, 0},     //set GammaS
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x0A},
	{WR_DATA, 0x00, 0x03},
	{WR_DATA, 0x00, 0x13},
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x2E},
	{WR_DATA, 0x00, 0x9B},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x0B},
	{WR_DATA, 0x00, 0x3F},
	{WR_DATA, 0x00, 0x3F},
	{WR_DATA, 0x00, 0x0F},
		
	{WR_CMND, 0xE1, 0}, 	//set Gamma
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x16},
	{WR_DATA, 0x00, 0x0D},
	{WR_DATA, 0x00, 0x3B},
	{WR_DATA, 0x00, 0x56},
	{WR_DATA, 0x00, 0x51},
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x18},
	{WR_DATA, 0x00, 0x0C},
	{WR_DATA, 0x00, 0x35},
	{WR_DATA, 0x00, 0x2A},
	{WR_DATA, 0x00, 0x0F},

	{WR_CMND, 0xF1, 0},
	{WR_DATA, 0x00, 0x36},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x00},			
	{WR_DATA, 0x00, 0x3C},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0xA4},
	{WR_DATA, 0x00, 0x02},
	
	{WR_CMND, 0xF2, 0},
	{WR_DATA, 0x00, 0x18},
	{WR_DATA, 0x00, 0xA3},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x72},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0xFF},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x00},
*/
	{WR_CMND, 0xE0, 0}, 	//set GammaS
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x1A}, //0x0A 
	{WR_DATA, 0x00, 0x03},
	{WR_DATA, 0x00, 0x0F}, //0x13
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x30}, //0x2E
	{WR_DATA, 0x00, 0x9B},
	{WR_DATA, 0x00, 0x47}, //0x44
	{WR_DATA, 0x00, 0x05}, //0x02
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x07}, //0x0B
	{WR_DATA, 0x00, 0x1F}, //0x3F
	{WR_DATA, 0x00, 0x2E}, //0x3F
	{WR_DATA, 0x00, 0x0F},
			
	{WR_CMND, 0xE1, 0}, 	//set Gamma
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x11}, //0x00
	{WR_DATA, 0x00, 0x20}, //0x00
	{WR_DATA, 0x00, 0x08}, //0x04
	{WR_DATA, 0x00, 0x16},
	{WR_DATA, 0x00, 0x0A}, //0x0D
	{WR_DATA, 0x00, 0x38}, //0x3B
	{WR_DATA, 0x00, 0x56},
	{WR_DATA, 0x00, 0x4f}, //0x51
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x10}, //0x18
	{WR_DATA, 0x00, 0x0C},
	{WR_DATA, 0x00, 0x25}, //0x35
	{WR_DATA, 0x00, 0x2A},
	{WR_DATA, 0x00, 0x0F},
			
	{WR_CMND, 0xF1, 0},
	{WR_DATA, 0x00, 0x36},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x00},			
	{WR_DATA, 0x00, 0x3C},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0xA4},
	{WR_DATA, 0x00, 0x02},
/*		
	{WR_CMND, 0xF2, 0},
	{WR_DATA, 0x00, 0x18},
	{WR_DATA, 0x00, 0xA3},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x72},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0xFF},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x00},
*/
//xiongbiao@wind-mobi.com 2011.11.29 end

	
	{WR_CMND, 0xF7, 0},
	{WR_DATA, 0x00, 0xA9},	
	{WR_DATA, 0x00, 0x91},
	{WR_DATA, 0x00, 0x2D},
	{WR_DATA, 0x00, 0x0A},
	{WR_DATA, 0x00, 0x4F},
	
	{WR_CMND, 0xF8, 0},
	{WR_DATA, 0x00, 0x21},
	{WR_DATA, 0x00, 0x04},

	{WR_CMND, 0xB4, 0},     //Set 2Dot Invertion
	{WR_DATA, 0x00, 0x02},
	
	{WR_CMND, 0xB1, 0},     //Set Frame Rate 
	{WR_DATA, 0x00, 0xB0},
	{WR_DATA, 0x00, 0x11},
	
	{WR_CMND, 0xC0, 0},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0x0F},		

	{WR_CMND, 0xC1, 0},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x07},	

	{WR_CMND, 0xC5, 0},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x26},
	{WR_DATA, 0x00, 0x80},

	{WR_CMND, 0x29, 0},    //Display on
	{SLEEP_MS, 0, 120},			
	{WR_CMND, 0x2C, 0},	
	{CTRL_END, 0, 0},
};

Lcd_init_t power_off_seq[] = {
//	{WR_CMND, 0x28, 0},
	{WR_CMND, 0x11, 0},
	{SLEEP_MS, 0, 120},
	{CTRL_END, 0, 0},
};
#endif
