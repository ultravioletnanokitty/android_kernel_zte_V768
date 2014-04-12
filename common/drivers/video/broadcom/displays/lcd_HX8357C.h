/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_HX8357C.h
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
*    This is the LCD-specific code for a HX8357C module.
*
*****************************************************************************/

#ifndef __BCM_LCD_HX8357C__
#define __BCM_LCD_HX8357C__

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

const char *LCD_panel_name = "HVGA HX8357C Controller";

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
	{SLEEP_MS,0, 120},	//Exit sleep

	{WR_CMND, 0xB9, 0}, 	
	{WR_DATA, 0x00, 0xFF},
	{WR_DATA, 0x00, 0x83},
	{WR_DATA, 0x00, 0x57},

	{WR_CMND, 0xCC, 0}, 	
	{WR_DATA, 0x00, 0x09},

	{WR_CMND, 0xB6, 0}, 	
	{WR_DATA, 0x00, 0x52},
	
	{WR_CMND, 0x36, 0}, 	
	{WR_DATA, 0x00, 0xC0},
	
	{WR_CMND, 0xB1, 0}, 	
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x25},
	{WR_DATA, 0x00, 0x2B},
	{WR_DATA, 0x00, 0x2B},
	{WR_DATA, 0x00, 0x83},
	{WR_DATA, 0x00, 0x48},
						
	{WR_CMND, 0xB4, 0}, 	
	{WR_DATA, 0x00, 0x02}, //NW
	{WR_DATA, 0x00, 0x40}, //RTN
	{WR_DATA, 0x00, 0x00}, //DIV
	{WR_DATA, 0x00, 0x2A}, //DUM
	{WR_DATA, 0x00, 0x2A}, //DUM
	{WR_DATA, 0x00, 0x0D}, //GDON
	{WR_DATA, 0x00, 0x78}, //GDOFF
	{WR_DATA, 0x00, 0xC0}, //STBA
	{WR_DATA, 0x00, 0x50}, //OPON
	{WR_DATA, 0x00, 0x50}, //OPON
	{WR_DATA, 0x00, 0x01}, //
	{WR_DATA, 0x00, 0x3C},
	{WR_DATA, 0x00, 0xC8},
	{WR_DATA, 0x00, 0x08},  //GEN

						
	{WR_CMND, 0xE0, 0}, 	
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x22},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x49},
	{WR_DATA, 0x00, 0x55},
	{WR_DATA, 0x00, 0x45},
	{WR_DATA, 0x00, 0x3E},
	{WR_DATA, 0x00, 0x39},
	{WR_DATA, 0x00, 0x30},
	{WR_DATA, 0x00, 0x2B},
	{WR_DATA, 0x00, 0x25},
	{WR_DATA, 0x00, 0x22},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x22},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x49},
	{WR_DATA, 0x00, 0x55},
	{WR_DATA, 0x00, 0x45},
	{WR_DATA, 0x00, 0x3E},
	{WR_DATA, 0x00, 0x39},
	{WR_DATA, 0x00, 0x30},
	{WR_DATA, 0x00, 0x2B},
	{WR_DATA, 0x00, 0x25},
	{WR_DATA, 0x00, 0x22},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x01},


	{WR_CMND, 0x29, 0},    //Display on
	
	{WR_CMND, 0x2C, 0},    
	
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

