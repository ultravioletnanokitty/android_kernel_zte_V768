/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_NT35510_4014TN.h
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
*  lcd_NT35510_4014TN.h
*
*  PURPOSE:
*    This is the LCD-specific code for a hx8363b module.
*   xiongbiao 2012.03.06
*****************************************************************************/

#ifndef __LCD_NT35510_4014TN_COMPATIBLE_H__
#define __LCD_NT35510_4014TN_COMPATIBLE_H__

#include "lcd_dsi_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif
#define LCD_INFO_ALIAS(var)	nt35510_4014tn##_##var

#define DSI_VC            (0)

#define LCD_HEIGHT		800
#define LCD_WIDTH		480

#define LCD_BITS_PER_PIXEL 32

const char LCD_INFO_ALIAS(LCD_panel_name)[] = "NT35510_4014TN LCD";

//Not used, just to satisfy the compilation
CSL_LCDC_PAR_SPEED_t timingReg_ns;
CSL_LCDC_PAR_SPEED_t timingMem_ns;


LCD_dev_info_t LCD_INFO_ALIAS(LCD_device)[1] = {
{
	 .panel		= LCD_main_panel,
	 .height	= LCD_HEIGHT,
	 .width		= LCD_WIDTH,
	 .bits_per_pixel= LCD_BITS_PER_PIXEL,
	 .te_supported	= true,
	}
};


//TE Info
CSL_DSI_TE_IN_CFG_t LCD_INFO_ALIAS(teInCfg) = 
{
    CSL_DSI_TE_MODE_VSYNC,      // te mode 
    CSL_DSI_TE_ACT_POL_LO,      // sync polarity
    0,                          // vsync_width [tectl_clk_count]
    0,                          // hsync_line
};    


// DSI Command Mode VC Configuration
CSL_DSI_CM_VC_t LCD_INFO_ALIAS(dsiVcCmCfg) = 
{
    DSI_VC,               			// VC
    DSI_DT_LG_DCS_WR,               // dsiCmnd       
    MIPI_DCS_WRITE_MEMORY_START,    // dcsCmndStart       
    MIPI_DCS_WRITE_MEMORY_CONTINUE, // dcsCmndCont  
    FALSE,                          // isLP 
    LCD_IF_CM_I_RGB888U,            // cm_in         
    LCD_IF_CM_O_RGB888,             // cm_out        
    // TE configuration
    {
        DSI_TE_CTRLR_INPUT_0,
        &LCD_INFO_ALIAS(teInCfg)                    // DSI Te Input Config
    },
 };

// DSI BUS CONFIGURATION
CSL_DSI_CFG_t LCD_INFO_ALIAS(dsiCfg) = {
    0,             // bus             set on open
    1,             // dlCount
    DSI_DPHY_0_92, // DSI_DPHY_SPEC_T
    // ESC CLK Config
    {156,2},       // escClk          fixed   156[MHz], DIV by 2 =  78[MHz] 

    // HS CLK Config
    {450,1},       // hsBitClk        PLL     300[MHz], DIV by 1 = 300[Mbps]
    // LP Speed
    5,             // lpBitRate_Mbps, Max 10[Mbps]
    FALSE,         // enaContClock 
    TRUE,          // enaRxCrc                
    TRUE,          // enaRxEcc               
    TRUE,          // enaHsTxEotPkt           
    FALSE,         // enaLpTxEotPkt        
    FALSE,         // enaLpRxEotPkt        
};    

static uint8_t LCD_INFO_ALIAS(param1)[] = {0x55,0xAA,0x52,0x08,0x01};
static uint8_t LCD_INFO_ALIAS(param2)[] = {0x00,0x90,0x1A};
static uint8_t LCD_INFO_ALIAS(param3)[] = {0x00, 0x4E};
static uint8_t LCD_INFO_ALIAS(param4)[] = {
		0x00,0x00,0x00,0x2D,0x00,0x5C,0x00,0x80,
		0x00,0xAB,0x00,0xE4,0x01,0x15,0x01,0x5C,
		0x01,0x8E,0x01,0xD3,0x02,0x03,0x02,0x45,
		0x02,0x77,0x02,0x78,0x02,0xA4,0x02,0xD1,
		0x02,0xEA,0x03,0x09,0x03,0x1A,0x03,0x32,
		0x03,0x40,0x03,0x59,0x03,0x68,0x03,0x7C,
		0x03,0xB2,0x03,0xD8};
//static uint8_t LCD_INFO_ALIAS(param5)[] = {0x00,0x00,0x00};
static uint8_t LCD_INFO_ALIAS(param5)[] = {0x03,0x03,0x03};
static uint8_t LCD_INFO_ALIAS(param6)[] = {0x54,0x54,0x54};
//static uint8_t LCD_INFO_ALIAS(param6)[] = {0x45,0x45,0x45};
//static uint8_t LCD_INFO_ALIAS(param7)[] = {0x26,0x26,0x26};
static uint8_t LCD_INFO_ALIAS(param7)[] = {0x35,0x35,0x35};
//static uint8_t LCD_INFO_ALIAS(param8)[] = {0x36,0x36,0x36};
static uint8_t LCD_INFO_ALIAS(param8)[] = {0x44,0x44,0x44};
static uint8_t LCD_INFO_ALIAS(param9)[] = {0x34,0x34,0x34};
static uint8_t LCD_INFO_ALIAS(param10)[] = {0x55,0xAA,0x52,0x08,0x00};
static uint8_t LCD_INFO_ALIAS(param11)[] = {0x71,0x71};
//static uint8_t LCD_INFO_ALIAS(param12)[] = {0x01,0x0A,0x0A,0x0A};
static uint8_t LCD_INFO_ALIAS(param12)[] = {0x01,0x07,0x07,0x07};
static uint8_t LCD_INFO_ALIAS(param13)[] = {0x05,0x05,0x05};
static uint8_t LCD_INFO_ALIAS(param14)[] = {0x01,0x84,0x07,0x31,0x00};
static uint8_t LCD_INFO_ALIAS(param15)[] = {0x11,0x00,0x00,0x00,0x00};



Lcd_init_t LCD_INFO_ALIAS(power_on_seq)[] = {
    {WR_CMND_MULTIPLE_DATA,0xF0, 0,5,  LCD_INFO_ALIAS(param1)},
    {WR_CMND_MULTIPLE_DATA,0xBC, 0,3,  LCD_INFO_ALIAS(param2)},
    {WR_CMND_MULTIPLE_DATA,0xBD, 0,3,  LCD_INFO_ALIAS(param2)},
    {WR_CMND_MULTIPLE_DATA,0xBE, 0,2,  LCD_INFO_ALIAS(param3)},  //VCOM
    {WR_CMND_MULTIPLE_DATA,0xD1, 0,52,  LCD_INFO_ALIAS(param4)},//R+
    {WR_CMND_MULTIPLE_DATA,0xD2, 0,52,  LCD_INFO_ALIAS(param4)},//G+
    {WR_CMND_MULTIPLE_DATA,0xD3, 0,52,  LCD_INFO_ALIAS(param4)},//B+
    {WR_CMND_MULTIPLE_DATA,0xD4, 0,52,  LCD_INFO_ALIAS(param4)},//R-
	{WR_CMND_MULTIPLE_DATA,0xD5, 0,52,	LCD_INFO_ALIAS(param4)},//G-
	{WR_CMND_MULTIPLE_DATA,0xD6, 0,52,	LCD_INFO_ALIAS(param4)},//B-
	{WR_CMND_MULTIPLE_DATA,0xB0, 0,3,  LCD_INFO_ALIAS(param5)},//Set AVDD Voltage
	{WR_CMND_MULTIPLE_DATA,0xB6, 0,3,  LCD_INFO_ALIAS(param6)},//AVDD=2.5X VDCI
	{WR_CMND_MULTIPLE_DATA,0xB8, 0,3,  LCD_INFO_ALIAS(param7)},
	{WR_CMND_MULTIPLE_DATA,0xB1, 0,3,  LCD_INFO_ALIAS(param5)},//Set AVEE Voltage 
	{WR_CMND_MULTIPLE_DATA,0xB7, 0,3,  LCD_INFO_ALIAS(param8)},//AVDD=-2.5X VDCI
	{WR_CMND_MULTIPLE_DATA,0xB9, 0,3,  LCD_INFO_ALIAS(param9)},//Set VGH
	{WR_CMND_MULTIPLE_DATA,0xBA, 0,3,  LCD_INFO_ALIAS(param7)},
	{WR_CMND_MULTIPLE_DATA,0xF0, 0,5,  LCD_INFO_ALIAS(param10)},
	{WR_CMND_DATA,	 0xB1, 0xFC},//RAM keep//
	{WR_CMND_DATA,	 0xB4, 0x10},//Vivid Color	//
	//{WR_CMND_DATA,	 0xB6, 0x07},//SDT//
	{WR_CMND_DATA,	 0xB6, 0x02},//SDT//
	{WR_CMND_MULTIPLE_DATA,0xB7, 0,2,  LCD_INFO_ALIAS(param11)},
	{WR_CMND_MULTIPLE_DATA,0xB8, 0,4,  LCD_INFO_ALIAS(param12)},
	{WR_CMND_MULTIPLE_DATA,0xBC, 0,3,  LCD_INFO_ALIAS(param13)},
	{WR_CMND_MULTIPLE_DATA,0xBD, 0,5,  LCD_INFO_ALIAS(param14)},
	{WR_CMND_MULTIPLE_DATA,0xBE, 0,5,  LCD_INFO_ALIAS(param14)},
	{WR_CMND_MULTIPLE_DATA,0xBF, 0,5,  LCD_INFO_ALIAS(param14)},
	{WR_CMND_DATA,	 0x35, 0x00},
	{WR_CMND,	0x11, 0},//Sleep Out	  
	{SLEEP_MS,	0,	400}, 
	
	{WR_CMND_DATA,	 0x3A, 0x77}, //
	{WR_CMND_MULTIPLE_DATA,0xF0, 0,5,  LCD_INFO_ALIAS(param10)},	
	{WR_CMND_DATA,	 0xC7, 0x02}, 
	{WR_CMND_MULTIPLE_DATA,0xC9, 0,5,  LCD_INFO_ALIAS(param15)},
	{WR_CMND,	0x21, 0},
    {SLEEP_MS,  0, 1},


    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,  100},

 
    {CTRL_END        , 0}
};

Lcd_init_t  LCD_INFO_ALIAS(enter_sleep_seq)[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
    	{DISPCTRL_SLEEP_MS,  0,  100},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
        {CTRL_END        , 0}
};

Lcd_init_t  LCD_INFO_ALIAS(exit_sleep_seq)[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
    	{DISPCTRL_SLEEP_MS,  0,  100},  
       {CTRL_END        , 0}
};

static lcd_dsi_drv_info_t lcd_nt35510_4014tn = {	
		LCD_INFO_ALIAS(LCD_device), \
		&LCD_INFO_ALIAS(dsiVcCmCfg),\
		&LCD_INFO_ALIAS(dsiCfg),\
		LCD_INFO_ALIAS(power_on_seq),\
		LCD_INFO_ALIAS(enter_sleep_seq), \
		LCD_INFO_ALIAS(exit_sleep_seq), \
		&LCD_INFO_ALIAS(LCD_panel_name),\
};

#undef LCD_INFO_ALIAS

#endif
