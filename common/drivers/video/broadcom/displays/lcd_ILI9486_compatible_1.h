/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/dsiplays/lcd_HX8357C_GZ3513_compatible.h
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
*  dsi_hx8357c.h
*
*  PURPOSE:
*    This is the LCD-specific code for a hx8357c module.
*
*****************************************************************************/

#ifndef __LCD_ILI94861_H__
#define __LCD_ILI94861_H__

#include "lcd_dsi_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif

#define LCD_INFO_ALIAS(var)	ili94861##_##var

#define DSI_VC            (0)
//#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS mode


#define LCD_WIDTH 320
#define LCD_HEIGHT 480


#define LCD_BITS_PER_PIXEL 32



static const char LCD_INFO_ALIAS(LCD_panel_name)[]  = "ili9486 LCD";

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
static CSL_DSI_TE_IN_CFG_t LCD_INFO_ALIAS(teInCfg) = 
{
    CSL_DSI_TE_MODE_VSYNC,      // te mode 
    CSL_DSI_TE_ACT_POL_LO,      // sync polarity
    0,                          // vsync_width [tectl_clk_count]
    0,                          // hsync_line
};    


// DSI Command Mode VC Configuration
static CSL_DSI_CM_VC_t LCD_INFO_ALIAS(dsiVcCmCfg) = 
{
    DSI_VC,               			// VC
    DSI_DT_LG_DCS_WR,               // dsiCmnd       
    MIPI_DCS_WRITE_MEMORY_START,    // dcsCmndStart       
    MIPI_DCS_WRITE_MEMORY_CONTINUE, // dcsCmndCont  
    FALSE,                          // isLP 
    LCD_IF_CM_I_RGB888U,            // cm_in         
    LCD_IF_CM_O_RGB666,             // cm_out        
    // TE configuration
    {
        DSI_TE_CTRLR_INPUT_0,
        &LCD_INFO_ALIAS(teInCfg)    // DSI Te Input Config
    },
 };

// DSI BUS CONFIGURATION
static CSL_DSI_CFG_t LCD_INFO_ALIAS(dsiCfg) = {
    0,             // bus             set on open
    1,             // dlCount
    DSI_DPHY_0_92, // DSI_DPHY_SPEC_T
    // ESC CLK Config
    {156,2},       // escClk          fixed   156[MHz], DIV by 2 =  78[MHz] 

    // HS CLK Config
	  {300,1},       // hsBitClk        PLL     300[MHz], DIV by 1 = 300[Mbps]
    // LP Speed
    5,             // lpBitRate_Mbps, Max 10[Mbps]
    FALSE,         // enaContClock 
    TRUE,          // enaRxCrc                
    TRUE,          // enaRxEcc               
    TRUE,          // enaHsTxEotPkt           
    FALSE,         // enaLpTxEotPkt        
    FALSE,         // enaLpRxEotPkt        
};    



static uint8_t LCD_INFO_ALIAS(param1)[] = { //0xF1
    0x36, 0x04, 0x00, 0x3C, 0x0F, 0x8F
};

static uint8_t LCD_INFO_ALIAS(param2)[] = { //0xF2
    0x18, 0xA3, 0x12, 0x02, 
    0xB2, 0x12, 0xFF, 0x10, 
    0x00
};

static uint8_t LCD_INFO_ALIAS(param3)[] = { //0xF8
    0x21, 0x04
};

static uint8_t LCD_INFO_ALIAS(param5)[] = { //0xF9
    0x00, 0x08 
};

static uint8_t LCD_INFO_ALIAS(param10)[] = {  //0xF7
    0xA9, 0x91, 0x2D, 0x8A, 0x4C
};

static uint8_t LCD_INFO_ALIAS(param16)[] = { //0xC5
    0x00, 0x18 //0x15
};

static uint8_t LCD_INFO_ALIAS(param17)[] = {
    0xB0, 0x11
};

static uint8_t LCD_INFO_ALIAS(param19)[] = {
#if 1
#if 1
    0x0F, 0x1F, 0x1A,
    0x0B, 0x0E, 0x09, 0x48, 
    0x99, 0x3B, 0x0A, 0x14,
    0x06, 0x15, 0x09, 0x00
#else
    0x00, 0x09, 0x15, 0x06,
    0x14, 0x0A, 0x3B, 0x99,
    0x48, 0x09, 0x0E, 0x0B,
    0x1A, 0x1F, 0x0F
#endif
#else
    0x0F, 0x1E, 0x1E, 0x05,
    0x12, 0x07, 0x4B, 0x98,
    0x3F, 0x09, 0x17, 0x04,
    0x0E, 0x0F, 0x00
#endif
};

static uint8_t LCD_INFO_ALIAS(param20)[] = {
#if 1
#if 1
    0x0F, 0x36, 0x32,
    0x09, 0x0A, 0x04, 0x49, 
    0x66, 0x37, 0x06, 0x10,
    0x04, 0x22, 0x20, 0x00
#else
    0x00, 0x20, 0x22, 0x04, 
    0x10, 0x06, 0x37, 0x66,
    0x49, 0x04, 0x0A, 0x09,
    0x32, 0x36, 0x0F
#endif
#else
    0x0F, 0x30, 0x31, 0x0B,
    0x08, 0x06, 0x48, 0x76,
    0x31, 0x08, 0x0D, 0x04,
    0x1E, 0x21, 0x00
#endif
};

static uint8_t LCD_INFO_ALIAS(param21)[] = {
    0x02, 0x42, 0x3B
};
static uint8_t LCD_INFO_ALIAS(param7)[] = {
    0x00, 0x00, 0x01, 0x3F
};

static uint8_t LCD_INFO_ALIAS(param4)[] = {
    0x00, 0x00, 0x01, 0xDF
};

static uint8_t LCD_INFO_ALIAS(param9)[] = {
    0x01, 0x01
};

static Lcd_init_t LCD_INFO_ALIAS(power_on_seq)[] = {
{WR_CMND_MULTIPLE_DATA     ,    0xF7    , 0     ,5,  LCD_INFO_ALIAS(param10)},	
{SLEEP_MS,  0,  5},
	
#if 1 
{WR_CMND_MULTIPLE_DATA     ,    0xF1    , 0     ,6,  LCD_INFO_ALIAS(param1)},
{WR_CMND_MULTIPLE_DATA     ,    0xF2    , 0     ,9,  LCD_INFO_ALIAS(param2)},
{WR_CMND_MULTIPLE_DATA     ,    0xF8    , 0     ,2,  LCD_INFO_ALIAS(param3)},
{WR_CMND_MULTIPLE_DATA     ,    0xF9    , 0     ,2,  LCD_INFO_ALIAS(param5)},  

{WR_CMND_MULTIPLE_DATA     ,    0xE0    , 0     ,15,  LCD_INFO_ALIAS(param19)}, 
{WR_CMND_MULTIPLE_DATA     ,    0xE1    , 0     ,15,  LCD_INFO_ALIAS(param20)}, 

{WR_CMND_MULTIPLE_DATA     ,    0xC5    , 0     ,2,  LCD_INFO_ALIAS(param16)}, 
{WR_CMND_DATA,   0xB4, 0x02},  
{WR_CMND_DATA,   0x36, 0x08},  
{WR_CMND_DATA,   0x3A, 0x66},
 
{WR_CMND_MULTIPLE_DATA     ,    0xB6    , 0     ,3,  LCD_INFO_ALIAS(param21)}, 




{WR_CMND_MULTIPLE_DATA     ,    0xB1    , 0     ,2,  LCD_INFO_ALIAS(param17)}, 

{WR_CMND_MULTIPLE_DATA     ,    0xC0    , 0     ,2,  LCD_INFO_ALIAS(param9)}, 
{WR_CMND_DATA,   0xC1, 0x41},
{WR_CMND_DATA,   0x35, 0x00},
#else
{WR_CMND_MULTIPLE_DATA     ,    0xB6    , 0     ,3,  LCD_INFO_ALIAS(param21)}, 
{WR_CMND_DATA,   0x36, 0x00}, 
{WR_CMND_DATA,	 0x3A, 0x66},
#endif

{WR_CMND,   0x11, 0},
{SLEEP_MS,  0, 200},      
{WR_CMND,   0x29, 0},
//{SLEEP_MS,  0,  10},  

{WR_CMND,	0x2C, 0},
{CTRL_END        , 0}
};


static Lcd_init_t  LCD_INFO_ALIAS(enter_sleep_seq)[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{CTRL_END        , 0}
};

static Lcd_init_t  LCD_INFO_ALIAS(exit_sleep_seq)[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{CTRL_END        , 0}
};

static lcd_dsi_drv_info_t lcd_ili9486_1 = {	LCD_INFO_ALIAS(LCD_device), \
		&LCD_INFO_ALIAS(dsiVcCmCfg),\
		&LCD_INFO_ALIAS(dsiCfg),\
		LCD_INFO_ALIAS(power_on_seq),\
		LCD_INFO_ALIAS(enter_sleep_seq), \
		LCD_INFO_ALIAS(exit_sleep_seq), \
		&LCD_INFO_ALIAS(LCD_panel_name),\
};



#undef LCD_INFO_ALIAS

#endif
