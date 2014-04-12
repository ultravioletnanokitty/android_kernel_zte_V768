/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_HX8369A_YT40F25J0.h
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
*  dsi_hx8363b.h
*
*  PURPOSE:
*    This is the LCD-specific code for a hx8363b module.
*
*****************************************************************************/

#ifndef __LCD_HX8369A_YT40F25J0_COMPATIBLE_H__
#define __LCD_HX8369A_YT40F25J0_COMPATIBLE_H__

#include "lcd_dsi_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif
#define LCD_INFO_ALIAS(var)	hx8369a_yt40f25j0##_##var

#define DSI_VC            (0)

#define LCD_HEIGHT		800
#define LCD_WIDTH		480

#define LCD_BITS_PER_PIXEL 32

const char LCD_INFO_ALIAS(LCD_panel_name)[] = "HX8369A_YT40F25J0 LCD";

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
    {400,1},       // hsBitClk        PLL     300[MHz], DIV by 1 = 300[Mbps]
    // LP Speed
    5,             // lpBitRate_Mbps, Max 10[Mbps]
    FALSE,         // enaContClock 
    TRUE,          // enaRxCrc                
    TRUE,          // enaRxEcc               
    TRUE,          // enaHsTxEotPkt           
    FALSE,         // enaLpTxEotPkt        
    FALSE,         // enaLpRxEotPkt        
};    

static uint8_t LCD_INFO_ALIAS(param1)[] = {0xFF, 0x83, 0x69};
static uint8_t LCD_INFO_ALIAS(param8)[] = {0x01, 0x0B};

static uint8_t LCD_INFO_ALIAS(param2)[] = {
	     0x01,
    	 0x00,
    	 0x34,
    	 0x07,
    	 0x00,
    	 0x0F,
		 0x0F,
		 0x21,
		 0x28,
		 0x3F,
		 0x3F,
		 0x07,
		 0x23,
		 0x01,
		 0xE6,
		 0xE6,
		 0xE6,
		 0xE6,
		 0xE6,			
    	 };

static uint8_t LCD_INFO_ALIAS(param3)[] = {0x00, 0x20,0x0A, 0x0A,0x70, 0x00,0xFF, 0x00,
							0x00, 0x00,0x00, 0x03,0x03, 0x00,0x01};
/*
static uint8_t param4[] =  {0x02, 0x18,
    	 0x9C, 0x08,
    	 0x18, 0x04,
    	 0x72};
*/
static uint8_t LCD_INFO_ALIAS(param4)[] = {0x00, 0x18, 0x80, 0x10,0x01};
static uint8_t LCD_INFO_ALIAS(param5)[] = {0x20, 0x20};
static uint8_t LCD_INFO_ALIAS(param9)[] = { 0x00, 0x05,0x03, 0x00,0x01, 0x09,0x10, 0x80,
							0x37, 0x37,0x20, 0x31,0x46, 0x8A,0x57, 0x9B,
							0x20, 0x31,0x46, 0x8A,0x57, 0x9B,0x07, 0x0F,
							0x07, 0x00};

static uint8_t LCD_INFO_ALIAS(param6)[] =  {	 0x00 
				,0x06 
				,0x06 
				,0x29 
				,0x2D 
				,0x3F //3C
				,0x13 //1F
				,0x32 //30
				,0x08 
				,0x0C 
				,0x0D //D1
				,0x11 //D6
				,0x14 //D8
				,0x11 //15
				,0x14 //18
				,0x0E //18
				,0x15 
				,0x00 
				,0x06 
				,0x06 
				,0x29 
				,0x2D //3C
				,0x3F //1F
				,0x13 //30
				,0x32 
				,0x08 
				,0x0C //D1
				,0x0D //D6
				,0x11 //D8
				,0x14 //15
				,0x11 //18
				,0x14 //14
				,0x11 //18
				,0x18 //18
    	 };

static uint8_t LCD_INFO_ALIAS(param7)[] =   {0x00, 0xA0, 0xC6, 0x00,0x0A,
                             0x00, 0x10, 0x30, 0x6C,0x02,
                             0x10, 0x18, 0x40};

Lcd_init_t LCD_INFO_ALIAS(power_on_seq)[] = {
    {WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3,  LCD_INFO_ALIAS(param1)},// SET password
    {SLEEP_MS,  0, 1},

    //{WR_CMND_MULTIPLE_DATA,0xB0, 0,2, param8},
	//{SLEEP_MS,	0, 1},

    {WR_CMND_MULTIPLE_DATA,0xB1, 0,19, LCD_INFO_ALIAS(param2)},
    {SLEEP_MS,  0, 10},
    
    {WR_CMND_MULTIPLE_DATA, 0xB2, 0,15, LCD_INFO_ALIAS(param3)}, // SET DISP	  
    {SLEEP_MS,  0, 1},
    
    {WR_CMND_MULTIPLE_DATA, 0xB4, 0,5, LCD_INFO_ALIAS(param4)}, 	      
    {SLEEP_MS,  0, 1}, 


    {WR_CMND_MULTIPLE_DATA, 0xB6, 0,2, LCD_INFO_ALIAS(param5)}, 	      
    {SLEEP_MS,  0, 1}, 

    {WR_CMND_MULTIPLE_DATA, 0xD5, 0,26, LCD_INFO_ALIAS(param9)}, 	      
    {SLEEP_MS,  0, 1}, 
	
    {WR_CMND_MULTIPLE_DATA, 0xE0, 0 ,34, LCD_INFO_ALIAS(param6)},
    {SLEEP_MS,  0, 1},   	 

    {WR_CMND_DATA,	 0x3A, 	0x77}, 
    {SLEEP_MS,  0, 1},

    {WR_CMND_DATA,	 0x35, 	0x00}, 
    {SLEEP_MS,  0, 1},

    {WR_CMND_MULTIPLE_DATA,	 0xBA, 	0,    13,    LCD_INFO_ALIAS(param7)}, 
    {SLEEP_MS,  0, 1},   	 
    
    //{WR_CMND_DATA,	 0x51, 	0x0F}, 
    //{SLEEP_MS,  0,  1},
    
    //{WR_CMND_DATA,	 0x53, 	0x24}, 
    //{SLEEP_MS,  0,  1},

    //{WR_CMND_DATA,	 0x55, 	0x00}, 
    //{SLEEP_MS,  0,  1},

    {WR_CMND,   0x11, 0},//Sleep Out      
    {SLEEP_MS,  0,  400}, 
    
    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,  20},

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

static lcd_dsi_drv_info_t lcd_hx8369a_yt40f25j0 = {	
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
