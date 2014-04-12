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
*    This is the LCD-specific code for a hx8357c module.the panel is TRULY.ltd
*
*****************************************************************************/

#ifndef __LCD_HX8357C_TFT8K7692_H__
#define __LCD_HX8357C_TFT8K7692_H__

#include "lcd_dsi_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif

#define LCD_INFO_ALIAS(var)	hx8357c_tft8k7692##_##var

#define DSI_VC            (0)
//#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS mode


#define LCD_WIDTH 320
#define LCD_HEIGHT 480


#define LCD_BITS_PER_PIXEL 32



static const char LCD_INFO_ALIAS(LCD_panel_name)[]  = "HX8357c TFT8k7692 LCD";

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
	  {200,1},       // hsBitClk        PLL     300[MHz], DIV by 1 = 300[Mbps]
    // LP Speed
    5,             // lpBitRate_Mbps, Max 10[Mbps]
    FALSE,         // enaContClock 
    TRUE,          // enaRxCrc                
    TRUE,          // enaRxEcc               
    TRUE,          // enaHsTxEotPkt           
    FALSE,         // enaLpTxEotPkt        
    FALSE,         // enaLpRxEotPkt        
};    

static uint8_t LCD_INFO_ALIAS(param1)[] = {0xFF, 0x83, 0x57};

static uint8_t LCD_INFO_ALIAS(param2)[] =  {0x00, 0xF8}; //0xF8

static uint8_t LCD_INFO_ALIAS(param3)[] = {0x00,//DP
    	 0x11,//BT
    	 0x1C,//VSPR
    	 0x1C,//VSNR
    	 0x83,//AP
    	 0xaa};//FS

static uint8_t LCD_INFO_ALIAS(param4)[] =  {0x50,//OPON
    	 0x50,//OPON
    	 0x01,//STBA
    	 0x3C,//STBA
    	 0x1e,//STBA
    	 0x08};

static uint8_t LCD_INFO_ALIAS(param5)[] =  {0x02,//Nw
    	 0x40,//RTN
    	 0x00,//DIV
    	 0x2A,//DUM
    	 0x2A,//DUM
    	 0x0D,//GDON
    	 0x78,//GDOFF
    	 };  	

static uint8_t LCD_INFO_ALIAS(param6)[] =  {//GAMMA
   
	0x00, 0x00, 0x02, 0x00, 0x1a, //V1  
	0x00, 0x23, 0x00, 0x2B, 0x00, //  
	0x2f, 0x00, 0x41, 0x00, 0x4b, //20  
	0x00, 0x53, 0x00, 0x46, 0x00,   
	0x40, 0x00, 0x3b, 0x00, 0x31, //57  
	0x00, 0x2f, 0x00, 0x26, 0x00, //  
	0x26, 0x00, 0x03, 0x01, 0x02, //v0  
	0x00, 0x1a, 0x00, 0x23, 0x00,   
	0x2b, 0x00, 0x2f, 0x00, 0x41, //V13  
	0x00, 0x4b, 0x00, 0x53, 0x00, //  
	0x46, 0x00, 0x40, 0x00, 0x3b, //50  
	0x00, 0x31, 0x00, 0x2f, 0x00, //  
	0x26, 0x00, 0x26, 0x00, 0x03, //63  
	0x00, 0x44, //CGPN[1:0]  

	};  

static uint8_t LCD_INFO_ALIAS(param9)[] =  { 0x00,0x00
};

//0xBA
static uint8_t LCD_INFO_ALIAS(param7)[] =  {
  0x00,0x56,0xD4,0x00,0x0A,
  0x00,0x10,0x32,0x6E,0x04,
  0x05,0x9A,0x14,0x19,0x10,
  0x40,
    	 }; 

static Lcd_init_t LCD_INFO_ALIAS(power_on_seq)[] = {
    {WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3,  LCD_INFO_ALIAS(param1)},// SET password
    {SLEEP_MS,  0, 10},
    
    {WR_CMND_DATA,	 0xCC,	0x05}, //Set Panel
    
    {WR_CMND_DATA,   0x35, 0},//TE ON
    {WR_CMND_MULTIPLE_DATA     ,    0x44    , 0     ,2,  LCD_INFO_ALIAS(param9)},//TE SCAN_LINE

    {WR_CMND_DATA,   0xB6, 0x34},//0x40
      
    {WR_CMND_MULTIPLE_DATA     ,    0xB1    , 0     ,6, 
     LCD_INFO_ALIAS(param3)},//FS
    	 
    {WR_CMND_MULTIPLE_DATA     ,    0xB4    , 0     ,7, 
     LCD_INFO_ALIAS(param5)},

    {WR_CMND_MULTIPLE_DATA     ,    0xC0    , 0     ,6, 
      LCD_INFO_ALIAS(param4)},

    {WR_CMND_MULTIPLE_DATA     ,    0xC6    , 0     ,2,  LCD_INFO_ALIAS(param2)},
    {SLEEP_MS,  0, 5},

	{WR_CMND_MULTIPLE_DATA     ,    0xBA    , 0     ,16,  LCD_INFO_ALIAS(param7)},
    
	 
    {WR_CMND_MULTIPLE_DATA     ,    0xE0    , 0     ,67,   	 
     LCD_INFO_ALIAS(param6)},
    {SLEEP_MS,  0,  20},
    
    {WR_CMND_DATA,   0xB0, 0x68},// 70 hz
    {SLEEP_MS,  0,  1},
    {WR_CMND_DATA,   0x3A, 0x06}, //0x07
    {SLEEP_MS,  0,  5},
    {WR_CMND,   0x11, 0},//Sleep Out      
    {SLEEP_MS,  0,  300},
   	 
    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,  15},  
    
    {WR_CMND,   0x2C, 0},  
    //--- END OF COMMAND LIST -----------------------
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

static lcd_dsi_drv_info_t lcd_hx8357c_tft8k7692 = {	LCD_INFO_ALIAS(LCD_device), \
		&LCD_INFO_ALIAS(dsiVcCmCfg),\
		&LCD_INFO_ALIAS(dsiCfg),\
		LCD_INFO_ALIAS(power_on_seq),\
		LCD_INFO_ALIAS(enter_sleep_seq), \
		LCD_INFO_ALIAS(exit_sleep_seq), \
		&LCD_INFO_ALIAS(LCD_panel_name),\
};



#undef LCD_INFO_ALIAS

#endif
