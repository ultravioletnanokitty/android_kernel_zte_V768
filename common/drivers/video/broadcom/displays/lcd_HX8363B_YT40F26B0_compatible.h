/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_hx8363b.h
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

#ifndef __LCD_HX8363B_YT40F26B0_H__
#define __LCD_HX8363B_YT40F26B0_H__

#include "lcd_dsi_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif
#define LCD_INFO_ALIAS(var)	hx8363b_yt40f26b0##_##var

#define DSI_VC            (0)

#define LCD_HEIGHT		800
#define LCD_WIDTH		480

#define LCD_BITS_PER_PIXEL 32
const char LCD_INFO_ALIAS(LCD_panel_name)[] = "HX8363B LCD";

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
//    CSL_DSI_TE_MODE_VSYNC_HSYNC,
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
//        DSI_TE_NONE,
//        NULL
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

static uint8_t LCD_INFO_ALIAS(param1)[] = {0xFF, 0x83, 0x63};

static uint8_t LCD_INFO_ALIAS(param2)[] = { /*0x78,
    	 0x00,
    	 0x44,
    	 0x07,
    	 0x01,
    	 0x0E,
	 0x0E,
	 0x21,
	 0x29,
	 0x3F,
	 0x3F,
	 0x40,
	 0x32,
	 0x00,
	 0xE6,
	 0xE6,
	 0xE6,
	 0xE6,
	 0xE6,			*/
0x78,0x00,0x44,0x07,0x01,
0x12,0x12,0x21,0x29,0x3F,
0x3F,0x40,0x32,0x00,0xE6,
0xE6,0xE6,0xE6,0xE6
    	 };

static uint8_t LCD_INFO_ALIAS(param3)[] = {0x08, 0x00}; //0x08, 0x00
/*
static uint8_t param4[] =  {0x02, 0x18,
    	 0x9C, 0x08,
    	 0x18, 0x04,
    	 0x72};
*/
static uint8_t LCD_INFO_ALIAS(param4)[] =  {/*0x02, 0x18, //0xA0, 0x0f
    	 0x9C, 0x08,//0x82, 0x03
    	 0x18, 0x04,//0x0C,0x20
    	 0x87 */
0x12,0x18,0x9C,0x08,0x18,
0x04,0x72
};
static uint8_t LCD_INFO_ALIAS(param5)[] =   {0x05, 0x60, 0x00, 0x10};

static uint8_t LCD_INFO_ALIAS(param6)[] =  {/*0x00 
				,0x06 
				,0x0A 
				,0x12 
				,0x15 
				,0x3B //3C
				,0x1D //1F
				,0x34 //30
				,0x87 
				,0x8E 
				,0xCC //D1
				,0xCF //D6
				,0xCE //D8
				,0x0E //15
				,0x12 //18
				,0x11 //14
				,0x18 //18
				,0x00 
				,0x06 
				,0x0A 
				,0x12 
				,0x15 
				,0x3B //3C
				,0x1D //1F
				,0x34 //30
				,0x87 
				,0x8E 
				,0xCC //D1
				,0xCF //D6
				,0xCE //D8
				,0x0E //15
				,0x12 //18
				,0x11 //14
				,0x18 //18
*/
0x00,0x02,0x02,0x19,0x1C,
0x2F,0x1B,0x36,0x89,0x8E,0xCF,
0xD2,0xD3,0x12,0x14,0x12,0x19,
0x00,0x02,0x02,0x19,0x1C,0x2F,
0x1B,0x36,0x89,0x8E,0xCF,0xD2,
0xD3,0x12,0x14,0x12,0x19
 	 };

static uint8_t LCD_INFO_ALIAS(param7)[] =   {0x10, 0x00, 0x56, 0xC6,0x10,
                             0x89, 0xFF, 0x0F, 0x33,0x6E,
                             0x04, 0x07, 0x9A, 0x10};

static uint8_t param8[] =   {0x01, 0x07}; //0x09
static uint8_t LCD_INFO_ALIAS(param10)[] = {0x00, 0x80};
Lcd_init_t LCD_INFO_ALIAS(power_on_seq)[] = {
    {WR_CMND,   0x11, 0},//Sleep Out      
    {SLEEP_MS,  0,  1400},
    {WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3,  LCD_INFO_ALIAS(param1)},// SET password
    {SLEEP_MS,  0, 1},
  
    {WR_CMND_MULTIPLE_DATA     ,    0xB1    , 0     ,19, //SET Power
    LCD_INFO_ALIAS(param2)},
    	
    {SLEEP_MS,  0, 1},
    {WR_CMND_MULTIPLE_DATA     ,    0xB2    , 0     ,2,  LCD_INFO_ALIAS(param3)}, // SET DISP	  
    {SLEEP_MS,  0, 1},
    {WR_CMND_MULTIPLE_DATA     ,    0xB4    , 0     ,7, // SET CYC
     LCD_INFO_ALIAS(param4)}, 	      
    {SLEEP_MS,  0, 1}, 
    {WR_CMND_MULTIPLE_DATA     ,    0xBF    , 0     ,4, // SET VCOM for Max=-2.5V
     LCD_INFO_ALIAS(param5)}, 	     
    {SLEEP_MS,  0, 1},    
    {WR_CMND_DATA,	 0xB6, 	0x30}, // SET VCOM  0x4E
    {SLEEP_MS,  0, 1},
    {WR_CMND_DATA,	 0xCC, 	0x0B}, //Set Panel  0x03
    {SLEEP_MS,  0, 1},

    {WR_CMND_MULTIPLE_DATA     ,    0xE0    , 0     ,34,  LCD_INFO_ALIAS(param6)},
    {SLEEP_MS,  0, 1},   	 

/* yaogangxiang@wind-mobi.com 2012.02.28 begin */
    {WR_CMND_MULTIPLE_DATA,  0xB0,  0,        2,     param8},
    {SLEEP_MS,      0, 10},
/* yaogangxiang@wind-mobi.com 2012.02.28 end */

    {WR_CMND_MULTIPLE_DATA,	 0xBA, 	0,    14,    LCD_INFO_ALIAS(param7)}, 
    {SLEEP_MS,  0, 10},   	 
    
    {WR_CMND_DATA,	 0xC2, 	0x04}, 
    {SLEEP_MS,  0,  10},
    
    {WR_CMND_DATA,	 0x3A, 	0x77}, 
    {SLEEP_MS,  0, 1},
    
/* yaogangxiang@wind-mobi.com 2012.03.23 begin */
    {WR_CMND_DATA, 0x35, 0x00},//TE ON
//    {WR_CMND_MULTIPLE_DATA,	 0x44, 	0, 2, LCD_INFO_ALIAS(param10)}, 
//    {SLEEP_MS,  0, 10},   	  
/* yaogangxiang@wind-mobi.com 2012.03.23 end */
    
    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,500},
        
    //--- END OF COMMAND LIST -----------------------

    {WR_CMND,   0x2C, 0},//Sleep Out      
    {SLEEP_MS,  0,  10},

    {CTRL_END        , 0}
};

Lcd_init_t  LCD_INFO_ALIAS(enter_sleep_seq)[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
       {CTRL_END        , 0}
};

Lcd_init_t  LCD_INFO_ALIAS(exit_sleep_seq)[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
       {CTRL_END        , 0}
};

static lcd_dsi_drv_info_t lcd_hx8363b_yt40f26b0 = {	
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
