/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/dsi/lcd/panel/dsi_hx8363b.h
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

#ifndef __DSI_HX8363B_H__
#define __DSI_HX8363B_H__

#define DSI_VC            (0)
#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS mode

#define PANEL_WIDTH 480
#define PANEL_HEIGHT 800


#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 800
#define INPUT_BPP 4
#define RESET_GPIO 45

// Display Info
DISPDRV_INFO_T DSI_Display_Info =
{
    DISPLAY_TYPE_LCD_STD,         // DISPLAY_TYPE_T          type;          
    PANEL_WIDTH,                  // UInt32                  width;         
    PANEL_HEIGHT,                 // UInt32                  height;        
    DISPDRV_FB_FORMAT_RGB888_U,   // DISPDRV_FB_FORMAT_T     input_format;
    DISPLAY_BUS_DSI,              // DISPLAY_BUS_T           bus_type;
    0,                            // UInt32                  interlaced;    
    DISPDRV_DITHER_NONE,          // DISPDRV_DITHER_T        output_dither; 
    0,                            // UInt32                  pixel_freq;    
    0,                            // UInt32                  line_rate;     
};

//TE Info
CSL_DSI_TE_IN_CFG_t teInCfg = 
{
    CSL_DSI_TE_MODE_VSYNC,      // te mode 
    CSL_DSI_TE_ACT_POL_LO,      // sync polarity
    0,                          // vsync_width [tectl_clk_count]
    0,                          // hsync_line
};    


// DSI Command Mode VC Configuration
CSL_DSI_CM_VC_t alexVcCmCfg = 
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
        //DSI_TE_CTRLR_INPUT_0,
        //&teInCfg                    // DSI Te Input Config
        DSI_TE_NONE,
        NULL
    },
 };

// DSI BUS CONFIGURATION
CSL_DSI_CFG_t dsiCfg = {
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


DISPCTRL_REC_T LCD_Init[] = {
    {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3, {0xFF, 0x83, 0x63}},// SET password
    {DISPCTRL_SLEEP_MS,  0, 1},
  
   {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xB1    , 0     ,19, //SET Power
    	{0x78,
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
	 0xE6,			
    	 }},
    {DISPCTRL_SLEEP_MS,  0, 1},
    {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xB2    , 0     ,2, {0x08, 0x00}}, // SET DISP	  
    {DISPCTRL_SLEEP_MS,  0, 1},
    {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xB4    , 0     ,7, // SET CYC
    	{0x02, 0x18,
    	 0x9C, 0x08,
    	 0x18, 0x04,
    	 0x72}}, 	      
    {DISPCTRL_SLEEP_MS,  0, 1}, 
    {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xBF    , 0     ,4, // SET VCOM for Max=-2.5V
    	{0x05, 0x60,
    	 0x00, 0x10}}, 	     
     {DISPCTRL_SLEEP_MS,  0, 1},    
    {DISPCTRL_WR_CMND_DATA,	 0xB6, 	0x40}, // SET VCOM  
    {DISPCTRL_SLEEP_MS,  0, 1},
    {DISPCTRL_WR_CMND_DATA,	 0xCC, 	0x03}, //Set Panel
    {DISPCTRL_SLEEP_MS,  0, 1},

     {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xE0    , 0     ,34,   	 
 			{	 0x00 
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
    	 }},
    {DISPCTRL_SLEEP_MS,  0, 1},   	 

    {DISPCTRL_WR_CMND_DATA,	 0xBA, 	0x10}, 
    
    {DISPCTRL_WR_CMND_DATA,	 0xC2, 	0x04}, 
    
    {DISPCTRL_WR_CMND_DATA,	 0x3A, 	0x77}, 
    //{DISPCTRL_SLEEP_MS,  0, 1},
    
    
    //{DISPCTRL_WR_CMND,   0x35, 0},//TE ON

/*
    {DISPCTRL_WR_CMND_DATA,	 0x51, 	0x03}, 
    {DISPCTRL_WR_CMND_DATA,	 0x53, 	0x24}, 
    {DISPCTRL_WR_CMND_DATA,	 0x55, 	0x00}, 
    {DISPCTRL_WR_CMND_DATA,	 0x35, 	0x00}, 
    {DISPCTRL_WR_CMND_DATA,	 0xC9, 	0x05}, 
    {DISPCTRL_WR_CMND_DATA,	 0x36, 	0x02}, 
 */
 
    {DISPCTRL_WR_CMND,   0x11, 0},//Sleep Out      
    {DISPCTRL_SLEEP_MS,  0,  150},
    
    {DISPCTRL_WR_CMND,   0x29, 0},// display on    
    {DISPCTRL_SLEEP_MS,  0,  100},
        
   // {DISPCTRL_WR_CMND_DATA,	 0xB0, 	0x68}, //70Hz      
    //--- END OF COMMAND LIST -----------------------

    {DISPCTRL_WR_CMND,   0x2C, 0},//Sleep Out      
    {DISPCTRL_SLEEP_MS,  0,  10},

    {DISPCTRL_WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3, {0xFF, 0x83, 0x00}},// SET password

    {DISPCTRL_LIST_END        , 0}
};

DISPCTRL_REC_T LCD_EnterSleep[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
	{DISPCTRL_LIST_END		  , 0}
};

DISPCTRL_REC_T LCD_ExitSleep[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
	{DISPCTRL_LIST_END		  , 0}
};

#endif
