/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/dsi/lcd/panel/dsi_hx8357c.h
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

#ifndef __LCD_HX8357C_H__
#define __LCD_HX8357C_H__

#define DSI_VC            (0)
#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS mode

#define LCD_WIDTH 320
#define LCD_HEIGHT 480


#define PANEL_WIDTH 320
#define PANEL_HEIGHT 480

#define LCD_BITS_PER_PIXEL 32
#define INPUT_BPP 4
#define RESET_GPIO 45
#define DSI_VC             (0)
#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS m

#define LCD_CMD(x) (x)
#define LCD_DATA(x) (x)

const char *LCD_panel_name = "HX8357c LCD";

int LCD_num_panels = 1;
LCD_Intf_t LCD_Intf = LCD_DSI;
LCD_Bus_t LCD_Bus = LCD_32BIT;


LCD_dev_info_t LCD_device[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= LCD_HEIGHT,
	 .width		= LCD_WIDTH,
	 .bits_per_pixel= LCD_BITS_PER_PIXEL,
	 .te_supported	= true,
	}
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
CSL_DSI_CM_VC_t dsiVcCmCfg = 
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
        &teInCfg                    // DSI Te Input Config
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

static uint8_t param1[] = {0xFF, 0x83, 0x57};

static uint8_t param2[] =  {0x00, 0xF8};

static uint8_t param3[] = {0x00,//DP
    	 0x11,//BT
    	 0x1E,//VSPR
    	 0x1E,//VSNR
    	 0x83,//AP
    	 0x48};//FS

static uint8_t param4[] =  {0x50,//OPON
    	 0x50,//OPON
    	 0x01,//STBA
    	 0x63,//STBA
    	 0x50,//STBA
    	 0x08};

static uint8_t param5[] =  {0x02,//Nw
    	 0x40,//RTN
    	 0x00,//DIV
    	 0x2A,//DUM
    	 0x2A,//DUM
    	 0x0D,//GDON
    	 0x78,//GDOFF
    	 };  	

static uint8_t param6[] =  {//GAMMA
			 0x00, //
			 0x00, //
	
			 0x20, //V0
			 0x00, //
			 
			 0x2B, //V1
			 0x00, //
			 
			 0x31, //V2
			 0x00, 
			 
			 0x3A, //V4
			 0x00, //
			 
			 0x41, //V6
			 0x00, //
			 
			 0x4F, //V13
			 0x00, //
			 
			 0x57, //20
			 0x00, //
			 
			 0x5E, //V27
			 0x00, //
			 
			 0x3C, //36
			 0x00, //
			 
			 0x37, //43
			 0x00, //
			 
			 0x33, //50
			 0x00, //
			 
			 0x2B, //57
			 0x00, //
			 
			 0x29, //59
			 0x00, //
			 
			 0x23, //61
			 0x00, //
			 
			 0x22, //62
			 0x00, //
			 
			 0x1B, //63
			 0x01, //
			 
			 0x20, //v0
			 0x00, //
			 
			 0x2B, //V1
			 0x00, //
			 
			 0x31, //V2
			 0x00, //
			 
			 0x3A, //V4
			 0x00, //
			 
			 0x41, //V6
			 0x00, //
			 
			 0x4F, //V13
			 0x00, //
			 
			 0x57, //V20
			 0x00, //
			 
			 0x5E, //27
			 0x00, //
			 
			 0x3C, //V36
			 0x00, //
			 
			 0x37, //43
			 0x00, //
			 
			 0x33, //50
			 0x00, //
			 
			 0x2B, //57
			 0x00, //
			 
			 0x29, //59
			 0x00, //
			 
			 0x23, //61
			 0x00, //
			 
			 0x22, //62
			 0x00, //
			 
			 0x1B, //63
			 0x00, //
			 0x00, //
			// 0x00, //
			// 0x01, //34
    	 };

Lcd_init_t power_on_seq[] = {
    {WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3,  param1},// SET password
    {SLEEP_MS,  0, 10},
    
   // {WR_CMND_DATA,	 0xB6, 	0x53}, //VCOMDC  
	{WR_CMND_DATA,	 0xCC,	0x05}, //Set Panel

    //{DISPCTRL_WR_CMND,   0x11, 0},//Sleep Out      
    //{DISPCTRL_SLEEP_MS,  0,  150},
    
    {WR_CMND_DATA,   0x35, 0},//TE ON

    {WR_CMND_DATA,   0xB6, 0x40},// 50
   /* 
    {WR_CMND_MULTIPLE_DATA     ,    0x44    , 0     ,2,  param2},//TE SCAN_LINE
    
    {WR_CMND_DATA,	 0x3A, 	0x06}, //262K
  
    {WR_CMND_DATA,	 0xB0, 	0x66}, //60Hz      
  
    {WR_CMND_DATA,	 0xCC, 	0x05}, //Set Panel
  */    
    {WR_CMND_MULTIPLE_DATA     ,    0xB1    , 0     ,6, 
     param3},//FS
    	 
     {WR_CMND_MULTIPLE_DATA     ,    0xB4    , 0     ,7, 
     param5},

	  {WR_CMND_MULTIPLE_DATA     ,    0xC0    , 0     ,6, 
      param4},

    {WR_CMND_MULTIPLE_DATA     ,    0xC6    , 0     ,2,  param2},

	 {SLEEP_MS,  0, 5},
	 
     {WR_CMND_MULTIPLE_DATA     ,    0xE0    , 0     ,67,   	 
     param6},
     {SLEEP_MS,  0,  10},
     
    {WR_CMND_DATA,   0xB0, 0x68},// 70 hz
     {SLEEP_MS,  0,  1},
 //---yuv and DITH enable start---//    
     {WR_CMND_DATA,   0x3A, 0x07},
     {SLEEP_MS,  0,  5},
     
     {WR_CMND_DATA,   0xE9, 0x30},
     {SLEEP_MS,  0,  5},
 //---yuv and DITH enable end---//      
    {WR_CMND,   0x11, 0},//Sleep Out      
    {SLEEP_MS,  0,  250},
   	 
    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,  10},  
    
    {WR_CMND,   0x2C, 0},  
    //--- END OF COMMAND LIST -----------------------
    {CTRL_END        , 0}
};

Lcd_init_t  enter_sleep_seq[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{CTRL_END        , 0}
};

Lcd_init_t  exit_sleep_seq[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
    	{DISPCTRL_SLEEP_MS,  0,  120},  
    	{CTRL_END        , 0}
};

#endif
