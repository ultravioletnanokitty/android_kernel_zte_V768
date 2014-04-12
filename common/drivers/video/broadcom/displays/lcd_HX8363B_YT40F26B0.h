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

#ifndef __LCD_HX8363B_H__
#define __LCD_HX8363B_H__

#define DSI_VC            (0)
#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS mode

#define LCD_HEIGHT		800
#define LCD_WIDTH		480

#define PANEL_WIDTH 480
#define PANEL_HEIGHT 800


#define LCD_BITS_PER_PIXEL 32
#define INPUT_BPP 4
#define RESET_GPIO 45
#define DSI_VC             (0)
#define DSI_CMND_IS_LP    TRUE  // display init comm LP or HS m


#define LCD_CMD(x) (x)
#define LCD_DATA(x) (x)

const char *LCD_panel_name = "HX8363B LCD";

int LCD_num_panels = 1;
LCD_Intf_t LCD_Intf = LCD_DSI;
LCD_Bus_t LCD_Bus = LCD_32BIT;


//Not used, just to satisfy the compilation
CSL_LCDC_PAR_SPEED_t timingReg_ns;
CSL_LCDC_PAR_SPEED_t timingMem_ns;


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

static uint8_t param1[] = {0xFF, 0x83, 0x63};

static uint8_t param2[] = {0x78,
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
    	 };

static uint8_t param3[] = {0x08, 0x00};
/*
static uint8_t param4[] =  {0x02, 0x18,
    	 0x9C, 0x08,
    	 0x18, 0x04,
    	 0x72};
*/
static uint8_t param4[] =  {0x0A, 0x18, //0x00
    	 0x9C, 0x08,
    	 0x18, 0x04,
    	 0x72};
static uint8_t param5[] =   {0x05, 0x60, 0x00, 0x10};

static uint8_t param6[] =  {	 0x00 
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
    	 };

static uint8_t param7[] =   {0x10, 0x00, 0x56, 0xC6,0x10,
                             0x89, 0xFF, 0x0F, 0x33,0x6E,
                             0x04, 0x07, 0x9A, 0x10};


static uint8_t param8[] =   {0x01, 0x09};

Lcd_init_t power_on_seq[] = {
    {WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3,  param1},// SET password
    {SLEEP_MS,  0, 1},

    {WR_CMND,   0x11, 0},//Sleep Out      
    {SLEEP_MS,  0,  1400},
  
   {WR_CMND_MULTIPLE_DATA     ,    0xB1    , 0     ,19, //SET Power
    param2},
    	
    {SLEEP_MS,  0, 1},
    {WR_CMND_MULTIPLE_DATA     ,    0xB2    , 0     ,2,  param3}, // SET DISP	  
    {SLEEP_MS,  0, 1},
    {WR_CMND_MULTIPLE_DATA     ,    0xB4    , 0     ,7, // SET CYC
     param4}, 	      
    {SLEEP_MS,  0, 1}, 
    {WR_CMND_MULTIPLE_DATA     ,    0xBF    , 0     ,4, // SET VCOM for Max=-2.5V
     param5}, 	     
    {SLEEP_MS,  0, 1},    
    {WR_CMND_DATA,	 0xB6, 	0x40}, // SET VCOM  0x40 
    {SLEEP_MS,  0, 1},
    {WR_CMND_DATA,	 0xCC, 	0x0B}, //Set Panel  0x03
    {SLEEP_MS,  0, 1},

     {WR_CMND_MULTIPLE_DATA     ,    0xE0    , 0     ,34,  param6},
    {SLEEP_MS,  0, 1},   
/* yaogangxiang@wind-mobi.com 2012.02.16 begin */
	{WR_CMND_MULTIPLE_DATA,  0xB0,	0,	  2,	 param8}, 
	{SLEEP_MS,	0, 10},   
/* yaogangxiang@wind-mobi.com 2012.02.16 begin */

    {WR_CMND_MULTIPLE_DATA,	 0xBA, 	0,    14,    param7}, 
    {SLEEP_MS,  0, 10},   	 
    
    {WR_CMND_DATA,	 0xC2, 	0x04}, 
    {SLEEP_MS,  0,  10},
    
    {WR_CMND_DATA,	 0x3A, 	0x77}, 
    //{SLEEP_MS,  0, 1},
    
    {WR_CMND,   0x35, 0},//TE ON

/*
    {WR_CMND_DATA,	 0x51, 	0x03}, 
    {WR_CMND_DATA,	 0x53, 	0x24}, 
    {WR_CMND_DATA,	 0x55, 	0x00}, 
    {WR_CMND_DATA,	 0x35, 	0x00}, 
    {WR_CMND_DATA,	 0xC9, 	0x05}, 
    {WR_CMND_DATA,	 0x36, 	0x02}, 
 */
 
    
    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,  500},
        
   // {DISPCTRL_WR_CMND_DATA,	 0xB0, 	0x68}, //70Hz      
    //--- END OF COMMAND LIST -----------------------

    {WR_CMND,   0x2C, 0},//Sleep Out      
    {SLEEP_MS,  0,  10},

    //{WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3, {0xFF, 0x83, 0x00}},// SET password

    {CTRL_END        , 0}
};

Lcd_init_t  enter_sleep_seq[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
    	{DISPCTRL_SLEEP_MS,  0,  800},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  800},
        {CTRL_END        , 0}
};

Lcd_init_t  exit_sleep_seq[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  800},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
    	{DISPCTRL_SLEEP_MS,  0,  800},  
       {CTRL_END        , 0}
};

#endif
