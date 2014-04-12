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

#ifndef __LCD_HX8369A_YT40F25J0_H__
#define __LCD_HX8369A_YT40F25J0_H__

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

const char *LCD_panel_name = "HX8369A_YT40F25J0 LCD";

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

static uint8_t param1[] = {0xFF, 0x83, 0x69};
static uint8_t param8[] = {0x01, 0x0B};

static uint8_t param2[] = {
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

<<<<<<< HEAD
static uint8_t param3[] = {0x00, 0x20,0x0A, 0x0A,0x70, 0x00,0xFF, 0x00,
=======
static uint8_t param3[] = {0x03, 0x20,0x0A, 0x0A,0x70, 0x00,0xFF, 0x00,
>>>>>>> 3de41054abba51e12c25d370ba7f77092494edce
							0x00, 0x00,0x00, 0x03,0x03, 0x00,0x01};
/*
static uint8_t param4[] =  {0x02, 0x18,
    	 0x9C, 0x08,
    	 0x18, 0x04,
    	 0x72};
*/
<<<<<<< HEAD
static uint8_t param4[] = {0x00, 0x18, 0x80, 0x10,0x01};
static uint8_t param5[] = {0x20, 0x20};
=======
static uint8_t param4[] = {0x00, 0x0C, 0x84, 0x0C,0x01};
static uint8_t param5[] = {0x2C, 0x2C};
>>>>>>> 3de41054abba51e12c25d370ba7f77092494edce
static uint8_t param9[] = { 0x00, 0x05,0x03, 0x00,0x01, 0x09,0x10, 0x80,
							0x37, 0x37,0x20, 0x31,0x46, 0x8A,0x57, 0x9B,
							0x20, 0x31,0x46, 0x8A,0x57, 0x9B,0x07, 0x0F,
							0x07, 0x00};

static uint8_t param6[] =  {	 0x00 
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

static uint8_t param7[] =   {0x00, 0xA0, 0xC6, 0x00,0x0A,
                             0x00, 0x10, 0x30, 0x6C,0x02,
                             0x10, 0x18, 0x40};

Lcd_init_t power_on_seq[] = {
    {WR_CMND_MULTIPLE_DATA     ,    0xB9    , 0     ,3,  param1},// SET password
    {SLEEP_MS,  0, 1},

<<<<<<< HEAD
    //{WR_CMND_MULTIPLE_DATA,0xB0, 0,2, param8},
	//{SLEEP_MS,	0, 1},
=======
    {WR_CMND_MULTIPLE_DATA,0xB0, 0,2, param8},
	{SLEEP_MS,	0, 1},
>>>>>>> 3de41054abba51e12c25d370ba7f77092494edce

    {WR_CMND_MULTIPLE_DATA,0xB1, 0,19, param2},
    {SLEEP_MS,  0, 10},
    
    {WR_CMND_MULTIPLE_DATA, 0xB2, 0,15, param3}, // SET DISP	  
    {SLEEP_MS,  0, 1},
    
    {WR_CMND_MULTIPLE_DATA, 0xB4, 0,5, param4}, 	      
    {SLEEP_MS,  0, 1}, 


    {WR_CMND_MULTIPLE_DATA, 0xB6, 0,2, param5}, 	      
    {SLEEP_MS,  0, 1}, 

    {WR_CMND_MULTIPLE_DATA, 0xD5, 0,26, param9}, 	      
    {SLEEP_MS,  0, 1}, 
	
    {WR_CMND_MULTIPLE_DATA, 0xE0, 0 ,34, param6},
    {SLEEP_MS,  0, 1},   	 

    {WR_CMND_DATA,	 0x3A, 	0x77}, 
    {SLEEP_MS,  0, 1},

    {WR_CMND_DATA,	 0x35, 	0x00}, 
    {SLEEP_MS,  0, 1},

    {WR_CMND_MULTIPLE_DATA,	 0xBA, 	0,    13,    param7}, 
    {SLEEP_MS,  0, 1},   	 
    
<<<<<<< HEAD
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
=======
    {WR_CMND_DATA,	 0x51, 	0x0F}, 
    {SLEEP_MS,  0,  1},
    
    {WR_CMND_DATA,	 0x53, 	0x24}, 
    {SLEEP_MS,  0,  1},

    {WR_CMND_DATA,	 0x55, 	0x00}, 
    {SLEEP_MS,  0,  1},

    {WR_CMND,   0x11, 0},//Sleep Out      
    {SLEEP_MS,  0,  50}, 
    
    {WR_CMND,   0x29, 0},// display on    
    {SLEEP_MS,  0,  10},
>>>>>>> 3de41054abba51e12c25d370ba7f77092494edce

    {CTRL_END        , 0}
};

Lcd_init_t  enter_sleep_seq[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_OFF, 0},
<<<<<<< HEAD
    	{DISPCTRL_SLEEP_MS,  0,  100},  
=======
    	{DISPCTRL_SLEEP_MS,  0,  400},  
>>>>>>> 3de41054abba51e12c25d370ba7f77092494edce
    	{DISPCTRL_WR_CMND,   MIPI_DCS_ENTER_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
       {CTRL_END        , 0}
};

Lcd_init_t  exit_sleep_seq[] = {
    	{DISPCTRL_WR_CMND,   MIPI_DCS_EXIT_SLEEP_MODE, 0},
    	{DISPCTRL_SLEEP_MS,  0,  400},  
    	{DISPCTRL_WR_CMND,   MIPI_DCS_SET_DISPLAY_ON, 0},
<<<<<<< HEAD
    	{DISPCTRL_SLEEP_MS,  0,  100},  
=======
    	{DISPCTRL_SLEEP_MS,  0,  400},  
>>>>>>> 3de41054abba51e12c25d370ba7f77092494edce
       {CTRL_END        , 0}
};

#endif
