#ifndef __LCD_HX8357B__
#define __LCD_HX8357B__
#include "lcd_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif

#define LCD_INFO_ALIAS(var)	HX8357B##_##var

LCD_dev_info_t LCD_INFO_ALIAS(LCD_device)[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= 480,
	 .width		= 320,
	 .bits_per_pixel= 16,
//xiongbiao@wind-mobi.com 2011.12.20 begin
//open LCD TE funtion to fix camera display.		 
	 .te_supported	= 1,    //.te_supported	= false
//xiongbiao@wind-mobi.com 2011.12.20 begin
	 }
};

//xiongbiao@wind-mobi.com 2011-11-23 begin
//fix lcd blur displuy issue
//reviewed by yuanlan@wind-mobi.com
//static lcd_timing_info_t LCD_INFO_ALIAS(reg_timing) = {180, 188, 0, 36, 50, 14};
//static lcd_timing_info_t LCD_INFO_ALIAS(mem_timing) = {180, 188, 0, 36, 50, 14};
static lcd_timing_info_t LCD_INFO_ALIAS(reg_timing) = {180, 188, 0, 44, 58, 22};
static lcd_timing_info_t LCD_INFO_ALIAS(mem_timing) = {180, 188, 0, 44, 58, 22};
//xiongbiao@wind-mobi.com 2011-11-23 end

static Lcd_init_t LCD_INFO_ALIAS(lcd_init)[] = {
	{WR_CMND, 0x11, 0}, 
	{SLEEP_MS,0, 200},	//Exit sleep

	{WR_CMND, 0xB4, 0}, 	//Set RM  DM
	{WR_DATA, 0x00, 0x00},
			
	{WR_CMND, 0xC0, 0}, 	//Set PANEL
	{WR_DATA, 0x00, 0x14},
	{WR_DATA, 0x00, 0x3B},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x11},
				
	{WR_CMND, 0xC5, 0}, 	
	{WR_DATA, 0x00, 0x0d},
	
//xiongbiao@wind-mobi.com 2011.12.20 begin
//open LCD TE funtion to fix camera display.		
	{WR_CMND, 0x35, 0}, 	
	{WR_DATA, 0x00, 0x01},  //open TE signal

	{WR_CMND, 0x44, 0}, 	
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x32},
//xiongbiao@wind-mobi.com 2011.12.20 end

	{WR_CMND, 0xC8, 0}, 	//set Gamma2.5
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x24},
	{WR_DATA, 0x00, 0x10},
	{WR_DATA, 0x00, 0x10},
	{WR_DATA, 0x00, 0x01},
	{WR_DATA, 0x00, 0x08},
	{WR_DATA, 0x00, 0x76},
	{WR_DATA, 0x00, 0x35},
	{WR_DATA, 0x00, 0x77},
	{WR_DATA, 0x00, 0x01},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x02},
		
	{WR_CMND, 0xD0, 0}, 	//Set Power
	{WR_DATA, 0x00, 0x44},	//DDVDH
	{WR_DATA, 0x00, 0x41},
	{WR_DATA, 0x00, 0x04},	//VREG1 		
	{WR_DATA, 0x00, 0xC4},	//XDK
		
	{WR_CMND, 0xD1, 0}, 	//Set VCOM
	{WR_DATA, 0x00, 0x70},	//VCOMH    70
//xiongbiao@wind-mobi.com 2011-11-23 begin
//improve display effect.
//reviewed by yuanlan@wind-mobi.com
//     {WR_DATA, 0x00, 0x19},  //VCOML  
       {WR_DATA, 0x00, 0x11},  //VCOML 
//xiongbiao@wind-mobi.com 2011-11-23 end
		
	{WR_CMND, 0xD2, 0}, 	//Set NOROW
	{WR_DATA, 0x00, 0x05},	//SAP
	{WR_DATA, 0x00, 0x12},	//DC10/00
		
	//{WR_CMND, 0xD3, 0},	//Set NOROW
	//{WR_DATA, 0x00, 0x04},	//SAP
	//{WR_DATA, 0x00, 0x12},	//DC10/00
			
	//{WR_CMND, 0xD4, 0},	//Set NOROW
	//{WR_DATA, 0x00, 0x07},	//SAP
	//{WR_DATA, 0x00, 0x12},	//DC10/00

	{WR_CMND, 0xEe, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x2c},

	{WR_CMND, 0xEd, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x9a},
	{WR_DATA, 0x00, 0x9a},
	{WR_DATA, 0x00, 0x9b},
	{WR_DATA, 0x00, 0x9b},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0xAE},
	{WR_DATA, 0x00, 0xAE},
	{WR_DATA, 0x00, 0x2c},
	{WR_DATA, 0x00, 0x9B},
	{WR_DATA, 0x00, 0x2c},
		
	{WR_CMND, 0xE9, 0}, 	//Set Color Mode 65K
	{WR_DATA, 0x00, 0x01},	//

	{WR_CMND, 0x35, 0}, 	
	{WR_DATA, 0x00, 0x00},

	{WR_CMND, 0xEA, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x03},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
			
	{WR_CMND, 0x3A, 0}, 	//Set EQ
	{WR_DATA, 0x00, 0x06},	//0x06

	{SLEEP_MS, 0, 5},
	{WR_CMND, 0x29, 0},    //Display on
	{SLEEP_MS, 0, 25},			

	{CTRL_END, 0, 0},
};

static Lcd_init_t LCD_INFO_ALIAS(poweroff)[] = {
	{WR_CMND, 0x28, 0},
	{WR_CMND, 0x10, 0},
	{SLEEP_MS, 0, 120},
	{CTRL_END, 0, 0},
};

//shaojiang@wind-mobi.com 2012.01.13 begin
//add lcd wakeup function
//reviewed by liubing@wind-mobi.com
static Lcd_init_t LCD_INFO_ALIAS(wakeup)[] = {
	{WR_CMND, 0x29, 0},
	{WR_CMND, 0x11, 0},
	{SLEEP_MS, 0, 120},
	{CTRL_END, 0, 0},
};
//shaojiang@wind-mobi.com 2012.01.13 end


static void LCD_INFO_ALIAS(lcd_set_addr)(LCD_dev_info_t *dev)
{
	Lcd_init_t set_seq[] = {
		{WR_CMND, 0x2A, 0},
		{WR_DATA, 0x00, (dev->col_start) >> 8},
		{WR_DATA, 0x00, dev->col_start & 0xFF},
		{WR_DATA, 0x01, (dev->col_end) >> 8},
		{WR_DATA, 0x3F, dev->col_end & 0xFF},
		{WR_CMND, 0x2B, 0},
		{WR_DATA, 0x00, (dev->row_start) >> 8},
		{WR_DATA, 0x00, dev->row_start & 0xFF},
		{WR_DATA, 0x01, (dev->row_end) >> 8},
		{WR_DATA, 0xE0, dev->row_end & 0xFF},
		{WR_CMND, 0x2C, 0},		
		{CTRL_END, 0, 0}
	};

	lcd_send_cmd_sequence(set_seq);
}
	
static uint32_t LCD_INFO_ALIAS(check_lcd_drv_id)()
{
	uint32_t drv_id[5] = {0};
	
	lcd_write_cmd(0xBF);
	US_DELAY(10);

	lcd_read_data(drv_id, 5);

    LCD_PRINT_LOG("HX8357B check_lcd_drv_id......%x.....\n", drv_id[0]);
    LCD_PRINT_LOG("HX8357B check_lcd_drv_id......%x.....\n", drv_id[1]);
    LCD_PRINT_LOG("HX8357B check_lcd_drv_id......%x.....\n", drv_id[2]);
    LCD_PRINT_LOG("HX8357B check_lcd_drv_id......%x.....\n", drv_id[3]);
    LCD_PRINT_LOG("HX8357B check_lcd_drv_id......%x.....\n", drv_id[4]);

	
	if (drv_id[3] == 0x83 && drv_id[4] == 0x57)
	{
		return 0x8357;
	}

	return 0;
}

//shaojiang@wind-mobi.com 2012.01.13 begin
//add lcd wakeup function
//reviewed by liubing@wind-mobi.com
static lcd_drv_info_t lcd_info_hx8357b = {LCD_18BIT, &LCD_INFO_ALIAS(reg_timing), \
											&LCD_INFO_ALIAS(mem_timing),\
											LCD_INFO_ALIAS(LCD_device), \
											LCD_INFO_ALIAS(lcd_init),\
											LCD_INFO_ALIAS(poweroff),\
											LCD_INFO_ALIAS(wakeup),\
											LCD_INFO_ALIAS(lcd_set_addr),\
											LCD_INFO_ALIAS(check_lcd_drv_id)\
											};
//shaojiang@wind-mobi.com 2012.01.13 end


#undef LCD_INFO_ALIAS

#endif
