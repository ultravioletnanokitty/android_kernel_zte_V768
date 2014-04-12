#ifndef __LCD_HX8357C__
#define __LCD_HX8357C__
#include "lcd_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif

#define LCD_INFO_ALIAS(var)	HX8357C##_##var

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
static lcd_timing_info_t LCD_INFO_ALIAS(reg_timing) = {180, 188, 0, 36, 50, 14};
static lcd_timing_info_t LCD_INFO_ALIAS(mem_timing) = {180, 188, 0, 36, 50, 14};
//static lcd_timing_info_t LCD_INFO_ALIAS(reg_timing) = {180, 188, 0, 44, 58, 22};
//static lcd_timing_info_t LCD_INFO_ALIAS(mem_timing) = {180, 188, 0, 44, 58, 22};
//xiongbiao@wind-mobi.com 2011-11-23 end

static Lcd_init_t LCD_INFO_ALIAS(lcd_init)[] = {
	{WR_CMND, 0x11, 0}, 
	{SLEEP_MS,0, 120},	//Exit sleep

	{WR_CMND, 0xB9, 0}, 	
	{WR_DATA, 0x00, 0xFF},
	{WR_DATA, 0x00, 0x83},
	{WR_DATA, 0x00, 0x57},

	{WR_CMND, 0xCC, 0}, 	
	{WR_DATA, 0x00, 0x09},

	{WR_CMND, 0xB6, 0}, 	
	{WR_DATA, 0x00, 0x52},//52
	
	{WR_CMND, 0x36, 0}, 	//add for handstand by xiongbiao
	{WR_DATA, 0x00, 0x00},
	
//xiongbiao@wind-mobi.com 2011.12.20 begin
	//open LCD TE funtion to fix camera display.		
	//{WR_CMND, 0x35, 0}, 	
	//{WR_DATA, 0x00, 0x01},	//open TE signal
		
	//{WR_CMND, 0x44, 0}, 	
	//{WR_DATA, 0x00, 0x00},
	//{WR_DATA, 0x00, 0x32},
//xiongbiao@wind-mobi.com 2011.12.20 end
   	{WR_CMND, 0x35, 0}, 	
	{WR_DATA, 0x00, 0x00}, 

    {WR_CMND, 0xB0, 0},   //debug by Marks 2011.12.14
	{WR_DATA, 0x00, 0x06},//debug by Marks 2011.12.14
	{WR_DATA, 0x00, 0x01},//debug by Marks 2011.12.14

	{WR_CMND, 0xB1, 0}, 	
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x24},//0x24//23
	{WR_DATA, 0x00, 0x26},//2B
	{WR_DATA, 0x00, 0x2B},
	{WR_DATA, 0x00, 0x86},
	{WR_DATA, 0x00, 0x42},//4A
						
	{WR_CMND, 0xB4, 0}, 	
	{WR_DATA, 0x00, 0x02},//02
	{WR_DATA, 0x00, 0x40}, //RTN
	{WR_DATA, 0x00, 0x00}, //DIV
	{WR_DATA, 0x00, 0x2A}, //DUM
	{WR_DATA, 0x00, 0x2A}, //DUM
	{WR_DATA, 0x00, 0x0D}, //GDON
	{WR_DATA, 0x00, 0x78}, //GDOFF//78//96

	{WR_CMND, 0xB5, 0}, 	
	{WR_DATA, 0x00, 0x03},//02
	{WR_DATA, 0x00, 0x03},

	{WR_CMND, 0xC0, 0}, //STBA
	{WR_DATA, 0x00, 0x50}, //OPON
	{WR_DATA, 0x00, 0x50}, //OPON
	{WR_DATA, 0x00, 0x01}, //
	{WR_DATA, 0x00, 0x3C},
	{WR_DATA, 0x00, 0xC8},
	{WR_DATA, 0x00, 0x08},  //GEN

						
	{WR_CMND, 0xE0, 0}, 	
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x22},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x50},
	{WR_DATA, 0x00, 0x56},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x3E},
	{WR_DATA, 0x00, 0x39},
	{WR_DATA, 0x00, 0x31},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x26},
	{WR_DATA, 0x00, 0x23},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x22},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x50},
	{WR_DATA, 0x00, 0x56},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x3E},
	{WR_DATA, 0x00, 0x39},
	{WR_DATA, 0x00, 0x31},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x26},
	{WR_DATA, 0x00, 0x23},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x01},
	
	{WR_CMND, 0xC6, 0}, 
	{WR_DATA, 0x00, 0x00}, 
	{WR_DATA, 0x00, 0xFC},

	{WR_CMND, 0xCC, 0}, 
	{WR_DATA, 0x00, 0x05},


	{WR_CMND, 0x29, 0},    //Display on
	
	{WR_CMND, 0x2C, 0},    
	
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
		{WR_DATA, 0x00, (dev->col_end) >> 8},
		{WR_DATA, 0x00, dev->col_end & 0xFF},
		{WR_CMND, 0x2B, 0},
		{WR_DATA, 0x00, (dev->row_start) >> 8},
		{WR_DATA, 0x00, dev->row_start & 0xFF},
		{WR_DATA, 0x00, (dev->row_end) >> 8},
		{WR_DATA, 0x00, dev->row_end & 0xFF},
		{WR_CMND, 0x2C, 0},		
		{CTRL_END, 0, 0}
	};

	lcd_send_cmd_sequence(set_seq);
}
	
static uint32_t LCD_INFO_ALIAS(check_lcd_drv_id)()
{
	uint32_t drv_id[2] = {0};

	Lcd_init_t enable_extc[] = {
		{WR_CMND, 0xB9, 0}, 	
		{WR_DATA, 0x00, 0xFF},
		{WR_DATA, 0x00, 0x83},
		{WR_DATA, 0x00, 0x57},	
		{CTRL_END, 0, 0}
	};
	lcd_send_cmd_sequence(enable_extc);
	US_DELAY(10);

	lcd_write_cmd(0xD0);
	US_DELAY(10);

	lcd_read_data(drv_id, 2);

    LCD_PRINT_LOG("HX8357C check_lcd_drv_id......%x.....\n", drv_id[0]);
    LCD_PRINT_LOG("HX8357C check_lcd_drv_id......%x.....\n", drv_id[1]);	
	
	if (drv_id[1] == 0x90)
	{
		return 0x90;
	}

	return 0;
}

//shaojiang@wind-mobi.com 2012.01.13 begin
//add lcd wakeup function
//reviewed by liubing@wind-mobi.com
static lcd_drv_info_t lcd_info_hx8357c = {LCD_18BIT, &LCD_INFO_ALIAS(reg_timing), \
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
