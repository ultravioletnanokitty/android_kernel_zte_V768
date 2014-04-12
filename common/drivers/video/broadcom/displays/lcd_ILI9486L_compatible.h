#ifndef __LCD_ILI9486__
#define __LCD_ILI9486__
#include "lcd_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif

#define OPEN_LCD_TE

#define LCD_INFO_ALIAS(var)	ILI9486##_##var

LCD_dev_info_t LCD_INFO_ALIAS(LCD_device)[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= 480,
	 .width		= 320,
	 .bits_per_pixel= 16,
//xiongbiao@wind-mobi.com 2011.12.20 begin
//open LCD TE funtion to fix camera display.	
#ifdef OPEN_LCD_TE
	 .te_supported	= 1,
#else
   	 .te_supported  = 0,
#endif
   //.te_supported	= false
//xiongbiao@wind-mobi.com 2011.12.20 end
	 }
};

static lcd_timing_info_t LCD_INFO_ALIAS(reg_timing) = {180, 188, 0, 36, 50, 14};
static lcd_timing_info_t LCD_INFO_ALIAS(mem_timing) = {180, 188, 0, 36, 50, 14};


static Lcd_init_t LCD_INFO_ALIAS(lcd_init)[] = {
	
//xiongbiao@wind-mobi.com 2011.12.20 begin
//open LCD TE funtion to fix camera display.		
	{WR_CMND, 0x36, 0x00},
#ifdef OPEN_LCD_TE
	{WR_DATA, 0x00, 0x08},
#else
	{WR_DATA, 0x00, 0x88},
#endif

#ifdef OPEN_LCD_TE	
	{WR_CMND, 0x35, 0}, 	
	{WR_DATA, 0x00, 0x00},	//open TE signal
			
	{WR_CMND, 0x44, 0}, 	
	{WR_DATA, 0x00, 0x20},
	{WR_DATA, 0x00, 0xC2},

	{WR_CMND, 0xB6, 0}, 	
	{WR_DATA, 0x00, 0x00},	
	{WR_DATA, 0x00, 0x42},//D6,D5 for direction	
	{WR_DATA, 0x00, 0x3B},	
#endif	
//xiongbiao@wind-mobi.com 2011.12.20 end

	{WR_CMND, 0x3A, 0},     //set pixel format  
	{WR_DATA, 0x00, 0x55},	//16bpp 0x66=>18bpp
//xiongbiao@wind-mobi.com 2011.11.29 begin
//reviewed by yuanlan@wind-mobi.com
//modify LCD effect in internet 
/*	
	{WR_CMND, 0xE0, 0}, 	//set GammaS
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x0A},
	{WR_DATA, 0x00, 0x03},
	{WR_DATA, 0x00, 0x13},
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x2E},
	{WR_DATA, 0x00, 0x9B},
	{WR_DATA, 0x00, 0x44},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x0B},
	{WR_DATA, 0x00, 0x3F},
	{WR_DATA, 0x00, 0x3F},
	{WR_DATA, 0x00, 0x0F},
				
	{WR_CMND, 0xE1, 0}, 	//set Gamma
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x16},
	{WR_DATA, 0x00, 0x0D},
	{WR_DATA, 0x00, 0x3B},
	{WR_DATA, 0x00, 0x56},
	{WR_DATA, 0x00, 0x51},
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x18},
	{WR_DATA, 0x00, 0x0C},
	{WR_DATA, 0x00, 0x35},
	{WR_DATA, 0x00, 0x2A},
	{WR_DATA, 0x00, 0x0F},
		
	{WR_CMND, 0xF1, 0},
	{WR_DATA, 0x00, 0x36},
	{WR_DATA, 0x00, 0x04},
	{WR_DATA, 0x00, 0x00},			
	{WR_DATA, 0x00, 0x3C},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0xA4},
	{WR_DATA, 0x00, 0x02},
			
	{WR_CMND, 0xF2, 0},
	{WR_DATA, 0x00, 0x18},
	{WR_DATA, 0x00, 0xA3},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0x72},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0xFF},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x00},
*/
	{WR_CMND, 0xE0, 0}, 	//set GammaS
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x05},
	{WR_DATA, 0x00, 0x1A}, //0x0A 
	{WR_DATA, 0x00, 0x03},
	{WR_DATA, 0x00, 0x0F}, //0x13
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x30}, //0x2E
	{WR_DATA, 0x00, 0x9B},
	{WR_DATA, 0x00, 0x47}, //0x44
	{WR_DATA, 0x00, 0x05}, //0x02
	{WR_DATA, 0x00, 0x09},
	{WR_DATA, 0x00, 0x07}, //0x0B
	{WR_DATA, 0x00, 0x1F}, //0x3F
	{WR_DATA, 0x00, 0x2E}, //0x3F
	{WR_DATA, 0x00, 0x0F},
					
	{WR_CMND, 0xE1, 0}, 	//set Gamma
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x11}, //0x00
	{WR_DATA, 0x00, 0x20}, //0x00
	{WR_DATA, 0x00, 0x08}, //0x04
	{WR_DATA, 0x00, 0x16},
	{WR_DATA, 0x00, 0x0A}, //0x0D
	{WR_DATA, 0x00, 0x38}, //0x3B
	{WR_DATA, 0x00, 0x56},
	{WR_DATA, 0x00, 0x4f}, //0x51
	{WR_DATA, 0x00, 0x07},
	{WR_DATA, 0x00, 0x10}, //0x18
	{WR_DATA, 0x00, 0x0C},
	{WR_DATA, 0x00, 0x25}, //0x35
	{WR_DATA, 0x00, 0x2A},
	{WR_DATA, 0x00, 0x0F},
					
	{WR_CMND, 0xF1, 0},
	{WR_DATA, 0x00, 0xF6}, //36
	{WR_DATA, 0x00, 0x04}, 
	{WR_DATA, 0x00, 0x00},			
	{WR_DATA, 0x00, 0x3C},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0xA4},
	{WR_DATA, 0x00, 0x02},
		
	{WR_CMND, 0xF2, 0},
	{WR_DATA, 0x00, 0x18},
	{WR_DATA, 0x00, 0xA3},
	{WR_DATA, 0x00, 0x12},
	{WR_DATA, 0x00, 0x02},
	{WR_DATA, 0x00, 0xB2},
	{WR_DATA, 0x00, 0x92},
	{WR_DATA, 0x00, 0xFF},
	{WR_DATA, 0x00, 0x10},
	{WR_DATA, 0x00, 0x00},//08

//xiongbiao@wind-mobi.com 2011.11.29 end

        {WR_CMND, 0xF4, 0},
        {WR_DATA, 0x00, 0x08},
        {WR_DATA, 0x00, 0x00},
        {WR_DATA, 0x00, 0x08},
        {WR_DATA, 0x00, 0x91},
        {WR_DATA, 0x00, 0x04},

	{WR_CMND, 0xF7, 0},
	{WR_DATA, 0x00, 0xA9},	
	{WR_DATA, 0x00, 0x91},
	{WR_DATA, 0x00, 0x2D},
	{WR_DATA, 0x00, 0x0A},
	{WR_DATA, 0x00, 0x4F},

	{WR_CMND, 0xF8, 0},
	{WR_DATA, 0x00, 0x21},
	{WR_DATA, 0x00, 0x04},

        {WR_CMND, 0xF9, 0},
        {WR_DATA, 0x00, 0x40}, //00
        {WR_DATA, 0x00, 0x08},

	{WR_CMND, 0xB4, 0},     //Set 2Dot Invertion
	{WR_DATA, 0x00, 0x02},
	
	{WR_CMND, 0xB1, 0},     //Set Frame Rate 
//xiongbiao@wind-mobi.com 2011.12.20 begin
//open LCD TE funtion to fix camera display.
#ifdef OPEN_LCD_TE
	{WR_DATA, 0x00, 0x90},
#else
	{WR_DATA, 0x00, 0xB0},
#endif
//xiongbiao@wind-mobi.com 2011.12.20 end	
	{WR_DATA, 0x00, 0x11},
	
	{WR_CMND, 0xC0, 0},
	{WR_DATA, 0x00, 0x0F},
	{WR_DATA, 0x00, 0x0F},	
	
	{WR_CMND, 0xC1, 0},
	{WR_DATA, 0x00, 0x41},
//	{WR_DATA, 0x00, 0x07},
	
	{WR_CMND, 0xC5, 0},
	{WR_DATA, 0x00, 0x00},
	{WR_DATA, 0x00, 0x2C},
	{WR_DATA, 0x00, 0x80},
	
	{WR_CMND, 0x11, 0}, 
	{SLEEP_MS,0, 200},      //Exit sleep
	
        {WR_CMND, 0x29, 0},    //Display on
	{SLEEP_MS, 0, 120},		
	
	{WR_CMND, 0x2C, 0},	
	
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
		{WR_DATA, 0, (dev->col_start) >> 8},
		{WR_DATA, 0, (dev->col_start) & 0xff},
		{WR_DATA, 0, (dev->col_end) >> 8},
		{WR_DATA, 0, (dev->col_end) & 0xFF},
		{WR_CMND, 0x2B, 0},
		{WR_DATA, 0, (dev->row_start) >> 8},
		{WR_DATA, 0, (dev->row_start) & 0xFF},
		{WR_DATA, 0, (dev->row_end) >> 8},
		{WR_DATA, 0, (dev->row_end) & 0xFF},
		{WR_CMND, 0x2C, 0},
		{CTRL_END, 0, 0}
	};

	lcd_send_cmd_sequence(set_seq);
}

static uint32_t LCD_INFO_ALIAS(check_lcd_drv_id)()
{
	uint32_t drv_id[4] = {0};
	
	lcd_write_cmd(0xD3);
	US_DELAY(10);
	
	lcd_read_data(drv_id, 4);

	LCD_PRINT_LOG("check_lcd_drv_id...........%x, %x, %x, %x\n", drv_id[0],drv_id[1],drv_id[2],drv_id[3]);
	if (drv_id[2] == 0x94 && drv_id[3] == 0x86)
	{
		return 0x9486;
	}

	return 0;
}

//shaojiang@wind-mobi.com 2012.01.13 begin
//add lcd wakeup function
//reviewed by liubing@wind-mobi.com
static lcd_drv_info_t lcd_info_ili9486 = {LCD_16BIT, &LCD_INFO_ALIAS(reg_timing), \
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
