#ifndef __LCD_ST7793__
#define __LCD_ST7793__
#include "lcd_drv.h"

#ifdef LCD_INFO_ALIAS
#undef LCD_INFO_ALIAS
#endif

#define LCD_INFO_ALIAS(var)	ST7793##_##var

LCD_dev_info_t LCD_INFO_ALIAS(LCD_device)[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= 400,
	 .width		= 240,
	 .bits_per_pixel= 16,
//open LCD TE funtion to fix camera display.	
	 .te_supported	= 1,    //.te_supported	= false
	 }
};

static lcd_timing_info_t LCD_INFO_ALIAS(reg_timing) = {180, 188, 0, 36, 50, 14};
static lcd_timing_info_t LCD_INFO_ALIAS(mem_timing) = {180, 188, 0, 36, 50, 14};

static Lcd_init_t LCD_INFO_ALIAS(lcd_init)[] = {
//display control setting
    {WR_CMND, 0x0001, 0},
    {WR_DATA, 0x00, 0x0100},
    {WR_CMND, 0x0003, 0},
    {WR_DATA, 0x00, 0x9030},
    {WR_CMND, 0x0008, 0},
    {WR_DATA, 0x00, 0x0808},
    {WR_CMND, 0x0090, 0},
    {WR_DATA, 0x00, 0x8000},
    {WR_CMND, 0x0400, 0},
    {WR_DATA, 0x00, 0x6200},
    {WR_CMND, 0x0401, 0},
    {WR_DATA, 0x00, 0x0001},

//power control registers initial
    {WR_CMND, 0x00FF, 0},
    {WR_DATA, 0x00, 0x0001},
    {WR_CMND, 0x0102, 0},
    {WR_DATA, 0x00, 0x01B0},
    {WR_CMND, 0x0710, 0},
    {WR_DATA, 0x00, 0x0016},
    {WR_CMND, 0x0712, 0},
    {WR_DATA, 0x00, 0x000F},
    {WR_CMND, 0x0752, 0},
    {WR_DATA, 0x00, 0x002F},
    {WR_CMND, 0x0724, 0},
    {WR_DATA, 0x00, 0x001A},
    {WR_CMND, 0x0754, 0},
    {WR_DATA, 0x00, 0x0018},

    {SLEEP_MS,0, 120},

//display window 240 * 400
    {WR_CMND, 0x0210, 0},
    {WR_DATA, 0x00, 0x0000},
    {WR_CMND, 0x0211, 0},
    {WR_DATA, 0x00, 0x00EF},
    {WR_CMND, 0x0212, 0},
    {WR_DATA, 0x00, 0x0000},
    {WR_CMND, 0x0213, 0},
    {WR_DATA, 0x00, 0x018f},

    {SLEEP_MS,0, 15},

//gamma cluster setting
    {WR_CMND, 0x0380, 0},
    {WR_DATA, 0x00, 0x0000},
    {WR_CMND, 0x0381, 0},
    {WR_DATA, 0x00, 0x5F10},
    {WR_CMND, 0x0382, 0},
    {WR_DATA, 0x00, 0x0B02},
    {WR_CMND, 0x0383, 0},
    {WR_DATA, 0x00, 0x0614},
    {WR_CMND, 0x0384, 0},
    {WR_DATA, 0x00, 0x0111},
    {WR_CMND, 0x0385, 0},
    {WR_DATA, 0x00, 0x0000},
    {WR_CMND, 0x0386, 0},
    {WR_DATA, 0x00, 0xA90B},
    {WR_CMND, 0x0387, 0},
    {WR_DATA, 0x00, 0x0606},
    {WR_CMND, 0x0388, 0},
    {WR_DATA, 0x00, 0x0612},
    {WR_CMND, 0x0389, 0},
    {WR_DATA, 0x00, 0x0111},

//vcom setting
    {WR_CMND, 0x0702, 0},
    {WR_DATA, 0x00, 0x003B},
    {WR_CMND, 0x00FF, 0},
    {WR_DATA, 0x00, 0x0000},

    {WR_CMND, 0x0007, 0},
    {WR_DATA, 0x00, 0x0100},
    {SLEEP_MS,0, 240},
    {WR_CMND, 0x0200, 0},
    {WR_DATA, 0x00, 0x0000},
    {WR_CMND, 0x0201, 0},
    {WR_DATA, 0x00, 0x0000},
    {CTRL_END, 0, 0},
};

static Lcd_init_t LCD_INFO_ALIAS(poweroff)[] = {
    {WR_CMND, 0x0007, 0},
    {WR_DATA, 0x00, 0x0000},
    {SLEEP_MS,0, 60},
    {WR_CMND, 0x0102, 0},
    {WR_DATA, 0x00, 0x0180},
    {SLEEP_MS, 0, 240},
    {CTRL_END, 0, 0},
};

static Lcd_init_t LCD_INFO_ALIAS(wakeup)[] = {
    {SLEEP_MS, 0, 240},
    {WR_CMND, 0x0102, 0},
    {WR_DATA, 0x00, 0x01B0},
    {SLEEP_MS,0, 60},
    {WR_CMND, 0x0007, 0},
    {WR_DATA, 0x00, 0x0100},
    {SLEEP_MS, 0, 240},
    {CTRL_END, 0, 0},
};


static void LCD_INFO_ALIAS(lcd_set_addr)(LCD_dev_info_t *dev)
{
/*
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
*/
    return;
}

static uint32_t LCD_INFO_ALIAS(check_lcd_drv_id)()
{
/*
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

*/	return 0;
}

//shaojiang@wind-mobi.com 2012.01.13 begin
//add lcd wakeup function
//reviewed by liubing@wind-mobi.com
static lcd_drv_info_t lcd_info_st7793 = {LCD_18BIT, &LCD_INFO_ALIAS(reg_timing), \
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
