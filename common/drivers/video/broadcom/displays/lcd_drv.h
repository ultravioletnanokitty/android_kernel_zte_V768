

#ifndef __LCD_DRV__
#define __LCD_DRV__

#ifdef BUILD_BOOTLOADER
#if defined(CONFIG_BCM_IDLE_PROFILER_SUPPORT)
#include <linux/broadcom/idle_prof.h>
#endif

#include <boot/lcd_common.h>
#include <boot/bcm2153_logo.h>
#include <boot/timer.h>
#include <boot/gpio.h>
#include <boot/reg_lcd.h>
#include <boot/boot.h>
#endif

#ifdef BUILD_BOOTLOADER
#define LCD_PRINT_LOG dprintf
#else
#define LCD_PRINT_LOG printk
#endif

//shaojiang@wind-mobi.com 2012.1.31 begin
//patch CSP496192 for lcd driver
//reviewed by liubing@wind-mobi.com
#ifdef BUILD_BOOTLOADER
typedef enum
{
	WR_CMND,
	WR_DATA,
	WR_CMND_DATA,
	SLEEP_MS,
	CTRL_END,
} ctrl_t;

typedef struct lcd_init_t
{
    ctrl_t type;
    uint16_t cmd;
    uint16_t data;

} Lcd_init_t;
#endif
//shaojiang@wind-mobi.com 2012.1.31 end

typedef struct {
	uint32_t rdHold;	
	uint32_t rdPulse; 
	uint32_t rdSetup;
	uint32_t wrHold;	
	uint32_t wrPulse;
	uint32_t wrSetup; 
} lcd_timing_info_t;

//shaojiang@wind-mobi.com 2012.01.13 begin
//add lcd wakeup function
//reviewed by liubing@wind-mobi.com
typedef struct lcd_drv_info_tag {	
	LCD_Bus_t lcd_bus;
	lcd_timing_info_t* reg_timing;
	lcd_timing_info_t* mem_timing;
	LCD_dev_info_t* lcd_dev;
	Lcd_init_t*	lcd_init_param;
	Lcd_init_t* lcd_power_off_param;	
	Lcd_init_t* lcd_wakeup_param;
	void (*lcd_set_addr)(LCD_dev_info_t *dev);
	uint32_t (*check_lcd_drv_id)();
} lcd_drv_info_t;
//shaojiang@wind-mobi.com 2012.01.13 end


#define US_DELAY(x)\
do {\
  volatile int i,j;\
  for(i=0;i<(x*2);i++)\
  {\
  	j++;\
  }\
} while(0);


extern lcd_drv_info_t* cur_lcd_drv_ptr;

extern void lcd_send_cmd_sequence(Lcd_init_t *init);
extern void lcd_write_cmd(uint32_t cmd);
extern void lcd_write_data(uint32_t data);
extern void lcd_write_param(uint32_t cmd);
extern void lcd_read_data(uint32_t data[], uint32_t len);

#endif
