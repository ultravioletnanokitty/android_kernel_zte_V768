

#ifndef __LCD_DSI_DRV__
#define __LCD_DSI_DRV__


typedef struct lcd_dsi_drv_info_tag {	
	LCD_dev_info_t* lcd_dev;
	CSL_DSI_CM_VC_t* dsi_cm_vc;
	CSL_DSI_CFG_t* dsi_cfg;
	Lcd_init_t* power_on_seq;
	Lcd_init_t* enter_sleep_seq;
	Lcd_init_t* exit_sleep_seq;
	char* panel_name;

} lcd_dsi_drv_info_t;




#endif
