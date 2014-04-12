/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/broadcom/dss/bcm215xx/lcdc.c
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
*  lcdc_parallel.c
*
*  PURPOSE:
*    This implements the code to use a Broadcom LCD host interface.
*
*  NOTES:
*    Uses device minor number to select panel:  0==main 1==sub
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include "lcdc_common.h"

#if defined(CONFIG_PANEL_MAGNA_D51E5TA7601)
#include "../../displays/panel-magna-d51e5ta7601.h"
#elif defined(CONFIG_BCM_LCD_NT35582)
#include "../../displays/lcd_NT35582.h"
#elif defined(CONFIG_BCM_LCD_R61581)
#include "../../displays/lcd_R61581.h"
#elif defined(CONFIG_BCM_LCD_S6D04H0A01)
#include "../../displays/lcd_tiama_s6d04h0_a01.h"
#elif defined(CONFIG_BCM_LCD_S6D04K1)
#include "../../displays/lcd_s6d04k1.h"
#elif defined(CONFIG_BCM_LCD_S6D04K1_LUISA_HW02)
#include "../../displays/lcd_s6d04k1_luisa_hw02.h"
#elif defined(CONFIG_BCM_LCD_ILI9341_BOE)
#include "../../displays/lcd_ili9341_boe.h"
#elif defined(CONFIG_BCM_LCD_ILI9341_BOE_REV05)
#include "../../displays/lcd_ili9341_boe_rev05.h"
#elif defined(CONFIG_BCM_LCD_ILI9481)
#include "../../displays/lcd_ILI9481.h"
#endif //defined(CONFIG_PANEL_MAGNA_D51E5TA7601)

#include "lcdc_parallel.h"

/**************************************************************************/
static CSL_LCDC_INIT_T ctrl_init;
static CSL_LCD_HANDLE handle;
static CSL_LCD_UPD_REQ_T req;
static CSL_LCDC_PAR_CTRL_T busCfg;

//static struct semaphore gDmaSema;

/*These structures will be updated during boot time based on the values
 * present in timingReg_ns and timingMem_ns and the AHB clock speed*/
CSL_LCDC_PAR_SPEED_t timingReg;
CSL_LCDC_PAR_SPEED_t timingMem;

/*High speed mode*/
CSL_LCDC_PAR_SPEED_t timingMem_hs;

/*----- Extern Declarations-----------------------------------------------*/
extern void __iomem *lcdc_base;

/* globals to: communicate with the update thread */
/*   control access to LCD registers */
/*  manage DMA channel */
extern bool ap_crashed;

/* ---- Functions -------------------------------------------------------- */


static void lcd_update_column(LCD_dev_info_t *dev, unsigned int column);
static void lcd_csl_cb(CSL_LCD_RES_T, CSL_LCD_HANDLE, void*);

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/
void lcd_write_cmd(uint32_t cmd)
{
	CSL_LCD_RES_T ret;
	CSL_LCDC_PAR_SetSpeed(handle, &timingReg);
	ret = CSL_LCDC_WrCmnd(handle, cmd);
	if (CSL_LCD_OK != ret)
		pr_info("CSL_LCDC_WrCmnd failed error: %d", ret);
}

void lcd_write_data(uint32_t data)
{
	CSL_LCD_RES_T ret;
	CSL_LCDC_PAR_SetSpeed(handle, &timingReg);
	ret = CSL_LCDC_WrData(handle, data);
	if (CSL_LCD_OK != ret)
		pr_info("CSL_LCDC_WrData failed error: %d", ret);
}

uint32_t lcd_read_reg(uint32_t reg)
{
	CSL_LCD_RES_T ret;
	uint32_t data;

	ret = CSL_LCDC_WrCmnd(handle, reg);
	if (CSL_LCD_OK != ret)
		pr_info("CSL_LCDC_WrCmnd failed error: %d", ret);

	ret = CSL_LCDC_PAR_RdData(handle, &data);
	if (CSL_LCD_OK != ret)
		pr_info("CSL_LCDC_PAR_RdData failed error: %d", ret);

	return data;
}


static void parallel_get_interface_info(uint32_t* phy_addr, int* width, int* height, int* bpp)
{
	*phy_addr = REG_LCDC_DATR_PADDR;
	 *width = LCD_WIDTH;
	 *height = LCD_HEIGHT;
	 *bpp = LCD_BITS_PER_PIXEL;
}

static void  parallel_update_column(LCD_dev_info_t * dev, unsigned int column)
{
	lcd_setup_for_data(dev);
	lcd_update_column(dev, column);
}

static int  parallel_update_rect_dma(LCD_dev_info_t * dev, void*  req)
{
	CSL_LCD_RES_T ret;
	int err = -EINVAL;

	lcd_setup_for_data(dev);

    if(hsm_supported && window_hsm_compatible(dev->dirty_rect))
        CSL_LCDC_PAR_SetSpeed(handle, &timingMem_hs);
    else
	CSL_LCDC_PAR_SetSpeed(handle, &timingMem);

	/*CP processor is setting IOCR6[19], which it shouldn't be doing. Remove this once the CP team fixes it.*/
	if (dev->te_supported) {
		board_sysconfig(SYSCFG_LCD, SYSCFG_ENABLE);
	}

	ret = CSL_LCDC_Update(handle,  (CSL_LCD_UPD_REQ_T*)req);
	if (CSL_LCD_OK != ret) {
		pr_info("CSL_LCDC_Update failed error: %d", ret);
		return err;
	}

	return 0;

}


/****************************************************************************
*
*  lcd_update_column
*
*  Update one column of LCD in non-DMA mode within dirty region.
*  Currently supports 8-bit and 16-bit bus width.
*
***************************************************************************/
static void lcd_update_column(LCD_dev_info_t * dev, unsigned int column)
{
	int i, stride;
	u32 source;

	stride = dev->width * dev->bits_per_pixel / 8;
	source = (u32)dev->frame_buffer.virtPtr + stride * dev->dirty_rect.top +
			column * dev->bits_per_pixel / 8;

	if (16 == LCD_device[0].bits_per_pixel) {
		uint16_t *p;
		if (readl(lcdc_base + REG_LCDC_CR) & REG_LCDC_CR_ENABLE_8_BIT_INTF) {
			 /* 8 bit bus */
			for (i = dev->dirty_rect.top; i <= dev->dirty_rect.bottom; i++) {
				p = (uint16_t *)source;
				lcd_write_data(*p >> 8);
				lcd_write_data(*p);
				source += stride;
			}
		} else {
			if (LCD_18BIT == LCD_Bus)
				CSL_LCDC_Enable_CE(handle, true);
			for (i = dev->dirty_rect.top; i <= dev->dirty_rect.bottom; i++) {
				p = (uint16_t *)source;
				lcd_write_data(*p);
				source += stride;
			}
			if (LCD_18BIT == LCD_Bus)
				CSL_LCDC_Enable_CE(handle, false);
		}
	} else if (32 == LCD_device[0].bits_per_pixel)  {
		uint32_t *p;
		uint32_t count;
		count = (dev->dirty_rect.bottom - dev->dirty_rect.top + 1);
		count &= ~1; /*Ignore one pixel in case count is odd*/
		CSL_LCDC_Enable_RGB888U(handle, true);
		for (i = 0; i < count; i++) {
			p = (uint32_t *)source;
			lcd_write_data(*p);
			source += stride;
		}
		CSL_LCDC_Enable_RGB888U(handle, false);
	} else {
		pr_info("bpp=%d is not supported\n", LCD_device[0].bits_per_pixel);
	}
}


/****************************************************************************/

//#ifdef CONFIG_BRCM_KPANIC_UI_IND
static void parallel_lock_csl_handle(void)
{
	CSL_LCDC_Lock_NoSem(handle);
}
static void parallel_unlock_csl_handle(void)
{
	CSL_LCDC_Unlock_NoSem(handle);
}

			
static void parallel_send_data(uint16_t * p, int img_width, int img_height, bool rle)
{
	int i;
	uint32_t rle_count;

	int len = img_width*img_height;

	lcd_setup_for_data(&LCD_device[0]);

	if (16 == LCD_device[0].bits_per_pixel) {
		uint16_t pixel_data;

		if (readl(lcdc_base + REG_LCDC_CR) & REG_LCDC_CR_ENABLE_8_BIT_INTF) {
			if (rle) {
				for (i = 0, rle_count =0; i < len; i++) {
					if (0 >= rle_count) {
						rle_count = *p++;
						pixel_data = *p++;
					}
					lcd_write_data(*p >> 8);
					lcd_write_data(pixel_data);
					--rle_count;
				}
			} else {
				for (i = 0; i < len; i++) {
					lcd_write_data(*p >> 8);
					lcd_write_data(*p++);
				}
			}
		} else {
			if (LCD_18BIT == LCD_Bus)
				CSL_LCDC_Enable_CE(handle, true);
			if (rle) {
				for (i = 0, rle_count =0; i < len; i++) {
					if (0 >= rle_count) {
						rle_count = *p++;
						pixel_data = *p++;
					}
					lcd_write_data(pixel_data);
					--rle_count;
				}
			} else {
				for (i = 0; i < len; i++)
					lcd_write_data(*p++);
			}
			if (LCD_18BIT == LCD_Bus)
				CSL_LCDC_Enable_CE(handle, false);
		}
	} else if (32 == LCD_device[0].bits_per_pixel)  {

		uint32_t pixel_data;

		CSL_LCDC_Enable_RGB888U(handle, true);
		if(rle) {
			for (i = 0, rle_count = 0; i < len; i++) {
				if (0 >= rle_count) {
					rle_count = *p++;
					rle_count |= (*p++ << 16);
					pixel_data = *p++;
					pixel_data |= (*p++ << 16);
				}
				lcd_write_data(pixel_data);
				--rle_count;
			}
		} else {
			for (i = 0; i < len; i++) {
				pixel_data = *p++;
				pixel_data |= (*p++ << 16);
				lcd_write_data(pixel_data);
			}
		}
		CSL_LCDC_Enable_RGB888U(handle, false);
	} else
		pr_info("lcd_send_data bpp=%d is not supported\n", LCD_device[0].bits_per_pixel);
}

//#endif //CONFIG_BRCM_KPANIC_UI_IND


#define MHz 1000000UL
#define KHz 1000UL

bool __is_setup_hold_more(unsigned long clk_Mhz, CSL_LCDC_PAR_SPEED_t *timing_ns, CSL_LCDC_PAR_SPEED_t *timing_ahb)
{
	int reqd = (timing_ns->wrSetup + timing_ns->wrHold) ? (((timing_ns->wrSetup + timing_ns->wrHold) * clk_Mhz + KHz - 1) / KHz) : 0;
	int actual = timing_ahb->wrSetup + timing_ahb->wrHold + 2;
	return (actual > reqd);
}

void convert_ns_to_ahb(unsigned long clk_rate, CSL_LCDC_PAR_SPEED_t timing_ns, CSL_LCDC_PAR_SPEED_t *timing)
{
	unsigned long clk_Mhz = clk_rate / MHz;
	CSL_LCDC_PAR_SPEED_t timing_ahb;

	/*Actual time on bus = (n + 1) cycles where n is the value computed below.
	Hence the values have to be rounded to the floor by integer division*/
	timing_ahb.rdHold = timing_ns.rdHold ? ((timing_ns.rdHold * clk_Mhz - 1) / KHz) : 0;
	timing_ahb.rdPulse = timing_ns.rdPulse ? ((timing_ns.rdPulse * clk_Mhz - 1) / KHz) : 0;
	timing_ahb.rdSetup = timing_ns.rdSetup ? ((timing_ns.rdSetup * clk_Mhz - 1) / KHz) : 0;
	timing_ahb.wrHold = timing_ns.wrHold ? ((timing_ns.wrHold * clk_Mhz - 1) / KHz) : 0;
	timing_ahb.wrPulse = timing_ns.wrPulse ? ((timing_ns.wrPulse * clk_Mhz - 1) / KHz) : 0;
	timing_ahb.wrSetup = timing_ns.wrSetup ? ((timing_ns.wrSetup * clk_Mhz - 1) / KHz) : 0;

#if 0
/*Hardware team suggested that this would not work out since the data lines would
 * be negated once the WR strobe is removed. Hence un-doing this change.*/

	if(timing_ns.wrSetup == 0) {
		/*Even if the required SetupTime = 0ns, BRCM LCDC imposes a minimum of
		1AHB cycle. Hence we can reduce the HoldTime by 1AHB cycle since the
		next Setup will immediately follow the current Hold duration*/
		if(timing_ahb.wrHold != 0)
			--timing_ahb.wrHold;
	} else if (__is_setup_hold_more(clk_Mhz, &timing_ns, &timing_ahb)) {
		if(timing_ahb.wrSetup != 0)
			--timing_ahb.wrSetup;
		else if (timing_ahb.wrHold != 0)
			--timing_ahb.wrHold;
	}
#endif

	*timing = timing_ahb;

	pr_info("%lu %lu %lu %lu %lu %lu\n", timing->rdHold, timing->rdPulse,
		timing->rdSetup, timing->wrHold, timing->wrPulse, timing->wrSetup);

}

/*
* lcd_calibrate_timing_values
*
* Converts values in CSL_LCDC_PAR_SPEED_t structures from ns to timing values
* in multiples of AHB cycle.
*
*/
void lcd_calibrate_timing_values(void)
{
	unsigned long clk_rate;
	struct clk *ahb_clk;

	ahb_clk = clk_get(NULL, "ahb");
	clk_rate = clk_get_rate(ahb_clk);
	pr_info("clk_speed=%lu\n", clk_rate);

	convert_ns_to_ahb(clk_rate, timingReg_ns, &timingReg);
	convert_ns_to_ahb(clk_rate, timingMem_ns, &timingMem);
    	if(hsm_supported)
	    convert_ns_to_ahb(clk_rate, timingMem_hs_ns, &timingMem_hs);
}

//#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA) || defined(CONFIG_BOARD_ACAR)
static void parallel_display_black_background(void)
{
	int transfer_count = PANEL_HEIGHT * PANEL_WIDTH;
	LCD_dev_info_t saved_dev_info = LCD_device[0];
#ifdef CONFIG_ARGB8888
	transfer_count *= 2;
#endif

	LCD_device[0].row_start = 0 - (PANEL_HEIGHT - LCD_HEIGHT) / 2;
	LCD_device[0].row_end = PANEL_HEIGHT - (PANEL_HEIGHT - LCD_HEIGHT) / 2;
	LCD_device[0].col_start = 0 - (PANEL_WIDTH - LCD_WIDTH) / 2;
	LCD_device[0].col_end = PANEL_WIDTH - (PANEL_WIDTH - LCD_WIDTH) / 2;
	lcd_setup_for_data(&LCD_device[0]);

	if (LCD_18BIT == LCD_Bus)
		CSL_LCDC_Enable_CE(handle, true);
	while (transfer_count--)
		lcd_write_data(0);
	if (LCD_18BIT == LCD_Bus)
		CSL_LCDC_Enable_CE(handle, false);
	LCD_device[0] = saved_dev_info;
}
//#endif //defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA) || defined(CONFIG_BOARD_ACAR)

/*Function related to panel*/
void lcd_send_cmd_sequence(Lcd_init_t *init)
{
    int i;

    for (i = 0; init[i].type != CTRL_END; i++) {
	switch (init[i].type) {
	case WR_CMND:
		lcd_write_cmd(LCD_CMD(init[i].cmd));
		break;
	case WR_DATA:
		lcd_write_data(LCD_CMD(init[i].data));
		break;
	case WR_CMND_DATA:
		lcd_write_cmd(LCD_CMD(init[i].cmd));
		lcd_write_data(LCD_DATA(init[i].data));
		break;
	case SLEEP_MS:
		if(ap_crashed)
			mdelay(init[i].data);
		else
			msleep(init[i].data);
		break;
	default:
		break;
	}
    }
}

static void parallel_init_panels(void)
{
	lcd_send_cmd_sequence(power_on_seq);
}

static void parallel_poweroff_panels(void)
{
	lcd_send_cmd_sequence(power_off_seq);
}


void lcd_setup_for_data(LCD_dev_info_t *dev)
{
#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
    /*Centre the display by offsetting the co-ordinates*/
    dev->row_start += (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->row_end += (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->col_start += (PANEL_WIDTH - LCD_WIDTH) / 2;
    dev->col_end += (PANEL_WIDTH - LCD_WIDTH) / 2;
#endif

	Lcd_init_t resetSeq[] = {
		RESET_SEQ,
		{CTRL_END, 0, 0},
	};
	lcd_send_cmd_sequence(resetSeq);

#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
    /* Roll back to (0,0) relative co-ordindates*/
    dev->row_start -= (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->row_end -= (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->col_start -= (PANEL_WIDTH - LCD_WIDTH) / 2;
    dev->col_end -= (PANEL_WIDTH - LCD_WIDTH) / 2;
#endif
}


static int parallel_probe(struct platform_device *pdev)
{
	int rc, i;
	struct resource *res;
	struct lcdc_platform_data_t *pdata;
	CSL_LCD_RES_T ret = 0;

	ctrl_init.maxBusWidth = LCD_Bus;
	ctrl_init.lcdc_base_address = (UInt32) lcdc_base;
	ret = CSL_LCDC_Init(&ctrl_init);
	if (CSL_LCD_OK != ret) {
		printk("Error: CSL_LCDC_Init returned %d\n", ret);
		return ret;
	}

	busCfg.cfg.busType = LCDC_BUS_Z80;
	busCfg.cfg.csBank = BUS_CH_0;
	busCfg.cfg.busWidth = LCD_Bus;
	if (32 == LCD_device[0].bits_per_pixel) {
		busCfg.cfg.colModeIn = LCD_IF_CM_I_RGB888U;
		busCfg.cfg.colModeOut = LCD_IF_CM_O_RGB888;
	} else {
		busCfg.cfg.colModeIn = LCD_IF_CM_I_RGB565P;
		busCfg.cfg.colModeOut = LCD_IF_CM_O_RGB666;
	}

	/*This will convert the timing values in ns to values as expected by LCDC
	in multiples of AHB cycles*/
	lcd_calibrate_timing_values();

	busCfg.speed = timingReg;
	busCfg.io.slewFast = FALSE;
	busCfg.io.driveStrength = 3;
	if (LCD_device[LCD_main_panel].te_supported) {
		pr_info("Tearing Effect control enabled\n");
		busCfg.teCfg.type = LCDC_TE_CTRLR;
		busCfg.teCfg.delay = 0;
		busCfg.teCfg.pinSel = 0;
		busCfg.teCfg.edgeRising = true;
	} else {
		pr_info("Tearing Effect control disabled\n");
		busCfg.teCfg.type = LCDC_TE_NONE;
	}
	ret = CSL_LCDC_PAR_Open(&busCfg, &handle);
	if (CSL_LCD_OK != ret) {
		printk("Error: CSL_LCDC_PAR_Open returned %d\n", ret);
		return ret;
	}

	CSL_LCDC_Enable_CE(handle, false);

	return 0;
}


static LCD_Interface_Drv_t parallel_interface = 
{
		.lcd_get_interface_info = parallel_get_interface_info,
		.lcd_init_panels = parallel_init_panels,
		.lcd_poweroff_panels = parallel_poweroff_panels,
		.lcd_display_black_background = parallel_display_black_background,
		.lcd_update_column = parallel_update_column,
		.lcd_update_rect_dma = parallel_update_rect_dma,
		.lcd_probe = parallel_probe,
		.lcd_send_data = parallel_send_data,
		.lcd_lock_csl_handle = parallel_lock_csl_handle,
		.lcd_unlock_csl_handle = parallel_unlock_csl_handle
};

LCD_Interface_Drv_t * get_parallel_interface()
{
	return &parallel_interface;
}
