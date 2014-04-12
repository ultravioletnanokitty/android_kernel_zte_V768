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
*  lcdc.c
*
*  PURPOSE:
*    This implements the code to use a Broadcom LCD host interface.
*
*  NOTES:
*    Uses device minor number to select panel:  0==main 1==sub
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include "lcdc.h"
#include <linux/broadcom/lcdc_dispimg.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

static char gBanner[] __initdata =
    KERN_INFO "lcdc: Broadcom LCD Controller Driver: 0.01";

static void __iomem *lcdc_base;
static int lcd_reset_gpio;

#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
static struct cpufreq_client_desc *lcdc_client;
#endif

/* globals to: communicate with the update thread */
/*   control access to LCD registers */
/*  manage DMA channel */
static int gInitialized;
#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock glcdfb_wake_lock;
#endif
static struct semaphore gDmaSema;
#ifdef CONFIG_REGULATOR
static struct regulator *lcdc_regulator = NULL;
#endif
bool ap_crashed = false;
/* forward func declarations */
static void lcd_exit(void);
static int lcd_init(void);
void lcd_pwr_on(void);
static long lcd_ioctl(struct file *file,
		     unsigned int cmd, unsigned long arg);
static int lcd_mmap(struct file *file, struct vm_area_struct *vma);
static int lcd_open(struct inode *inode, struct file *file);
static int lcd_release(struct inode *inode, struct file *file);
void lcd_display_test(LCD_dev_info_t *dev);
void lcd_display_rect(LCD_dev_info_t *dev, LCD_Rect_t *r);
static void lcd_update_column(LCD_dev_info_t *dev, unsigned int column);
static int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
			 PM_PowerLevel sysPowerLevel);


//shaojiang@wind-mobi.com 2011.11.14 begin
//support lcd compatible
//reviewed by liubing@wind-mobi.com
//static void lcd_send_cmd_sequence(Lcd_init_t *init);
//shaojiang@wind-mobi.com 2011.11.14 end

static inline void lcd_init_panels(void);
static inline void lcd_poweroff_panels(void);
static inline void lcd_wakeup_panels(void);
static void lcd_setup_for_data(LCD_dev_info_t *dev);
static void lcd_csl_cb(CSL_LCD_RES_T, CSL_LCD_HANDLE, void*);

#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA) || defined(CONFIG_BOARD_ACAR) || defined(CONFIG_BOARD_PASCAL) ||  defined(CONFIG_BOARD_L400)
void display_black_background(void);
#endif

static int __init lcdc_probe(struct platform_device *pdev);
static int lcdc_remove(struct platform_device *pdev);
int fb_notifier_call_chain(unsigned long val, void *v);


/**************************************************************************/
static CSL_LCDC_INIT_T ctrl_init;
static CSL_LCD_HANDLE handle;
static CSL_LCD_UPD_REQ_T req;
static CSL_LCDC_PAR_CTRL_T busCfg;

/*These structures will be updated during boot time based on the values
 * present in timingReg_ns and timingMem_ns and the AHB clock speed*/
CSL_LCDC_PAR_SPEED_t timingReg;
CSL_LCDC_PAR_SPEED_t timingMem;
/*High speed mode*/
CSL_LCDC_PAR_SPEED_t timingMem_hs;

//shaojiang@wind-mobi.com 2011.11.14 begin
//support lcd compatible
//reviewed by liubing@wind-mobi.com
#if defined(CONFIG_LCD_DRV_ALL)
static lcd_drv_info_t* lcd_drv[] = {
	&lcd_info_ili9486,
	&lcd_info_hx8357b,
	&lcd_info_hx8357c,
	0,
};
lcd_drv_info_t* cur_lcd_drv_ptr = &lcd_info_ili9486;

CSL_LCDC_PAR_SPEED_t timingReg_ns = {180, 188, 0, 36, 50, 14};
CSL_LCDC_PAR_SPEED_t timingMem_ns = {180, 188, 0, 36, 50, 14};

int LCD_num_panels = 1;
#define LCD_CMD(x) (x)
#define LCD_DATA(x) (x)

#define LCD_HEIGHT              cur_lcd_drv_ptr->lcd_dev->height
#define LCD_WIDTH               cur_lcd_drv_ptr->lcd_dev->width

#define PANEL_HEIGHT            LCD_HEIGHT
#define PANEL_WIDTH             LCD_WIDTH

extern uint32_t lcd_drv_index;
static LCD_dev_info_t* LCD_device;
LCD_Bus_t LCD_Bus = LCD_18BIT;
#endif
//shaojiang@wind-mobi.com 2011.11.14 end

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations lcd_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = lcd_ioctl,
	.mmap = lcd_mmap,
	.open = lcd_open,
	.release = lcd_release,
};

static struct platform_driver lcdc_driver = {
	.probe = lcdc_probe,
	.remove = lcdc_remove,
	.driver = {
		   .name = "LCDC",
		   .owner = THIS_MODULE,
		   },
};

static 	spinlock_t	lcdc_lock;

/* ---- Functions -------------------------------------------------------- */

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
//shaojiang@wind-mobi.com 2011.11.14 begin
//support lcd compatible
//reviewed by liubing@wind-mobi.com
void lcd_read_data(uint32_t data[], uint32_t len)
{
}
//shaojiang@wind-mobi.com 2011.11.14 end

/***************************************************************************
*  lcd_csl_cb
*
*   Callback from CSL after DMA transfer completion
*
***************************************************************************/
static void lcd_csl_cb(CSL_LCD_RES_T res, CSL_LCD_HANDLE handle, void *cbRef)
{
	up(&gDmaSema);
	if (CSL_LCD_OK != res)
		pr_info("lcd_csl_cb: res =%d\n", res);
}

static inline bool is_unaligned(LCD_dev_info_t * dev)
{
	return ((dev->dirty_rect.left * dev->bits_per_pixel >> 3) & 2);
}

static inline bool is_odd_total(LCD_dev_info_t * dev)
{
	return (((dev->dirty_rect.right - dev->dirty_rect.left + 1) * (dev->dirty_rect.bottom - dev->dirty_rect.top + 1)) % 2);
}

static inline bool is_odd_stride(LCD_dev_info_t * dev)
{
	return (((dev->dirty_rect.right - dev->dirty_rect.left + 1) * dev->bits_per_pixel >> 3) & 2);
}

static inline bool is_out_of_bounds(LCD_dev_info_t * dev)
{
	return ((dev->dirty_rect.right >= dev->width) || (dev->dirty_rect.left >= dev->width));
}

static inline bool is_tx_done_16(LCD_dev_info_t * dev)
{
	return ((dev->dirty_rect.right <= dev->dirty_rect.left) || (dev->dirty_rect.right == 0));
}

static inline bool is_tx_done_32(LCD_dev_info_t * dev)
{
	return (dev->dirty_rect.right < dev->dirty_rect.left);
}

/****************************************************************************
*
*  lcd_dev_dirty_rect
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/
static void lcd_dev_dirty_rect(LCD_dev_info_t * dev,
			       LCD_DirtyRect_t * dirtyRect)
{
	CSL_LCD_RES_T ret;
	int i;
	int err = -EINVAL;

	OSDAL_Dma_Buffer_List *buffer_list, *temp_list;

    if (unlikely(ap_crashed)) {
        return;
    }

    if(!dev) {
        pr_info("dev pointer is NULL\n");
        return;
    }
    if(!dirtyRect) {
        pr_info("dirtyRect pointer is NULL\n");
        return;
    }
	if ((dirtyRect->top > dirtyRect->bottom)
	    || ((dirtyRect->bottom - dirtyRect->top) >= dev->height)
	    || (dirtyRect->left > dirtyRect->right)
	    || ((dirtyRect->right - dirtyRect->left) >= dev->width)) {
		LCD_DEBUG("invalid dirty-rows params - ignoring\n");
		LCD_DEBUG("top = %u,  bottom = %u, left = %u, right = %u\n",
			  dirtyRect->top, dirtyRect->bottom,
			  dirtyRect->left, dirtyRect->right);
		return;
	}

	down_interruptible(&gDmaSema);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&glcdfb_wake_lock);
#endif

	dev->dirty_rect = *dirtyRect;
	dev->row_start = dev->dirty_rect.top % dev->height;
	dev->row_end = dev->dirty_rect.bottom % dev->height;

	/*If start address is aligned to odd boundary */
	if (is_unaligned(dev)) {
		dev->col_start = dev->dirty_rect.left;
		dev->col_end = dev->dirty_rect.left;
		lcd_setup_for_data(dev);
		lcd_update_column(dev, dev->dirty_rect.left);
		dev->dirty_rect.left += 1;
	}

	/*If length is odd multiple */
	if (is_odd_stride(dev) || ((dev->bits_per_pixel == 32) && is_odd_total(dev))) {
		dev->col_start = dev->dirty_rect.right;
		dev->col_end = dev->dirty_rect.right;
		lcd_setup_for_data(dev);
		lcd_update_column(dev, dev->dirty_rect.right);
		dev->dirty_rect.right -= 1;
	}

	if (is_out_of_bounds(dev) || ((32 != dev->bits_per_pixel) && is_tx_done_16(dev))
		|| ((32 == dev->bits_per_pixel) && is_tx_done_32(dev))) {
		/* Dirty columns have been transferred. No further processing required.*/
		goto done;
	}

	buffer_list = kzalloc((dev->dirty_rect.bottom - dev->dirty_rect.top + 1) * sizeof(OSDAL_Dma_Buffer_List), GFP_KERNEL);
	if (!buffer_list) {
		pr_info("Couldn't allocate memory for dma buffer list\n");
		goto done;
	}

	temp_list = buffer_list;
	for (i = dev->dirty_rect.top; i <= dev->dirty_rect.bottom; i++) {
		temp_list->buffers[0].srcAddr = (UInt32)dev->frame_buffer.physPtr + (i * dev->width + dev->dirty_rect.left) * dev->bits_per_pixel / 8;
		temp_list->buffers[0].destAddr = REG_LCDC_DATR_PADDR;
		temp_list->buffers[0].length =
		    (dev->dirty_rect.right - dev->dirty_rect.left +
		     1) * dev->bits_per_pixel / 8;
		temp_list->buffers[0].bRepeat = 0;
		temp_list++;
	}
	temp_list--;		/* Go back to the last list item to set interrupt = 1 */
	temp_list->buffers[0].interrupt = 1;

	req.buff = (void *)buffer_list;
	req.buffBpp = dev->bits_per_pixel;
	req.lineLenP = dev->dirty_rect.right - dev->dirty_rect.left + 1;
	req.lineCount = dev->dirty_rect.bottom - dev->dirty_rect.top + 1;
	req.timeOut_ms = 100;
	req.cslLcdCb = lcd_csl_cb;
	req.cslLcdCbRef = NULL;
	req.multiLLI = true;

	dev->col_start = dev->dirty_rect.left;
	dev->col_end = dev->dirty_rect.right;

	lcd_setup_for_data(dev);

//    if(hsm_supported && window_hsm_compatible(dev->dirty_rect))
//        CSL_LCDC_PAR_SetSpeed(handle, &timingMem_hs);
//    else
        CSL_LCDC_PAR_SetSpeed(handle, &timingMem);

	/*CP processor is setting IOCR6[19], which it shouldn't be doing. Remove this once the CP team fixes it.*/
	if (dev->te_supported) {
		board_sysconfig(SYSCFG_LCD, SYSCFG_ENABLE);
	}

	ret = CSL_LCDC_Update(handle, &req);
	if (CSL_LCD_OK != ret) {
		pr_info("CSL_LCDC_Update failed error: %d", ret);
		goto fail;
	}
	err = 0;

fail:
	kfree(buffer_list);

done:
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&glcdfb_wake_lock);
#endif
	if (err < 0)
		up(&gDmaSema);
}

void lcd_dev_dirty_rows(LCD_dev_info_t * dev, LCD_DirtyRows_t * dirtyRows)
{
	LCD_DirtyRect_t dirtyRect = {
		.top = dirtyRows->top,
		.bottom = dirtyRows->bottom,
		.left = 0,
		.right = LCD_device[LCD_main_panel].width - 1,
	};
	lcd_dev_dirty_rect(dev, &dirtyRect);
}

void lcd_dirty_rows(LCD_DirtyRows_t * dirtyRows)
{
	lcd_dev_dirty_rows(&LCD_device[LCD_main_panel], dirtyRows);
}

void lcd_dirty_rect(LCD_DirtyRect_t * dirtyRect)
{
	lcd_dev_dirty_rect(&LCD_device[LCD_main_panel], dirtyRect);
}

#ifdef CONFIG_BRCM_KPANIC_UI_IND
static void lcd_send_data(uint16_t * p, int len, bool rle)
{
	int i;
	uint32_t rle_count;

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

static int lcdc_set_dev_border(LCD_dev_info_t *dev, int img_width, int img_height)
{
	if((dev->width < img_width) || (dev->height < img_height)) {
		pr_warning("LCDC ERROR:img width %d is bigger than panel size %d!\n", img_width, dev->width);
		pr_warning("LCDC ERROR:img height %d is bigger than panel size %d!\n", img_height, dev->height);
		return -1;
	}

	dev->col_start = (dev->width - img_width) / 2;
	dev->col_end = dev->width - (dev->width - img_width) / 2 - 1;
	dev->row_start = (dev->height - img_height) / 2;
	dev->row_end = dev->height - (dev->height - img_height) / 2 - 1;

	return 0;
}

static int lcdc_img_index = -1;
bool lcdc_showing_dump(void)
{
	return (lcdc_img_index!=-1);
}

int lcdc_disp_img(int img_index)
{
    pr_info("lcdc_disp_img img_index=%d\n", img_index);
	uint16_t *data_ptr;
	int transfer_count;
	LCD_dev_info_t *dev;
	unsigned long irql ;

	CSL_LCDC_Lock(handle);
	spin_lock_irqsave( &lcdc_lock, irql ) ;
	CSL_LCDC_Lock_NoSem(handle);

	ap_crashed = true;
	if(!gInitialized) {
		lcd_pwr_on();
		lcd_init_panels();
		gInitialized = 1;
		int data = FB_BLANK_UNBLANK;
		struct fb_event event;
		event.data = &data;
		fb_notifier_call_chain(FB_EVENT_BLANK, &event);
	}

	dev = &LCD_device[0];
	switch (img_index)
	{
	case IMG_INDEX_AP_DUMP:
		if (lcdc_set_dev_border(dev, dump_ap_start_img_w, dump_ap_start_img_h))
			goto failure;
		data_ptr = ap_dump_start_img;
		transfer_count = dump_ap_start_img_w * dump_ap_start_img_h;
		break;

	case IMG_INDEX_CP_DUMP:
		if (lcdc_set_dev_border(dev, dump_cp_start_img_w, dump_cp_start_img_h))
			goto failure;
		data_ptr = cp_dump_start_img;
		transfer_count = dump_cp_start_img_w * dump_cp_start_img_h;
		break;

	case IMG_INDEX_END_DUMP:
		if (lcdc_set_dev_border(dev, dump_end_img_w, dump_end_img_h))
			goto failure;
		data_ptr = dump_end_img;
		transfer_count = dump_end_img_w * dump_end_img_h;
		break;

	default:
		goto failure;
	}


	lcd_setup_for_data(dev);

	lcd_send_data(data_ptr, transfer_count, true);

	lcdc_img_index = img_index;
	CSL_LCDC_Unlock_NoSem(handle);
	spin_unlock_irqrestore( &lcdc_lock, irql ) ;
	CSL_LCDC_Unlock(handle);
	
	return 0;
	
failure:
	CSL_LCDC_Unlock_NoSem(handle);
	spin_unlock_irqrestore( &lcdc_lock, irql ) ;
	CSL_LCDC_Unlock(handle);
	return -1;
}

#endif

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


/****************************************************************************
*
*  lcd_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
void __exit lcd_exit(void)
{
	LCD_PUTS("enter");

#ifdef CONFIG_REGULATOR
	if (!IS_ERR_OR_NULL(lcdc_regulator)) {
		regulator_put(lcdc_regulator);
		lcdc_regulator = NULL;
	}
#endif
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&glcdfb_wake_lock);
#endif

	platform_driver_unregister(&lcdc_driver);
}				/* lcd_exit */

static int lcd_alloc_fb(LCD_dev_info_t * dev)
{
	if (dev->frame_buffer.virtPtr != NULL)
		return 0;

	/* dma_alloc_writecombine allocates uncached, buffered memory, that is */
	/* io_remappable */
	/* DBFrame : */
	dev->frame_buffer.sizeInBytes =
	    dev->width * dev->height * 2 * dev->bits_per_pixel / 8;

	dev->frame_buffer.virtPtr = dma_alloc_writecombine(NULL,
							   dev->
							   frame_buffer.sizeInBytes,
							   &dev->
							   frame_buffer.physPtr,
							   GFP_KERNEL);

	pr_info
	    ("[lcd] lcd_alloc_fb size=%#x virtPtr = 0x%lx, physPtr = 0x%lx \r\n",
	     dev->frame_buffer.sizeInBytes, (long)dev->frame_buffer.virtPtr,
	     (long)dev->frame_buffer.physPtr);

	if (dev->frame_buffer.virtPtr == NULL)
		return -ENOMEM;

	if ((dev->frame_buffer.physPtr & ~PAGE_MASK) != 0) {
		panic("lcd_init: We didn't get a page aligned buffer");
		return -ENOMEM;
	}

	memset(dev->frame_buffer.virtPtr, 0, dev->frame_buffer.sizeInBytes);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lcd_early_suspend(struct early_suspend *h)
{
    if (likely(!ap_crashed))
	lcd_pm_update(PM_COMP_PWR_OFF, 0);
}

static void lcd_late_resume(struct early_suspend *h)
{
    if (likely(!ap_crashed))
	lcd_pm_update(PM_COMP_PWR_ON, 0);
}

static struct early_suspend lcd_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
	.suspend = lcd_early_suspend,
	.resume = lcd_late_resume,
};
#endif

/****************************************************************************
*
*  lcd_pm_update
*
*     Called by power manager to update component power level
*
***************************************************************************/
static inline int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
			 PM_PowerLevel sysPowerLevel)
{
	static PM_CompPowerLevel powerLevel = PM_COMP_PWR_ON;

	/* Nothing to do if power level did not change */
	if (compPowerLevel == powerLevel)
		return 0;

	/* Save new power level */
	powerLevel = compPowerLevel;
	switch (powerLevel) {
	case PM_COMP_PWR_OFF:
	case PM_COMP_PWR_STANDBY:
		{
			pr_info("\nLCDC: Power off panel\n");
			lcd_poweroff_panels();
#ifdef CONFIG_REGULATOR
			//shaojiang@wind-mobi.com 2012.01.13 begin
			//speed the lcd wakeup	process up
			//reviewed by liubing@wind-mobi.com
			//if (!IS_ERR_OR_NULL(lcdc_regulator))
			//	regulator_disable(lcdc_regulator);
			//shaojiang@wind-mobi.com 2012.01.13 end
#endif
#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
			cpufreq_bcm_dvfs_enable(lcdc_client);
#endif
			gInitialized = 0;
			break;
		}
	case PM_COMP_PWR_ON:
		{
			//shaojiang@wind-mobi.com 2012.01.13 begin
			//speed the lcd wakeup	process up
			//reviewed by liubing@wind-mobi.com
			pr_info("\nLCDC: Power on panel\n");
#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
			cpufreq_bcm_dvfs_disable(lcdc_client);
#endif
#ifdef CONFIG_REGULATOR
			//if (!IS_ERR_OR_NULL(lcdc_regulator))
			//	regulator_enable(lcdc_regulator);
#endif
			//lcd_pwr_on();
#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
			display_black_background();
#endif
			//lcd_init_panels();			
#if defined(CONFIG_BOARD_ACAR) || defined(CONFIG_BOARD_PASCAL) || defined(CONFIG_BOARD_L400) 
/* songjinguo@wind-mobi.com 2011.12.26 start */
/* fix bug 6330; lcd light time too long when wake up */
/* review by liubing */		
	//	display_black_background();
/* songjinguo@wind-mobi.com 2011.12.26 end */
#endif
			lcd_wakeup_panels();
			LCD_DirtyRect_t dirtyRect = {0, 0, LCD_WIDTH-1, LCD_HEIGHT-1};
			lcd_dirty_rect(&dirtyRect);
			gInitialized = 1;
			break;			
			//shaojiang@wind-mobi.com 2012.01.13 end
		}
	default:
		break;
	}

	return 0;
}

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

    //if(hsm_supported)
	 //   convert_ns_to_ahb(clk_rate, timingMem_hs_ns, &timingMem_hs);
}

static int __init lcdc_probe(struct platform_device *pdev)
{
	int rc, i;
	struct resource *res;
	struct lcdc_platform_data_t *pdata;
	CSL_LCD_RES_T ret;
//shaojiang@wind-mobi.com 2011.11.14 begin
//support lcd compatible
//reviewed by liubing@wind-mobi.com
#if defined(CONFIG_LCD_DRV_ALL)
	printk("lcdc_probe............lcd_drv_index=%d...........\n", lcd_drv_index);
	
	cur_lcd_drv_ptr = lcd_drv[lcd_drv_index];
	LCD_device = cur_lcd_drv_ptr->lcd_dev;
	LCD_Bus = cur_lcd_drv_ptr->lcd_bus;
	memcpy(&timingReg_ns, cur_lcd_drv_ptr->reg_timing, sizeof(timingReg_ns));	
	memcpy(&timingMem_ns, cur_lcd_drv_ptr->mem_timing, sizeof(timingReg_ns));
#endif	
	//pr_info("%s for %s\n", gBanner, LCD_panel_name);
//shaojiang@wind-mobi.com 2011.11.14 end
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get device resources\n");
		return -ENOENT;
	}

	lcdc_base = (void __iomem *)res->start;
	if (!lcdc_base) {
		dev_err(&pdev->dev, "couldn't get lcdc_base!\n");
		return -ENOENT;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform_data NULL!\n");
		return -ENOENT;
	}

	lcd_reset_gpio = pdata->gpio;
	if (!lcd_reset_gpio) {
		dev_err(&pdev->dev, "couldn't get GPIO!\n");
		return -ENOENT;
	}
	pr_info("lcd:probe lcdc_base = 0x%x gpio = %d\n", (int)lcdc_base,
		lcd_reset_gpio);

	/* Disable Tearing Effect control if the board doesn't support*/
//xiongbiao@wind-mobi.com 2011.12.20 begin
//add LCD TE funtion in lcd compatible.	
#if defined(CONFIG_LCD_DRV_ALL)
	if (!LCD_device->te_supported){
#else
	if (!pdata->te_supported) {
#endif	
//xiongbiao@wind-mobi.com 2011.12.20 end

		pr_info("lcdc: Disabling te_support since the board doesn't support\n");
		LCD_device[LCD_main_panel].te_supported = false;
	}

#ifdef CONFIG_REGULATOR
	lcdc_regulator = regulator_get(NULL, "lcd_vcc");
	if (!IS_ERR_OR_NULL(lcdc_regulator))
		regulator_enable(lcdc_regulator);
#endif

	/* Register our device with Linux */
	rc = register_chrdev(BCM_LCD_MAJOR, "lcd", &lcd_fops);
	if (rc < 0) {
		pr_warning("lcd: register_chrdev failed for major %d\n",
			   BCM_LCD_MAJOR);
		return rc;
	}
	/* Allocate memory for the framebuffers. */
	for (i = 0; i < LCD_num_panels; i++) {
		LCD_dev_info_t *dev = &LCD_device[i];

		rc = lcd_alloc_fb(dev);
		if (rc)
			return rc;
	}

#ifndef CONFIG_BCM_LCD_SKIP_INIT
	/*GPIO configuration must be done before CSL Init */
	lcd_pwr_on();
#endif

	sema_init(&gDmaSema, 1);

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

#ifndef CONFIG_BCM_LCD_SKIP_INIT
	lcd_init_panels();

#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
	display_black_background();
#endif

#endif

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&glcdfb_wake_lock, WAKE_LOCK_SUSPEND, "lcdfb_wake_lock");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&lcd_early_suspend_desc);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
	lcdc_client = cpufreq_bcm_client_get("lcdc");
	if (!lcdc_client) {
		pr_err("%s: cpufreq_bcm_client_get failed\n", __func__);
		return -EIO;
	}

	/* Turn off dvfs at boot up so that the boot process is not
	 * slowed down.
	 */
	cpufreq_bcm_dvfs_disable(lcdc_client);
#endif
	gInitialized = 1;

	return 0;
}

#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA) || defined(CONFIG_BOARD_ACAR) || defined(CONFIG_BOARD_PASCAL) \
     || defined(CONFIG_BOARD_L400) 
void display_black_background(void)
{
	int transfer_count = PANEL_HEIGHT * PANEL_WIDTH;
#ifdef CONFIG_ARGB8888
	transfer_count *= 2;
#endif
	LCD_dev_info_t saved_dev_info = LCD_device[0];
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
#endif

static int lcdc_remove(struct platform_device *pdev)
{
	gpio_free(lcd_reset_gpio);
	return 0;
}

/****************************************************************************
*
*  lcd_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init lcd_init(void)
{
	LCD_PUTS("enter");
	spin_lock_init( &lcdc_lock ) ;

	return platform_driver_register(&lcdc_driver);
}

/****************************************************************************
*
*  lcd_pwr_on
*
*     Power on controller
*
***************************************************************************/
void lcd_pwr_on(void)
{
	LCD_PUTS("enter");

	if (gInitialized) {
		pr_info("GPIO already configured\n");
		return;
	}

	board_sysconfig(SYSCFG_LCD, SYSCFG_INIT);

	gpio_request(lcd_reset_gpio, "LCD Reset");
	/* Configure the GPIO pins */
	gpio_direction_output(lcd_reset_gpio, 1);
	/* drop then raise the RESET line */
	gpio_set_value(lcd_reset_gpio, 0);
	if(ap_crashed)
		mdelay(2);
	else
		msleep(2);
	gpio_set_value(lcd_reset_gpio, 1);
	if(ap_crashed)
		mdelay(20);
	else
		msleep(20);

	board_sysconfig(SYSCFG_LCD, SYSCFG_DISABLE);
}

/****************************************************************************
*
*  lcd_ioctl - TODO - lots of stuff needs to be filled in
*
***************************************************************************/
static long lcd_ioctl(struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int err = 0;
	LCD_dev_info_t *dev = (LCD_dev_info_t *) file->private_data;

	LCD_DEBUG("[lcd] lcdioctl: %d  type: '%c' cmd: 0x%x\r\n", dev->panel,
		  _IOC_TYPE(cmd), _IOC_NR(cmd));

	switch (cmd) {
	case LCD_IOCTL_RESET:
		gpio_set_value(lcd_reset_gpio, (int)arg);
		break;

	case LCD_IOCTL_ENABLE_BACKLIGHT:
		break;

	case LCD_IOCTL_ENABLE_SUB_BACKLIGHT:
		break;

	case LCD_IOCTL_ENABLE_CS:
		break;

	case LCD_IOCTL_SCOPE_TIMEOUT:
		break;

	case LCD_IOCTL_INIT:
		lcd_init_panels();
		break;

	case LCD_IOCTL_INIT_ALL:
		break;

	case LCD_IOCTL_SETUP:
		break;

	case LCD_IOCTL_HOLD:
		break;

	case LCD_IOCTL_PULSE:
		break;

	case LCD_IOCTL_REG:
		{
			LCD_Reg_t r;

			if (copy_from_user(&r, (LCD_Reg_t *) arg, sizeof(r)) !=
			    0)
				return -EFAULT;

			break;
		}

	case LCD_IOCTL_RECT:
		{
			LCD_Rect_t r;

			if (copy_from_user(&r, (LCD_Rect_t *) arg, sizeof(r)) !=
			    0)
				return -EFAULT;

			lcd_display_rect(dev, &r);
			break;
		}

	case LCD_IOCTL_COLOR_TEST:
		break;

	case LCD_IOCTL_DIRTY_ROWS:
		{
			LCD_DirtyRows_t dirtyRows;

			if (copy_from_user(&dirtyRows, (LCD_DirtyRows_t *) arg,
					   sizeof (LCD_DirtyRows_t)) != 0)
				return -EFAULT;

			lcd_dev_dirty_rows(dev, &dirtyRows);
			break;
		}

	case LCD_IOCTL_DIRTY_RECT:
		{
			LCD_DirtyRect_t dirtyRect;

			if (copy_from_user(&dirtyRect, (LCD_DirtyRect_t *) arg,
					   sizeof (LCD_DirtyRect_t)) != 0)
				return -EFAULT;

			lcd_dev_dirty_rect(dev, &dirtyRect);
			break;
		}

	case LCD_IOCTL_PRINT_REGS:
		break;

	case LCD_IOCTL_PRINT_DATA:
		break;

	case LCD_IOCTL_PWR_OFF:
		break;

	case LCD_IOCTL_INFO:
		{
			LCD_Info_t lcdInfo;

			LCD_DEBUG("cmd=LCD_IOCTL_INFO arg=0x%08lX", arg);
			lcd_get_info(&lcdInfo);
			err =
			    copy_to_user((void *)arg, &lcdInfo,
					 sizeof(LCD_Info_t));
			break;
		}

	default:
		LCD_DEBUG("Unrecognized ioctl: '0x%x'\n", cmd);
		return -ENOTTY;
	}

	return err;
}

/****************************************************************************
*
*  lcd_get_info
*
*
*
***************************************************************************/
void lcd_get_info(LCD_Info_t * lcdInfo)
{
	LCD_dev_info_t *dev = &LCD_device[LCD_main_panel];

	lcdInfo->bitsPerPixel = dev->bits_per_pixel;
	lcdInfo->height = dev->height;
	lcdInfo->width = dev->width;
	lcdInfo->physical_width = dev->physical_width;
	lcdInfo->physical_height = dev->physical_height;
}

/****************************************************************************
*
*  lcd_mmap
*
*   Note that the bulk of this code came from the fb_mmap routine found in
*   drivers/video/fbmem.c
*
***************************************************************************/
static int lcd_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset;
	unsigned long start;
	unsigned long len;
	LCD_dev_info_t *dev;
	LCD_FrameBuffer_t *fb;

	/* vma->vm_start    is the start of the memory region, in user space */
	/* vma->vm_end      is one byte beyond the end of the memory region, in user space */
	/* vma->vm_pgoff    is the offset (in pages) within the vm_start to vm_end region */

	LCD_PUTS("enter");
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT)) {
		pr_info("lcd: vm_pgoff is out of range\n");
		return -EINVAL;
	}

	if (file == NULL || file->private_data == NULL) {
		pr_info("lcd: bad file pointer or LCD-device pointer\n");
		return -EINVAL;
	}

	dev = (LCD_dev_info_t *) file->private_data;
	fb = &dev->frame_buffer;

	/* Convert offset into a byte offset, rather than a page offset */
	offset = vma->vm_pgoff << PAGE_SHIFT;
	start = (unsigned long)fb->physPtr;	/* already page-aligned */
	pr_info("[lcd] lcd_mmap %lx %lx  \r\n", offset, start);

	len = PAGE_ALIGN(start + fb->sizeInBytes);

	if (offset > len) {
		/* The pointer requested by the user isn't inside of our frame buffer */
		LCD_DEBUG("offset is too large, offset = %lu, len = %lu\n",
			  offset, len);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	offset += start;
	vma->vm_pgoff = offset >> PAGE_SHIFT;

	if (0 != io_remap_pfn_range(vma,
				    vma->vm_start,
				    offset >> PAGE_SHIFT,
				    vma->vm_end - vma->vm_start,
				    vma->vm_page_prot)) {
		LCD_DEBUG("remap_page_range failed\n");
		return -EAGAIN;
	}

	return 0;
}

/****************************************************************************
*
*  lcd_open
*
***************************************************************************/
static int lcd_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);

	LCD_DEBUG("major = %d, minor = %d\n", imajor(inode), minor);

	/* minor number must match values for LCD_panel_t */
	if (minor < 0 || minor >= LCD_num_panels) {
		pr_info("lcd: bad minor number %d; range is 0..%d\n",
			minor, LCD_num_panels - 1);
		return -EINVAL;
	}
	/* set our private pointer to the correct LCD_dev_info_t */
	file->private_data = (void *)&LCD_device[minor];

	return 0;
}

/****************************************************************************
*
*  lcd_release
*
***************************************************************************/
static int lcd_release(struct inode *inode, struct file *file)
{
	LCD_PUTS("enter");
	return 0;
}

/****************************************************************************
*
*  lcd_get_framebuffer_addr
*
*   Gets the address of the primary frame buffer
*
***************************************************************************/
void *lcd_get_framebuffer_addr(int *frame_size, dma_addr_t * dma_addr)
{
	int rc;
	LCD_dev_info_t *dev = &LCD_device[LCD_main_panel];

	/*  Check if we have been initialized yet.  If not, another driver wants */
	/*  access to our framebuffer before we have been inited.  In this case, */
	/*  allocate the framebuffer now to avoid the other driver failing. */
	/* (lcd_alloc_fb() takes care not to reinitialize itself.) */

	rc = lcd_alloc_fb(dev);

	if (rc)
		return NULL;

	if (dma_addr)
		*dma_addr = dev->frame_buffer.physPtr;

	if (frame_size)
		*frame_size = dev->frame_buffer.sizeInBytes;

	pr_info("[lcd] lcd_get_frame %d 0x%x \r\n",
		dev->frame_buffer.sizeInBytes,
		(unsigned)dev->frame_buffer.virtPtr);

	return dev->frame_buffer.virtPtr;
}

//shaojiang@wind-mobi.com 2011.11.14 begin
//support lcd compatible
//reviewed by liubing@wind-mobi.com
/*Function related to panel*/
void lcd_send_cmd_sequence(Lcd_init_t *init)
//shaojiang@wind-mobi.com 2011.11.14 end
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

//shaojiang@wind-mobi.com 2012.01.13 begin
//support lcd compatible
//speed the lcd wakeup  process up
//reviewed by liubing@wind-mobi.com
static inline void lcd_init_panels(void)
{
#if defined(CONFIG_LCD_DRV_ALL)
	lcd_send_cmd_sequence(cur_lcd_drv_ptr->lcd_init_param);
#else
	lcd_send_cmd_sequence(power_on_seq);
#endif	
}

static inline void lcd_wakeup_panels(void)
{
#if defined(CONFIG_LCD_DRV_ALL)
	lcd_send_cmd_sequence(cur_lcd_drv_ptr->lcd_wakeup_param);
#else

#endif
}

static inline void lcd_poweroff_panels(void)
{
#if defined(CONFIG_LCD_DRV_ALL)
	lcd_send_cmd_sequence(cur_lcd_drv_ptr->lcd_power_off_param);
#else
	lcd_send_cmd_sequence(power_off_seq);
#endif
}
//shaojiang@wind-mobi.com 2011.11.14 end


static void lcd_setup_for_data(LCD_dev_info_t *dev)
{
#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
    /*Centre the display by offsetting the co-ordinates*/
    dev->row_start += (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->row_end += (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->col_start += (PANEL_WIDTH - LCD_WIDTH) / 2;
    dev->col_end += (PANEL_WIDTH - LCD_WIDTH) / 2;
#endif

//shaojiang@wind-mobi.com 2011.11.14 begin
//support lcd compatible
//reviewed by liubing@wind-mobi.com
#if defined(CONFIG_LCD_DRV_ALL)
	cur_lcd_drv_ptr->lcd_set_addr(dev);
#else
	Lcd_init_t resetSeq[] = {
		RESET_SEQ,
		{CTRL_END, 0, 0},
	};
	lcd_send_cmd_sequence(resetSeq);
#endif
//shaojiang@wind-mobi.com 2011.11.14 end

#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
    /* Roll back to (0,0) relative co-ordindates*/
    dev->row_start -= (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->row_end -= (PANEL_HEIGHT - LCD_HEIGHT) / 2;
    dev->col_start -= (PANEL_WIDTH - LCD_WIDTH) / 2;
    dev->col_end -= (PANEL_WIDTH - LCD_WIDTH) / 2;
#endif
}


/* XXX - temp hacks */
void lcd_display_test(LCD_dev_info_t * dev)
{
	int i, j;
	uint16_t *fb;
	LCD_DirtyRows_t dirtyRows;

	fb = dev->frame_buffer.virtPtr;
	dirtyRows.top = 0;
	dirtyRows.bottom = 479;
	for (i = 50; i < 100; i++)
		for (j = 50; j < 100; j++)
			if (i < j)
				fb[i * dev->width + j] = LCD_COLOR_YELLOW >> 1;

	lcd_dev_dirty_rows(dev, &dirtyRows);
}

void lcd_display_rect(LCD_dev_info_t * dev, LCD_Rect_t * r)
{
	int i, j;
	uint16_t *fb;
	LCD_DirtyRows_t dirtyRows;

	fb = dev->frame_buffer.virtPtr;


	if (r->top > dev->height)
		r->top = dev->height;

	if (r->left > dev->width)
		r->left = dev->width;

	for (i = r->top; i < (r->top + r->height); i++)
		for (j = r->left; j < (r->left + r->width); j++)
			fb[i * dev->width + j] = r->color;

	dirtyRows.top = r->top;
	dirtyRows.bottom = r->top + r->height;

	if (dirtyRows.bottom >= dev->height)
		dirtyRows.bottom -= 1;

	lcd_dev_dirty_rows(dev, &dirtyRows);
}

/****************************************************************************/

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom L2F50219P00 LCD Driver");
