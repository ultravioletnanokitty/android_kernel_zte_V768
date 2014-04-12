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
*  lcdc_common.c
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
#include <linux/broadcom/lcdc_dispimg.h>

#ifdef CONFIG_BRCM_KPANIC_UI_IND
#include "ap_start_dump_img.h"
#include "cp_start_dump_img.h"
#include "dump_end_img.h"
#endif

#include "lcdc_parallel.h"
#include "lcdc_dsi.h"
#include <mach/reg_syscfg.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h>
//xiongbiao@wind-mobi.com 2012.02.09 begin
//support mipi lcd compatible on L301
#if defined(CONFIG_LCD_DSI_DRV_ALL)||defined(CONFIG_LCD_L400_DSI_DRV_ALL)
#include "../../displays/lcd_dsi_drv.h"

extern lcd_dsi_drv_info_t* cur_lcd_dsi_drv_ptr;
extern lcd_dsi_drv_info_t* lcd_drv[];

LCD_dev_info_t *LCD_device;  

int LCD_num_panels = 1;
//extern int LCD_num_panels;

#else
extern LCD_dev_info_t LCD_device[];
extern const char *LCD_panel_name;
extern int LCD_num_panels;
#endif
//xiongbiao@wind-mobi.com 2012.02.09 end

static LCD_Interface_Drv_t * lcd_interface = NULL;
static struct semaphore gDmaSema;
static UInt32 lcd_dest_phy_addr = 0;


/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */
static char gBanner[] __initdata =
    KERN_INFO "lcdc: Broadcom LCD Controller Driver: 0.01";
void __iomem *lcdc_base;
static int lcd_reset_gpio;

#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
static struct cpufreq_client_desc *lcdc_client;
#endif

#if defined(CONFIG_LCD_DSI_DRV_ALL)
#define GPIO_LCD_ID0  (39)
#define GPIO_LCD_ID1  (33)
#elif defined(CONFIG_LCD_L400_DSI_DRV_ALL)
#define GPIO_LCD_ID0  (0)
#define GPIO_LCD_ID1  (3)	
#endif
 int lcd_id = 0;
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
static int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
			 PM_PowerLevel sysPowerLevel);

static int __init lcdc_probe(struct platform_device *pdev);
static int lcdc_remove(struct platform_device *pdev);
int fb_notifier_call_chain(unsigned long val, void *v);


/* ---- Functions -------------------------------------------------------- */
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

struct struct_frame_buf_mark {
u32 special_mark_1;
u32 special_mark_2;
u32 special_mark_3;
u32 special_mark_4;
void *p_fb; // it must be physical address
u32 resX;
u32 resY;
u32 bpp;    // color depth : 16 or 24
u32 frames; // frame buffer count : 2
};

static struct struct_frame_buf_mark  frame_buf_mark = {
.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0)),
.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0)),
.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0)),
.special_mark_4 = (('f' << 24) | ('b' << 16) | ('u' << 8) | ('f' << 0)),
.p_fb   = 0,
.resX   = 0,    // it has dependency on h/w
.resY   = 0,     // it has dependency on h/w
.bpp    = 0,      // it has dependency on h/w
.frames = 2
};

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
	CSL_LCD_UPD_REQ_T req;
	
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
		lcd_interface->lcd_update_column(dev, dev->dirty_rect.left);
		dev->dirty_rect.left += 1;
	}

	/*If length is odd multiple */
	if (is_odd_stride(dev) || ((dev->bits_per_pixel == 32) && (dev->dirty_rect.left == dev->dirty_rect.right))) {
		dev->col_start = dev->dirty_rect.right;
		dev->col_end = dev->dirty_rect.right;
		lcd_interface->lcd_update_column(dev, dev->dirty_rect.right);
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
		temp_list->buffers[0].destAddr = lcd_dest_phy_addr;
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


	err = lcd_interface->lcd_update_rect_dma(dev, &req);

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

/****************************************************************************/

#ifdef CONFIG_BRCM_KPANIC_UI_IND
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
	uint16_t *data_ptr;
	int transfer_count;
	int img_w,img_h;
	LCD_dev_info_t *dev;
	unsigned long irql ;

       pr_info("lcdc_disp_img img_index=%d\n", img_index);
	spin_lock_irqsave( &lcdc_lock, irql ) ;

	lcd_interface->lcd_lock_csl_handle();

	ap_crashed = true;
	if(!gInitialized) {
		int data = FB_BLANK_UNBLANK;
		struct fb_event event;

		lcd_pwr_on();
		lcd_interface->lcd_init_panels();
		gInitialized = 1;
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
		img_w = dump_ap_start_img_w;
		img_h = dump_ap_start_img_h;
		break;

	case IMG_INDEX_CP_DUMP:
		if (lcdc_set_dev_border(dev, dump_cp_start_img_w, dump_cp_start_img_h))
			goto failure;
		data_ptr = cp_dump_start_img;
		transfer_count = dump_cp_start_img_w * dump_cp_start_img_h;
		img_w = dump_cp_start_img_w;
		img_h = dump_cp_start_img_h;
		break;

	case IMG_INDEX_END_DUMP:
		if (lcdc_set_dev_border(dev, dump_end_img_w, dump_end_img_h))
			goto failure;
		data_ptr = dump_end_img;
		transfer_count = dump_end_img_w * dump_end_img_h;
		img_w = dump_end_img_w;
		img_h = dump_end_img_h;
		break;

	default:
		goto failure;
	}

	lcd_interface->lcd_send_data(data_ptr, img_w, img_h, true);

	lcdc_img_index = img_index;

	lcd_interface->lcd_unlock_csl_handle();

	spin_unlock_irqrestore( &lcdc_lock, irql ) ;
	return 0;

failure:

	lcd_interface->lcd_unlock_csl_handle();

	spin_unlock_irqrestore( &lcdc_lock, irql ) ;
	return -1;
}

#endif //CONFIG_BRCM_KPANIC_UI_IND



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

}/* lcd_exit */

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
*  lcd_pm_update
*
*     Called by power manager to update component power level
*
***************************************************************************/
static int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
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
			//lcd_poweroff_panels();
			lcd_interface->lcd_poweroff_panels();
#ifndef CONFIG_REGULATOR
			if (!IS_ERR_OR_NULL(lcdc_regulator))
				regulator_disable(lcdc_regulator);
#endif
#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
			cpufreq_bcm_dvfs_enable(lcdc_client);
#endif
			gInitialized = 0;
			break;
		}
	case PM_COMP_PWR_ON:
		{
			pr_info("\nLCDC: Power on panel\n");
#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
			cpufreq_bcm_dvfs_disable(lcdc_client);
#endif
#ifndef CONFIG_REGULATOR
			if (!IS_ERR_OR_NULL(lcdc_regulator))
				regulator_enable(lcdc_regulator);
#endif
			if(lcd_id==1)
			lcd_pwr_on();//only for lide lcd 20121024
#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
			lcd_interface->lcd_display_black_background();
#endif
			lcd_interface->lcd_init_panels();
#if defined(CONFIG_BOARD_ACAR)
			lcd_interface->lcd_display_black_background();
#endif
			gInitialized = 1;
			break;
		}
	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

//#ifdef CONFIG_LCD_CONTROLLER_DSI
#if 0
#define SYSCFG_DSICR 0x08880038
#define CLK_DSIPLL_ENABLE  0x08140220
#define CLK_MIPIDSI_CMI_CLK_EN 0x0814012C
#endif

static void lcd_early_suspend(struct early_suspend *h)
{	
//songjinguo@wind-mobi.com 20120301 start
//fix standby larger current; 
//review by liubing
/* yaogangxiang@wind-mobi.com 2012.03.17 begin */
//the input gpio shouldnot be set to output value 
/*
    unsigned int reg_sysctl = 0;
	if(lcd_id){

//		writel((readl(ADDR_SYSCFG_IOCR1) & ~0x9),	ADDR_SYSCFG_IOCR1);
//		pr_info("suspend to change lcd_id =%d\n", lcd_id);
		switch( lcd_id){
		case 1:	
//			gpio_request(GPIO_LCD_ID1, "lcd_id_gpio3");
			gpio_direction_output(GPIO_LCD_ID1, 1);
			break;
		case 2:
//			gpio_request(GPIO_LCD_ID0, "lcd_id_gpio0");
//			gpio_direction_output(GPIO_LCD_ID0, 1);
			break;
		case 3:
//			gpio_request(GPIO_LCD_ID1, "lcd_id_gpio3");
			gpio_direction_output(GPIO_LCD_ID1, 1);
//			gpio_request(GPIO_LCD_ID0, "lcd_id_gpio0");
			gpio_direction_output(GPIO_LCD_ID0,1);
			break;
		default:
			break;

		}
	}

*///songjinguo@wind-mobi.com 20120301 end	
/* yaogangxiang@wind-mobi.com 2012.03.17 end */

    #if defined(CONFIG_LCD_L400_DSI_DRV_ALL)
	/* feixiaoping@wind-mobi.com 2012.04.09 begin */
	//when GPIO_LCD_ID0=1 &GPIO_LCD_ID1=1 current is large about 5ma larger 
	//when lcd enter sleep config this two gpio as input status otherwise gpio status isn't right
	  static bool SetGpioFlag=1;
	  if(SetGpioFlag==1)
	  	{
	  		SetGpioFlag=0;
			switch(lcd_id)
			{
			case 2:	
			case 0:
				writel((readl(ADDR_SYSCFG_IOCR1) & ~0x9), ADDR_SYSCFG_IOCR1);
				gpio_request(GPIO_LCD_ID1,"lcd_id_gpio3");
				gpio_direction_input(GPIO_LCD_ID1);

				gpio_request(GPIO_LCD_ID0,"lcd_id_gpio0");
				gpio_direction_input(GPIO_LCD_ID0);
				//printk("####enter_sleep_fxp##");
				break;
			default:
				break;

			}
	  	}
	/* feixiaoping@wind-mobi.com 2012.04.09 end */
	#endif
    if (likely(!ap_crashed))
	lcd_pm_update(PM_COMP_PWR_OFF, 0);
//#ifdef CONFIG_LCD_CONTROLLER_DSI
#if 0
    writel(0, IO_ADDRESS(SYSCFG_DSICR));
	writel(0, IO_ADDRESS(CLK_MIPIDSI_CMI_CLK_EN));
   	writel(0, IO_ADDRESS(CLK_DSIPLL_ENABLE));
#endif
}

static void lcd_late_resume(struct early_suspend *h)
{
//#ifdef CONFIG_LCD_CONTROLLER_DSI
#if 0
     writel(1, IO_ADDRESS(SYSCFG_DSICR));
     writel(1, IO_ADDRESS(CLK_MIPIDSI_CMI_CLK_EN));
     writel(1, IO_ADDRESS(CLK_DSIPLL_ENABLE));
#endif
    if (likely(!ap_crashed))
	lcd_pm_update(PM_COMP_PWR_ON, 0);
}

static struct early_suspend lcd_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
	.suspend = lcd_early_suspend,
	.resume = lcd_late_resume,
};

#endif //CONFIG_HAS_EARLYSUSPEND

/* yaogangxiang@wind-mobi.com 2012.08.23 begin */
static int read_lcdid(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int ret;

	ret = sprintf(buf, "%d", lcd_id);

	return ret;
}
/* yaogangxiang@wind-mobi.com 2012.08.23 end */

static int __init lcdc_probe(struct platform_device *pdev)
{
	int rc, i;
	struct resource *res;
	struct lcdc_platform_data_t *pdata;
	CSL_LCD_RES_T ret = 0;
	int width = 0;
	int height = 0;
	int bpp = 0;
#if 0 // defined(CONFIG_LCD_DSI_DRV_ALL)
#define GPIO_LCD_ID0  (39)
#define GPIO_LCD_ID1  (33)
//#elif defined(CONFIG_LCD_L400_DSI_DRV_ALL)
#define GPIO_LCD_ID0  (0)
#define GPIO_LCD_ID1  (3)	
#endif

/* yaogangxiang@wind-mobi.com 2012.08.23 begin */
    static struct proc_dir_entry *lcd_pdir;
    static struct proc_dir_entry *lcdid_point;
/* yaogangxiang@wind-mobi.com 2012.08.23 end */


//xiongbiao@wind-mobi.com 2012.02.09 begin
//support mipi lcd compatible on L301
#if defined(CONFIG_LCD_DSI_DRV_ALL)||defined(CONFIG_LCD_L400_DSI_DRV_ALL)
/* yaogangxiang@wind-mobi.com 2012.03.17 begin */
//for XAL vesion cannot enter sleep mode, it power off when entering sleep
  
    writel((readl(ADDR_SYSCFG_IOCR1) & ~0x9), ADDR_SYSCFG_IOCR1);
	
	
    gpio_request(GPIO_LCD_ID1,"lcd_id_gpio3");
    gpio_direction_input(GPIO_LCD_ID1);
	
    gpio_request(GPIO_LCD_ID0,"lcd_id_gpio0");
    gpio_direction_input(GPIO_LCD_ID0);
	/* yaogangxiang@wind-mobi.com 2012.03.17 end */
   
	/* feixiaoping@wind-mobi.com 2012.04.09 begin */
	//add baolongda lcd id value
	if((gpio_get_value(GPIO_LCD_ID0) == 1) && (gpio_get_value(GPIO_LCD_ID1) == 1))
	{
		lcd_id = 0;//jing dong fang 
	}	
	/* feixiaoping@wind-mobi.com 2012.04.09 end */
	else if((gpio_get_value(GPIO_LCD_ID0) == 0) && (gpio_get_value(GPIO_LCD_ID1) == 1))
	{
		/* feixiaoping@wind-mobi.com 2012.10.23 begin */
		//now the boot2.img and kernel all use the new driver to support old lcd and new problem lide lcd .
		lcd_id = 1;//feixiaoping 20121020 1 ->2 
		/* feixiaoping@wind-mobi.com 2012.10.23 end */
	}
	else if((gpio_get_value(GPIO_LCD_ID0) == 1) && (gpio_get_value(GPIO_LCD_ID1) == 0))
	{
		lcd_id = 2;//truly lcd 

	}
	else if((gpio_get_value(GPIO_LCD_ID0) == 0) && (gpio_get_value(GPIO_LCD_ID1) == 0))
	{
		lcd_id = 3;
	}
	
	printk("============mipi lcd_id=%d\n", lcd_id);	
	cur_lcd_dsi_drv_ptr = lcd_drv[lcd_id];
	LCD_device = cur_lcd_dsi_drv_ptr->lcd_dev;
	//LCD_panel_name = cur_lcd_dsi_drv_ptr->panel_name;
#endif

#if defined(CONFIG_LCD_DSI_DRV_ALL)||defined(CONFIG_LCD_L400_DSI_DRV_ALL)
	pr_info("%s for %s\n", gBanner, cur_lcd_dsi_drv_ptr->panel_name);
	printk("%s for %s\n", gBanner, cur_lcd_dsi_drv_ptr->panel_name);
#else
	pr_info("%s for %s\n", gBanner, LCD_panel_name);
	printk("%s for %s\n", gBanner, LCD_panel_name);
#endif
	//xiongbiao@wind-mobi.com 2012.02.09 end

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
	if (!pdata->te_supported) {
		pr_info("lcdc: Disabling te_support since the board doesn't support\n");
		LCD_device[LCD_main_panel].te_supported = false;
	}

#ifdef CONFIG_REGULATOR
	lcdc_regulator = regulator_get(NULL, "lcd_vcc");
	if (!IS_ERR_OR_NULL(lcdc_regulator))
		regulator_enable(lcdc_regulator);
#endif

/* yaogangxiang@wind-mobi.com 2012.08.23 begin */
	lcd_pdir = proc_mkdir("lcddir", NULL);
    if (lcd_pdir == NULL)
		pr_err("create /proc/lcddir error!\n");

	lcdid_point = create_proc_read_entry("lcd_id",0444,lcd_pdir,read_lcdid,NULL);
	if (lcdid_point == NULL) {
		pr_err("create /proc/lcddir error!\n");
		remove_proc_entry("lcddir",NULL);
	}
/* yaogangxiang@wind-mobi.com 2012.08.23 end */

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
	frame_buf_mark.p_fb = (void*)(LCD_device[0].frame_buffer.physPtr - CONFIG_SDRAM_BASE_ADDR);  // it has dependency on project
	frame_buf_mark.bpp = LCD_device[0].bits_per_pixel;     // it has dependency on h/w 

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

#ifndef CONFIG_BCM_LCD_SKIP_INIT
	/*GPIO configuration must be done before CSL Init */
	lcd_pwr_on();
#endif

#ifdef CONFIG_LCD_CONTROLLER_LEGACY
		lcd_interface = get_parallel_interface();		
#else
		lcd_interface = get_dsic_interface();
#endif

	if (!lcd_interface) {
		pr_err("%s: get lcd interface function failed\n", __func__);
		return -EIO;
	}

	sema_init(&gDmaSema, 1);

	lcd_interface->lcd_get_interface_info(&lcd_dest_phy_addr, &width, &height, &bpp);

	frame_buf_mark.resX   = width; 
	frame_buf_mark.resY   = height;    
	frame_buf_mark.bpp    = bpp;   
	
	lcd_interface->lcd_probe(pdev);

#ifndef CONFIG_BCM_LCD_SKIP_INIT
	//lcd_init_panels();
	lcd_interface->lcd_init_panels();
#if defined(CONFIG_ENABLE_QVGA) || defined(CONFIG_ENABLE_HVGA)
	//display_black_background();
	lcd_interface->lcd_display_black_background();
#endif
#endif

	if(ret)
	{
		dev_err(&pdev->dev, "Probe Initialization failed\n");
		return -ENOENT;
	}
	gInitialized = 1;
	return 0;
}

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
		lcd_interface->lcd_init_panels();
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

			if(!dev) 
			{
		        	pr_info("dev pointer is NULL\n");
		        	return;
		    	}
			
			/*  LCD_DEBUG("top = %u,  bottom = %u\n", dirtyRows->top, dirtyRows->bottom); */
			if ((dirtyRows.top > dirtyRows.bottom)
			    || ((dirtyRows.bottom - dirtyRows.top) >= dev->height)) {
				LCD_DEBUG("invalid dirty-rows params - ignoring\n");
				LCD_DEBUG("top = %u,  bottom = %u\n",
					  dirtyRows.top, dirtyRows.bottom);

				return;
			}

			lcd_dev_dirty_rows(dev, &dirtyRows);
			break;
		}

	case LCD_IOCTL_DIRTY_RECT:
		{
			LCD_DirtyRect_t dirtyRect;

			if (copy_from_user(&dirtyRect, (LCD_DirtyRect_t *) arg,
					   sizeof (LCD_DirtyRect_t)) != 0)
				return -EFAULT;
			
			if(!dev) 
			{
		        	pr_info("dev pointer is NULL\n");
		        	return;
		    	}

			if ((dirtyRect.top > dirtyRect.bottom)
			    || ((dirtyRect.bottom - dirtyRect.top) >= dev->height)
			    || (dirtyRect.left > dirtyRect.right)
			    || ((dirtyRect.right - dirtyRect.left) >= dev->width)) {
				LCD_DEBUG("invalid dirty-rows params - ignoring\n");
				LCD_DEBUG("top = %u,  bottom = %u, left = %u, right = %u\n",
					  dirtyRect.top, dirtyRect.bottom,
					  dirtyRect.left, dirtyRect.right);
				return;
			}

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
MODULE_DESCRIPTION("Broadcom LCD Driver");
