/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	arch/arm/mach-bcm215xx/board-athenaray.c
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

/*
 * AthenaRay board specific driver definitions
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cpufreq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/reg_irq.h>

#include <linux/serial_8250.h>
#include <mach/hardware.h>
#include <mach/reg_syscfg.h>
#include <mach/reg_auxmic.h>
#include <asm/mach/map.h>
#include <mach/setup.h>
#include <mach/gpt.h>
#include <linux/broadcom/gpt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/leds.h>
#if defined (CONFIG_TOUCHSCREEN_MSI_ATMEGA168)
#include <linux/i2c/brcmmsi_atmega168_ts.h>
#define BOARD_VIRTUALKEY

#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
#include <linux/mpu.h>
#endif
/* songjinguo@wind-mobi.com 2011.11.28 start */
/* enable headset detected; */
/* review by liubing */
#define CONFIG_BRCM_HEADSET_MODULE 1
/* songjinguo@wind-mobi.com 2011.11.28 end */
#if defined (CONFIG_ANDROID_PMEM)
#include <linux/android_pmem.h>
#include <linux/dma-mapping.h>

#if defined (CONFIG_BMEM)
#define PMEM_ADSP_SIZE (2 * PAGE_SIZE)
#else
#define PMEM_ADSP_SIZE (1024 * 1024 * 8)
#endif

#endif

#ifdef CONFIG_MMC_BCM
#include <mach/sdio.h>
#include <mach/reg_clkpwr.h>
#endif

#if defined(CONFIG_MFD_MAX8986)
#include <linux/power_supply.h>
#include <linux/mfd/max8986/max8986.h>
#include <linux/broadcom/max8986/max8986-audio.h>
#endif

#ifdef CONFIG_USB_DWC_OTG
#include <mach/usbctl.h>
#endif
#ifdef CONFIG_KEYBOARD_BCM
#include <mach/bcm_keymap.h>
#include <plat/bcm_keypad.h>
#endif
#ifdef CONFIG_BACKLIGHT_FAN5626
#include <linux/pwm_backlight.h>
#endif

/* yaogangxiang@wind-mobi.com 2011.10.21 begin */
/* add lcd backlight driver head file */
#ifdef CONFIG_BACKLIGHT_TPS61165
#include <linux/tps61165_bl.h>
#endif
/* yaogangxiang@wind-mobi.com 2011.10.21 end */

#if defined (CONFIG_BCM_AUXADC)
#include <plat/bcm_auxadc.h>
#endif
#include "device.h"

/* yuanlan@wind-mobi.com 2011.06.28 begin */
/* Add gt818 Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_GT818
#include <linux/gt818-ts.h>
#endif
/* yuanlan@wind-mobi.com 2011.06.28 end */
/* xiongbiao@wind-mobi.com 2011.12.13 begin*/
/* Add FT5x06 Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_FT5X06
#include <linux/ft5x06-ts.h>
#define BOARD_VIRTUALKEY_FT5X06
#endif
/* xiongbiao@wind-mobi.com 2011.12.13 end*/

/* xiongbiao@wind-mobi.com 2011.12.15 begin*/
/* Add MXT140 Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_MXT140
#include <linux/i2c/mxt140_ts.h>
#define BOARD_VIRTUALKEY_MXT140
#endif
/* xiongbiao@wind-mobi.com 2011.12.15 end*/

//xiongbiao@wind-mobi.com 2011.12.16 begin
//Add lis33de g-sensor support	
#if defined (CONFIG_SENSORS_LIS33DE)
#include <linux/lis33de.h>
#endif
//xiongbiao@wind-mobi.com 2011.12.16 begin


#if defined (CONFIG_SENSORS_AK8975)
#include <linux/akm8975.h>
#endif

#if defined (CONFIG_SENSORS_AK8962)
#include <linux/akm8962.h>
#endif


#if defined(CONFIG_SENSORS_BMA250)
#include <linux/bma250.h>
#endif

#if defined(CONFIG_SENSORS_AL3006)
#include <linux/al3006.h>
#endif


#if defined(CONFIG_BRCM_HEADSET)  || defined(CONFIG_BRCM_HEADSET_MODULE)
#include <plat/brcm_headset_pd.h>
#endif

#include <linux/spinlock.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_SPI
#include <plat/bcm_spi.h>
#endif

#include <plat/syscfg.h>
#include <plat/timer.h>

#if defined (CONFIG_I2C_GPIO)
#include <linux/i2c-gpio.h>
#endif


#if defined (CONFIG_BCM_PWM)
#include <plat/bcm_pwm_block.h>
#endif

#include <plat/bcm_i2c.h>

#ifdef CONFIG_BCM215XX_DSS
#include <plat/bcm_lcdc.h>
#endif

#define BCM2091_CLK_REQ_PASS_THOUGH 36
#define GPS_CNTIN_CLK_ENABLE 37

#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))
#include <linux/broadcom/bcmblt-rfkill.h>
#include <plat/bcm_rfkill.h>
#endif

#include <linux/broadcom/types.h>
#include <linux/broadcom/bcmtypes.h>

#if defined(CONFIG_BRCM_FUSE_SYSPARM)
#include <linux/broadcom/bcm_fuse_sysparm.h>
#endif


#if defined(CONFIG_USB_ANDROID)
#include <linux/usb/android.h>
#endif

#ifdef CONFIG_BCM_GPIO_VIBRATOR
#include <linux/timed_gpio.h>
#endif

#if defined(CONFIG_BCMI2CNFC)
#include <linux/bcmi2cnfc.h>
#endif 

#if defined(CONFIG_TMD2771X) || defined(CONFIG_TMD2771X_MODULE)
#include <linux/tmd2771x.h>
#endif
/* songjinguo@wind-mobi.com 2011.12.31 start */
/* modify for bug 6183; csp:479804 */
/* review by liubing */
#define   BATTERY_ADC_FIX       
/* songjinguo@wind-mobi.com 2011.12.31 end */

#include <linux/clk.h>
#include <mach/clkmgr.h>
#define   SYSCFG_IOCR0_GPIO53_GPEN8_L_MUX	    	    (1 << 13)
#define   SYSCFG_IOCR2_OTGCTRL1_GPIO41_MUX(x)	    (((x) & 0x3) << 14)
#define   SYSCFG_IOCR0_GPIO53_GPEN8_H_MUX	    	    (1 << 21)

/*yaogangxiang@wind-mobi.com 2011.10.21 begin */
/*added L300 backlight driver  & define GPIO 17 for control */
#ifdef CONFIG_BACKLIGHT_TPS61165
#define GPIO_LCD_BACKLIGHT (17)
#endif
/*yaogangxiang@wind-mobi.com 2011.10.21 end */

#define ADDR_GPIO_TYPE0				(HW_GPIO_BASE + 0x0)
#define ADDR_GPIO_TYPE1				(HW_GPIO_BASE + 0x4)
#define ADDR_GPIO_TYPE2				(HW_GPIO_BASE + 0x8)
#define ADDR_GPIO_TYPE3				(HW_GPIO_BASE + 0xc)

#define ADDR_GPIO_OUT_VAL1				(HW_GPIO_BASE + 0x10)
#define ADDR_GPIO_OUT_VAL2				(HW_GPIO_BASE + 0x14)

#define ADDR_GPIO_INPUT_PULL_EN0				(HW_GPIO_BASE + 0x20)
#define ADDR_GPIO_INPUT_PULL_EN1				(HW_GPIO_BASE + 0x24)
#define ADDR_GPIO_INPUT_PULL_TYPE0				(HW_GPIO_BASE + 0x28)
#define ADDR_GPIO_INPUT_PULL_TYPE1				(HW_GPIO_BASE + 0x2c)


#define GPIO_BLUE_LED 			(11)

//heweimao@wind-mobi.com 2012.02.21 begin
//L400 green led gpio
//reviewed by liubing@wind-mobi.com 
/* yaogangxiang@wind-mobi.com 2012.01.04 begin */
//modify L301 LEDs GPIO config
//red-led ==> gpio30, green-led ==> gpio19
#ifdef CONFIG_BOARD_L400_TYPE_EDN10
//heweimao@wind-mobi.com 2011.11.04 begin
//L300 green led gpio
//reviewed by liubing@wind-mobi.com 
#define GPIO_GREEN_LED 			(19)
//#define GPIO_GREEN_LED 			(31)
//heweimao@wind-mobi.com 2011.11.04 end
//heweimao@wind-mobi.com 2012.02.21 end
#else
#define GPIO_GREEN_LED 			(22)
#endif

#define GPIO_KEYPAD_BL_LED 	(24)

//heweimao@wind-mobi.com 2012.02.21 begin
//L400 red led gpio
//reviewed by liubing@wind-mobi.com 
//heweimao@wind-mobi.com 2011.11.04 begin
//L300 red led gpio
//reviewed by liubing@wind-mobi.com 
//#define GPIO_RED_LED 			(39)
#define GPIO_RED_LED 			(39)
//heweimao@wind-mobi.com 2011.11.04 end
//heweimao@wind-mobi.com 2012.02.21 end
/* yaogangxiang@wind-mobi.com 2012.01.04 end */

#define FREQ_MHZ(mhz)		((mhz)*1000UL*1000UL)
#define FREQ_KHZ(khz)		((khz)*1000UL)

/*
 * BITMAP to indicate the available GPTs on AP
 * Reset the bitmap to indicate the GPT not to be used on AP
 */
#define GPT_AVAIL_BITMAP         0x3F

extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
void __init gpt_init(struct gpt_base_config *base_config);

//#ifdef CONFIG_BCM_4319_DRIVER

extern void bcmsdhc_sdio_host_force_scan(struct platform_device *pdev, bool on);
void bcm_wlan_power_on(int val);
void bcm_wlan_power_off(int val);
int bcm_set_core_power(unsigned int bcm_core, unsigned int pow_on,
		       unsigned int reset);
//#endif /* 4319 */
#ifdef CONFIG_MMC_BCM
extern int bcmsdhc_cfg_card_detect(void __iomem *ioaddr, u8 ctrl_slot);
extern int bcmsdhc_external_reset(void __iomem *ioaddr, u8 ctrl_slot);
extern int bcmsdhc_enable_int(void __iomem *ioaddr, u8 ctrl_slot);
extern void *cam_mempool_base;
extern int camera_id;
int bcmsdhc_set_sys_interface(u8 ctrl_slot);
int bcmsdhc_request_cfg_pullup(void __iomem *ioaddr, u8 ctrl_slot);
int bcmsdhc_finish_cfg_pullup(void __iomem *ioaddr, u8 ctrl_slot);

static struct bcmsdhc_platform_data bcm21553_sdhc_data1 = {
	.base_clk = FREQ_MHZ(48),
	.flags = SDHC_DEVTYPE_SDIO | SDHC_MANUAL_SUSPEND_RESUME | SDHC_DISABLE_PED_MODE,
	.cd_pullup_cfg = SDCD_PULLUP | SDCD_UPDOWN_ENABLE,
	.irq_cd = -1,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};

#if !defined(CONFIG_MTD_ONENAND) && !defined(CONFIG_MTD_NAND)
/*
 * SDHC2 is used for eMMC and SDHC2 shares pin mux with FLASH(OneNAND)
 * So both OneNAND and SDHC2 cannot co-exist
 */

static struct bcmsdhc_platform_data bcm21553_sdhc_data2 = {
	.base_clk = FREQ_MHZ(50),
	.flags = SDHC_DEVTYPE_EMMC | SDIO_CARD_ALWAYS_PRESENT,
	.cd_pullup_cfg = SDCD_PULLUP | SDCD_UPDOWN_ENABLE,
	.irq_cd = -1,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};
#endif /*CONFIG_MTD_ONENAND */

static struct bcmsdhc_platform_data bcm21553_sdhc_data3 = {
	.base_clk = FREQ_MHZ(26),	//modify by songjinguo
	.flags = SDHC_DEVTYPE_SD | SDHC_DISABLE_PED_MODE,
	.cd_pullup_cfg = 0,	//using external pull-up
	.irq_cd = 5,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};
#endif /*CONFIG_MMC_BCM */

#if defined(CONFIG_TMD2771X) || defined(CONFIG_TMD2771X_MODULE)

#define TMD2771X_GPIO		(14)            //(15)

struct tmd2771x_platform_data tmd2771x_pdata = {
	.i2c_pdata = {.i2c_spd = I2C_SPD_400K,},
    .power_on = 0, /* TMD2771X_PON, */
	.wait_enable = TMD2771X_WEN,
	.wait_long = 0, /*TMD2771X_WLONG,*/
	.wait_time = 0xEE,      /* 256 - (50ms / 2.72) */
	/* Proximity */
	.ps_enable = 0, /*TMD2771X_PEN, */
	.ps_interrupt_h_thres = 0,
	.ps_interrupt_l_thres = 0,
	.ps_interrupt_enable = TMD2771X_PIEN,
	.ps_time = 0xFF,
	.ps_interrupt_persistence = 0, /* (2 << TMD2771X_PPERS_SHIFT), */
      //liutao@wind-mobi.com 2012-4-18 begin
	//screen display slowly or can't wake up 
	//yuanlan@wind-mobi.com 2012-4-18 review
	//liutao@wind-mobi.com 2012-3-2 begin
	//proximity is sometimes invalid
	//yuanlan@wind-mobi.com 2012-3-2 review
	//.ps_pulse_count = 8,
	//liutao@wind-mobi.com 2012-3-15 begin
	//improve proximity sensor calibrate
	//yuanlan@wind-mobi.com 2012-3-15 review
	//.ps_pulse_count = 3,
	.ps_pulse_count = 2,
	//liutao@wind-mobi.com 2012-3-15 end
	//liutao@wind-mobi.com 2012-3-2 end
	//liutao@wind-mobi.com 2012-4-18 end
	.ps_drive_strength = TMD2771X_PDRIVE_100MA,
	.ps_diode = TMD2771X_PDIODE_CH1_DIODE,
	/* Ambient Light */
	.als_enable = 0, /* TMD2771X_AEN, */
	.als_interrupt_h_thres = 0,
	.als_interrupt_l_thres = 0,
	.als_interrupt_enable = TMD2771X_AIEN,
	.als_time = 0xEE,		//  modify by wilson    /* 256 - (50ms / 2.72) */    /* 256 - (200ms / 2.72) */
	//.als_time = 0xB6,		/* 256 - (200ms / 2.72) */
	.als_interrupt_persistence = 0, /* (2 << TMD2771X_APERS_SHIFT), */
	.als_gain = TMD2771X_AGAIN_1X,
	.glass_attenuation = 50,  //modify by wilson  100
	//.glass_attenuation = 100,
};
#endif



//#ifdef CONFIG_BCM_4319_DRIVER

#define BCM4325_BT 1
#define BCM4325_WLAN 2
#define BCM4325_BT_RESET 38
#define BCM4325_WLAN_RESET 38
#define GPIO_WLAN_BT_REG_ON 38
#define BT_VREG_CTRL	21
#define BT_RST_CHECK	20
int bcm_set_core_power(unsigned int bcm_core, unsigned int pow_on,
		       unsigned int reset)
{
	unsigned gpio_rst, gpio_rst_another;

	if ((reset != 2) && (reset != 1)) {
		pr_info("%s: Error!! BAD Argument. ", __FUNCTION__);
		return -EINVAL;
	}

	switch (bcm_core) {
	case BCM4325_BT:
		gpio_rst = BCM4325_BT_RESET;
		gpio_rst_another = BCM4325_WLAN_RESET;
		break;

	case BCM4325_WLAN:
		gpio_rst = BCM4325_WLAN_RESET;
		gpio_rst_another = BCM4325_BT_RESET;
		break;

	default:
		pr_err("bcm_power: Unknown bcm core!\n");
		return -1;
	}

/*    mutex_lock(&bcm4325_pwr_lock); */
	//xiaocui@wind-mobi.com 2012-04-10 brgin
	//fix bug 10433,copy from L300
	gpio_request(GPIO_WLAN_BT_REG_ON, "sdio1_wlan_reset");
	//xiaocui@wind-mobi.com 2012-04-10 end
	/* Since reg on is coupled, check whether the other core is ON before
	   touching it */
	if ((gpio_get_value(gpio_rst_another) == 0) && ((reset == 1)|| (reset == 2))) {

		/* Make GPIO38 to out direction, and set value 0 */
			writel(readl(ADDR_SYSCFG_IOCR10) &
				~SYSCFG_IOCR10_BSC3_GPIOH_ENABLE,
				ADDR_SYSCFG_IOCR10);
			//xiaocui@wind-mobi.com 2012-04-10 brgin
			//fix bug 10433,copy from L300	
			//gpio_request(GPIO_WLAN_BT_REG_ON, "sdio1_wlan_reset");
			//xiaocui@wind-mobi.com 2012-04-10 end
			gpio_direction_output(GPIO_WLAN_BT_REG_ON, pow_on);



		/* enable WLAN_BT_REG_ON */
///		gpio_direction_output(GPIO_WLAN_BT_REG_ON, pow_on);
		pr_info
		    ("bcm_power: Set WLAN_BT_REG_ON %s because %s is OFF now.\n",
		     gpio_get_value(GPIO_WLAN_BT_REG_ON) ? "High" : "Low",
		     bcm_core ? "BT" : "WLAN");
		msleep(150);
	}
	/* enable specified core */
	gpio_direction_output(gpio_rst, pow_on);
	pr_info("bcm_power: Set %s %s\n",
		bcm_core ? "WLAN_RESET" : "BT_RESET",
		gpio_get_value(gpio_rst) ? "High [chip out of reset]" :
		"Low [put into reset]");

			gpio_free(GPIO_WLAN_BT_REG_ON);


/*    mutex_unlock(&bcm4325_pwr_lock); */

	return 0;
}

#define BCM_RESET 2
#define BCM_POWER 1
#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

/** @addtogroup BoardBravaAPIGroup
	@{
*/
/**
* @brief	Power ON the WLAN
*
* @param val	Start/Power ON
*/
void bcm_wlan_power_on(int val)
{

	int err = 0;
	pr_info("%s: Enter.\n ", __FUNCTION__);



		if( (gpio_get_value(BT_RST_CHECK) == 0) )
		{
			gpio_direction_output(BT_VREG_CTRL, 1);
			msleep(500);
		}


	switch (val) {
	case BCM_POWER:
		pr_info("SHRI %s: WIFI POWER UP!!  \n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, TRUE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		bcmsdhc_sdio_host_force_scan(&bcm21553_sdhc_slot1, true);
		break;
	case BCM_RESET:
		pr_info("%s: WIFI DRIVER START!!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, TRUE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		break;
	default:
		pr_info("%s: INVALID ARG!!\n ", __FUNCTION__);





		}
	/* Note: that the platform device struct has to be exported from where it is defined */
	/* The below function would induce a forced mmc_rescan to detect the newly */
	/* powered up card. */

}


/**
* @brief	Power OFF the WLAN
*
* @param val	Stop/Power OFF
*/
void bcm_wlan_power_off(int val)
{

	int err = 0;
	pr_info("%s: Enter.\n ", __FUNCTION__);



	switch (val) {
	case BCM_POWER:
		pr_info("%s: WIFI POWER DOWN!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, FALSE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		bcmsdhc_sdio_host_force_scan(&bcm21553_sdhc_slot1, false);
		break;
	case BCM_RESET:
		pr_info("%s: WIFI DRIVER STOP!!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, FALSE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		break;
	default:
		pr_info("%s: INVALID ARG!!\n ", __FUNCTION__);

	}



		if( (gpio_get_value(BT_RST_CHECK) == 0) )
		{
			gpio_direction_output(BT_VREG_CTRL, 0);
			msleep(500);
		}


	/* Note: that the platform device struct has to be exported from where it is defined */
	/* The below function would induce a forced mmc_rescan to detect the newly */
	/* powered up card. */

}
/** @} */
EXPORT_SYMBOL(bcm_wlan_power_on);
EXPORT_SYMBOL(bcm_wlan_power_off);

//#endif /* 4319 */
#ifdef CONFIG_KEYBOARD_BCM
static void bcm_keypad_config_iocr(int row, int col)
{
	row = (1 << row) - 1;
	col = (1 << col) - 1;
	/* Set lower "row" & "col" number of bits to 1 to indicate configuration of keypad */
	writel((SYSCFG_IOCR1_KEY_ROW(row) | SYSCFG_IOCR1_KEY_COL(col)),
				ADDR_SYSCFG_IOCR1);
}

static struct bcm_keymap newKeymap[] = {
/* yaogangxiang@wind-mobi.com 2012.03.17 begin */
//the XAL version cannot enter sleep mode 
//	{BCM_KEY_ROW_0, BCM_KEY_COL_0, "Back Key", KEY_BACK},
	{BCM_KEY_ROW_0, BCM_KEY_COL_0, "unused", 0},
/* yaogangxiang@wind-mobi.com 2012.03.17 end */
	{BCM_KEY_ROW_0, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_7, "unused", 0},
	//heweimao@wind-mobi.com 2011.11.09 begin
	//L300 Volume Down --> Volume Up
	//reviewed by liubing@wind-mobi.com
	//{BCM_KEY_ROW_1, BCM_KEY_COL_0, "Volume Down", KEY_VOLUMEDOWN},
	{BCM_KEY_ROW_1, BCM_KEY_COL_0, "Volume Up", KEY_VOLUMEUP},
	//heweimao@wind-mobi.com 2011.11.09 end
	{BCM_KEY_ROW_1, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_7, "unused", 0},
	//heweimao@wind-mobi.com 2011.11.09 begin
	//L300  Volume Up --> Volume Down
	//reviewed by liubing@wind-mobi.com
	//{BCM_KEY_ROW_2, BCM_KEY_COL_0, "Volume Up", KEY_VOLUMEUP},
	{BCM_KEY_ROW_2, BCM_KEY_COL_0, "Volume Down", KEY_VOLUMEDOWN},
	//heweimao@wind-mobi.com 2011.11.09 end
	{BCM_KEY_ROW_2, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_7, "unused", 0},
/* yaogangxiang@wind-mobi.com 2012.03.17 begin */
//the XAL version cannot enter sleep mode 
//	{BCM_KEY_ROW_3, BCM_KEY_COL_0, "Home Key", KEY_HOME},
	{BCM_KEY_ROW_3, BCM_KEY_COL_0, "unused", 0},
/* yaogangxiang@wind-mobi.com 2012.03.17 end */
	{BCM_KEY_ROW_3, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_7, "unused", 0},
};

/* yaogangxiang@wind-mobi.com 2012.03.17 begin */
//the XAL version cannot enter sleep mode 
static struct bcm_keypad_platform_info bcm215xx_keypad_data = {
	.row_num = 3,
	.col_num = 1,
	.keymap = newKeymap,
	.iocr_cfg = bcm_keypad_config_iocr,
	.bcm_keypad_base = (void *)__iomem IO_ADDRESS(BCM21553_KEYPAD_BASE),
};
/* yaogangxiang@wind-mobi.com 2012.03.17 end */

#endif

#ifdef CONFIG_BACKLIGHT_FAN5626
struct platform_device bcm_backlight_devices = {
	.name = "pwm-backlight",
	.id = 0,
};

static struct platform_pwm_backlight_data bcm_backlight_data = {
	/* backlight */
	.pwm_id = 1,
	.max_brightness = 32,	/* Android calibrates to 32 levels*/
	.dft_brightness = 32,
	/*set pwm clock rate to 200Hz. min value of the operating range of the max1561
         *step-up converter(200Hz - 200KHz) which takes this PWM as an input signal*/
	.pwm_period_ns =  5000000,
};
#endif /*CONFIG_BACKLIGHT_FAN5626 */

/* songjinguo@wind-mobi.com 2011.10.21 begin */
/* add lcd backlight driver for platform data */
#ifdef CONFIG_BACKLIGHT_TPS61165
static struct platform_tps61165_backlight_data bcm_backlight_data = {
	.max_brightness = 32,	/* Android calibrates to 32 levels*/
	.dft_brightness = 32,
	.gpio_lcd_backlight =GPIO_LCD_BACKLIGHT,
};

struct platform_device bcm_backlight_devices = {
	.name = "tps61165-backlight",
	.id = 0,
	.dev = {
     .platform_data = &bcm_backlight_data,
	 },
};
#endif /*CONFIG_BACKLIGHT_TPS61165 */
/* songjinguo@wind-mobi.com 2011.10.21 end */

#ifdef CONFIG_BCM_PWM
static struct pwm_platform_data pwm_dev = {
	.max_pwm_id = 6,
	.syscfg_inf = board_sysconfig,
};
#endif


#ifdef CONFIG_LEDS_GPIO
// shaojiang@wind-mobi.com 2012.01.09 begin
// fix bug 7061: leds display error    
//reviewed by liubing@wind-mobi.com

//heweimao@wind-mobi.com 20120612 begin
//remove red greed LED for stk  
#ifdef CONFIG_REMOVE_SANTOK_LED
static struct gpio_led gpio_leds[] = {
	{
		.name	= "blue",
		.default_trigger = "timer",
		.gpio	= GPIO_BLUE_LED ,
		.active_low = 0,
		.retain_state_suspended = 1,
	},
	{
		.name	= "keypad_bl",
		.default_trigger = "timer",
		.gpio	= GPIO_KEYPAD_BL_LED ,
		.active_low = 0,
	}
};
//heweimao@wind-mobi.com 20120612 end
#else 
static struct gpio_led gpio_leds[] = {
	{
		.name	= "blue",
		.default_trigger = "timer",
		.gpio	= GPIO_BLUE_LED ,
		.active_low = 0,
		.retain_state_suspended = 1,
	},
	{
		.name	= "green",
		.default_trigger = "timer",
		.gpio	= GPIO_GREEN_LED ,
		.active_low = 0,
		.retain_state_suspended = 1,
	},
	{
		.name	= "red",
		.default_trigger = "timer",
		.gpio	= GPIO_RED_LED ,
		.active_low = 0,
		.retain_state_suspended = 1,
	},
	{
		.name	= "keypad_bl",
		.default_trigger = "timer",
		.gpio	= GPIO_KEYPAD_BL_LED ,
		.active_low = 0,
	}
};
#endif

// shaojiang@wind-mobi.com 2012.01.09 end

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};
#endif

#if defined (CONFIG_MTD_ONENAND_BCM_XBOARD)
static struct resource onenand_resources[] = {
	{
	 .start = IO_ADDRESS(BCM21553_ONENAND_BASE),
	 .end = IO_ADDRESS(BCM21553_ONENAND_BASE) + SZ_128K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

struct flash_platform_data onenand_data = {
	.parts = NULL,
	.nr_parts = 0,
};

struct platform_device athenaray_device_onenand = {
	.name = "onenand",
	.resource = onenand_resources,
	.num_resources = ARRAY_SIZE(onenand_resources),
	.dev = {
		.platform_data = &onenand_data,
		},
};
#endif

#ifdef CONFIG_MPU_SENSORS_MPU3050
#define IH_GPIO_BASE 0
#define MPUIRQ_GPIO 10
#define ACCEL_IRQ_GPIO 4
#define COMPASS_IRQ_GPIO 53
static struct mpu_platform_data mpu3050_data = {
      .int_config = 0x10,
      .orientation = { -1, 0, 0,
                       0,  1, 0,
                       0,  0, -1},
}; 

/*accl*/
static struct ext_slave_platform_data inv_mpu_kxtf9_data ={
      .bus = EXT_SLAVE_BUS_SECONDARY,
      .orientation = {  0, -1, 0,
                        1, 0, 0,
                        0, 0, 1},
};

/*compass*/
static struct ext_slave_platform_data inv_mpu_ak8975_data ={
      .bus = EXT_SLAVE_BUS_PRIMARY,
      .orientation = { 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1},
};
#endif //#ifdef CONFIG_MPU_SENSORS_MPU3050

#if defined (CONFIG_MTD_NAND_BRCM_NVSRAM)
static struct resource nand_resources[] = {
	{
	 .start = BCM21553_NAND_BASE,
	 .end = BCM21553_NAND_BASE + SZ_32M - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct flash_platform_data nand_data = {
	.parts = NULL,
	.nr_parts = 0,
};

static struct platform_device bcm21553_device_nand = {
	.name = "bcm-nand",
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev = {
		.platform_data = &nand_data,
		},
};
#endif

#if defined (CONFIG_ANDROID_PMEM)
static struct android_pmem_platform_data android_pmem_pdata = {
        .name = "pmem_adsp",
        .no_allocator = 0,
        .cached = 0,
};

static struct platform_device android_pmem_device = {
        .name = "android_pmem",
        .id = 0,
        .dev = {.platform_data = &android_pmem_pdata},
};
#endif
#ifdef CONFIG_SERIAL_8250
static void acar_serial_gpio_set(bool enable)
{
    bcm_gpio_pull_up(48,enable);
    bcm_gpio_pull_up(49,enable);
    bcm_gpio_pull_up(50,enable);
    bcm_gpio_pull_up(51,enable);

    gpio_request(48, "gpio_48");
	gpio_direction_output(48, enable);
	gpio_free(48);
    gpio_request(49, "gpio_49");
	gpio_direction_output(49, enable);
	gpio_free(49);
    gpio_request(50, "gpio_50");
	gpio_direction_output(50, enable);
	gpio_free(50);
    gpio_request(51, "gpio_51");
	gpio_direction_output(51, enable);
	gpio_free(51);
}
#endif

#if (defined (CONFIG_TOUCHSCREEN_MSI_ATMEGA168)|| defined (CONFIG_TOUCHSCREEN_FT5X06) || defined(CONFIG_TOUCHSCREEN_GT818) || defined(CONFIG_TOUCHSCREEN_MXT140))
#define PEN_IRQ_GPIO (29)
#define PEN_RST_GPIO (28)
#define TP_RST PEN_RST_GPIO
#define TP_SCL_GPIO  (26)
#define TP_SDA_GPIO  (27)
#endif
#if defined (CONFIG_TOUCHSCREEN_MSI_ATMEGA168)



 int atmega168_msi_pen_reset(void)
{
    gpio_request(PEN_RST_GPIO, "tp_reset");
	gpio_direction_output(PEN_RST_GPIO,1);
	mdelay(1);
    gpio_direction_output(PEN_RST_GPIO,0);
	mdelay(50);
	gpio_direction_output(PEN_RST_GPIO,1);
	gpio_free(PEN_RST_GPIO);
	return 0;
}
EXPORT_SYMBOL(atmega168_msi_pen_reset);

static int atmega168_msi_pen_pulldown(void)
{

    gpio_request(PEN_IRQ_GPIO, "tp_pendown");
    gpio_direction_output(PEN_IRQ_GPIO,0);
    //gpio_set_value(PEN_IRQ_GPIO,0);
    mdelay(70);
	
	return 0;
}

static int atmega168_msi_pen_pullup(void)
{

  
    gpio_direction_output(PEN_IRQ_GPIO,1);
    //gpio_set_value(PEN_IRQ_GPIO,0);
    mdelay(70);
	gpio_free(PEN_IRQ_GPIO);
	return 0;
}


 int atmega168_msi_pen_down_state(void)
{
    gpio_get_value(PEN_IRQ_GPIO);
	return (gpio_get_value(PEN_IRQ_GPIO)) ? 1 : 0;
}
 EXPORT_SYMBOL(atmega168_msi_pen_down_state);

static int atmega168_msi_init_platform_hw(void)
{
	pr_info("atmega168_msi_init_platform_hw\n");
	bcm_gpio_pull_up_down_enable(TP_SCL_GPIO, false);
	bcm_gpio_pull_up_down_enable(TP_SDA_GPIO, false);
	bcm_gpio_pull_up_down_enable(PEN_IRQ_GPIO, false);

	gpio_request(PEN_IRQ_GPIO, "atmega168_msi_ts");
	gpio_direction_input(PEN_IRQ_GPIO);
	set_irq_type(GPIO_TO_IRQ(PEN_IRQ_GPIO), IRQF_TRIGGER_FALLING);
	return 0;
}

 EXPORT_SYMBOL(atmega168_msi_init_platform_hw);
static struct atmega168_msi_platform_data atmega168_msi_pdata = {
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
	.platform_init  = atmega168_msi_init_platform_hw,
	.x_size         = 320,
	.y_size         = 510,
	.x_min		= 0,
	.y_min		= 0,
	.x_max		= 320,
	.y_max		= 510,
	.max_area	= 0xff,
	.blen           = 33,
	.threshold      = 70,
	.voltage        = 2700000,              /* 2.8V */
	.orient         = 0x7,
	.platform_reset   = atmega168_msi_pen_reset,
	.pen_pullup = atmega168_msi_pen_pullup,
	.pen_pulldown = atmega168_msi_pen_pulldown,
	.attb_read_val   = atmega168_msi_pen_down_state,
};

#endif

#ifdef CONFIG_TOUCHSCREEN_GT818

static struct goodix_i2c_rmi_platform_data racer2_gt818_pdata = {
        .i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
        .scr_x_min = 0,
        .scr_x_max = 240,
        .scr_y_min = 0,
        .scr_y_max = 320,
        .int_port = PEN_IRQ_GPIO,
	    .tp_rst   = TP_RST,
        
        //.virtual_keys = racer2_virtualkeys,
        //.init_platform_hw = racer2_init_gt818_hw,
        //.exit_platform_hw = racer2_exit_gt818_hw,
       // .wakeup_chip = racer2_gt818_wakeup_chip,
};
#endif

/* xiongbiao@wind-mobi.com 2011.12.13 begin*/
/* Add FT5x06 Capacitive touch panel support */
/* Add power management support for FT5x06 driver */
#ifdef CONFIG_TOUCHSCREEN_FT5X06
static int acar_init_ft5x06_hw(void)
{
    gpio_request(PEN_IRQ_GPIO, "ft5x06-ts");
    gpio_direction_input(PEN_IRQ_GPIO);
	bcm_gpio_pull_up_down_enable(PEN_IRQ_GPIO, true);
	bcm_gpio_pull_up(PEN_IRQ_GPIO, true);
    set_irq_type(GPIO_TO_IRQ(PEN_IRQ_GPIO), IRQF_TRIGGER_FALLING);

	return 0;
}

static void acar_exit_ft5x06_hw(void)
{
	gpio_free(PEN_IRQ_GPIO);
}

static int acar_ft5x06_wakeup_chip(void)
{
	gpio_direction_output(PEN_IRQ_GPIO, 0);
	msleep(5);
	gpio_direction_input(PEN_IRQ_GPIO);

	return 0;
}

static int acar_virtualkeys[] = {
	 KEY_MENU ,KEY_HOME, KEY_BACK, KEY_SEARCH
};

static struct ft5x06_platform_data acar_ft5x06_pdata = {
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
	.scr_x_min = 0,
	.scr_x_max = 320,
	.scr_y_min = 0,
	.scr_y_max = 480,
	.virtual_keys = acar_virtualkeys,
	.init_platform_hw = acar_init_ft5x06_hw,
	.exit_platform_hw = acar_exit_ft5x06_hw,
	.wakeup_chip = acar_ft5x06_wakeup_chip,
};


/* Add FT5x06 Virtual Key support */
#endif
/* xiongbiao@wind-mobi.com 2011.12.13 end*/

/* xiongbiao@wind-mobi.com 2011.12.15 begin*/
/* Add MXT140 Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_MXT140
#define PEN_IRQ_GPIO   29
static int mxt140_init_platform_hw(void)
{
	gpio_request(PEN_IRQ_GPIO, "mxt140_ts");
	gpio_direction_input(PEN_IRQ_GPIO);
	bcm_gpio_pull_up(PEN_IRQ_GPIO, true);
	bcm_gpio_pull_up_down_enable(PEN_IRQ_GPIO, true);
	set_irq_type(GPIO_TO_IRQ(PEN_IRQ_GPIO),IRQF_TRIGGER_FALLING);
	return 0;
}

static struct mxt140_platform_data mxt140_platform_data = {
	.i2c_pdata = {.i2c_spd = I2C_SPD_400K,},
	.platform_init  = mxt140_init_platform_hw,
	.x_line         = 15,
	.y_line         = 9,
	.x_size         = 520,
	.y_size         = 320,
	.x_min			= 0,
	.y_min		= 0,
	.x_max		= 480,
	.y_max		= 320,
	.max_area	= 0xff,
	.blen           = 33,
	.threshold      = 50,
	.voltage        = 2700000,              /* 2.8V */
	.orient         = MXT140_DIAGONAL,
};
/* Add MXT140 Virtual Key support */

#endif
/* xiongbiao@wind-mobi.com 2011.12.15 end*/


#if defined(CONFIG_I2C_GPIO)
static struct i2c_board_info __initdata i2c_gpio_devices[] = {  
#if defined (CONFIG_TOUCHSCREEN_MSI_ATMEGA168)
	{I2C_BOARD_INFO("atmega168_msi_ts", 0x5c),
	 .platform_data = (void *)&atmega168_msi_pdata,
	 .irq = GPIO_TO_IRQ(PEN_IRQ_GPIO),
	},
#endif /* CONFIG_TOUCHSCREEN_MSI_ATMEGA168 */

 /* yuanlan@wind-mobi.com 2011.06.28 begin */
 /* Add GT818  Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_GT818
		 {
		  I2C_BOARD_INFO("gt818-ts", 0x5D),
		  .platform_data = (void *)&racer2_gt818_pdata,
		  .irq = GPIO_TO_IRQ(PEN_IRQ_GPIO),
		  },
#endif /* CONFIG_TOUCHSCREEN_GT818 */
 /* yuanlan@wind-mobi.com 2011.06.28 end */

 /* xiongbiao@wind-mobi.com 2011.12.13 begin*/
 /* Add FT5x06 Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_FT5X06
	 {
	  I2C_BOARD_INFO("ft5206", 0x3E),
	  .platform_data = (void *)&acar_ft5x06_pdata,
	  .irq = GPIO_TO_IRQ(PEN_IRQ_GPIO),
	  },
#endif 
 /* xiongbiao@wind-mobi.com 2011.12.13 end*/

/* xiongbiao@wind-mobi.com 2011.12.15 begin*/
/* Add MXT140 Capacitive touch panel support */
#ifdef CONFIG_TOUCHSCREEN_MXT140
  {
	  I2C_BOARD_INFO("mxt140_ts", 0x4A),
	  .platform_data = &mxt140_platform_data,
	  .irq = GPIO_TO_IRQ(PEN_IRQ_GPIO),
  },
#endif
 /* xiongbiao@wind-mobi.com 2011.12.15 end*/

 };  

static void bcm21553_add_gpio_i2c_slaves(void)
{

    //atmega168_msi_pdata.platform_init = atmega168_msi_init_platform_hw();
	//atmega168_msi_pdata.i2c_pdata = (struct i2c_slave_platform_data) {
	//.i2c_spd = I2C_SPD_400K,};
    i2c_register_board_info(3, i2c_gpio_devices, ARRAY_SIZE(i2c_gpio_devices)); 

}

#endif
#if defined (CONFIG_BRCM_HAL_CAM)

#if (defined(CONFIG_BCM_CAM_S5K5CA)&& defined( CONFIG_BCM_CAM_OV7690))
#define I2C_PRIMARY_CAM_SLAVE_ADDRESS          0x5A   /**/
#define I2C_SECONDARY_CAM_SLAVE_ADDRESS        0x42   /*ov7692*/  //0x78:write   0x79:read 

struct i2c_slave_platform_data s5k5ca_cam_pdata = {
        .i2c_spd = I2C_SPD_100K,
};

struct i2c_slave_platform_data ov7690_cam_pdata = {
        .i2c_spd = I2C_SPD_400K,
};

static struct i2c_board_info __initdata bcm21553_cam_i2c_board_info[] = {
        {I2C_BOARD_INFO("primary_cami2c", (I2C_PRIMARY_CAM_SLAVE_ADDRESS >> 1)),
         .platform_data = (void *)&s5k5ca_cam_pdata,
         .irq = IRQ_CAM_CSI2,
         },

              {I2C_BOARD_INFO("secondary_cami2c", (I2C_SECONDARY_CAM_SLAVE_ADDRESS >> 1)),
         .platform_data = (void *)&ov7690_cam_pdata,
         .irq = IRQ_CAM_CCP2,
         },
};       

 /*todo*/
#endif 

///-----------------OV5640   and    OV7690    start  --------------
//add-s hanwei@wind-mobi.com  2012.03.02 
//For Camera OV5640 driver I2C driver config
//reviewed by liubing@wind-mobi.com
#if (defined(CONFIG_BCM_CAM_OV5640)&& defined( CONFIG_BCM_CAM_OV7690))
#define I2C_PRIMARY_CAM_SLAVE_ADDRESS          0x78   /**/
#define I2C_SECONDARY_CAM_SLAVE_ADDRESS        0x42   /*ov7692*/  //0x78:write   0x79:read 

struct i2c_slave_platform_data ov5640_cam_pdata = {
        .i2c_spd = I2C_SPD_400K,//I2C_SPD_100K  improve I2C speed fix bug#9461 
};

struct i2c_slave_platform_data ov7690_cam_pdata = {
        .i2c_spd = I2C_SPD_400K,
};

static struct i2c_board_info __initdata bcm21553_cam_i2c_board_info[] = {
        {I2C_BOARD_INFO("primary_cami2c", (I2C_PRIMARY_CAM_SLAVE_ADDRESS >> 1)),
         .platform_data = (void *)&ov5640_cam_pdata,
         .irq = IRQ_CAM_CSI2,
         },

              {I2C_BOARD_INFO("secondary_cami2c", (I2C_SECONDARY_CAM_SLAVE_ADDRESS >> 1)),
         .platform_data = (void *)&ov7690_cam_pdata,
         .irq = IRQ_CAM_CCP2,
         },
};  
//add-s hanwei@wind-mobi.com  2012.03.02 
///-----------------OV5640   and    OV7690    end --------------
 /*todo*/
#endif //#if (defined(CONFIG_BCM_CAM_MT9T111) && defined(CONFIG_BCM_CAM_TCM9001MD))
#endif // #if defined (CONFIG_BRCM_HAL_CAM)

//hanwei@wind-mobi.com 20111021 end


#if defined (CONFIG_SENSORS_AK8975)

#define AKM_IRQ_GPIO (53)
static int bcm_akm8975_gpio_setup(void)
{
	/* GPIO 53 muxed with GPEN8 */
	board_sysconfig(SYSCFG_COMPASS, SYSCFG_INIT);
	gpio_request(AKM_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM_IRQ_GPIO);
	set_irq_type(GPIO_TO_IRQ(AKM_IRQ_GPIO), IRQF_TRIGGER_RISING);

	bcm_gpio_pull_up_down_enable(AKM_IRQ_GPIO, true);
	bcm_gpio_pull_up(AKM_IRQ_GPIO, false);

	return 0;
}

struct akm8975_platform_data akm8975_pdata = {
	.layout = 6,
    .gpio_DRDY = AKM_IRQ_GPIO,
	.init = bcm_akm8975_gpio_setup,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};

#endif


#if defined (CONFIG_SENSORS_AK8962)

#define AKM_IRQ_GPIO (53)
static int bcm_akm8962_gpio_setup(void)
{
	/* GPIO 53 muxed with GPEN8 */
	board_sysconfig(SYSCFG_COMPASS, SYSCFG_INIT);
	gpio_request(AKM_IRQ_GPIO, "akm8962");
	gpio_direction_input(AKM_IRQ_GPIO);
	set_irq_type(GPIO_TO_IRQ(AKM_IRQ_GPIO), IRQF_TRIGGER_RISING);

	bcm_gpio_pull_up_down_enable(AKM_IRQ_GPIO, true);
	bcm_gpio_pull_up(AKM_IRQ_GPIO, false);

	return 0;
}

struct akm8962_platform_data akm8962_pdata = {
	.layout = 6,
    .gpio_DRDY = AKM_IRQ_GPIO,
	.init = bcm_akm8962_gpio_setup,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};

#endif

#if defined(CONFIG_BCMI2CNFC)
static int bcmi2cnfc_gpio_setup(void)
{
	gpio_request(12, "nfc_irq");
	gpio_direction_input(12);
	set_irq_type(GPIO_TO_IRQ(12), IRQF_TRIGGER_RISING);
	pr_err("bcmi2cnfc_gpio_setup nfc irq\n");

	board_sysconfig(SYSCFG_NFC_GPIO, SYSCFG_INIT);
	gpio_request(54, "nfc_en");
	gpio_direction_output(54,1);
	pr_err("bcmi2cnfc_gpio_setup nfc en\n");

	gpio_request(32, "nfc_wake");
	gpio_direction_output(32,0);
	pr_err("bcmi2cnfc_gpio_setup nfc waje\n");

	return 0;
}
static int bcmi2cnfc_gpio_clear(void)
{
	gpio_direction_output(54,0);
	pr_err("bcmi2cnfc_gpio_clear nfc en\n");
	gpio_direction_output(32,1);
	pr_err("bcmi2cnfc_gpio_clear nfc waje\n");

	return 0;
}


static struct bcmi2cnfc_i2c_platform_data bcmi2cnfc_pdata = {
	.irq_gpio = 12,//NFC_IRQ,
	.en_gpio = 54,//NFC_EN,
	.wake_gpio = 32,//NFC_WAKE,
	.init = bcmi2cnfc_gpio_setup,
	.reset = bcmi2cnfc_gpio_clear,
	.i2c_pdata = {.i2c_spd = I2C_SPD_400K,},
};
#endif


#if defined(CONFIG_SENSORS_BMA222)
int bma_gpio_init(struct device *dev)
{
	gpio_request(4, "bma222"); //todo add gpio 6
	gpio_direction_input(4);
	set_irq_type(GPIO_TO_IRQ(4), IRQF_TRIGGER_RISING);
	return 0;
}

static struct bma222_accl_platform_data bma_pdata = {
	//heweimao@wind-mobi.com	2011.11.1 begin
	//bma222 modfiy orientation and invert 	
	//.orientation = BMA_ROT_270,
	//.invert = false,//true,
	.orientation = BMA_ROT_90,
	.invert = true,
	//heweimao@wind-mobi.com
	.init = bma_gpio_init,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};
#endif

#if defined(CONFIG_SENSORS_BMA250)
int bma_gpio_init(struct device *dev)
{
	gpio_request(4, "bma250"); //todo add gpio 6
	gpio_direction_input(4);
	set_irq_type(GPIO_TO_IRQ(4), IRQF_TRIGGER_RISING);
	return 0;
}

static struct bma250_accl_platform_data bma_pdata = {
	//heweimao@wind-mobi.com	2011.11.1 begin
	//bma250 modfiy orientation and invert 	
	//.orientation = BMA_ROT_270,
	//.invert = false,//true,
	.orientation = BMA_ROT_90,
	.invert = true,
	//heweimao@wind-mobi.com
	.init = bma_gpio_init,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};
#endif

#if defined(CONFIG_SENSORS_LIS33DE)
int st_gpio_init(struct device *dev)
{
	gpio_request(4, "lis33de");
	gpio_direction_input(4);
	set_irq_type(GPIO_TO_IRQ(4), IRQF_TRIGGER_RISING);
	return 0;
}

static struct lis33de_accl_platform_data st_pdata = {
// xiongbiao@wind-mobi.com 2012.01.04 begin
// modify lis33du g-sensor rotate direction.
	//.orientation = ST_ROT_90,
	.orientation = ST_ROT_180,
// xiongbiao@wind-mobi.com 2012.01.04 end	
	.invert = false,
	.init = st_gpio_init,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};
#endif


#if defined(CONFIG_SENSORS_AL3006)
static int dyna_gpio_init(void)
{
	gpio_request(14, "al3006"); // PS sensor interrupt
	gpio_direction_input(14);
	set_irq_type(GPIO_TO_IRQ(14), IRQF_TRIGGER_FALLING);
	return 0;
}

static void dyna_gpio_free(struct device *dev)
{
	gpio_free(14);
}

static struct al3006_platform_data dyna_pdata = {
	.init = dyna_gpio_init,
	.exit = dyna_gpio_free,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};
#endif

#if defined(CONFIG_MFD_MAX8986)

#define SYSPARM_PMU_REG(index, dst)					\
	do {								\
		unsigned int parm;					\
		int ret = SYSPARM_GetPMURegSettings(index, &parm);	\
		if (ret == 0) {						\
			pr_info("sysparm: %s: %x\n", #index, parm);	\
			dst = parm;					\
		}							\
	} while (0)

/* Indexes of PMU registers in syparm */
enum {
	PMU_REG_0X07_PM_A1OPMODCTRL = 0x07,
	PMU_REG_0X08_PM_D1OPMODCTRL = 0x08,
	PMU_REG_0X09_PM_A8OPMODCTRL = 0x09,
	PMU_REG_0X0A_PM_A2OPMODCTRL = 0x0A,
	PMU_REG_0X0B_PM_H1OPMODCTRL = 0x0B,
	PMU_REG_0X0C_PM_H2OPMODCTRL = 0x0C,
	PMU_REG_0X0D_PM_D2OPMODCTRL = 0x0D,
	PMU_REG_0X0E_PM_A5OPMODCTRL = 0x0E,
	PMU_REG_0X0F_PM_A4OPMODCTRL = 0x0F,
	PMU_REG_0X10_PM_LVOPMODCTRL = 0x10,
	PMU_REG_0X11_PM_SIMOPMODCTRL = 0x11,
	PMU_REG_0X15_PM_CSROPMODCTRL = 0x15,
	PMU_REG_0X17_PM_IOSROPMODCTRL = 0x17,
	PMU_REG_0x1B_PM_MBCCTRL4 = 0x1B,
	PMU_REG_0X2B_PM_A6OPMODCTRL = 0x2B,
	PMU_REG_0X2C_PM_A3OPMODCTRL = 0x2C,
	PMU_REG_0X2D_PM_AX1OPMODCTRL = 0x2D,
	PMU_REG_0X2E_PM_A7OPMODCTRL = 0x2E,
	PMU_REG_0X41_PM_D3OPMODCTRL = 0x41,
	PMU_REG_0x43_PM_D4OPMODCTRL = 0x43,
	PMU_REG_0X45_PM_A9OPMODCTRL = 0x45,
	PMU_REG_0X47_PM_TSROPMODCTRL = 0x47,
};

/*
DLDO1 supplies - compass, accelarometer, SD-A VDD
Max4996 mux, vddo_sdio1
*/
static struct regulator_consumer_supply dldo1_consumers[] = {
	{
		.dev = NULL,
		.supply = "sd_a_vdd",
	},
	{
		.dev = NULL,
		.supply = "max4996_vcc",
	},
	{
		.dev = NULL,
		.supply = "vddo_sdio3",
	},
	{
		.dev = NULL,
		.supply = "compass_vdd",
	},
	{
		.dev = NULL,
		.supply = "accel_vdd",
	},

};

static struct regulator_init_data dldo1_init_data = {
	.constraints = {
		.min_uV = 2500000,
		.max_uV = 3000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(dldo1_consumers),
	.consumer_supplies = dldo1_consumers,
};

static struct regulator_consumer_supply dldo3_consumers[] = {
	{
		.dev = NULL,
		.supply = "cam_vdd",
	},
	{
		.dev = NULL,
		.supply = "haptic_pwm",
	},
};

static struct regulator_init_data dldo3_init_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 3000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(dldo3_consumers),
	.consumer_supplies = dldo3_consumers,
};

static struct regulator_consumer_supply aldo6_consumers[] = {
	{
		.dev = NULL,
		.supply = "lcd_vcc",
	},
};

static struct regulator_init_data aldo6_init_data = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(aldo6_consumers),
	.consumer_supplies = aldo6_consumers,
};

static struct regulator_consumer_supply aldo8_consumers[] = {
	{
		.dev = NULL,
		.supply = "cam_2v8",
	},
};

static struct regulator_init_data aldo8_init_data = {
	.constraints = {
		.min_uV = 2500000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(aldo8_consumers),
	.consumer_supplies = aldo8_consumers,
};

static struct regulator_consumer_supply sim_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "sim_vcc",
	},
};

static struct regulator_init_data sim_init_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(sim_consumers),
	.consumer_supplies = sim_consumers,
};

static struct regulator_consumer_supply sim1_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "sim1_vcc",
	},
};

static struct regulator_init_data sim1_init_data = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE|REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(sim1_consumers),
	.consumer_supplies = sim1_consumers,
};
static struct regulator_consumer_supply csr_nm1_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "csr_nm1",
	},
};

static struct regulator_init_data csr_nm1_init_data = {
	.constraints = {
		.min_uV = 900000,
		.max_uV = 1380000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(csr_nm1_consumers),
	.consumer_supplies = csr_nm1_consumers,
};

static struct regulator_consumer_supply csr_nm2_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "csr_nm2",
	},
};

static struct regulator_init_data csr_nm2_init_data = {
	.constraints = {
		.min_uV = 900000,
		.max_uV = 1380000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(csr_nm2_consumers),
	.consumer_supplies = csr_nm2_consumers,
};

/*
ACAR uses ALDO9 to power TOUCH SCREEN
*/
static struct regulator_consumer_supply aldo9_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "vddo_tp",
	},
};

static struct regulator_init_data aldo9_init_data = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS|
					REGULATOR_CHANGE_VOLTAGE,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(aldo9_consumers),
	.consumer_supplies = aldo9_consumers,
};

static struct regulator_consumer_supply hcldo1_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "hcldo1_3v3",
	},
};

static struct regulator_init_data hcldo1_init_data = {
	.constraints = {
		.min_uV = 2500000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS|REGULATOR_CHANGE_VOLTAGE,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(hcldo1_consumers),
	.consumer_supplies = hcldo1_consumers,
};
/*
HCLDO2 regulator is used for CAM_AF_VDD_2V8
*/
static struct regulator_consumer_supply hcldo2_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "cam_af_vdd",
	},
};

static struct regulator_init_data hcldo2_init_data = {
	.constraints = {
		.min_uV = 2500000,
		.max_uV = 3000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(hcldo2_consumers),
	.consumer_supplies = hcldo2_consumers,
};

/* GPS */
static struct regulator_consumer_supply aldo1_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "aldo1_3v0",
	},
};

static struct regulator_init_data aldo1_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY,
		.always_on = 0,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(aldo1_consumers),
	.consumer_supplies = aldo1_consumers,
};

static struct max8986_regl_init_data bcm21553_regulators[] = {
	{
		.regl_id = MAX8986_REGL_SIMLDO,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &sim_init_data,
	},
	{
		.regl_id = MAX8986_REGL_DLDO4,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &sim1_init_data,
	},
	{
		.regl_id = MAX8986_REGL_DLDO1,
		.dsm_opmode = MAX8986_REGL_OFF_IN_DSM,
		.init_data = &dldo1_init_data,
	},
	{
		.regl_id = MAX8986_REGL_DLDO3,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &dldo3_init_data,
	},
	{
		.regl_id = MAX8986_REGL_ALDO6,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &aldo6_init_data,
	},
	{
		.regl_id = MAX8986_REGL_ALDO8,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &aldo8_init_data,
	},
	{
		.regl_id = MAX8986_REGL_CSR_NM1,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &csr_nm1_init_data,
	},
	{
		.regl_id = MAX8986_REGL_CSR_NM2,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &csr_nm2_init_data,
	},
	{
		.regl_id = MAX8986_REGL_ALDO9,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &aldo9_init_data,
	},
    {
		.regl_id = MAX8986_REGL_HCLDO1,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &hcldo1_init_data,
	},
	{
		.regl_id = MAX8986_REGL_HCLDO2,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &hcldo2_init_data,
	},
	{
		.regl_id = MAX8986_REGL_ALDO1,
		.dsm_opmode = MAX8986_REGL_OFF_IN_DSM,
		.init_data = &aldo1_init_data,
	},

};

static struct max8986_regl_pdata regl_pdata = {
	.num_regulators = ARRAY_SIZE(bcm21553_regulators),
	.regl_init =  bcm21553_regulators,
	.regl_default_pmmode = {
		[MAX8986_REGL_ALDO1]	= 0x02,
		[MAX8986_REGL_ALDO2]	= 0x11,
		[MAX8986_REGL_ALDO3]	= 0x11,
/*liubing@wind-mobi.com 2011.12.16 begin*/
/*ALDO4_2V8 is used for ASM VDD power,it should be set to ON if the system is not in sleep mode */
		//[MAX8986_REGL_ALDO4]	= 0xAA, /*Not used*/
		[MAX8986_REGL_ALDO4]	= 0x02, /*ASM VDD in L301*/
/*liubing@wind-mobi.com 2011.12.16 end*/
		[MAX8986_REGL_ALDO5]	= 0x01,
		[MAX8986_REGL_ALDO6]	= 0x11,
		[MAX8986_REGL_ALDO7]	= 0x11,
		[MAX8986_REGL_ALDO8]	= 0xAA, /*3M back camera*/
		[MAX8986_REGL_ALDO9]	= 0x01, //Touch Panel power
		[MAX8986_REGL_DLDO1]	= 0x01,
		[MAX8986_REGL_DLDO2]	= 0x01,  /*internal for PMU irq reset*/
		[MAX8986_REGL_DLDO3]	= 0xAA,  //for Camera 1.8v 
		[MAX8986_REGL_DLDO4]	= 0xAA, /*SIMB - not used*/
/* xiongbiao@wind-mobi.com 2011.12.15 begin*/
/* Add MXT140 Capacitive touch panel support */
		//[MAX8986_REGL_HCLDO1]	= 0xAA,
		[MAX8986_REGL_HCLDO1]	= 0x00,
/* xiongbiao@wind-mobi.com 2011.12.15 end*/
		[MAX8986_REGL_HCLDO2]	= 0xAA, //3Mega Camera auto focus motor
		[MAX8986_REGL_LVLDO]	= 0x11,
		[MAX8986_REGL_SIMLDO]	= 0xAA,
		[MAX8986_REGL_AUXLDO1]	= 0xAA, /*Not used*/
		[MAX8986_REGL_TSRLDO]	= 0x01,
		/*USE MAX8986_REGL_CSR_LPM set CSR op mode*/
		[MAX8986_REGL_CSR_LPM]	= 0xCD,
		[MAX8986_REGL_IOSR]	= 0x01,

	},
};

static void __init update_regl_sysparm(void)
{
#if defined(CONFIG_BRCM_FUSE_SYSPARM)

#endif /* CONFIG_BRCM_FUSE_SYSPARM */
}

/*Default settings for audio amp */
static struct max8986_audio_pdata audio_pdata = {
	.ina_def_mode = MAX8986_INPUT_MODE_SINGLE_ENDED,
	.inb_def_mode = MAX8986_INPUT_MODE_DIFFERENTIAL,
	.ina_def_preampgain = PREAMP_GAIN_NEG_3DB,
	.inb_def_preampgain = PREAMP_GAIN_NEG_3DB,

	.lhs_def_mixer_in = MAX8986_MIXER_INA1,
	.rhs_def_mixer_in = MAX8986_MIXER_INA2,
	.ihf_def_mixer_in = MAX8986_MIXER_INB2,

	.hs_input_path = MAX8986_INPUTA,
	.ihf_input_path = MAX8986_INPUTB,
};
static struct max8986_power_pdata power_pdata = {
	.usb_charging_cc = MAX8986_CHARGING_CURR_500MA,
//songjinguo@wind-mobi.com 20120329 start
//change charging current and EOC current
#ifdef CONFIG_XIALANG_BATTERY	//for xialang battery
	.wac_charging_cc = MAX8986_CHARGING_CURR_550MA,
	.eoc_current = MAX8986_EOC_50MA,
#else	
	.wac_charging_cc = MAX8986_CHARGING_CURR_950MA,
	.eoc_current = MAX8986_EOC_100MA,
#endif	
//songjinguo@wind-mobi.com 20120329 end	
	.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION,
};

struct max8986_muic_pdata muic_data = {
	.mic_insert_adc_val = PMU_MUIC_ADC_OUTPUT_1M,
	.mic_btn_press_adc_val = PMU_MUIC_ADC_OUTPUT_2K,
};


static void __init update_power_sysparm(void)
{
#if defined(CONFIG_BRCM_FUSE_SYSPARM)
//	SYSPARM_PMU_REG(PMU_REG_0x1B_PM_MBCCTRL4, power_pdata.usb_charging_cc);
//	SYSPARM_PMU_REG(PMU_REG_0x1B_PM_MBCCTRL4, power_pdata.wac_charging_cc);
#endif /* CONFIG_BRCM_FUSE_SYSPARM */
}

/*****************************************************************************
 * platform specific usb interface routines for the power driver
 *****************************************************************************/
/* indicates whether the usb stack is ready or not */
static bool usb_driver_initialized;
/* indicates whether usb charger is detected or not */
static bool usb_charger_detected;
extern void Android_usb_cable_connection(bool is_connected);
extern void dwc_otg_pcd_StartUsb(int is_on);
extern void Android_PMU_USB_Start(void);
extern void Android_PMU_USB_Stop(void);


/* if both the power driver and the usb driver are ready, then start the
 * enumeration seqence.
 */
static void pmu_usb_start(void)
{
	pr_info("%s\n", __func__);

#ifdef CONFIG_USB_ANDROID
	if (usb_driver_initialized == true && usb_charger_detected == true)
	{
		Android_usb_cable_connection(1);
		Android_PMU_USB_Start();
		pr_info("%s: usb stack is UP\n", __func__);
	} else
	{
		Android_usb_cable_connection(0);
		pr_info("%s: usb stack is not up\n", __func__);
	}
#endif
}

static void pmu_usb_stop(void)
{
	pr_info("%s\n", __func__);
#ifdef CONFIG_USB_DWC_OTG
	Android_usb_cable_connection(0);
	Android_PMU_USB_Stop();
#endif
}

/*****************************************************************************
 * USB driver call backs
 *****************************************************************************/

/* this function is called from the usb driver once the USB reset event is
 * detected.
 */
void pmu_usb_enum_started(void)
{
	pr_info("%s\n", __func__);
}
EXPORT_SYMBOL(pmu_usb_enum_started);

/* this function is called by the usb driver once the usb stack init is
 * completed.
 */
void pmu_usb_driver_initialized(void)
{
	pr_info("%s\n", __func__);

	/* start usb enumeration if this function is called for the first
	 * time
	 */
	if (usb_driver_initialized == false) {
		usb_driver_initialized = true;
		pmu_usb_start();
	}
}
EXPORT_SYMBOL(pmu_usb_driver_initialized);

#define PMU_IRQ_GPIO    (34)
/* songjinguo@wind-mobi.com 2011.12.31 start */
/* modify for bug 6183; csp:479804 */
/* review by liubing */
#ifdef  BATTERY_ADC_FIX
static u32 pre_bat_vol= 0;
static ktime_t  charger_begin_time ;
static ktime_t  charger_end_time ;
static ktime_t  boot_begin_time;
#define ADC_BOOT_TIME   (130) 
#define STABLE_BOOT_TIME (40)
#define ADC_FIX_TIME   (130)
#define ADC_FIX (8)

extern int pmu_get_charging_status(void);

static inline int check_time_delta(ktime_t a, ktime_t b)
{
    ktime_t res = ktime_sub(a,b);
    return res.tv.sec;
}                  
#endif
/* songjinguo@wind-mobi.com 2011.12.31 end */
static u32 pmu_event_callback(int event, int param)
{
	u32 ret = 0;
	/*
	bat_adc_volt_mul calculated as follows:
	V(mV) = ADC *((R1+R2)/ R2)*( ADC input range /10-bit ADC)
	V(mV) = ADC * ((750K+243K)/ 243K)*(1200/1023)
	V(mV) = ADC * (4.08642)*(1.173021)
	V(mV) = ADC * (4.793454)
	*/
	const u32 bat_adc_volt_mul = 4793;
/* songjinguo@wind-mobi.com 2011.12.20 start */
/* update charging curve */
/* review by liubing */
#ifdef CONFIG_XIALANG_BATTERY	//This is for xialang battery
	pr_info("<<<<  config xialang battery >>>>");
      static u16 bat_vol[] = {3400, 3600, 3650, 3700, 3750, 3800, 3850, 3900, 3950, 4000, 4050, 4100, 4210};
	static u8 bat_per[] = {0, 12, 23, 38, 53, 63, 70, 78, 84, 90, 94, 98, 100};
#ifdef BATTERY_ADC_FIX
static u16 bat_vol_charging_usb[] = {3490, 3680, 3725, 3775, 3825, 3875, 3925, 3975, 4025, 4075, 4125, 4165, 4220};
static u16 bat_vol_charging_ac[] = {3530, 3730, 3775, 3825, 3875, 3925, 3975, 4025, 4075, 4125, 4175, 4200, 4230};
	u16 *bat_vol_charging;
    ktime_t  read_time;
    int fix_adc = 0;
#endif
#else
      static u16 bat_vol[] = {3400, 3600, 3695, 3725, 3760, 3790, 3820, 3850, 3880, 3920, 3955, 4015, 4160};
	static u8 bat_per[] = {0, 16, 26, 32, 42, 50, 56, 62, 68, 76, 83, 92, 100};
#ifdef BATTERY_ADC_FIX
static u16 bat_vol_charging_usb[] = {3490, 3680, 3780, 3800, 3835, 3865, 3895, 3925, 3955, 3985, 4020, 4080, 4210};
static u16 bat_vol_charging_ac[] = {3670, 3750, 3845, 3875, 3910, 3940, 3970, 4000, 4030, 4070, 4105, 4135, 4220};
	u16 *bat_vol_charging;
    ktime_t  read_time;
    int fix_adc = 0;
#endif
#endif
/* songjinguo@wind-mobi.com 2011.12.31 end */
	switch (event) {

	case PMU_EVENT_INIT_PLATFORM:
	pr_info("%s: PMU: init hardware\n", __func__);
/* songjinguo@wind-mobi.com 2011.12.31 start */
/* modify for bug 6183; csp:479804 */
/* review by liubing */
#ifdef  BATTERY_ADC_FIX
    charger_begin_time= ktime_set(0,0);
    charger_end_time= ktime_set(0,0);
    boot_begin_time= ktime_get(); 
#endif
/* songjinguo@wind-mobi.com 2011.12.31 end */
	if (gpio_request(PMU_IRQ_GPIO, "pmu_irq") == 0) {
		gpio_direction_input(PMU_IRQ_GPIO);
		bcm_gpio_pull_up_down_enable(PMU_IRQ_GPIO, true);
		bcm_gpio_pull_up(PMU_IRQ_GPIO, true);

		set_irq_type(GPIO_TO_IRQ(PMU_IRQ_GPIO), IRQF_TRIGGER_FALLING);
	} else {
		pr_err("%s: failed to allocate GPIO for PMU IRQ\n",
			__func__);
	}
	/* init batt params */
	power_pdata.batt_lvl_tbl.num_entries = ARRAY_SIZE(bat_vol);
	power_pdata.batt_lvl_tbl.bat_percentage = bat_per;
	power_pdata.batt_lvl_tbl.bat_vol = bat_vol;
	break;

	case PMU_EVENT_BATT_ADC_TO_VOLTAGE:
		ret = param*bat_adc_volt_mul;
/* songjinguo@wind-mobi.com 2011.12.31 start */
/* modify for bug 6183; csp:479804 */
/* review by liubing */
#ifdef  BATTERY_ADC_FIX
        read_time= ktime_get();
        if(check_time_delta(read_time, boot_begin_time) < ADC_FIX_TIME)
        {
            fix_adc = (ADC_FIX*(ADC_FIX_TIME - check_time_delta(read_time,boot_begin_time)))/ADC_FIX_TIME;
            ret=(param + fix_adc) * bat_adc_volt_mul;
            pr_info("%s: PMU: fix_adc= %d , voltage= %d\n", __func__,fix_adc,ret);
        }
         if(pmu_get_charging_status()==POWER_SUPPLY_STATUS_CHARGING){
            pr_info("%s: PMU: in charging, origin voltage= %d\n", __func__,ret);     
            int lnx=0;
            int lnx_max=ARRAY_SIZE(bat_vol)-1;
            u32 vol = ret;
		if(max89xx_muic_get_charger_type() == PMU_MUIC_CHGTYP_USB)
		 {	bat_vol_charging = bat_vol_charging_usb;
		}	
		else{
		  	bat_vol_charging = bat_vol_charging_ac;
		}
            if(vol < bat_vol_charging[0]*1000){
                ret = bat_vol[0]*1000;
            }else if(vol >= bat_vol_charging[lnx_max]*1000) {
                ret = bat_vol[lnx_max]*1000;
            }else{
                if(check_time_delta(read_time,boot_begin_time)< STABLE_BOOT_TIME){
                     ret = param*bat_adc_volt_mul;
                }else{
                     for (lnx = 0; lnx < lnx_max; lnx++ ){ 
                            if((vol >= bat_vol_charging[lnx]*1000) && (vol < bat_vol_charging[lnx+1]*1000)){           
                                ret =  (bat_vol[lnx+1]-bat_vol[lnx])*( vol - bat_vol_charging[lnx]*1000);
                                ret /= bat_vol_charging[lnx+1]-bat_vol_charging[lnx];
                                ret = ret  + bat_vol[lnx]*1000;   
                                break;  
                        }
                     }
              }
           }
           if(param!=0xffff){                
                if((pre_bat_vol > ret) && (check_time_delta(read_time,charger_begin_time) <  ADC_BOOT_TIME) && (ktime_to_ns(charger_begin_time) != 0)) 
                {
                    pr_info("%s: PMU in charging ,pre-volatge = %d , voltage = %d \n",__func__,pre_bat_vol,ret);
                    ret=pre_bat_vol;
                }   
                pre_bat_vol = ret;
             }  
        }
        else{
                if(param!=0xffff){         
                    if(pre_bat_vol != 0){
                        if(check_time_delta(read_time,boot_begin_time) <  ADC_FIX_TIME){
                            if (pre_bat_vol > ret ){
                                 pr_info("%s: PMU in discharging_boot pre_voltage= %d , voltage= %d \n",__func__,pre_bat_vol,ret);
                                 ret=pre_bat_vol;
                            }                                                                                                     
                       }
                       if( (check_time_delta(read_time,charger_end_time) <  ADC_BOOT_TIME) && (ktime_to_ns(charger_end_time) != 0)){
                             if(pre_bat_vol <  ret){
                                pr_info("%s: PMU in discharging pre_voltage= %d , voltage= %d \n",__func__,pre_bat_vol,ret);
                                ret=pre_bat_vol;
                             }
                       }
                    }   

                pre_bat_vol = ret;
                }
            }
#endif
/* songjinguo@wind-mobi.com 2011.12.31 end */
		break;

	case PMU_EVENT_CHARGER_INSERT:
	pr_info("%s: PMU_EVENT_CHARGER_INSERT\n", __func__);

	/* try to start the usb enumeration process. this may not succeed if
	 * the usb stack is still not up.
	 */
/* songjinguo@wind-mobi.com 2011.12.31start */
/* modify for bug 6183; csp:479804 */
/* review by liubing */	
#ifdef BATTERY_ADC_FIX
    charger_begin_time = ktime_get();
#endif
/* songjinguo@wind-mobi.com 2011.12.31 end */
	if (param == PMU_MUIC_CHGTYP_USB ||
			param == PMU_MUIC_CHGTYP_DOWNSTREAM_PORT)
	{
//maxiaohui@wind-mobi.com 2012.05.20 begin
		if(!usb_charger_detected)
//maxiaohui@wind-mobi.com 2012.05.20 end
{
		usb_charger_detected = true;
		pmu_usb_start();
}
	}
	break;

	case PMU_EVENT_CHARGER_REMOVE:
	pr_info("%s: PMU_EVENT_CHARGER_REMOVE\n", __func__);
/* songjinguo@wind-mobi.com 2011.12.31 start */
/* modify for bug 6183; csp:479804 */
/* review by liubing */
#ifdef BATTERY_ADC_FIX
    charger_end_time= ktime_get();
#endif
/* songjinguo@wind-mobi.com 2011.12.31 end */
	if (param == PMU_MUIC_CHGTYP_USB ||
			param == PMU_MUIC_CHGTYP_DOWNSTREAM_PORT)
	{
		usb_charger_detected = false;
		pmu_usb_stop();
	}
	break;

	default:
	pr_err("%s: unrecognized event: %d\n", __func__, event);
	break;

	}
	return ret;
}

struct max8986_platform_data max8986_pdata = {
	.i2c_pdata = {.i2c_spd = I2C_SPD_400K,},
	.flags = MAX8986_USE_REGULATORS | MAX8986_REGISTER_POWER_OFF |
		MAX8986_ENABLE_AUDIO | MAX8986_USE_PONKEY | MAX8986_ENABLE_MUIC |
		MAX8986_USE_RTC | MAX8986_USE_POWER | MAX8986_ENABLE_DVS,
	.pmu_event_cb = pmu_event_callback,
	.regulators = &regl_pdata,
	.audio_pdata = &audio_pdata,
	.power = &power_pdata,
	.muic_pdata = &muic_data,
	/* CSR Voltage data */
	.csr_nm_volt = CSR_VOUT_1_36V,
	.csr_nm2_volt = -1, /*AVS driver will set this*/
	.csr_lpm_volt = CSR_VOUT_0_90V,
};
#endif /*CONFIG_MFD_MAX8986*/

#ifdef CONFIG_BCM_AUXADC
static struct bcm_plat_auxadc adcplat = {
	.readmask = 0x3FF,
	.croff = 0,
	.rdoff = 8,
	.ready = (0x1 << 15),
	.start = (0x1 << 3),
	.auxpm = 0,
	.regoff = (0x1 << 23),
	.bgoff = (0x1 << 22),
};
#endif


#if defined(CONFIG_BRCM_HEADSET)  || defined(CONFIG_BRCM_HEADSET_MODULE)

/* songjinguo@wind-mobi.com 2011.11.28 start */
/* config headset detected pin to gpio 31, enable headset detected pin; */
/* review by liubing */
#define HEADSET_DET_GPIO        31    /* HEADSET DET */
void check_hs_state (int *headset_state)
{
	*headset_state = !gpio_get_value(HEADSET_DET_GPIO);
}

static struct brcm_headset_pd headset_pd = {
	.hsirq		= GPIO_TO_IRQ(HEADSET_DET_GPIO), //IRQ_MICIN, //0,
	.hsbirq		= IRQ_MICON,
	.check_hs_state = check_hs_state,
	.hsgpio		= HEADSET_DET_GPIO,
	.debounce_ms    = 400,
//    .keypress_threshold_lp  = 0x5200,
//    .keypress_threshold_fp  = 0x6000,
//songjinguo@wind-mobi.com 2011.03.28 start
//Enable headset key function
//Review by yuanlan
	.keypress_threshold_lp  = 0x5E00,
      .keypress_threshold_fp  = 0x6000,
//songjinguo@wind-mobi.com 2012.03.28 end  
};
#endif
/* songjinguo@wind-mobi.com 2011.11.28 end	*/

#if defined(CONFIG_I2C_BOARDINFO)
struct i2c_host_platform_data i2c1_host_pdata = {
	.retries = 3,
	.timeout = 20 * HZ,
};

/* I2C_1: Compass AK8975, Accel BMA222, Proximity sensor */
static struct i2c_board_info __initdata athenaray_i2c1_board_info[] = {
#if defined (CONFIG_SENSORS_AK8975)
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = (void *)&akm8975_pdata,
		.irq = GPIO_TO_IRQ(AKM_IRQ_GPIO), /* GPEN08 */
	},
#endif

#if defined (CONFIG_SENSORS_AK8962)
	{
		I2C_BOARD_INFO("akm8962", 0x0f),
		.platform_data = (void *)&akm8962_pdata,
		.irq = GPIO_TO_IRQ(AKM_IRQ_GPIO), /* GPEN08 */
	},
#endif

#if defined(CONFIG_SENSORS_BMA222)
	{
		I2C_BOARD_INFO("bma222_accl", 0x08),
		.platform_data = (void *)&bma_pdata,
		.irq = GPIO_TO_IRQ(4),
	},
#endif

#if defined(CONFIG_SENSORS_BMA250)
	{
		I2C_BOARD_INFO("bma250_accl", 0x18),
		.platform_data = (void *)&bma_pdata,
		.irq = GPIO_TO_IRQ(4),
	},
#endif

#if defined(CONFIG_SENSORS_LIS33DE)
	{
		I2C_BOARD_INFO("lis33de_accl", 0x1C),
		.platform_data = (void *)&st_pdata,
	 	.irq = GPIO_TO_IRQ(4),
	 },
#endif

#if defined(CONFIG_TMD2771X) || defined(CONFIG_TMD2771X_MODULE)
    {I2C_BOARD_INFO("tsl2771", 0x39),
     .platform_data = (void *)&tmd2771x_pdata,
     .irq = GPIO_TO_IRQ(14),
    },
#endif

#if defined(CONFIG_SENSORS_AL3006)
    {
		I2C_BOARD_INFO("al3006", 0x1c),
		.platform_data = (void *)&dyna_pdata,
		.irq = GPIO_TO_IRQ(14),
	},
#endif
    #ifdef CONFIG_MPU_SENSORS_MPU3050
{
      I2C_BOARD_INFO("mpu3050", 0x68),
      .irq = GPIO_TO_IRQ(IH_GPIO_BASE + MPUIRQ_GPIO),
      .platform_data = &mpu3050_data,
},

{
      I2C_BOARD_INFO("bma250", 0x18),
     // .irq = GPIO_TO_IRQ(IH_GPIO_BASE + ACCEL_IRQ_GPIO),
      .platform_data = &inv_mpu_kxtf9_data,
},
{
      I2C_BOARD_INFO("ak8975", 0x0F),
      //.irq = GPIO_TO_IRQ(IH_GPIO_BASE + COMPASS_IRQ_GPIO),
      .platform_data = &inv_mpu_ak8975_data,
},
#endif

};

/* I2C_2: 3M sesor, VGA sensor, NFC	*/
static struct i2c_board_info __initdata athenaray_i2c2_board_info[] = {
#if defined(CONFIG_BCMI2CNFC)
	{I2C_BOARD_INFO("bcmi2cnfc", 0x1FA),
	 .flags = I2C_CLIENT_TEN,
	 .platform_data = (void *)&bcmi2cnfc_pdata,
	 .irq = GPIO_TO_IRQ(12), 
	 },
#endif

};

static struct i2c_board_info __initdata athenaray_i2c3_board_info[] = {
#if defined(CONFIG_MFD_MAX8986)
	{
		I2C_BOARD_INFO("max8986", 0x08),
		.platform_data = (void *)&max8986_pdata,
		.irq = GPIO_TO_IRQ(34),
	}
#endif /*CONFIG_MFD_MAX8986*/
};

#endif




static void athenaray_add_i2c_slaves(void)
{
	i2c_register_board_info(I2C_BSC_ID0, athenaray_i2c1_board_info,
				ARRAY_SIZE(athenaray_i2c1_board_info));
	i2c_register_board_info(I2C_BSC_ID1, athenaray_i2c2_board_info,
				ARRAY_SIZE(athenaray_i2c2_board_info));
	i2c_register_board_info(I2C_BSC_ID2, athenaray_i2c3_board_info,
				ARRAY_SIZE(athenaray_i2c3_board_info));
#if defined (CONFIG_BRCM_HAL_CAM)
        i2c_register_board_info(I2C_BSC_ID1, bcm21553_cam_i2c_board_info,
                                ARRAY_SIZE(bcm21553_cam_i2c_board_info));
#endif
}
#if defined (CONFIG_I2C_GPIO)
/* Adding the device here as this device is board specific */
struct i2c_gpio_platform_data bcm21553_i2c_gpio_data = {
	.sda_pin               = 27,
	.scl_pin               = 26,
	.udelay = 1,
	.sda_is_open_drain     = 0,
	.scl_is_open_drain     = 0,
 };
struct platform_device bcm21553_i2c_gpio_device =  {
	.name   = "i2c-gpio",
	.id     = 0x03,
	.dev    = {
		.platform_data = &bcm21553_i2c_gpio_data,
	},
};
#endif

#ifdef CONFIG_SPI
static DEFINE_SPINLOCK(bcm_spi_lock);
static int bcm21xx_cs_control(struct driver_data *drv_data, u8 cs)
{
	unsigned long flags;

	if (!drv_data)
		return -EINVAL;

	spin_lock_irqsave(&bcm_spi_lock, flags);

	writeb(SPI_SPIFSSCR_FSSNEW(cs) |
	       SPI_SPIFSSCR_FSSSELNEW, drv_data->ioaddr + SPI_SPIFSSCR);

	spin_unlock_irqrestore(&bcm_spi_lock, flags);

	return 0;
}

static struct bcm21xx_spi_platform_data bcm21xx_spi_info = {
	.slot_id = 0,
	.enable_dma = 1,
	.cs_line = 1,
	.mode = 0,		/* Configure for master mode */
	.syscfg_inf = board_sysconfig,
	.cs_control = bcm21xx_cs_control,
};

/*
 * SPI board info for the slaves
 */
static struct spi_board_info spi_slave_board_info[] __initdata = {
	{
	 .modalias = "spidev",	/* use spidev generic driver */
	 .max_speed_hz = 60000000,	/* use max speed */
	 .bus_num = 0,		/* framework bus number */
	 .chip_select = 0,	/* for each slave */
	 .platform_data = NULL,	/* no spi_driver specific */
	 .controller_data = &bcm21xx_spi_info,
	 .irq = 0,		/* IRQ for this device */
	 .mode = SPI_LOOP,	/* SPI mode 0 */
	 },
	/* TODO: adding more slaves here */
};

#endif

#if defined(CONFIG_FB_BCM)
/*!
 *  * The LCD definition structure.
 *   */

struct platform_device bcm21553_device_fb = {
	.name   = "LCDfb",
	.id     = -1,
	.dev    = {
		.dma_mask               = (u64 *)~(u32)0,
		.coherent_dma_mask      = ~(u32)0,
	},
	.num_resources = 0,
};
#endif

#ifdef CONFIG_BCM215XX_DSS
static struct lcdc_platform_data_t lcdc_pdata = {
	.gpio = 45,
	.te_supported = true,
};
#endif



#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))

#define BCMBLT_VREG_GPIO        21
#define BCMBLT_N_RESET_GPIO     20
#define BCMBLT_AUX0_GPIO        (-1)	/* clk32 */
#define BCMBLT_AUX1_GPIO        (-1) /* UARTB_SEL */

static struct bcmblt_rfkill_platform_data board_bcmblt_rfkill_cfg = {
	.vreg_gpio = BCMBLT_VREG_GPIO,
	.n_reset_gpio = BCMBLT_N_RESET_GPIO,
	.aux0_gpio = BCMBLT_AUX0_GPIO,	/* CLK32 */
	.aux1_gpio = BCMBLT_AUX1_GPIO,	/* UARTB_SEL, probably not required */
};

static struct platform_device board_bcmblt_rfkill_device = {
	.name = "bcmblt-rfkill",
	.id = -1,
};
#endif

#if defined(CONFIG_CRYPTO_DEV_BRCM_BLKCRYPTO)
static struct resource blkcrypto_resources[] = {
        {
         .start = IO_ADDRESS(BCM21553_CRYPTO_BASE),
         .end = IO_ADDRESS(BCM21553_CRYPTO_BASE) + SZ_64K - 1,
         .flags = IORESOURCE_MEM,
         },
	{
	 .start = BCM21553_CRYPTO_BASE,
	 .end	= BCM21553_CRYPTO_BASE + SZ_64K - 1,
	 .flags	= IORESOURCE_MEM,
	}
};

static struct platform_device board_blkcrypto_device = {
	.name           =       "brcm_blkcrypto",
	.id             =       0,
	.resource	=	blkcrypto_resources,
	.num_resources	=	ARRAY_SIZE(blkcrypto_resources),
};
#endif

#ifdef CONFIG_USB_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
         .vendor_id = 0x19D2,	
         .product_id = 0x0083,
	.adb_product_id = 0x0002,
	.version = 0x0100,
	.product_name = "BCM21553-Thunderbird",
	.manufacturer_name = "Broadcom",
	.serial_number="0123456789ABCDEF",
	.nluns = 1,
};

struct platform_device android_usb_device = {
      .name = "android_usb",
      .id         = -1,
      .dev        = {
            .platform_data = &android_usb_pdata,
      },
};
#endif


#ifdef CONFIG_BCM_GPIO_VIBRATOR
static struct timed_gpio timed_gpios[] = {
	{
	 .name = "vibrator",
	 .gpio = 16,
	 .max_timeout = 15000,
	 .active_low = 0,
	 },
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios = ARRAY_SIZE(timed_gpios),
	.gpios = timed_gpios,
};

static struct platform_device bcm_timed_gpio = {
	.name = "timed-gpio",
	.id = -1,
	.dev = {
		.platform_data = &timed_gpio_data,
		},
};

#endif

#ifdef CONFIG_FB_BRCM_DSI
static struct platform_device alex_dsi_display_device = {
	.name    = "dsi_fb",
	.id      = 0,
	.dev = {
		.platform_data		= NULL,
		.dma_mask		= (u64 *) ~(u32)0,
		.coherent_dma_mask	= ~(u32)0,
	},
};
#endif


/* These are active devices on this board */
static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	&bcm21553_serial_device0,
	&bcm21553_serial_device1,
	&bcm21553_serial_device2,
#endif
#if defined(CONFIG_DMADEVICES) && defined(CONFIG_OSDAL_SUPPORT)
	&bcm21xx_dma_device,
#endif
#ifdef CONFIG_MMC_BCM
	&bcm21553_sdhc_slot1,
#if !defined(CONFIG_MTD_ONENAND) && !defined(CONFIG_MTD_NAND)
	&bcm21553_sdhc_slot2,
#endif
	&bcm21553_sdhc_slot3,
#endif

#if defined(CONFIG_BCM_WATCHDOG)
	&bcm_watchdog_device,
#endif

#ifdef CONFIG_KEYBOARD_BCM
	&bcm215xx_kp_device,
#endif
#ifdef CONFIG_BCM_AUXADC
	&auxadc_device,
#endif
#if defined(CONFIG_BRCM_HEADSET) || defined(CONFIG_BRCM_HEADSET_MODULE)
	&bcm_headset_device,
#endif
#ifdef CONFIG_BCM_PWM
	&bcm_pwm_device,
#ifdef CONFIG_BACKLIGHT_FAN5626
	&bcm_backlight_devices,
#endif

/* yaogangxiang@wind-mobi.com 2011.10.21 begin */
/* add lcd backlight driver for platform divice */
#ifdef CONFIG_BACKLIGHT_TPS61165
	&bcm_backlight_devices,
#endif
/* yaogangxiang@wind-mobi.com 2011.10.21 end */

#endif
#ifdef CONFIG_MTD_ONENAND_BCM_XBOARD
	&athenaray_device_onenand,
#endif
#ifdef CONFIG_MTD_NAND_BRCM_NVSRAM
	&bcm21553_device_nand,
#endif
#ifdef CONFIG_I2C
	&bcm21553_device_i2c1,
	&bcm21553_device_i2c2,
	&bcm21553_device_i2c3,
#endif
#ifdef CONFIG_SPI
	&bcm21xx_device_spi,
#endif
#ifdef CONFIG_FB_BCM
	&bcm21553_device_fb,
#endif
#ifdef CONFIG_BCM215XX_DSS
	&bcm215xx_lcdc_device,
#endif
#if defined (CONFIG_BCM_OTP)
	&bcm_otp_device,
#endif
#ifdef CONFIG_BCM_I2SDAI
    &i2sdai_device,
#endif
#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))
	&board_bcmblt_rfkill_device,
#endif
#ifdef CONFIG_BCM_CPU_FREQ
	&bcm21553_cpufreq_drv,
#endif
#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
	&bcm21553_cpufreq_gov,
#endif
#ifdef CONFIG_BCM_AVS
	&bcm215xx_avs_device,
#endif
#ifdef CONFIG_CRYPTO_DEV_BRCM_BLKCRYPTO
       &board_blkcrypto_device,
#endif
#ifdef CONFIG_LEDS_GPIO
	&leds_gpio,
#endif
#ifdef CONFIG_USB_ANDROID
        &android_usb_device,
#endif
#ifdef CONFIG_BCM_GPIO_VIBRATOR
	&bcm_timed_gpio,
#endif
#if defined (CONFIG_BRCM_V3D)
    &v3d_device,
#endif
#ifdef CONFIG_I2C_GPIO
	&bcm21553_i2c_gpio_device,
#endif
#ifdef CONFIG_FB_BRCM_DSI
	&alex_dsi_display_device,
#endif
};

#ifdef CONFIG_MMC_BCM
int bcmsdhc_ctrl_slot_is_invalid(u8 ctrl_slot)
{
	if (ctrl_slot > 3)
		return -EINVAL;

	return 0;
}
#endif /*CONFIG_MMC_BCM */

static void __init bcm21553_timer_init(void)
{
	struct timer_config bcm21553_timer_config = {
		.ce_module = TIMER_MODULE_GPT,
		.cs_module = TIMER_MODULE_GPT,
		.cp_cs_module = TIMER_MODULE_GPT,
		.ce_base = (void __iomem *)io_p2v(BCM21553_GPTIMER_BASE),
		.cp_cs_base = (void __iomem *)io_p2v(BCM21553_SLPTMR1_BASE),
		.cs_index = 0,
		.ce_index = 1,
		.cp_cs_index = 0,
		.irq = IRQ_GPT1,
	};

	struct gpt_base_config base_config = {
		.avail_bitmap = GPT_AVAIL_BITMAP,
		.base_addr = (void __iomem *)io_p2v(BCM21553_GPTIMER_BASE),
		.irq = IRQ_GPT1,
	};

	gpt_init(&base_config);
	bcm_timer_init(&bcm21553_timer_config);
}

static struct sys_timer bcm21553_timer = {
	.init = bcm21553_timer_init,
};

int board_sysconfig(uint32_t module, uint32_t op)
{
	static DEFINE_SPINLOCK(bcm_syscfg_lock);
	unsigned long flags, auxmic_base;
	int ret = 0;
	u32 val2 = 0;
	struct clk *usb_phy_clk;
	static bool usb_phy_clk_enabled=false; 
	spin_lock_irqsave(&bcm_syscfg_lock, flags);
	auxmic_base = (unsigned long) io_p2v(BCM21553_AUXMIC_BASE);
	switch (module) {
	case SYSCFG_LCD:
		if(op == SYSCFG_ENABLE) {
			writel(readl(ADDR_SYSCFG_IOCR6) &
				~(SYSCFG_IOCR6_GPIO25_24_MUX),
				ADDR_SYSCFG_IOCR6);
			writel(readl(ADDR_SYSCFG_IOCR2) |
				(SYSCFG_IOCR2_OSC2_ENABLE),
				ADDR_SYSCFG_IOCR2);
		} else {
			/* Configure IOCR0[29].IOCR0[25] = 10 (GPIO[49:44])*/
			/* Configure IOCR0[29].IOCR0[25] = 00 (LCDCTRL,LCDD[0])*/
			writel((readl(ADDR_SYSCFG_IOCR0) &
				~(SYSCFG_IOCR0_LCD_CTRL_MUX | SYSCFG_IOCR0_MPHI_MUX)),
				ADDR_SYSCFG_IOCR0);
			if (op == SYSCFG_INIT) {
				writel((readl(ADDR_SYSCFG_IOCR0) | SYSCFG_IOCR0_LCD_CTRL_MUX),
				ADDR_SYSCFG_IOCR0);
			}
		}
		break;
	case SYSCFG_PWM0:
#define GPIO16_AS_PWM0 ((1 << 10))
#define GPIO16_AS_GPIO (~(GPIO16_AS_PWM0))
			if (op == SYSCFG_ENABLE) {
				writel(readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x00) |
					GPIO16_AS_PWM0,
					io_p2v(BCM21553_SYSCFG_BASE) + 0x00);
			}
			if (op == SYSCFG_DISABLE) {
				writel(readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x00) &
					GPIO16_AS_GPIO,
					io_p2v(BCM21553_SYSCFG_BASE) + 0x00);
			}
		break;
	case (SYSCFG_PWM0 + 1): /* PWM1 => LCD Backlight */
		if ((op == SYSCFG_INIT) || (op == SYSCFG_DISABLE))
			writel(readl(ADDR_SYSCFG_IOCR0) &
				~SYSCFG_IOCR0_GPIO17_PWM0_MUX, ADDR_SYSCFG_IOCR0);
		else if (op == SYSCFG_ENABLE)
			writel(readl(ADDR_SYSCFG_IOCR0) |
				SYSCFG_IOCR0_GPIO17_PWM0_MUX, ADDR_SYSCFG_IOCR0);

		break;
	case SYSCFG_RESETREASON_SOFT_RESET:
		if (op == SYSCFG_INIT) {
		 	ret =  readl(ADDR_SYSCFG_PUMR);

		} else if (op == SYSCFG_DISABLE) {

			writel(PUMR_VAL_SOFT_RESET, ADDR_SYSCFG_PUMR);
		} else if (op == SYSCFG_ENABLE) {
		}
		break;
	case SYSCFG_RESETREASON_AP_ONLY_BOOT:
		if (op == SYSCFG_INIT) {
			ret =  readl(ADDR_SYSCFG_PUMR);

		} else if (op == SYSCFG_DISABLE) {

			writel(PUMR_VAL_AP_ONLY_BOOT, ADDR_SYSCFG_PUMR);
		} else if (op == SYSCFG_ENABLE) {
		}
		break;

	 case SYSCFG_CAMERA:
                 if(op == SYSCFG_INIT){
                        u32 val;
                        /* IOCR 0 */
                        val = readl(ADDR_SYSCFG_IOCR0) & ~(SYSCFG_IOCR0_CAMCK_GPIO_MUX);
                        writel(val, ADDR_SYSCFG_IOCR0);
                        /* IOCR 3 */
                        val = readl(ADDR_SYSCFG_IOCR3);
                         /* Clear bits 6,5 and 4 */
                        val &= ~(SYSCFG_IOCR3_CAMCK_DIS |
				SYSCFG_IOCR3_CAMDCK_PU | SYSCFG_IOCR3_CAMDCK_PD);
                        val |= SYSCFG_IOCR3_CAMDCK_PD;
                        writel(val, ADDR_SYSCFG_IOCR3);
                        /* IOCR 4*/
                        val = readl(ADDR_SYSCFG_IOCR4);
                        /* Bits 14:12 */
                        val |= SYSCFG_IOCR4_CAM_DRV_STGTH(0x7);
                        writel(val, ADDR_SYSCFG_IOCR4);
                  } else if(op == SYSCFG_ENABLE){
                        u32 val;
                         /* IOCR 0 */
                        val = readl(ADDR_SYSCFG_IOCR0) & ~(SYSCFG_IOCR0_CAMCK_GPIO_MUX);
                        writel(val, ADDR_SYSCFG_IOCR0);
                        /* IOCR 3 */
                        val = readl(ADDR_SYSCFG_IOCR3);
                        val &= ~(SYSCFG_IOCR3_CAMCK_DIS | SYSCFG_IOCR3_CAMDCK_PU |
				 SYSCFG_IOCR3_CAMDCK_PD | SYSCFG_IOCR3_CAMHVS_PU |
				 SYSCFG_IOCR3_CAMHVS_PD | SYSCFG_IOCR3_CAMD_PU |
				 SYSCFG_IOCR3_CAMD_PD);
                        val |= SYSCFG_IOCR3_CAMDCK_PD;
//#if defined (CONFIG_CAM_CPI)
   //                     val |=  (SYSCFG_IOCR3_CAMD_PU | SYSCFG_IOCR3_CAMHVS_PD);
//#endif
if(1==camera_id)
	val |=  (SYSCFG_IOCR3_CAMD_PU | SYSCFG_IOCR3_CAMHVS_PD);

                        writel(val, ADDR_SYSCFG_IOCR3);
                        /* IOCR 5 for trace muxing */
                        val = readl(ADDR_SYSCFG_IOCR5);
                        val &= ~(SYSCFG_IOCR5_CAM_TRACE_EN);
//#if defined (CONFIG_CAM_CPI)
		if(1==camera_id)
			val |= SYSCFG_IOCR5_CAM_TRACE_EN;
//#endif

                        writel(val, ADDR_SYSCFG_IOCR5);
                        /* IOCR 4*/
                        val = readl(ADDR_SYSCFG_IOCR4);
                        /* Bits 14:12 Cam drive strength */
                        val |= SYSCFG_IOCR4_CAM_DRV_STGTH(0x7);
                        writel(val, ADDR_SYSCFG_IOCR4);
                        /* IOCR 6 for cam afe and mode */
                        val = readl(ADDR_SYSCFG_IOCR6);
                        val &= ~(SYSCFG_IOCR6_CAM2_CAM1_B);
                        val |= (SYSCFG_IOCR6_CAM2_CAM1_B); /* CAM 1 interface for now */
                        val &= ~(SYSCFG_IOCR6_CAM_MODE(0x3)); /* Clear */
//#if defined (CONFIG_CAM_CPI)
					if(1==camera_id)
                        val |= (SYSCFG_IOCR6_CAM_MODE(0x3));
				else if(camera_id==0)
					val |= (SYSCFG_IOCR6_CAM_MODE(0x2));
//#endif
                        writel(val, ADDR_SYSCFG_IOCR6);
                        printk(KERN_INFO"Board sys Done enable 0x%x\n", readl(ADDR_SYSCFG_IOCR6));
                  } else if(op == SYSCFG_DISABLE) {
			u32 val;
                          /* IOCR 3 */
                        val = readl(ADDR_SYSCFG_IOCR3);
                        val &= ~(SYSCFG_IOCR3_CAMCK_DIS);
                        val |= SYSCFG_IOCR3_CAMCK_DIS; /* Disable CAM outputs */
                        writel(val, ADDR_SYSCFG_IOCR3);
                        /* IOCR 4*/
                        val = readl(ADDR_SYSCFG_IOCR4);
                        /* Bits 14:12 Cam drive strength */
                        val &= ~(SYSCFG_IOCR4_CAM_DRV_STGTH(0x7));
                        /* Drive strength minimum */
                        val |= (SYSCFG_IOCR4_CAM_DRV_STGTH(1));
                        writel(val, ADDR_SYSCFG_IOCR4);
                        /* IOCR 6 for cam afe and mode */
                        val = readl(ADDR_SYSCFG_IOCR6);
                        val &= ~(SYSCFG_IOCR6_CAM_MODE(0x3)); /* Clear */
                        writel(val, ADDR_SYSCFG_IOCR6);
                   }
                break;
	case SYSCFG_SDHC1:
		if (op == SYSCFG_INIT) {

			/* Offset for IOCR0 = 0x00 */
			/* SPI_GPIO_MASK = 0x18 */
			writel((readl(ADDR_SYSCFG_IOCR0) & ~SYSCFG_IOCR0_SD1_MUX(0x3)),
			       ADDR_SYSCFG_IOCR0);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2) &
			       ~(SYSCFG_IOCR2_SD1CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN) |
				 SYSCFG_IOCR2_SD1DAT_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);


			/* Make GPIO36 to pull down */
			bcm_gpio_pull_up_down_enable(BCM2091_CLK_REQ_PASS_THOUGH, true);
			bcm_gpio_pull_up(BCM2091_CLK_REQ_PASS_THOUGH, false);

			/* Make GPIO38 to out direction, and set value 0 */
			writel(readl(ADDR_SYSCFG_IOCR10) &
				~SYSCFG_IOCR10_BSC3_GPIOH_ENABLE,
				ADDR_SYSCFG_IOCR10);
			gpio_request(BCM4325_WLAN_RESET, "sdio1_wlan_reset");
			gpio_direction_output(BCM4325_WLAN_RESET, 0);
			gpio_set_value(BCM4325_WLAN_RESET, 0);
			gpio_free(BCM4325_WLAN_RESET);

			//Set SDIO1 Driving Strength
			writel((readl(ADDR_SYSCFG_IOCR4) | SYSCFG_IOCR4_SD1_DAT_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR4);

			writel((readl(ADDR_SYSCFG_IOCR6) | SYSCFG_IOCR6_SD1_CLK_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR6);

			// This is to Enable the SYS_CLK_REQ From 2091 for P102 WCG WLAN CARD
			writel((readl(ADDR_CLKPWR_CLK_SYSCLK_REQ_MASK) | 0x00000001),
                		ADDR_CLKPWR_CLK_SYSCLK_REQ_MASK);

		} else if (op == SYSCFG_ENABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD1CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD1CMD_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD1DAT_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD1DAT_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);

			//This is to Enable the SYS_CLK_REQ From 2091 for P102 WCG WLAN CARD
			writel((readl(ADDR_CLKPWR_CLK_SYSCLK_REQ_MASK) | 0x00000001),
                		ADDR_CLKPWR_CLK_SYSCLK_REQ_MASK);

			//Set SDIO1 Driving Strength
			writel((readl(ADDR_SYSCFG_IOCR4) | SYSCFG_IOCR4_SD1_DAT_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR4);

			writel((readl(ADDR_SYSCFG_IOCR6) | SYSCFG_IOCR6_SD1_CLK_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR6);

		} else if (op == SYSCFG_DISABLE) {
			/* Offset for IOCR2 = 0x0c */
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD1CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD1CMD_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD1DAT_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD1DAT_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);

		}
		break;
	case SYSCFG_SDHC2:
		if (op == SYSCFG_INIT) {
			/* Offset for IOCR0 = 0x00 */
			writel((readl(ADDR_SYSCFG_IOCR0) |
				SYSCFG_IOCR0_FLASH_SD2_MUX),
			       ADDR_SYSCFG_IOCR0);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2) &
			       ~(SYSCFG_IOCR2_SD2CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN) |
				 SYSCFG_IOCR2_SD2DATL_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
		} else if (op == SYSCFG_ENABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD2CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD2CMD_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD2DATL_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD2DATL_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);
		} else if (op == SYSCFG_DISABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2) &
				~(SYSCFG_IOCR2_SD2CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN) |
			         SYSCFG_IOCR2_SD2DATL_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
		}
		break;
	case SYSCFG_SDHC3:
		
		val2 = readl(ADDR_SYSCFG_IOCR4);
		
		if (op == SYSCFG_INIT) {
			/* Offset for IOCR0 = 0x00 */
			/* MSPRO_SD3 = 0x3 */
			writel((readl(ADDR_SYSCFG_IOCR0) &
				~SYSCFG_IOCR0_SD3_MUX(3)),
			       ADDR_SYSCFG_IOCR0);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2) &
			       ~(SYSCFG_IOCR2_SD3CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN) |
				 SYSCFG_IOCR2_SD3DAT_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			writel((readl(ADDR_SYSCFG_IOCR4) | SYSCFG_IOCR4_SD3_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR4);
			writel((readl(ADDR_SYSCFG_IOCR4) | SYSCFG_IOCR4_SD3_CLK_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR4);
		} else if (op == SYSCFG_ENABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD3CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD3CMD_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       & ~(SYSCFG_IOCR2_SD3DAT_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2)
			       | SYSCFG_IOCR2_SD3DAT_PULL_CTRL(SD_PULL_UP),
			       ADDR_SYSCFG_IOCR2);
			writel((readl(ADDR_SYSCFG_IOCR4) | SYSCFG_IOCR4_SD3_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR4);
			writel((readl(ADDR_SYSCFG_IOCR4) | SYSCFG_IOCR4_SD3_CLK_DRV_STGTH(0x7)),
					ADDR_SYSCFG_IOCR4);
		} else if (op == SYSCFG_DISABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(ADDR_SYSCFG_IOCR2) &
				~(SYSCFG_IOCR2_SD3CMD_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN) |
			         SYSCFG_IOCR2_SD3DAT_PULL_CTRL(SD_PULL_UP | SD_PULL_DOWN)),
			       ADDR_SYSCFG_IOCR2);
		}
		val2 = readl(ADDR_SYSCFG_IOCR4);
		
		break;
#ifdef CONFIG_USB_DWC_OTG
	case SYSCFG_USB:
		if (op == SYSCFG_DISABLE)
			{
				REG_SYS_ANACR9 = (ANACR9_USB_IDLE_ENABLE |
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_UTMI_SOFT_RESET_DISABLE);
				REG_SYS_ANACR9 &= ~(ANACR9_PLL_SUSPEND_ENABLE |
					ANACR9_USB_PLL_POWER_ON);

				REG_CLKPWR_CLK_USB_48_ENABLE &= ~1;
				//REG_CLKPWR_CLK_ANALOG_PHASE_ENABLE |= 2; //bit 1
				if(usb_phy_clk_enabled == true)
				{
					usb_phy_clk=clk_get(NULL, BCM_CLK_USB_PHY_REF_STR_ID);
					clk_disable(usb_phy_clk);
					usb_phy_clk_enabled = false;
				}	
				REG_CLKPWR_CLK_USB_48_ENABLE &= ~1;
				REG_CLKPWR_USBPLL_ENABLE &= ~1;
				REG_SYS_ANACR9 |= ANACR9_USB_AFE_NON_DRVING;

			}
			else if (op == SYSCFG_ENABLE)
			{
				REG_CLKPWR_CLK_USB_48_ENABLE |= 1;
				//REG_CLKPWR_CLK_ANALOG_PHASE_ENABLE &= ~2; //bit 1
				if(usb_phy_clk_enabled == false)
				{
					usb_phy_clk=clk_get(NULL, BCM_CLK_USB_PHY_REF_STR_ID);
					clk_enable(usb_phy_clk);
					usb_phy_clk_enabled = true;
				}
				REG_CLKPWR_CLK_USB_48_ENABLE |= 1;
				REG_CLKPWR_USBPLL_ENABLE |= 1;

				REG_SYS_ANACR9 = (
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_UTMI_SOFT_RESET_DISABLE |
				ANACR9_USB_PLL_POWER_ON |
				ANACR9_USB_PLL_CAL_ENABLE |
				ANACR9_USB_SELECT_OTG_MODE |
				ANACR9_USB_SELECT_DEVICE_MODE |
				ANACR9_PLL_SUSPEND_ENABLE |
				ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS |
				ANACR9_USB_AFE_NON_DRVING);

				/* Reset the PHY since this could
				 * be a mode change too */
				REG_SYS_ANACR9 = (
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_PLL_POWER_ON |
				ANACR9_USB_PLL_CAL_ENABLE |
				ANACR9_USB_SELECT_OTG_MODE |
				ANACR9_USB_SELECT_DEVICE_MODE |
				ANACR9_PLL_SUSPEND_ENABLE |
				ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS |
				ANACR9_USB_AFE_NON_DRVING);

				/* De-activate PHY reset */
				REG_SYS_ANACR9 = (
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_UTMI_SOFT_RESET_DISABLE |
				ANACR9_USB_PLL_POWER_ON |
				ANACR9_USB_PLL_CAL_ENABLE |
				ANACR9_USB_SELECT_OTG_MODE |
				ANACR9_USB_SELECT_DEVICE_MODE |
				ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS |
				ANACR9_PLL_SUSPEND_ENABLE |
				ANACR9_USB_AFE_NON_DRVING);
			}
		break;
#endif
	case SYSCFG_HEADSET:
		/* power up analog block for headset */
		if (op == SYSCFG_INIT)
			writel(SYSCFG_MON5_MIC_PWR_DOWN, ADDR_SYSCFG_ANACR2);
		break;
	case SYSCFG_AUXMIC:
		if (op == SYSCFG_INIT) {
			writel(AUXMIC_PRB_CYC_MS(128), auxmic_base + AUXMIC_PRB_CYC);
			writel(AUXMIC_MSR_DLY_MS(4), auxmic_base + AUXMIC_MSR_DLY);
			writel(AUXMIC_MSR_INTVL_MS(4), auxmic_base + AUXMIC_MSR_INTVL);
			writel(readl(auxmic_base + AUXMIC_CMC) & ~AUXMIC_CMC_PROB_CYC_INF,
				auxmic_base + AUXMIC_CMC);
			writel(readl(auxmic_base + AUXMIC_MIC) | AUXMIC_MIC_SPLT_MSR_INTR,
					auxmic_base + AUXMIC_MIC);
			writel(readl(auxmic_base + AUXMIC_AUXEN) & ~AUXMIC_ENABLE_BIAS,
					auxmic_base + AUXMIC_AUXEN);
			writel(AUXMIC_MICINTH_ADJ_VAL(13),
					auxmic_base + AUXMIC_MICINTH_ADJ);
//			writel(AUXMIC_MICINENTH_ADJ_VAL(9),
//					auxmic_base + AUXMIC_MICINENTH_ADJ);
			writel(AUXMIC_MICONTH_ADJ_VAL(0x00),
					auxmic_base + AUXMIC_MICONTH_ADJ);
			writel(AUXMIC_MICONENTH_ADJ_VAL(0x00),
					auxmic_base + AUXMIC_MICONENTH_ADJ);
			writel(readl(auxmic_base + AUXMIC_F_PWRDWN) & ~AUXMIC_F_PWRDWN_ENABLE,
					auxmic_base + AUXMIC_F_PWRDWN);
			/* Enable the debounce counter for headset insertion and headset button */
			writel((readl(ADDR_IRQ_ICCR) |
					(INTC_ICCR_MICINDBEN | INTC_ICCR_MICINDBC(3)) |
					(INTC_ICCR_MICONDBEN | INTC_ICCR_MICONDBC(2))),
					ADDR_IRQ_ICCR);
		}
		if(op == SYSCFG_DISABLE){
			writel(readl(auxmic_base + AUXMIC_AUXEN) & ~AUXMIC_ENABLE_BIAS,
					auxmic_base + AUXMIC_AUXEN);
	//		writel(readl(auxmic_base + AUXMIC_CMC) | AUXMIC_CMC_PROB_CYC_INF,
	//				auxmic_base + AUXMIC_CMC);
		}
		break;
	case SYSCFG_SPI1:
		/* During init 21:11 is set to 01 to select UARTC
		 * To enable SPI1 set [21:11] =  00
		 * To disable SPI1 set [21:11] = 01 to revert UARTC setting
		 * We toggle Bit 11 to enable/disable SPI, and bit 21 need
		 * not be touched
		 */
		if (op == SYSCFG_ENABLE)	{
			 writel((readl(ADDR_SYSCFG_IOCR0) &
				~(SYSCFG_IOCR0_SPIL_GPIO_MUX
				| SYSCFG_IOCR0_SPIH_GPIO_MUX)),
		                ADDR_SYSCFG_IOCR0);
		}
		/* Set Bits [21:11] = 01 to disable SPI1 and enable UARTC */
		if (op == SYSCFG_DISABLE)	{
			 writel((readl(ADDR_SYSCFG_IOCR0) |
				(SYSCFG_IOCR0_SPIL_GPIO_MUX
				 | SYSCFG_IOCR0_SPIH_GPIO_MUX)),
                		ADDR_SYSCFG_IOCR0);
		}
		break;
	case SYSCFG_SYSTEMINIT:
		if (op == SYSCFG_INIT) {
			u32 val = 0;
			
			/*    IOCR0 BIT 0  to BIT 31 select below    */
			/* GPIO16 working as timed gpio to control vibrator*/
			/*  SD3/SD1/I2S/GPIO[29:28]/PWM1/GPIO16/UARTC/GPEN[7]/GPIO53/GPIO54/GPIO55/
			      AFCPDM/LCDD[17:16]/GPIO56/PCM/DIGMIC_SEL/LCDD[15:1]/LCDD0/FLASH/CAMCK 
			*/
			val = 
				SYSCFG_IOCR0_SPI_UARTC_MUX |
				SYSCFG_IOCR0_GPIO53_GPEN8_L_MUX |
				SYSCFG_IOCR0_GPEN9_SPI_GPIO54_L_MUX |
				SYSCFG_IOCR0_GPEN9_SPI_GPIO54_H_MUX |
				SYSCFG_IOCR0_GPIO55_GPEN10_MUX(0x3) |
				SYSCFG_IOCR0_GPEN11_SPI_GPIO56_L_MUX |
				SYSCFG_IOCR0_GPEN11_SPI_GPIO56_H_MUX |
				SYSCFG_IOCR0_SPIH_GPIO_MUX;
				
			writel(val, ADDR_SYSCFG_IOCR0);
			acar_serial_gpio_set(false); 
			
			/* SD1/SD2/SD3 DAT[0:3]/CMD lines pulled down
			 * UARTA enable internal pullup */
			/* Select GPIO24/GPIO25/GPIO41/soft rest/ANA_SYSCLKEN */
			val = SYSCFG_IOCR2_LCDDATA_PULL_CTRL(0x1) |
				SYSCFG_IOCR2_UARTA_PULL_CTRL(0x1) |
				SYSCFG_IOCR0_GPIO53_GPEN8_H_MUX |
				SYSCFG_IOCR2_OTGCTRL1_GPIO41_MUX(0x3) |
				SYSCFG_IOCR2_SD3CMD_PULL_CTRL(SD_PULL_DOWN) |
				SYSCFG_IOCR2_SD3DAT_PULL_CTRL(SD_PULL_DOWN) |
				SYSCFG_IOCR2_SD1CMD_PULL_CTRL(SD_PULL_DOWN) |
				SYSCFG_IOCR2_SD1DAT_PULL_CTRL(SD_PULL_DOWN) |
				SYSCFG_IOCR2_SD2CMD_PULL_CTRL(SD_PULL_DOWN) |
				SYSCFG_IOCR2_SD2DATL_PULL_CTRL(SD_PULL_DOWN);
			
			writel(val, ADDR_SYSCFG_IOCR2);
			
			/* BIT21 | BIT27 | BIT31 */
			writel(readl(ADDR_SYSCFG_IOCR3) |
				SYSCFG_IOCR3_TWIF_ENB |
				SYSCFG_IOCR3_SIMDAT_PU | SYSCFG_IOCR3_I2C3_EN
				, ADDR_SYSCFG_IOCR3);
			
			/* UARTB enable internal pullup */
			/* Select GPIO[35:30]	GPIO[27:26]*/
			//hanwei@wind-mobi.com add-s 2011.11.18 start
			//config gpio:30 control camera flash light
			//reviewed by liubing@wind-mobi.com
			writel(readl(ADDR_SYSCFG_GPORCMA)&~ADDR_SYSCFG_IOCR5_OFFSET,ADDR_SYSCFG_GPORCMA);
			writel(readl(ADDR_SYSCFG_IOCR5)&~ADDR_SYSCFG_GPORCMA_OFFSET
			    , ADDR_SYSCFG_IOCR5);  
			 //hanwei@wind-mobi.com add-e 2011.11.18 end
			writel(readl(ADDR_SYSCFG_IOCR5) |
			    SYSCFG_IOCR5_GPIO15L_DRV_STGTH(0x1), ADDR_SYSCFG_IOCR5);
			
			/* SD2 DAT[7:4] pulled down */
			/* BIT21 */
			writel(readl(ADDR_SYSCFG_IOCR6) |
			    SYSCFG_IOCR6_SD2DATH_PULL_CTRL(SD_PULL_DOWN)
			    , ADDR_SYSCFG_IOCR6);
			
			/*Select GPIO51, working as GPS_REGPU */ 
			writel(readl(ADDR_SYSCFG_IOCR7) |
				SYSCFG_IOCR7_RFGPIO5_GPIO7_MUX |
			//heweimao@wind-mobi.com 2011.11.04 begin
			//L300 red led gpio48 set bit
			//reviewed by liubing@wind-mobi.com
				SYSCFG_IOCR7_RFGPIO48_GPIO7_MUX,
			//heweimao@wind-mobi.com 2011.11.04 end
					ADDR_SYSCFG_IOCR7);
			
#ifdef CONFIG_ARCH_BCM21553_B1
			/*Disable Hantro performance ECO*/
			writel(readl(ADDR_SYSCFG_IOCR10) |
				SYSCFG_IOCR10_DEC_SYNC_BRG_INCR_TO_INCR4 |
				SYSCFG_IOCR10_ENC_SYNC_BRG_INCR_TO_INCR4,
				ADDR_SYSCFG_IOCR10);
#endif

			/* Enable BSC3 on GPIO7 and GPIO15 */
			/* Enable DIGMICDATA internal pull down to save power */
			writel(readl(ADDR_SYSCFG_IOCR3) |
				SYSCFG_IOCR3_I2C3_EN | (1 << 29),
				ADDR_SYSCFG_IOCR3);
			writel(readl(ADDR_SYSCFG_IOCR10) &
				~SYSCFG_IOCR10_BSC3_GPIOH_ENABLE,
				ADDR_SYSCFG_IOCR10);
//songjinguo@wind-mobi.com 20120520 start
//Change I2C1 pull-up resistors to 995 Ohm;
			writel(readl(ADDR_SYSCFG_IOCR10) | (0x3 << 4),
				ADDR_SYSCFG_IOCR10);
//songjinguo@wind-mobi.com 20120530 end

			/* Config the unused GPIOs to input and pull down*/
			/* Unused GPIOs :
			     	GPIO19 -- reserved for sim protection use
			     	GPIO25 -- reserved for LCD TE
			     	GPIO31, GPIO41
			     	GPIO54 -- reserved for NFC use. Set it to output/low/disable pull to save power
			*/
			/* Set gpio type */
			writel(readl(ADDR_GPIO_TYPE1) &
				(~(0x3 << 6)) & 
				(~(0x3 << 18)) &
				(~(0x3 << 30)), 
				ADDR_GPIO_TYPE1);
			writel(readl(ADDR_GPIO_TYPE2) &
				(~(0x3 << 18)), 
				ADDR_GPIO_TYPE2);
			writel(readl(ADDR_GPIO_TYPE3) |
				(0x2 << 12), 
				ADDR_GPIO_TYPE3);
			
			/*Set pull type */
			writel(readl(ADDR_GPIO_INPUT_PULL_TYPE0) &
				(~(0x1 << 19)) & 
				(~(0x1 << 25)) &
				(~(0x1 << 31)), 
				ADDR_GPIO_INPUT_PULL_TYPE0);
			writel(readl(ADDR_GPIO_INPUT_PULL_TYPE1) &
				(~(0x1 << 9)),
				ADDR_GPIO_INPUT_PULL_TYPE1);
			
			/*Set pull enable/disable*/
			writel(readl(ADDR_GPIO_INPUT_PULL_EN0) |
				(0x1 << 19) | (0x1 << 25) |(0x1 << 31), 
				ADDR_GPIO_INPUT_PULL_EN0);
			writel(readl(ADDR_GPIO_INPUT_PULL_EN1) |
				(0x1 << 9) , 
				ADDR_GPIO_INPUT_PULL_EN1);
			writel(readl(ADDR_GPIO_INPUT_PULL_EN1) 
				& (~(0x1 << 22)), 
				ADDR_GPIO_INPUT_PULL_EN1);

			/*Set output GPIO value*/
			writel(readl(ADDR_GPIO_OUT_VAL2) &
				(~(0x1 << 22)),
				ADDR_GPIO_OUT_VAL2);

			/*GPIO 4/6 are using as Accel int. but our driver use polling method, ignore the int pins.
			For power saving, config them to input, pull down*/
			writel(readl(ADDR_GPIO_TYPE0) &
				(~(0x3 << 8)) & 
				(~(0x3 << 12)), 
				ADDR_GPIO_TYPE0);

			writel(readl(ADDR_GPIO_INPUT_PULL_TYPE0) &
				(~(0x1 << 4)) & 
				(~(0x1 << 6)) , 
				ADDR_GPIO_INPUT_PULL_TYPE0);

			writel(readl(ADDR_GPIO_INPUT_PULL_EN0) |
				(0x1 << 4) | (0x1 << 6), 
				ADDR_GPIO_INPUT_PULL_EN0);

			/*there is a external 1M pull-up for gpio5, so disable internal pull to power saving*/
			writel(readl(ADDR_GPIO_INPUT_PULL_EN0) &
				(~(0x1 << 5)), 
				ADDR_GPIO_INPUT_PULL_EN0);

			

			bcm21553_cam_lmatrix_prio();
#define EXT_CTRL_REQ13 26
#define EXT_CTRL_REQ2	4
			writel(EXT_BY_32AHB_L_CYC << EXT_CTRL_REQ13,
					ADDR_SYSCFG_SYSCONF_AHB_CLK_EXTEND0);
			writel(EXT_BY_32AHB_L_CYC << EXT_CTRL_REQ2,
					ADDR_SYSCFG_SYSCONF_AHB_CLK_EXTEND1);
		}
		break;
#ifdef CONFIG_BRCM_V3D
	case SYSCFG_V3D:
		if (op == SYSCFG_INIT) {
			writel(SYSCFG_V3DRSTR_RST, ADDR_SYSCFG_V3DRSTR);
			writel(~SYSCFG_V3DRSTR_RST, ADDR_SYSCFG_V3DRSTR);
		}
		break;
#endif

	case SYSCFG_COMPASS:
		if (op == SYSCFG_INIT) {
			/* Selecting GPIO53 */
			writel(readl(ADDR_SYSCFG_IOCR0) |
				SYSCFG_IOCR0_GPIO53_GPEN8_L_MUX, ADDR_SYSCFG_IOCR0);
			writel(readl(ADDR_SYSCFG_IOCR2) |
				SYSCFG_IOCR0_GPIO53_GPEN8_H_MUX, ADDR_SYSCFG_IOCR2);
		}
		break;
	case SYSCFG_CSL_DMA:
		if(op == SYSCFG_ENABLE)
			writel(SYSCFG_AHB_PER_CLK_EN, ADDR_SYSCFG_DMAC_AHB_CLK_MODE);
		else if(op == SYSCFG_DISABLE)
			writel(~SYSCFG_AHB_PER_CLK_EN, ADDR_SYSCFG_DMAC_AHB_CLK_MODE);
		break;
	case SYSCFG_CP_START:
		if (op == SYSCFG_INIT) {
			writel(readl(ADDR_SYSCFG_IOCR3) |
				SYSCFG_IOCR3_TWIF_ENB,
				ADDR_SYSCFG_IOCR3);
		}
		break;
#if defined(CONFIG_BCMI2CNFC)		
	case SYSCFG_NFC_GPIO:
		if (op == SYSCFG_INIT) {
			writel((readl(ADDR_SYSCFG_IOCR0) |
				(SYSCFG_IOCR0_GPEN9_SPI_GPIO54_H_MUX | SYSCFG_IOCR0_GPEN9_SPI_GPIO54_L_MUX)),
				ADDR_SYSCFG_IOCR0);
		}
		break;
#endif		
	#if defined(CONFIG_BOARD_L400)		
	case SYSCFG_SERIAL:
	{	/* 21:11 is set to 01 to select UARTC
		 */
		if (op == SYSCFG_ENABLE)	{
			 writel((readl(ADDR_SYSCFG_IOCR0) &
				(SYSCFG_IOCR0_SPIL_GPIO_MUX
				| ~SYSCFG_IOCR0_SPIH_GPIO_MUX)),
		                ADDR_SYSCFG_IOCR0);
					 
		}
		/* Set Bits [21:11] = 11 */
		if (op == SYSCFG_DISABLE)	{
			 writel((readl(ADDR_SYSCFG_IOCR0) |
				(SYSCFG_IOCR0_SPIL_GPIO_MUX
				 | SYSCFG_IOCR0_SPIH_GPIO_MUX)),
                		ADDR_SYSCFG_IOCR0);
			 acar_serial_gpio_set(false);
		}
		break;
	}	
#endif	
	default:
		pr_info("%s: inval arguments\n", __func__);
		spin_unlock_irqrestore(&bcm_syscfg_lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&bcm_syscfg_lock, flags);
	return ret;
}

EXPORT_SYMBOL(board_sysconfig);




static void __init bcm21553_anthenaray_init(void)
{
	/* Configure the SYSCFG Registers */
	board_sysconfig(SYSCFG_SYSTEMINIT, SYSCFG_INIT);
}

static void athenaray_add_platform_data(void)
{
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	bcm21553_serial_device0.dev.platform_data = &serial_platform_data0;
	bcm21553_serial_device1.dev.platform_data = &serial_platform_data1;
	bcm21553_serial_device2.dev.platform_data = &serial_platform_data2;
#endif
#ifdef CONFIG_MMC
	bcm21553_sdhc_slot1.dev.platform_data = &bcm21553_sdhc_data1;
#if !defined(CONFIG_MTD_ONENAND) && !defined(CONFIG_MTD_NAND)
	bcm21553_sdhc_slot2.dev.platform_data = &bcm21553_sdhc_data2;
#endif
	bcm21553_sdhc_slot3.dev.platform_data = &bcm21553_sdhc_data3;
#endif
#ifdef CONFIG_BCM_PWM
	bcm_pwm_device.dev.platform_data = &pwm_dev;
#ifdef CONFIG_BACKLIGHT_FAN5626
	bcm_backlight_devices.dev.platform_data = &bcm_backlight_data;
#endif

/* yaogangxiang@wind-mobi.com 2011.10.21 begin */
#ifdef CONFIG_BACKLIGHT_TPS61165
	bcm_backlight_devices.dev.platform_data = &bcm_backlight_data;
#endif
/* yaogangxiang@wind-mobi.com 2011.10.21 end */

#endif
#if defined(CONFIG_I2C_BOARDINFO)
	bcm21553_device_i2c1.dev.platform_data = &i2c1_host_pdata;
#endif
#ifdef CONFIG_KEYBOARD_BCM
	bcm215xx_kp_device.dev.platform_data = &bcm215xx_keypad_data;
#endif
#ifdef CONFIG_BCM_AUXADC
	auxadc_device.dev.platform_data = &adcplat;
#endif
#ifdef CONFIG_SPI
	bcm21xx_device_spi.dev.platform_data = &bcm21xx_spi_info;
#endif
#if defined(CONFIG_BRCM_HEADSET)  || defined(CONFIG_BRCM_HEADSET_MODULE)
	bcm_headset_device.dev.platform_data = &headset_pd;
#endif
#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))
	board_bcmblt_rfkill_device.dev.platform_data = &board_bcmblt_rfkill_cfg;
#endif
#ifdef CONFIG_BCM215XX_DSS
	bcm215xx_lcdc_device.dev.platform_data = &lcdc_pdata;
#endif
}


static void gpio_led_init(void)
{
	pr_info("gpio_led_init entry\n");

//#define LED_HW_CHECK

#ifdef LED_HW_CHECK

	/* Turn on/off all the LEDs to help check the LED HW*/
	gpio_request(GPIO_BLUE_LED, "gpio_blue_led");
	gpio_direction_output(GPIO_BLUE_LED, 1);
	gpio_free(GPIO_BLUE_LED);
	msleep(2000);
	gpio_request(GPIO_BLUE_LED, "gpio_blue_led");
	gpio_direction_output(GPIO_BLUE_LED, 0);
	gpio_free(GPIO_BLUE_LED);

	gpio_request(GPIO_RED_LED, "gpio_red_led");
	gpio_direction_output(GPIO_RED_LED, 1);
	gpio_free(GPIO_RED_LED);
	msleep(2000);
	gpio_request(GPIO_RED_LED, "gpio_red_led");
	gpio_direction_output(GPIO_RED_LED, 0);
	gpio_free(GPIO_RED_LED);

	gpio_request(GPIO_GREEN_LED, "gpio_green_led");
	gpio_direction_output(GPIO_GREEN_LED, 1);
	gpio_free(GPIO_GREEN_LED);
	msleep(2000);
	gpio_request(GPIO_GREEN_LED, "gpio_green_led");
	gpio_direction_output(GPIO_GREEN_LED, 0);
	gpio_free(GPIO_GREEN_LED);

	gpio_request(GPIO_KEYPAD_BL_LED, "gpio_keypad_bl_led");
	gpio_direction_output(GPIO_KEYPAD_BL_LED, 1);
	gpio_free(GPIO_KEYPAD_BL_LED);
	msleep(2000);
	gpio_request(GPIO_KEYPAD_BL_LED, "gpio_keypad_bl_led");
	gpio_direction_output(GPIO_KEYPAD_BL_LED, 0);
	gpio_free(GPIO_KEYPAD_BL_LED);

	pr_info("gpio led hardware check\n");
#endif


	/* Configure GPIOs to close LEDs to save power */
	gpio_request(GPIO_BLUE_LED, "gpio_blue_led");
	gpio_direction_output(GPIO_BLUE_LED, 0);
	gpio_free(GPIO_BLUE_LED);

	gpio_request(GPIO_RED_LED, "gpio_red_led");
	gpio_direction_output(GPIO_RED_LED, 0);
	gpio_free(GPIO_RED_LED);

	gpio_request(GPIO_GREEN_LED, "gpio_green_led");
	gpio_direction_output(GPIO_GREEN_LED, 0);
	gpio_free(GPIO_GREEN_LED);

	gpio_request(GPIO_KEYPAD_BL_LED, "gpio_keypad_bl_led");
	gpio_direction_output(GPIO_KEYPAD_BL_LED, 0);
	gpio_free(GPIO_KEYPAD_BL_LED);


	pr_info("gpio_led_init exit\n");
}

static int __init arch_late_init(void)
{
	static dma_addr_t dma_address;
	static void *alloc_mem;
	pr_info("arch_late_init\n");

	gpio_request(GPS_CNTIN_CLK_ENABLE, "cntin clock");
	gpio_direction_input(GPS_CNTIN_CLK_ENABLE);
	bcm_gpio_pull_up(GPS_CNTIN_CLK_ENABLE, false);
	bcm_gpio_pull_up_down_enable(GPS_CNTIN_CLK_ENABLE, true);
	gpio_free(GPS_CNTIN_CLK_ENABLE);
	
	gpio_led_init();
/* songjinguo@wind-mobi.com 2011.11.28 start */
/* config gpio 31 for headset deteced pin; */
/* review by liubing */
#define GPIO_HEADSET_DET	(31)
	/* GPIO31_MUX set to GPIO31 */
	writel((readl(ADDR_SYSCFG_IOCR5) & ~(SYSCFG_IOCR5_GPIO31_MUX(0x3))),
				ADDR_SYSCFG_IOCR5);
/* songjinguo@wind-mobi.com 2011.11.28 end */
	/* Disable HEADSET_DETECT GPIO to save power */
	gpio_request(GPIO_HEADSET_DET, "gpio_headset_det");
	bcm_gpio_pull_up_down_enable(GPIO_HEADSET_DET, false);
	gpio_free(GPIO_HEADSET_DET);

#ifdef CONFIG_BOARD_L400_TYPE_EDN10
#define GPIO_GPS_CLK_EN		(22)
	gpio_request(GPIO_GPS_CLK_EN, "gpio_gps_clk_en");
	gpio_direction_output(GPIO_GPS_CLK_EN, 0);
	gpio_free(GPIO_GPS_CLK_EN);
#endif

#define GPIO_GPS_REGPU		(51)
	gpio_request(GPIO_GPS_REGPU, "gpio_gps_regpu");
	gpio_direction_input(GPIO_GPS_REGPU);
	bcm_gpio_pull_up(GPIO_GPS_REGPU, false);
	bcm_gpio_pull_up_down_enable(GPIO_GPS_REGPU, false);
	gpio_free(GPIO_GPS_REGPU);

#define GPIO_CAM_3M_RESET  	(56)
	gpio_request(GPIO_CAM_3M_RESET, "gpio_cam_3M_reset");
	gpio_direction_output(GPIO_CAM_3M_RESET, 0);
	gpio_free(GPIO_CAM_3M_RESET);
#if 0
#define GPIO_SPEAKER_EN  	(13)
	gpio_request(GPIO_SPEAKER_EN, "gpio_speaker_en");
	gpio_direction_output(GPIO_SPEAKER_EN, 1);
	gpio_free(GPIO_SPEAKER_EN);
#endif
#define GPIO_GPS_SYNC  	(55)
	gpio_request(GPIO_GPS_SYNC, "gpio_gps_sync");
	gpio_direction_input(GPIO_GPS_SYNC);
	bcm_gpio_pull_up(GPIO_GPS_SYNC, false);
	bcm_gpio_pull_up_down_enable(GPIO_GPS_SYNC, true);
	gpio_free(GPIO_GPS_SYNC);

#define GPIO_FLASH_TRIGGER		(30)
	gpio_request(GPIO_FLASH_TRIGGER, "gpio_flash_trigger");
	gpio_direction_output(GPIO_FLASH_TRIGGER, 0);
	gpio_free(GPIO_FLASH_TRIGGER);
//songjinguo@wind-mobi.com 2011.11.26 start
//fix the standby larger current, GPIO54 default is pull up, it will consumes 9mA current;
//review by liubing
#define GPIO_NFC_REGPU  	(54)
	gpio_request(GPIO_NFC_REGPU, "gpio_nfc_regpu");
	gpio_direction_output(GPIO_NFC_REGPU,0);
	gpio_free(GPIO_NFC_REGPU);
//songjinguo@wind-mobi.com 2011.11.26 end	

/* songjinguo@wind-mobi.com 2011.11.28 start */
/* config gpio 33 to input and pull up; if pull down will cause to current leakage; */
/* review by liubing */
#define GPIO_FLASH_BB          (33)
       gpio_request(GPIO_FLASH_BB, "gpio_flash_bb");
       gpio_direction_input(GPIO_FLASH_BB);
       bcm_gpio_pull_up(GPIO_FLASH_BB, false);
       bcm_gpio_pull_up_down_enable(GPIO_FLASH_BB, true);
       gpio_free(GPIO_FLASH_BB);
/* songjinguo@wind-mobi.com 2011.11.28 end */
/* songjinguo@wind-mobi.com 2011.12.05 start */
/* set gpio 26 27 for input down, can reduce the standby current */ 
/* review by liubing */
#define TP_SCL_GPIO  (26)
//	gpio_request(TP_SCL_GPIO, "gpio_scl_tp");
	gpio_direction_input(TP_SCL_GPIO);
	bcm_gpio_pull_up(TP_SCL_GPIO, false);
	bcm_gpio_pull_up_down_enable(TP_SCL_GPIO, false);
//	gpio_free(TP_SCL_GPIO);
#define TP_SDA_GPIO  (27)
//	gpio_request(TP_SDA_GPIO, "gpio_sda_tp");
	gpio_direction_input(TP_SDA_GPIO);
	bcm_gpio_pull_up(TP_SDA_GPIO, false);
	bcm_gpio_pull_up_down_enable(TP_SDA_GPIO, false);
//	gpio_free(TP_SDA_GPIO);
/* songjinguo@wind-mobi.com 2011.12.05 end */	
	#if defined (CONFIG_ANDROID_PMEM)
		alloc_mem = cam_mempool_base;
		dma_address = (dma_addr_t) virt_to_phys(alloc_mem);
		if (alloc_mem != NULL) {
                         android_pmem_pdata.start = dma_address;
                         android_pmem_pdata.size = PMEM_ADSP_SIZE;
                         platform_device_register(&android_pmem_device);
                        printk(" ****** %s:Success PMEM alloc 0x%x ****** \n", __func__,
                                 dma_address);
                 } else {
                        printk("******* %s:Fail to alloc memory ****** \n", __func__);
                 }
	#endif

#if defined (CONFIG_TMD2771X) || defined (CONFIG_TMD2771X_MODULE)
    if(gpio_request(TMD2771X_GPIO, "tmd2771x_irq") == 0)
	{
		gpio_direction_input(TMD2771X_GPIO);
		set_irq_type(GPIO_TO_IRQ(TMD2771X_GPIO), IRQF_TRIGGER_FALLING);
    }
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
      if(gpio_request(MPUIRQ_GPIO, "mpu_irq") == 0)
      {
                gpio_direction_input(MPUIRQ_GPIO);
                set_irq_type(GPIO_TO_IRQ(MPUIRQ_GPIO), IRQF_TRIGGER_RISING);
      }
/*
      if(gpio_request(ACCEL_IRQ_GPIO, "mpu_accel_irq") == 0)
      {
                gpio_direction_input(ACCEL_IRQ_GPIO);
                set_irq_type(GPIO_TO_IRQ(ACCEL_IRQ_GPIO), IRQF_TRIGGER_FALLING);
      }
       if(gpio_request(COMPASS_IRQ_GPIO, "mpu_compass_irq") == 0)
      {
                gpio_direction_input(COMPASS_IRQ_GPIO);
                set_irq_type(GPIO_TO_IRQ(COMPASS_IRQ_GPIO), IRQF_TRIGGER_FALLING);
      }
*/

#endif
//songjinguo@wind-mobi.com  20120323 start
//modify for standby larger current
#define CAM_SEC_POWDN 9
	writel((readl(ADDR_SYSCFG_IOCR1) & ~(1<<9)), ADDR_SYSCFG_IOCR1);
	gpio_request(CAM_SEC_POWDN, "gpio_cam_powdn");
	gpio_direction_output(CAM_SEC_POWDN, 0);
	msleep(10);
	gpio_direction_output(CAM_SEC_POWDN, 1);
//songjinguo@wind-mobi.com 20120323 end
	return 0;
}
late_initcall(arch_late_init);

static void __init update_pm_sysparm(void)
{
#if defined(CONFIG_BCM_AVS)
	update_avs_sysparm();
#endif
#if defined(CONFIG_MFD_MAX8986)
	update_regl_sysparm();
#endif
#if defined(CONFIG_MFD_MAX8986)
	update_power_sysparm();
#endif
}

static void __init bcm21553_init_machine(void)
{
#if defined(BOARD_VIRTUALKEY)|| defined(BOARD_VIRTUALKEY_FT5X06) || defined(BOARD_VIRTUALKEY_MXT140)

    struct kobject *properties_kobj;
    int ret;
#endif
	bcm21553_platform_init();
	bcm21553_anthenaray_init();
	athenaray_add_i2c_slaves();
#if defined(CONFIG_I2C_GPIO)	
	bcm21553_add_gpio_i2c_slaves();
#endif
	athenaray_add_platform_data();
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_SPI
	/*Function to register SPI board info : required when spi device is
	   present */
	spi_register_board_info(spi_slave_board_info,
				ARRAY_SIZE(spi_slave_board_info));
#endif
#if 0
        {
                static dma_addr_t dma_address;
                static void *alloc_mem;

                alloc_mem =
                    dma_alloc_coherent(NULL, PMEM_ADSP_SIZE, &dma_address,
                                       GFP_ATOMIC | GFP_DMA);
                if (alloc_mem != NULL) {
                        android_pmem_pdata.start = dma_address;
                        android_pmem_pdata.size = PMEM_ADSP_SIZE;
                        platform_device_register(&android_pmem_device);
                        pr_info("%s:Success PMEM alloc 0x%x\n", __func__,
                                dma_address);
                } else {
                        pr_info("%s:Fail to alloc memory\n", __func__);
                }
        }
#endif

	
	update_pm_sysparm();
#if defined(CONFIG_BCM_CPU_FREQ)
	update_turbo_freq();
#endif
}

/* TODO: Replace BCM1160 with BCM21553/AthenaRay once registered */
//shengbotao@wind-mobi.com 20120530 start
//fix bug12926:Hardware version [21553] change to [21552G]
//review by heweimao@wind-mobi.com
//MACHINE_START(BCM1160, "BCM21553 ThunderbirdEDN31 platform")
MACHINE_START(BCM1160, "BCM21552G platform")
//shengbotao@wind-mobi.com 20120530 end
	/* Maintainer: Broadcom Corporation */
	.phys_io = BCM21553_UART_A_BASE,
	.io_pg_offst = (IO_ADDRESS(BCM21553_UART_A_BASE) >> 18) & 0xfffc,
	.boot_params = (PHYS_OFFSET + 0x100),
	.map_io = bcm21553_map_io,
	.init_irq = bcm21553_init_irq,
	.timer = &bcm21553_timer,
	.init_machine = bcm21553_init_machine,
MACHINE_END

