/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/power/max8986-power.c
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
/**
 *
 *   @file   max8986-muic.c
 *
 *   @brief  MUIC driver for Maxim MAX8986 PMU
 *
 ****************************************************************************/


 #include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mfd/max8986/max8986.h>

// shaojiang@wind-mobi.com 2012.01.09 begin
// fix bug 7061: leds display error
// reviewed by liubing@wind-mobi.com
#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif /*CONFIG_HAS_WAKELOCK*/
// shaojiang@wind-mobi.com 2012.01.09 end

#define CALL_EVENT_HANDLER(event,param) if(max8986_muic->handler[(event)].handler)\
	max8986_muic->handler[(event)].handler((event),\
	(param),\
	max8986_muic->handler[(event)].data)

struct muic_event_handler
{
	void (*handler) (int, u32, void *);
	void* data;
};
struct max8986_muic {
	struct max8986 *max8986;

	/*MUIC state variables */
	u8 adc_low_status;
	u8 dcd_timeout_status; /*DCD - data contact detect*/
	u8 charger_detect_status;
	u8 dbc_status; /*Dead Batt Charging status */

	u8 charger_type;
	u8 mic_pluggedin;
	u8 mic_btn_pressed;
	u8 mic_insert_adc_val;
	u8 mic_btn_press_adc_val;
	struct muic_event_handler handler[MAX8986_MUIC_EVENT_MAX];

// shaojiang@wind-mobi.com 2012.01.09 begin
// fix bug 7061: leds display error
// reviewed by liubing@wind-mobi.com
#if defined(CONFIG_HAS_WAKELOCK)
	struct wake_lock charge_type_wl; 
#endif
// shaojiang@wind-mobi.com 2012.01.09 end
};

static struct max8986_muic* max8986_muic_ptr = NULL;

void max89xx_muic_force_charger_detection()
{
	if(max8986_muic_ptr)
	{
		u8 reg_val;
		struct max8986 *max8986 = max8986_muic_ptr->max8986;
		max8986->read_dev(max8986, MAX8986_MUIC_REG_CDETCTRL, &reg_val);
		reg_val |= MAX8986_M_CDETCTRL0_CHGTYPM;
		max8986->write_dev(max8986, MAX8986_MUIC_REG_CDETCTRL, reg_val);
	}
}
EXPORT_SYMBOL(max89xx_muic_force_charger_detection);

int max89xx_muic_register_event_handler(int event,
					void (*handler)(int, u32, void *),
					void *data)
{
	if(max8986_muic_ptr)
	{
		if(max8986_muic_ptr->handler[event].handler == NULL)
		{
			max8986_muic_ptr->handler[event].handler = handler;
			max8986_muic_ptr->handler[event].data = data;
		}
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(max89xx_muic_register_event_handler);

extern void max89xx_muic_unregister_event_handler(int event)
{
	if(max8986_muic_ptr)
	{
		max8986_muic_ptr->handler[event].handler = NULL;
		max8986_muic_ptr->handler[event].data = NULL;
	}
}
EXPORT_SYMBOL(max89xx_muic_unregister_event_handler);

pmu_muic_chgtyp max89xx_muic_get_charger_type()
{
	if(max8986_muic_ptr)
	{
		printk(KERN_INFO "%s:type = %d\n",__func__,max8986_muic_ptr->charger_type);
		return 	max8986_muic_ptr->charger_type;
	}
	return PMU_MUIC_CHGTYP_NONE;
}
EXPORT_SYMBOL(max89xx_muic_get_charger_type);

static void max89xx_muic_adc_isr(int intr, void *data)
{
	u8 status1,reg_ctrl1;
	u8 adc_val;
	u8 vbus_val;
	
	struct max8986_muic *max8986_muic = data;
	struct max8986 *max8986 = max8986_muic->max8986;

	max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS1, &status1);
	adc_val = MAX8986_M_STATUS1_ADC_MASK & status1;
	pr_info("%s: adc_val = %x\n",__func__,adc_val);
	//feixiaoping@wind_mobi.com 2012-0830 begin
	//modify bug#14666 
	//we use vbus_val value to confim the line is usb line or headset line ;
	max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS2, &status1);
	vbus_val = 0x40 & status1;
	//pr_info("%s: STATUS2 = %x\n",__func__,status1);
		
	//max8986->read_dev(max8986, MAX8986_MUIC_REG_CONTROL1, &status1);
	//pr_info("%s: CONTROL1 = %x\n",__func__,status1);

	//feixiaoping@wind_mobi.com 2012-0830 end

	if((adc_val == PMU_MUIC_ADC_OUTPUT_OPEN)&&(vbus_val==0))
	{
		if(max8986_muic->mic_pluggedin)
		{
			reg_ctrl1 = (MAX8986_M_CONTROL1_MIC_OPEN << MAX8986_M_CONTROL1_MIC_SHIFT) |
						(MAX8986_M_CONTROL1_COMN1_OPEN << MAX8986_M_CONTROL1_COMN1SW_SHIFT) |
						(MAX8986_M_CONTROL1_COMP2_OPEN << MAX8986_M_CONTROL1_COMP2SW_SHIFT);

			max8986->write_dev(max8986, MAX8986_MUIC_REG_CONTROL1, reg_ctrl1);
			CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_HEADSET,0);
			max8986_muic->mic_pluggedin = false;
			max8986_muic->mic_btn_pressed = false;
			pr_info("%s:headet removed\n",__func__);
		}
	}
	else if((adc_val == max8986_muic->mic_insert_adc_val )&&(vbus_val==0))
	{
		if(max8986_muic->mic_btn_pressed)
		{
			max8986_muic->mic_btn_pressed = false;
			CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_HEADSET_BUTTON,0);
			pr_info("%s: headset button released\n",__func__);
		}
		else
		{
			max8986_muic->mic_pluggedin = true;
			reg_ctrl1 = (MAX8986_M_CONTROL1_MIC_CONNECTED << MAX8986_M_CONTROL1_MIC_SHIFT) |
						(MAX8986_M_CONTROL1_COMN1_AUDIO_L << MAX8986_M_CONTROL1_COMN1SW_SHIFT) |
						(MAX8986_M_CONTROL1_COMP2_AUDIO_R << MAX8986_M_CONTROL1_COMP2SW_SHIFT);

			max8986->write_dev(max8986, MAX8986_MUIC_REG_CONTROL1, reg_ctrl1);
			CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_HEADSET,1);
			pr_info("%s:headset insert \n",__func__);
		}
	}
	else if((adc_val == max8986_muic->mic_btn_press_adc_val && max8986_muic->mic_pluggedin)&&(vbus_val==0))
	{
		max8986_muic->mic_btn_pressed = true;
		CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_HEADSET_BUTTON,1);
		pr_info("%s:headset button pressed \n",__func__);
	}
	else
		pr_info("%s: Unhandled ADC isr adc = %d\n", __func__, adc_val);
}


static void max89xx_muic_chgtype_isr(int intr, void *data)
{
	u8 reg_st2,reg_ctrl1;
	u8 type;
	struct max8986_muic *max8986_muic = data;
	struct max8986 *max8986 = max8986_muic->max8986;

	max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS2, &reg_st2);
	type = (MAX8986_M_STATUS2_CHGTYP_MASK & reg_st2) >> MAX8986_M_STATUS2_CHGTYP_SHIFT;

	pr_info("%s: type = %d\n", __func__, type);
	switch(type)
	{
	case PMU_MUIC_CHGTYP_NONE:
		pr_info("%s: Charger not connected/removed..... ", __func__);
		reg_ctrl1 = (MAX8986_M_CONTROL1_MIC_OPEN << MAX8986_M_CONTROL1_MIC_SHIFT) |
						(MAX8986_M_CONTROL1_COMN1_OPEN << MAX8986_M_CONTROL1_COMN1SW_SHIFT) |
						(MAX8986_M_CONTROL1_COMP2_OPEN << MAX8986_M_CONTROL1_COMP2SW_SHIFT);

		max8986->write_dev(max8986, MAX8986_MUIC_REG_CONTROL1, reg_ctrl1);
		break;

	case PMU_MUIC_CHGTYP_USB:
		pr_info("%s: USB Plugged in..... ", __func__);
		reg_ctrl1 = (MAX8986_M_CONTROL1_MIC_OPEN << MAX8986_M_CONTROL1_MIC_SHIFT) |
						(MAX8986_M_CONTROL1_COMN1_USB << MAX8986_M_CONTROL1_COMN1SW_SHIFT) |
						(MAX8986_M_CONTROL1_COMP2_USB << MAX8986_M_CONTROL1_COMP2SW_SHIFT);

		max8986->write_dev(max8986, MAX8986_MUIC_REG_CONTROL1, reg_ctrl1);
		break;

	case PMU_MUIC_CHGTYP_DOWNSTREAM_PORT:
		pr_info("%s: Charger type:PMU_MUIC_CHGTYP_DOWNSTREAM_PORT..... ", __func__);
		reg_ctrl1 = (MAX8986_M_CONTROL1_MIC_OPEN << MAX8986_M_CONTROL1_MIC_SHIFT) |
						(MAX8986_M_CONTROL1_COMN1_USB << MAX8986_M_CONTROL1_COMN1SW_SHIFT) |
						(MAX8986_M_CONTROL1_COMP2_USB << MAX8986_M_CONTROL1_COMP2SW_SHIFT);

		max8986->write_dev(max8986, MAX8986_MUIC_REG_CONTROL1, reg_ctrl1);
		if (max8986->pdata->pmu_event_cb)
			max8986->pdata->pmu_event_cb(PMU_EVENT_CHARGER_INSERT,
					3);

		break;
		break;
	case PMU_MUIC_CHGTYP_DEDICATED_CHGR:
		pr_info("%s: Charger type:PMU_MUIC_CHGTYP_DEDICATED_CHGR..... ", __func__);
		break;
	case PMU_MUIC_CHGTYP_SPL_500MA:
		pr_info("%s: Charger type:PMU_MUIC_CHGTYP_SPL_500MA..... ", __func__);
		break;
	case PMU_MUIC_CHGTYP_SPL_1A:
		pr_info("%s: Charger type:PMU_MUIC_CHGTYP_SPL_1A..... ", __func__);
		break;

	case PMU_MUIC_CHGTYP_DEAD_BATT_CHG:
		pr_info("%s: PMU_MUIC_CHGTYP_DEAD_BATT_CHG..... ", __func__);
		break;

	default:
		pr_info("%s: ERROR -- unknown charger type..... ", __func__);
		return;
	}
	max8986_muic->charger_type = type;
	CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_CHARGER_TYPE,type);
}

static void max89xx_muic_isr(int intr, void *data)
{
	u8 reg_val;
	struct max8986_muic *max8986_muic = data;
	struct max8986 *max8986 = max8986_muic->max8986;

	pr_info("%s: intr = %d\n", __func__, intr);
	
// shaojiang@wind-mobi.com 2012.01.09 begin
// fix bug 7061: leds display error
// reviewed by liubing@wind-mobi.com
#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_timeout(&max8986_muic->charge_type_wl, msecs_to_jiffies(5000));
#endif
// shaojiang@wind-mobi.com 2012.01.09 end

	switch(intr)
	{
	case MAX8986_MUIC_INT2_CHGDETRUN:

		max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS2, &reg_val);
		max8986_muic->charger_detect_status = (MAX8986_M_STATUS2_CHGDETRUN & reg_val) ?
															1 : 0 ;
		pr_info("%s: MAX8986_MUIC_INT2_CHGDETRUN - %d\n", __func__, max8986_muic->charger_detect_status);
		break;

	case MAX8986_MUIC_INT2_DCDTMR:
		max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS2, &reg_val);
		max8986_muic->dcd_timeout_status = (MAX8986_M_STATUS2_DCDTMR & reg_val) ?
															1 : 0 ;
		pr_info("%s: MAX8986_MUIC_INT2_DCDTMR - %d\n", __func__, max8986_muic->dcd_timeout_status);
		CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_DCDTMR,max8986_muic->dcd_timeout_status);
		break;

	case MAX8986_MUIC_INT2_DBCHG:
		max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS2, &reg_val);
		max8986_muic->dbc_status = (MAX8986_M_STATUS2_DBCHG & reg_val) ?
															1 : 0 ;
		pr_info("%s: MAX8986_MUIC_INT2_DBCHG - %d\n", __func__, max8986_muic->dbc_status);
		CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_DBC,max8986_muic->dbc_status);
		break;

	case MAX8986_MUIC_INT2_VBVOLT:
		max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS2, &reg_val);
		pr_info("%s: MAX8986_MUIC_INT2_VBVOLT - %d\n", __func__, MAX8986_M_STATUS2_VBVOLT & reg_val);
		break;

	case MAX8986_MUIC_INT3_OVP:
		max8986->read_dev(max8986, MAX8986_PM_REG_ENV1, &reg_val);
		pr_info("%s: MAX8986_MUIC_INT3_OVP - %d\n", __func__, MAX8986_ENV1_CGHC & reg_val);
		CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_CHARGER_OVP,MAX8986_ENV1_CGHC & reg_val);
		break;

	case MAX8986_MUIC_INT1_ADCLOW:
		max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS1, &reg_val);
		max8986_muic->adc_low_status = (MAX8986_M_STATUS1_ADCLOW & reg_val) ?
															1 : 0 ;
		pr_info("%s: MAX8986_MUIC_INT1_ADCLOW - %d\n", __func__, max8986_muic->adc_low_status);
		break;
	case MAX8986_MUIC_INT1_ADCERR:
		max8986->read_dev(max8986, MAX8986_MUIC_REG_STATUS1, &reg_val);
		max8986_muic->mic_pluggedin = false;
		max8986_muic->mic_btn_pressed = false;
		pr_info("%s: MAX8986_MUIC_INT1_ADCERR - %d\n", __func__, MAX8986_M_STATUS1_ADCERR & reg_val);
		break;
	}
}

static int __devinit max89xx_muic_probe(struct platform_device *pdev)
{
	struct max8986_muic *max8986_muic;
	struct max8986 *max8986 = dev_get_drvdata(pdev->dev.parent);
	int ret = 0;
	u8 reg_val;

	pr_info("%s\n", __func__);

	max8986_muic = kzalloc(sizeof(struct max8986_muic), GFP_KERNEL);
	if (unlikely(!max8986_muic))
	{
		pr_err("%s: max8986_muic memory alloc failed\n", __func__);
		return -ENOMEM;
	}
	max8986_muic->max8986 = max8986;
	max8986_muic->adc_low_status = 0;
	max8986_muic->mic_pluggedin = false;
	max8986_muic->mic_btn_pressed = false;
	max8986_muic->dcd_timeout_status = 0;
	max8986_muic->charger_detect_status = 0;
	max8986_muic->dbc_status	= 0;
	max8986_muic->charger_type = PMU_MUIC_CHGTYP_NONE;
	max8986_muic->mic_insert_adc_val = PMU_MUIC_ADC_OUTPUT_1M;
	max8986_muic->mic_btn_press_adc_val = PMU_MUIC_ADC_OUTPUT_2K;

	max8986_muic_ptr = max8986_muic;
	platform_set_drvdata(pdev, max8986_muic);
	if(max8986->pdata->muic_pdata)
	{
		max8986_muic->mic_insert_adc_val = max8986->pdata->muic_pdata->mic_insert_adc_val;
		max8986_muic->mic_btn_press_adc_val = max8986->pdata->muic_pdata->mic_btn_press_adc_val ;
	}
	reg_val = MAX8986_M_CDETCTRL0_CHGDETEN | MAX8986_M_CDETCTRL0_DCD2SCT | MAX8986_M_CDETCTRL0_DCDEN;
	max8986->write_dev(max8986, MAX8986_MUIC_REG_CDETCTRL, reg_val);

// shaojiang@wind-mobi.com 2012.01.09 begin
// fix bug 7061: leds display error
// reviewed by liubing@wind-mobi.com
#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_init(&max8986_muic->charge_type_wl, WAKE_LOCK_SUSPEND,
							__stringify(charge_type_wl));
#endif
// shaojiang@wind-mobi.com 2012.01.09 end

	/*Register interrupt handlers*/
	max89xx_request_irq(max8986, MAX8986_MUIC_INT2_CHGTYP, true,
	/*charger type detected interrupt */
	max89xx_muic_chgtype_isr, max8986_muic);
	max89xx_request_irq(max8986, MAX8986_MUIC_INT1_ADC, true,
	max89xx_muic_adc_isr, max8986_muic);  /*MUIC ADC interrupt */
		/*charger detection running status interrupt */
	max89xx_request_irq(max8986, MAX8986_MUIC_INT2_CHGDETRUN, true,
		max89xx_muic_isr, max8986_muic);
	max89xx_request_irq(max8986, MAX8986_MUIC_INT2_DCDTMR, true,
		max89xx_muic_isr, max8986_muic);  /*DCD timer interrupt */
	max89xx_request_irq(max8986, MAX8986_MUIC_INT2_DBCHG, true,
		/*Dead battery charging interrupt */
		max89xx_muic_isr, max8986_muic);
	max89xx_request_irq(max8986, MAX8986_MUIC_INT2_VBVOLT, true,
		max89xx_muic_isr, max8986_muic);  /*VB voltage interrupt */
	/*VB Over voltage protection interrupt */
	max89xx_request_irq(max8986, MAX8986_MUIC_INT3_OVP, true,
		max89xx_muic_isr, max8986_muic);
	max89xx_request_irq(max8986, MAX8986_MUIC_INT1_ADCLOW, true,
		max89xx_muic_isr, max8986_muic);
	max89xx_request_irq(max8986, MAX8986_MUIC_INT1_ADCERR, true,
		max89xx_muic_isr, max8986_muic);

	max89xx_muic_adc_isr(MAX8986_MUIC_INT1_ADC, max8986_muic);

	return ret;
}

static int __devexit max89xx_muic_remove(struct platform_device *pdev)
{
	struct max8986_muic *max8986_muic = platform_get_drvdata(pdev);
// shaojiang@wind-mobi.com 2012.01.09 begin
// fix bug 7061: leds display error
//reviewed by liubing@wind-mobi.com
	if (max8986_muic)
	{	
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock_destroy(&max8986_muic->charge_type_wl);	
#endif
		kfree(max8986_muic);
	}
// shaojiang@wind-mobi.com 2012.01.09 begin
	return 0;
}

static struct platform_driver max8986_muic_driver = {
	.driver = {
		.name = "max8986-muic",
		.owner = THIS_MODULE,
	},
	.remove = __devexit_p(max89xx_muic_remove),
	.probe = max89xx_muic_probe,
};

static int __init max89xx_muic_init(void)
{
	return platform_driver_register(&max8986_muic_driver);
}

module_init(max89xx_muic_init);

static void __exit max89xx_muic_exit(void)
{
	platform_driver_unregister(&max8986_muic_driver);
}

module_exit(max89xx_muic_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MUIC Driver for Maxim MAX89xx PMU");
MODULE_ALIAS("platform:max89xx-muic");
