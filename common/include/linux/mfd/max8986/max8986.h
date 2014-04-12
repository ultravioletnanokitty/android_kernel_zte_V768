/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*       @file   include/linux/mfd/max8986/max8986.h
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
*
*****************************************************************************
*
*  max8986.h
*
*  PURPOSE:
*
*  This file defines the registers of MAX8986.
*
*  NOTES:
*
*****************************************************************************/

#ifndef __MAX8986_H__
#define __MAX8986_H__

#include <linux/mfd/max8986/max8986-private.h>

/*
*CSR output voltage CSRCTRL2[6:2]
*When DVS is on CSR_VOUT_H[4:0] in CSRCTRL10[4:0], CSR_VOUT_L[4:0] in
*CSRCTRL1[4:0], and CSR_VOUT_T[4:0] in CSRCTRL3[4:0] are used for
*CSR output voltage
*/
#ifdef CONFIG_MFD_MAX8999_REV0
enum {
	CSR_VOUT_0_84V,
	CSR_VOUT_0_86V,
	CSR_VOUT_1_46V,
	CSR_VOUT_1_44V,
	CSR_VOUT_1_42V,
	CSR_VOUT_1_40V,
	CSR_VOUT_1_38V,
	CSR_VOUT_1_36V,
	CSR_VOUT_1_34V,
	CSR_VOUT_1_32V,
	CSR_VOUT_1_30V,
	CSR_VOUT_1_28V,
	CSR_VOUT_1_26V,
	CSR_VOUT_1_24V,
	CSR_VOUT_1_22V,
	CSR_VOUT_1_20V,
	CSR_VOUT_1_18V,
	CSR_VOUT_1_16V,
	CSR_VOUT_1_14V,
	CSR_VOUT_1_12V,
	CSR_VOUT_1_10V,
	CSR_VOUT_1_08V,
	CSR_VOUT_1_06V,
	CSR_VOUT_1_04V,
	CSR_VOUT_1_02V,
	CSR_VOUT_1_00V,
	CSR_VOUT_0_98V,
	CSR_VOUT_0_96V,
	CSR_VOUT_0_94V,
	CSR_VOUT_0_92V,
	CSR_VOUT_0_90V,
	CSR_VOUT_0_88V,
	CSR_VOUT_MAX
};
#else
enum {
	CSR_VOUT_0_76V = 0,
	CSR_VOUT_0_78V,
	CSR_VOUT_0_80V,
	CSR_VOUT_0_82V,
	CSR_VOUT_0_84V,		/*default */
	CSR_VOUT_0_86V,
	CSR_VOUT_1_38V,
	CSR_VOUT_1_36V,
	CSR_VOUT_1_34V,
	CSR_VOUT_1_32V,
	CSR_VOUT_1_30V,
	CSR_VOUT_1_28V,
	CSR_VOUT_1_26V,
	CSR_VOUT_1_24V,
	CSR_VOUT_1_22V,
	CSR_VOUT_1_20V,
	CSR_VOUT_1_18V,
	CSR_VOUT_1_16V,
	CSR_VOUT_1_14V,
	CSR_VOUT_1_12V,
	CSR_VOUT_1_10V,
	CSR_VOUT_1_08V,
	CSR_VOUT_1_06V,
	CSR_VOUT_1_04V,
	CSR_VOUT_1_02V,
	CSR_VOUT_1_00V,
	CSR_VOUT_0_98V,
	CSR_VOUT_0_96V,
	CSR_VOUT_0_94V,
	CSR_VOUT_0_92V,
	CSR_VOUT_0_90V,
	CSR_VOUT_0_88V,
	CSR_VOUT_MAX
};
#endif /* CONFIG_MFD_MAX8999_REV0 */

/* Main battery control charging voltage  MBCCTRL3[3:0] */
enum {
	MBCCV_4_20V = 0,	/*normal RC2 mode */
	MBCCV_4_00V,
	MBCCV_4_02V,
	MBCCV_4_04V,
	MBCCV_4_06V,
	MBCCV_4_08V,
	MBCCV_4_10V,
	MBCCV_4_12V,
	MBCCV_4_14V,
	MBCCV_4_16V,
	MBCCV_4_18V,
	MBCCV_4_22V,
	MBCCV_4_24V,
	MBCCV_4_26V,
	MBCCV_4_28V,
	MBCCV_4_35V,
	MBCCV_MAX
};

/* Charging current MBCCTRL4[4:0]
MBCICHFC [4]  - 1: 200mA to 950mA settings
				0: 90mA
*/
typedef enum {
	MAX8986_CHARGING_CURR_90MA = 0,
	MAX8986_CHARGING_CURR_200MA = 0x10,
	MAX8986_CHARGING_CURR_250MA,
	MAX8986_CHARGING_CURR_300MA,
	MAX8986_CHARGING_CURR_350MA,
	MAX8986_CHARGING_CURR_400MA,	/*default */
	MAX8986_CHARGING_CURR_450MA,
	MAX8986_CHARGING_CURR_500MA,
	MAX8986_CHARGING_CURR_550MA,
	MAX8986_CHARGING_CURR_600MA,
	MAX8986_CHARGING_CURR_650MA,
	MAX8986_CHARGING_CURR_700MA,
	MAX8986_CHARGING_CURR_750MA,
	MAX8986_CHARGING_CURR_800MA,
	MAX8986_CHARGING_CURR_850MA,
	MAX8986_CHARGING_CURR_900MA,
	MAX8986_CHARGING_CURR_950MA,
	MAX8986_CHARGING_CURR_MAX,
	MAX8986_CHARGING_CURR_UNKNOWN = 0xFF
}pmu_charging_current;

/* Charger types MUIC STATUS2[2:0] */
typedef enum  {
	PMU_MUIC_CHGTYP_NONE,
	PMU_MUIC_CHGTYP_USB,
	PMU_MUIC_CHGTYP_DOWNSTREAM_PORT,
	PMU_MUIC_CHGTYP_DEDICATED_CHGR,
	PMU_MUIC_CHGTYP_SPL_500MA,
	PMU_MUIC_CHGTYP_SPL_1A,
	PMU_MUIC_CHGTYP_RESERVED,
	PMU_MUIC_CHGTYP_DEAD_BATT_CHG,

	PMU_MUIC_CHGTYP_INIT
}pmu_muic_chgtyp;

/* EOC current MBCCTRL7[3:0] */
enum {
	MAX8986_EOC_50MA = 0,
	MAX8986_EOC_60MA,	/*default */
	MAX8986_EOC_70MA,
	MAX8986_EOC_80MA,
	MAX8986_EOC_90MA,
	MAX8986_EOC_100MA,
	MAX8986_EOC_110MA,
	MAX8986_EOC_120MA,
	MAX8986_EOC_130MA,
	MAX8986_EOC_140MA,
	MAX8986_EOC_150MA,
	MAX8986_EOC_160MA,
	MAX8986_EOC_170MA,
	MAX8986_EOC_180MA,
	MAX8986_EOC_190MA,
	MAX8986_EOC_200MA,
	MAX8986_EOC_MAX
};

/* LDO modes */
enum {
	MAX8986_LDO_NORMAL_MODE = 0,
	MAX8986_LDO_LP_MODE,
	MAX8986_LDO_OFF
};

/* Charger Input OVP threshold OTPCGHCVS[1:0] */
enum {
	MAX8986_OTPCGHCVS_7_5V = 0,
	MAX8986_OTPCGHCVS_6_0V,
	MAX8986_OTPCGHCVS_6_5V,
	MAX8986_OTPCGHCVS_7_0V,
	MAX8986_OTPCGHCVS_MAX
};


#define MAX8986_REG_ENCODE(reg, slave)			((slave << 8) | reg)
#define MAX8986_REG_DECODE(val)				(val & 0xFF)
#define MAX8986_REG_SLAVE(val)				(val >> 8)

/* Power Management registers */
#define MAX8986_PM_I2C_ADDRESS				0x08

#define MAX8986_PM_REG_ID				\
		MAX8986_REG_ENCODE(0x00, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_INT1				\
		MAX8986_REG_ENCODE(0x01, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_INT2				\
		MAX8986_REG_ENCODE(0x02, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_INT3				\
		MAX8986_REG_ENCODE(0x03, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_INTMSK1				\
		MAX8986_REG_ENCODE(0x0B, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_INTMSK2				\
		MAX8986_REG_ENCODE(0x0C, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_INTMSK3				\
		MAX8986_REG_ENCODE(0x0D, MAX8986_PM_I2C_ADDRESS)

#define MAX8986_PM_REG_A1OPMODCTRL			\
		MAX8986_REG_ENCODE(0x16, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D1OPMODCTRL			\
		MAX8986_REG_ENCODE(0x17, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A8OPMODCTRL			\
		MAX8986_REG_ENCODE(0x18, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A2OPMODCTRL			\
		MAX8986_REG_ENCODE(0x19, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_H1OPMODCTRL			\
		MAX8986_REG_ENCODE(0x1A, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_H2OPMODCTRL			\
		MAX8986_REG_ENCODE(0x1B, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D2OPMODCTRL			\
		MAX8986_REG_ENCODE(0x1C, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A5OPMODCTRL			\
		MAX8986_REG_ENCODE(0x1D, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A4OPMODCTRL			\
		MAX8986_REG_ENCODE(0x1E, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_LVOPMODCTRL			\
		MAX8986_REG_ENCODE(0x1F, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_SIMOPMODCTRL			\
		MAX8986_REG_ENCODE(0x20, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_CSRCTRL1				\
		MAX8986_REG_ENCODE(0x21, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_CSRCTRL2				\
		MAX8986_REG_ENCODE(0x22, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_CSRCTRL3				\
		MAX8986_REG_ENCODE(0x23, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_CSROPMODCTRL			\
		MAX8986_REG_ENCODE(0x2A, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_IOSRCTRL2			\
		MAX8986_REG_ENCODE(0x2C, MAX8986_PM_I2C_ADDRESS)
#ifdef CONFIG_MFD_MAX8999_REV0
#define MAX8986_PM_REG_IOSRCTRL3			\
		MAX8986_REG_ENCODE(0x2D, MAX8986_PM_I2C_ADDRESS)
#endif
#define MAX8986_PM_REG_IOSROPMODCTRL			\
		MAX8986_REG_ENCODE(0x34, MAX8986_PM_I2C_ADDRESS)

#define MAX8986_PM_REG_MBCCTRL1				\
		MAX8986_REG_ENCODE(0x36, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_MBCCTRL2				\
		MAX8986_REG_ENCODE(0x37, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_MBCCTRL3				\
		MAX8986_REG_ENCODE(0x38, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_MBCCTRL4				\
		MAX8986_REG_ENCODE(0x39, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_MBCCTRL7				\
		MAX8986_REG_ENCODE(0x3C, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_MBCCTRL8				\
		MAX8986_REG_ENCODE(0x3D, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_ENV1				\
		MAX8986_REG_ENCODE(0x4C, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_ENV2				\
		MAX8986_REG_ENCODE(0x4D, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_HOSTACT				\
		MAX8986_REG_ENCODE(0x51, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_PONKEYBDB			\
		MAX8986_REG_ENCODE(0x5D, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_HFPWRBDB				\
		MAX8986_REG_ENCODE(0x5E, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_BBCCTRL				\
		MAX8986_REG_ENCODE(0x6F, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_OTPCGHCVS			\
		MAX8986_REG_ENCODE(0x7A, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_SMPLSET				\
		MAX8986_REG_ENCODE(0x88, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A5_A3_PWRGRP			\
		MAX8986_REG_ENCODE(0x9E, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_AX1_A7_PWRGRP			\
		MAX8986_REG_ENCODE(0x9F, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_LV_A6_PWRGRP			\
		MAX8986_REG_ENCODE(0xA0, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D2_PWRGRP			\
		MAX8986_REG_ENCODE(0xA1, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_C_IOSR_PWRGRP			\
		MAX8986_REG_ENCODE(0xA2, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A6OPMODCTRL			\
		MAX8986_REG_ENCODE(0xA4, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A3OPMODCTRL			\
		MAX8986_REG_ENCODE(0xA5, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_AX1OPMODCTRL			\
		MAX8986_REG_ENCODE(0xA6, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A7OPMODCTRL			\
		MAX8986_REG_ENCODE(0xA7, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_AX1LDOCTRL			\
		MAX8986_REG_ENCODE(0xA8, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A7LDOCTRL			\
		MAX8986_REG_ENCODE(0xA9, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A1_D1_PWRGRP			\
		MAX8986_REG_ENCODE(0xAA, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A8_A2_PWRGRP			\
		MAX8986_REG_ENCODE(0xAB, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A4SIM_PWRGRP			\
		MAX8986_REG_ENCODE(0xAC, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_HC_PWRGRP			\
		MAX8986_REG_ENCODE(0xAD, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_CSRCTRL10			\
		MAX8986_REG_ENCODE(0xAE, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A1_D1_LDOCTRL			\
		MAX8986_REG_ENCODE(0xB3, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A8_A2_LDOCTRL			\
		MAX8986_REG_ENCODE(0xB4, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_HCLDOCTRL			\
		MAX8986_REG_ENCODE(0xB5, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D2LDOCTRL			\
		MAX8986_REG_ENCODE(0xB6, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A5_A3_LDOCTRL			\
		MAX8986_REG_ENCODE(0xB7, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A4_SIM_LDOCTRL			\
		MAX8986_REG_ENCODE(0xB8, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_LV_A6_LDOCTRL			\
		MAX8986_REG_ENCODE(0xB9, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_PONKEYBDB1			\
		MAX8986_REG_ENCODE(0xBA, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_ADISCHARGE1			\
		MAX8986_REG_ENCODE(0xE0, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_ADISCHARGE2			\
		MAX8986_REG_ENCODE(0xE1, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_ADISCHARGE3			\
		MAX8986_REG_ENCODE(0xE2, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D3OPMODCTRL			\
		MAX8986_REG_ENCODE(0xE3, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D3LDOCTRL			\
		MAX8986_REG_ENCODE(0xE4, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D4OPMODCTRL			\
		MAX8986_REG_ENCODE(0xE5, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_D4LDOCTRL			\
		MAX8986_REG_ENCODE(0xE6, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A9OPMODCTRL			\
		MAX8986_REG_ENCODE(0xE7, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_A9LDOCTRL			\
		MAX8986_REG_ENCODE(0xE8, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_TSROPMODCTRL			\
		MAX8986_REG_ENCODE(0xE9, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_TSRCTRL				\
		MAX8986_REG_ENCODE(0xEA, MAX8986_PM_I2C_ADDRESS)
#define MAX8986_PM_REG_TSRPWRGRP			\
		MAX8986_REG_ENCODE(0xEB, MAX8986_PM_I2C_ADDRESS)

/* RTC registers */
#define MAX8986_RTC_I2C_ADDRESS				0x68

#define MAX8986_RTC_REG_CTRL_MASK			\
		MAX8986_REG_ENCODE(0x02, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_CONTROL				\
		MAX8986_REG_ENCODE(0x03, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_UPDATE1				\
		MAX8986_REG_ENCODE(0x04, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_UPDATE2				\
		MAX8986_REG_ENCODE(0x05, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_WTSR_SMPL			\
		MAX8986_REG_ENCODE(0x06, MAX8986_RTC_I2C_ADDRESS)

#define MAX8986_RTC_REG_SECOND				\
		MAX8986_REG_ENCODE(0x10, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_MINUTE				\
		MAX8986_REG_ENCODE(0x11, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_HOUR				\
		MAX8986_REG_ENCODE(0x12, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_DOW				\
		MAX8986_REG_ENCODE(0x13, MAX8986_RTC_I2C_ADDRESS)	/*day of week */
#define MAX8986_RTC_REG_MONTH				\
		MAX8986_REG_ENCODE(0x14, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_YEAR				\
		MAX8986_REG_ENCODE(0x15, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_DOM				\
		MAX8986_REG_ENCODE(0x16, MAX8986_RTC_I2C_ADDRESS)	/*day ofmonth */

#define MAX8986_RTC_REG_SEC_ALARM1			\
		MAX8986_REG_ENCODE(0x17, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_MIN_ALARM1			\
		MAX8986_REG_ENCODE(0x18, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_HR_ALARM1			\
		MAX8986_REG_ENCODE(0x19, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_DOW_ALARM1			\
		MAX8986_REG_ENCODE(0x1A, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_MONTH_ALARM1			\
		MAX8986_REG_ENCODE(0x1B, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_YEAR_ALARM1			\
		MAX8986_REG_ENCODE(0x1C, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_DOM_ALARM1			\
		MAX8986_REG_ENCODE(0x1D, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_SEC_ALARM2			\
		MAX8986_REG_ENCODE(0x1E, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_MIN_ALARM2			\
		MAX8986_REG_ENCODE(0x1F, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_HR_ALARM2			\
		MAX8986_REG_ENCODE(0x20, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_DOW_ALARM2			\
		MAX8986_REG_ENCODE(0x21, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_MONTH_ALARM2			\
		MAX8986_REG_ENCODE(0x22, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_YEAR_ALARM2			\
		MAX8986_REG_ENCODE(0x23, MAX8986_RTC_I2C_ADDRESS)
#define MAX8986_RTC_REG_DOM_ALARM2			\
		MAX8986_REG_ENCODE(0x24, MAX8986_RTC_I2C_ADDRESS)

/* Audio Subsystem registers */
#define MAX8986_AUDIO_I2C_ADDRESS			0x4D

#define MAX8986_AUDIO_REG_INPUT_GAIN			\
		MAX8986_REG_ENCODE(0x00, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_HP_MIXER			\
		MAX8986_REG_ENCODE(0x01, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_SP_MIXER			\
		MAX8986_REG_ENCODE(0x02, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_HP_LEFT			\
		MAX8986_REG_ENCODE(0x03, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_HP_RIGHT			\
		MAX8986_REG_ENCODE(0x04, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_SPEAKER			\
		MAX8986_REG_ENCODE(0x05, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_RESERVED1			\
		MAX8986_REG_ENCODE(0x06, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_LIMITER			\
		MAX8986_REG_ENCODE(0x07, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_PM				\
		MAX8986_REG_ENCODE(0x08, MAX8986_AUDIO_I2C_ADDRESS)
#define MAX8986_AUDIO_REG_CHARGE_PUMP			\
		MAX8986_REG_ENCODE(0x09, MAX8986_AUDIO_I2C_ADDRESS)

/* MUIC registers */
#define MAX8986_MUIC_I2C_ADDRESS			0x25

#define MAX8986_MUIC_REG_DEVICEID			\
		MAX8986_REG_ENCODE(0x00, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_INT1				\
		MAX8986_REG_ENCODE(0x01, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_INT2				\
		MAX8986_REG_ENCODE(0x02, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_INT3				\
		MAX8986_REG_ENCODE(0x03, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_STATUS1			\
		MAX8986_REG_ENCODE(0x04, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_STATUS2			\
		MAX8986_REG_ENCODE(0x05, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_STATUS3			\
		MAX8986_REG_ENCODE(0x06, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_INTMSK1			\
		MAX8986_REG_ENCODE(0x07, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_INTMASK2			\
		MAX8986_REG_ENCODE(0x08, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_INTMSK3			\
		MAX8986_REG_ENCODE(0x09, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_CDETCTRL			\
		MAX8986_REG_ENCODE(0x0A, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_RFU				\
		MAX8986_REG_ENCODE(0x0B, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_CONTROL1			\
		MAX8986_REG_ENCODE(0x0C, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_CONTROL2			\
		MAX8986_REG_ENCODE(0x0D, MAX8986_MUIC_I2C_ADDRESS)
#define MAX8986_MUIC_REG_CONTROL3			\
		MAX8986_REG_ENCODE(0x0E, MAX8986_MUIC_I2C_ADDRESS)

/*Bitwise definitions for PM registers */

/* INT1 */
#define MAX8986_INT1_PWRONBR		0x01
#define MAX8986_INT1_PWRONBF		(0x01 << 1)
#define MAX8986_INT1_PWRONBH		(0x01 << 2)
#define MAX8986_INT1_RTC60S		(0x01 << 3)
#define MAX8986_INT1_RTCA		(0x01 << 4)
#define MAX8986_INT1_SMPL_INT		(0x01 << 5)
#define MAX8986_INT1_RTC1S		(0x01 << 7)

/* INT2 */
#define MAX8986_INT2_CHGINS		0x01
#define MAX8986_INT2_CHGRM		(0x01 << 1)
/* When charger voltage is higher than Over Voltage Threshold */
#define MAX8986_INT2_CHGERR		(0x01 << 2)
#define MAX8986_INT2_EOC		(0x01 << 3)
/* Main battery Fast charging timer expired */
#define MAX8986_INT2_MBCCHGERR		(0x01 << 7)

/* INT3 */
#define MAX8986_INT3_JIGONBINS		0x01
#define MAX8986_INT3_JIGONBRM		(0x01 << 1)
#define MAX8986_INT3_MUIC		(0x01 << 2)
/* The main batter voltage falling below 3.1V */
#define MAX8986_INT3_VERYLOWBAT		(0x01 << 6)

/* LDO PC1 and PC2 combination MASK */
#define MAX8986_LDO_00_MASK		0x03	/*PC2 = 0, PC1 = 0 */
#define MAX8986_LDO_01_MASK		(0x03 << 2)	/*PC2 = 0, PC1 = 1 */
#define MAX8986_LDO_10_MASK		(0x03 << 4)	/*PC2 = 1, PC1 = 0 */
#define MAX8986_LDO_11_MASK		(0x03 << 6)	/*PC2 = 1, PC1 = 1 */

/* CSRCTRL1 */
#define MAX8986_CSRCTRL1_CSR_VOUT_L_MASK	0x1F
/* CSR DVS enable: 0: disable DVS 1: enable DVS */
#define MAX8986_CSRCTRL1_CSR_DVS_EN	(0x01 << 5)
/* RESET\ dependency on CSR_OK */
#define MAX8986_CSRCTRL1_CSROKACT	(0x01 << 6)

/* CSRCTRL2 */
#define MAX8986_CSRCTRL2_CSR_VOUT_MASK	0x7C	/*NO DVS */

/* CSRCTRL3 */
#define MAX8986_CSRCTRL3_CSR_VOUT_T_MASK	0x1F

/* MBCCTRL1 */
#define MAX8986_MBCCTRL1_TFCH_MASK	(0x07 << 4)
/* Fast charge timer setting for fast charging process MBCCTRL1[6:4] */
#define MAX8986_MBCCTRL1_TFCH_5HR	(0x02 << 4)
#define MAX8986_MBCCTRL1_TFCH_6HR	(0x03 << 4)
#define MAX8986_MBCCTRL1_TFCH_7HR	(0x04 << 4)
#define MAX8986_MBCCTRL1_TFCH_DISABLE (0x07 << 4)

/* MBCCTRL2 */
#define MAX8986_MBCCTRL2_MBCHOSTEN	(0x01 << 6)
/* Charger Fast Charge enable from the host */
#define MAX8986_MBCCTRL2_VCHGR_FC	(0x01 << 7)

/* MBCCTRL3 */
#define MAX8986_MBCCTRL3_MBCCV_MASK	0x0F

/* MBCCTRL4 */
#define MAX8986_MBCCTRL4_MBCICHFC_MASK	0x1F
#define MAX8986_MBCCTRL4_MBCICHFC4	(1 << 4) /*1: 200mA to 950mA settings 0: 90mA */

/* MBCCTRL7 */
#define MAX8986_MBCCTRL7_EOCS_MASK	0x0F

/* MBCCTRL8 */
/* AUTOSTOP Pause of charging process enable/disable */
#define MAX8986_MBCCTRL8_AUTOSTOP	(0x01 << 5)

/* ENV1 (Read only bits) */
/* Charger input presence detected (UVLO<VBUS<OVP) */
#define MAX8986_ENV1_CGPD		(0x01 << 1)
/* Main battery voltage higher than the working voltage */
#define MAX8986_ENV1_MBWV		(0x01 << 3)
/* Charger input voltage exceeds over-voltage threshold */
#define MAX8986_ENV1_CGHC		(0x01 << 4)
/* Charger input voltage is higher than main battery voltage */
#define MAX8986_ENV1_CGMBC		(0x01 << 7)

/* ENV2 (Read Only Bits) */
/* PWRONB pressed with debounce time */
#define MAX8986_ENV2_POWERONB		0x01
/* HF Accessory detected with debounce time */
#define MAX8986_ENV2_JIGONB		(0x01 << 1)

/* HOSTACT */
/* Host controller dictates control of the PMUs power-up/down */
#define MAX8986_HOSTACT_HOSTDICOFF	(0x01 << 2)

/* PWRONBDB */
#define MAX8986_PONKEY_DB_50MS		0x00
#define MAX8986_PONKEY_DB_100MS		0x02
#define MAX8986_PONKEY_DB_500MS		0x03
#define MAX8986_PONKEY_DB_1000MS	0x04
#define MAX8986_PONKEY_DB_2000MS	0x05
#define MAX8986_PONKEY_DB_3000MS	0x06
#define MAX8986_PWRONBHOLD_MASK		0x07
#define MAX8986_PWRONBRF_MASK		(0x07 << 3)
/* PWRONB lock (for turning off PMU only) */
#define MAX8986_PWRONBDB_KEYLOCK	(0x01 << 6)

/* PWRONBDB1 */
#define MAX8986_PWRONBOFFHOLD_MASK	0x07
#define MAX8986_OTPCGHCVS_MASK		0x03


/* OTPCGHCVS */
/* songjinguo@wind-mobi.com 2011.11.30 start */
/* define charge Input VOP chreshold */
/* review by liubing */
#define MAX8986_OTPCGHCVS_MASK_7_5V		0x00	//set VOP 7.5v
#define MAX8986_OTPCGHCVS_MASK_6_0V		0x01	//set VOP 6.0v
#define MAX8986_OTPCGHCVS_MASK_6_5V		0x10	//set VOP 6.5v
#define MAX8986_OTPCGHCVS_MASK_7_0V		0x11	//set VOP 7.0v
/* songjinguo@wind-mobi.com 2011.11.30 end */

/* SMPL */
#define MAX8986_SMPL_ON			0x01

/* CSRCTRL10 */
#define MAX8986_MBCCTRL10_CSR_VOUT_H_MASK	0x1F

/* ACTIVE DISCHARGE1 */
/* 0: No active discharge when disabled. 1: Active discharge when disabled */
#define MAX8986_ALDO1ADISCHG		0x01
#define MAX8986_DLDO1ADISCHG		(0x01 << 1)
#define MAX8986_ALDO8ADISCHG		(0x01 << 2)
#define MAX8986_ALDO2ADISCHG		(0x01 << 3)
#define MAX8986_HCLDO1ADISCHG		(0x01 << 4)
#define MAX8986_HCLDO2ADISCHG		(0x01 << 5)
#define MAX8986_ALDO5ADISCHG		(0x01 << 6)
#define MAX8986_ALDO3ADISCHG		(0x01 << 7)

/* ACTIVE DISCHARGE2 */
#define MAX8986_LVLDOADISCHG		0x01
#define MAX8986_ALDO6ADISCHG		(0x01 << 1)
#define MAX8986_ALDO4ADISCHG		(0x01 << 2)
#define MAX8986_AUXLDO1ADISCHG		(0x01 << 3)
#define MAX8986_ALDO7ADISCHG		(0x01 << 4)
#define MAX8986_DLDO3ADISCHG		(0x01 << 5)
#define MAX8986_DLDO4ADISCHG		(0x01 << 6)
#define MAX8986_ALDO9ADISCHG		(0x01 << 7)

/* Active Discharge3 */
#define MAX8986_DLDO2ADISCHG		0x01
#define MAX8986_IOSRADISCHG		(0x01 << 1)
#define MAX8986_CSRADISCHG		(0x01 << 2)
#define MAX8986_TSRADISCHG		(0x01 << 3)
#define MAX8986_32KHZ			(0x01 << 4)	/* 0: OFF, 1: ON */

/* Bit wise definition of MUIC regusters */

/* MUIC INT1 (read and clear bits) */
#define MAX8986_M_INT1_ADC		0x01	/* ADC Change Interrupt */
/* ADC Low bit change Interrupt */
#define MAX8986_M_INT1_ADCLOW		(0x01 << 1)
#define MAX8986_M_INT1_ADCERR		(0x01 << 2)	/* ADC Error Interrupt */

/* MUIC INT2 */
#define MAX8986_M_INT2_CHGTYP		0x01	/* Charge Type Interrupt */
/* Charger Detection Running Status Interrupt */
#define MAX8986_M_INT2_CHGDETRUN	(0x01 << 1)
#define MAX8986_M_INT2_DCDTMR		(0x01 << 2)	/* DCD Timer Interrupt */
/* Dead Battery Charging Interrupt */
#define MAX8986_M_INT2_DBCHG		(0x01 << 3)
#define MAX8986_M_INT2_VBVOLT		(0x01 << 4)	/* VB Voltage Interrupt */

/* MUIC INT3 */
/* VB Over Voltage Protection Interrupt */
#define MAX8986_M_INT3_OVP		(0x01 << 2)

/* MUIC STATUS1 */
#define MAX8986_M_STATUS1_ADCLOW	(0x01 << 5)	/* Low bit of ADC */
#define MAX8986_M_STATUS1_ADCERR	(0x01 << 6)
#define MAX8986_M_STATUS1_ADC_MASK	0x1F

/* MUIC STATUS2 */
/* Charger Detection State Machine Running */
#define MAX8986_M_STATUS2_CHGDETRUN	(0x01 << 3)
/* Data Contact Detect Time Wait */
#define MAX8986_M_STATUS2_DCDTMR	(0x01 << 4)
/* Dead Battery Charger Mode. If DBChg=1, 45min timer
 * is running and is not expired
 */
#define MAX8986_M_STATUS2_DBCHG		(0x01 << 5)
/* Output of VB detection comparator */
#define MAX8986_M_STATUS2_VBVOLT	(0x01 << 6)
/* Charger type bits */
#define MAX8986_M_STATUS2_CHGTYP_MASK	(0x07)
#define MAX8986_M_STATUS2_CHGTYP_SHIFT	0

/* MUIC STATUS3 */
/* VB Overvoltage Protection Trip Level Indication */
#define MAX8986_M_STATUS3_OVP		(0x01 << 2)

/* MUIC CDETCTRL0 */
/* Enables the USB Charger Detection for a rising edge on VB */
#define MAX8986_M_CDETCTRL0_CHGDETEN	0x01
/* Charger Type Manual Detection */
#define MAX8986_M_CDETCTRL0_CHGTYPM	(0x01 << 1)
/* Enable Data Contact Detect State Machine */
#define MAX8986_M_CDETCTRL0_DCDEN	(0x01 << 2)
/* Automatically exit Data Contact Detection when 2s interrupt is set */
#define MAX8986_M_CDETCTRL0_DCD2SCT	(0x01 << 3)
/* Sets Time for Charger Type Detection */
#define MAX8986_M_CDETCTRL0_DCHKTM	(0x01 << 4)
/* USB Charger Downstream Detection method */
#define MAX8986_M_CDETCTRL0_CDPDET	(0x01 << 7)

/* MUIC Control1 */
#define MAX8986_M_CONTROL1_COMN1SW_MASK	0x07
#define MAX8986_M_CONTROL1_COMN1SW_SHIFT 0
#define MAX8986_M_CONTROL1_COMN1_OPEN	0x00
/* COMN1 connected to DN1(USB) */
#define MAX8986_M_CONTROL1_COMN1_USB	0x01
/* COMN1 connected to SL1 (Audio Left) */
#define MAX8986_M_CONTROL1_COMN1_AUDIO_L	0x02
/* COMN1 connected to UT1 (UART TX) */
#define MAX8986_M_CONTROL1_COMN1_UART_TX	0x03
#define MAX8986_M_CONTROL1_COMP2SW_MASK		(0x07 << 3)
#define MAX8986_M_CONTROL1_COMP2SW_SHIFT	3
#define MAX8986_M_CONTROL1_COMP2_OPEN	0x00
/* COMP2 connected to DN1(USB) */
#define MAX8986_M_CONTROL1_COMP2_USB	0x01
/* COMP2 connected to SL1 (Audio Left) */
#define MAX8986_M_CONTROL1_COMP2_AUDIO_R	0x02
/* COMP2 connected to UT1 (UART TX) */
#define MAX8986_M_CONTROL1_COMP2_UART_TX	0x03
/* Connects MIC to VB Note 1 */
#define MAX8986_M_CONTROL1_MIC_CONNECTED	1
#define MAX8986_M_CONTROL1_MIC_OPEN			0
#define MAX8986_M_CONTROL1_MIC_SHIFT		6

/* MUIC Control2 */
/* Enables no accessory low power pulse mode for ADC */
#define MAX8986_M_CONTROL2_LOWPWR	0x01
/* Manual Control of ADC enable */
#define MAX8986_M_CONTROL2_ADCEN	(0x01 << 1)
/* Controls the charge pump required for analog switch operation */
#define MAX8986_M_CONTROL2_CPEN		(0x01 << 2)
/* Enables Factory Accessory Detection State Machine */
#define MAX8986_M_CONTROL2_ACCDET	(0x01 << 5)
/* Sets if Battery Charger is USB2.0 compliant */
#define MAX8986_M_CONTROL2_USBCPLNT	(0x01 << 6)
/* Sets the position of the click/pop resistors on both SL1 and SR2 */
#define MAX8986_M_CONTROL2_RCPS		(0x01 << 7)

/* MUIC Control3 */
#define MAX8986_M_CONTROL3_WBTH_MASK	(0x03 << 6)
#define MAX8986_M_CONTROL3_WBTH_3_7V	0x00
#define MAX8986_M_CONTROL3_WBTH_3_5V	0x01
#define MAX8986_M_CONTROL3_WBTH_3_3V	0x02
#define MAX8986_M_CONTROL3_WBTH_3_1V	0x03

/* MAX Interrupts */
/* Number of interrupt registers i.e. INT1-3 */
#define MAX8986_NUM_INT_REG		 0x03
#define MAX8986_NUM_MUIC_INT_REG 0x03

typedef enum {
	MAX8986_IRQID_INT1_PWRONBR,
	MAX8986_IRQID_INT1_PWRONBF,
	MAX8986_IRQID_INT1_PWRONBH,
	MAX8986_IRQID_INT1_RTC60S,
	MAX8986_IRQID_INT1_RTCA,
	MAX8986_IRQID_INT1_SMPL,
	MAX8986_IRQID_INT1_RSVD_BIT6,
	MAX8986_IRQID_INT1_RTC1S,

	MAX8986_IRQID_INT2_CHGINS,
	MAX8986_IRQID_INT2_CHGRM,
	MAX8986_IRQID_INT2_CHGERR,
	MAX8986_IRQID_INT2_CHGEOC,
	MAX8986_IRQID_INT2_RSVD_BIT4,
	MAX8986_IRQID_INT2_RSVD_BIT5,
	MAX8986_IRQID_INT2_RSVD_BIT6,
	MAX8986_IRQID_INT2_MBCCHGERR,

	MAX8986_IRQID_INT3_JIGONBINS,
	MAX8986_IRQID_INT3_JIGONBRM,
	MAX8986_IRQID_INT3_MUIC,
	MAX8986_IRQID_INT3_RSVD_BIT3,
	MAX8986_IRQID_INT3_RSVD_BIT4,
	MAX8986_IRQID_INT3_RSVD_BIT5,
	MAX8986_IRQID_INT3_VERYLOWBAT,
	MAX8986_IRQID_INT3_RSVD_BIT6,

	MAX8986_MUIC_INT1_ADC,
	MAX8986_MUIC_INT1_ADCLOW,
	MAX8986_MUIC_INT1_ADCERR,
	MAX8986_MUIC_INT1_BIT3,
	MAX8986_MUIC_INT1_BIT4,
	MAX8986_MUIC_INT1_BIT5,
	MAX8986_MUIC_INT1_BIT6,
	MAX8986_MUIC_INT1_BIT7,

	MAX8986_MUIC_INT2_CHGTYP,
	MAX8986_MUIC_INT2_CHGDETRUN,
	MAX8986_MUIC_INT2_DCDTMR,
	MAX8986_MUIC_INT2_DBCHG,
	MAX8986_MUIC_INT2_VBVOLT,
	MAX8986_MUIC_INT2_BIT5,
	MAX8986_MUIC_INT2_BIT6,
	MAX8986_MUIC_INT2_BIT7,

	MAX8986_MUIC_INT3_BIT0,
	MAX8986_MUIC_INT3_BIT1,
	MAX8986_MUIC_INT3_OVP,
	MAX8986_MUIC_INT3_BIT3,
	MAX8986_MUIC_INT3_BIT4,
	MAX8986_MUIC_INT3_BIT5,
	MAX8986_MUIC_INT3_BIT6,
	MAX8986_MUIC_INT3_BIT7,

	MAX8986_TOTAL_IRQ
} MAX8986_IRQ_ID;

/*MUIC events*/
enum
{
	MAX8986_MUIC_EVENT_CHARGER_TYPE,   /*Params -  holds the charger type*/
	MAX8986_MUIC_EVENT_CHARGER_OVP,
	MAX8986_MUIC_EVENT_DCDTMR,
	MAX8986_MUIC_EVENT_DBC,
	MAX8986_MUIC_EVENT_HEADSET, /*Param - 0 - plugout, 1 - plugin*/
	MAX8986_MUIC_EVENT_HEADSET_BUTTON, /*Param - 0 - release, 1 - press*/
	MAX8986_MUIC_EVENT_MAX
};

/* exported functions */
extern int max89xx_enable_irq(struct max8986 *max8986, int irq);
extern int max89xx_disable_irq(struct max8986 *max8986, int irq);
extern int max89xx_free_irq(struct max8986 *max8986, int irq);
extern int max89xx_request_irq(struct max8986 *max8986, int irq,
			       bool enable_irq, void (*handler) (int, void *),
			       void *data);
extern int max89xx_register_ioctl_handler(struct max8986 *max8986,
					  u8 sub_dev_id,
					  pmu_subdev_ioctl_handler handler,
					  void *pri_data);
extern int max89xx_csr_reg2volt(int reg);

extern int max89xx_muic_register_event_handler(int event,
	   void (*handler)(int, u32, void *),
	   void *data);
extern void max89xx_muic_unregister_event_handler(int event);
extern pmu_muic_chgtyp max89xx_muic_get_charger_type(void);
extern void max89xx_muic_force_charger_detection(void);
extern void pmu_set_charging_current(pmu_charging_current charging_cur);
extern pmu_charging_current pmu_get_charging_current(void);
extern void pmu_start_charging(void);
extern void pmu_stop_charging(void);

#endif /* __MAX8986_H__ */