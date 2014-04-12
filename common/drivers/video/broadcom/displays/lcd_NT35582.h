/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/displays/lcd_NT35582.h
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
*  lcd_NT35582.h
*
*  PURPOSE:
*    This is the LCD-specific code for a Novatek NT35582 module.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#ifndef __BCM_LCD_NT35582_H
#define __BCM_LCD_NT35582_H

/*  LCD command definitions */
#define NT35582_NOP                 0x0000
#define NT35582_SOFT_RST            0x0100
#define NT35582_GET_PWR_MODE        0x0A00
#define NT35582_GET_ADDR_MODE       0x0B00
#define NT35582_GET_PIXEL_FMT       0x0C00
#define NT35582_GET_DISP_MODE       0x0D00
#define NT35582_GET_SIGNAL_MODE     0x0E00
#define NT35582_SLEEP_IN            0x1000
#define NT35582_SLEEP_OUT           0x1100
#define NT35582_PARTIAL_ON          0x1200
#define NT35582_NORMAL_ON           0x1300
#define NT35582_INVERT_OFF          0x2000
#define NT35582_INVERT_ON           0x2100
#define NT35582_DISPLAY_OFF         0x2800
#define NT35582_DISPLAY_ON          0x2900
#define NT35582_SET_HOR_ADDR_S_MSB  0x2A00      /* RAM WIN X S   0x00 */
#define NT35582_SET_HOR_ADDR_S_LSB  0x2A01      /*               0x00 */
#define NT35582_SET_HOR_ADDR_E_MSB  0x2A02      /* RAM WIN X E   0x01 */
#define NT35582_SET_HOR_ADDR_E_LSB  0x2A03      /*               0xDF  479 */
#define NT35582_SET_VER_ADDR_S_MSB  0x2B00      /* RAM WIN Y S   0x00 */
#define NT35582_SET_VER_ADDR_S_LSB  0x2B01      /*               0x00 */
#define NT35582_SET_VER_ADDR_E_MSB  0x2B02      /* RAM WIN Y E   0x03 */
#define NT35582_SET_VER_ADDR_E_LSB  0x2B03      /*               0x1F  799 */
#define NT35582_WR_MEM_START        0x2C00
#define NT35582_SET_RAM_ADDR_X_MSB  0x2D00      /* RAM ADDR X S */
#define NT35582_SET_RAM_ADDR_X_LSB  0x2D01
#define NT35582_SET_RAM_ADDR_Y_MSB  0x2D02      /* RAM ADDR Y S */
#define NT35582_SET_RAM_ADDR_Y_LSB  0x2D03
#define NT35582_RD_MEM_START        0x2E00
#define NT35582_SET_PART_Y_S_MSB    0x3000
#define NT35582_SET_PART_Y_S_LSB    0x3001
#define NT35582_SET_PART_Y_E_MSB    0x3002
#define NT35582_SET_PART_Y_E_LSB    0x3003
#define NT35582_SET_TEAR_OFF        0x3400
#define NT35582_SET_TEAR_ON         0x3500
#define NT35582_SET_ADDR_MODE_0     0x3600
#define NT35582_SET_ADDR_MODE_1     0x3601
#define NT35582_IDLE_OFF            0x3800
#define NT35582_IDLE_ON             0x3900
#define NT35582_SET_PIXEL_FORMAT    0x3A00
#define NT35582_RGB_CTRL            0x3B00
#define NT35582_VBP                 0x3B02
#define NT35582_VFP                 0x3B03
#define NT35582_HBP                 0x3B04
#define NT35582_HFP                 0x3B05
#define NT35582_SET_TEAR_LINE_MSB   0x4400
#define NT35582_SET_TEAR_LINE_LSB   0x4401
#define NT35582_SET_VPA             0xB100
#define NT35582_SET_TLINE_MSB       0xB101
#define NT35582_SET_TLINE_LSB       0xB102
#define NT35582_SD_OP_SET_0         0xB600
#define NT35582_SD_OP_SET_1         0xB601
#define NT35582_SD_OP_SET_2         0xB602
#define NT35582_SD_OP_SET_3         0xB603
#define NT35582_SD_OP_SET_4         0xB604
#define NT35582_SD_OP_SET_5         0xB605
#define NT35582_PWCTR1_0            0xC000
#define NT35582_PWCTR1_1            0xC001
#define NT35582_PWCTR1_2            0xC002
#define NT35582_PWCTR1_3            0xC003
#define NT35582_PWCTR2_0            0xC100
#define NT35582_PWCTR2_1            0xC101
#define NT35582_PWCTR2_2            0xC102
#define NT35582_PWCTR3_0            0xC200
#define NT35582_PWCTR3_1            0xC201
#define NT35582_PWCTR3_2            0xC202
#define NT35582_PWCTR3_3            0xC203
#define NT35582_PWCTR4_0            0xC300
#define NT35582_PWCTR4_1            0xC301
#define NT35582_PWCTR4_2            0xC302
#define NT35582_PWCTR4_3            0xC303
#define NT35582_PWCTR5_0            0xC400
#define NT35582_PWCTR5_1            0xC401
#define NT35582_PWCTR5_2            0xC402
#define NT35582_PWCTR5_3            0xC403
#define NT35582_VCOM                0xC700
#define NT35582_RVCOM               0xC800
#define NT35582_GMACTRL_1_00        0xE000
#define NT35582_GMACTRL_1_01        0xE001
#define NT35582_GMACTRL_1_02        0xE002
#define NT35582_GMACTRL_1_03        0xE003
#define NT35582_GMACTRL_1_04        0xE004
#define NT35582_GMACTRL_1_05        0xE005
#define NT35582_GMACTRL_1_06        0xE006
#define NT35582_GMACTRL_1_07        0xE007
#define NT35582_GMACTRL_1_08        0xE008
#define NT35582_GMACTRL_1_09        0xE009
#define NT35582_GMACTRL_1_0A        0xE00A
#define NT35582_GMACTRL_1_0B        0xE00B
#define NT35582_GMACTRL_1_0C        0xE00C
#define NT35582_GMACTRL_1_0D        0xE00D
#define NT35582_GMACTRL_1_0E        0xE00E
#define NT35582_GMACTRL_1_0F        0xE00F
#define NT35582_GMACTRL_1_10        0xE010
#define NT35582_GMACTRL_1_11        0xE011
#define NT35582_GMACTRL_2_00        0xE100
#define NT35582_GMACTRL_2_01        0xE101
#define NT35582_GMACTRL_2_02        0xE102
#define NT35582_GMACTRL_2_03        0xE103
#define NT35582_GMACTRL_2_04        0xE104
#define NT35582_GMACTRL_2_05        0xE105
#define NT35582_GMACTRL_2_06        0xE106
#define NT35582_GMACTRL_2_07        0xE107
#define NT35582_GMACTRL_2_08        0xE108
#define NT35582_GMACTRL_2_09        0xE109
#define NT35582_GMACTRL_2_0A        0xE10A
#define NT35582_GMACTRL_2_0B        0xE10B
#define NT35582_GMACTRL_2_0C        0xE10C
#define NT35582_GMACTRL_2_0D        0xE10D
#define NT35582_GMACTRL_2_0E        0xE10E
#define NT35582_GMACTRL_2_0F        0xE10F
#define NT35582_GMACTRL_2_10        0xE110
#define NT35582_GMACTRL_2_11        0xE111
#define NT35582_GMACTRL_3_00        0xE200
#define NT35582_GMACTRL_3_01        0xE201
#define NT35582_GMACTRL_3_02        0xE202
#define NT35582_GMACTRL_3_03        0xE203
#define NT35582_GMACTRL_3_04        0xE204
#define NT35582_GMACTRL_3_05        0xE205
#define NT35582_GMACTRL_3_06        0xE206
#define NT35582_GMACTRL_3_07        0xE207
#define NT35582_GMACTRL_3_08        0xE208
#define NT35582_GMACTRL_3_09        0xE209
#define NT35582_GMACTRL_3_0A        0xE20A
#define NT35582_GMACTRL_3_0B        0xE20B
#define NT35582_GMACTRL_3_0C        0xE20C
#define NT35582_GMACTRL_3_0D        0xE20D
#define NT35582_GMACTRL_3_0E        0xE20E
#define NT35582_GMACTRL_3_0F        0xE20F
#define NT35582_GMACTRL_3_10        0xE210
#define NT35582_GMACTRL_3_11        0xE211
#define NT35582_GMACTRL_4_00        0xE300
#define NT35582_GMACTRL_4_01        0xE301
#define NT35582_GMACTRL_4_02        0xE302
#define NT35582_GMACTRL_4_03        0xE303
#define NT35582_GMACTRL_4_04        0xE304
#define NT35582_GMACTRL_4_05        0xE305
#define NT35582_GMACTRL_4_06        0xE306
#define NT35582_GMACTRL_4_07        0xE307
#define NT35582_GMACTRL_4_08        0xE308
#define NT35582_GMACTRL_4_09        0xE309
#define NT35582_GMACTRL_4_0A        0xE30A
#define NT35582_GMACTRL_4_0B        0xE30B
#define NT35582_GMACTRL_4_0C        0xE30C
#define NT35582_GMACTRL_4_0D        0xE30D
#define NT35582_GMACTRL_4_0E        0xE30E
#define NT35582_GMACTRL_4_0F        0xE30F
#define NT35582_GMACTRL_4_10        0xE310
#define NT35582_GMACTRL_4_11        0xE311
#define NT35582_GMACTRL_5_00        0xE400
#define NT35582_GMACTRL_5_01        0xE401
#define NT35582_GMACTRL_5_02        0xE402
#define NT35582_GMACTRL_5_03        0xE403
#define NT35582_GMACTRL_5_04        0xE404
#define NT35582_GMACTRL_5_05        0xE405
#define NT35582_GMACTRL_5_06        0xE406
#define NT35582_GMACTRL_5_07        0xE407
#define NT35582_GMACTRL_5_08        0xE408
#define NT35582_GMACTRL_5_09        0xE409
#define NT35582_GMACTRL_5_0A        0xE40A
#define NT35582_GMACTRL_5_0B        0xE40B
#define NT35582_GMACTRL_5_0C        0xE40C
#define NT35582_GMACTRL_5_0D        0xE40D
#define NT35582_GMACTRL_5_0E        0xE40E
#define NT35582_GMACTRL_5_0F        0xE40F
#define NT35582_GMACTRL_5_10        0xE410
#define NT35582_GMACTRL_5_11        0xE411
#define NT35582_GMACTRL_6_00        0xE500
#define NT35582_GMACTRL_6_01        0xE501
#define NT35582_GMACTRL_6_02        0xE502
#define NT35582_GMACTRL_6_03        0xE503
#define NT35582_GMACTRL_6_04        0xE504
#define NT35582_GMACTRL_6_05        0xE505
#define NT35582_GMACTRL_6_06        0xE506
#define NT35582_GMACTRL_6_07        0xE507
#define NT35582_GMACTRL_6_08        0xE508
#define NT35582_GMACTRL_6_09        0xE509
#define NT35582_GMACTRL_6_0A        0xE50A
#define NT35582_GMACTRL_6_0B        0xE50B
#define NT35582_GMACTRL_6_0C        0xE50C
#define NT35582_GMACTRL_6_0D        0xE50D
#define NT35582_GMACTRL_6_0E        0xE50E
#define NT35582_GMACTRL_6_0F        0xE50F
#define NT35582_GMACTRL_6_10        0xE510
#define NT35582_GMACTRL_6_11        0xE511


#define PIXEL_FORMAT_RGB565  0x55   /* for MPU & RGB interface */
#define PIXEL_FORMAT_RGB666  0x66   /* for MPU & RGB interface */
#define PIXEL_FORMAT_RGB888  0x77   /* for MPU & RGB interface */

#ifdef CONFIG_ENABLE_HVGA
#define LCD_HEIGHT		480
#define LCD_WIDTH		320
#elif defined(CONFIG_ENABLE_QVGA)
#define LCD_HEIGHT		320
#define LCD_WIDTH		240
#else
#define LCD_HEIGHT		800
#define LCD_WIDTH		480
#endif

#if defined(CONFIG_ENABLE_HVGA) || defined(CONFIG_ENABLE_QVGA)
#define PANEL_HEIGHT 800
#define PANEL_WIDTH 480
#endif

#define VSYNC_PORCH 70

#ifdef CONFIG_ARGB8888
#define LCD_BITS_PER_PIXEL	32
#define TLINE   430
#define PIXEL_FORMAT PIXEL_FORMAT_RGB888
#else
#define LCD_BITS_PER_PIXEL	16
#define TLINE   350
#define PIXEL_FORMAT PIXEL_FORMAT_RGB666
#endif

#define TEAR_LINE 800

#define LCD_CMD(x) (x)
#define LCD_DATA(x) (x)

#define RESET_SEQ {WR_CMND_DATA, NT35582_SET_HOR_ADDR_S_MSB, (dev->col_start) >> 8},\
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_S_LSB, dev->col_start & 0xFF},\
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_E_MSB, (dev->col_end) >> 8},\
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_E_LSB, dev->col_end & 0xFF},\
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_S_MSB, (dev->row_start) >> 8},\
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_S_LSB, dev->row_start & 0xFF},\
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_E_MSB, (dev->row_end) >> 8},\
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_E_LSB, dev->row_end & 0xFF},\
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_X_MSB, (dev->col_start) >> 8},\
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_X_LSB, dev->col_start & 0xFF},\
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_Y_MSB, (dev->row_start) >> 8},\
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_Y_LSB, dev->row_start & 0xFF},\
		{WR_CMND, NT35582_WR_MEM_START, 0}

const char *LCD_panel_name = "Novatek NT35582 LCD";

int LCD_num_panels = 1;
LCD_Intf_t LCD_Intf = LCD_Z80;
#ifdef CONFIG_ARGB8888
LCD_Bus_t LCD_Bus = LCD_16BIT;
#else
LCD_Bus_t LCD_Bus = LCD_18BIT;
#endif

/* Fill these structures with the requirements mentioned in panel spec.
 * Unit: ns*/
CSL_LCDC_PAR_SPEED_t timingReg_ns = {250, 250, 12, 33, 33, 0};
CSL_LCDC_PAR_SPEED_t timingMem_ns = {250, 255, 12, 33, 33, 0};

bool hsm_supported = true;

/*High speed mode*/
CSL_LCDC_PAR_SPEED_t timingMem_hs_ns = {250, 255, 12, 14, 14, 0};

LCD_dev_info_t LCD_device[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= LCD_HEIGHT,
	 .width		= LCD_WIDTH,
	 .bits_per_pixel = LCD_BITS_PER_PIXEL,
	 .te_supported	= true,
	 }
};

Lcd_init_t power_on_seq[] = {
	{WR_CMND, NT35582_SLEEP_OUT, 0},
	{WR_CMND_DATA, NT35582_PWCTR1_0, 0x86},
	{WR_CMND_DATA, NT35582_PWCTR1_1, 0x00},
	{WR_CMND_DATA, NT35582_PWCTR1_2, 0x86},
	{WR_CMND_DATA, NT35582_PWCTR1_3, 0x00},
	{WR_CMND_DATA, NT35582_PWCTR2_0, 0x45},
	{WR_CMND_DATA, NT35582_PWCTR3_0, 0x21},
	{WR_CMND_DATA, NT35582_PWCTR3_2, 0x02},
	{WR_CMND_DATA, NT35582_SD_OP_SET_0, 0x30},
	{WR_CMND_DATA, NT35582_SD_OP_SET_2, 0x30},
	{WR_CMND_DATA, NT35582_VCOM, 0x8F},
	{WR_CMND_DATA, NT35582_SET_PIXEL_FORMAT, PIXEL_FORMAT},
	{WR_CMND_DATA, NT35582_GMACTRL_1_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_1_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_1_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_1_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_1_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_1_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_1_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_1_07, 0x3D},
	{WR_CMND_DATA, NT35582_GMACTRL_1_08, 0x22},
	{WR_CMND_DATA, NT35582_GMACTRL_1_09, 0x2A},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0A, 0x87},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_1_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_1_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_2_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_2_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_2_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_2_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_2_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_2_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_2_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_2_07, 0x3F},
	{WR_CMND_DATA, NT35582_GMACTRL_2_08, 0x20},
	{WR_CMND_DATA, NT35582_GMACTRL_2_09, 0x26},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0A, 0x83},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_2_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_2_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_3_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_3_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_3_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_3_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_3_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_3_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_3_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_3_07, 0x3D},
	{WR_CMND_DATA, NT35582_GMACTRL_3_08, 0x22},
	{WR_CMND_DATA, NT35582_GMACTRL_3_09, 0x2A},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0A, 0x87},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_3_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_3_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_4_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_4_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_4_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_4_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_4_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_4_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_4_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_4_07, 0x3F},
	{WR_CMND_DATA, NT35582_GMACTRL_4_08, 0x20},
	{WR_CMND_DATA, NT35582_GMACTRL_4_09, 0x26},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0A, 0x83},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_4_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_4_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_5_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_5_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_5_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_5_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_5_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_5_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_5_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_5_07, 0x3D},
	{WR_CMND_DATA, NT35582_GMACTRL_5_08, 0x22},
	{WR_CMND_DATA, NT35582_GMACTRL_5_09, 0x2A},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0A, 0x87},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_5_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_5_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_6_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_6_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_6_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_6_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_6_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_6_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_6_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_6_07, 0x3F},
	{WR_CMND_DATA, NT35582_GMACTRL_6_08, 0x20},
	{WR_CMND_DATA, NT35582_GMACTRL_6_09, 0x26},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0A, 0x83},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_6_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_6_11, 0x4D},
	{WR_CMND, NT35582_SET_TEAR_ON, 0},
	{WR_CMND_DATA, NT35582_SET_TEAR_LINE_MSB, TEAR_LINE >> 8},
	{WR_CMND_DATA, NT35582_SET_TEAR_LINE_LSB, TEAR_LINE & 0xFF},
	{WR_CMND_DATA, NT35582_SET_VPA, VSYNC_PORCH},
	{WR_CMND_DATA, NT35582_SET_TLINE_MSB, TLINE >> 8},
	{WR_CMND_DATA, NT35582_SET_TLINE_LSB, TLINE & 0xff},
	{SLEEP_MS, 0, 120},
	{WR_CMND, NT35582_DISPLAY_ON, 0},
	{CTRL_END, 0, 0}
};

Lcd_init_t power_off_seq[] = {
	{WR_CMND, NT35582_DISPLAY_OFF, 0},
	{WR_CMND, NT35582_SLEEP_IN, 0},
	{SLEEP_MS, 0, 60}, /* >50ms recommended*/
	{CTRL_END, 0, 0}
};

static inline bool window_hsm_compatible(LCD_DirtyRect_t rect){
    /* Min 32 pixels in a line and in multiple of 2 pixels */
    if (!hsm_supported || ((rect.right - rect.left + 1) < 32) || ((rect.right - rect.left + 1) & 1)) {
//        pr_info("false %d %d %d %d", rect.right, rect.left, rect.top, rect.bottom);
        return false;
    } else {
//        pr_info("true %d %d %d %d", rect.right, rect.left, rect.top, rect.bottom);
        return true;
    }
}

#endif /* __BCM_LCD_NT35582_H */
