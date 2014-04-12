/******************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement
governing use of this software, this software is licensed to you under the
terms of the GNU General Public License version 2, available at
http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software
in any way with any other Broadcom software provided under a license other than
the GPL, without Broadcom's express prior written consent.
******************************************************************************/

/**
*
*   @file   camdrv_ov5640.c
*
*   @brief  This file is the lower level driver API of OV5640(3M 2048*1536
*	Pixel) ISP/sensor.
*
*/
/**
 * @addtogroup CamDrvGroup
 * @{
 */

  /****************************************************************************/
  /*                          Include block                                   */
  /****************************************************************************/
#include <stdarg.h>
#include <stddef.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/gpio.h>
#if 0
#include <mach/reg_camera.h>
#include <mach/reg_lcd.h>
#endif
#include <mach/reg_clkpwr.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include <linux/broadcom/types.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/hal_camera.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/PowerManager.h>
#include <plat/dma.h>
#include <linux/dma-mapping.h>

#include "hal_cam_drv_ath.h"
//#include "hal_cam.h"
#include "hal_cam_drv.h"
//#include "cam_cntrl_2153.h"
#include "camdrv_dev.h"

#include <plat/csl/csl_cam.h>

#define CAMERA_IMAGE_INTERFACE  CSL_CAM_INTF_CPI
#define CAMERA_PHYS_INTF_PORT   CSL_CAM_PORT_AFE_1


#define HAL_CAM_RESET 56 /*gpio 56*/
#define FLASH_TRIGGER  30   /*gpio 30 for flash */
#define HAL_CAM_PD 19

#define HAL_CAM_VGA_STANDBY  9

#define HAL_CAM_VGA_RESET 10
#define g_Capture_PCLK_Frequency          36
#define FULL_PERIOD_PIXEL_NUMS          1940
#define FULL_EXPOSURE_LIMITATION        1236
#define PV_PERIOD_PIXEL_NUM              970
#define g_Capture_Max_Gain16          (8*16)
#define SENSOR_FORMAT_PREVIEW 0
#define SENSOR_FORMAT_STILL 1


int real_write=0;
CamFocusControlMode_t FOCUS_MODE;
//****************************************************************************
//                                                      OV5640 Paramter 
//****************************************************************************
UInt16 exposure_line_h = 0, exposure_line_l = 0;
UInt16 extra_exposure_line_h = 0, extra_exposure_line_l = 0;
UInt16 extra_exposure_lines = 0;
UInt16 dummy_pixels=0, dummy_lines=0;

UInt16 g_Capture_Gain16=0 ;
UInt16 g_Capture_Shutter=0;
UInt16 g_Capture_Extra_Lines=0;

UInt16  g_PV_Dummy_Pixels = 0;
UInt16  g_Capture_Dummy_Pixels = 0;
UInt16  g_Capture_Dummy_Lines = 0;
UInt16  g_PV_Gain16 = 0;
UInt16  g_PV_Shutter = 0;
UInt16  g_PV_Extra_Lines = 0;
CamZoom_t       g_Zoom_Level = CamZoom_1_0;

int preview_sysclk, preview_HTS, preview_VTS,  XVCLK = 2400;	// real clock/10000


/*****************************************************************************/
/* start of CAM configuration */
/*****************************************************************************/

/*****************************************************************************/
/*  Sensor Resolution Tables:												 */
/*****************************************************************************/
/* Resolutions & Sizes available for OV5640  ISP/Sensor (QXGA max) */
static CamResolutionConfigure_t sSensorResInfo_OV5640_st[] = {
	/* width    height  Preview_Capture_Index       Still_Capture_Index */
	{128, 96, CamImageSize_SQCIF, -1},	/* CamImageSize_SQCIF */
	{160, 120, CamImageSize_QQVGA, -1},	/* CamImageSize_QQVGA */
	{176, 144, CamImageSize_QCIF, CamImageSize_QCIF},	/* CamImageSize_QCIF */
	{240, 180, -1, -1},	/* CamImageSize_240x180 */
	{240, 320, -1, -1},	/* CamImageSize_R_QVGA */
	{320, 240, CamImageSize_QVGA, CamImageSize_QVGA},	/* CamImageSize_QVGA */
	{352, 288, CamImageSize_CIF, CamImageSize_CIF},	/* CamImageSize_CIF */
	{426, 320, -1, -1},	/* CamImageSize_426x320 */
	{640, 480, CamImageSize_VGA, CamImageSize_VGA},	/* CamImageSize_VGA */
	{800, 600, CamImageSize_SVGA, CamImageSize_SVGA},	/* CamImageSize_SVGA */
	{1024, 768, -1, CamImageSize_XGA},	/* CamImageSize_XGA */
	{1280, 960, -1, -1},	/* CamImageSize_4VGA */
	{1280, 1024, -1, CamImageSize_SXGA},	/* CamImageSize_SXGA */
	{1600, 1200, -1, CamImageSize_UXGA},	/* CamImageSize_UXGA */
	{2048, 1536, -1, CamImageSize_QXGA},	/* CamImageSize_QXGA */
	{2560, 2048, -1, -1},	/* CamImageSize_QSXGA */
	{144,	  176,   CamImageSize_R_QCIF,	CamImageSize_R_QCIF  } 	  //  CamImageSize_R_QCIF	 
};


static CamSensorIntfCntrl_st_t CamInitPowerOnSeq[] = {

	{PAUSE, 1, Nop_Cmd},

	{GPIO_CNTRL, HAL_CAM_VGA_STANDBY, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},/*for vga camera shutdown*/

	{GPIO_CNTRL, HAL_CAM_VGA_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd}, /*for vga camera shutdown*/

	/* -------Turn everything OFF   */
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetLow},
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
	//{GPIO_CNTRL, HAL_CAM_PD,       GPIO_SetHigh},

	{PAUSE, 10, Nop_Cmd},

	/* -------Enable Clock to Cameras @ Main clock speed*/
	{MCLK_CNTRL, CamDrv_24MHz, CLK_TurnOn},
	{PAUSE, 10, Nop_Cmd},


	{GPIO_CNTRL, HAL_CAM_PD,	  GPIO_SetLow },
		{PAUSE, 10, Nop_Cmd}, 

	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
		{PAUSE, 10, Nop_Cmd},

	/* -------Raise Reset to ISP*/
	
	

	


};



/*****************************************************************************/
/*  Power On/Off Tables for Main Sensor */
/*****************************************************************************/

static CamSensorIntfCntrl_st_t CamPowerOnSeq[] = {

	{PAUSE, 1, Nop_Cmd},

	{GPIO_CNTRL, HAL_CAM_VGA_STANDBY, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},/*for vga camera shutdown*/

	{GPIO_CNTRL, HAL_CAM_VGA_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd}, /*for vga camera shutdown*/

	/* -------Turn everything OFF   */
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
	{GPIO_CNTRL, HAL_CAM_PD,       GPIO_SetHigh},

		{PAUSE, 10, Nop_Cmd},
	/* -------Enable Clock to Cameras @ Main clock speed*/
	{MCLK_CNTRL, CamDrv_24MHz, CLK_TurnOn},
	{PAUSE, 10, Nop_Cmd},

	/* -------Raise Reset to ISP*/
	{GPIO_CNTRL, HAL_CAM_PD,      GPIO_SetLow },
	{PAUSE, 10, Nop_Cmd}


};

/*---------Sensor Power Off*/
static CamSensorIntfCntrl_st_t CamPowerOffSeq[] = {

	/* No Hardware Standby available. */

	//{PAUSE, 50, Nop_Cmd},
	
	/*--let primary camera enter soft standby mode , reset should be high-- */
	//{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	
	{GPIO_CNTRL, HAL_CAM_PD,       GPIO_SetHigh},

	{PAUSE, 10, Nop_Cmd},

	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
	
	
	/* -------Disable Clock to Cameras*/
	
	//{GPIO_CNTRL, HAL_CAM_PD,       GPIO_SetLow},
	//{PAUSE,      50,               Nop_Cmd}
};



/** Primary Sensor Configuration and Capabilities  */
static HAL_CAM_ConfigCaps_st_t CamPrimaryCfgCap_st = {
	// CAMDRV_DATA_MODE_S *video_mode
	{
		320,                           // unsigned short        max_width;                //Maximum width resolution
		240,                           // unsigned short        max_height;                //Maximum height resolution
		0,                             // UInt32                data_size;                //Minimum amount of data sent by the camera
		15,                            // UInt32                framerate_lo_absolute;  //Minimum possible framerate u24.8 format
		30,                            // UInt32                framerate_hi_absolute;  //Maximum possible framerate u24.8 format
		CAMDRV_TRANSFORM_NONE,         // CAMDRV_TRANSFORM_T    transform;            //Possible transformations in this mode / user requested transformations
		CAMDRV_IMAGE_YUV422,           // CAMDRV_IMAGE_TYPE_T    format;                //Image format of the frame.
		CAMDRV_IMAGE_YUV422_YCbYCr,    // CAMDRV_IMAGE_ORDER_T    image_order;        //Format pixel order in the frame.
		CAMDRV_DATA_SIZE_16BIT,        // CAMDRV_DATA_SIZE_T    image_data_size;    //Packing mode of the data.
		CAMDRV_DECODE_NONE,            // CAMDRV_DECODE_T        periph_dec;         //The decoding that the VideoCore transciever (eg CCP2) should perform on the data after reception.
		CAMDRV_ENCODE_NONE,            // CAMDRV_ENCODE_T        periph_enc;            //The encoding that the camera IF transciever (eg CCP2) should perform on the data before writing to memory.
		0,                             // int                    block_length;        //Block length for DPCM encoded data - specified by caller
		CAMDRV_DATA_SIZE_NONE,         // CAMDRV_DATA_SIZE_T    embedded_data_size; //The embedded data size from the frame.
		CAMDRV_MODE_VIDEO,             // CAMDRV_CAPTURE_MODE_T    flags;            //A bitfield of flags that can be set on the mode.
		30,                            // UInt32                framerate;            //Framerate achievable in this mode / user requested framerate u24.8 format
		0,                             // UInt8                mechanical_shutter;    //It is possible to use mechanical shutter in this mode (set by CDI as it depends on lens driver) / user requests this feature */
		3                              // UInt32                pre_frame;            //Frames to throw out for ViewFinder/Video capture
	},

	// CAMDRV_DATA_MODE_S *stills_mode
	{

		2560,                           // unsigned short max_width;   Maximum width resolution
		1920,                           // unsigned short max_height;  Maximum height resolution
		0,                              // UInt32                data_size;                //Minimum amount of data sent by the camera
		15,                             // UInt32                framerate_lo_absolute;  //Minimum possible framerate u24.8 format
		15,                             // UInt32                framerate_hi_absolute;  //Maximum possible framerate u24.8 format
		CAMDRV_TRANSFORM_NONE,          // CAMDRV_TRANSFORM_T    transform;            //Possible transformations in this mode / user requested transformations
		CAMDRV_IMAGE_JPEG,              // CAMDRV_IMAGE_TYPE_T    format;                //Image format of the frame.
		CAMDRV_IMAGE_YUV422_YCbYCr,     // CAMDRV_IMAGE_ORDER_T    image_order;        //Format pixel order in the frame.
		CAMDRV_DATA_SIZE_16BIT,         // CAMDRV_DATA_SIZE_T    image_data_size;    //Packing mode of the data.
		CAMDRV_DECODE_NONE,             // CAMDRV_DECODE_T        periph_dec;         //The decoding that the VideoCore transciever (eg CCP2) should perform on the data after reception.
		CAMDRV_ENCODE_NONE,             // PERIPHERAL_ENCODE_T    periph_enc;            //The encoding that the camera IF transciever (eg CCP2) should perform on the data before writing to memory.
		0,                              // int                    block_length;        //Block length for DPCM encoded data - specified by caller
		CAMDRV_DATA_SIZE_NONE,          // CAMDRV_DATA_SIZE_T    embedded_data_size; //The embedded data size from the frame.
		CAMDRV_MODE_VIDEO,              // CAMDRV_CAPTURE_MODE_T    flags;            //A bitfield of flags that can be set on the mode.
		15,                             // UInt32                framerate;            //Framerate achievable in this mode / user requested framerate u24.8 format
		0,                              // UInt8                mechanical_shutter;    //It is possible to use mechanical shutter in this mode (set by CDI as it depends on lens driver) / user requests this feature */
		8                               // UInt32                pre_frame;            //Frames to throw out for Stills capture
	},

	///< Focus Settings & Capabilities:  CAMDRV_FOCUSCONTROL_S *focus_control_st;
	{
#ifdef AUTOFOCUS_ENABLED
		CamFocusControlAuto,        	// CAMDRV_FOCUSCTRLMODE_T default_setting=CamFocusControlOff;
		CamFocusControlAuto,        	// CAMDRV_FOCUSCTRLMODE_T cur_setting;
		CamFocusControlOn |             // UInt32 settings;  Settings Allowed: CamFocusControlMode_t bit masked
			CamFocusControlOff |
			CamFocusControlAuto |
			CamFocusControlAutoLock |
			CamFocusControlCentroid |
			CamFocusControlQuickSearch |
			CamFocusControlInfinity |
			CamFocusControlMacro
#else
			CamFocusControlOff,             // CAMDRV_FOCUSCTRLMODE_T default_setting=CamFocusControlOff;
		CamFocusControlOff,             // CAMDRV_FOCUSCTRLMODE_T cur_setting;
		CamFocusControlOff              // UInt32 settings;  Settings Allowed: CamFocusControlMode_t bit masked
#endif
	},

	///< Digital Zoom Settings & Capabilities:  CAMDRV_DIGITALZOOMMODE_S *digital_zoom_st;
	{
		CamZoom_1_0,        ///< CAMDRV_ZOOM_T default_setting;  default=CamZoom_1_0:  Values allowed  CamZoom_t
		CamZoom_1_0,        ///< CAMDRV_ZOOM_T cur_setting;  CamZoom_t
		CamZoom_4_0,        ///< CAMDRV_ZOOM_T max_zoom;  Max Zoom Allowed (256/max_zoom = *zoom)
		TRUE                    ///< Boolean capable;  Sensor capable: TRUE/FALSE:
	},

	/*< Sensor ESD Settings & Capabilities:  CamESD_st_t esd_st; */
	{
		0x01,			/*< UInt8 ESDTimer;  Periodic timer to retrieve
					  the camera status (ms) */
		FALSE			/*< Boolean capable;  TRUE/FALSE: */
	},

	CAMERA_IMAGE_INTERFACE,                ///< UInt32 intf_mode;  Sensor Interfaces to Baseband
	CAMERA_PHYS_INTF_PORT,   
	/*< Sensor version string */
	"OV5640"
};

/*---------Sensor Primary Configuration CCIR656*/
static CamIntfConfig_CCIR656_st_t CamPrimaryCfg_CCIR656_st = {
	// Vsync, Hsync, Clock
 CSL_CAM_SYNC_EXTERNAL,                   ///< UInt32 sync_mode;                    (default)CAM_SYNC_EXTERNAL:        Sync External or Embedded
        CSL_CAM_SYNC_DEFINES_ACTIVE,             ///< UInt32 vsync_control;                (default)CAM_SYNC_DEFINES_ACTIVE:  VSYNCS determines active data
        CSL_CAM_SYNC_ACTIVE_LOW,                 ///< UInt32 vsync_polarity;               (default)ACTIVE_LOW/ACTIVE_HIGH:   Vsync active
        CSL_CAM_SYNC_DEFINES_ACTIVE,            ///< UInt32 hsync_control;                 (default)FALSE/TRUE:               HSYNCS determines active data
        CSL_CAM_SYNC_ACTIVE_HIGH,               ///< UInt32 hsync_polarity;                (default)ACTIVE_HIGH/ACTIVE_LOW:   Hsync active
        CSL_CAM_CLK_EDGE_POS,                   ///< UInt32 data_clock_sample;             (default)RISING_EDGE/FALLING_EDGE: Pixel Clock Sample edge
        CSL_CAM_PIXEL_16BIT,                    ///< UInt32 bus_width;                     (default)CAM_BITWIDTH_8:           Camera bus width
        0,                                      ///< UInt32 data_shift;                    (default)0:                        data shift (+) left shift  (-) right shift
        CSL_CAM_FIELD_H_V,                      ///< UInt32 field_mode;                    (default)CAM_FIELD_H_V:            field calculated
        CSL_CAM_INT_FRAME_END,                  ///< UInt32 data_intr_enable;              CAM_INTERRUPT_t:
        CSL_CAM_INT_FRAME_END 

};
// --------Primary Sensor Frame Rate Settings
static CamFrameRate_st_t PrimaryFrameRate_st =
{
	CamRate_30,                     ///< CamRates_t default_setting; 
	CamRate_30,                     ///< CamRates_t cur_setting; 
	CamRate_30                      ///< CamRates_t max_setting;
};

// ************************* REMEMBER TO CHANGE IT WHEN YOU CHANGE TO CCP ***************************
//CSI host connection
//---------Sensor Primary Configuration CCP CSI (sensor_config_ccp_csi)
CamIntfConfig_CCP_CSI_st_t  CamPrimaryCfg_CCP_CSI_st =
{
	CSL_CAM_INPUT_DUAL_LANE,                    ///< UInt32 input_mode;     CSL_CAM_INPUT_MODE_T:
	CSL_CAM_INPUT_MODE_DATA_CLOCK,              ///< UInt32 clk_mode;       CSL_CAM_CLOCK_MODE_T:
	CSL_CAM_ENC_NONE,                           ///< UInt32 encoder;        CSL_CAM_ENCODER_T
	FALSE,                                      ///< UInt32 enc_predictor;  CSL_CAM_PREDICTOR_MODE_t
	CSL_CAM_DEC_NONE,                           ///< UInt32 decoder;        CSL_CAM_DECODER_T
	FALSE,                                      ///< UInt32 dec_predictor;  CSL_CAM_PREDICTOR_MODE_t
	CSL_CAM_PORT_CHAN_0,                                 ///< UInt32 sub_channel;    CSL_CAM_CHAN_SEL_t
	TRUE,                                       ///< UInt32 term_sel;       BOOLEAN
	CSL_CAM_PIXEL_8BIT,                             ///< UInt32 bus_width;      CSL_CAM_BITWIDTH_t
	CSL_CAM_PIXEL_NONE,                         ///< UInt32 emb_data_type;  CSL_CAM_DATA_TYPE_T
	CSL_CAM_PORT_CHAN_0,                                 ///< UInt32 emb_data_channel; CSL_CAM_CHAN_SEL_t
	FALSE,                                      ///< UInt32 short_pkt;      BOOLEAN
	CSL_CAM_PIXEL_NONE,                         ///< UInt32 short_pkt_chan; CSL_CAM_CHAN_SEL_t
	CSL_CAM_INT_FRAME_END,                         ///< UInt32 data_intr_enable; CSL_CAM_INTERRUPT_t:
	CSL_CAM_INT_FRAME_END,                          ///< UInt32 pkt_intr_enable;  CSL_CAM_INTERRUPT_t:
};


/*---------Sensor Primary Configuration YCbCr Input*/
static CamIntfConfig_YCbCr_st_t CamPrimaryCfg_YCbCr_st = {
	/* YCbCr(YUV422) Input format = YCbCr=YUV= Y0 U0 Y1 V0  Y2 U2 Y3 V2 ....*/
	TRUE,			/*[00] Boolean yuv_full_range;
				  (default)FALSE: CROPPED YUV=16-240
TRUE: FULL RANGE YUV= 1-254  */
		SensorYCSeq_CbYCrY,	/*[01] CamSensorYCbCrSeq_t sensor_yc_seq;
						(default) SensorYCSeq_YCbYCr */
	//SensorYCSeq_YCbYCr,

	/* Currently Unused */
	FALSE,			/*[02] Boolean input_byte_swap;
				  Currently UNUSED!! (default)FALSE:  TRUE: */
	FALSE,			/*[03] Boolean input_word_swap;
				  Currently UNUSED!! (default)FALSE:  TRUE: */
	FALSE,			/*[04] Boolean output_byte_swap;
				  Currently UNUSED!! (default)FALSE:  TRUE: */
	FALSE,			/*[05] Boolean output_word_swap;
				  Currently UNUSED!! (default)FALSE:  TRUE: */

	/* Sensor olor Conversion Coefficients:
	   color conversion fractional coefficients are scaled by 2^8 */
	/* e.g. for R0 = 1.164, round(1.164 * 256) = 298 or 0x12a */
	CAM_COLOR_R1R0,		/*[06] UInt32 cc_red R1R0;
				  YUV422 to RGB565 color conversion red */
	CAM_COLOR_G1G0,		/*[07] UInt32 cc_green G1G0;
				  YUV422 to RGB565 color conversion green */
	CAM_COLOR_B1		/*[08] UInt32 cc_blue B1;
				  YUV422 to RGB565 color conversion blue */
};

/*---------Sensor Primary Configuration I2C */
static CamIntfConfig_I2C_st_t CamPrimaryCfg_I2C_st = {
	0x00,			/*I2C_SPD_430K, [00] UInt32 i2c_clock_speed;
				  max clock speed for I2C */
	I2C_CAM_DEVICE_ID,	/*[01] UInt32 i2c_device_id; I2C device ID */
	0x00,			/*I2C_BUS2_ID, [02] I2C_BUS_ID_t i2c_access_mode;
				  I2C baseband port */
	0x00,			/*I2CSUBADDR_16BIT,[03] I2CSUBADDR_t i2c_sub_adr_op;
				  I2C sub-address size */
	0xFFFF,			/*[04] UInt32 i2c_page_reg;
				  I2C page register addr (if applicable)
				 **UNUSED by this driver** */
	I2C_CAM_MAX_PAGE	/*[05] UInt32 i2c_max_page; I2C max page
				  (if not used by camera drivers, set = 0xFFFF)
				 **UNUSED by this driver** */
};

/*---------Sensor Primary Configuration IOCR */
static CamIntfConfig_IOCR_st_t CamPrimaryCfg_IOCR_st = {
	FALSE,			/*[00] Boolean cam_pads_data_pd;
				  (default)FALSE: IOCR2 D0-D7 pull-down disabled
TRUE: IOCR2 D0-D7 pull-down enabled */
	FALSE,			/*[01] Boolean cam_pads_data_pu;
				  (default)FALSE: IOCR2 D0-D7 pull-up disabled
TRUE: IOCR2 D0-D7 pull-up enabled */
	FALSE,			/*[02] Boolean cam_pads_vshs_pd;
				  (default)FALSE: IOCR2 Vsync/Hsync pull-down disabled
TRUE: IOCR2 Vsync/Hsync pull-down enabled */
	FALSE,			/*[03] Boolean cam_pads_vshs_pu;
				  (default)FALSE: IOCR2 Vsync/Hsync pull-up disabled
TRUE: IOCR2 Vsync/Hsync pull-up enabled */
	FALSE,			/*[04] Boolean cam_pads_clk_pd;
				  (default)FALSE: IOCR2 CLK pull-down disabled
TRUE: IOCR2 CLK pull-down enabled */
	FALSE,			/*[05] Boolean cam_pads_clk_pu;
				  (default)FALSE: IOCR2 CLK pull-up disabled
TRUE: IOCR2 CLK pull-up enabled */

	7 << 12,		/*[06] UInt32 i2c_pwrup_drive_strength;
				  (default)IOCR4_CAM_DR_12mA:
				  IOCR drive strength */
	0x00,			/*[07] UInt32 i2c_pwrdn_drive_strength;
				  (default)0x00:    I2C2 disabled */
	0x00,			/*[08] UInt32 i2c_slew; (default) 0x00: 42ns */

	7 << 12,		/*[09] UInt32 cam_pads_pwrup_drive_strength;
				  (default)IOCR4_CAM_DR_12mA:  IOCR drive strength */
	1 << 12			/*[10] UInt32 cam_pads_pwrdn_drive_strength;
				  (default)IOCR4_CAM_DR_2mA:   IOCR drive strength */
};

/* XXXXXXXXXXXXXXXXXXXXXXXXXXX IMPORTANT XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */
/* TO DO: MURALI */
/* HAVE TO PROGRAM THIS IN THE ISP. */
/*---------Sensor Primary Configuration JPEG */

static CamIntfConfig_Jpeg_st_t CamPrimaryCfg_Jpeg_st = {
	1024,			/*< UInt32 jpeg_packet_size_bytes;     Bytes/Hsync */
	1360,			/*< UInt32 jpeg_max_packets;           Max Hsyncs/Vsync = (3.2Mpixels/4) / 512 */
	CamJpeg_FixedPkt_VarLine,	/*< CamJpegPacketFormat_t pkt_format;  Jpeg Packet Format */
};

/* XXXXXXXXXXXXXXXXXXXXXXXXXXX IMPORTANT XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */
/* TO DO: MURALI */
/* WILL NEED TO MODIFY THIS. */
/*---------Sensor Primary Configuration Stills n Thumbs */
static CamIntfConfig_PktMarkerInfo_st_t CamPrimaryCfg_PktMarkerInfo_st = {
	2,			/*< UInt8       marker_bytes; # of bytes for marker,
				  (= 0 if not used) */
	0,			/*< UInt8       pad_bytes; # of bytes for padding,
				  (= 0 if not used) */
	TRUE,			/*< Boolean     TN_marker_used; Thumbnail marker used */
	0xFFBE,			/*< UInt32      SOTN_marker; Start of Thumbnail marker,
				  (= 0 if not used) */
	0xFFBF,			/*< UInt32      EOTN_marker; End of Thumbnail marker,
				  (= 0 if not used) */
	TRUE,			/*< Boolean     SI_marker_used; Status Info marker used */
	0xFFBC,			/*< UInt32      SOSI_marker; Start of Status Info marker,
				  (= 0 if not used) */
	0xFFBD,			/*< UInt32      EOSI_marker; End of Status Info marker,
				  (= 0 if not used) */
	FALSE,			/*< Boolean     Padding_used; Status Padding bytes used */
	0x0000,			/*< UInt32      SOPAD_marker; Start of Padding marker,
				  (= 0 if not used) */
	0x0000,			/*< UInt32      EOPAD_marker; End of Padding marker,
				  (= 0 if not used) */
	0x0000			/*< UInt32      PAD_marker; Padding Marker,
				  (= 0 if not used) */
};

/*---------Sensor Primary Configuration Stills n Thumbs */
static CamIntfConfig_InterLeaveMode_st_t CamPrimaryCfg_InterLeaveStills_st = {
	CamInterLeave_SingleFrame,	/*< CamInterLeaveMode_t mode;
					  Interleave Mode */
	CamInterLeave_PreviewLast,	/*< CamInterLeaveSequence_t sequence;
					  InterLeaving Sequence */
	CamInterLeave_PktFormat	/*< CamInterLeaveOutputFormat_t format;
				  InterLeaving Output Format */
};

/*---------Sensor Primary Configuration */
static CamIntfConfig_st_t CamSensorCfg_st = {
	&CamPrimaryCfgCap_st,	/* *sensor_config_caps; */
	&CamPrimaryCfg_CCIR656_st,	/* *sensor_config_ccir656; */
	&CamPrimaryCfg_CCP_CSI_st,
	&CamPrimaryCfg_YCbCr_st,	/* *sensor_config_ycbcr; */
	NULL,	                     /* *sensor_config_i2c; */
	&CamPrimaryCfg_IOCR_st,	/* *sensor_config_iocr; */
	&CamPrimaryCfg_Jpeg_st,	/* *sensor_config_jpeg; */
	NULL,			/* *sensor_config_interleave_video; */
	&CamPrimaryCfg_InterLeaveStills_st,	/**sensor_config_interleave_stills; */
	&CamPrimaryCfg_PktMarkerInfo_st	/* *sensor_config_pkt_marker_info; */
};

FlashLedState_t flash_switch;

/* I2C transaction result */
static HAL_CAM_Result_en_t sCamI2cStatus = HAL_CAM_SUCCESS;
void zoomfunction(unsigned char, unsigned int ,unsigned int ,unsigned int, unsigned int);
static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution,
		CamDataFmt_t image_format);
static HAL_CAM_Result_en_t Init_OV5640(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt8 ov5640_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t ov5640_write(unsigned int sub_addr, UInt8 data);
static HAL_CAM_Result_en_t CAMDRV_SetFrameRate_Pri(CamRates_t fps,
		CamSensorSelect_t sensor);
static HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode_Pri(CamImageSize_t image_resolution,
		CamDataFmt_t image_format,
		CamSensorSelect_t sensor);
static HAL_CAM_Result_en_t CAMDRV_SetZoom_Pri(CamZoom_t step,CamSensorSelect_t sensor);
static void control_flash(int flash_on);
static void  write_OV5640_gain(UInt16 gain);
static void     OV5640_set_dummy(UInt16 pixels, UInt16 lines);
static UInt16  read_OV5640_gain(void);

static int OV5640_get_sysclk()
{
	 // calculate sysclk
	 int temp1, temp2;
	 int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv, sysclk;

	 int sclk_rdiv_map[] = {
		 1, 2, 4, 8};

	 temp1 = ov5640_read(0x3034);
	 temp2 = temp1 & 0x0f;
	 if (temp2 == 8 || temp2 == 10) {
		 Bit_div2x = temp2 / 2;
	 }

	 temp1 = ov5640_read(0x3035);
	 SysDiv = temp1>>4;
	 if(SysDiv == 0) {
		 SysDiv = 16;
	 }

	 temp1 = ov5640_read(0x3036);
	 Multiplier = temp1;

	 temp1 = ov5640_read(0x3037);
	 PreDiv = temp1 & 0x0f;
	 Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	 temp1 = ov5640_read(0x3108);
	 temp2 = temp1 & 0x03;
	 sclk_rdiv = sclk_rdiv_map[temp2]; 

	 VCO = XVCLK * Multiplier / PreDiv;

	 sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	 return sysclk;
}

static int OV5640_get_HTS()
{
	 // read HTS from register settings
	 int HTS;

	 HTS = ov5640_read(0x380c);
	 HTS = (HTS<<8) + ov5640_read(0x380d);

	 return HTS;
}

static int OV5640_get_VTS()
{
	 // read VTS from register settings
	 int VTS;

	 VTS = ov5640_read(0x380e);
	 VTS = (VTS<<8) + ov5640_read(0x380f);

	 return VTS;
}

static int OV5640_set_VTS(int VTS)
{
	 // write VTS to registers
	 int temp;

	 temp = VTS & 0xff;
	 ov5640_write(0x380f, temp);

	 temp = VTS>>8;
	 ov5640_write(0x380e, temp);

	 return 0;
}

static int OV5640_get_shutter()
{
	 // read shutter, in number of line period
	 int shutter;

	 shutter = (ov5640_read(0x03500) & 0x0f);
	 shutter = (shutter<<8) + ov5640_read(0x3501);
	 shutter = (shutter<<4) + (ov5640_read(0x3502)>>4);

	 return shutter;
}

static int OV5640_set_shutter(int shutter)
{
	 // write shutter, in number of line period
	 int temp;
	 
	 shutter = shutter & 0xffff;

	 temp = shutter & 0x0f;
	 temp = temp<<4;
	 ov5640_write(0x3502, temp);

	 temp = shutter & 0xfff;
	 temp = temp>>4;
	 ov5640_write(0x3501, temp);

	 temp = shutter>>12;
	 ov5640_write(0x3500, temp);

	 return 0;
}

static int OV5640_get_gain16()
{
	 // read gain, 16 = 1x
	 int gain16;

	 gain16 = ov5640_read(0x350a) & 0x03;
	 gain16 = (gain16<<8) + ov5640_read(0x350b);

	 return gain16;
}

static int OV5640_set_gain16(int gain16)
{
	 // write gain, 16 = 1x
	 int temp;
	 gain16 = gain16 & 0x3ff;

	 temp = gain16 & 0xff;
	 ov5640_write(0x350b, temp);

	 temp = gain16>>8;
	 ov5640_write(0x350a, temp);

	 return 0;
}

static int OV5640_get_light_frequency()
{
	 // get banding filter value
	 int temp, temp1, light_frequency;

	 temp = ov5640_read(0x3c01);

	 if (temp & 0x80) {
		 // manual
		 temp1 = ov5640_read(0x3c00);
		 if (temp1 & 0x04) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
			 light_frequency = 60;
		 }
	 }
	 else {
		 // auto
		 temp1 = ov5640_read(0x3c0c);
		 if (temp1 & 0x01) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
		 }
	 }
	 return light_frequency;
}

static HAL_CAM_Result_en_t OV5640_capture_setting(CamImageSize_t stills_resolution)
{
		HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	
		
		UInt8 read_out;
		
		if(flash_switch==FlashLight_Auto)
		{
			read_out=ov5640_read(0x56a1);
	
			if(read_out < 0x10)
				control_flash(1);
		}
		else if(flash_switch==Flash_On)
			control_flash(1);
	
		
	
	
	// JPEG with variable width
	
	//YUV capture 
				 
		ov5640_write(0x3035, 0x31);//ov5640_write(0x3035, 0x21);  /* control the framerate of still picture */ // ov5640_write(0x3035, 0x71);
		ov5640_write(0x3036, 0x69);
		ov5640_write(0x3c07, 0x07);
		ov5640_write(0x3820, 0x40);
		ov5640_write(0x3821, 0x06);
		ov5640_write(0x3814, 0x11);
		ov5640_write(0x3815, 0x11);
		ov5640_write(0x3803, 0x00);
		ov5640_write(0x3807, 0x9f);
		ov5640_write(0x3808, 0x0a);
		ov5640_write(0x3809, 0x20);
		ov5640_write(0x380a, 0x07);
		ov5640_write(0x380b, 0x98);
		ov5640_write(0x380c, 0x0b);
		ov5640_write(0x380d, 0x1c);
		ov5640_write(0x380e, 0x07);
		ov5640_write(0x380f, 0xb0);
		ov5640_write(0x3813, 0x04);
		ov5640_write(0x3618, 0x04);
		ov5640_write(0x3612, 0x2b);
		ov5640_write(0x3708, 0x21);
		ov5640_write(0x3709, 0x12);
		ov5640_write(0x370c, 0x00);
		ov5640_write(0x3a02, 0x07);
		ov5640_write(0x3a03, 0xb0);
		ov5640_write(0x3a0e, 0x06);
		ov5640_write(0x3a0d, 0x08);
		ov5640_write(0x3a14, 0x07);
		ov5640_write(0x3a15, 0xb0);
		ov5640_write(0x4004, 0x06); 				 
		ov5640_write(0x4713, 0x02);
		ov5640_write(0x4407, 0x0c);
		ov5640_write(0x460b, 0x37);
		ov5640_write(0x460c, 0x20);
		ov5640_write(0x3824, 0x01); 			 
		ov5640_write(0x5001, 0x83);
		//ov5640_write(0x3035, 0x21); 
		//ov5640_write(0x3820, 0x47);
		//ov5640_write(0x3821, 0x01);	
	
	
	
	
		
	
	
		switch (stills_resolution) {
			case CamImageSize_QCIF:
	
				break;
	
			case CamImageSize_QVGA:
				/*ov */
				ov5640_write(0x3808 ,0x1 ); /*w high bit*/
					ov5640_write(0x3809 ,0x40);/*w low bit */
					ov5640_write(0x380a ,0x0 ); /*h high bits*/
					ov5640_write(0x380b ,0xf0); /*height low bits*/
			ov5640_write(0x5001 ,0xa3); 	
			
				
				break;
	
			case CamImageSize_CIF:
				break;
	
			case CamImageSize_VGA:
				ov5640_write(0x3808 ,0x2 );  
					ov5640_write(0x3809 ,0x80);
					ov5640_write(0x380a ,0x01 );
					ov5640_write(0x380b ,0xe0);
			ov5640_write(0x5001 ,0xa3); 	
	
				break;
	
			case CamImageSize_SVGA:
				break;
	
			case CamImageSize_XGA:
	
				ov5640_write(0x3808 ,0x04 );  
					ov5640_write(0x3809 ,0x00);
					ov5640_write(0x380a ,0x03 );
					ov5640_write(0x380b ,0x00);
			ov5640_write(0x5001 ,0xa3); 
	
				break;
	
			case CamImageSize_SXGA:
				ov5640_write(0x3808 ,0x05 );  
					ov5640_write(0x3809 ,0x00);
					ov5640_write(0x380a ,0x04 );
					ov5640_write(0x380b ,0x00);
			ov5640_write(0x5001 ,0xa3); 
	
				break;
	
			case CamImageSize_UXGA:
				ov5640_write(0x3808 ,0x06 );  
					ov5640_write(0x3809 ,0x40);
					ov5640_write(0x380a ,0x04 );
					ov5640_write(0x380b ,0xb0);
			ov5640_write(0x5001 ,0xa3); 
	
	
				break;
	
			case CamImageSize_QXGA:
				ov5640_write(0x3808 ,0x08 );  
					ov5640_write(0x3809 ,0x00);
					ov5640_write(0x380a ,0x06 );
					ov5640_write(0x380b ,0x00);
			ov5640_write(0x5001 ,0xa3); 
	
				break;
			case CamImageSize_QSXGA:
				ov5640_write(0x3808 ,0x0a );  
					ov5640_write(0x3809 ,0x00);
					ov5640_write(0x380a ,0x07 );
					ov5640_write(0x380b ,0x80);
			ov5640_write(0x5001 ,0x83);
	
				/*
			ov5640_write(0x3808 ,0x0a );   //2592X1944
					ov5640_write(0x3809 ,0x20);
					ov5640_write(0x380a ,0x07 );
					ov5640_write(0x380b ,0x98);
			ov5640_write(0x5001 ,0x83);*/
			
				break;
	
			default:
				break;
		}
	
	
	
		
		if (sCamI2cStatus != HAL_CAM_SUCCESS) {
			printk
				("CAMDRV_CfgStillnThumbCapture():Error sending capture mode \r\n");
			result = sCamI2cStatus;
		}
	
	
		return result;
	}


/*****************************************************************************
 *
 * Function Name:   CAMDRV_GetIntfConfigTbl
 *
 * Description: Return Camera Sensor Interface Configuration
 *
 * Notes:
 *
 *****************************************************************************/
static CamIntfConfig_st_t *CAMDRV_GetIntfConfig_Pri(CamSensorSelect_t nSensor)
{

	/* ---------Default to no configuration Table */
	CamIntfConfig_st_t *config_tbl = NULL;

	switch (nSensor) {
		case CamSensorPrimary:	/* Primary Sensor Configuration */
		default:
			CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;
			break;
		case CamSensorSecondary:	/* Secondary Sensor Configuration */
			CamSensorCfg_st.sensor_config_caps = NULL;
			break;
	}
	config_tbl = &CamSensorCfg_st;

#ifdef ALLOC_TN_BUFFER
	if (thumbnail_data == NULL) {
		/*tn_buffer = (u8 *)dma_alloc_coherent( NULL, TN_BUFFER_SIZE,
		  &physPtr, GFP_KERNEL); */
		thumbnail_data = (u8 *) kmalloc(TN_BUFFER_SIZE, GFP_KERNEL);
		if (thumbnail_data == NULL) {
			pr_info
				("Error in allocating %d bytes for OV5640 sensor\n",
				 TN_BUFFER_SIZE);
			return NULL;
		}
		memset(thumbnail_data, 0, TN_BUFFER_SIZE);
	}
#endif
	return config_tbl;
}

/*****************************************************************************
 *
 * Function Name:   CAMDRV_GetInitPwrUpSeq
 *
 * Description: Return Camera Sensor Init Power Up sequence
 *
 * Notes:
 *
 *****************************************************************************/
static CamSensorIntfCntrl_st_t *CAMDRV_GetIntfSeqSel_Pri(CamSensorSelect_t nSensor,
		CamSensorSeqSel_t nSeqSel,
		UInt32 *pLength)
{

	/* ---------Default to no Sequence  */
	CamSensorIntfCntrl_st_t *power_seq = NULL;
	*pLength = 0;

	switch (nSeqSel) {
		case SensorInitPwrUp:	/* Camera Init Power Up (Unused) */

			if (nSensor == CamSensorPrimary) {
				printk("SensorInitPwrUp Sequence\r\n");
				*pLength = sizeof(CamInitPowerOnSeq);
				power_seq = CamInitPowerOnSeq;
			}
			break;
		case SensorPwrUp:
			if ((nSensor == CamSensorPrimary)
					|| (nSensor == CamSensorSecondary)) {
				printk("SensorPwrUp Sequence\r\n");
				*pLength = sizeof(CamPowerOnSeq);
				power_seq = CamPowerOnSeq;
			}
			break;

		case SensorInitPwrDn:	/* Camera Init Power Down (Unused) */
		case SensorPwrDn:	/* Both off */
			if ((nSensor == CamSensorPrimary)
					|| (nSensor == CamSensorSecondary)) {
				printk("SensorPwrDn Sequence\r\n");
				*pLength = sizeof(CamPowerOffSeq);
				power_seq = CamPowerOffSeq;
			}
			break;

		case SensorFlashEnable:	/* Flash Enable */
			break;

		case SensorFlashDisable:	/* Flash Disable */
			break;

		default:
			break;
	}
	return power_seq;

}

/***************************************************************************
 * *
 *       CAMDRV_Supp_Init performs additional device specific initialization
 *
 *   @return  HAL_CAM_Result_en_t
 *
 *       Notes:
 */
static HAL_CAM_Result_en_t CAMDRV_Supp_Init_Pri(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t ret_val = HAL_CAM_SUCCESS;

	return ret_val;
}				/* CAMDRV_Supp_Init() */

/****************************************************************************
 *
 * Function Name:   HAL_CAM_Result_en_t CAMDRV_Wakeup(CamSensorSelect_t sensor)
 *
 * Description: This function wakesup camera via I2C command.  Assumes camera
 *              is enabled.
 *
 * Notes:
 *
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_Wakeup_Pri(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	result = Init_OV5640(sensor);
	printk("Init_OV5640 result =%d\r\n", result);
	return result;
}
static HAL_CAM_Result_en_t ov5640_write(unsigned int sub_addr, UInt8 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;

	result |= CAM_WriteI2c(sub_addr, 1, &data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("ov5640_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
	return result;
}

static UInt8 ov5640_read(unsigned int sub_addr)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 data;

	result |= CAM_ReadI2c(sub_addr, 1,&data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("ov5640_read(): ERROR: %d\r\n", result);
	}


	return data;
}


	static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format)
{
	UInt32 format = 0;
	UInt8 temp_AE_reg, temp_AWB_reg;
	UInt16 iDummyPixels = 0, iDummyLines = 0;

	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;


	printk(KERN_ERR"SensorSetPreviewMode image_resolution 0x%x image_format %d  ",image_resolution,image_format  );

// YUV 30 fps 
	

	    ov5640_write(0x3503, 0x00);
		ov5640_write(0x3035, 0x11);
		ov5640_write(0x3036, 0x46);
		ov5640_write(0x3c07, 0x08);
		ov5640_write(0x3820, 0x41);
		ov5640_write(0x3821, 0x07);
		ov5640_write(0x3814, 0x31);
		ov5640_write(0x3815, 0x31);
		ov5640_write(0x3803, 0x04);
		ov5640_write(0x3807, 0x9b);
		ov5640_write(0x3808, 0x02);
		ov5640_write(0x3809, 0x80);
		ov5640_write(0x380a, 0x01);
		ov5640_write(0x380b, 0xe0);
		ov5640_write(0x380c, 0x07);
		ov5640_write(0x380d, 0x68);
		ov5640_write(0x380e, 0x03);
		ov5640_write(0x380f, 0xd8);
		ov5640_write(0x3813, 0x06);
		ov5640_write(0x3618, 0x00);
		ov5640_write(0x3612, 0x29);
		ov5640_write(0x3708, 0x62);
		ov5640_write(0x3709, 0x52);
		ov5640_write(0x370c, 0x03);
		ov5640_write(0x3a02, 0x03);
		ov5640_write(0x3a03, 0xd8);
		ov5640_write(0x3a0e, 0x03);
		ov5640_write(0x3a0d, 0x04);
		ov5640_write(0x3a14, 0x03);
		ov5640_write(0x3a15, 0xd8);
		ov5640_write(0x4004, 0x02); 			 
		ov5640_write(0x4713, 0x03);
		ov5640_write(0x4407, 0x04);
		ov5640_write(0x460b, 0x35);
		ov5640_write(0x460c, 0x20); //0x22
		ov5640_write(0x3824, 0x02); 			 
		ov5640_write(0x5001, 0xa3);
		//ov5640_write(0x3035, 0x21);
		//ov5640_write(0x3820, 0x47);
		//ov5640_write(0x3821, 0x01); 

		/*add more resolution here */


	switch (image_resolution) {
		
		case CamImageSize_QVGA:
			/*ov */
			ov5640_write(0x3808 ,0x1 ); /*w high bit*/
				ov5640_write(0x3809 ,0x40);/*w low bit */
				ov5640_write(0x380a ,0x0 ); /*h high bits*/
				ov5640_write(0x380b ,0xf0); /*height low bits*/
		ov5640_write(0x5001 ,0xa3);	/*aspact: 4:3 is ths same   */	
		
			
			break;

		case CamImageSize_VGA:
			ov5640_write(0x3808 ,0x2 );  
				ov5640_write(0x3809 ,0x80);
				ov5640_write(0x380a ,0x01 );
				ov5640_write(0x380b ,0xe0);
		ov5640_write(0x5001 ,0xa3);		

			break;
			
		case CamImageSize_CIF:
			ov5640_write(0x3808 ,0x1 );  
				ov5640_write(0x3809 ,0x60);
				ov5640_write(0x380a ,0x01 );
				ov5640_write(0x380b ,0x20);
		ov5640_write(0x5001 ,0xa3);		

			break;

			case CamImageSize_QCIF:
			ov5640_write(0x3808 ,0x0 );  
				ov5640_write(0x3809 ,0xb0);
				ov5640_write(0x380a ,0x00 );
				ov5640_write(0x380b ,0x90);
		ov5640_write(0x5001 ,0xa3);		

			break;  //added later 
			

	}

	

if(CamFocusControlAuto ==FOCUS_MODE)  /*release AF,to infinite status*/
{
			ov5640_write(0x3023, 0x01);
			ov5640_write(0x3022, 0x08);
}


		// read preview PCLK
		 preview_sysclk = OV5640_get_sysclk();

		printk(KERN_ERR"youle_ov: preview_sysclk =%d \n",preview_sysclk);
		 // read preview HTS
		 preview_HTS = OV5640_get_HTS();
	
		 // read preview VTS
		 preview_VTS = OV5640_get_VTS();
		 


	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("SensorSetPreviewMode(): Error[%d] sending preview mode  r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}
	printk("SensorSetPreviewMode(): Resolution:0x%x, Format:0x%x r\n",
			image_resolution, image_format);

	printk("**************Venky end of the Preview %d \n",result);

	return result;

}
/** @} */

/****************************************************************************
 *
 * Function Name:   HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode
 (CamImageSize_t image_resolution, CamDataFmt_t image_format)
 *
 * Description: This function configures Video Capture
 (Same as ViewFinder Mode)
 *
 * Notes:
 *
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode_Pri(CamImageSize_t image_resolution,
		CamDataFmt_t image_format,
		CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	/* --------Set up Camera Isp for Output Resolution & Format */
	result = SensorSetPreviewMode(image_resolution, image_format);
	return result;
}

/****************************************************************************
 *
 * Function Name:   HAL_CAM_Result_en_t CAMDRV_SetExposure_Pri(CamRates_t fps)
 *
 * Description: This function sets the Exposure compensation
 *
 * Notes:   
 *
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetExposure_Pri(int value)
{ 
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	printk("CAMDRV_SetFrameRate_Pri %d \r\n",value );

	if(value==0)
	{
		ov5640_write(0x3a0f ,0x38);
		ov5640_write(0x3a10 ,0x30);
		ov5640_write(0x3a11 ,0x61);
		ov5640_write(0x3a1b ,0x38);
		ov5640_write(0x3a1e ,0x30);
		ov5640_write(0x3a1f ,0x10);

	}
	else if(value>0)
	{

		ov5640_write(0x3a0f ,0x50);
		ov5640_write(0x3a10 ,0x48); 
		ov5640_write(0x3a11 ,0x90); 
		ov5640_write(0x3a1b ,0x50); 
		ov5640_write(0x3a1e ,0x48); 
		ov5640_write(0x3a1f ,0x20); 

	}
	else
	{
		ov5640_write(0x3a0f ,0x20);
		ov5640_write(0x3a10 ,0x18);
		ov5640_write(0x3a11 ,0x41);
		ov5640_write(0x3a1b ,0x20);
		ov5640_write(0x3a1e ,0x18);
		ov5640_write(0x3a1f ,0x10);

	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetExposure_Pri(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}


/****************************************************************************
 *
 * Function Name:   HAL_CAM_Result_en_t CAMDRV_SetFrameRate(CamRates_t fps)
 *
 * Description: This function sets the frame rate of the Camera Sensor
 *
 * Notes:    15 or 30 fps are supported.
 *
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetFrameRate_Pri(CamRates_t fps,
		CamSensorSelect_t sensor)
{ 
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	
	printk("youle:    CAMDRV_SetFrameRate %d \r\n",fps );
	//fps=CamRate_10;
	if (fps > CamRate_30 || fps<CamRate_5)
	{
		result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
	}
	else
	{
		if (fps == CamRate_30)
		{
			ov5640_write(0x3035, 0x11);
   			ov5640_write(0x3036, 0x46);
			ov5640_write(0x3a08, 0x1);
   			ov5640_write(0x3a09, 0x27);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0xf6);
			ov5640_write(0x3a0e, 0x03);
   			ov5640_write(0x3a0d, 0x04);

		}
		else
		{
			if(fps==CamRate_25)
			{
			ov5640_write(0x3035, 0x11);
   			ov5640_write(0x3036, 0x3b);
			ov5640_write(0x3a08, 0x0);
   			ov5640_write(0x3a09, 0xf9);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0xcf);
			ov5640_write(0x3a0e, 0x03);
   			ov5640_write(0x3a0d, 0x04);
				
				// To do Venky 	
			}
			else if(fps==CamRate_20)
			{
			ov5640_write(0x3035, 0x11);
   			ov5640_write(0x3036, 0x2f);
			ov5640_write(0x3a08, 0x0);
   			ov5640_write(0x3a09, 0xc6);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0xa5);
			ov5640_write(0x3a0e, 0x04);
   			ov5640_write(0x3a0d, 0x05);
			
				// To do Venky 
			}
			else if(fps==CamRate_15)
			{printk(KERN_ERR"youle:    --------------15 ---------\r\n" );
			ov5640_write(0x3035, 0x21);
   			ov5640_write(0x3036, 0x46);
			ov5640_write(0x3a08, 0x0);
   			ov5640_write(0x3a09, 0x94);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0x7b);
			ov5640_write(0x3a0e, 0x06);
   			ov5640_write(0x3a0d, 0x08);
		
				//To do Venky
			}
			else if(fps==CamRate_10)
			{printk(KERN_ERR"youle:    --------------10 ---------\r\n" );
			ov5640_write(0x3035, 0x21);
   			ov5640_write(0x3036, 0x2f);
			ov5640_write(0x3a08, 0x0);
   			ov5640_write(0x3a09, 0x63);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0x53);
			ov5640_write(0x3a0e, 0x09);
   			ov5640_write(0x3a0d, 0x0a);
		
				//To do Venky
			}
			else if(fps==CamRate_7_5)
			{

			printk(KERN_ERR"youle:    --------------5555555---------\r\n" );
			#if 0
			ov5640_write(0x3035, 0x41);
   			ov5640_write(0x3036, 0x2f);
			ov5640_write(0x3a08, 0x0);
   			ov5640_write(0x3a09, 0x63);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0x53);
			ov5640_write(0x3a0e, 0x09);
   			ov5640_write(0x3a0d, 0x0a);
			#else
			ov5640_write(0x3035, 0x41);
   			ov5640_write(0x3036, 0x46);
			ov5640_write(0x3a08, 0x0);
   			ov5640_write(0x3a09, 0x94);
			ov5640_write(0x3a0a, 0x0);
   			ov5640_write(0x3a0b, 0x7b);
			ov5640_write(0x3a0e, 0x06);
   			ov5640_write(0x3a0d, 0x08);

			#endif

				//To do Venky
			}
			else 
			{
				printk("CAMDRV_SetFrameRate(): Error HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
			}	
		}       // else (if (ImageSettingsConfig_st.sensor_framerate->cur_setting == CamRate_Auto))
	}       // else (if (fps <= CamRate_Auto))

	//	 if (sCamI2cStatus != HAL_CAM_SUCCESS) {
	//		 printk("CAMDRV_SetFrameRate(): Error[%d] \r\n",
	//			  sCamI2cStatus);
	//		 result = sCamI2cStatus;
	//	 }

	return result;

}

/****************************************************************************
  /
  / Function Name:   HAL_CAM_Result_en_t
  CAMDRV_EnableVideoCapture(CamSensorSelect_t sensor)
  /
  / Description: This function starts camera video capture mode
  /
  / Notes: camera_enable for preview after set preview mode  ;we can ignore this function ,for it has been done in  set preview mode
  /
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_EnableVideoCapture_Pri(CamSensorSelect_t sensor)
{return HAL_CAM_SUCCESS;
	/*[Enable stream] */
}

/****************************************************************************
  /
  / Function Name:   void CAMDRV_SetCamSleep(CamSensorSelect_t sensor )
  /
  / Description: This function puts ISP in sleep mode & shuts down power.
  /
  / Notes:
  /
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetCamSleep_Pri(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	printk("CAMDRV_SetCamSleep(): OV5640 enter soft standby mode \r\n");


	return result;
}

/*camera_disable for still mode */
static HAL_CAM_Result_en_t CAMDRV_DisableCapture_Pri(CamSensorSelect_t sensor)
{
	//return HAL_CAM_SUCCESS;
	/*[Disable stream] */
	//printk(KERN_ERR"in 9t111 CAMDRV_DisableCapture \r\n");
	control_flash(0);

	/*[Preview on] */

	//printk("CAMDRV_DisableCapture(): \r\n");
	return sCamI2cStatus;
}

/****************************************************************************
  /
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_DisablePreview(void)
  /
  / Description: This function halts MT9M111 camera video
  /
  / Notes:   camera_disable for preview mode , after this ,preview will not output data
  /
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_DisablePreview_Pri(CamSensorSelect_t sensor)
{return HAL_CAM_SUCCESS;
	/* [Disable stream] */

	printk("ov5640 CAMDRV_DisablePreview(): \r\n");
	return sCamI2cStatus;
}

/****************************************************************************
  /
  / Function Name:   voidcontrol_flash(int flash_on)
  /
  / Description: flash_on ==1:  turn on flash
  /			flash_on==0:   turn off flash
  /
  / Notes:
  /
 ****************************************************************************/

static void control_flash(int flash_on)
{
	gpio_request(FLASH_TRIGGER, "hal_cam_gpio_cntrl");

	if(flash_on==1)
	{
		if (gpio_is_valid(FLASH_TRIGGER))
		{
			gpio_direction_output(FLASH_TRIGGER, 1);
			msleep(5);		
		}
	}
	else
	{
		if (gpio_is_valid(FLASH_TRIGGER))
		{
			gpio_direction_output(FLASH_TRIGGER, 0);	
		}
	}

}


/****************************************************************************
  /
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_CfgStillnThumbCapture(
  CamImageSize_t image_resolution,
  CamDataFmt_t format,
  CamSensorSelect_t sensor)
  /
  / Description: This function configures Stills Capture
  /
  / Notes:
  /
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_CfgStillnThumbCapture_Pri(CamImageSize_t
		stills_resolution,
		CamDataFmt_t stills_format,
		CamImageSize_t
		thumb_resolution,
		CamDataFmt_t thumb_format,
		CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

		 // set OV5640 to capture mode
	
		 int preview_shutter, preview_gain16, average;
		 int capture_shutter, capture_gain16;
		 int capture_sysclk, capture_HTS, capture_VTS;
		 int light_frequency, capture_bandingfilter, capture_max_band;
		 long capture_gain16_shutter;
	
		ov5640_write(0x3503, 0x07); //disable aec/agc

		ov5640_write(0x302C, 0x82); //added by leyou
	
		 // read preview shutter
		 preview_shutter = OV5640_get_shutter();

		printk(KERN_ERR"youle_ov: preview_shutter =%d \n",preview_shutter);
		 // read preview gain
		 preview_gain16 = OV5640_get_gain16();
		printk(KERN_ERR"youle_ov: preview_gain16 =%d \n",preview_gain16);
		 // get average
		 average = ov5640_read(0x56a1);
	
		 // turn off night mode for capture
		/* OV5640_set_night_mode(0);   midifyied by leyou */
	
		 // turn off overlay
		/*  OV5640_write_i2c(0x3022, 0x06);*/
	
		 // Write capture setting
		 OV5640_capture_setting(stills_resolution);	 
	
		 // read capture VTS
		 capture_VTS = OV5640_get_VTS();
		 capture_HTS = OV5640_get_HTS();
		 capture_sysclk = OV5640_get_sysclk();

		 printk(KERN_ERR"youle_ov: capture_sysclk =%d \n",capture_sysclk);
	
		 // calculate capture banding filter
		 light_frequency = OV5640_get_light_frequency();
		 if (light_frequency == 60) {
			 // 60Hz
			 capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
		 }
		 else {
			 // 50Hz
			 capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
		 }
		 capture_max_band = (int) ((capture_VTS - 4)/capture_bandingfilter);
#if 1
		 // calculate capture shutter/gain16
	
		 capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
	
	
		 // gain to shutter
		 if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
			 // shutter < 1/100
			 capture_shutter = capture_gain16_shutter/16;
			 if(capture_shutter <1)
				 capture_shutter = 1;
	
			 capture_gain16 = capture_gain16_shutter/capture_shutter;
			 if(capture_gain16 < 16)
				 capture_gain16 = 16;
		 }
		 else {
			 if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
				 // exposure reach max
				 capture_shutter = capture_bandingfilter*capture_max_band;
				 capture_gain16 = capture_gain16_shutter / capture_shutter;
			 }
			 else {
				 // 1/100 < capture_shutter =< max, capture_shutter = n/100
				 capture_shutter = ((int) (capture_gain16_shutter/16/capture_bandingfilter)) * capture_bandingfilter;
				 capture_gain16 = capture_gain16_shutter / capture_shutter;
			 }
		 }
	
#else
	capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
	capture_gain16 = preview_gain16;
#endif
		 // write capture gain
		 OV5640_set_gain16(capture_gain16);
	 printk(KERN_ERR"youle_ov  -2: capture_gain16 =%d \n",capture_gain16);
		 // write capture shutter
		 if (capture_shutter > (capture_VTS - 4)) {
			 capture_VTS = capture_shutter + 4;
			 OV5640_set_VTS(capture_VTS);
		 }
		 OV5640_set_shutter(capture_shutter);
	 printk(KERN_ERR"youle_ov -2: capture_shutter =%d \n",capture_shutter);
	
		 // skip 2 vysnc
		 printk("youle_ov: sleep 500ms has added \n");
	//msleep(500);
		 // start capture at 3rd vsync

	return result;
}

/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetSceneMode(
  /					CamSceneMode_t scene_mode)
  /
  / Description: This function will set the scene mode of camera
  / Notes:
 ****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetSceneMode_Pri(CamSceneMode_t scene_mode,
		CamSensorSelect_t sensor)
{return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	switch(scene_mode) {
		case CamSceneMode_Auto:

			ov5640_write(0x3a00 ,0x78); 

/*这里需要加上framerate的设置*/
			ov5640_write(0x3a18 ,0x00); //gain
			ov5640_write(0x3a19 ,0xf8);
			pr_info("CAMDRV_SetSceneMode() called for AUTO\n");
			break;
		case CamSceneMode_Night:
			pr_info("CAMDRV_SetSceneMode() called for Night\n");
			#if 0
			ov5640_write(0x3034 ,0x1a);
			ov5640_write(0x3035 ,0x21);
			ov5640_write(0x3036 ,0x46);
			ov5640_write(0x3037 ,0x13);
			ov5640_write(0x3038 ,0x00);
			ov5640_write(0x3039 ,0x00);
			ov5640_write(0x3a00 ,0x7c);
			ov5640_write(0x3a08 ,0x01);
			ov5640_write(0x3a09 ,0x27);
			ov5640_write(0x3a0a ,0x00);
			ov5640_write(0x3a0b ,0xf6);
			ov5640_write(0x3a0d ,0x04);
			ov5640_write(0x3a0e ,0x04);
			ov5640_write(0x3a02 ,0x0b);
			ov5640_write(0x3a03 ,0x88);
			ov5640_write(0x3a14 ,0x0b);
			ov5640_write(0x3a15 ,0x88);
			#else

			/*这里需要加上framerate的设置*/
			ov5640_write(0x3a18 ,0x01); //gain
			ov5640_write(0x3a19 ,0xf8);

			#endif
			break;
		default:
			pr_info("CAMDRV_SetSceneMode() not supported for %d\n", scene_mode);
			break;
	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetSceneMode(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetWBMode(CamWB_WBMode_t wb_mode)
  /
  / Description: This function will set the white balance of camera
  / Notes:
 ****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetWBMode_Pri(CamWB_WBMode_t wb_mode,
		CamSensorSelect_t sensor)
{//return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetWBMode()  called\n");
	if (wb_mode == CamWB_Auto) {
		ov5640_write(0x3406 ,0x00);

	} else if (wb_mode == CamWB_Incandescent) {
		ov5640_write(0x3406 ,0x1 );
		ov5640_write(0x3400 ,0x4 );
		ov5640_write(0x3401 ,0x88);
		ov5640_write(0x3402 ,0x4 );
		ov5640_write(0x3403 ,0x0 );
		ov5640_write(0x3404 ,0x8 );
		ov5640_write(0x3405 ,0xb6);
	 
	} else if ((wb_mode == CamWB_DaylightFluorescent)
			|| (wb_mode == CamWB_WarmFluorescent)) {
		ov5640_write(0x3406 ,0x1 ),
		ov5640_write(0x3400 ,0x6 );
		ov5640_write(0x3401 ,0x2a);
		ov5640_write(0x3402 ,0x4 );
		ov5640_write(0x3403 ,0x0 );
		ov5640_write(0x3404 ,0x7 );
		ov5640_write(0x3405 ,0x24);

	} else if (wb_mode == CamWB_Daylight) {
		ov5640_write(0x3406 ,0x1 );
		ov5640_write(0x3400 ,0x7 );
		ov5640_write(0x3401 ,0x02);
		ov5640_write(0x3402 ,0x4 );
		ov5640_write(0x3403 ,0x0 );
		ov5640_write(0x3404 ,0x5 );
		ov5640_write(0x3405 ,0x15);


	} else if (wb_mode == CamWB_Cloudy) {

		ov5640_write(0x3406 ,0x1 );
		ov5640_write(0x3400 ,0x7 );
		ov5640_write(0x3401 ,0x88);
		ov5640_write(0x3402 ,0x4 );
		ov5640_write(0x3403 ,0x0 );
		ov5640_write(0x3404 ,0x5 );
		ov5640_write(0x3405 ,0x00);



	} else if (wb_mode == CamWB_Twilight) {
		//To do Venky

	} else {
		printk("Am here in wb:%d\n", wb_mode);
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetWBMode(): Error[%d] \r\n", sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetAntiBanding(
  /					CamAntiBanding_t effect)
  /
  / Description: This function will set the antibanding effect of camera
  / Notes:
 ****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetAntiBanding_Pri(CamAntiBanding_t effect,
		CamSensorSelect_t sensor)
{return HAL_CAM_SUCCESS;

	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	printk("CAMDRV_SetAntiBanding()  called\n");
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) {

		//To do Venky 
	} else if (effect == CamAntiBanding50Hz) {
		ov5640_write(0x3c00, 0x04); //bit[2]select 50/60hz banding, 0:50hz
		ov5640_write(0x3c01, 0x80); //bit[7] banding filter Auto Detection on/off, 1 off
		//ov5640_write(0x3a08, 0x01); //50Hz banding filter value 8 MSB
		//ov5640_write(0x3a09, 0x27); //50Hz banding filter value 8 LSB
		//ov5640_write(0x3a0e, 0x04);


		//To do Venky 
		;
	} else if (effect == CamAntiBanding60Hz) {
		ov5640_write(0x3c00, 0x00);
		ov5640_write(0x3c01, 0x80);
		//ov5640_write(0x3a0a, 0x00); //60Hz banding filter value 8MSB
		//ov5640_write(0x3a0b, 0xf6); //60Hz banding filter value 8 LSB
		//ov5640_write(0x3a0d, 0x06); //60Hz maximum banding step


		//To do Venky 
		;
	} else {
		// To do Venky 
		;
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetAntiBanding(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetFlashMode(
  FlashLedState_t effect)
  /
  / Description: This function will set the flash mode of camera
  / Notes:
 ****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetFlashMode_Pri(FlashLedState_t effect,
		CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	printk("CAMDRV_SetFlashMode()  called\n");

	flash_switch=effect;
	if (effect == Flash_Off) {
		/* do sth */
	} else if (effect == Flash_On) {
		/* do sth */
	} else if (effect == Torch_On) {
		/* do sth */
	} else if (effect == FlashLight_Auto) {
		/* do sth */
	} else {
		/* do sth */
	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetFlashMode(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}

/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetFocusMode(
  /					CamFocusStatus_t effect)
  /
  / Description: This function will set the focus mode of camera
  / Notes:
 ****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetFocusMode_Pri(CamFocusControlMode_t effect,
		CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	int max_sleep=50;
	
	printk("CAMDRV_SetFocusMode()  called effect 0x%x \n",effect);
	if (effect == CamFocusControlAuto) {
		
		ov5640_write(0x3023, 0x01);
			ov5640_write(0x3022, 0x08);/* leyou:    is it right ???  */
	}
	else if (effect == CamFocusControlMacro) {

			ov5640_write(0x3027, 0x00);
			ov5640_write(0x3028, 0xff);
			
			ov5640_write(0x3023, 0x01);
			ov5640_write(0x3022, 0x1a);

			
			
			while(  (ov5640_read(0x3023)!=0) &&( max_sleep>0) )
			{
				msleep(5);
				max_sleep --;
			}
			
			

			if(max_sleep<=0)
				printk("CAMDRV_SetFocusMode_Pri auto-focus failed  \r\n");
			else
				printk("CAMDRV_SetFocusMode_Pri auto-focus suceed   \r\n");
			

		// To do Venky
	} else if (effect == CamFocusControlInfinity) {

		//ov5640_write(0x3027, 0x00);
			//ov5640_write(0x3028, 0xff);
			
			ov5640_write(0x3023, 0x01);
			ov5640_write(0x3022, 0x08);

			
			
			while(  (ov5640_read(0x3023)!=0) &&( max_sleep>0) )
			{
				msleep(5);
				max_sleep --;
			}
			
			

			if(max_sleep<=0)
				printk("CAMDRV_SetFocusMode_Pri auto-focus failed  \r\n");
			else
				printk("CAMDRV_SetFocusMode_Pri auto-focus suceed   \r\n");	
			
		// To do Venky

	} else {

		printk(KERN_ERR" can not support effect focus 0x%x\r\n",effect);

	}
	//if (sCamI2cStatus != HAL_CAM_SUCCESS) {
	//	printk("CAMDRV_SetFocusMode(): Error[%d] \r\n",
	//		 sCamI2cStatus);
	//	result = sCamI2cStatus;
	//}

	FOCUS_MODE=effect;

	return result;


}

static HAL_CAM_Result_en_t CAMDRV_TurnOffAF_Pri()
{

	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	return result;
}

static HAL_CAM_Result_en_t CAMDRV_TurnOnAF_Pri()
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	UInt8 read_out;
	int max_sleep=50;
		

	switch (FOCUS_MODE)
	{
		case CamFocusControlAuto: /*AF trigger*/

			ov5640_write(0x3023, 0x01);
			ov5640_write(0x3022, 0x03);

			while(  (ov5640_read(0x3023)!=0) &&( max_sleep>0) )
			{
				msleep(5);
				max_sleep --;
			}
			
			read_out=ov5640_read(0x3028);

			if(read_out==0)
				printk("CAMDRV_SetFocusMode_Pri auto-focus failed  \r\n");
			else
				printk("CAMDRV_SetFocusMode_Pri auto-focus suceed   \r\n");
			

			// To do Venky 
			break;

		case CamFocusControlMacro:/*we need not doing trigger for this mode */

			msleep(100);
			break;

		case CamFocusControlInfinity:/*we need not doing trigger for this mode */

			msleep(100);
			break;

		default:
			printk(KERN_ERR"error in CAMDRV_TurnOnAF_Pri , can not support focus mode 0x%x \n",FOCUS_MODE);

	}


	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_TurnOnAF(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}

/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetJpegQuality(
  /					CamFocusStatus_t effect)
  /
  / Description: This function will set the focus mode of camera
  / Notes:
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetJpegQuality_Pri(CamJpegQuality_t effect,
		CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetJpegQuality()  called\n");
	if (effect == CamJpegQuality_Min) {
		/* do sth */
	} else if (effect == CamJpegQuality_Nom) {
		/* do sth */
	} else {
		/* do sth */
	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetJpegQuality(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}
void zoomfunction(unsigned char bSensorFormat, unsigned int zoom_level,unsigned int iStepSize,unsigned int dvp_w, unsigned int dvp_h)
{

	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	UInt8 Reg3800, Reg3801, Reg3804, Reg3805;
	UInt8 Reg3808, Reg3809,Reg4402;
	UInt8 Reg3810, Reg3811, Reg3812, Reg3813;
	unsigned int MinLevel, MaxLevel;
	unsigned int windw, windh,iTmp;
	unsigned int zoom;

	if ((dvp_w >=1280) &&(iStepSize >= 100))
		iStepSize = 50;
	zoom = 100+zoom_level * iStepSize;//(zoom_level +1)*100;
	MinLevel = 100;

	if (bSensorFormat == SENSOR_FORMAT_PREVIEW)
	{
		MaxLevel = (1280 / dvp_w)*100;
	}
	else if (bSensorFormat == 1)
	{
		MaxLevel = (2592 / dvp_w)*100;
	}

	if (zoom < MinLevel)
	{
		zoom = MinLevel;
	}
	else if (zoom > MaxLevel)
	{
		zoom = MaxLevel;
	}
	if (bSensorFormat == SENSOR_FORMAT_PREVIEW)
	{
		windw = 16+ (zoom - 100) * (1280 / 2) / zoom;
		windh = 4 + (zoom - 100) * (960 / 2) / zoom;
	}
	else{
		windw = 16+ (zoom - 100) * (2592 / 2) / zoom;
		windh = 4 + (zoom - 100) * (1944 / 2) / zoom;
	}
	Reg3800 = ov5640_read(0x3800);
	Reg3801 =  ov5640_read(0x3801);
	Reg3804 = ov5640_read(0x3804);
	Reg3805 = ov5640_read(0x3805);
	Reg3808 = ov5640_read(0x3808);
	Reg3809 =  ov5640_read(0x3809);
	Reg3810  = ov5640_read(0x3810);
	Reg3811=ov5640_read(0x3811);
	ov5640_write(0x3810,(windw >> 8));
	ov5640_write(0x3811,(windw & 0xFF));
	ov5640_write(0x3812,(windh >> 8));
	ov5640_write(0x3813,(windh & 0xFF));

	iTmp =( ((Reg3804 <<8) +Reg3805 -((Reg3800 << 8) +Reg3801) + 1 ) -(2*((Reg3810 <<8) + Reg3811))) / ((Reg3808<<8) +Reg3809);

	if (iTmp <2)
	{
		ov5640_write(0x4402, 0x90);
	}else
	{
		ov5640_write(0x4402, 0x10);
	}
	//return result;
}



/****************************************************************************
  / Function Name:   HAL_CAM_Result_en_t CAMDRV_SetDigitalEffect(
  /					CamDigEffect_t effect)
  /
  / Description: This function will set the digital effect of camera
  / Notes:
 ****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetDigitalEffect_Pri(CamDigEffect_t effect,
		CamSensorSelect_t sensor)
{//return HAL_CAM_SUCCESS;
	UInt8 reg_value;
	reg_value=ov5640_read(0x5580);
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	if (effect == CamDigEffect_NoEffect) {
		ov5640_write(0x5001 ,0xff);
		reg_value = reg_value & 0xa7;
		ov5640_write(0x5580 ,reg_value); 

	} else if (effect == CamDigEffect_MonoChrome) {
		ov5640_write(0x5001 ,0xff);
		reg_value = reg_value | 0x18;
		ov5640_write(0x5580 ,reg_value); 

		ov5640_write(0x5583 ,0x80);
		ov5640_write(0x5584 ,0x80);

		

	} else if ((effect == CamDigEffect_NegColor)
			|| (effect == CamDigEffect_NegMono)) {
			ov5640_write(0x5001 ,0xff);
			reg_value = reg_value | 0x40;
			ov5640_write(0x5580 ,reg_value); 

		
	} else if ((effect == CamDigEffect_SolarizeColor)
			|| (effect == CamDigEffect_SolarizeMono)) {
		
		;

	} else if (effect == CamDigEffect_SepiaGreen) {
		ov5640_write(0x5001 ,0xff);
		reg_value = reg_value | 0x18;
		ov5640_write(0x5580 ,reg_value); 

		ov5640_write(0x5583 ,0x40);
		ov5640_write(0x5584 ,0xa0);

		
	} else if (effect == CamDigEffect_Auqa) {
		//To do Venky 
	} else {
		;
		//To do Venky 
	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

static HAL_CAM_Result_en_t Init_OV5640(CamSensorSelect_t sensor)
{
	
		HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
		static ktime_t tm1;
		tm1 = ktime_get();
	
		pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);
	
		CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;
		printk("Init Primary Sensor OV5640: \r\n");

		   ov5640_write(0x3103, 0x11);
   ov5640_write(0x3008, 0x82);
   ov5640_write(0x3008, 0x42);
   ov5640_write(0x3103, 0x03);
   ov5640_write(0x3017, 0xff);
   ov5640_write(0x3018, 0xff);
   ov5640_write(0x3034, 0x1a);
   ov5640_write(0x3035, 0x11);
   ov5640_write(0x3036, 0x46);
   ov5640_write(0x3037, 0x13);
   ov5640_write(0x3108, 0x01);
   ov5640_write(0x3630, 0x36);
   ov5640_write(0x3631, 0x0e);
   ov5640_write(0x3632, 0xe2);
   ov5640_write(0x3633, 0x12);
   ov5640_write(0x3621, 0xe0);
   ov5640_write(0x3704, 0xa0);
   ov5640_write(0x3703, 0x5a);
   ov5640_write(0x3715, 0x78);
   ov5640_write(0x3717, 0x01);
   ov5640_write(0x370b, 0x60);
   ov5640_write(0x3705, 0x1a);
   ov5640_write(0x3905, 0x02);
   ov5640_write(0x3906, 0x10);
   ov5640_write(0x3901, 0x0a);
   ov5640_write(0x3731, 0x12);
   ov5640_write(0x3600, 0x08);
   ov5640_write(0x3601, 0x33);
   ov5640_write(0x302d, 0x60);
   ov5640_write(0x3620, 0x52);
   ov5640_write(0x371b, 0x20);
   ov5640_write(0x471c, 0x50);
   ov5640_write(0x3a13, 0x43);
   ov5640_write(0x3a18, 0x00);
   ov5640_write(0x3a19, 0xf8);
   ov5640_write(0x3635, 0x13);
   ov5640_write(0x3636, 0x03);
   ov5640_write(0x3634, 0x40);
   ov5640_write(0x3622, 0x01);
   ov5640_write(0x3c01, 0xB4);
   ov5640_write(0x3c00, 0x04);
   ov5640_write(0x3c04, 0x28);
   ov5640_write(0x3c05, 0x98);
   ov5640_write(0x3c06, 0x00);
   ov5640_write(0x3c07, 0x08);
   ov5640_write(0x3c08, 0x00);
   ov5640_write(0x3c09, 0x1c);
   ov5640_write(0x3c0a, 0x9c);
   ov5640_write(0x3c0b, 0x40);
   ov5640_write(0x3820, 0x41);
   ov5640_write(0x3821, 0x07);
   ov5640_write(0x3814, 0x31);
   ov5640_write(0x3815, 0x31);
   ov5640_write(0x3800, 0x00);
   ov5640_write(0x3801, 0x00);
   ov5640_write(0x3802, 0x00);
   ov5640_write(0x3803, 0x04);
   ov5640_write(0x3804, 0x0a);
   ov5640_write(0x3805, 0x3f);
   ov5640_write(0x3806, 0x07);
   ov5640_write(0x3807, 0x9b);
   ov5640_write(0x3808, 0x02);
   ov5640_write(0x3809, 0x80);
   ov5640_write(0x380a, 0x01);
   ov5640_write(0x380b, 0xe0);
   ov5640_write(0x380c, 0x07);
   ov5640_write(0x380d, 0x68);
   ov5640_write(0x380e, 0x03);
   ov5640_write(0x380f, 0xd8);
   ov5640_write(0x3810, 0x00);
   ov5640_write(0x3811, 0x10);
   ov5640_write(0x3812, 0x00);
   ov5640_write(0x3813, 0x06);
   ov5640_write(0x3618, 0x00);
   ov5640_write(0x3612, 0x29);
   ov5640_write(0x3708, 0x62);
   ov5640_write(0x3709, 0x52);
   ov5640_write(0x370c, 0x03);
   ov5640_write(0x3a02, 0x03);
   ov5640_write(0x3a03, 0xd8);
   ov5640_write(0x3a08, 0x01);
   ov5640_write(0x3a09, 0x27);
   ov5640_write(0x3a0a, 0x00);
   ov5640_write(0x3a0b, 0xf6);
   ov5640_write(0x3a0e, 0x03);
   ov5640_write(0x3a0d, 0x04);
   ov5640_write(0x3a14, 0x03);
   ov5640_write(0x3a15, 0xd8);
   ov5640_write(0x4001, 0x02);
   ov5640_write(0x4004, 0x02);
   ov5640_write(0x3000, 0x00);
   ov5640_write(0x3002, 0x1c);
   ov5640_write(0x3004, 0xff);
   ov5640_write(0x3006, 0xc3);
   ov5640_write(0x300e, 0x58);
   ov5640_write(0x302e, 0x00);
   ov5640_write(0x4300, 0x30);
   ov5640_write(0x501f, 0x00);
   ov5640_write(0x4713, 0x03);
   ov5640_write(0x4407, 0x04);
   ov5640_write(0x440e, 0x00);
   ov5640_write(0x460b, 0x35);
   ov5640_write(0x460c, 0x22);
   ov5640_write(0x3824, 0x02);
   ov5640_write(0x5000, 0xa7);
   ov5640_write(0x5001, 0xa3);
   

   ov5640_write(0x5300, 0x08);
   ov5640_write(0x5301, 0x30);
   ov5640_write(0x5302, 0x10);
   ov5640_write(0x5303, 0x00);
   ov5640_write(0x5304, 0x08);
   ov5640_write(0x5305, 0x30);
   ov5640_write(0x5306, 0x08);
   ov5640_write(0x5307, 0x16);
   ov5640_write(0x5309, 0x08);
   ov5640_write(0x530a, 0x30);
   ov5640_write(0x530b, 0x04);
   ov5640_write(0x530c, 0x06);
   ov5640_write(0x5480, 0x01);   
   ov5640_write(0x5580, 0x02);
   ov5640_write(0x5583, 0x40);
   ov5640_write(0x5584, 0x10);
   ov5640_write(0x5589, 0x10);
   ov5640_write(0x558a, 0x00);
   ov5640_write(0x558b, 0xf8);
  #if 0
     ov5640_write(0x5381, 0x1e);
   ov5640_write(0x5382, 0x5b);
   ov5640_write(0x5383, 0x08);
   ov5640_write(0x5384, 0x0a);
   ov5640_write(0x5385, 0x7e);
   ov5640_write(0x5386, 0x88);
   ov5640_write(0x5387, 0x7c);
   ov5640_write(0x5388, 0x6c);
   ov5640_write(0x5389, 0x10);
   ov5640_write(0x538a, 0x01);
   ov5640_write(0x538b, 0x98);   
   ov5640_write(0x5180, 0xff);
   ov5640_write(0x5181, 0xf2);
   ov5640_write(0x5182, 0x00);
   ov5640_write(0x5183, 0x14);
   ov5640_write(0x5184, 0x25);
   ov5640_write(0x5185, 0x24);
   ov5640_write(0x5186, 0x09);
   ov5640_write(0x5187, 0x09);
   ov5640_write(0x5188, 0x09);
   ov5640_write(0x5189, 0x75);
   ov5640_write(0x518a, 0x54);
   ov5640_write(0x518b, 0xe0);
   ov5640_write(0x518c, 0xb2);
   ov5640_write(0x518d, 0x42);
   ov5640_write(0x518e, 0x3d);
   ov5640_write(0x518f, 0x56);
   ov5640_write(0x5190, 0x46);
   ov5640_write(0x5191, 0xf8);
   ov5640_write(0x5192, 0x04);
   ov5640_write(0x5193, 0x70);
   ov5640_write(0x5194, 0xf0);
   ov5640_write(0x5195, 0xf0);
   ov5640_write(0x5196, 0x03);
   ov5640_write(0x5197, 0x01);
   ov5640_write(0x5198, 0x04);
   ov5640_write(0x5199, 0x12);
   ov5640_write(0x519a, 0x04);
   ov5640_write(0x519b, 0x00);
   ov5640_write(0x519c, 0x06);
   ov5640_write(0x519d, 0x82);
   ov5640_write(0x519e, 0x38);
   ov5640_write(0x5481, 0x08);
   ov5640_write(0x5482, 0x14);
   ov5640_write(0x5483, 0x28);
   ov5640_write(0x5484, 0x51);
   ov5640_write(0x5485, 0x65);
   ov5640_write(0x5486, 0x71);
   ov5640_write(0x5487, 0x7d);
   ov5640_write(0x5488, 0x87);
   ov5640_write(0x5489, 0x91);
   ov5640_write(0x548a, 0x9a);
   ov5640_write(0x548b, 0xaa);
   ov5640_write(0x548c, 0xb8);
   ov5640_write(0x548d, 0xcd);
   ov5640_write(0x548e, 0xdd);
   ov5640_write(0x548f, 0xea);
   ov5640_write(0x5490, 0x1d);  
   ov5640_write(0x5800, 0x23);
   ov5640_write(0x5801, 0x14);
   ov5640_write(0x5802, 0x0f);
   ov5640_write(0x5803, 0x0f);
   ov5640_write(0x5804, 0x12);
   ov5640_write(0x5805, 0x26);
   ov5640_write(0x5806, 0x0c);
   ov5640_write(0x5807, 0x08);
   ov5640_write(0x5808, 0x05);
   ov5640_write(0x5809, 0x05);
   ov5640_write(0x580a, 0x08);
   ov5640_write(0x580b, 0x0d);
   ov5640_write(0x580c, 0x08);
   ov5640_write(0x580d, 0x03);
   ov5640_write(0x580e, 0x00);
   ov5640_write(0x580f, 0x00);
   ov5640_write(0x5810, 0x03);
   ov5640_write(0x5811, 0x09);
   ov5640_write(0x5812, 0x07);
   ov5640_write(0x5813, 0x03);
   ov5640_write(0x5814, 0x00);
   ov5640_write(0x5815, 0x01);
   ov5640_write(0x5816, 0x03);
   ov5640_write(0x5817, 0x08);
   ov5640_write(0x5818, 0x0d);
   ov5640_write(0x5819, 0x08);
   ov5640_write(0x581a, 0x05);
   ov5640_write(0x581b, 0x06);
   ov5640_write(0x581c, 0x08);
   ov5640_write(0x581d, 0x0e);
   ov5640_write(0x581e, 0x29);
   ov5640_write(0x581f, 0x17);
   ov5640_write(0x5820, 0x11);
   ov5640_write(0x5821, 0x11);
   ov5640_write(0x5822, 0x15);
   ov5640_write(0x5823, 0x28);
   ov5640_write(0x5824, 0x46);
   ov5640_write(0x5825, 0x26);
   ov5640_write(0x5826, 0x08);
   ov5640_write(0x5827, 0x26);
   ov5640_write(0x5828, 0x64);
   ov5640_write(0x5829, 0x26);
   ov5640_write(0x582a, 0x24);
   ov5640_write(0x582b, 0x22);
   ov5640_write(0x582c, 0x24);
   ov5640_write(0x582d, 0x24);
   ov5640_write(0x582e, 0x06);
   ov5640_write(0x582f, 0x22);
   ov5640_write(0x5830, 0x40);
   ov5640_write(0x5831, 0x42);
   ov5640_write(0x5832, 0x24);
   ov5640_write(0x5833, 0x26);
   ov5640_write(0x5834, 0x24);
   ov5640_write(0x5835, 0x22);
   ov5640_write(0x5836, 0x22);
   ov5640_write(0x5837, 0x26);
   ov5640_write(0x5838, 0x44);
   ov5640_write(0x5839, 0x24);
   ov5640_write(0x583a, 0x26);
   ov5640_write(0x583b, 0x28);
   ov5640_write(0x583c, 0x42);
   ov5640_write(0x583d, 0xce);
   ov5640_write(0x3a0f, 0x30);
   ov5640_write(0x3a10, 0x28);
   ov5640_write(0x3a1b, 0x30);
   ov5640_write(0x3a1e, 0x26);
   ov5640_write(0x3a11, 0x60);
   ov5640_write(0x3a1f, 0x14);
   #else
   ov5640_write(0x5381, 0x1c); //cmx
   ov5640_write(0x5382, 0x5a);
   ov5640_write(0x5383, 0x04);
   ov5640_write(0x5384, 0x05);
   ov5640_write(0x5385, 0x62);
   ov5640_write(0x5386, 0x68);
   ov5640_write(0x5387, 0x62);
   ov5640_write(0x5388, 0x56);
   ov5640_write(0x5389, 0x0c);
   ov5640_write(0x538a, 0x01);
   ov5640_write(0x538b, 0x98);   
   ov5640_write(0x5180, 0xff); //awb
   ov5640_write(0x5181, 0xf2);
   ov5640_write(0x5182, 0x00);
   ov5640_write(0x5183, 0x14);
   ov5640_write(0x5184, 0x25);
   ov5640_write(0x5185, 0x24);
   ov5640_write(0x5186, 0x0b);
   ov5640_write(0x5187, 0x30);
   ov5640_write(0x5188, 0x15);
   ov5640_write(0x5189, 0x73);
   ov5640_write(0x518a, 0x5b);
   ov5640_write(0x518b, 0xff);
   ov5640_write(0x518c, 0xbb);
   ov5640_write(0x518d, 0x42);
   ov5640_write(0x518e, 0x39);
   ov5640_write(0x518f, 0x56);
   ov5640_write(0x5190, 0x48);
   ov5640_write(0x5191, 0xf8);
   ov5640_write(0x5192, 0x04);
   ov5640_write(0x5193, 0x70);
   ov5640_write(0x5194, 0xf0);
   ov5640_write(0x5195, 0xf0);
   ov5640_write(0x5196, 0x03);
   ov5640_write(0x5197, 0x01);
   ov5640_write(0x5198, 0x05);
   ov5640_write(0x5199, 0xec);
   ov5640_write(0x519a, 0x04);
   ov5640_write(0x519b, 0x00);
   ov5640_write(0x519c, 0x05);
   ov5640_write(0x519d, 0x6b);
   ov5640_write(0x519e, 0x38);
   ov5640_write(0x5481, 0x0b); //gamma
   ov5640_write(0x5482, 0x16);
   ov5640_write(0x5483, 0x28);
   ov5640_write(0x5484, 0x43);
   ov5640_write(0x5485, 0x55);
   ov5640_write(0x5486, 0x62);
   ov5640_write(0x5487, 0x70);
   ov5640_write(0x5488, 0x7f);
   ov5640_write(0x5489, 0x88);
   ov5640_write(0x548a, 0x93);
   ov5640_write(0x548b, 0xa8);
   ov5640_write(0x548c, 0xb4);
   ov5640_write(0x548d, 0xce);
   ov5640_write(0x548e, 0xe3);
   ov5640_write(0x548f, 0xed);
   ov5640_write(0x5490, 0x19);  
   ov5640_write(0x5800, 0x35); //lens shading
   ov5640_write(0x5801, 0x17);
   ov5640_write(0x5802, 0x11);
   ov5640_write(0x5803, 0x11);
   ov5640_write(0x5804, 0x17);
   ov5640_write(0x5805, 0x2f);
   ov5640_write(0x5806, 0x0e);
   ov5640_write(0x5807, 0x08);
   ov5640_write(0x5808, 0x05);
   ov5640_write(0x5809, 0x05);
   ov5640_write(0x580a, 0x08);
   ov5640_write(0x580b, 0x0e);
   ov5640_write(0x580c, 0x0a);
   ov5640_write(0x580d, 0x04);
   ov5640_write(0x580e, 0x00);
   ov5640_write(0x580f, 0x00);
   ov5640_write(0x5810, 0x03);
   ov5640_write(0x5811, 0x0a);
   ov5640_write(0x5812, 0x0b);
   ov5640_write(0x5813, 0x04);
   ov5640_write(0x5814, 0x00);
   ov5640_write(0x5815, 0x00);
   ov5640_write(0x5816, 0x03);
   ov5640_write(0x5817, 0x0a);
   ov5640_write(0x5818, 0x10);
   ov5640_write(0x5819, 0x0a);
   ov5640_write(0x581a, 0x07);
   ov5640_write(0x581b, 0x06);
   ov5640_write(0x581c, 0x0a);
   ov5640_write(0x581d, 0x10);
   ov5640_write(0x581e, 0x34);
   ov5640_write(0x581f, 0x1a);
   ov5640_write(0x5820, 0x15);
   ov5640_write(0x5821, 0x14);
   ov5640_write(0x5822, 0x19);
   ov5640_write(0x5823, 0x2e);
   ov5640_write(0x5824, 0x68);
   ov5640_write(0x5825, 0x27);
   ov5640_write(0x5826, 0x27);
   ov5640_write(0x5827, 0x25);
   ov5640_write(0x5828, 0x39);
   ov5640_write(0x5829, 0x28);
   ov5640_write(0x582a, 0x14);
   ov5640_write(0x582b, 0x13);
   ov5640_write(0x582c, 0x24);
   ov5640_write(0x582d, 0x28);
   ov5640_write(0x582e, 0x16);
   ov5640_write(0x582f, 0x22);
   ov5640_write(0x5830, 0x33);
   ov5640_write(0x5831, 0x32);
   ov5640_write(0x5832, 0x16);
   ov5640_write(0x5833, 0x19);
   ov5640_write(0x5834, 0x24);
   ov5640_write(0x5835, 0x23);
   ov5640_write(0x5836, 0x24);
   ov5640_write(0x5837, 0x18);
   ov5640_write(0x5838, 0x59);
   ov5640_write(0x5839, 0x28);
   ov5640_write(0x583a, 0x19);
   ov5640_write(0x583b, 0x19);
   ov5640_write(0x583c, 0x38);
   ov5640_write(0x583d, 0xdf);
   ov5640_write(0x3a0f, 0x38); //AE target
	ov5640_write(0x3a10, 0x30);
	ov5640_write(0x3a1b, 0x38);
	ov5640_write(0x3a1e, 0x30);
	ov5640_write(0x3a11, 0x61);
	ov5640_write(0x3a1f, 0x10);


   #endif

   ov5640_write(0x5688, 0x11);
   ov5640_write(0x5689, 0x11);
   ov5640_write(0x568a, 0x31);
   ov5640_write(0x568b, 0x13);
   ov5640_write(0x568c, 0x31);
   ov5640_write(0x568d, 0x13);
   ov5640_write(0x568e, 0x11);
   ov5640_write(0x568f, 0x11);
   ov5640_write(0x4009, 0x10);  
   ov5640_write(0x5025, 0x00);
 
   ov5640_write(0x3008, 0x02);				 
  // ov5640_write(0x3035, 0x21);
   ov5640_write(0x4005, 0x1a);
   ov5640_write(0x3820, 0x41);
   ov5640_write(0x3821, 0x07);

   tm1 = ktime_get();
   printk("Exit Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
			printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
					sCamI2cStatus);
			result = sCamI2cStatus;
		}

	return result;
	
}


UInt32 CAMDRV_GetJpegSize(CamSensorSelect_t sensor, void *data)
{
	printk("***********8venky %s",__func__);
        UInt8 reg_val;
    	UInt32 jpeg_size;

        /*---------Read Jpeg File Size */
    jpeg_size = ov5640_read(0x4414);            
    reg_val = ov5640_read(0x4415);      
    jpeg_size |= (UInt32)(reg_val << 8);
    reg_val = ov5640_read(0x4416);    
    jpeg_size |= (UInt32)(reg_val << 16);
    printk("**********************Venky  value of Jpeg lenght is %x",jpeg_size);
    return jpeg_size;

}
UInt16 *CAMDRV_GetJpeg(short *buf)
{
        return buf;
}


UInt8 * CAMDRV_GetThumbnail(void *buf, UInt32 offset)
{
    int length = (offset + 1) >> 1; // length in u16
    u8 *thumbnail = (char *)buf + (length << 1) + 2;
    return thumbnail;
}
/****************************************************************************
/
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetZoom(UInt8 numer, UInt8 denum)
/
/ Description: This function performs zooming via camera sensor
/
/ Notes:
/
****************************************************************************/
HAL_CAM_Result_en_t CAMDRV_SetZoom(CamZoom_t step, CamSensorSelect_t nSensor)
{
        HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
        return result;
}


void CAMDRV_StoreBaseAddress(void *virt_ptr)
{
}

HAL_CAM_Result_en_t CAMDRV_SetFirm_Pri(void)
{
	 HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
		printk("CAMDRV_SetFirm_Pri ------------ in \n");

		ov5640_write( 0x3000, 0x20);
ov5640_write( 0x8000, 0x02);
ov5640_write( 0x8001, 0x0b);
ov5640_write( 0x8002, 0x8b);
ov5640_write( 0x8003, 0x02);
ov5640_write( 0x8004, 0x07);
ov5640_write( 0x8005, 0x9b);
ov5640_write( 0x8006, 0x22);
ov5640_write( 0x8007, 0x00);
ov5640_write( 0x8008, 0x00);
ov5640_write( 0x8009, 0x00);
ov5640_write( 0x800a, 0x00);
ov5640_write( 0x800b, 0x02);
ov5640_write( 0x800c, 0x0b);
ov5640_write( 0x800d, 0x5b);
ov5640_write( 0x800e, 0xe5);
ov5640_write( 0x800f, 0x1f);
ov5640_write( 0x8010, 0x70);
ov5640_write( 0x8011, 0x58);
ov5640_write( 0x8012, 0xf5);
ov5640_write( 0x8013, 0x1e);
ov5640_write( 0x8014, 0xd2);
ov5640_write( 0x8015, 0x35);
ov5640_write( 0x8016, 0xff);
ov5640_write( 0x8017, 0xef);
ov5640_write( 0x8018, 0x25);
ov5640_write( 0x8019, 0xe0);
ov5640_write( 0x801a, 0x24);
ov5640_write( 0x801b, 0x4b);
ov5640_write( 0x801c, 0xf8);
ov5640_write( 0x801d, 0xe4);
ov5640_write( 0x801e, 0xf6);
ov5640_write( 0x801f, 0x08);
ov5640_write( 0x8020, 0xf6);
ov5640_write( 0x8021, 0x0f);
ov5640_write( 0x8022, 0xbf);
ov5640_write( 0x8023, 0x34);
ov5640_write( 0x8024, 0xf2);
ov5640_write( 0x8025, 0x78);
ov5640_write( 0x8026, 0xba);
ov5640_write( 0x8027, 0x76);
ov5640_write( 0x8028, 0x07);
ov5640_write( 0x8029, 0x18);
ov5640_write( 0x802a, 0x76);
ov5640_write( 0x802b, 0x04);
ov5640_write( 0x802c, 0xe6);
ov5640_write( 0x802d, 0x18);
ov5640_write( 0x802e, 0xf6);
ov5640_write( 0x802f, 0x08);
ov5640_write( 0x8030, 0xe6);
ov5640_write( 0x8031, 0x78);
ov5640_write( 0x8032, 0xb6);
ov5640_write( 0x8033, 0xf6);
ov5640_write( 0x8034, 0x78);
ov5640_write( 0x8035, 0xb9);
ov5640_write( 0x8036, 0xe6);
ov5640_write( 0x8037, 0x78);
ov5640_write( 0x8038, 0xb7);
ov5640_write( 0x8039, 0xf6);
ov5640_write( 0x803a, 0x78);
ov5640_write( 0x803b, 0xbc);
ov5640_write( 0x803c, 0x76);
ov5640_write( 0x803d, 0x2e);
ov5640_write( 0x803e, 0x08);
ov5640_write( 0x803f, 0x76);
ov5640_write( 0x8040, 0x04);
ov5640_write( 0x8041, 0xe4);
ov5640_write( 0x8042, 0x78);
ov5640_write( 0x8043, 0xb5);
ov5640_write( 0x8044, 0xf6);
ov5640_write( 0x8045, 0x90);
ov5640_write( 0x8046, 0x0e);
ov5640_write( 0x8047, 0x78);
ov5640_write( 0x8048, 0x93);
ov5640_write( 0x8049, 0xf5);
ov5640_write( 0x804a, 0x48);
ov5640_write( 0x804b, 0xe4);
ov5640_write( 0x804c, 0x78);
ov5640_write( 0x804d, 0xb3);
ov5640_write( 0x804e, 0xf6);
ov5640_write( 0x804f, 0x08);
ov5640_write( 0x8050, 0xf6);
ov5640_write( 0x8051, 0x74);
ov5640_write( 0x8052, 0xff);
ov5640_write( 0x8053, 0x78);
ov5640_write( 0x8054, 0xbe);
ov5640_write( 0x8055, 0xf6);
ov5640_write( 0x8056, 0x08);
ov5640_write( 0x8057, 0xf6);
ov5640_write( 0x8058, 0x75);
ov5640_write( 0x8059, 0x1f);
ov5640_write( 0x805a, 0x01);
ov5640_write( 0x805b, 0x78);
ov5640_write( 0x805c, 0xb9);
ov5640_write( 0x805d, 0xe6);
ov5640_write( 0x805e, 0x75);
ov5640_write( 0x805f, 0xf0);
ov5640_write( 0x8060, 0x05);
ov5640_write( 0x8061, 0xa4);
ov5640_write( 0x8062, 0xf5);
ov5640_write( 0x8063, 0x49);
ov5640_write( 0x8064, 0x12);
ov5640_write( 0x8065, 0x08);
ov5640_write( 0x8066, 0x61);
ov5640_write( 0x8067, 0xc2);
ov5640_write( 0x8068, 0x37);
ov5640_write( 0x8069, 0x22);
ov5640_write( 0x806a, 0x78);
ov5640_write( 0x806b, 0xb5);
ov5640_write( 0x806c, 0xe6);
ov5640_write( 0x806d, 0xd3);
ov5640_write( 0x806e, 0x94);
ov5640_write( 0x806f, 0x00);
ov5640_write( 0x8070, 0x40);
ov5640_write( 0x8071, 0x02);
ov5640_write( 0x8072, 0x16);
ov5640_write( 0x8073, 0x22);
ov5640_write( 0x8074, 0xe5);
ov5640_write( 0x8075, 0x1f);
ov5640_write( 0x8076, 0x64);
ov5640_write( 0x8077, 0x05);
ov5640_write( 0x8078, 0x70);
ov5640_write( 0x8079, 0x29);
ov5640_write( 0x807a, 0x90);
ov5640_write( 0x807b, 0x0e);
ov5640_write( 0x807c, 0x77);
ov5640_write( 0x807d, 0x93);
ov5640_write( 0x807e, 0xf5);
ov5640_write( 0x807f, 0x48);
ov5640_write( 0x8080, 0xe4);
ov5640_write( 0x8081, 0xf5);
ov5640_write( 0x8082, 0x1f);
ov5640_write( 0x8083, 0xc2);
ov5640_write( 0x8084, 0x01);
ov5640_write( 0x8085, 0x78);
ov5640_write( 0x8086, 0xb3);
ov5640_write( 0x8087, 0xe6);
ov5640_write( 0x8088, 0xfe);
ov5640_write( 0x8089, 0x08);
ov5640_write( 0x808a, 0xe6);
ov5640_write( 0x808b, 0xff);
ov5640_write( 0x808c, 0x78);
ov5640_write( 0x808d, 0x4b);
ov5640_write( 0x808e, 0xa6);
ov5640_write( 0x808f, 0x06);
ov5640_write( 0x8090, 0x08);
ov5640_write( 0x8091, 0xa6);
ov5640_write( 0x8092, 0x07);
ov5640_write( 0x8093, 0xa2);
ov5640_write( 0x8094, 0x37);
ov5640_write( 0x8095, 0xe4);
ov5640_write( 0x8096, 0x33);
ov5640_write( 0x8097, 0xf5);
ov5640_write( 0x8098, 0x3c);
ov5640_write( 0x8099, 0x90);
ov5640_write( 0x809a, 0x30);
ov5640_write( 0x809b, 0x28);
ov5640_write( 0x809c, 0xf0);
ov5640_write( 0x809d, 0x75);
ov5640_write( 0x809e, 0x1e);
ov5640_write( 0x809f, 0x10);
ov5640_write( 0x80a0, 0xd2);
ov5640_write( 0x80a1, 0x35);
ov5640_write( 0x80a2, 0x22);
ov5640_write( 0x80a3, 0xe5);
ov5640_write( 0x80a4, 0x49);
ov5640_write( 0x80a5, 0x75);
ov5640_write( 0x80a6, 0xf0);
ov5640_write( 0x80a7, 0x05);
ov5640_write( 0x80a8, 0x84);
ov5640_write( 0x80a9, 0x78);
ov5640_write( 0x80aa, 0xb9);
ov5640_write( 0x80ab, 0xf6);
ov5640_write( 0x80ac, 0x90);
ov5640_write( 0x80ad, 0x0e);
ov5640_write( 0x80ae, 0x85);
ov5640_write( 0x80af, 0xe4);
ov5640_write( 0x80b0, 0x93);
ov5640_write( 0x80b1, 0xff);
ov5640_write( 0x80b2, 0x25);
ov5640_write( 0x80b3, 0xe0);
ov5640_write( 0x80b4, 0x24);
ov5640_write( 0x80b5, 0x0a);
ov5640_write( 0x80b6, 0xf8);
ov5640_write( 0x80b7, 0xe6);
ov5640_write( 0x80b8, 0xfc);
ov5640_write( 0x80b9, 0x08);
ov5640_write( 0x80ba, 0xe6);
ov5640_write( 0x80bb, 0xfd);
ov5640_write( 0x80bc, 0x78);
ov5640_write( 0x80bd, 0xb9);
ov5640_write( 0x80be, 0xe6);
ov5640_write( 0x80bf, 0x25);
ov5640_write( 0x80c0, 0xe0);
ov5640_write( 0x80c1, 0x24);
ov5640_write( 0x80c2, 0x4b);
ov5640_write( 0x80c3, 0xf8);
ov5640_write( 0x80c4, 0xa6);
ov5640_write( 0x80c5, 0x04);
ov5640_write( 0x80c6, 0x08);
ov5640_write( 0x80c7, 0xa6);
ov5640_write( 0x80c8, 0x05);
ov5640_write( 0x80c9, 0xef);
ov5640_write( 0x80ca, 0x12);
ov5640_write( 0x80cb, 0x0a);
ov5640_write( 0x80cc, 0xb8);
ov5640_write( 0x80cd, 0xd3);
ov5640_write( 0x80ce, 0x78);
ov5640_write( 0x80cf, 0xb4);
ov5640_write( 0x80d0, 0x96);
ov5640_write( 0x80d1, 0xee);
ov5640_write( 0x80d2, 0x18);
ov5640_write( 0x80d3, 0x96);
ov5640_write( 0x80d4, 0x40);
ov5640_write( 0x80d5, 0x0d);
ov5640_write( 0x80d6, 0x78);
ov5640_write( 0x80d7, 0xb9);
ov5640_write( 0x80d8, 0xe6);
ov5640_write( 0x80d9, 0x78);
ov5640_write( 0x80da, 0xb6);
ov5640_write( 0x80db, 0xf6);
ov5640_write( 0x80dc, 0x78);
ov5640_write( 0x80dd, 0xb3);
ov5640_write( 0x80de, 0xa6);
ov5640_write( 0x80df, 0x06);
ov5640_write( 0x80e0, 0x08);
ov5640_write( 0x80e1, 0xa6);
ov5640_write( 0x80e2, 0x07);
ov5640_write( 0x80e3, 0x90);
ov5640_write( 0x80e4, 0x0e);
ov5640_write( 0x80e5, 0x85);
ov5640_write( 0x80e6, 0xe4);
ov5640_write( 0x80e7, 0x93);
ov5640_write( 0x80e8, 0x12);
ov5640_write( 0x80e9, 0x0a);
ov5640_write( 0x80ea, 0xb8);
ov5640_write( 0x80eb, 0xc3);
ov5640_write( 0x80ec, 0x78);
ov5640_write( 0x80ed, 0xbf);
ov5640_write( 0x80ee, 0x96);
ov5640_write( 0x80ef, 0xee);
ov5640_write( 0x80f0, 0x18);
ov5640_write( 0x80f1, 0x96);
ov5640_write( 0x80f2, 0x50);
ov5640_write( 0x80f3, 0x0d);
ov5640_write( 0x80f4, 0x78);
ov5640_write( 0x80f5, 0xb9);
ov5640_write( 0x80f6, 0xe6);
ov5640_write( 0x80f7, 0x78);
ov5640_write( 0x80f8, 0xb7);
ov5640_write( 0x80f9, 0xf6);
ov5640_write( 0x80fa, 0x78);
ov5640_write( 0x80fb, 0xbe);
ov5640_write( 0x80fc, 0xa6);
ov5640_write( 0x80fd, 0x06);
ov5640_write( 0x80fe, 0x08);
ov5640_write( 0x80ff, 0xa6);
ov5640_write( 0x8100, 0x07);
ov5640_write( 0x8101, 0x78);
ov5640_write( 0x8102, 0xb3);
ov5640_write( 0x8103, 0xe6);
ov5640_write( 0x8104, 0xfe);
ov5640_write( 0x8105, 0x08);
ov5640_write( 0x8106, 0xe6);
ov5640_write( 0x8107, 0xc3);
ov5640_write( 0x8108, 0x78);
ov5640_write( 0x8109, 0xbf);
ov5640_write( 0x810a, 0x96);
ov5640_write( 0x810b, 0xff);
ov5640_write( 0x810c, 0xee);
ov5640_write( 0x810d, 0x18);
ov5640_write( 0x810e, 0x96);
ov5640_write( 0x810f, 0x78);
ov5640_write( 0x8110, 0xc0);
ov5640_write( 0x8111, 0xf6);
ov5640_write( 0x8112, 0x08);
ov5640_write( 0x8113, 0xa6);
ov5640_write( 0x8114, 0x07);
ov5640_write( 0x8115, 0x90);
ov5640_write( 0x8116, 0x0e);
ov5640_write( 0x8117, 0x8a);
ov5640_write( 0x8118, 0xe4);
ov5640_write( 0x8119, 0x18);
ov5640_write( 0x811a, 0x12);
ov5640_write( 0x811b, 0x0a);
ov5640_write( 0x811c, 0x9f);
ov5640_write( 0x811d, 0x40);
ov5640_write( 0x811e, 0x02);
ov5640_write( 0x811f, 0xd2);
ov5640_write( 0x8120, 0x37);
ov5640_write( 0x8121, 0x78);
ov5640_write( 0x8122, 0xb9);
ov5640_write( 0x8123, 0xe6);
ov5640_write( 0x8124, 0x08);
ov5640_write( 0x8125, 0x26);
ov5640_write( 0x8126, 0x08);
ov5640_write( 0x8127, 0xf6);
ov5640_write( 0x8128, 0xe5);
ov5640_write( 0x8129, 0x1f);
ov5640_write( 0x812a, 0x64);
ov5640_write( 0x812b, 0x01);
ov5640_write( 0x812c, 0x70);
ov5640_write( 0x812d, 0x47);
ov5640_write( 0x812e, 0xe6);
ov5640_write( 0x812f, 0xc3);
ov5640_write( 0x8130, 0x78);
ov5640_write( 0x8131, 0xbd);
ov5640_write( 0x8132, 0x12);
ov5640_write( 0x8133, 0x0a);
ov5640_write( 0x8134, 0x95);
ov5640_write( 0x8135, 0x40);
ov5640_write( 0x8136, 0x05);
ov5640_write( 0x8137, 0x12);
ov5640_write( 0x8138, 0x0a);
ov5640_write( 0x8139, 0x90);
ov5640_write( 0x813a, 0x40);
ov5640_write( 0x813b, 0x36);
ov5640_write( 0x813c, 0x78);
ov5640_write( 0x813d, 0xba);
ov5640_write( 0x813e, 0x76);
ov5640_write( 0x813f, 0xfd);
ov5640_write( 0x8140, 0x78);
ov5640_write( 0x8141, 0xb6);
ov5640_write( 0x8142, 0xe6);
ov5640_write( 0x8143, 0x24);
ov5640_write( 0x8144, 0x04);
ov5640_write( 0x8145, 0x78);
ov5640_write( 0x8146, 0xbc);
ov5640_write( 0x8147, 0xf6);
ov5640_write( 0x8148, 0xc3);
ov5640_write( 0x8149, 0xe6);
ov5640_write( 0x814a, 0x64);
ov5640_write( 0x814b, 0x80);
ov5640_write( 0x814c, 0x94);
ov5640_write( 0x814d, 0x89);
ov5640_write( 0x814e, 0x50);
ov5640_write( 0x814f, 0x08);
ov5640_write( 0x8150, 0xe4);
ov5640_write( 0x8151, 0x08);
ov5640_write( 0x8152, 0xf6);
ov5640_write( 0x8153, 0x18);
ov5640_write( 0x8154, 0x76);
ov5640_write( 0x8155, 0x09);
ov5640_write( 0x8156, 0x80);
ov5640_write( 0x8157, 0x07);
ov5640_write( 0x8158, 0x78);
ov5640_write( 0x8159, 0xbc);
ov5640_write( 0x815a, 0xe6);
ov5640_write( 0x815b, 0x24);
ov5640_write( 0x815c, 0xf7);
ov5640_write( 0x815d, 0x08);
ov5640_write( 0x815e, 0xf6);
ov5640_write( 0x815f, 0x78);
ov5640_write( 0x8160, 0xbc);
ov5640_write( 0x8161, 0xe6);
ov5640_write( 0x8162, 0x18);
ov5640_write( 0x8163, 0xf6);
ov5640_write( 0x8164, 0x75);
ov5640_write( 0x8165, 0x1f);
ov5640_write( 0x8166, 0x02);
ov5640_write( 0x8167, 0x78);
ov5640_write( 0x8168, 0xb5);
ov5640_write( 0x8169, 0x76);
ov5640_write( 0x816a, 0x01);
ov5640_write( 0x816b, 0x90);
ov5640_write( 0x816c, 0x0e);
ov5640_write( 0x816d, 0x77);
ov5640_write( 0x816e, 0xe4);
ov5640_write( 0x816f, 0x93);
ov5640_write( 0x8170, 0xf5);
ov5640_write( 0x8171, 0x48);
ov5640_write( 0x8172, 0x02);
ov5640_write( 0x8173, 0x02);
ov5640_write( 0x8174, 0x3d);
ov5640_write( 0x8175, 0xe5);
ov5640_write( 0x8176, 0x1f);
ov5640_write( 0x8177, 0x64);
ov5640_write( 0x8178, 0x02);
ov5640_write( 0x8179, 0x60);
ov5640_write( 0x817a, 0x03);
ov5640_write( 0x817b, 0x02);
ov5640_write( 0x817c, 0x02);
ov5640_write( 0x817d, 0x1d);
ov5640_write( 0x817e, 0x90);
ov5640_write( 0x817f, 0x0e);
ov5640_write( 0x8180, 0x78);
ov5640_write( 0x8181, 0x93);
ov5640_write( 0x8182, 0xf5);
ov5640_write( 0x8183, 0x48);
ov5640_write( 0x8184, 0x78);
ov5640_write( 0x8185, 0xbb);
ov5640_write( 0x8186, 0xe6);
ov5640_write( 0x8187, 0xff);
ov5640_write( 0x8188, 0xc3);
ov5640_write( 0x8189, 0x78);
ov5640_write( 0x818a, 0xbd);
ov5640_write( 0x818b, 0x12);
ov5640_write( 0x818c, 0x0a);
ov5640_write( 0x818d, 0x96);
ov5640_write( 0x818e, 0x40);
ov5640_write( 0x818f, 0x08);
ov5640_write( 0x8190, 0x12);
ov5640_write( 0x8191, 0x0a);
ov5640_write( 0x8192, 0x90);
ov5640_write( 0x8193, 0x50);
ov5640_write( 0x8194, 0x03);
ov5640_write( 0x8195, 0x02);
ov5640_write( 0x8196, 0x02);
ov5640_write( 0x8197, 0x1b);
ov5640_write( 0x8198, 0x78);
ov5640_write( 0x8199, 0xba);
ov5640_write( 0x819a, 0x76);
ov5640_write( 0x819b, 0x01);
ov5640_write( 0x819c, 0x78);
ov5640_write( 0x819d, 0xb6);
ov5640_write( 0x819e, 0xe6);
ov5640_write( 0x819f, 0x78);
ov5640_write( 0x81a0, 0xba);
ov5640_write( 0x81a1, 0x26);
ov5640_write( 0x81a2, 0x78);
ov5640_write( 0x81a3, 0xbc);
ov5640_write( 0x81a4, 0xf6);
ov5640_write( 0x81a5, 0x78);
ov5640_write( 0x81a6, 0xb6);
ov5640_write( 0x81a7, 0xe6);
ov5640_write( 0x81a8, 0xc3);
ov5640_write( 0x81a9, 0x78);
ov5640_write( 0x81aa, 0xba);
ov5640_write( 0x81ab, 0x96);
ov5640_write( 0x81ac, 0x78);
ov5640_write( 0x81ad, 0xbd);
ov5640_write( 0x81ae, 0xf6);
ov5640_write( 0x81af, 0x18);
ov5640_write( 0x81b0, 0x12);
ov5640_write( 0x81b1, 0x0a);
ov5640_write( 0x81b2, 0xc3);
ov5640_write( 0x81b3, 0x40);
ov5640_write( 0x81b4, 0x04);
ov5640_write( 0x81b5, 0xe6);
ov5640_write( 0x81b6, 0xff);
ov5640_write( 0x81b7, 0x80);
ov5640_write( 0x81b8, 0x02);
ov5640_write( 0x81b9, 0x7f);
ov5640_write( 0x81ba, 0x00);
ov5640_write( 0x81bb, 0x78);
ov5640_write( 0x81bc, 0xbc);
ov5640_write( 0x81bd, 0xa6);
ov5640_write( 0x81be, 0x07);
ov5640_write( 0x81bf, 0x08);
ov5640_write( 0x81c0, 0x12);
ov5640_write( 0x81c1, 0x0a);
ov5640_write( 0x81c2, 0xc3);
ov5640_write( 0x81c3, 0x40);
ov5640_write( 0x81c4, 0x04);
ov5640_write( 0x81c5, 0xe6);
ov5640_write( 0x81c6, 0xff);
ov5640_write( 0x81c7, 0x80);
ov5640_write( 0x81c8, 0x02);
ov5640_write( 0x81c9, 0x7f);
ov5640_write( 0x81ca, 0x00);
ov5640_write( 0x81cb, 0x78);
ov5640_write( 0x81cc, 0xbd);
ov5640_write( 0x81cd, 0xa6);
ov5640_write( 0x81ce, 0x07);
ov5640_write( 0x81cf, 0xc3);
ov5640_write( 0x81d0, 0x18);
ov5640_write( 0x81d1, 0xe6);
ov5640_write( 0x81d2, 0x64);
ov5640_write( 0x81d3, 0x80);
ov5640_write( 0x81d4, 0x94);
ov5640_write( 0x81d5, 0xb3);
ov5640_write( 0x81d6, 0x50);
ov5640_write( 0x81d7, 0x04);
ov5640_write( 0x81d8, 0xe6);
ov5640_write( 0x81d9, 0xff);
ov5640_write( 0x81da, 0x80);
ov5640_write( 0x81db, 0x02);
ov5640_write( 0x81dc, 0x7f);
ov5640_write( 0x81dd, 0x33);
ov5640_write( 0x81de, 0x78);
ov5640_write( 0x81df, 0xbc);
ov5640_write( 0x81e0, 0xa6);
ov5640_write( 0x81e1, 0x07);
ov5640_write( 0x81e2, 0xc3);
ov5640_write( 0x81e3, 0x08);
ov5640_write( 0x81e4, 0xe6);
ov5640_write( 0x81e5, 0x64);
ov5640_write( 0x81e6, 0x80);
ov5640_write( 0x81e7, 0x94);
ov5640_write( 0x81e8, 0xb3);
ov5640_write( 0x81e9, 0x50);
ov5640_write( 0x81ea, 0x04);
ov5640_write( 0x81eb, 0xe6);
ov5640_write( 0x81ec, 0xff);
ov5640_write( 0x81ed, 0x80);
ov5640_write( 0x81ee, 0x02);
ov5640_write( 0x81ef, 0x7f);
ov5640_write( 0x81f0, 0x33);
ov5640_write( 0x81f1, 0x78);
ov5640_write( 0x81f2, 0xbd);
ov5640_write( 0x81f3, 0xa6);
ov5640_write( 0x81f4, 0x07);
ov5640_write( 0x81f5, 0xd3);
ov5640_write( 0x81f6, 0x78);
ov5640_write( 0x81f7, 0xba);
ov5640_write( 0x81f8, 0xe6);
ov5640_write( 0x81f9, 0x64);
ov5640_write( 0x81fa, 0x80);
ov5640_write( 0x81fb, 0x94);
ov5640_write( 0x81fc, 0x80);
ov5640_write( 0x81fd, 0x40);
ov5640_write( 0x81fe, 0x06);
ov5640_write( 0x81ff, 0x78);
ov5640_write( 0x8200, 0xbd);
ov5640_write( 0x8201, 0xe6);
ov5640_write( 0x8202, 0xff);
ov5640_write( 0x8203, 0x80);
ov5640_write( 0x8204, 0x04);
ov5640_write( 0x8205, 0x78);
ov5640_write( 0x8206, 0xbc);
ov5640_write( 0x8207, 0xe6);
ov5640_write( 0x8208, 0xff);
ov5640_write( 0x8209, 0x78);
ov5640_write( 0x820a, 0xbb);
ov5640_write( 0x820b, 0xa6);
ov5640_write( 0x820c, 0x07);
ov5640_write( 0x820d, 0x75);
ov5640_write( 0x820e, 0x1f);
ov5640_write( 0x820f, 0x03);
ov5640_write( 0x8210, 0x90);
ov5640_write( 0x8211, 0x0e);
ov5640_write( 0x8212, 0x79);
ov5640_write( 0x8213, 0xe4);
ov5640_write( 0x8214, 0x93);
ov5640_write( 0x8215, 0xf5);
ov5640_write( 0x8216, 0x48);
ov5640_write( 0x8217, 0xe4);
ov5640_write( 0x8218, 0x78);
ov5640_write( 0x8219, 0xb5);
ov5640_write( 0x821a, 0xf6);
ov5640_write( 0x821b, 0x80);
ov5640_write( 0x821c, 0x20);
ov5640_write( 0x821d, 0xe5);
ov5640_write( 0x821e, 0x1f);
ov5640_write( 0x821f, 0x64);
ov5640_write( 0x8220, 0x03);
ov5640_write( 0x8221, 0x70);
ov5640_write( 0x8222, 0x26);
ov5640_write( 0x8223, 0x78);
ov5640_write( 0x8224, 0xbb);
ov5640_write( 0x8225, 0xe6);
ov5640_write( 0x8226, 0xff);
ov5640_write( 0x8227, 0xc3);
ov5640_write( 0x8228, 0x78);
ov5640_write( 0x8229, 0xbd);
ov5640_write( 0x822a, 0x12);
ov5640_write( 0x822b, 0x0a);
ov5640_write( 0x822c, 0x96);
ov5640_write( 0x822d, 0x40);
ov5640_write( 0x822e, 0x05);
ov5640_write( 0x822f, 0x12);
ov5640_write( 0x8230, 0x0a);
ov5640_write( 0x8231, 0x90);
ov5640_write( 0x8232, 0x40);
ov5640_write( 0x8233, 0x09);
ov5640_write( 0x8234, 0x78);
ov5640_write( 0x8235, 0xb6);
ov5640_write( 0x8236, 0xe6);
ov5640_write( 0x8237, 0x78);
ov5640_write( 0x8238, 0xbb);
ov5640_write( 0x8239, 0xf6);
ov5640_write( 0x823a, 0x75);
ov5640_write( 0x823b, 0x1f);
ov5640_write( 0x823c, 0x04);
ov5640_write( 0x823d, 0x78);
ov5640_write( 0x823e, 0xbb);
ov5640_write( 0x823f, 0xe6);
ov5640_write( 0x8240, 0x75);
ov5640_write( 0x8241, 0xf0);
ov5640_write( 0x8242, 0x05);
ov5640_write( 0x8243, 0xa4);
ov5640_write( 0x8244, 0xf5);
ov5640_write( 0x8245, 0x49);
ov5640_write( 0x8246, 0x02);
ov5640_write( 0x8247, 0x08);
ov5640_write( 0x8248, 0x61);
ov5640_write( 0x8249, 0xe5);
ov5640_write( 0x824a, 0x1f);
ov5640_write( 0x824b, 0xb4);
ov5640_write( 0x824c, 0x04);
ov5640_write( 0x824d, 0x10);
ov5640_write( 0x824e, 0x90);
ov5640_write( 0x824f, 0x0e);
ov5640_write( 0x8250, 0x89);
ov5640_write( 0x8251, 0xe4);
ov5640_write( 0x8252, 0x78);
ov5640_write( 0x8253, 0xc0);
ov5640_write( 0x8254, 0x12);
ov5640_write( 0x8255, 0x0a);
ov5640_write( 0x8256, 0x9f);
ov5640_write( 0x8257, 0x40);
ov5640_write( 0x8258, 0x02);
ov5640_write( 0x8259, 0xd2);
ov5640_write( 0x825a, 0x37);
ov5640_write( 0x825b, 0x75);
ov5640_write( 0x825c, 0x1f);
ov5640_write( 0x825d, 0x05);
ov5640_write( 0x825e, 0x22);
ov5640_write( 0x825f, 0xef);
ov5640_write( 0x8260, 0x8d);
ov5640_write( 0x8261, 0xf0);
ov5640_write( 0x8262, 0xa4);
ov5640_write( 0x8263, 0xa8);
ov5640_write( 0x8264, 0xf0);
ov5640_write( 0x8265, 0xcf);
ov5640_write( 0x8266, 0x8c);
ov5640_write( 0x8267, 0xf0);
ov5640_write( 0x8268, 0xa4);
ov5640_write( 0x8269, 0x28);
ov5640_write( 0x826a, 0xce);
ov5640_write( 0x826b, 0x8d);
ov5640_write( 0x826c, 0xf0);
ov5640_write( 0x826d, 0xa4);
ov5640_write( 0x826e, 0x2e);
ov5640_write( 0x826f, 0xfe);
ov5640_write( 0x8270, 0x22);
ov5640_write( 0x8271, 0xbc);
ov5640_write( 0x8272, 0x00);
ov5640_write( 0x8273, 0x0b);
ov5640_write( 0x8274, 0xbe);
ov5640_write( 0x8275, 0x00);
ov5640_write( 0x8276, 0x29);
ov5640_write( 0x8277, 0xef);
ov5640_write( 0x8278, 0x8d);
ov5640_write( 0x8279, 0xf0);
ov5640_write( 0x827a, 0x84);
ov5640_write( 0x827b, 0xff);
ov5640_write( 0x827c, 0xad);
ov5640_write( 0x827d, 0xf0);
ov5640_write( 0x827e, 0x22);
ov5640_write( 0x827f, 0xe4);
ov5640_write( 0x8280, 0xcc);
ov5640_write( 0x8281, 0xf8);
ov5640_write( 0x8282, 0x75);
ov5640_write( 0x8283, 0xf0);
ov5640_write( 0x8284, 0x08);
ov5640_write( 0x8285, 0xef);
ov5640_write( 0x8286, 0x2f);
ov5640_write( 0x8287, 0xff);
ov5640_write( 0x8288, 0xee);
ov5640_write( 0x8289, 0x33);
ov5640_write( 0x828a, 0xfe);
ov5640_write( 0x828b, 0xec);
ov5640_write( 0x828c, 0x33);
ov5640_write( 0x828d, 0xfc);
ov5640_write( 0x828e, 0xee);
ov5640_write( 0x828f, 0x9d);
ov5640_write( 0x8290, 0xec);
ov5640_write( 0x8291, 0x98);
ov5640_write( 0x8292, 0x40);
ov5640_write( 0x8293, 0x05);
ov5640_write( 0x8294, 0xfc);
ov5640_write( 0x8295, 0xee);
ov5640_write( 0x8296, 0x9d);
ov5640_write( 0x8297, 0xfe);
ov5640_write( 0x8298, 0x0f);
ov5640_write( 0x8299, 0xd5);
ov5640_write( 0x829a, 0xf0);
ov5640_write( 0x829b, 0xe9);
ov5640_write( 0x829c, 0xe4);
ov5640_write( 0x829d, 0xce);
ov5640_write( 0x829e, 0xfd);
ov5640_write( 0x829f, 0x22);
ov5640_write( 0x82a0, 0xed);
ov5640_write( 0x82a1, 0xf8);
ov5640_write( 0x82a2, 0xf5);
ov5640_write( 0x82a3, 0xf0);
ov5640_write( 0x82a4, 0xee);
ov5640_write( 0x82a5, 0x84);
ov5640_write( 0x82a6, 0x20);
ov5640_write( 0x82a7, 0xd2);
ov5640_write( 0x82a8, 0x1c);
ov5640_write( 0x82a9, 0xfe);
ov5640_write( 0x82aa, 0xad);
ov5640_write( 0x82ab, 0xf0);
ov5640_write( 0x82ac, 0x75);
ov5640_write( 0x82ad, 0xf0);
ov5640_write( 0x82ae, 0x08);
ov5640_write( 0x82af, 0xef);
ov5640_write( 0x82b0, 0x2f);
ov5640_write( 0x82b1, 0xff);
ov5640_write( 0x82b2, 0xed);
ov5640_write( 0x82b3, 0x33);
ov5640_write( 0x82b4, 0xfd);
ov5640_write( 0x82b5, 0x40);
ov5640_write( 0x82b6, 0x07);
ov5640_write( 0x82b7, 0x98);
ov5640_write( 0x82b8, 0x50);
ov5640_write( 0x82b9, 0x06);
ov5640_write( 0x82ba, 0xd5);
ov5640_write( 0x82bb, 0xf0);
ov5640_write( 0x82bc, 0xf2);
ov5640_write( 0x82bd, 0x22);
ov5640_write( 0x82be, 0xc3);
ov5640_write( 0x82bf, 0x98);
ov5640_write( 0x82c0, 0xfd);
ov5640_write( 0x82c1, 0x0f);
ov5640_write( 0x82c2, 0xd5);
ov5640_write( 0x82c3, 0xf0);
ov5640_write( 0x82c4, 0xea);
ov5640_write( 0x82c5, 0x22);
ov5640_write( 0x82c6, 0xe8);
ov5640_write( 0x82c7, 0x8f);
ov5640_write( 0x82c8, 0xf0);
ov5640_write( 0x82c9, 0xa4);
ov5640_write( 0x82ca, 0xcc);
ov5640_write( 0x82cb, 0x8b);
ov5640_write( 0x82cc, 0xf0);
ov5640_write( 0x82cd, 0xa4);
ov5640_write( 0x82ce, 0x2c);
ov5640_write( 0x82cf, 0xfc);
ov5640_write( 0x82d0, 0xe9);
ov5640_write( 0x82d1, 0x8e);
ov5640_write( 0x82d2, 0xf0);
ov5640_write( 0x82d3, 0xa4);
ov5640_write( 0x82d4, 0x2c);
ov5640_write( 0x82d5, 0xfc);
ov5640_write( 0x82d6, 0x8a);
ov5640_write( 0x82d7, 0xf0);
ov5640_write( 0x82d8, 0xed);
ov5640_write( 0x82d9, 0xa4);
ov5640_write( 0x82da, 0x2c);
ov5640_write( 0x82db, 0xfc);
ov5640_write( 0x82dc, 0xea);
ov5640_write( 0x82dd, 0x8e);
ov5640_write( 0x82de, 0xf0);
ov5640_write( 0x82df, 0xa4);
ov5640_write( 0x82e0, 0xcd);
ov5640_write( 0x82e1, 0xa8);
ov5640_write( 0x82e2, 0xf0);
ov5640_write( 0x82e3, 0x8b);
ov5640_write( 0x82e4, 0xf0);
ov5640_write( 0x82e5, 0xa4);
ov5640_write( 0x82e6, 0x2d);
ov5640_write( 0x82e7, 0xcc);
ov5640_write( 0x82e8, 0x38);
ov5640_write( 0x82e9, 0x25);
ov5640_write( 0x82ea, 0xf0);
ov5640_write( 0x82eb, 0xfd);
ov5640_write( 0x82ec, 0xe9);
ov5640_write( 0x82ed, 0x8f);
ov5640_write( 0x82ee, 0xf0);
ov5640_write( 0x82ef, 0xa4);
ov5640_write( 0x82f0, 0x2c);
ov5640_write( 0x82f1, 0xcd);
ov5640_write( 0x82f2, 0x35);
ov5640_write( 0x82f3, 0xf0);
ov5640_write( 0x82f4, 0xfc);
ov5640_write( 0x82f5, 0xeb);
ov5640_write( 0x82f6, 0x8e);
ov5640_write( 0x82f7, 0xf0);
ov5640_write( 0x82f8, 0xa4);
ov5640_write( 0x82f9, 0xfe);
ov5640_write( 0x82fa, 0xa9);
ov5640_write( 0x82fb, 0xf0);
ov5640_write( 0x82fc, 0xeb);
ov5640_write( 0x82fd, 0x8f);
ov5640_write( 0x82fe, 0xf0);
ov5640_write( 0x82ff, 0xa4);
ov5640_write( 0x8300, 0xcf);
ov5640_write( 0x8301, 0xc5);
ov5640_write( 0x8302, 0xf0);
ov5640_write( 0x8303, 0x2e);
ov5640_write( 0x8304, 0xcd);
ov5640_write( 0x8305, 0x39);
ov5640_write( 0x8306, 0xfe);
ov5640_write( 0x8307, 0xe4);
ov5640_write( 0x8308, 0x3c);
ov5640_write( 0x8309, 0xfc);
ov5640_write( 0x830a, 0xea);
ov5640_write( 0x830b, 0xa4);
ov5640_write( 0x830c, 0x2d);
ov5640_write( 0x830d, 0xce);
ov5640_write( 0x830e, 0x35);
ov5640_write( 0x830f, 0xf0);
ov5640_write( 0x8310, 0xfd);
ov5640_write( 0x8311, 0xe4);
ov5640_write( 0x8312, 0x3c);
ov5640_write( 0x8313, 0xfc);
ov5640_write( 0x8314, 0x22);
ov5640_write( 0x8315, 0x75);
ov5640_write( 0x8316, 0xf0);
ov5640_write( 0x8317, 0x08);
ov5640_write( 0x8318, 0x75);
ov5640_write( 0x8319, 0x82);
ov5640_write( 0x831a, 0x00);
ov5640_write( 0x831b, 0xef);
ov5640_write( 0x831c, 0x2f);
ov5640_write( 0x831d, 0xff);
ov5640_write( 0x831e, 0xee);
ov5640_write( 0x831f, 0x33);
ov5640_write( 0x8320, 0xfe);
ov5640_write( 0x8321, 0xcd);
ov5640_write( 0x8322, 0x33);
ov5640_write( 0x8323, 0xcd);
ov5640_write( 0x8324, 0xcc);
ov5640_write( 0x8325, 0x33);
ov5640_write( 0x8326, 0xcc);
ov5640_write( 0x8327, 0xc5);
ov5640_write( 0x8328, 0x82);
ov5640_write( 0x8329, 0x33);
ov5640_write( 0x832a, 0xc5);
ov5640_write( 0x832b, 0x82);
ov5640_write( 0x832c, 0x9b);
ov5640_write( 0x832d, 0xed);
ov5640_write( 0x832e, 0x9a);
ov5640_write( 0x832f, 0xec);
ov5640_write( 0x8330, 0x99);
ov5640_write( 0x8331, 0xe5);
ov5640_write( 0x8332, 0x82);
ov5640_write( 0x8333, 0x98);
ov5640_write( 0x8334, 0x40);
ov5640_write( 0x8335, 0x0c);
ov5640_write( 0x8336, 0xf5);
ov5640_write( 0x8337, 0x82);
ov5640_write( 0x8338, 0xee);
ov5640_write( 0x8339, 0x9b);
ov5640_write( 0x833a, 0xfe);
ov5640_write( 0x833b, 0xed);
ov5640_write( 0x833c, 0x9a);
ov5640_write( 0x833d, 0xfd);
ov5640_write( 0x833e, 0xec);
ov5640_write( 0x833f, 0x99);
ov5640_write( 0x8340, 0xfc);
ov5640_write( 0x8341, 0x0f);
ov5640_write( 0x8342, 0xd5);
ov5640_write( 0x8343, 0xf0);
ov5640_write( 0x8344, 0xd6);
ov5640_write( 0x8345, 0xe4);
ov5640_write( 0x8346, 0xce);
ov5640_write( 0x8347, 0xfb);
ov5640_write( 0x8348, 0xe4);
ov5640_write( 0x8349, 0xcd);
ov5640_write( 0x834a, 0xfa);
ov5640_write( 0x834b, 0xe4);
ov5640_write( 0x834c, 0xcc);
ov5640_write( 0x834d, 0xf9);
ov5640_write( 0x834e, 0xa8);
ov5640_write( 0x834f, 0x82);
ov5640_write( 0x8350, 0x22);
ov5640_write( 0x8351, 0xb8);
ov5640_write( 0x8352, 0x00);
ov5640_write( 0x8353, 0xc1);
ov5640_write( 0x8354, 0xb9);
ov5640_write( 0x8355, 0x00);
ov5640_write( 0x8356, 0x59);
ov5640_write( 0x8357, 0xba);
ov5640_write( 0x8358, 0x00);
ov5640_write( 0x8359, 0x2d);
ov5640_write( 0x835a, 0xec);
ov5640_write( 0x835b, 0x8b);
ov5640_write( 0x835c, 0xf0);
ov5640_write( 0x835d, 0x84);
ov5640_write( 0x835e, 0xcf);
ov5640_write( 0x835f, 0xce);
ov5640_write( 0x8360, 0xcd);
ov5640_write( 0x8361, 0xfc);
ov5640_write( 0x8362, 0xe5);
ov5640_write( 0x8363, 0xf0);
ov5640_write( 0x8364, 0xcb);
ov5640_write( 0x8365, 0xf9);
ov5640_write( 0x8366, 0x78);
ov5640_write( 0x8367, 0x18);
ov5640_write( 0x8368, 0xef);
ov5640_write( 0x8369, 0x2f);
ov5640_write( 0x836a, 0xff);
ov5640_write( 0x836b, 0xee);
ov5640_write( 0x836c, 0x33);
ov5640_write( 0x836d, 0xfe);
ov5640_write( 0x836e, 0xed);
ov5640_write( 0x836f, 0x33);
ov5640_write( 0x8370, 0xfd);
ov5640_write( 0x8371, 0xec);
ov5640_write( 0x8372, 0x33);
ov5640_write( 0x8373, 0xfc);
ov5640_write( 0x8374, 0xeb);
ov5640_write( 0x8375, 0x33);
ov5640_write( 0x8376, 0xfb);
ov5640_write( 0x8377, 0x10);
ov5640_write( 0x8378, 0xd7);
ov5640_write( 0x8379, 0x03);
ov5640_write( 0x837a, 0x99);
ov5640_write( 0x837b, 0x40);
ov5640_write( 0x837c, 0x04);
ov5640_write( 0x837d, 0xeb);
ov5640_write( 0x837e, 0x99);
ov5640_write( 0x837f, 0xfb);
ov5640_write( 0x8380, 0x0f);
ov5640_write( 0x8381, 0xd8);
ov5640_write( 0x8382, 0xe5);
ov5640_write( 0x8383, 0xe4);
ov5640_write( 0x8384, 0xf9);
ov5640_write( 0x8385, 0xfa);
ov5640_write( 0x8386, 0x22);
ov5640_write( 0x8387, 0x78);
ov5640_write( 0x8388, 0x18);
ov5640_write( 0x8389, 0xef);
ov5640_write( 0x838a, 0x2f);
ov5640_write( 0x838b, 0xff);
ov5640_write( 0x838c, 0xee);
ov5640_write( 0x838d, 0x33);
ov5640_write( 0x838e, 0xfe);
ov5640_write( 0x838f, 0xed);
ov5640_write( 0x8390, 0x33);
ov5640_write( 0x8391, 0xfd);
ov5640_write( 0x8392, 0xec);
ov5640_write( 0x8393, 0x33);
ov5640_write( 0x8394, 0xfc);
ov5640_write( 0x8395, 0xc9);
ov5640_write( 0x8396, 0x33);
ov5640_write( 0x8397, 0xc9);
ov5640_write( 0x8398, 0x10);
ov5640_write( 0x8399, 0xd7);
ov5640_write( 0x839a, 0x05);
ov5640_write( 0x839b, 0x9b);
ov5640_write( 0x839c, 0xe9);
ov5640_write( 0x839d, 0x9a);
ov5640_write( 0x839e, 0x40);
ov5640_write( 0x839f, 0x07);
ov5640_write( 0x83a0, 0xec);
ov5640_write( 0x83a1, 0x9b);
ov5640_write( 0x83a2, 0xfc);
ov5640_write( 0x83a3, 0xe9);
ov5640_write( 0x83a4, 0x9a);
ov5640_write( 0x83a5, 0xf9);
ov5640_write( 0x83a6, 0x0f);
ov5640_write( 0x83a7, 0xd8);
ov5640_write( 0x83a8, 0xe0);
ov5640_write( 0x83a9, 0xe4);
ov5640_write( 0x83aa, 0xc9);
ov5640_write( 0x83ab, 0xfa);
ov5640_write( 0x83ac, 0xe4);
ov5640_write( 0x83ad, 0xcc);
ov5640_write( 0x83ae, 0xfb);
ov5640_write( 0x83af, 0x22);
ov5640_write( 0x83b0, 0x75);
ov5640_write( 0x83b1, 0xf0);
ov5640_write( 0x83b2, 0x10);
ov5640_write( 0x83b3, 0xef);
ov5640_write( 0x83b4, 0x2f);
ov5640_write( 0x83b5, 0xff);
ov5640_write( 0x83b6, 0xee);
ov5640_write( 0x83b7, 0x33);
ov5640_write( 0x83b8, 0xfe);
ov5640_write( 0x83b9, 0xed);
ov5640_write( 0x83ba, 0x33);
ov5640_write( 0x83bb, 0xfd);
ov5640_write( 0x83bc, 0xcc);
ov5640_write( 0x83bd, 0x33);
ov5640_write( 0x83be, 0xcc);
ov5640_write( 0x83bf, 0xc8);
ov5640_write( 0x83c0, 0x33);
ov5640_write( 0x83c1, 0xc8);
ov5640_write( 0x83c2, 0x10);
ov5640_write( 0x83c3, 0xd7);
ov5640_write( 0x83c4, 0x07);
ov5640_write( 0x83c5, 0x9b);
ov5640_write( 0x83c6, 0xec);
ov5640_write( 0x83c7, 0x9a);
ov5640_write( 0x83c8, 0xe8);
ov5640_write( 0x83c9, 0x99);
ov5640_write( 0x83ca, 0x40);
ov5640_write( 0x83cb, 0x0a);
ov5640_write( 0x83cc, 0xed);
ov5640_write( 0x83cd, 0x9b);
ov5640_write( 0x83ce, 0xfd);
ov5640_write( 0x83cf, 0xec);
ov5640_write( 0x83d0, 0x9a);
ov5640_write( 0x83d1, 0xfc);
ov5640_write( 0x83d2, 0xe8);
ov5640_write( 0x83d3, 0x99);
ov5640_write( 0x83d4, 0xf8);
ov5640_write( 0x83d5, 0x0f);
ov5640_write( 0x83d6, 0xd5);
ov5640_write( 0x83d7, 0xf0);
ov5640_write( 0x83d8, 0xda);
ov5640_write( 0x83d9, 0xe4);
ov5640_write( 0x83da, 0xcd);
ov5640_write( 0x83db, 0xfb);
ov5640_write( 0x83dc, 0xe4);
ov5640_write( 0x83dd, 0xcc);
ov5640_write( 0x83de, 0xfa);
ov5640_write( 0x83df, 0xe4);
ov5640_write( 0x83e0, 0xc8);
ov5640_write( 0x83e1, 0xf9);
ov5640_write( 0x83e2, 0x22);
ov5640_write( 0x83e3, 0xeb);
ov5640_write( 0x83e4, 0x9f);
ov5640_write( 0x83e5, 0xf5);
ov5640_write( 0x83e6, 0xf0);
ov5640_write( 0x83e7, 0xea);
ov5640_write( 0x83e8, 0x9e);
ov5640_write( 0x83e9, 0x42);
ov5640_write( 0x83ea, 0xf0);
ov5640_write( 0x83eb, 0xe9);
ov5640_write( 0x83ec, 0x9d);
ov5640_write( 0x83ed, 0x42);
ov5640_write( 0x83ee, 0xf0);
ov5640_write( 0x83ef, 0xe8);
ov5640_write( 0x83f0, 0x9c);
ov5640_write( 0x83f1, 0x45);
ov5640_write( 0x83f2, 0xf0);
ov5640_write( 0x83f3, 0x22);
ov5640_write( 0x83f4, 0xe8);
ov5640_write( 0x83f5, 0x60);
ov5640_write( 0x83f6, 0x0f);
ov5640_write( 0x83f7, 0xef);
ov5640_write( 0x83f8, 0xc3);
ov5640_write( 0x83f9, 0x33);
ov5640_write( 0x83fa, 0xff);
ov5640_write( 0x83fb, 0xee);
ov5640_write( 0x83fc, 0x33);
ov5640_write( 0x83fd, 0xfe);
ov5640_write( 0x83fe, 0xed);
ov5640_write( 0x83ff, 0x33);
ov5640_write( 0x8400, 0xfd);
ov5640_write( 0x8401, 0xec);
ov5640_write( 0x8402, 0x33);
ov5640_write( 0x8403, 0xfc);
ov5640_write( 0x8404, 0xd8);
ov5640_write( 0x8405, 0xf1);
ov5640_write( 0x8406, 0x22);
ov5640_write( 0x8407, 0xe4);
ov5640_write( 0x8408, 0x93);
ov5640_write( 0x8409, 0xfc);
ov5640_write( 0x840a, 0x74);
ov5640_write( 0x840b, 0x01);
ov5640_write( 0x840c, 0x93);
ov5640_write( 0x840d, 0xfd);
ov5640_write( 0x840e, 0x74);
ov5640_write( 0x840f, 0x02);
ov5640_write( 0x8410, 0x93);
ov5640_write( 0x8411, 0xfe);
ov5640_write( 0x8412, 0x74);
ov5640_write( 0x8413, 0x03);
ov5640_write( 0x8414, 0x93);
ov5640_write( 0x8415, 0xff);
ov5640_write( 0x8416, 0x22);
ov5640_write( 0x8417, 0xe6);
ov5640_write( 0x8418, 0xfb);
ov5640_write( 0x8419, 0x08);
ov5640_write( 0x841a, 0xe6);
ov5640_write( 0x841b, 0xf9);
ov5640_write( 0x841c, 0x08);
ov5640_write( 0x841d, 0xe6);
ov5640_write( 0x841e, 0xfa);
ov5640_write( 0x841f, 0x08);
ov5640_write( 0x8420, 0xe6);
ov5640_write( 0x8421, 0xcb);
ov5640_write( 0x8422, 0xf8);
ov5640_write( 0x8423, 0x22);
ov5640_write( 0x8424, 0xec);
ov5640_write( 0x8425, 0xf6);
ov5640_write( 0x8426, 0x08);
ov5640_write( 0x8427, 0xed);
ov5640_write( 0x8428, 0xf6);
ov5640_write( 0x8429, 0x08);
ov5640_write( 0x842a, 0xee);
ov5640_write( 0x842b, 0xf6);
ov5640_write( 0x842c, 0x08);
ov5640_write( 0x842d, 0xef);
ov5640_write( 0x842e, 0xf6);
ov5640_write( 0x842f, 0x22);
ov5640_write( 0x8430, 0xa4);
ov5640_write( 0x8431, 0x25);
ov5640_write( 0x8432, 0x82);
ov5640_write( 0x8433, 0xf5);
ov5640_write( 0x8434, 0x82);
ov5640_write( 0x8435, 0xe5);
ov5640_write( 0x8436, 0xf0);
ov5640_write( 0x8437, 0x35);
ov5640_write( 0x8438, 0x83);
ov5640_write( 0x8439, 0xf5);
ov5640_write( 0x843a, 0x83);
ov5640_write( 0x843b, 0x22);
ov5640_write( 0x843c, 0xd0);
ov5640_write( 0x843d, 0x83);
ov5640_write( 0x843e, 0xd0);
ov5640_write( 0x843f, 0x82);
ov5640_write( 0x8440, 0xf8);
ov5640_write( 0x8441, 0xe4);
ov5640_write( 0x8442, 0x93);
ov5640_write( 0x8443, 0x70);
ov5640_write( 0x8444, 0x12);
ov5640_write( 0x8445, 0x74);
ov5640_write( 0x8446, 0x01);
ov5640_write( 0x8447, 0x93);
ov5640_write( 0x8448, 0x70);
ov5640_write( 0x8449, 0x0d);
ov5640_write( 0x844a, 0xa3);
ov5640_write( 0x844b, 0xa3);
ov5640_write( 0x844c, 0x93);
ov5640_write( 0x844d, 0xf8);
ov5640_write( 0x844e, 0x74);
ov5640_write( 0x844f, 0x01);
ov5640_write( 0x8450, 0x93);
ov5640_write( 0x8451, 0xf5);
ov5640_write( 0x8452, 0x82);
ov5640_write( 0x8453, 0x88);
ov5640_write( 0x8454, 0x83);
ov5640_write( 0x8455, 0xe4);
ov5640_write( 0x8456, 0x73);
ov5640_write( 0x8457, 0x74);
ov5640_write( 0x8458, 0x02);
ov5640_write( 0x8459, 0x93);
ov5640_write( 0x845a, 0x68);
ov5640_write( 0x845b, 0x60);
ov5640_write( 0x845c, 0xef);
ov5640_write( 0x845d, 0xa3);
ov5640_write( 0x845e, 0xa3);
ov5640_write( 0x845f, 0xa3);
ov5640_write( 0x8460, 0x80);
ov5640_write( 0x8461, 0xdf);
ov5640_write( 0x8462, 0x90);
ov5640_write( 0x8463, 0x38);
ov5640_write( 0x8464, 0x04);
ov5640_write( 0x8465, 0x78);
ov5640_write( 0x8466, 0x4f);
ov5640_write( 0x8467, 0x12);
ov5640_write( 0x8468, 0x09);
ov5640_write( 0x8469, 0x56);
ov5640_write( 0x846a, 0x90);
ov5640_write( 0x846b, 0x38);
ov5640_write( 0x846c, 0x00);
ov5640_write( 0x846d, 0xe0);
ov5640_write( 0x846e, 0xfe);
ov5640_write( 0x846f, 0xa3);
ov5640_write( 0x8470, 0xe0);
ov5640_write( 0x8471, 0xfd);
ov5640_write( 0x8472, 0xed);
ov5640_write( 0x8473, 0xff);
ov5640_write( 0x8474, 0xc3);
ov5640_write( 0x8475, 0x12);
ov5640_write( 0x8476, 0x09);
ov5640_write( 0x8477, 0x0f);
ov5640_write( 0x8478, 0x90);
ov5640_write( 0x8479, 0x38);
ov5640_write( 0x847a, 0x10);
ov5640_write( 0x847b, 0x12);
ov5640_write( 0x847c, 0x09);
ov5640_write( 0x847d, 0x03);
ov5640_write( 0x847e, 0x90);
ov5640_write( 0x847f, 0x38);
ov5640_write( 0x8480, 0x06);
ov5640_write( 0x8481, 0x78);
ov5640_write( 0x8482, 0x51);
ov5640_write( 0x8483, 0x12);
ov5640_write( 0x8484, 0x09);
ov5640_write( 0x8485, 0x56);
ov5640_write( 0x8486, 0x90);
ov5640_write( 0x8487, 0x38);
ov5640_write( 0x8488, 0x02);
ov5640_write( 0x8489, 0xe0);
ov5640_write( 0x848a, 0xfe);
ov5640_write( 0x848b, 0xa3);
ov5640_write( 0x848c, 0xe0);
ov5640_write( 0x848d, 0xfd);
ov5640_write( 0x848e, 0xed);
ov5640_write( 0x848f, 0xff);
ov5640_write( 0x8490, 0xc3);
ov5640_write( 0x8491, 0x12);
ov5640_write( 0x8492, 0x09);
ov5640_write( 0x8493, 0x0f);
ov5640_write( 0x8494, 0x90);
ov5640_write( 0x8495, 0x38);
ov5640_write( 0x8496, 0x12);
ov5640_write( 0x8497, 0x12);
ov5640_write( 0x8498, 0x09);
ov5640_write( 0x8499, 0x03);
ov5640_write( 0x849a, 0xa3);
ov5640_write( 0x849b, 0xe0);
ov5640_write( 0x849c, 0xb4);
ov5640_write( 0x849d, 0x31);
ov5640_write( 0x849e, 0x07);
ov5640_write( 0x849f, 0x78);
ov5640_write( 0x84a0, 0x4f);
ov5640_write( 0x84a1, 0x79);
ov5640_write( 0x84a2, 0x4f);
ov5640_write( 0x84a3, 0x12);
ov5640_write( 0x84a4, 0x09);
ov5640_write( 0x84a5, 0x6c);
ov5640_write( 0x84a6, 0x90);
ov5640_write( 0x84a7, 0x38);
ov5640_write( 0x84a8, 0x14);
ov5640_write( 0x84a9, 0xe0);
ov5640_write( 0x84aa, 0xb4);
ov5640_write( 0x84ab, 0x71);
ov5640_write( 0x84ac, 0x15);
ov5640_write( 0x84ad, 0x78);
ov5640_write( 0x84ae, 0x4f);
ov5640_write( 0x84af, 0xe6);
ov5640_write( 0x84b0, 0xfe);
ov5640_write( 0x84b1, 0x08);
ov5640_write( 0x84b2, 0xe6);
ov5640_write( 0x84b3, 0x78);
ov5640_write( 0x84b4, 0x02);
ov5640_write( 0x84b5, 0xce);
ov5640_write( 0x84b6, 0xc3);
ov5640_write( 0x84b7, 0x13);
ov5640_write( 0x84b8, 0xce);
ov5640_write( 0x84b9, 0x13);
ov5640_write( 0x84ba, 0xd8);
ov5640_write( 0x84bb, 0xf9);
ov5640_write( 0x84bc, 0x79);
ov5640_write( 0x84bd, 0x50);
ov5640_write( 0x84be, 0xf7);
ov5640_write( 0x84bf, 0xee);
ov5640_write( 0x84c0, 0x19);
ov5640_write( 0x84c1, 0xf7);
ov5640_write( 0x84c2, 0x90);
ov5640_write( 0x84c3, 0x38);
ov5640_write( 0x84c4, 0x15);
ov5640_write( 0x84c5, 0xe0);
ov5640_write( 0x84c6, 0xb4);
ov5640_write( 0x84c7, 0x31);
ov5640_write( 0x84c8, 0x07);
ov5640_write( 0x84c9, 0x78);
ov5640_write( 0x84ca, 0x51);
ov5640_write( 0x84cb, 0x79);
ov5640_write( 0x84cc, 0x51);
ov5640_write( 0x84cd, 0x12);
ov5640_write( 0x84ce, 0x09);
ov5640_write( 0x84cf, 0x6c);
ov5640_write( 0x84d0, 0x90);
ov5640_write( 0x84d1, 0x38);
ov5640_write( 0x84d2, 0x15);
ov5640_write( 0x84d3, 0xe0);
ov5640_write( 0x84d4, 0xb4);
ov5640_write( 0x84d5, 0x71);
ov5640_write( 0x84d6, 0x15);
ov5640_write( 0x84d7, 0x78);
ov5640_write( 0x84d8, 0x51);
ov5640_write( 0x84d9, 0xe6);
ov5640_write( 0x84da, 0xfe);
ov5640_write( 0x84db, 0x08);
ov5640_write( 0x84dc, 0xe6);
ov5640_write( 0x84dd, 0x78);
ov5640_write( 0x84de, 0x02);
ov5640_write( 0x84df, 0xce);
ov5640_write( 0x84e0, 0xc3);
ov5640_write( 0x84e1, 0x13);
ov5640_write( 0x84e2, 0xce);
ov5640_write( 0x84e3, 0x13);
ov5640_write( 0x84e4, 0xd8);
ov5640_write( 0x84e5, 0xf9);
ov5640_write( 0x84e6, 0x79);
ov5640_write( 0x84e7, 0x52);
ov5640_write( 0x84e8, 0xf7);
ov5640_write( 0x84e9, 0xee);
ov5640_write( 0x84ea, 0x19);
ov5640_write( 0x84eb, 0xf7);
ov5640_write( 0x84ec, 0x79);
ov5640_write( 0x84ed, 0x4f);
ov5640_write( 0x84ee, 0x12);
ov5640_write( 0x84ef, 0x09);
ov5640_write( 0x84f0, 0x3e);
ov5640_write( 0x84f1, 0x09);
ov5640_write( 0x84f2, 0x12);
ov5640_write( 0x84f3, 0x09);
ov5640_write( 0x84f4, 0x3e);
ov5640_write( 0x84f5, 0xaf);
ov5640_write( 0x84f6, 0x45);
ov5640_write( 0x84f7, 0x12);
ov5640_write( 0x84f8, 0x08);
ov5640_write( 0x84f9, 0xf4);
ov5640_write( 0x84fa, 0x7d);
ov5640_write( 0x84fb, 0x50);
ov5640_write( 0x84fc, 0x12);
ov5640_write( 0x84fd, 0x02);
ov5640_write( 0x84fe, 0x71);
ov5640_write( 0x84ff, 0x78);
ov5640_write( 0x8500, 0x57);
ov5640_write( 0x8501, 0xa6);
ov5640_write( 0x8502, 0x06);
ov5640_write( 0x8503, 0x08);
ov5640_write( 0x8504, 0xa6);
ov5640_write( 0x8505, 0x07);
ov5640_write( 0x8506, 0xaf);
ov5640_write( 0x8507, 0x43);
ov5640_write( 0x8508, 0x12);
ov5640_write( 0x8509, 0x08);
ov5640_write( 0x850a, 0xf4);
ov5640_write( 0x850b, 0x7d);
ov5640_write( 0x850c, 0x50);
ov5640_write( 0x850d, 0x12);
ov5640_write( 0x850e, 0x02);
ov5640_write( 0x850f, 0x71);
ov5640_write( 0x8510, 0x78);
ov5640_write( 0x8511, 0x53);
ov5640_write( 0x8512, 0xa6);
ov5640_write( 0x8513, 0x06);
ov5640_write( 0x8514, 0x08);
ov5640_write( 0x8515, 0xa6);
ov5640_write( 0x8516, 0x07);
ov5640_write( 0x8517, 0xaf);
ov5640_write( 0x8518, 0x46);
ov5640_write( 0x8519, 0x78);
ov5640_write( 0x851a, 0x51);
ov5640_write( 0x851b, 0x12);
ov5640_write( 0x851c, 0x08);
ov5640_write( 0x851d, 0xf6);
ov5640_write( 0x851e, 0x7d);
ov5640_write( 0x851f, 0x3c);
ov5640_write( 0x8520, 0x12);
ov5640_write( 0x8521, 0x02);
ov5640_write( 0x8522, 0x71);
ov5640_write( 0x8523, 0x78);
ov5640_write( 0x8524, 0x59);
ov5640_write( 0x8525, 0xa6);
ov5640_write( 0x8526, 0x06);
ov5640_write( 0x8527, 0x08);
ov5640_write( 0x8528, 0xa6);
ov5640_write( 0x8529, 0x07);
ov5640_write( 0x852a, 0xaf);
ov5640_write( 0x852b, 0x44);
ov5640_write( 0x852c, 0x7e);
ov5640_write( 0x852d, 0x00);
ov5640_write( 0x852e, 0x78);
ov5640_write( 0x852f, 0x51);
ov5640_write( 0x8530, 0x12);
ov5640_write( 0x8531, 0x08);
ov5640_write( 0x8532, 0xf8);
ov5640_write( 0x8533, 0x7d);
ov5640_write( 0x8534, 0x3c);
ov5640_write( 0x8535, 0x12);
ov5640_write( 0x8536, 0x02);
ov5640_write( 0x8537, 0x71);
ov5640_write( 0x8538, 0x78);
ov5640_write( 0x8539, 0x55);
ov5640_write( 0x853a, 0xa6);
ov5640_write( 0x853b, 0x06);
ov5640_write( 0x853c, 0x08);
ov5640_write( 0x853d, 0xa6);
ov5640_write( 0x853e, 0x07);
ov5640_write( 0x853f, 0xc3);
ov5640_write( 0x8540, 0x78);
ov5640_write( 0x8541, 0x58);
ov5640_write( 0x8542, 0xe6);
ov5640_write( 0x8543, 0x94);
ov5640_write( 0x8544, 0x08);
ov5640_write( 0x8545, 0x18);
ov5640_write( 0x8546, 0xe6);
ov5640_write( 0x8547, 0x94);
ov5640_write( 0x8548, 0x00);
ov5640_write( 0x8549, 0x50);
ov5640_write( 0x854a, 0x05);
ov5640_write( 0x854b, 0x76);
ov5640_write( 0x854c, 0x00);
ov5640_write( 0x854d, 0x08);
ov5640_write( 0x854e, 0x76);
ov5640_write( 0x854f, 0x08);
ov5640_write( 0x8550, 0xc3);
ov5640_write( 0x8551, 0x78);
ov5640_write( 0x8552, 0x5a);
ov5640_write( 0x8553, 0xe6);
ov5640_write( 0x8554, 0x94);
ov5640_write( 0x8555, 0x08);
ov5640_write( 0x8556, 0x18);
ov5640_write( 0x8557, 0xe6);
ov5640_write( 0x8558, 0x94);
ov5640_write( 0x8559, 0x00);
ov5640_write( 0x855a, 0x50);
ov5640_write( 0x855b, 0x05);
ov5640_write( 0x855c, 0x76);
ov5640_write( 0x855d, 0x00);
ov5640_write( 0x855e, 0x08);
ov5640_write( 0x855f, 0x76);
ov5640_write( 0x8560, 0x08);
ov5640_write( 0x8561, 0x78);
ov5640_write( 0x8562, 0x57);
ov5640_write( 0x8563, 0x12);
ov5640_write( 0x8564, 0x09);
ov5640_write( 0x8565, 0x2b);
ov5640_write( 0x8566, 0xff);
ov5640_write( 0x8567, 0xd3);
ov5640_write( 0x8568, 0x78);
ov5640_write( 0x8569, 0x54);
ov5640_write( 0x856a, 0xe6);
ov5640_write( 0x856b, 0x9f);
ov5640_write( 0x856c, 0x18);
ov5640_write( 0x856d, 0xe6);
ov5640_write( 0x856e, 0x9e);
ov5640_write( 0x856f, 0x40);
ov5640_write( 0x8570, 0x0e);
ov5640_write( 0x8571, 0x78);
ov5640_write( 0x8572, 0x57);
ov5640_write( 0x8573, 0xe6);
ov5640_write( 0x8574, 0x13);
ov5640_write( 0x8575, 0xfe);
ov5640_write( 0x8576, 0x08);
ov5640_write( 0x8577, 0xe6);
ov5640_write( 0x8578, 0x78);
ov5640_write( 0x8579, 0x54);
ov5640_write( 0x857a, 0x12);
ov5640_write( 0x857b, 0x09);
ov5640_write( 0x857c, 0x61);
ov5640_write( 0x857d, 0x80);
ov5640_write( 0x857e, 0x04);
ov5640_write( 0x857f, 0x7e);
ov5640_write( 0x8580, 0x00);
ov5640_write( 0x8581, 0x7f);
ov5640_write( 0x8582, 0x00);
ov5640_write( 0x8583, 0x78);
ov5640_write( 0x8584, 0x5b);
ov5640_write( 0x8585, 0x12);
ov5640_write( 0x8586, 0x09);
ov5640_write( 0x8587, 0x23);
ov5640_write( 0x8588, 0xff);
ov5640_write( 0x8589, 0xd3);
ov5640_write( 0x858a, 0x78);
ov5640_write( 0x858b, 0x56);
ov5640_write( 0x858c, 0xe6);
ov5640_write( 0x858d, 0x9f);
ov5640_write( 0x858e, 0x18);
ov5640_write( 0x858f, 0xe6);
ov5640_write( 0x8590, 0x9e);
ov5640_write( 0x8591, 0x40);
ov5640_write( 0x8592, 0x0e);
ov5640_write( 0x8593, 0x78);
ov5640_write( 0x8594, 0x59);
ov5640_write( 0x8595, 0xe6);
ov5640_write( 0x8596, 0x13);
ov5640_write( 0x8597, 0xfe);
ov5640_write( 0x8598, 0x08);
ov5640_write( 0x8599, 0xe6);
ov5640_write( 0x859a, 0x78);
ov5640_write( 0x859b, 0x56);
ov5640_write( 0x859c, 0x12);
ov5640_write( 0x859d, 0x09);
ov5640_write( 0x859e, 0x61);
ov5640_write( 0x859f, 0x80);
ov5640_write( 0x85a0, 0x04);
ov5640_write( 0x85a1, 0x7e);
ov5640_write( 0x85a2, 0x00);
ov5640_write( 0x85a3, 0x7f);
ov5640_write( 0x85a4, 0x00);
ov5640_write( 0x85a5, 0xe4);
ov5640_write( 0x85a6, 0xfc);
ov5640_write( 0x85a7, 0xfd);
ov5640_write( 0x85a8, 0x78);
ov5640_write( 0x85a9, 0x5f);
ov5640_write( 0x85aa, 0x12);
ov5640_write( 0x85ab, 0x04);
ov5640_write( 0x85ac, 0x24);
ov5640_write( 0x85ad, 0x78);
ov5640_write( 0x85ae, 0x57);
ov5640_write( 0x85af, 0x12);
ov5640_write( 0x85b0, 0x09);
ov5640_write( 0x85b1, 0x2b);
ov5640_write( 0x85b2, 0x78);
ov5640_write( 0x85b3, 0x54);
ov5640_write( 0x85b4, 0x26);
ov5640_write( 0x85b5, 0xff);
ov5640_write( 0x85b6, 0xee);
ov5640_write( 0x85b7, 0x18);
ov5640_write( 0x85b8, 0x36);
ov5640_write( 0x85b9, 0xfe);
ov5640_write( 0x85ba, 0x78);
ov5640_write( 0x85bb, 0x63);
ov5640_write( 0x85bc, 0x12);
ov5640_write( 0x85bd, 0x09);
ov5640_write( 0x85be, 0x23);
ov5640_write( 0x85bf, 0x78);
ov5640_write( 0x85c0, 0x56);
ov5640_write( 0x85c1, 0x26);
ov5640_write( 0x85c2, 0xff);
ov5640_write( 0x85c3, 0xee);
ov5640_write( 0x85c4, 0x18);
ov5640_write( 0x85c5, 0x36);
ov5640_write( 0x85c6, 0xfe);
ov5640_write( 0x85c7, 0xe4);
ov5640_write( 0x85c8, 0xfc);
ov5640_write( 0x85c9, 0xfd);
ov5640_write( 0x85ca, 0x78);
ov5640_write( 0x85cb, 0x67);
ov5640_write( 0x85cc, 0x12);
ov5640_write( 0x85cd, 0x04);
ov5640_write( 0x85ce, 0x24);
ov5640_write( 0x85cf, 0x12);
ov5640_write( 0x85d0, 0x09);
ov5640_write( 0x85d1, 0x33);
ov5640_write( 0x85d2, 0x78);
ov5640_write( 0x85d3, 0x63);
ov5640_write( 0x85d4, 0x12);
ov5640_write( 0x85d5, 0x04);
ov5640_write( 0x85d6, 0x17);
ov5640_write( 0x85d7, 0xd3);
ov5640_write( 0x85d8, 0x12);
ov5640_write( 0x85d9, 0x03);
ov5640_write( 0x85da, 0xe3);
ov5640_write( 0x85db, 0x40);
ov5640_write( 0x85dc, 0x08);
ov5640_write( 0x85dd, 0x12);
ov5640_write( 0x85de, 0x09);
ov5640_write( 0x85df, 0x33);
ov5640_write( 0x85e0, 0x78);
ov5640_write( 0x85e1, 0x63);
ov5640_write( 0x85e2, 0x12);
ov5640_write( 0x85e3, 0x04);
ov5640_write( 0x85e4, 0x24);
ov5640_write( 0x85e5, 0x78);
ov5640_write( 0x85e6, 0x51);
ov5640_write( 0x85e7, 0x12);
ov5640_write( 0x85e8, 0x09);
ov5640_write( 0x85e9, 0x35);
ov5640_write( 0x85ea, 0x78);
ov5640_write( 0x85eb, 0x67);
ov5640_write( 0x85ec, 0x12);
ov5640_write( 0x85ed, 0x04);
ov5640_write( 0x85ee, 0x17);
ov5640_write( 0x85ef, 0xd3);
ov5640_write( 0x85f0, 0x12);
ov5640_write( 0x85f1, 0x03);
ov5640_write( 0x85f2, 0xe3);
ov5640_write( 0x85f3, 0x40);
ov5640_write( 0x85f4, 0x0a);
ov5640_write( 0x85f5, 0x78);
ov5640_write( 0x85f6, 0x51);
ov5640_write( 0x85f7, 0x12);
ov5640_write( 0x85f8, 0x09);
ov5640_write( 0x85f9, 0x35);
ov5640_write( 0x85fa, 0x78);
ov5640_write( 0x85fb, 0x67);
ov5640_write( 0x85fc, 0x12);
ov5640_write( 0x85fd, 0x04);
ov5640_write( 0x85fe, 0x24);
ov5640_write( 0x85ff, 0x78);
ov5640_write( 0x8600, 0x5e);
ov5640_write( 0x8601, 0xe6);
ov5640_write( 0x8602, 0x90);
ov5640_write( 0x8603, 0x60);
ov5640_write( 0x8604, 0x01);
ov5640_write( 0x8605, 0xf0);
ov5640_write( 0x8606, 0x78);
ov5640_write( 0x8607, 0x62);
ov5640_write( 0x8608, 0xe6);
ov5640_write( 0x8609, 0xa3);
ov5640_write( 0x860a, 0xf0);
ov5640_write( 0x860b, 0x78);
ov5640_write( 0x860c, 0x66);
ov5640_write( 0x860d, 0xe6);
ov5640_write( 0x860e, 0xa3);
ov5640_write( 0x860f, 0xf0);
ov5640_write( 0x8610, 0x78);
ov5640_write( 0x8611, 0x52);
ov5640_write( 0x8612, 0xe6);
ov5640_write( 0x8613, 0xa3);
ov5640_write( 0x8614, 0xf0);
ov5640_write( 0x8615, 0x7d);
ov5640_write( 0x8616, 0x01);
ov5640_write( 0x8617, 0x78);
ov5640_write( 0x8618, 0x5e);
ov5640_write( 0x8619, 0x12);
ov5640_write( 0x861a, 0x09);
ov5640_write( 0x861b, 0x4e);
ov5640_write( 0x861c, 0x24);
ov5640_write( 0x861d, 0x01);
ov5640_write( 0x861e, 0x12);
ov5640_write( 0x861f, 0x09);
ov5640_write( 0x8620, 0x17);
ov5640_write( 0x8621, 0x78);
ov5640_write( 0x8622, 0x62);
ov5640_write( 0x8623, 0x12);
ov5640_write( 0x8624, 0x09);
ov5640_write( 0x8625, 0x4e);
ov5640_write( 0x8626, 0x24);
ov5640_write( 0x8627, 0x02);
ov5640_write( 0x8628, 0x12);
ov5640_write( 0x8629, 0x09);
ov5640_write( 0x862a, 0x17);
ov5640_write( 0x862b, 0x78);
ov5640_write( 0x862c, 0x66);
ov5640_write( 0x862d, 0x12);
ov5640_write( 0x862e, 0x09);
ov5640_write( 0x862f, 0x4e);
ov5640_write( 0x8630, 0x24);
ov5640_write( 0x8631, 0x03);
ov5640_write( 0x8632, 0x12);
ov5640_write( 0x8633, 0x09);
ov5640_write( 0x8634, 0x17);
ov5640_write( 0x8635, 0x78);
ov5640_write( 0x8636, 0x6a);
ov5640_write( 0x8637, 0x12);
ov5640_write( 0x8638, 0x09);
ov5640_write( 0x8639, 0x4e);
ov5640_write( 0x863a, 0x24);
ov5640_write( 0x863b, 0x04);
ov5640_write( 0x863c, 0x12);
ov5640_write( 0x863d, 0x09);
ov5640_write( 0x863e, 0x17);
ov5640_write( 0x863f, 0x0d);
ov5640_write( 0x8640, 0xbd);
ov5640_write( 0x8641, 0x05);
ov5640_write( 0x8642, 0xd4);
ov5640_write( 0x8643, 0xc2);
ov5640_write( 0x8644, 0x0e);
ov5640_write( 0x8645, 0xc2);
ov5640_write( 0x8646, 0x06);
ov5640_write( 0x8647, 0x22);
ov5640_write( 0x8648, 0x85);
ov5640_write( 0x8649, 0x08);
ov5640_write( 0x864a, 0x41);
ov5640_write( 0x864b, 0x90);
ov5640_write( 0x864c, 0x30);
ov5640_write( 0x864d, 0x24);
ov5640_write( 0x864e, 0xe0);
ov5640_write( 0x864f, 0xf5);
ov5640_write( 0x8650, 0x3d);
ov5640_write( 0x8651, 0xa3);
ov5640_write( 0x8652, 0xe0);
ov5640_write( 0x8653, 0xf5);
ov5640_write( 0x8654, 0x3e);
ov5640_write( 0x8655, 0xa3);
ov5640_write( 0x8656, 0xe0);
ov5640_write( 0x8657, 0xf5);
ov5640_write( 0x8658, 0x3f);
ov5640_write( 0x8659, 0xa3);
ov5640_write( 0x865a, 0xe0);
ov5640_write( 0x865b, 0xf5);
ov5640_write( 0x865c, 0x40);
ov5640_write( 0x865d, 0xa3);
ov5640_write( 0x865e, 0xe0);
ov5640_write( 0x865f, 0xf5);
ov5640_write( 0x8660, 0x3c);
ov5640_write( 0x8661, 0xd2);
ov5640_write( 0x8662, 0x34);
ov5640_write( 0x8663, 0xe5);
ov5640_write( 0x8664, 0x41);
ov5640_write( 0x8665, 0x12);
ov5640_write( 0x8666, 0x04);
ov5640_write( 0x8667, 0x3c);
ov5640_write( 0x8668, 0x06);
ov5640_write( 0x8669, 0xa5);
ov5640_write( 0x866a, 0x03);
ov5640_write( 0x866b, 0x06);
ov5640_write( 0x866c, 0xa9);
ov5640_write( 0x866d, 0x04);
ov5640_write( 0x866e, 0x06);
ov5640_write( 0x866f, 0xaf);
ov5640_write( 0x8670, 0x07);
ov5640_write( 0x8671, 0x06);
ov5640_write( 0x8672, 0xb8);
ov5640_write( 0x8673, 0x08);
ov5640_write( 0x8674, 0x06);
ov5640_write( 0x8675, 0xc9);
ov5640_write( 0x8676, 0x12);
ov5640_write( 0x8677, 0x06);
ov5640_write( 0x8678, 0xe1);
ov5640_write( 0x8679, 0x18);
ov5640_write( 0x867a, 0x06);
ov5640_write( 0x867b, 0xf7);
ov5640_write( 0x867c, 0x19);
ov5640_write( 0x867d, 0x06);
ov5640_write( 0x867e, 0xcc);
ov5640_write( 0x867f, 0x1a);
ov5640_write( 0x8680, 0x06);
ov5640_write( 0x8681, 0xd8);
ov5640_write( 0x8682, 0x1b);
ov5640_write( 0x8683, 0x07);
ov5640_write( 0x8684, 0x1c);
ov5640_write( 0x8685, 0x80);
ov5640_write( 0x8686, 0x07);
ov5640_write( 0x8687, 0x21);
ov5640_write( 0x8688, 0x81);
ov5640_write( 0x8689, 0x07);
ov5640_write( 0x868a, 0x7f);
ov5640_write( 0x868b, 0x8f);
ov5640_write( 0x868c, 0x07);
ov5640_write( 0x868d, 0x6e);
ov5640_write( 0x868e, 0x90);
ov5640_write( 0x868f, 0x07);
ov5640_write( 0x8690, 0x7f);
ov5640_write( 0x8691, 0x91);
ov5640_write( 0x8692, 0x07);
ov5640_write( 0x8693, 0x7f);
ov5640_write( 0x8694, 0x92);
ov5640_write( 0x8695, 0x07);
ov5640_write( 0x8696, 0x7f);
ov5640_write( 0x8697, 0x93);
ov5640_write( 0x8698, 0x07);
ov5640_write( 0x8699, 0x7f);
ov5640_write( 0x869a, 0x94);
ov5640_write( 0x869b, 0x07);
ov5640_write( 0x869c, 0x7f);
ov5640_write( 0x869d, 0x98);
ov5640_write( 0x869e, 0x07);
ov5640_write( 0x869f, 0x7c);
ov5640_write( 0x86a0, 0x9f);
ov5640_write( 0x86a1, 0x00);
ov5640_write( 0x86a2, 0x00);
ov5640_write( 0x86a3, 0x07);
ov5640_write( 0x86a4, 0x9a);
ov5640_write( 0x86a5, 0x12);
ov5640_write( 0x86a6, 0x0a);
ov5640_write( 0x86a7, 0xf8);
ov5640_write( 0x86a8, 0x22);
ov5640_write( 0x86a9, 0x12);
ov5640_write( 0x86aa, 0x0a);
ov5640_write( 0x86ab, 0xf8);
ov5640_write( 0x86ac, 0xd2);
ov5640_write( 0x86ad, 0x03);
ov5640_write( 0x86ae, 0x22);
ov5640_write( 0x86af, 0xa2);
ov5640_write( 0x86b0, 0x37);
ov5640_write( 0x86b1, 0xe4);
ov5640_write( 0x86b2, 0x33);
ov5640_write( 0x86b3, 0xf5);
ov5640_write( 0x86b4, 0x3c);
ov5640_write( 0x86b5, 0x02);
ov5640_write( 0x86b6, 0x07);
ov5640_write( 0x86b7, 0x7f);
ov5640_write( 0x86b8, 0xc2);
ov5640_write( 0x86b9, 0x01);
ov5640_write( 0x86ba, 0xc2);
ov5640_write( 0x86bb, 0x02);
ov5640_write( 0x86bc, 0xc2);
ov5640_write( 0x86bd, 0x03);
ov5640_write( 0x86be, 0x12);
ov5640_write( 0x86bf, 0x09);
ov5640_write( 0x86c0, 0x76);
ov5640_write( 0x86c1, 0x75);
ov5640_write( 0x86c2, 0x1e);
ov5640_write( 0x86c3, 0x70);
ov5640_write( 0x86c4, 0xd2);
ov5640_write( 0x86c5, 0x35);
ov5640_write( 0x86c6, 0x02);
ov5640_write( 0x86c7, 0x07);
ov5640_write( 0x86c8, 0x7f);
ov5640_write( 0x86c9, 0x02);
ov5640_write( 0x86ca, 0x07);
ov5640_write( 0x86cb, 0x69);
ov5640_write( 0x86cc, 0x85);
ov5640_write( 0x86cd, 0x40);
ov5640_write( 0x86ce, 0x48);
ov5640_write( 0x86cf, 0x85);
ov5640_write( 0x86d0, 0x3c);
ov5640_write( 0x86d1, 0x49);
ov5640_write( 0x86d2, 0x12);
ov5640_write( 0x86d3, 0x08);
ov5640_write( 0x86d4, 0x61);
ov5640_write( 0x86d5, 0x02);
ov5640_write( 0x86d6, 0x07);
ov5640_write( 0x86d7, 0x7f);
ov5640_write( 0x86d8, 0x85);
ov5640_write( 0x86d9, 0x48);
ov5640_write( 0x86da, 0x40);
ov5640_write( 0x86db, 0x85);
ov5640_write( 0x86dc, 0x49);
ov5640_write( 0x86dd, 0x3c);
ov5640_write( 0x86de, 0x02);
ov5640_write( 0x86df, 0x07);
ov5640_write( 0x86e0, 0x7f);
ov5640_write( 0x86e1, 0xe4);
ov5640_write( 0x86e2, 0xf5);
ov5640_write( 0x86e3, 0x22);
ov5640_write( 0x86e4, 0xf5);
ov5640_write( 0x86e5, 0x23);
ov5640_write( 0x86e6, 0x85);
ov5640_write( 0x86e7, 0x40);
ov5640_write( 0x86e8, 0x31);
ov5640_write( 0x86e9, 0x85);
ov5640_write( 0x86ea, 0x3f);
ov5640_write( 0x86eb, 0x30);
ov5640_write( 0x86ec, 0x85);
ov5640_write( 0x86ed, 0x3e);
ov5640_write( 0x86ee, 0x2f);
ov5640_write( 0x86ef, 0x85);
ov5640_write( 0x86f0, 0x3d);
ov5640_write( 0x86f1, 0x2e);
ov5640_write( 0x86f2, 0x12);
ov5640_write( 0x86f3, 0x0a);
ov5640_write( 0x86f4, 0xca);
ov5640_write( 0x86f5, 0x80);
ov5640_write( 0x86f6, 0x1f);
ov5640_write( 0x86f7, 0x75);
ov5640_write( 0x86f8, 0x22);
ov5640_write( 0x86f9, 0x00);
ov5640_write( 0x86fa, 0x75);
ov5640_write( 0x86fb, 0x23);
ov5640_write( 0x86fc, 0x01);
ov5640_write( 0x86fd, 0x74);
ov5640_write( 0x86fe, 0xff);
ov5640_write( 0x86ff, 0xf5);
ov5640_write( 0x8700, 0x2d);
ov5640_write( 0x8701, 0xf5);
ov5640_write( 0x8702, 0x2c);
ov5640_write( 0x8703, 0xf5);
ov5640_write( 0x8704, 0x2b);
ov5640_write( 0x8705, 0xf5);
ov5640_write( 0x8706, 0x2a);
ov5640_write( 0x8707, 0x12);
ov5640_write( 0x8708, 0x0a);
ov5640_write( 0x8709, 0xca);
ov5640_write( 0x870a, 0x85);
ov5640_write( 0x870b, 0x2d);
ov5640_write( 0x870c, 0x40);
ov5640_write( 0x870d, 0x85);
ov5640_write( 0x870e, 0x2c);
ov5640_write( 0x870f, 0x3f);
ov5640_write( 0x8710, 0x85);
ov5640_write( 0x8711, 0x2b);
ov5640_write( 0x8712, 0x3e);
ov5640_write( 0x8713, 0x85);
ov5640_write( 0x8714, 0x2a);
ov5640_write( 0x8715, 0x3d);
ov5640_write( 0x8716, 0xe4);
ov5640_write( 0x8717, 0xf5);
ov5640_write( 0x8718, 0x3c);
ov5640_write( 0x8719, 0x02);
ov5640_write( 0x871a, 0x07);
ov5640_write( 0x871b, 0x7f);
ov5640_write( 0x871c, 0x12);
ov5640_write( 0x871d, 0x0b);
ov5640_write( 0x871e, 0x41);
ov5640_write( 0x871f, 0x80);
ov5640_write( 0x8720, 0x5e);
ov5640_write( 0x8721, 0x85);
ov5640_write( 0x8722, 0x3d);
ov5640_write( 0x8723, 0x43);
ov5640_write( 0x8724, 0x85);
ov5640_write( 0x8725, 0x3e);
ov5640_write( 0x8726, 0x44);
ov5640_write( 0x8727, 0xe5);
ov5640_write( 0x8728, 0x45);
ov5640_write( 0x8729, 0xc3);
ov5640_write( 0x872a, 0x13);
ov5640_write( 0x872b, 0xff);
ov5640_write( 0x872c, 0xe5);
ov5640_write( 0x872d, 0x43);
ov5640_write( 0x872e, 0xc3);
ov5640_write( 0x872f, 0x9f);
ov5640_write( 0x8730, 0x50);
ov5640_write( 0x8731, 0x02);
ov5640_write( 0x8732, 0x8f);
ov5640_write( 0x8733, 0x43);
ov5640_write( 0x8734, 0xe5);
ov5640_write( 0x8735, 0x46);
ov5640_write( 0x8736, 0xc3);
ov5640_write( 0x8737, 0x13);
ov5640_write( 0x8738, 0xff);
ov5640_write( 0x8739, 0xe5);
ov5640_write( 0x873a, 0x44);
ov5640_write( 0x873b, 0xc3);
ov5640_write( 0x873c, 0x9f);
ov5640_write( 0x873d, 0x50);
ov5640_write( 0x873e, 0x02);
ov5640_write( 0x873f, 0x8f);
ov5640_write( 0x8740, 0x44);
ov5640_write( 0x8741, 0xe5);
ov5640_write( 0x8742, 0x45);
ov5640_write( 0x8743, 0xc3);
ov5640_write( 0x8744, 0x13);
ov5640_write( 0x8745, 0xff);
ov5640_write( 0x8746, 0xfd);
ov5640_write( 0x8747, 0xe5);
ov5640_write( 0x8748, 0x43);
ov5640_write( 0x8749, 0x90);
ov5640_write( 0x874a, 0x0e);
ov5640_write( 0x874b, 0x7f);
ov5640_write( 0x874c, 0x12);
ov5640_write( 0x874d, 0x0b);
ov5640_write( 0x874e, 0x14);
ov5640_write( 0x874f, 0x40);
ov5640_write( 0x8750, 0x04);
ov5640_write( 0x8751, 0xee);
ov5640_write( 0x8752, 0x9f);
ov5640_write( 0x8753, 0xf5);
ov5640_write( 0x8754, 0x43);
ov5640_write( 0x8755, 0xe5);
ov5640_write( 0x8756, 0x46);
ov5640_write( 0x8757, 0xc3);
ov5640_write( 0x8758, 0x13);
ov5640_write( 0x8759, 0xff);
ov5640_write( 0x875a, 0xfd);
ov5640_write( 0x875b, 0xe5);
ov5640_write( 0x875c, 0x44);
ov5640_write( 0x875d, 0x90);
ov5640_write( 0x875e, 0x0e);
ov5640_write( 0x875f, 0x80);
ov5640_write( 0x8760, 0x12);
ov5640_write( 0x8761, 0x0b);
ov5640_write( 0x8762, 0x14);
ov5640_write( 0x8763, 0x40);
ov5640_write( 0x8764, 0x04);
ov5640_write( 0x8765, 0xee);
ov5640_write( 0x8766, 0x9f);
ov5640_write( 0x8767, 0xf5);
ov5640_write( 0x8768, 0x44);
ov5640_write( 0x8769, 0x12);
ov5640_write( 0x876a, 0x04);
ov5640_write( 0x876b, 0x62);
ov5640_write( 0x876c, 0x80);
ov5640_write( 0x876d, 0x11);
ov5640_write( 0x876e, 0x85);
ov5640_write( 0x876f, 0x40);
ov5640_write( 0x8770, 0x46);
ov5640_write( 0x8771, 0x85);
ov5640_write( 0x8772, 0x3f);
ov5640_write( 0x8773, 0x45);
ov5640_write( 0x8774, 0x85);
ov5640_write( 0x8775, 0x3e);
ov5640_write( 0x8776, 0x44);
ov5640_write( 0x8777, 0x85);
ov5640_write( 0x8778, 0x3d);
ov5640_write( 0x8779, 0x43);
ov5640_write( 0x877a, 0x80);
ov5640_write( 0x877b, 0x03);
ov5640_write( 0x877c, 0x02);
ov5640_write( 0x877d, 0x04);
ov5640_write( 0x877e, 0x62);
ov5640_write( 0x877f, 0x90);
ov5640_write( 0x8780, 0x30);
ov5640_write( 0x8781, 0x24);
ov5640_write( 0x8782, 0xe5);
ov5640_write( 0x8783, 0x3d);
ov5640_write( 0x8784, 0xf0);
ov5640_write( 0x8785, 0xa3);
ov5640_write( 0x8786, 0xe5);
ov5640_write( 0x8787, 0x3e);
ov5640_write( 0x8788, 0xf0);
ov5640_write( 0x8789, 0xa3);
ov5640_write( 0x878a, 0xe5);
ov5640_write( 0x878b, 0x3f);
ov5640_write( 0x878c, 0xf0);
ov5640_write( 0x878d, 0xa3);
ov5640_write( 0x878e, 0xe5);
ov5640_write( 0x878f, 0x40);
ov5640_write( 0x8790, 0xf0);
ov5640_write( 0x8791, 0xa3);
ov5640_write( 0x8792, 0xe5);
ov5640_write( 0x8793, 0x3c);
ov5640_write( 0x8794, 0xf0);
ov5640_write( 0x8795, 0x90);
ov5640_write( 0x8796, 0x30);
ov5640_write( 0x8797, 0x23);
ov5640_write( 0x8798, 0xe4);
ov5640_write( 0x8799, 0xf0);
ov5640_write( 0x879a, 0x22);
ov5640_write( 0x879b, 0xc0);
ov5640_write( 0x879c, 0xe0);
ov5640_write( 0x879d, 0xc0);
ov5640_write( 0x879e, 0x83);
ov5640_write( 0x879f, 0xc0);
ov5640_write( 0x87a0, 0x82);
ov5640_write( 0x87a1, 0xc0);
ov5640_write( 0x87a2, 0xd0);
ov5640_write( 0x87a3, 0x90);
ov5640_write( 0x87a4, 0x3f);
ov5640_write( 0x87a5, 0x0c);
ov5640_write( 0x87a6, 0xe0);
ov5640_write( 0x87a7, 0xf5);
ov5640_write( 0x87a8, 0x32);
ov5640_write( 0x87a9, 0xe5);
ov5640_write( 0x87aa, 0x32);
ov5640_write( 0x87ab, 0x30);
ov5640_write( 0x87ac, 0xe3);
ov5640_write( 0x87ad, 0x74);
ov5640_write( 0x87ae, 0x30);
ov5640_write( 0x87af, 0x36);
ov5640_write( 0x87b0, 0x66);
ov5640_write( 0x87b1, 0x90);
ov5640_write( 0x87b2, 0x60);
ov5640_write( 0x87b3, 0x19);
ov5640_write( 0x87b4, 0xe0);
ov5640_write( 0x87b5, 0xf5);
ov5640_write( 0x87b6, 0x0a);
ov5640_write( 0x87b7, 0xa3);
ov5640_write( 0x87b8, 0xe0);
ov5640_write( 0x87b9, 0xf5);
ov5640_write( 0x87ba, 0x0b);
ov5640_write( 0x87bb, 0x90);
ov5640_write( 0x87bc, 0x60);
ov5640_write( 0x87bd, 0x1d);
ov5640_write( 0x87be, 0xe0);
ov5640_write( 0x87bf, 0xf5);
ov5640_write( 0x87c0, 0x14);
ov5640_write( 0x87c1, 0xa3);
ov5640_write( 0x87c2, 0xe0);
ov5640_write( 0x87c3, 0xf5);
ov5640_write( 0x87c4, 0x15);
ov5640_write( 0x87c5, 0x90);
ov5640_write( 0x87c6, 0x60);
ov5640_write( 0x87c7, 0x21);
ov5640_write( 0x87c8, 0xe0);
ov5640_write( 0x87c9, 0xf5);
ov5640_write( 0x87ca, 0x0c);
ov5640_write( 0x87cb, 0xa3);
ov5640_write( 0x87cc, 0xe0);
ov5640_write( 0x87cd, 0xf5);
ov5640_write( 0x87ce, 0x0d);
ov5640_write( 0x87cf, 0x90);
ov5640_write( 0x87d0, 0x60);
ov5640_write( 0x87d1, 0x29);
ov5640_write( 0x87d2, 0xe0);
ov5640_write( 0x87d3, 0xf5);
ov5640_write( 0x87d4, 0x0e);
ov5640_write( 0x87d5, 0xa3);
ov5640_write( 0x87d6, 0xe0);
ov5640_write( 0x87d7, 0xf5);
ov5640_write( 0x87d8, 0x0f);
ov5640_write( 0x87d9, 0x90);
ov5640_write( 0x87da, 0x60);
ov5640_write( 0x87db, 0x31);
ov5640_write( 0x87dc, 0xe0);
ov5640_write( 0x87dd, 0xf5);
ov5640_write( 0x87de, 0x10);
ov5640_write( 0x87df, 0xa3);
ov5640_write( 0x87e0, 0xe0);
ov5640_write( 0x87e1, 0xf5);
ov5640_write( 0x87e2, 0x11);
ov5640_write( 0x87e3, 0x90);
ov5640_write( 0x87e4, 0x60);
ov5640_write( 0x87e5, 0x39);
ov5640_write( 0x87e6, 0xe0);
ov5640_write( 0x87e7, 0xf5);
ov5640_write( 0x87e8, 0x12);
ov5640_write( 0x87e9, 0xa3);
ov5640_write( 0x87ea, 0xe0);
ov5640_write( 0x87eb, 0xf5);
ov5640_write( 0x87ec, 0x13);
ov5640_write( 0x87ed, 0x30);
ov5640_write( 0x87ee, 0x01);
ov5640_write( 0x87ef, 0x06);
ov5640_write( 0x87f0, 0x30);
ov5640_write( 0x87f1, 0x33);
ov5640_write( 0x87f2, 0x03);
ov5640_write( 0x87f3, 0xd3);
ov5640_write( 0x87f4, 0x80);
ov5640_write( 0x87f5, 0x01);
ov5640_write( 0x87f6, 0xc3);
ov5640_write( 0x87f7, 0x92);
ov5640_write( 0x87f8, 0x09);
ov5640_write( 0x87f9, 0x30);
ov5640_write( 0x87fa, 0x02);
ov5640_write( 0x87fb, 0x06);
ov5640_write( 0x87fc, 0x30);
ov5640_write( 0x87fd, 0x33);
ov5640_write( 0x87fe, 0x03);
ov5640_write( 0x87ff, 0xd3);
ov5640_write( 0x8800, 0x80);
ov5640_write( 0x8801, 0x01);
ov5640_write( 0x8802, 0xc3);
ov5640_write( 0x8803, 0x92);
ov5640_write( 0x8804, 0x0a);
ov5640_write( 0x8805, 0x30);
ov5640_write( 0x8806, 0x33);
ov5640_write( 0x8807, 0x0c);
ov5640_write( 0x8808, 0x30);
ov5640_write( 0x8809, 0x03);
ov5640_write( 0x880a, 0x09);
ov5640_write( 0x880b, 0x20);
ov5640_write( 0x880c, 0x02);
ov5640_write( 0x880d, 0x06);
ov5640_write( 0x880e, 0x20);
ov5640_write( 0x880f, 0x01);
ov5640_write( 0x8810, 0x03);
ov5640_write( 0x8811, 0xd3);
ov5640_write( 0x8812, 0x80);
ov5640_write( 0x8813, 0x01);
ov5640_write( 0x8814, 0xc3);
ov5640_write( 0x8815, 0x92);
ov5640_write( 0x8816, 0x0b);
ov5640_write( 0x8817, 0x90);
ov5640_write( 0x8818, 0x30);
ov5640_write( 0x8819, 0x01);
ov5640_write( 0x881a, 0xe0);
ov5640_write( 0x881b, 0x44);
ov5640_write( 0x881c, 0x40);
ov5640_write( 0x881d, 0xf0);
ov5640_write( 0x881e, 0xe0);
ov5640_write( 0x881f, 0x54);
ov5640_write( 0x8820, 0xbf);
ov5640_write( 0x8821, 0xf0);
ov5640_write( 0x8822, 0xe5);
ov5640_write( 0x8823, 0x32);
ov5640_write( 0x8824, 0x30);
ov5640_write( 0x8825, 0xe1);
ov5640_write( 0x8826, 0x14);
ov5640_write( 0x8827, 0x30);
ov5640_write( 0x8828, 0x34);
ov5640_write( 0x8829, 0x11);
ov5640_write( 0x882a, 0x90);
ov5640_write( 0x882b, 0x30);
ov5640_write( 0x882c, 0x22);
ov5640_write( 0x882d, 0xe0);
ov5640_write( 0x882e, 0xf5);
ov5640_write( 0x882f, 0x08);
ov5640_write( 0x8830, 0xe4);
ov5640_write( 0x8831, 0xf0);
ov5640_write( 0x8832, 0x30);
ov5640_write( 0x8833, 0x00);
ov5640_write( 0x8834, 0x03);
ov5640_write( 0x8835, 0xd3);
ov5640_write( 0x8836, 0x80);
ov5640_write( 0x8837, 0x01);
ov5640_write( 0x8838, 0xc3);
ov5640_write( 0x8839, 0x92);
ov5640_write( 0x883a, 0x08);
ov5640_write( 0x883b, 0xe5);
ov5640_write( 0x883c, 0x32);
ov5640_write( 0x883d, 0x30);
ov5640_write( 0x883e, 0xe5);
ov5640_write( 0x883f, 0x12);
ov5640_write( 0x8840, 0x90);
ov5640_write( 0x8841, 0x56);
ov5640_write( 0x8842, 0xa1);
ov5640_write( 0x8843, 0xe0);
ov5640_write( 0x8844, 0xf5);
ov5640_write( 0x8845, 0x09);
ov5640_write( 0x8846, 0x30);
ov5640_write( 0x8847, 0x31);
ov5640_write( 0x8848, 0x09);
ov5640_write( 0x8849, 0x30);
ov5640_write( 0x884a, 0x05);
ov5640_write( 0x884b, 0x03);
ov5640_write( 0x884c, 0xd3);
ov5640_write( 0x884d, 0x80);
ov5640_write( 0x884e, 0x01);
ov5640_write( 0x884f, 0xc3);
ov5640_write( 0x8850, 0x92);
ov5640_write( 0x8851, 0x0d);
ov5640_write( 0x8852, 0x90);
ov5640_write( 0x8853, 0x3f);
ov5640_write( 0x8854, 0x0c);
ov5640_write( 0x8855, 0xe5);
ov5640_write( 0x8856, 0x32);
ov5640_write( 0x8857, 0xf0);
ov5640_write( 0x8858, 0xd0);
ov5640_write( 0x8859, 0xd0);
ov5640_write( 0x885a, 0xd0);
ov5640_write( 0x885b, 0x82);
ov5640_write( 0x885c, 0xd0);
ov5640_write( 0x885d, 0x83);
ov5640_write( 0x885e, 0xd0);
ov5640_write( 0x885f, 0xe0);
ov5640_write( 0x8860, 0x32);
ov5640_write( 0x8861, 0x90);
ov5640_write( 0x8862, 0x0e);
ov5640_write( 0x8863, 0x7d);
ov5640_write( 0x8864, 0xe4);
ov5640_write( 0x8865, 0x93);
ov5640_write( 0x8866, 0xfe);
ov5640_write( 0x8867, 0x74);
ov5640_write( 0x8868, 0x01);
ov5640_write( 0x8869, 0x93);
ov5640_write( 0x886a, 0xff);
ov5640_write( 0x886b, 0xc3);
ov5640_write( 0x886c, 0x90);
ov5640_write( 0x886d, 0x0e);
ov5640_write( 0x886e, 0x7b);
ov5640_write( 0x886f, 0x74);
ov5640_write( 0x8870, 0x01);
ov5640_write( 0x8871, 0x93);
ov5640_write( 0x8872, 0x9f);
ov5640_write( 0x8873, 0xff);
ov5640_write( 0x8874, 0xe4);
ov5640_write( 0x8875, 0x93);
ov5640_write( 0x8876, 0x9e);
ov5640_write( 0x8877, 0xfe);
ov5640_write( 0x8878, 0xe4);
ov5640_write( 0x8879, 0x8f);
ov5640_write( 0x887a, 0x3b);
ov5640_write( 0x887b, 0x8e);
ov5640_write( 0x887c, 0x3a);
ov5640_write( 0x887d, 0xf5);
ov5640_write( 0x887e, 0x39);
ov5640_write( 0x887f, 0xf5);
ov5640_write( 0x8880, 0x38);
ov5640_write( 0x8881, 0xab);
ov5640_write( 0x8882, 0x3b);
ov5640_write( 0x8883, 0xaa);
ov5640_write( 0x8884, 0x3a);
ov5640_write( 0x8885, 0xa9);
ov5640_write( 0x8886, 0x39);
ov5640_write( 0x8887, 0xa8);
ov5640_write( 0x8888, 0x38);
ov5640_write( 0x8889, 0xaf);
ov5640_write( 0x888a, 0x49);
ov5640_write( 0x888b, 0xfc);
ov5640_write( 0x888c, 0xfd);
ov5640_write( 0x888d, 0xfe);
ov5640_write( 0x888e, 0x12);
ov5640_write( 0x888f, 0x02);
ov5640_write( 0x8890, 0xc6);
ov5640_write( 0x8891, 0x12);
ov5640_write( 0x8892, 0x0b);
ov5640_write( 0x8893, 0x26);
ov5640_write( 0x8894, 0xe4);
ov5640_write( 0x8895, 0x7b);
ov5640_write( 0x8896, 0xff);
ov5640_write( 0x8897, 0xfa);
ov5640_write( 0x8898, 0xf9);
ov5640_write( 0x8899, 0xf8);
ov5640_write( 0x889a, 0x12);
ov5640_write( 0x889b, 0x03);
ov5640_write( 0x889c, 0x51);
ov5640_write( 0x889d, 0x12);
ov5640_write( 0x889e, 0x0b);
ov5640_write( 0x889f, 0x26);
ov5640_write( 0x88a0, 0x90);
ov5640_write( 0x88a1, 0x0e);
ov5640_write( 0x88a2, 0x69);
ov5640_write( 0x88a3, 0xe4);
ov5640_write( 0x88a4, 0x12);
ov5640_write( 0x88a5, 0x0b);
ov5640_write( 0x88a6, 0x3b);
ov5640_write( 0x88a7, 0x12);
ov5640_write( 0x88a8, 0x0b);
ov5640_write( 0x88a9, 0x26);
ov5640_write( 0x88aa, 0xe4);
ov5640_write( 0x88ab, 0x85);
ov5640_write( 0x88ac, 0x48);
ov5640_write( 0x88ad, 0x37);
ov5640_write( 0x88ae, 0xf5);
ov5640_write( 0x88af, 0x36);
ov5640_write( 0x88b0, 0xf5);
ov5640_write( 0x88b1, 0x35);
ov5640_write( 0x88b2, 0xf5);
ov5640_write( 0x88b3, 0x34);
ov5640_write( 0x88b4, 0xaf);
ov5640_write( 0x88b5, 0x37);
ov5640_write( 0x88b6, 0xae);
ov5640_write( 0x88b7, 0x36);
ov5640_write( 0x88b8, 0xad);
ov5640_write( 0x88b9, 0x35);
ov5640_write( 0x88ba, 0xac);
ov5640_write( 0x88bb, 0x34);
ov5640_write( 0x88bc, 0xa3);
ov5640_write( 0x88bd, 0x12);
ov5640_write( 0x88be, 0x0b);
ov5640_write( 0x88bf, 0x3b);
ov5640_write( 0x88c0, 0x8f);
ov5640_write( 0x88c1, 0x37);
ov5640_write( 0x88c2, 0x8e);
ov5640_write( 0x88c3, 0x36);
ov5640_write( 0x88c4, 0x8d);
ov5640_write( 0x88c5, 0x35);
ov5640_write( 0x88c6, 0x8c);
ov5640_write( 0x88c7, 0x34);
ov5640_write( 0x88c8, 0xe5);
ov5640_write( 0x88c9, 0x3b);
ov5640_write( 0x88ca, 0x45);
ov5640_write( 0x88cb, 0x37);
ov5640_write( 0x88cc, 0xf5);
ov5640_write( 0x88cd, 0x3b);
ov5640_write( 0x88ce, 0xe5);
ov5640_write( 0x88cf, 0x3a);
ov5640_write( 0x88d0, 0x45);
ov5640_write( 0x88d1, 0x36);
ov5640_write( 0x88d2, 0xf5);
ov5640_write( 0x88d3, 0x3a);
ov5640_write( 0x88d4, 0xe5);
ov5640_write( 0x88d5, 0x39);
ov5640_write( 0x88d6, 0x45);
ov5640_write( 0x88d7, 0x35);
ov5640_write( 0x88d8, 0xf5);
ov5640_write( 0x88d9, 0x39);
ov5640_write( 0x88da, 0xe5);
ov5640_write( 0x88db, 0x38);
ov5640_write( 0x88dc, 0x45);
ov5640_write( 0x88dd, 0x34);
ov5640_write( 0x88de, 0xf5);
ov5640_write( 0x88df, 0x38);
ov5640_write( 0x88e0, 0xe4);
ov5640_write( 0x88e1, 0xf5);
ov5640_write( 0x88e2, 0x22);
ov5640_write( 0x88e3, 0xf5);
ov5640_write( 0x88e4, 0x23);
ov5640_write( 0x88e5, 0x85);
ov5640_write( 0x88e6, 0x3b);
ov5640_write( 0x88e7, 0x31);
ov5640_write( 0x88e8, 0x85);
ov5640_write( 0x88e9, 0x3a);
ov5640_write( 0x88ea, 0x30);
ov5640_write( 0x88eb, 0x85);
ov5640_write( 0x88ec, 0x39);
ov5640_write( 0x88ed, 0x2f);
ov5640_write( 0x88ee, 0x85);
ov5640_write( 0x88ef, 0x38);
ov5640_write( 0x88f0, 0x2e);
ov5640_write( 0x88f1, 0x02);
ov5640_write( 0x88f2, 0x0a);
ov5640_write( 0x88f3, 0xca);
ov5640_write( 0x88f4, 0x78);
ov5640_write( 0x88f5, 0x4f);
ov5640_write( 0x88f6, 0x7e);
ov5640_write( 0x88f7, 0x00);
ov5640_write( 0x88f8, 0xe6);
ov5640_write( 0x88f9, 0xfc);
ov5640_write( 0x88fa, 0x08);
ov5640_write( 0x88fb, 0xe6);
ov5640_write( 0x88fc, 0xfd);
ov5640_write( 0x88fd, 0x12);
ov5640_write( 0x88fe, 0x02);
ov5640_write( 0x88ff, 0x5f);
ov5640_write( 0x8900, 0x7c);
ov5640_write( 0x8901, 0x00);
ov5640_write( 0x8902, 0x22);
ov5640_write( 0x8903, 0xe0);
ov5640_write( 0x8904, 0xa3);
ov5640_write( 0x8905, 0xe0);
ov5640_write( 0x8906, 0x75);
ov5640_write( 0x8907, 0xf0);
ov5640_write( 0x8908, 0x02);
ov5640_write( 0x8909, 0xa4);
ov5640_write( 0x890a, 0xff);
ov5640_write( 0x890b, 0xae);
ov5640_write( 0x890c, 0xf0);
ov5640_write( 0x890d, 0xc3);
ov5640_write( 0x890e, 0x08);
ov5640_write( 0x890f, 0xe6);
ov5640_write( 0x8910, 0x9f);
ov5640_write( 0x8911, 0xf6);
ov5640_write( 0x8912, 0x18);
ov5640_write( 0x8913, 0xe6);
ov5640_write( 0x8914, 0x9e);
ov5640_write( 0x8915, 0xf6);
ov5640_write( 0x8916, 0x22);
ov5640_write( 0x8917, 0xff);
ov5640_write( 0x8918, 0xe5);
ov5640_write( 0x8919, 0xf0);
ov5640_write( 0x891a, 0x34);
ov5640_write( 0x891b, 0x60);
ov5640_write( 0x891c, 0x8f);
ov5640_write( 0x891d, 0x82);
ov5640_write( 0x891e, 0xf5);
ov5640_write( 0x891f, 0x83);
ov5640_write( 0x8920, 0xec);
ov5640_write( 0x8921, 0xf0);
ov5640_write( 0x8922, 0x22);
ov5640_write( 0x8923, 0xe4);
ov5640_write( 0x8924, 0xfc);
ov5640_write( 0x8925, 0xfd);
ov5640_write( 0x8926, 0x12);
ov5640_write( 0x8927, 0x04);
ov5640_write( 0x8928, 0x24);
ov5640_write( 0x8929, 0x78);
ov5640_write( 0x892a, 0x59);
ov5640_write( 0x892b, 0xe6);
ov5640_write( 0x892c, 0xc3);
ov5640_write( 0x892d, 0x13);
ov5640_write( 0x892e, 0xfe);
ov5640_write( 0x892f, 0x08);
ov5640_write( 0x8930, 0xe6);
ov5640_write( 0x8931, 0x13);
ov5640_write( 0x8932, 0x22);
ov5640_write( 0x8933, 0x78);
ov5640_write( 0x8934, 0x4f);
ov5640_write( 0x8935, 0xe6);
ov5640_write( 0x8936, 0xfe);
ov5640_write( 0x8937, 0x08);
ov5640_write( 0x8938, 0xe6);
ov5640_write( 0x8939, 0xff);
ov5640_write( 0x893a, 0xe4);
ov5640_write( 0x893b, 0xfc);
ov5640_write( 0x893c, 0xfd);
ov5640_write( 0x893d, 0x22);
ov5640_write( 0x893e, 0xe7);
ov5640_write( 0x893f, 0xc4);
ov5640_write( 0x8940, 0xf8);
ov5640_write( 0x8941, 0x54);
ov5640_write( 0x8942, 0xf0);
ov5640_write( 0x8943, 0xc8);
ov5640_write( 0x8944, 0x68);
ov5640_write( 0x8945, 0xf7);
ov5640_write( 0x8946, 0x09);
ov5640_write( 0x8947, 0xe7);
ov5640_write( 0x8948, 0xc4);
ov5640_write( 0x8949, 0x54);
ov5640_write( 0x894a, 0x0f);
ov5640_write( 0x894b, 0x48);
ov5640_write( 0x894c, 0xf7);
ov5640_write( 0x894d, 0x22);
ov5640_write( 0x894e, 0xe6);
ov5640_write( 0x894f, 0xfc);
ov5640_write( 0x8950, 0xed);
ov5640_write( 0x8951, 0x75);
ov5640_write( 0x8952, 0xf0);
ov5640_write( 0x8953, 0x04);
ov5640_write( 0x8954, 0xa4);
ov5640_write( 0x8955, 0x22);
ov5640_write( 0x8956, 0xe0);
ov5640_write( 0x8957, 0xfe);
ov5640_write( 0x8958, 0xa3);
ov5640_write( 0x8959, 0xe0);
ov5640_write( 0x895a, 0xfd);
ov5640_write( 0x895b, 0xee);
ov5640_write( 0x895c, 0xf6);
ov5640_write( 0x895d, 0xed);
ov5640_write( 0x895e, 0x08);
ov5640_write( 0x895f, 0xf6);
ov5640_write( 0x8960, 0x22);
ov5640_write( 0x8961, 0x13);
ov5640_write( 0x8962, 0xff);
ov5640_write( 0x8963, 0xc3);
ov5640_write( 0x8964, 0xe6);
ov5640_write( 0x8965, 0x9f);
ov5640_write( 0x8966, 0xff);
ov5640_write( 0x8967, 0x18);
ov5640_write( 0x8968, 0xe6);
ov5640_write( 0x8969, 0x9e);
ov5640_write( 0x896a, 0xfe);
ov5640_write( 0x896b, 0x22);
ov5640_write( 0x896c, 0xe6);
ov5640_write( 0x896d, 0xc3);
ov5640_write( 0x896e, 0x13);
ov5640_write( 0x896f, 0xf7);
ov5640_write( 0x8970, 0x08);
ov5640_write( 0x8971, 0xe6);
ov5640_write( 0x8972, 0x13);
ov5640_write( 0x8973, 0x09);
ov5640_write( 0x8974, 0xf7);
ov5640_write( 0x8975, 0x22);
ov5640_write( 0x8976, 0xe4);
ov5640_write( 0x8977, 0xf5);
ov5640_write( 0x8978, 0x49);
ov5640_write( 0x8979, 0x90);
ov5640_write( 0x897a, 0x0e);
ov5640_write( 0x897b, 0x77);
ov5640_write( 0x897c, 0x93);
ov5640_write( 0x897d, 0xff);
ov5640_write( 0x897e, 0xe4);
ov5640_write( 0x897f, 0x8f);
ov5640_write( 0x8980, 0x37);
ov5640_write( 0x8981, 0xf5);
ov5640_write( 0x8982, 0x36);
ov5640_write( 0x8983, 0xf5);
ov5640_write( 0x8984, 0x35);
ov5640_write( 0x8985, 0xf5);
ov5640_write( 0x8986, 0x34);
ov5640_write( 0x8987, 0xaf);
ov5640_write( 0x8988, 0x37);
ov5640_write( 0x8989, 0xae);
ov5640_write( 0x898a, 0x36);
ov5640_write( 0x898b, 0xad);
ov5640_write( 0x898c, 0x35);
ov5640_write( 0x898d, 0xac);
ov5640_write( 0x898e, 0x34);
ov5640_write( 0x898f, 0x90);
ov5640_write( 0x8990, 0x0e);
ov5640_write( 0x8991, 0x6a);
ov5640_write( 0x8992, 0x12);
ov5640_write( 0x8993, 0x0b);
ov5640_write( 0x8994, 0x3b);
ov5640_write( 0x8995, 0x8f);
ov5640_write( 0x8996, 0x37);
ov5640_write( 0x8997, 0x8e);
ov5640_write( 0x8998, 0x36);
ov5640_write( 0x8999, 0x8d);
ov5640_write( 0x899a, 0x35);
ov5640_write( 0x899b, 0x8c);
ov5640_write( 0x899c, 0x34);
ov5640_write( 0x899d, 0x90);
ov5640_write( 0x899e, 0x0e);
ov5640_write( 0x899f, 0x72);
ov5640_write( 0x89a0, 0x12);
ov5640_write( 0x89a1, 0x04);
ov5640_write( 0x89a2, 0x07);
ov5640_write( 0x89a3, 0xef);
ov5640_write( 0x89a4, 0x45);
ov5640_write( 0x89a5, 0x37);
ov5640_write( 0x89a6, 0xf5);
ov5640_write( 0x89a7, 0x37);
ov5640_write( 0x89a8, 0xee);
ov5640_write( 0x89a9, 0x45);
ov5640_write( 0x89aa, 0x36);
ov5640_write( 0x89ab, 0xf5);
ov5640_write( 0x89ac, 0x36);
ov5640_write( 0x89ad, 0xed);
ov5640_write( 0x89ae, 0x45);
ov5640_write( 0x89af, 0x35);
ov5640_write( 0x89b0, 0xf5);
ov5640_write( 0x89b1, 0x35);
ov5640_write( 0x89b2, 0xec);
ov5640_write( 0x89b3, 0x45);
ov5640_write( 0x89b4, 0x34);
ov5640_write( 0x89b5, 0xf5);
ov5640_write( 0x89b6, 0x34);
ov5640_write( 0x89b7, 0xe4);
ov5640_write( 0x89b8, 0xf5);
ov5640_write( 0x89b9, 0x22);
ov5640_write( 0x89ba, 0xf5);
ov5640_write( 0x89bb, 0x23);
ov5640_write( 0x89bc, 0x85);
ov5640_write( 0x89bd, 0x37);
ov5640_write( 0x89be, 0x31);
ov5640_write( 0x89bf, 0x85);
ov5640_write( 0x89c0, 0x36);
ov5640_write( 0x89c1, 0x30);
ov5640_write( 0x89c2, 0x85);
ov5640_write( 0x89c3, 0x35);
ov5640_write( 0x89c4, 0x2f);
ov5640_write( 0x89c5, 0x85);
ov5640_write( 0x89c6, 0x34);
ov5640_write( 0x89c7, 0x2e);
ov5640_write( 0x89c8, 0x12);
ov5640_write( 0x89c9, 0x0a);
ov5640_write( 0x89ca, 0xca);
ov5640_write( 0x89cb, 0xe4);
ov5640_write( 0x89cc, 0xf5);
ov5640_write( 0x89cd, 0x22);
ov5640_write( 0x89ce, 0xf5);
ov5640_write( 0x89cf, 0x23);
ov5640_write( 0x89d0, 0x90);
ov5640_write( 0x89d1, 0x0e);
ov5640_write( 0x89d2, 0x72);
ov5640_write( 0x89d3, 0x12);
ov5640_write( 0x89d4, 0x0b);
ov5640_write( 0x89d5, 0x2f);
ov5640_write( 0x89d6, 0x12);
ov5640_write( 0x89d7, 0x0a);
ov5640_write( 0x89d8, 0xca);
ov5640_write( 0x89d9, 0xe4);
ov5640_write( 0x89da, 0xf5);
ov5640_write( 0x89db, 0x22);
ov5640_write( 0x89dc, 0xf5);
ov5640_write( 0x89dd, 0x23);
ov5640_write( 0x89de, 0x90);
ov5640_write( 0x89df, 0x0e);
ov5640_write( 0x89e0, 0x6e);
ov5640_write( 0x89e1, 0x12);
ov5640_write( 0x89e2, 0x0b);
ov5640_write( 0x89e3, 0x2f);
ov5640_write( 0x89e4, 0x02);
ov5640_write( 0x89e5, 0x0a);
ov5640_write( 0x89e6, 0xca);
ov5640_write( 0x89e7, 0x75);
ov5640_write( 0x89e8, 0x89);
ov5640_write( 0x89e9, 0x03);
ov5640_write( 0x89ea, 0x75);
ov5640_write( 0x89eb, 0xa8);
ov5640_write( 0x89ec, 0x01);
ov5640_write( 0x89ed, 0x75);
ov5640_write( 0x89ee, 0xb8);
ov5640_write( 0x89ef, 0x04);
ov5640_write( 0x89f0, 0x75);
ov5640_write( 0x89f1, 0x34);
ov5640_write( 0x89f2, 0xff);
ov5640_write( 0x89f3, 0x75);
ov5640_write( 0x89f4, 0x35);
ov5640_write( 0x89f5, 0x0e);
ov5640_write( 0x89f6, 0x75);
ov5640_write( 0x89f7, 0x36);
ov5640_write( 0x89f8, 0x15);
ov5640_write( 0x89f9, 0x75);
ov5640_write( 0x89fa, 0x37);
ov5640_write( 0x89fb, 0x0d);
ov5640_write( 0x89fc, 0x12);
ov5640_write( 0x89fd, 0x0a);
ov5640_write( 0x89fe, 0x50);
ov5640_write( 0x89ff, 0x12);
ov5640_write( 0x8a00, 0x00);
ov5640_write( 0x8a01, 0x06);
ov5640_write( 0x8a02, 0x12);
ov5640_write( 0x8a03, 0x0b);
ov5640_write( 0x8a04, 0x41);
ov5640_write( 0x8a05, 0x12);
ov5640_write( 0x8a06, 0x0b);
ov5640_write( 0x8a07, 0x7f);
ov5640_write( 0x8a08, 0xd2);
ov5640_write( 0x8a09, 0x00);
ov5640_write( 0x8a0a, 0xd2);
ov5640_write( 0x8a0b, 0x34);
ov5640_write( 0x8a0c, 0xd2);
ov5640_write( 0x8a0d, 0xaf);
ov5640_write( 0x8a0e, 0x75);
ov5640_write( 0x8a0f, 0x34);
ov5640_write( 0x8a10, 0xff);
ov5640_write( 0x8a11, 0x75);
ov5640_write( 0x8a12, 0x35);
ov5640_write( 0x8a13, 0x0e);
ov5640_write( 0x8a14, 0x75);
ov5640_write( 0x8a15, 0x36);
ov5640_write( 0x8a16, 0x49);
ov5640_write( 0x8a17, 0x75);
ov5640_write( 0x8a18, 0x37);
ov5640_write( 0x8a19, 0x03);
ov5640_write( 0x8a1a, 0x12);
ov5640_write( 0x8a1b, 0x0a);
ov5640_write( 0x8a1c, 0x50);
ov5640_write( 0x8a1d, 0x30);
ov5640_write( 0x8a1e, 0x08);
ov5640_write( 0x8a1f, 0x09);
ov5640_write( 0x8a20, 0xc2);
ov5640_write( 0x8a21, 0x34);
ov5640_write( 0x8a22, 0x12);
ov5640_write( 0x8a23, 0x06);
ov5640_write( 0x8a24, 0x48);
ov5640_write( 0x8a25, 0xc2);
ov5640_write( 0x8a26, 0x08);
ov5640_write( 0x8a27, 0xd2);
ov5640_write( 0x8a28, 0x34);
ov5640_write( 0x8a29, 0x30);
ov5640_write( 0x8a2a, 0x09);
ov5640_write( 0x8a2b, 0x09);
ov5640_write( 0x8a2c, 0xc2);
ov5640_write( 0x8a2d, 0x36);
ov5640_write( 0x8a2e, 0x12);
ov5640_write( 0x8a2f, 0x00);
ov5640_write( 0x8a30, 0x0e);
ov5640_write( 0x8a31, 0xc2);
ov5640_write( 0x8a32, 0x09);
ov5640_write( 0x8a33, 0xd2);
ov5640_write( 0x8a34, 0x36);
ov5640_write( 0x8a35, 0x30);
ov5640_write( 0x8a36, 0x0e);
ov5640_write( 0x8a37, 0x03);
ov5640_write( 0x8a38, 0x12);
ov5640_write( 0x8a39, 0x04);
ov5640_write( 0x8a3a, 0x62);
ov5640_write( 0x8a3b, 0x30);
ov5640_write( 0x8a3c, 0x35);
ov5640_write( 0x8a3d, 0xdf);
ov5640_write( 0x8a3e, 0x90);
ov5640_write( 0x8a3f, 0x30);
ov5640_write( 0x8a40, 0x29);
ov5640_write( 0x8a41, 0xe5);
ov5640_write( 0x8a42, 0x1e);
ov5640_write( 0x8a43, 0xf0);
ov5640_write( 0x8a44, 0xb4);
ov5640_write( 0x8a45, 0x10);
ov5640_write( 0x8a46, 0x05);
ov5640_write( 0x8a47, 0x90);
ov5640_write( 0x8a48, 0x30);
ov5640_write( 0x8a49, 0x23);
ov5640_write( 0x8a4a, 0xe4);
ov5640_write( 0x8a4b, 0xf0);
ov5640_write( 0x8a4c, 0xc2);
ov5640_write( 0x8a4d, 0x35);
ov5640_write( 0x8a4e, 0x80);
ov5640_write( 0x8a4f, 0xcd);
ov5640_write( 0x8a50, 0xae);
ov5640_write( 0x8a51, 0x35);
ov5640_write( 0x8a52, 0xaf);
ov5640_write( 0x8a53, 0x36);
ov5640_write( 0x8a54, 0xe4);
ov5640_write( 0x8a55, 0xfd);
ov5640_write( 0x8a56, 0xed);
ov5640_write( 0x8a57, 0xc3);
ov5640_write( 0x8a58, 0x95);
ov5640_write( 0x8a59, 0x37);
ov5640_write( 0x8a5a, 0x50);
ov5640_write( 0x8a5b, 0x33);
ov5640_write( 0x8a5c, 0x12);
ov5640_write( 0x8a5d, 0x0b);
ov5640_write( 0x8a5e, 0x97);
ov5640_write( 0x8a5f, 0xe4);
ov5640_write( 0x8a60, 0x93);
ov5640_write( 0x8a61, 0xf5);
ov5640_write( 0x8a62, 0x38);
ov5640_write( 0x8a63, 0x74);
ov5640_write( 0x8a64, 0x01);
ov5640_write( 0x8a65, 0x93);
ov5640_write( 0x8a66, 0xf5);
ov5640_write( 0x8a67, 0x39);
ov5640_write( 0x8a68, 0x45);
ov5640_write( 0x8a69, 0x38);
ov5640_write( 0x8a6a, 0x60);
ov5640_write( 0x8a6b, 0x23);
ov5640_write( 0x8a6c, 0x85);
ov5640_write( 0x8a6d, 0x39);
ov5640_write( 0x8a6e, 0x82);
ov5640_write( 0x8a6f, 0x85);
ov5640_write( 0x8a70, 0x38);
ov5640_write( 0x8a71, 0x83);
ov5640_write( 0x8a72, 0xe0);
ov5640_write( 0x8a73, 0xfc);
ov5640_write( 0x8a74, 0x12);
ov5640_write( 0x8a75, 0x0b);
ov5640_write( 0x8a76, 0x97);
ov5640_write( 0x8a77, 0x74);
ov5640_write( 0x8a78, 0x03);
ov5640_write( 0x8a79, 0x93);
ov5640_write( 0x8a7a, 0x52);
ov5640_write( 0x8a7b, 0x04);
ov5640_write( 0x8a7c, 0x12);
ov5640_write( 0x8a7d, 0x0b);
ov5640_write( 0x8a7e, 0x97);
ov5640_write( 0x8a7f, 0x74);
ov5640_write( 0x8a80, 0x02);
ov5640_write( 0x8a81, 0x93);
ov5640_write( 0x8a82, 0x42);
ov5640_write( 0x8a83, 0x04);
ov5640_write( 0x8a84, 0x85);
ov5640_write( 0x8a85, 0x39);
ov5640_write( 0x8a86, 0x82);
ov5640_write( 0x8a87, 0x85);
ov5640_write( 0x8a88, 0x38);
ov5640_write( 0x8a89, 0x83);
ov5640_write( 0x8a8a, 0xec);
ov5640_write( 0x8a8b, 0xf0);
ov5640_write( 0x8a8c, 0x0d);
ov5640_write( 0x8a8d, 0x80);
ov5640_write( 0x8a8e, 0xc7);
ov5640_write( 0x8a8f, 0x22);
ov5640_write( 0x8a90, 0x78);
ov5640_write( 0x8a91, 0xbb);
ov5640_write( 0x8a92, 0xe6);
ov5640_write( 0x8a93, 0xd3);
ov5640_write( 0x8a94, 0x08);
ov5640_write( 0x8a95, 0xff);
ov5640_write( 0x8a96, 0xe6);
ov5640_write( 0x8a97, 0x64);
ov5640_write( 0x8a98, 0x80);
ov5640_write( 0x8a99, 0xf8);
ov5640_write( 0x8a9a, 0xef);
ov5640_write( 0x8a9b, 0x64);
ov5640_write( 0x8a9c, 0x80);
ov5640_write( 0x8a9d, 0x98);
ov5640_write( 0x8a9e, 0x22);
ov5640_write( 0x8a9f, 0x93);
ov5640_write( 0x8aa0, 0xff);
ov5640_write( 0x8aa1, 0x7e);
ov5640_write( 0x8aa2, 0x00);
ov5640_write( 0x8aa3, 0xe6);
ov5640_write( 0x8aa4, 0xfc);
ov5640_write( 0x8aa5, 0x08);
ov5640_write( 0x8aa6, 0xe6);
ov5640_write( 0x8aa7, 0xfd);
ov5640_write( 0x8aa8, 0x12);
ov5640_write( 0x8aa9, 0x02);
ov5640_write( 0x8aaa, 0x5f);
ov5640_write( 0x8aab, 0x78);
ov5640_write( 0x8aac, 0xbe);
ov5640_write( 0x8aad, 0xe6);
ov5640_write( 0x8aae, 0xfc);
ov5640_write( 0x8aaf, 0x08);
ov5640_write( 0x8ab0, 0xe6);
ov5640_write( 0x8ab1, 0xfd);
ov5640_write( 0x8ab2, 0xd3);
ov5640_write( 0x8ab3, 0xef);
ov5640_write( 0x8ab4, 0x9d);
ov5640_write( 0x8ab5, 0xee);
ov5640_write( 0x8ab6, 0x9c);
ov5640_write( 0x8ab7, 0x22);
ov5640_write( 0x8ab8, 0x25);
ov5640_write( 0x8ab9, 0xe0);
ov5640_write( 0x8aba, 0x24);
ov5640_write( 0x8abb, 0x0a);
ov5640_write( 0x8abc, 0xf8);
ov5640_write( 0x8abd, 0xe6);
ov5640_write( 0x8abe, 0xfe);
ov5640_write( 0x8abf, 0x08);
ov5640_write( 0x8ac0, 0xe6);
ov5640_write( 0x8ac1, 0xff);
ov5640_write( 0x8ac2, 0x22);
ov5640_write( 0x8ac3, 0xd3);
ov5640_write( 0x8ac4, 0xe6);
ov5640_write( 0x8ac5, 0x64);
ov5640_write( 0x8ac6, 0x80);
ov5640_write( 0x8ac7, 0x94);
ov5640_write( 0x8ac8, 0x80);
ov5640_write( 0x8ac9, 0x22);
ov5640_write( 0x8aca, 0xa2);
ov5640_write( 0x8acb, 0xaf);
ov5640_write( 0x8acc, 0x92);
ov5640_write( 0x8acd, 0x32);
ov5640_write( 0x8ace, 0xc2);
ov5640_write( 0x8acf, 0xaf);
ov5640_write( 0x8ad0, 0xe5);
ov5640_write( 0x8ad1, 0x23);
ov5640_write( 0x8ad2, 0x45);
ov5640_write( 0x8ad3, 0x22);
ov5640_write( 0x8ad4, 0x90);
ov5640_write( 0x8ad5, 0x0e);
ov5640_write( 0x8ad6, 0x5d);
ov5640_write( 0x8ad7, 0x60);
ov5640_write( 0x8ad8, 0x0e);
ov5640_write( 0x8ad9, 0x12);
ov5640_write( 0x8ada, 0x0b);
ov5640_write( 0x8adb, 0x74);
ov5640_write( 0x8adc, 0xe0);
ov5640_write( 0x8add, 0xf5);
ov5640_write( 0x8ade, 0x2c);
ov5640_write( 0x8adf, 0x12);
ov5640_write( 0x8ae0, 0x0b);
ov5640_write( 0x8ae1, 0x71);
ov5640_write( 0x8ae2, 0xe0);
ov5640_write( 0x8ae3, 0xf5);
ov5640_write( 0x8ae4, 0x2d);
ov5640_write( 0x8ae5, 0x80);
ov5640_write( 0x8ae6, 0x0c);
ov5640_write( 0x8ae7, 0x12);
ov5640_write( 0x8ae8, 0x0b);
ov5640_write( 0x8ae9, 0x74);
ov5640_write( 0x8aea, 0xe5);
ov5640_write( 0x8aeb, 0x30);
ov5640_write( 0x8aec, 0xf0);
ov5640_write( 0x8aed, 0x12);
ov5640_write( 0x8aee, 0x0b);
ov5640_write( 0x8aef, 0x71);
ov5640_write( 0x8af0, 0xe5);
ov5640_write( 0x8af1, 0x31);
ov5640_write( 0x8af2, 0xf0);
ov5640_write( 0x8af3, 0xa2);
ov5640_write( 0x8af4, 0x32);
ov5640_write( 0x8af5, 0x92);
ov5640_write( 0x8af6, 0xaf);
ov5640_write( 0x8af7, 0x22);
ov5640_write( 0x8af8, 0xd2);
ov5640_write( 0x8af9, 0x01);
ov5640_write( 0x8afa, 0xc2);
ov5640_write( 0x8afb, 0x02);
ov5640_write( 0x8afc, 0xe4);
ov5640_write( 0x8afd, 0xf5);
ov5640_write( 0x8afe, 0x1f);
ov5640_write( 0x8aff, 0xf5);
ov5640_write( 0x8b00, 0x1e);
ov5640_write( 0x8b01, 0xd2);
ov5640_write( 0x8b02, 0x35);
ov5640_write( 0x8b03, 0xd2);
ov5640_write( 0x8b04, 0x33);
ov5640_write( 0x8b05, 0xd2);
ov5640_write( 0x8b06, 0x36);
ov5640_write( 0x8b07, 0xd2);
ov5640_write( 0x8b08, 0x01);
ov5640_write( 0x8b09, 0xc2);
ov5640_write( 0x8b0a, 0x02);
ov5640_write( 0x8b0b, 0xf5);
ov5640_write( 0x8b0c, 0x1f);
ov5640_write( 0x8b0d, 0xf5);
ov5640_write( 0x8b0e, 0x1e);
ov5640_write( 0x8b0f, 0xd2);
ov5640_write( 0x8b10, 0x35);
ov5640_write( 0x8b11, 0xd2);
ov5640_write( 0x8b12, 0x33);
ov5640_write( 0x8b13, 0x22);
ov5640_write( 0x8b14, 0x2d);
ov5640_write( 0x8b15, 0xfd);
ov5640_write( 0x8b16, 0xe4);
ov5640_write( 0x8b17, 0x33);
ov5640_write( 0x8b18, 0xfc);
ov5640_write( 0x8b19, 0xe4);
ov5640_write( 0x8b1a, 0x93);
ov5640_write( 0x8b1b, 0xfe);
ov5640_write( 0x8b1c, 0xfb);
ov5640_write( 0x8b1d, 0xd3);
ov5640_write( 0x8b1e, 0xed);
ov5640_write( 0x8b1f, 0x9b);
ov5640_write( 0x8b20, 0x74);
ov5640_write( 0x8b21, 0x80);
ov5640_write( 0x8b22, 0xf8);
ov5640_write( 0x8b23, 0x6c);
ov5640_write( 0x8b24, 0x98);
ov5640_write( 0x8b25, 0x22);
ov5640_write( 0x8b26, 0x8f);
ov5640_write( 0x8b27, 0x3b);
ov5640_write( 0x8b28, 0x8e);
ov5640_write( 0x8b29, 0x3a);
ov5640_write( 0x8b2a, 0x8d);
ov5640_write( 0x8b2b, 0x39);
ov5640_write( 0x8b2c, 0x8c);
ov5640_write( 0x8b2d, 0x38);
ov5640_write( 0x8b2e, 0x22);
ov5640_write( 0x8b2f, 0x12);
ov5640_write( 0x8b30, 0x04);
ov5640_write( 0x8b31, 0x07);
ov5640_write( 0x8b32, 0x8f);
ov5640_write( 0x8b33, 0x31);
ov5640_write( 0x8b34, 0x8e);
ov5640_write( 0x8b35, 0x30);
ov5640_write( 0x8b36, 0x8d);
ov5640_write( 0x8b37, 0x2f);
ov5640_write( 0x8b38, 0x8c);
ov5640_write( 0x8b39, 0x2e);
ov5640_write( 0x8b3a, 0x22);
ov5640_write( 0x8b3b, 0x93);
ov5640_write( 0x8b3c, 0xf9);
ov5640_write( 0x8b3d, 0xf8);
ov5640_write( 0x8b3e, 0x02);
ov5640_write( 0x8b3f, 0x03);
ov5640_write( 0x8b40, 0xf4);
ov5640_write( 0x8b41, 0x90);
ov5640_write( 0x8b42, 0x0e);
ov5640_write( 0x8b43, 0x81);
ov5640_write( 0x8b44, 0x12);
ov5640_write( 0x8b45, 0x04);
ov5640_write( 0x8b46, 0x07);
ov5640_write( 0x8b47, 0x8f);
ov5640_write( 0x8b48, 0x46);
ov5640_write( 0x8b49, 0x8e);
ov5640_write( 0x8b4a, 0x45);
ov5640_write( 0x8b4b, 0x8d);
ov5640_write( 0x8b4c, 0x44);
ov5640_write( 0x8b4d, 0x8c);
ov5640_write( 0x8b4e, 0x43);
ov5640_write( 0x8b4f, 0xd2);
ov5640_write( 0x8b50, 0x06);
ov5640_write( 0x8b51, 0x30);
ov5640_write( 0x8b52, 0x06);
ov5640_write( 0x8b53, 0x03);
ov5640_write( 0x8b54, 0xd3);
ov5640_write( 0x8b55, 0x80);
ov5640_write( 0x8b56, 0x01);
ov5640_write( 0x8b57, 0xc3);
ov5640_write( 0x8b58, 0x92);
ov5640_write( 0x8b59, 0x0e);
ov5640_write( 0x8b5a, 0x22);
ov5640_write( 0x8b5b, 0xc0);
ov5640_write( 0x8b5c, 0xe0);
ov5640_write( 0x8b5d, 0xc0);
ov5640_write( 0x8b5e, 0x83);
ov5640_write( 0x8b5f, 0xc0);
ov5640_write( 0x8b60, 0x82);
ov5640_write( 0x8b61, 0x90);
ov5640_write( 0x8b62, 0x3f);
ov5640_write( 0x8b63, 0x0d);
ov5640_write( 0x8b64, 0xe0);
ov5640_write( 0x8b65, 0xf5);
ov5640_write( 0x8b66, 0x33);
ov5640_write( 0x8b67, 0xe5);
ov5640_write( 0x8b68, 0x33);
ov5640_write( 0x8b69, 0xf0);
ov5640_write( 0x8b6a, 0xd0);
ov5640_write( 0x8b6b, 0x82);
ov5640_write( 0x8b6c, 0xd0);
ov5640_write( 0x8b6d, 0x83);
ov5640_write( 0x8b6e, 0xd0);
ov5640_write( 0x8b6f, 0xe0);
ov5640_write( 0x8b70, 0x32);
ov5640_write( 0x8b71, 0x90);
ov5640_write( 0x8b72, 0x0e);
ov5640_write( 0x8b73, 0x5f);
ov5640_write( 0x8b74, 0xe4);
ov5640_write( 0x8b75, 0x93);
ov5640_write( 0x8b76, 0xfe);
ov5640_write( 0x8b77, 0x74);
ov5640_write( 0x8b78, 0x01);
ov5640_write( 0x8b79, 0x93);
ov5640_write( 0x8b7a, 0xf5);
ov5640_write( 0x8b7b, 0x82);
ov5640_write( 0x8b7c, 0x8e);
ov5640_write( 0x8b7d, 0x83);
ov5640_write( 0x8b7e, 0x22);
ov5640_write( 0x8b7f, 0xe4);
ov5640_write( 0x8b80, 0xf5);
ov5640_write( 0x8b81, 0x49);
ov5640_write( 0x8b82, 0x12);
ov5640_write( 0x8b83, 0x08);
ov5640_write( 0x8b84, 0x61);
ov5640_write( 0x8b85, 0x12);
ov5640_write( 0x8b86, 0x09);
ov5640_write( 0x8b87, 0x76);
ov5640_write( 0x8b88, 0xc2);
ov5640_write( 0x8b89, 0x01);
ov5640_write( 0x8b8a, 0x22);
ov5640_write( 0x8b8b, 0x78);
ov5640_write( 0x8b8c, 0x7f);
ov5640_write( 0x8b8d, 0xe4);
ov5640_write( 0x8b8e, 0xf6);
ov5640_write( 0x8b8f, 0xd8);
ov5640_write( 0x8b90, 0xfd);
ov5640_write( 0x8b91, 0x75);
ov5640_write( 0x8b92, 0x81);
ov5640_write( 0x8b93, 0xca);
ov5640_write( 0x8b94, 0x02);
ov5640_write( 0x8b95, 0x09);
ov5640_write( 0x8b96, 0xe7);
ov5640_write( 0x8b97, 0x8f);
ov5640_write( 0x8b98, 0x82);
ov5640_write( 0x8b99, 0x8e);
ov5640_write( 0x8b9a, 0x83);
ov5640_write( 0x8b9b, 0x75);
ov5640_write( 0x8b9c, 0xf0);
ov5640_write( 0x8b9d, 0x04);
ov5640_write( 0x8b9e, 0xed);
ov5640_write( 0x8b9f, 0x02);
ov5640_write( 0x8ba0, 0x04);
ov5640_write( 0x8ba1, 0x30);
ov5640_write( 0x8ba2, 0x00);
ov5640_write( 0x8ba3, 0x00);
ov5640_write( 0x8ba4, 0x00);
ov5640_write( 0x8ba5, 0x00);
ov5640_write( 0x8ba6, 0x00);
ov5640_write( 0x8ba7, 0x00);
ov5640_write( 0x8ba8, 0x00);
ov5640_write( 0x8ba9, 0x00);
ov5640_write( 0x8baa, 0x00);
ov5640_write( 0x8bab, 0x00);
ov5640_write( 0x8bac, 0x00);
ov5640_write( 0x8bad, 0x00);
ov5640_write( 0x8bae, 0x00);
ov5640_write( 0x8baf, 0x00);
ov5640_write( 0x8bb0, 0x00);
ov5640_write( 0x8bb1, 0x00);
ov5640_write( 0x8bb2, 0x00);
ov5640_write( 0x8bb3, 0x00);
ov5640_write( 0x8bb4, 0x00);
ov5640_write( 0x8bb5, 0x00);
ov5640_write( 0x8bb6, 0x00);
ov5640_write( 0x8bb7, 0x00);
ov5640_write( 0x8bb8, 0x00);
ov5640_write( 0x8bb9, 0x00);
ov5640_write( 0x8bba, 0x00);
ov5640_write( 0x8bbb, 0x00);
ov5640_write( 0x8bbc, 0x00);
ov5640_write( 0x8bbd, 0x00);
ov5640_write( 0x8bbe, 0x00);
ov5640_write( 0x8bbf, 0x00);
ov5640_write( 0x8bc0, 0x00);
ov5640_write( 0x8bc1, 0x00);
ov5640_write( 0x8bc2, 0x00);
ov5640_write( 0x8bc3, 0x00);
ov5640_write( 0x8bc4, 0x00);
ov5640_write( 0x8bc5, 0x00);
ov5640_write( 0x8bc6, 0x00);
ov5640_write( 0x8bc7, 0x00);
ov5640_write( 0x8bc8, 0x00);
ov5640_write( 0x8bc9, 0x00);
ov5640_write( 0x8bca, 0x00);
ov5640_write( 0x8bcb, 0x00);
ov5640_write( 0x8bcc, 0x00);
ov5640_write( 0x8bcd, 0x00);
ov5640_write( 0x8bce, 0x00);
ov5640_write( 0x8bcf, 0x00);
ov5640_write( 0x8bd0, 0x00);
ov5640_write( 0x8bd1, 0x00);
ov5640_write( 0x8bd2, 0x00);
ov5640_write( 0x8bd3, 0x00);
ov5640_write( 0x8bd4, 0x00);
ov5640_write( 0x8bd5, 0x00);
ov5640_write( 0x8bd6, 0x00);
ov5640_write( 0x8bd7, 0x00);
ov5640_write( 0x8bd8, 0x00);
ov5640_write( 0x8bd9, 0x00);
ov5640_write( 0x8bda, 0x00);
ov5640_write( 0x8bdb, 0x00);
ov5640_write( 0x8bdc, 0x00);
ov5640_write( 0x8bdd, 0x00);
ov5640_write( 0x8bde, 0x00);
ov5640_write( 0x8bdf, 0x00);
ov5640_write( 0x8be0, 0x00);
ov5640_write( 0x8be1, 0x00);
ov5640_write( 0x8be2, 0x00);
ov5640_write( 0x8be3, 0x00);
ov5640_write( 0x8be4, 0x00);
ov5640_write( 0x8be5, 0x00);
ov5640_write( 0x8be6, 0x00);
ov5640_write( 0x8be7, 0x00);
ov5640_write( 0x8be8, 0x00);
ov5640_write( 0x8be9, 0x00);
ov5640_write( 0x8bea, 0x00);
ov5640_write( 0x8beb, 0x00);
ov5640_write( 0x8bec, 0x00);
ov5640_write( 0x8bed, 0x00);
ov5640_write( 0x8bee, 0x00);
ov5640_write( 0x8bef, 0x00);
ov5640_write( 0x8bf0, 0x00);
ov5640_write( 0x8bf1, 0x00);
ov5640_write( 0x8bf2, 0x00);
ov5640_write( 0x8bf3, 0x00);
ov5640_write( 0x8bf4, 0x00);
ov5640_write( 0x8bf5, 0x00);
ov5640_write( 0x8bf6, 0x00);
ov5640_write( 0x8bf7, 0x00);
ov5640_write( 0x8bf8, 0x00);
ov5640_write( 0x8bf9, 0x00);
ov5640_write( 0x8bfa, 0x00);
ov5640_write( 0x8bfb, 0x00);
ov5640_write( 0x8bfc, 0x00);
ov5640_write( 0x8bfd, 0x00);
ov5640_write( 0x8bfe, 0x00);
ov5640_write( 0x8bff, 0x00);
ov5640_write( 0x8c00, 0x00);
ov5640_write( 0x8c01, 0x00);
ov5640_write( 0x8c02, 0x00);
ov5640_write( 0x8c03, 0x00);
ov5640_write( 0x8c04, 0x00);
ov5640_write( 0x8c05, 0x00);
ov5640_write( 0x8c06, 0x00);
ov5640_write( 0x8c07, 0x00);
ov5640_write( 0x8c08, 0x00);
ov5640_write( 0x8c09, 0x00);
ov5640_write( 0x8c0a, 0x00);
ov5640_write( 0x8c0b, 0x00);
ov5640_write( 0x8c0c, 0x00);
ov5640_write( 0x8c0d, 0x00);
ov5640_write( 0x8c0e, 0x00);
ov5640_write( 0x8c0f, 0x00);
ov5640_write( 0x8c10, 0x00);
ov5640_write( 0x8c11, 0x00);
ov5640_write( 0x8c12, 0x00);
ov5640_write( 0x8c13, 0x00);
ov5640_write( 0x8c14, 0x00);
ov5640_write( 0x8c15, 0x00);
ov5640_write( 0x8c16, 0x00);
ov5640_write( 0x8c17, 0x00);
ov5640_write( 0x8c18, 0x00);
ov5640_write( 0x8c19, 0x00);
ov5640_write( 0x8c1a, 0x00);
ov5640_write( 0x8c1b, 0x00);
ov5640_write( 0x8c1c, 0x00);
ov5640_write( 0x8c1d, 0x00);
ov5640_write( 0x8c1e, 0x00);
ov5640_write( 0x8c1f, 0x00);
ov5640_write( 0x8c20, 0x00);
ov5640_write( 0x8c21, 0x00);
ov5640_write( 0x8c22, 0x00);
ov5640_write( 0x8c23, 0x00);
ov5640_write( 0x8c24, 0x00);
ov5640_write( 0x8c25, 0x00);
ov5640_write( 0x8c26, 0x00);
ov5640_write( 0x8c27, 0x00);
ov5640_write( 0x8c28, 0x00);
ov5640_write( 0x8c29, 0x00);
ov5640_write( 0x8c2a, 0x00);
ov5640_write( 0x8c2b, 0x00);
ov5640_write( 0x8c2c, 0x00);
ov5640_write( 0x8c2d, 0x00);
ov5640_write( 0x8c2e, 0x00);
ov5640_write( 0x8c2f, 0x00);
ov5640_write( 0x8c30, 0x00);
ov5640_write( 0x8c31, 0x00);
ov5640_write( 0x8c32, 0x00);
ov5640_write( 0x8c33, 0x00);
ov5640_write( 0x8c34, 0x00);
ov5640_write( 0x8c35, 0x00);
ov5640_write( 0x8c36, 0x00);
ov5640_write( 0x8c37, 0x00);
ov5640_write( 0x8c38, 0x00);
ov5640_write( 0x8c39, 0x00);
ov5640_write( 0x8c3a, 0x00);
ov5640_write( 0x8c3b, 0x00);
ov5640_write( 0x8c3c, 0x00);
ov5640_write( 0x8c3d, 0x00);
ov5640_write( 0x8c3e, 0x00);
ov5640_write( 0x8c3f, 0x00);
ov5640_write( 0x8c40, 0x00);
ov5640_write( 0x8c41, 0x00);
ov5640_write( 0x8c42, 0x00);
ov5640_write( 0x8c43, 0x00);
ov5640_write( 0x8c44, 0x00);
ov5640_write( 0x8c45, 0x00);
ov5640_write( 0x8c46, 0x00);
ov5640_write( 0x8c47, 0x00);
ov5640_write( 0x8c48, 0x00);
ov5640_write( 0x8c49, 0x00);
ov5640_write( 0x8c4a, 0x00);
ov5640_write( 0x8c4b, 0x00);
ov5640_write( 0x8c4c, 0x00);
ov5640_write( 0x8c4d, 0x00);
ov5640_write( 0x8c4e, 0x00);
ov5640_write( 0x8c4f, 0x00);
ov5640_write( 0x8c50, 0x00);
ov5640_write( 0x8c51, 0x00);
ov5640_write( 0x8c52, 0x00);
ov5640_write( 0x8c53, 0x00);
ov5640_write( 0x8c54, 0x00);
ov5640_write( 0x8c55, 0x00);
ov5640_write( 0x8c56, 0x00);
ov5640_write( 0x8c57, 0x00);
ov5640_write( 0x8c58, 0x00);
ov5640_write( 0x8c59, 0x00);
ov5640_write( 0x8c5a, 0x00);
ov5640_write( 0x8c5b, 0x00);
ov5640_write( 0x8c5c, 0x00);
ov5640_write( 0x8c5d, 0x00);
ov5640_write( 0x8c5e, 0x00);
ov5640_write( 0x8c5f, 0x00);
ov5640_write( 0x8c60, 0x00);
ov5640_write( 0x8c61, 0x00);
ov5640_write( 0x8c62, 0x00);
ov5640_write( 0x8c63, 0x00);
ov5640_write( 0x8c64, 0x00);
ov5640_write( 0x8c65, 0x00);
ov5640_write( 0x8c66, 0x00);
ov5640_write( 0x8c67, 0x00);
ov5640_write( 0x8c68, 0x00);
ov5640_write( 0x8c69, 0x00);
ov5640_write( 0x8c6a, 0x00);
ov5640_write( 0x8c6b, 0x00);
ov5640_write( 0x8c6c, 0x00);
ov5640_write( 0x8c6d, 0x00);
ov5640_write( 0x8c6e, 0x00);
ov5640_write( 0x8c6f, 0x00);
ov5640_write( 0x8c70, 0x00);
ov5640_write( 0x8c71, 0x00);
ov5640_write( 0x8c72, 0x00);
ov5640_write( 0x8c73, 0x00);
ov5640_write( 0x8c74, 0x00);
ov5640_write( 0x8c75, 0x00);
ov5640_write( 0x8c76, 0x00);
ov5640_write( 0x8c77, 0x00);
ov5640_write( 0x8c78, 0x00);
ov5640_write( 0x8c79, 0x00);
ov5640_write( 0x8c7a, 0x00);
ov5640_write( 0x8c7b, 0x00);
ov5640_write( 0x8c7c, 0x00);
ov5640_write( 0x8c7d, 0x00);
ov5640_write( 0x8c7e, 0x00);
ov5640_write( 0x8c7f, 0x00);
ov5640_write( 0x8c80, 0x00);
ov5640_write( 0x8c81, 0x00);
ov5640_write( 0x8c82, 0x00);
ov5640_write( 0x8c83, 0x00);
ov5640_write( 0x8c84, 0x00);
ov5640_write( 0x8c85, 0x00);
ov5640_write( 0x8c86, 0x00);
ov5640_write( 0x8c87, 0x00);
ov5640_write( 0x8c88, 0x00);
ov5640_write( 0x8c89, 0x00);
ov5640_write( 0x8c8a, 0x00);
ov5640_write( 0x8c8b, 0x00);
ov5640_write( 0x8c8c, 0x00);
ov5640_write( 0x8c8d, 0x00);
ov5640_write( 0x8c8e, 0x00);
ov5640_write( 0x8c8f, 0x00);
ov5640_write( 0x8c90, 0x00);
ov5640_write( 0x8c91, 0x00);
ov5640_write( 0x8c92, 0x00);
ov5640_write( 0x8c93, 0x00);
ov5640_write( 0x8c94, 0x00);
ov5640_write( 0x8c95, 0x00);
ov5640_write( 0x8c96, 0x00);
ov5640_write( 0x8c97, 0x00);
ov5640_write( 0x8c98, 0x00);
ov5640_write( 0x8c99, 0x00);
ov5640_write( 0x8c9a, 0x00);
ov5640_write( 0x8c9b, 0x00);
ov5640_write( 0x8c9c, 0x00);
ov5640_write( 0x8c9d, 0x00);
ov5640_write( 0x8c9e, 0x00);
ov5640_write( 0x8c9f, 0x00);
ov5640_write( 0x8ca0, 0x00);
ov5640_write( 0x8ca1, 0x00);
ov5640_write( 0x8ca2, 0x00);
ov5640_write( 0x8ca3, 0x00);
ov5640_write( 0x8ca4, 0x00);
ov5640_write( 0x8ca5, 0x00);
ov5640_write( 0x8ca6, 0x00);
ov5640_write( 0x8ca7, 0x00);
ov5640_write( 0x8ca8, 0x00);
ov5640_write( 0x8ca9, 0x00);
ov5640_write( 0x8caa, 0x00);
ov5640_write( 0x8cab, 0x00);
ov5640_write( 0x8cac, 0x00);
ov5640_write( 0x8cad, 0x00);
ov5640_write( 0x8cae, 0x00);
ov5640_write( 0x8caf, 0x00);
ov5640_write( 0x8cb0, 0x00);
ov5640_write( 0x8cb1, 0x00);
ov5640_write( 0x8cb2, 0x00);
ov5640_write( 0x8cb3, 0x00);
ov5640_write( 0x8cb4, 0x00);
ov5640_write( 0x8cb5, 0x00);
ov5640_write( 0x8cb6, 0x00);
ov5640_write( 0x8cb7, 0x00);
ov5640_write( 0x8cb8, 0x00);
ov5640_write( 0x8cb9, 0x00);
ov5640_write( 0x8cba, 0x00);
ov5640_write( 0x8cbb, 0x00);
ov5640_write( 0x8cbc, 0x00);
ov5640_write( 0x8cbd, 0x00);
ov5640_write( 0x8cbe, 0x00);
ov5640_write( 0x8cbf, 0x00);
ov5640_write( 0x8cc0, 0x00);
ov5640_write( 0x8cc1, 0x00);
ov5640_write( 0x8cc2, 0x00);
ov5640_write( 0x8cc3, 0x00);
ov5640_write( 0x8cc4, 0x00);
ov5640_write( 0x8cc5, 0x00);
ov5640_write( 0x8cc6, 0x00);
ov5640_write( 0x8cc7, 0x00);
ov5640_write( 0x8cc8, 0x00);
ov5640_write( 0x8cc9, 0x00);
ov5640_write( 0x8cca, 0x00);
ov5640_write( 0x8ccb, 0x00);
ov5640_write( 0x8ccc, 0x00);
ov5640_write( 0x8ccd, 0x00);
ov5640_write( 0x8cce, 0x00);
ov5640_write( 0x8ccf, 0x00);
ov5640_write( 0x8cd0, 0x00);
ov5640_write( 0x8cd1, 0x00);
ov5640_write( 0x8cd2, 0x00);
ov5640_write( 0x8cd3, 0x00);
ov5640_write( 0x8cd4, 0x00);
ov5640_write( 0x8cd5, 0x00);
ov5640_write( 0x8cd6, 0x00);
ov5640_write( 0x8cd7, 0x00);
ov5640_write( 0x8cd8, 0x00);
ov5640_write( 0x8cd9, 0x00);
ov5640_write( 0x8cda, 0x00);
ov5640_write( 0x8cdb, 0x00);
ov5640_write( 0x8cdc, 0x00);
ov5640_write( 0x8cdd, 0x00);
ov5640_write( 0x8cde, 0x00);
ov5640_write( 0x8cdf, 0x00);
ov5640_write( 0x8ce0, 0x00);
ov5640_write( 0x8ce1, 0x00);
ov5640_write( 0x8ce2, 0x00);
ov5640_write( 0x8ce3, 0x00);
ov5640_write( 0x8ce4, 0x00);
ov5640_write( 0x8ce5, 0x00);
ov5640_write( 0x8ce6, 0x00);
ov5640_write( 0x8ce7, 0x00);
ov5640_write( 0x8ce8, 0x00);
ov5640_write( 0x8ce9, 0x00);
ov5640_write( 0x8cea, 0x00);
ov5640_write( 0x8ceb, 0x00);
ov5640_write( 0x8cec, 0x00);
ov5640_write( 0x8ced, 0x00);
ov5640_write( 0x8cee, 0x00);
ov5640_write( 0x8cef, 0x00);
ov5640_write( 0x8cf0, 0x00);
ov5640_write( 0x8cf1, 0x00);
ov5640_write( 0x8cf2, 0x00);
ov5640_write( 0x8cf3, 0x00);
ov5640_write( 0x8cf4, 0x00);
ov5640_write( 0x8cf5, 0x00);
ov5640_write( 0x8cf6, 0x00);
ov5640_write( 0x8cf7, 0x00);
ov5640_write( 0x8cf8, 0x00);
ov5640_write( 0x8cf9, 0x00);
ov5640_write( 0x8cfa, 0x00);
ov5640_write( 0x8cfb, 0x00);
ov5640_write( 0x8cfc, 0x00);
ov5640_write( 0x8cfd, 0x00);
ov5640_write( 0x8cfe, 0x00);
ov5640_write( 0x8cff, 0x00);
ov5640_write( 0x8d00, 0x00);
ov5640_write( 0x8d01, 0x00);
ov5640_write( 0x8d02, 0x00);
ov5640_write( 0x8d03, 0x00);
ov5640_write( 0x8d04, 0x00);
ov5640_write( 0x8d05, 0x00);
ov5640_write( 0x8d06, 0x00);
ov5640_write( 0x8d07, 0x00);
ov5640_write( 0x8d08, 0x00);
ov5640_write( 0x8d09, 0x00);
ov5640_write( 0x8d0a, 0x00);
ov5640_write( 0x8d0b, 0x00);
ov5640_write( 0x8d0c, 0x00);
ov5640_write( 0x8d0d, 0x00);
ov5640_write( 0x8d0e, 0x00);
ov5640_write( 0x8d0f, 0x00);
ov5640_write( 0x8d10, 0x00);
ov5640_write( 0x8d11, 0x00);
ov5640_write( 0x8d12, 0x00);
ov5640_write( 0x8d13, 0x00);
ov5640_write( 0x8d14, 0x00);
ov5640_write( 0x8d15, 0x00);
ov5640_write( 0x8d16, 0x00);
ov5640_write( 0x8d17, 0x00);
ov5640_write( 0x8d18, 0x00);
ov5640_write( 0x8d19, 0x00);
ov5640_write( 0x8d1a, 0x00);
ov5640_write( 0x8d1b, 0x00);
ov5640_write( 0x8d1c, 0x00);
ov5640_write( 0x8d1d, 0x00);
ov5640_write( 0x8d1e, 0x00);
ov5640_write( 0x8d1f, 0x00);
ov5640_write( 0x8d20, 0x00);
ov5640_write( 0x8d21, 0x00);
ov5640_write( 0x8d22, 0x00);
ov5640_write( 0x8d23, 0x00);
ov5640_write( 0x8d24, 0x00);
ov5640_write( 0x8d25, 0x00);
ov5640_write( 0x8d26, 0x00);
ov5640_write( 0x8d27, 0x00);
ov5640_write( 0x8d28, 0x00);
ov5640_write( 0x8d29, 0x00);
ov5640_write( 0x8d2a, 0x00);
ov5640_write( 0x8d2b, 0x00);
ov5640_write( 0x8d2c, 0x00);
ov5640_write( 0x8d2d, 0x00);
ov5640_write( 0x8d2e, 0x00);
ov5640_write( 0x8d2f, 0x00);
ov5640_write( 0x8d30, 0x00);
ov5640_write( 0x8d31, 0x00);
ov5640_write( 0x8d32, 0x00);
ov5640_write( 0x8d33, 0x00);
ov5640_write( 0x8d34, 0x00);
ov5640_write( 0x8d35, 0x00);
ov5640_write( 0x8d36, 0x00);
ov5640_write( 0x8d37, 0x00);
ov5640_write( 0x8d38, 0x00);
ov5640_write( 0x8d39, 0x00);
ov5640_write( 0x8d3a, 0x00);
ov5640_write( 0x8d3b, 0x00);
ov5640_write( 0x8d3c, 0x00);
ov5640_write( 0x8d3d, 0x00);
ov5640_write( 0x8d3e, 0x00);
ov5640_write( 0x8d3f, 0x00);
ov5640_write( 0x8d40, 0x00);
ov5640_write( 0x8d41, 0x00);
ov5640_write( 0x8d42, 0x00);
ov5640_write( 0x8d43, 0x00);
ov5640_write( 0x8d44, 0x00);
ov5640_write( 0x8d45, 0x00);
ov5640_write( 0x8d46, 0x00);
ov5640_write( 0x8d47, 0x00);
ov5640_write( 0x8d48, 0x00);
ov5640_write( 0x8d49, 0x00);
ov5640_write( 0x8d4a, 0x00);
ov5640_write( 0x8d4b, 0x00);
ov5640_write( 0x8d4c, 0x00);
ov5640_write( 0x8d4d, 0x00);
ov5640_write( 0x8d4e, 0x00);
ov5640_write( 0x8d4f, 0x00);
ov5640_write( 0x8d50, 0x00);
ov5640_write( 0x8d51, 0x00);
ov5640_write( 0x8d52, 0x00);
ov5640_write( 0x8d53, 0x00);
ov5640_write( 0x8d54, 0x00);
ov5640_write( 0x8d55, 0x00);
ov5640_write( 0x8d56, 0x00);
ov5640_write( 0x8d57, 0x00);
ov5640_write( 0x8d58, 0x00);
ov5640_write( 0x8d59, 0x00);
ov5640_write( 0x8d5a, 0x00);
ov5640_write( 0x8d5b, 0x00);
ov5640_write( 0x8d5c, 0x00);
ov5640_write( 0x8d5d, 0x00);
ov5640_write( 0x8d5e, 0x00);
ov5640_write( 0x8d5f, 0x00);
ov5640_write( 0x8d60, 0x00);
ov5640_write( 0x8d61, 0x00);
ov5640_write( 0x8d62, 0x00);
ov5640_write( 0x8d63, 0x00);
ov5640_write( 0x8d64, 0x00);
ov5640_write( 0x8d65, 0x00);
ov5640_write( 0x8d66, 0x00);
ov5640_write( 0x8d67, 0x00);
ov5640_write( 0x8d68, 0x00);
ov5640_write( 0x8d69, 0x00);
ov5640_write( 0x8d6a, 0x00);
ov5640_write( 0x8d6b, 0x00);
ov5640_write( 0x8d6c, 0x00);
ov5640_write( 0x8d6d, 0x00);
ov5640_write( 0x8d6e, 0x00);
ov5640_write( 0x8d6f, 0x00);
ov5640_write( 0x8d70, 0x00);
ov5640_write( 0x8d71, 0x00);
ov5640_write( 0x8d72, 0x00);
ov5640_write( 0x8d73, 0x00);
ov5640_write( 0x8d74, 0x00);
ov5640_write( 0x8d75, 0x00);
ov5640_write( 0x8d76, 0x00);
ov5640_write( 0x8d77, 0x00);
ov5640_write( 0x8d78, 0x00);
ov5640_write( 0x8d79, 0x00);
ov5640_write( 0x8d7a, 0x00);
ov5640_write( 0x8d7b, 0x00);
ov5640_write( 0x8d7c, 0x00);
ov5640_write( 0x8d7d, 0x00);
ov5640_write( 0x8d7e, 0x00);
ov5640_write( 0x8d7f, 0x00);
ov5640_write( 0x8d80, 0x00);
ov5640_write( 0x8d81, 0x00);
ov5640_write( 0x8d82, 0x00);
ov5640_write( 0x8d83, 0x00);
ov5640_write( 0x8d84, 0x00);
ov5640_write( 0x8d85, 0x00);
ov5640_write( 0x8d86, 0x00);
ov5640_write( 0x8d87, 0x00);
ov5640_write( 0x8d88, 0x00);
ov5640_write( 0x8d89, 0x00);
ov5640_write( 0x8d8a, 0x00);
ov5640_write( 0x8d8b, 0x00);
ov5640_write( 0x8d8c, 0x00);
ov5640_write( 0x8d8d, 0x00);
ov5640_write( 0x8d8e, 0x00);
ov5640_write( 0x8d8f, 0x00);
ov5640_write( 0x8d90, 0x00);
ov5640_write( 0x8d91, 0x00);
ov5640_write( 0x8d92, 0x00);
ov5640_write( 0x8d93, 0x00);
ov5640_write( 0x8d94, 0x00);
ov5640_write( 0x8d95, 0x00);
ov5640_write( 0x8d96, 0x00);
ov5640_write( 0x8d97, 0x00);
ov5640_write( 0x8d98, 0x00);
ov5640_write( 0x8d99, 0x00);
ov5640_write( 0x8d9a, 0x00);
ov5640_write( 0x8d9b, 0x00);
ov5640_write( 0x8d9c, 0x00);
ov5640_write( 0x8d9d, 0x00);
ov5640_write( 0x8d9e, 0x00);
ov5640_write( 0x8d9f, 0x00);
ov5640_write( 0x8da0, 0x00);
ov5640_write( 0x8da1, 0x00);
ov5640_write( 0x8da2, 0x00);
ov5640_write( 0x8da3, 0x00);
ov5640_write( 0x8da4, 0x00);
ov5640_write( 0x8da5, 0x00);
ov5640_write( 0x8da6, 0x00);
ov5640_write( 0x8da7, 0x00);
ov5640_write( 0x8da8, 0x00);
ov5640_write( 0x8da9, 0x00);
ov5640_write( 0x8daa, 0x00);
ov5640_write( 0x8dab, 0x00);
ov5640_write( 0x8dac, 0x00);
ov5640_write( 0x8dad, 0x00);
ov5640_write( 0x8dae, 0x00);
ov5640_write( 0x8daf, 0x00);
ov5640_write( 0x8db0, 0x00);
ov5640_write( 0x8db1, 0x00);
ov5640_write( 0x8db2, 0x00);
ov5640_write( 0x8db3, 0x00);
ov5640_write( 0x8db4, 0x00);
ov5640_write( 0x8db5, 0x00);
ov5640_write( 0x8db6, 0x00);
ov5640_write( 0x8db7, 0x00);
ov5640_write( 0x8db8, 0x00);
ov5640_write( 0x8db9, 0x00);
ov5640_write( 0x8dba, 0x00);
ov5640_write( 0x8dbb, 0x00);
ov5640_write( 0x8dbc, 0x00);
ov5640_write( 0x8dbd, 0x00);
ov5640_write( 0x8dbe, 0x00);
ov5640_write( 0x8dbf, 0x00);
ov5640_write( 0x8dc0, 0x00);
ov5640_write( 0x8dc1, 0x00);
ov5640_write( 0x8dc2, 0x00);
ov5640_write( 0x8dc3, 0x00);
ov5640_write( 0x8dc4, 0x00);
ov5640_write( 0x8dc5, 0x00);
ov5640_write( 0x8dc6, 0x00);
ov5640_write( 0x8dc7, 0x00);
ov5640_write( 0x8dc8, 0x00);
ov5640_write( 0x8dc9, 0x00);
ov5640_write( 0x8dca, 0x00);
ov5640_write( 0x8dcb, 0x00);
ov5640_write( 0x8dcc, 0x00);
ov5640_write( 0x8dcd, 0x00);
ov5640_write( 0x8dce, 0x00);
ov5640_write( 0x8dcf, 0x00);
ov5640_write( 0x8dd0, 0x00);
ov5640_write( 0x8dd1, 0x00);
ov5640_write( 0x8dd2, 0x00);
ov5640_write( 0x8dd3, 0x00);
ov5640_write( 0x8dd4, 0x00);
ov5640_write( 0x8dd5, 0x00);
ov5640_write( 0x8dd6, 0x00);
ov5640_write( 0x8dd7, 0x00);
ov5640_write( 0x8dd8, 0x00);
ov5640_write( 0x8dd9, 0x00);
ov5640_write( 0x8dda, 0x00);
ov5640_write( 0x8ddb, 0x00);
ov5640_write( 0x8ddc, 0x00);
ov5640_write( 0x8ddd, 0x00);
ov5640_write( 0x8dde, 0x00);
ov5640_write( 0x8ddf, 0x00);
ov5640_write( 0x8de0, 0x00);
ov5640_write( 0x8de1, 0x00);
ov5640_write( 0x8de2, 0x00);
ov5640_write( 0x8de3, 0x00);
ov5640_write( 0x8de4, 0x00);
ov5640_write( 0x8de5, 0x00);
ov5640_write( 0x8de6, 0x00);
ov5640_write( 0x8de7, 0x00);
ov5640_write( 0x8de8, 0x00);
ov5640_write( 0x8de9, 0x00);
ov5640_write( 0x8dea, 0x00);
ov5640_write( 0x8deb, 0x00);
ov5640_write( 0x8dec, 0x00);
ov5640_write( 0x8ded, 0x00);
ov5640_write( 0x8dee, 0x00);
ov5640_write( 0x8def, 0x00);
ov5640_write( 0x8df0, 0x00);
ov5640_write( 0x8df1, 0x00);
ov5640_write( 0x8df2, 0x00);
ov5640_write( 0x8df3, 0x00);
ov5640_write( 0x8df4, 0x00);
ov5640_write( 0x8df5, 0x00);
ov5640_write( 0x8df6, 0x00);
ov5640_write( 0x8df7, 0x00);
ov5640_write( 0x8df8, 0x00);
ov5640_write( 0x8df9, 0x00);
ov5640_write( 0x8dfa, 0x00);
ov5640_write( 0x8dfb, 0x00);
ov5640_write( 0x8dfc, 0x00);
ov5640_write( 0x8dfd, 0x00);
ov5640_write( 0x8dfe, 0x00);
ov5640_write( 0x8dff, 0x00);
ov5640_write( 0x8e00, 0x11);
ov5640_write( 0x8e01, 0x09);
ov5640_write( 0x8e02, 0x02);
ov5640_write( 0x8e03, 0x15);
ov5640_write( 0x8e04, 0x42);
ov5640_write( 0x8e05, 0x44);
ov5640_write( 0x8e06, 0x5a);
ov5640_write( 0x8e07, 0x54);
ov5640_write( 0x8e08, 0x45);
ov5640_write( 0x8e09, 0x20);
ov5640_write( 0x8e0a, 0x20);
ov5640_write( 0x8e0b, 0x20);
ov5640_write( 0x8e0c, 0x20);
ov5640_write( 0x8e0d, 0x20);
ov5640_write( 0x8e0e, 0x90);
ov5640_write( 0x8e0f, 0x01);
ov5640_write( 0x8e10, 0x10);
ov5640_write( 0x8e11, 0x00);
ov5640_write( 0x8e12, 0x56);
ov5640_write( 0x8e13, 0x40);
ov5640_write( 0x8e14, 0x1a);
ov5640_write( 0x8e15, 0x30);
ov5640_write( 0x8e16, 0x29);
ov5640_write( 0x8e17, 0x7e);
ov5640_write( 0x8e18, 0x00);
ov5640_write( 0x8e19, 0x30);
ov5640_write( 0x8e1a, 0x04);
ov5640_write( 0x8e1b, 0x20);
ov5640_write( 0x8e1c, 0xdf);
ov5640_write( 0x8e1d, 0x30);
ov5640_write( 0x8e1e, 0x05);
ov5640_write( 0x8e1f, 0x40);
ov5640_write( 0x8e20, 0xbf);
ov5640_write( 0x8e21, 0x50);
ov5640_write( 0x8e22, 0x03);
ov5640_write( 0x8e23, 0x00);
ov5640_write( 0x8e24, 0xfd);
ov5640_write( 0x8e25, 0x50);
ov5640_write( 0x8e26, 0x27);
ov5640_write( 0x8e27, 0x01);
ov5640_write( 0x8e28, 0xfe);
ov5640_write( 0x8e29, 0x60);
ov5640_write( 0x8e2a, 0x00);
ov5640_write( 0x8e2b, 0x11);
ov5640_write( 0x8e2c, 0x00);
ov5640_write( 0x8e2d, 0x3f);
ov5640_write( 0x8e2e, 0x05);
ov5640_write( 0x8e2f, 0x30);
ov5640_write( 0x8e30, 0x00);
ov5640_write( 0x8e31, 0x3f);
ov5640_write( 0x8e32, 0x06);
ov5640_write( 0x8e33, 0x22);
ov5640_write( 0x8e34, 0x00);
ov5640_write( 0x8e35, 0x3f);
ov5640_write( 0x8e36, 0x01);
ov5640_write( 0x8e37, 0x2a);
ov5640_write( 0x8e38, 0x00);
ov5640_write( 0x8e39, 0x3f);
ov5640_write( 0x8e3a, 0x02);
ov5640_write( 0x8e3b, 0x00);
ov5640_write( 0x8e3c, 0x00);
ov5640_write( 0x8e3d, 0x36);
ov5640_write( 0x8e3e, 0x06);
ov5640_write( 0x8e3f, 0x07);
ov5640_write( 0x8e40, 0x00);
ov5640_write( 0x8e41, 0x3f);
ov5640_write( 0x8e42, 0x0b);
ov5640_write( 0x8e43, 0x0f);
ov5640_write( 0x8e44, 0xf0);
ov5640_write( 0x8e45, 0x00);
ov5640_write( 0x8e46, 0x00);
ov5640_write( 0x8e47, 0x00);
ov5640_write( 0x8e48, 0x00);
ov5640_write( 0x8e49, 0x30);
ov5640_write( 0x8e4a, 0x01);
ov5640_write( 0x8e4b, 0x40);
ov5640_write( 0x8e4c, 0xbf);
ov5640_write( 0x8e4d, 0x30);
ov5640_write( 0x8e4e, 0x01);
ov5640_write( 0x8e4f, 0x00);
ov5640_write( 0x8e50, 0xbf);
ov5640_write( 0x8e51, 0x30);
ov5640_write( 0x8e52, 0x29);
ov5640_write( 0x8e53, 0x70);
ov5640_write( 0x8e54, 0x00);
ov5640_write( 0x8e55, 0x3a);
ov5640_write( 0x8e56, 0x00);
ov5640_write( 0x8e57, 0x00);
ov5640_write( 0x8e58, 0xff);
ov5640_write( 0x8e59, 0x3a);
ov5640_write( 0x8e5a, 0x00);
ov5640_write( 0x8e5b, 0x00);
ov5640_write( 0x8e5c, 0xff);
ov5640_write( 0x8e5d, 0x36);
ov5640_write( 0x8e5e, 0x03);
ov5640_write( 0x8e5f, 0x36);
ov5640_write( 0x8e60, 0x02);
ov5640_write( 0x8e61, 0x41);
ov5640_write( 0x8e62, 0x44);
ov5640_write( 0x8e63, 0x58);
ov5640_write( 0x8e64, 0x20);
ov5640_write( 0x8e65, 0x18);
ov5640_write( 0x8e66, 0x10);
ov5640_write( 0x8e67, 0x0a);
ov5640_write( 0x8e68, 0x04);
ov5640_write( 0x8e69, 0x04);
ov5640_write( 0x8e6a, 0x00);
ov5640_write( 0x8e6b, 0x03);
ov5640_write( 0x8e6c, 0xff);
ov5640_write( 0x8e6d, 0x64);
ov5640_write( 0x8e6e, 0x00);
ov5640_write( 0x8e6f, 0x00);
ov5640_write( 0x8e70, 0x80);
ov5640_write( 0x8e71, 0x00);
ov5640_write( 0x8e72, 0x00);
ov5640_write( 0x8e73, 0x00);
ov5640_write( 0x8e74, 0x00);
ov5640_write( 0x8e75, 0x00);
ov5640_write( 0x8e76, 0x00);
ov5640_write( 0x8e77, 0x02);
ov5640_write( 0x8e78, 0x04);
ov5640_write( 0x8e79, 0x06);
ov5640_write( 0x8e7a, 0x00);
ov5640_write( 0x8e7b, 0x03);
ov5640_write( 0x8e7c, 0x98);
ov5640_write( 0x8e7d, 0x00);
ov5640_write( 0x8e7e, 0xcc);
ov5640_write( 0x8e7f, 0x50);
ov5640_write( 0x8e80, 0x3c);
ov5640_write( 0x8e81, 0x28);
ov5640_write( 0x8e82, 0x1e);
ov5640_write( 0x8e83, 0x10);
ov5640_write( 0x8e84, 0x10);
ov5640_write( 0x8e85, 0x02);
ov5640_write( 0x8e86, 0x00);
ov5640_write( 0x8e87, 0x00);
ov5640_write( 0x8e88, 0x6e);
ov5640_write( 0x8e89, 0x06);
ov5640_write( 0x8e8a, 0x05);
ov5640_write( 0x8e8b, 0x00);
ov5640_write( 0x8e8c, 0xa5);
ov5640_write( 0x8e8d, 0x5a);
ov5640_write( 0x8e8e, 0x00);
ov5640_write( 0x3022, 0x00);
ov5640_write( 0x3023, 0x00);
ov5640_write( 0x3024, 0x00);
ov5640_write( 0x3025, 0x00);
ov5640_write( 0x3026, 0x00);
ov5640_write( 0x3027, 0x00);
ov5640_write( 0x3028, 0x00);
ov5640_write( 0x3029, 0x7F);
ov5640_write( 0x3000, 0x00);
	
		if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetFirm_Pri(): Error[%d] \r\n",
				sCamI2cStatus);
		result = sCamI2cStatus;
	}
		
        return result;
}


struct sens_methods sens_meth_pri = {
DRV_GetIntfConfig: CAMDRV_GetIntfConfig_Pri,
		   DRV_GetIntfSeqSel : CAMDRV_GetIntfSeqSel_Pri,
		   DRV_Wakeup : CAMDRV_Wakeup_Pri,
		   DRV_SetVideoCaptureMode : CAMDRV_SetVideoCaptureMode_Pri,
		   DRV_SetFrameRate : CAMDRV_SetFrameRate_Pri,
		   DRV_EnableVideoCapture : CAMDRV_EnableVideoCapture_Pri,
		   DRV_SetCamSleep : CAMDRV_SetCamSleep_Pri,
		   DRV_GetJpegSize : CAMDRV_GetJpegSize,
		   DRV_GetJpeg:CAMDRV_GetJpeg,
		   DRV_GetThumbnail:CAMDRV_GetThumbnail,		
		   DRV_DisableCapture : CAMDRV_DisableCapture_Pri,
		   DRV_DisablePreview : CAMDRV_DisablePreview_Pri,
		   DRV_CfgStillnThumbCapture : CAMDRV_CfgStillnThumbCapture_Pri,
		   DRV_StoreBaseAddress:CAMDRV_StoreBaseAddress,
		   DRV_SetSceneMode : CAMDRV_SetSceneMode_Pri,
		   DRV_SetWBMode : CAMDRV_SetWBMode_Pri,
		   DRV_SetAntiBanding : CAMDRV_SetAntiBanding_Pri,
		   DRV_SetFocusMode : CAMDRV_SetFocusMode_Pri,
		   DRV_SetDigitalEffect : CAMDRV_SetDigitalEffect_Pri,
		   DRV_SetFlashMode : CAMDRV_SetFlashMode_Pri,
		   DRV_SetJpegQuality : CAMDRV_SetJpegQuality_Pri,
		   DRV_TurnOnAF : CAMDRV_TurnOnAF_Pri,
		   DRV_TurnOffAF : CAMDRV_TurnOffAF_Pri,
		   DRV_SetExposure : CAMDRV_SetExposure_Pri,
		   DRV_StoreBaseAddress:CAMDRV_StoreBaseAddress,  	
		   DRV_SetFirmware:CAMDRV_SetFirm_Pri,
};

struct sens_methods *CAMDRV_primary_get(void)
{
	return &sens_meth_pri;
}
