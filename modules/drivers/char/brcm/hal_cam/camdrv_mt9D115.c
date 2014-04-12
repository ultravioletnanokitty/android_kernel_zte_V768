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
*   @file   camdrv_mt9d111.c
*
*   @brief  This file is the lower level driver API of MT9D115(2M 
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
//#include "hal_cam_drv.h"
//#include "cam_cntrl_2153.h"
#include "camdrv_dev.h"

#include <plat/csl/csl_cam.h>

//#if defined (CONFIG_CAM_CSI2)
#define CAMERA_IMAGE_INTERFACE  CSL_CAM_INTF_CSI
//#endif

//#if defined (CONFIG_CAM_CPI)
//#define CAMERA_IMAGE_INTERFACE  CSL_CAM_INTF_CPI
//#endif

#define CAMERA_PHYS_INTF_PORT   CSL_CAM_PORT_AFE_1

#define MT9D115_ID 0x2580

#define HAL_CAM_RESET 56   /*gpio 56*/
#define HAL_CAM_PWDN 13   /*gpio 13*/

#define FLASH_TRIGGER  30   /*gpio 30 for flash */


#define HAL_CAM_VGA_STANDBY  9

#define HAL_CAM_VGA_RESET 10

int real_write=0;
CamFocusControlMode_t FOCUS_MODE;


/*****************************************************************************/
/* start of CAM configuration */
/*****************************************************************************/

/*****************************************************************************/
/*  Sensor Resolution Tables:												 */
/*****************************************************************************/
/* Resolutions & Sizes available for MT9D115 ISP/Sensor (QXGA max) */
static CamResolutionConfigure_t sSensorResInfo_MT9D115_st[] = {
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

/*****************************************************************************/
/*  Power On/Off Tables for Main Sensor */
/*****************************************************************************/

/*---------Sensor Power On */
static CamSensorIntfCntrl_st_t CamPowerOnSeq[] = {

    {PAUSE, 1, Nop_Cmd},
    {GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
    {PAUSE, 10, Nop_Cmd},
    
// -------Turn everything OFF   
    //{GPIO_CNTRL, HAL_CAM_RESET,    GPIO_SetLow},
    {MCLK_CNTRL, CamDrv_NO_CLK,    CLK_TurnOff},
    {GPIO_CNTRL, HAL_CAM_PWDN,       GPIO_SetLow},
    {PAUSE, 10, Nop_Cmd},
    //{PAUSE,      50,               Nop_Cmd},
// -------Disable Reset 
    //{GPIO_CNTRL, HAL_CAM_RESET,    GPIO_SetHigh},
// -------Enable Clock to Cameras @ Main clock speed
    {MCLK_CNTRL, CamDrv_24MHz,     CLK_TurnOn},  //Mclock for zte objective test recording slow 20120628
    {PAUSE,      5,                Nop_Cmd},
    {GPIO_CNTRL, HAL_CAM_RESET,    GPIO_SetLow},
    {PAUSE,      1,                Nop_Cmd},
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	{PAUSE,      5,                Nop_Cmd},
};

/*---------Sensor Power Off*/
static CamSensorIntfCntrl_st_t CamPowerOffSeq[] = {
	{PAUSE,      50,               Nop_Cmd},
// -------Disable Clock to Cameras 
	{MCLK_CNTRL, CamDrv_NO_CLK,    CLK_TurnOff},
// -------Lower Reset to ISP,Enable Reset    
    {GPIO_CNTRL, HAL_CAM_RESET,    GPIO_SetLow},
    {PAUSE,      5,                Nop_Cmd},
// -------Turn Power OFF    
    {GPIO_CNTRL, HAL_CAM_PWDN,       GPIO_SetLow},
    
};

/** Primary Sensor Configuration and Capabilities  */
static HAL_CAM_ConfigCaps_st_t CamPrimaryCfgCap_st = {
	// CAMDRV_DATA_MODE_S *video_mode
	{
        800,                           // unsigned short        max_width;                //Maximum width resolution
        600,                           // unsigned short        max_height;                //Maximum height resolution
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
        1                              // UInt32                pre_frame;            //Frames to throw out for ViewFinder/Video capture
    },

	// CAMDRV_DATA_MODE_S *stills_mode
	{

        2048,                           // unsigned short max_width;   Maximum width resolution
        1536,                           // unsigned short max_height;  Maximum height resolution
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
        4                               // UInt32                pre_frame;            //Frames to throw out for Stills capture
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
	"MT9D115"
};

/*---------Sensor Primary Configuration CCIR656*/
static CamIntfConfig_CCIR656_st_t CamPrimaryCfg_CCIR656_st = {
	// Vsync, Hsync, Clock
	CSL_CAM_SYNC_EXTERNAL,				///< UInt32 sync_mode;				(default)CAM_SYNC_EXTERNAL:  Sync External or Embedded
	CSL_CAM_SYNC_DEFINES_ACTIVE,		///< UInt32 vsync_control;			(default)CAM_SYNC_DEFINES_ACTIVE:		VSYNCS determines active data
	CSL_CAM_SYNC_ACTIVE_HIGH,			///< UInt32 vsync_polarity; 		   default)ACTIVE_LOW/ACTIVE_HIGH:		  Vsync active
	CSL_CAM_SYNC_DEFINES_ACTIVE,		///< UInt32 hsync_control;			(default)FALSE/TRUE:					HSYNCS determines active data
	CSL_CAM_SYNC_ACTIVE_HIGH,			///< UInt32 hsync_polarity; 		(default)ACTIVE_HIGH/ACTIVE_LOW:		Hsync active
	CSL_CAM_CLK_EDGE_POS,				///< UInt32 data_clock_sample;		(default)RISING_EDGE/FALLING_EDGE:		Pixel Clock Sample edge
	CSL_CAM_PIXEL_8BIT, 				///< UInt32 bus_width;				(default)CAM_BITWIDTH_8:				Camera bus width
	0,							///< UInt32 data_shift; 				   (default)0:							   data shift (+) left shift  (-) right shift
	CSL_CAM_FIELD_H_V,					///< UInt32 field_mode; 			(default)CAM_FIELD_H_V: 				field calculated
	CSL_CAM_INT_FRAME_END,				///< UInt32 data_intr_enable;		CAM_INTERRUPT_t:
	CSL_CAM_INT_FRAME_END,				///< UInt32 pkt_intr_enable;		CAM_INTERRUPT_t:

};

// ************************* REMEMBER TO CHANGE IT WHEN YOU CHANGE TO CCP ***************************
//CSI host connection
//---------Sensor Primary Configuration CCP CSI (sensor_config_ccp_csi)
CamIntfConfig_CCP_CSI_st_t  CamPrimaryCfg_CCP_CSI_st =
{
    CSL_CAM_INPUT_SINGLE_LANE, //CSL_CAM_INPUT_SINGLE_LANE ,  // ,                CSL_CAM_INPUT_DUAL_LANE    ///< UInt32 input_mode;     CSL_CAM_INPUT_MODE_T:
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
	1200,			/*< UInt32 jpeg_packet_size_bytes;     Bytes/Hsync */
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

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution,
		     CamDataFmt_t image_format);
static HAL_CAM_Result_en_t Init_MT9D115(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt16 mt9d115_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t mt9d115_write(unsigned int sub_addr, UInt16 data);
static HAL_CAM_Result_en_t CAMDRV_SetFrameRate_Pri(CamRates_t fps,
					CamSensorSelect_t sensor);
static HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode_Pri(CamImageSize_t image_resolution,
					       CamDataFmt_t image_format,
					       CamSensorSelect_t sensor);
static HAL_CAM_Result_en_t CAMDRV_SetZoom_Pri(CamZoom_t step,CamSensorSelect_t sensor);
static void control_flash(int flash_on);

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
			    ("Error in allocating %d bytes for MT9D115 sensor\n",
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
	printk("\n------------CAMDRV_Wakeup_Pri  ----------\r\n");
	result = Init_MT9D115(sensor);
	
	return result;
}

static UInt16 CAMDRV_GetDeviceID_Pri(CamSensorSelect_t sensor)
{
	return mt9d115_read(0x0000);
}

static int checkCameraID(CamSensorSelect_t sensor)
{
	UInt16 devId = CAMDRV_GetDeviceID_Pri(sensor);

	if (devId == MT9D115_ID) {
		printk("-------Camera MT9D115 Device ID = 0x%x ------\r\n",devId);
		return 0;
	} else {
		printk("Camera Id wrong. Expected 0x%x but got 0x%x\r\n",
			 MT9D115_ID, devId);
		return -1;
	}
}

static HAL_CAM_Result_en_t mt9d115_write(unsigned int sub_addr, UInt16 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 bytes[2];
	bytes[0] = (data & 0xFF00) >> 8;
	bytes[1] = data & 0xFF;

	result |= CAM_WriteI2c(sub_addr, 2, bytes);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9d115_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
	return result;
}

static UInt16 mt9d115_read(unsigned int sub_addr)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt16 data;
	UInt16 temp;

	result |= CAM_ReadI2c(sub_addr, 2, (UInt8 *) &data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9d115_read(): ERROR: %d\r\n", result);
	}

	temp = data;
	data = ((temp & 0xFF) << 8) | ((temp & 0xFF00) >> 8);

	return data;
}

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format)
{
	UInt32 data = 0, i = 10;
	
	mt9d115_write( 0x098C, 0xA115);   //Output Width (A)
	mt9d115_write( 0x0990, 0x0000);   //      = 640
	mt9d115_write( 0x098C, 0xA103);   //Output Width (A)
	mt9d115_write( 0x0990, 0x0001);   //      = 640
//zhaoqiang@wind-mobi.com chg 2012.04.05 start
//add read reg 0xa104 for change to preview mode correctlly
//reviewed by yuanlan@wind-mobi.com
	msleep(200);
		
	while(i--)
	{
		mt9d115_write( 0x098C, 0xA104 );
		data = mt9d115_read(0x0990);
		msleep(50);
	
		//printk("%s:data=0x%x\n",__func__,data);
		if (data == 0x03)
			break;
	}
	printk("i=%d\n",i);
	if(i == 0)
	{
		mt9d115_write( 0x098C, 0xA115);
		mt9d115_write( 0x0990, 0x0000);
		mt9d115_write( 0x098C, 0xA103);
		mt9d115_write( 0x0990, 0x0001);
		msleep(200);
	}

//zhaoqiang@wind-mobi.com chg 2012.04.05 end	
	return HAL_CAM_SUCCESS;
	
	UInt32 x = 0, y = 0;
	UInt32 format = 0;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	printk(KERN_ERR"SensorSetPreviewMode image_resolution 0x%x image_format %d  ",image_resolution,image_format  );

#if 0
	mt9d115_write( 0x098E, 0xEC05 );	// MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN]
	mt9d115_write( 0x0990, 0x0005 	);// MCU_DATA_0
	mt9d115_write( 0x098E, 0x8400 	);// MCU_ADDRESS [SEQ_CMD]
	mt9d115_write( 0x0990, 0x0001 	);// MCU_DATA_0

	return HAL_CAM_SUCCESS;
#endif 
	switch (image_resolution) {
	
	case CamImageSize_R_QCIF:
		x = 144;
		y = 176;
		break;
	
	case CamImageSize_R_QVGA:
		x = 240;
		y = 320;
		break;
		
	case CamImageSize_SQCIF:
		x = 128;
		y = 96;
		break;

	case CamImageSize_QQVGA:
		x = 160;
		y = 120;
		break;

	case CamImageSize_QCIF:
		x = 176;
		y = 144;
		break;

	case CamImageSize_QVGA:
		x = 320;
		y = 240;
		break;

	case CamImageSize_CIF:
		x = 352;
		y = 288;
		break;

	case CamImageSize_VGA:
		x = 640;
		y = 480;
		break;

	case CamImageSize_SVGA:
		x = 800;
		y = 600;
		break;

	default:
		x = 320;
		y = 240;
		break;
	}

	/* Choose Format for Viewfinder mode  */
	/* Set the format for the Viewfinder mode */
	switch (image_format) {

	case CamDataFmtYCbCr:
		format = 1;
		break;

	case CamDataFmtRGB565:
		format = 4;
		break;

	default:
		format = 1;
		break;
	}

	mt9d115_write( 0x098E, 0x6800 );	// MCU_ADDRESS [PRI_A_IMAGE_WIDTH]
	mt9d115_write( 0x0990, x );	// MCU_DATA_0
	mt9d115_write( 0x098E, 0x6802 );	// MCU_ADDRESS [PRI_A_IMAGE_HEIGHT]
	mt9d115_write( 0x0990, y );	// MCU_DATA_0
	mt9d115_write( 0x098E, 0x8400 );	// MCU_ADDRESS [SEQ_CMD]
	mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0

	if (image_format == CamDataFmtYCbCr) { /*YUV order*/
		UInt32 output_order = 2;	/* Switch low, high bytes. Y and C. */

		/* [Set Output Format Order] */
		/*MCU_ADDRESS[PRI_A_CONFIG_JPEG_JP_MODE] */
		mt9d115_write(0x098E, 0x6809);
		mt9d115_write(0x0990, output_order);	/* MCU_DATA_0 */
		mt9d115_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
		mt9d115_write(0x0990, 0x0006);	/* MCU_DATA_0 */

		printk
		    ("SensorSetPreviewMode(): Output Format Order = 0x%x\r\n",
		     output_order);
	}

	printk("SensorSetPreviewMode(): Resolution:0x%x, Format:0x%x r\n",
		 image_resolution, image_format);

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk
		    ("SensorSetPreviewMode(): Error[%d] sending preview mode  r\n",
		     sCamI2cStatus);
		result = sCamI2cStatus;
	}
	/*[Enable stream] */
	//mt9d115_write(0x001A, 0x0218);
	
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
	//hanwei@wind-mobi.com add 2012.02.20 start 
	//fix bug#7911 add match exposure command
	//reviewed by liubing@wind-mobi.com
	printk("CAMDRV_SetExposure_Pri  exposure_value = %d        \r\n",value );

	if(value==0)
	{
		printk("CAMDRV_SetExposure_Pri      =   0     \r\n" );
		//[EV  +0]
		mt9d115_write(0x098C, 0xA24F);        	// MCU_ADDRESS [AE_BASETARGET]
		mt9d115_write(0x0990, 0x0038);         	// MCU_DATA_0 46 //modify by zhaoqiang
    // Saturation
		mt9d115_write( 0x098C, 0xAB20 ); // MCU_ADDRESS [HG_LL_SAT1]
		mt9d115_write( 0x0990, 0x007A ); // MCU_DATA_0

		mt9d115_write(0x098C, 0xA103 );         
		mt9d115_write(0x0990, 0x0005 );
	}
	else if(value>0)
	{
		printk("CAMDRV_SetExposure_Pri      > 0    \r\n" );
		//[EV  +2]
		mt9d115_write(0x098C, 0xA24F);        	// MCU_ADDRESS [AE_BASETARGET]
		mt9d115_write(0x0990, 0x0048);         	// MCU_DATA_0 //modify by zhaoqiang
    // Saturation
		mt9d115_write( 0x098C, 0xAB20 ); // MCU_ADDRESS [HG_LL_SAT1]
		mt9d115_write( 0x0990, 0x007A ); // MCU_DATA_0

		mt9d115_write(0x098C, 0xA103 );         
		mt9d115_write(0x0990, 0x0005 );
	}
	else
	{
		printk("CAMDRV_SetExposure_Pri      < 0    \r\n" );
		//[EV  -2]
		mt9d115_write(0x098C, 0xA24F);         	// MCU_ADDRESS [AE_BASETARGET]
		mt9d115_write(0x0990, 0x0028);//16);         	// MCU_DATA_0  fix bug#7911  //modify by zhaoqiang
    // Saturation
		mt9d115_write( 0x098C, 0xAB20 ); // MCU_ADDRESS [HG_LL_SAT1]
		mt9d115_write( 0x0990, 0x007A ); // MCU_DATA_0

		mt9d115_write(0x098C, 0xA103 );         
		mt9d115_write(0x0990, 0x0005 );
	}
	
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		 printk("CAMDRV_SetExposure_Pri(): Error[%d] \r\n",
			  sCamI2cStatus);
		 result = sCamI2cStatus;
	}
	//hanwei@wind-mobi.com add 2012.02.20 end
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
{ return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	printk("CAMDRV_SetFrameRate %d \r\n",fps );
	
					
	if (fps > CamRate_30)
	{
		result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
	}
	else
	{
		if (fps == CamRate_Auto)
		{

		}
		else
		{
			if(fps>=CamRate_15)
			{
				
				mt9d115_write(0x98E, 0x68AA);	//TX FIFO Watermark (A)
				mt9d115_write(0x990, 0x0218);	//		= 536
				mt9d115_write(0x98E, 0x6815);	//Max FD Zone 50 Hz
				mt9d115_write(0x990, 0x0007);	//		= 7
				mt9d115_write(0x98E, 0x6817 );//Max FD Zone 60 Hz
				mt9d115_write(0x990, 0x0008 );//	  = 8
				mt9d115_write(0x98E, 0x682D);	//AE Target FD Zone
				mt9d115_write(0x990, 0x0007);	//		= 7
				
				mt9d115_write( 0x098E, 0x8400); 	// MCU_ADDRESS [SEQ_CMD]
				mt9d115_write( 0x0990, 0x0006); 	// MCU_DATA_0
			}
			else if(fps>=CamRate_10)
			{
				mt9d115_write(0x98E, 0x68AA);	//TX FIFO Watermark (A)
				mt9d115_write(0x990, 0x026A);	//      = 618
				mt9d115_write(0x98E, 0x6815);	//Max FD Zone 50 Hz
				mt9d115_write(0x990, 0x000A);	//      = 10
				mt9d115_write(0x98E, 0x6817);	//Max FD Zone 60 Hz
				mt9d115_write(0x990, 0x000C);	//      = 12
				mt9d115_write(0x98E, 0x682D);	//AE Target FD Zone
				mt9d115_write(0x990, 0x000A);	//      = 10

				mt9d115_write( 0x098E, 0x8400); 	// MCU_ADDRESS [SEQ_CMD]
				mt9d115_write( 0x0990, 0x0006); 	// MCU_DATA_0
			}
			else if(fps>=CamRate_5)
			{
				mt9d115_write(0x98E, 0x68AA);	//TX FIFO Watermark (A)
				mt9d115_write(0x990, 0x0218);	//       = 536
				mt9d115_write(0x98E, 0x6815);	//Max FD Zone 50 Hz
				mt9d115_write(0x990, 0x0014);	//       = 20
				mt9d115_write(0x98E, 0x6817);	//Max FD Zone 60 Hz
				mt9d115_write(0x990, 0x0018);	//       = 24
				mt9d115_write(0x98E, 0x682D);	//AE Target FD Zone
				mt9d115_write(0x990, 0x0014);	//      = 20

				mt9d115_write( 0x098E, 0x8400); 	// MCU_ADDRESS [SEQ_CMD]
				mt9d115_write( 0x0990, 0x0006); 	// MCU_DATA_0
			}
			else
			{
				printk("CAMDRV_SetFrameRate(): Error HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
			}	
		}       // else (if (ImageSettingsConfig_st.sensor_framerate->cur_setting == CamRate_Auto))
	}       // else (if (fps <= CamRate_Auto))

	 if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		 printk("CAMDRV_SetFrameRate(): Error[%d] \r\n",
			  sCamI2cStatus);
		 result = sCamI2cStatus;
	 }

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
{
	UInt16 readout;
	readout=mt9d115_read(0x3400);
	readout &= 0xFFFD;
	mt9d115_write(0x3400,readout);
	
	return HAL_CAM_SUCCESS;
	
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
	return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	UInt16 readout;
	printk("CAMDRV_SetCamSleep(): MT9D115 enter soft standby mode \r\n");
	
	readout=mt9d115_read(0x0028);	/* MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN] */
	readout&=0xFFFE;
	mt9d115_write(0x0028,readout);
	
	readout=mt9d115_read(0x0018);
	readout |= 0x1;
	mt9d115_write(0x0018,readout);
	msleep(3);
	
	readout=mt9d115_read(0x0018);
	if(!(readout&0x4000)){
		printk("failed: CAMDRV_SetCamSleep_Pri(): MT9D115 enter soft standby mode   \r\n");
		result=HAL_CAM_ERROR_OTHERS;
	}
	
	return result;
}

/*camera_disable for still mode */
static HAL_CAM_Result_en_t CAMDRV_DisableCapture_Pri(CamSensorSelect_t sensor)
{
	UInt16 readout;
	readout=mt9d115_read(0x3400);
	readout |= 0x2;
	mt9d115_write(0x3400,readout);
	printk("---DisableCapture : 0606 \n");
	return HAL_CAM_SUCCESS;
	
}

/****************************************************************************
/
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_DisablePreview(void)
/
/ Description: This function halts MT9D115 camera video
/
/ Notes:   camera_disable for preview mode , after this ,preview will not output data
/
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_DisablePreview_Pri(CamSensorSelect_t sensor)
{
	
	UInt16 readout;
	readout=mt9d115_read(0x3400);
	readout |= 0x2;
	mt9d115_write(0x3400,readout);
	printk("----DisablePreview : 0606 \n");
	return HAL_CAM_SUCCESS;
	
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
{//return HAL_CAM_SUCCESS;
	UInt32 x = 0, y = 0;
	UInt32 tx = 0, ty = 0;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	UInt32 data = 0, i = 10;
	/*printk("***************************** \
		CAMDRV_CfgStillnThumbCapture():STARTS ************************* \r\n");*/

	/*if(flash_switch==Flash_On)
		control_flash(1);*/

	switch (stills_resolution) {
	case CamImageSize_QCIF:

		x = 176;
		y = 144;
		break;

	case CamImageSize_QVGA:
		x = 320;
		y = 240;
		break;

	case CamImageSize_CIF:
		x = 352;
		y = 288;
		break;

	case CamImageSize_VGA:
		x = 640;
		y = 480;
		break;

	case CamImageSize_SVGA:
		x = 800;
		y = 600;
		break;

	case CamImageSize_XGA:
		x = 1024;
		y = 768;
		break;

	case CamImageSize_SXGA:
		x = 1280;
		y = 1024;
		break;

	case CamImageSize_UXGA:
		x = 1600;
		y = 1200;
		break;

	case CamImageSize_QXGA:
		x = 2048;
		y = 1536;
		break;

	default:
		x = 640;
		y = 480;
		break;
	}
	
#if 1
mt9d115_write( 0x098C, 0x2707);   //Output Width (A)
mt9d115_write( 0x0990, x);   //      = 640
mt9d115_write( 0x098C, 0x2709);   //Output Width (A)
mt9d115_write( 0x0990, y);   //      = 640
#endif 


mt9d115_write( 0x098C, 0xA115);   //Output Width (A)
mt9d115_write( 0x0990, 0x0002);   //      = 640
mt9d115_write( 0x098C, 0xA103);   //Output Width (A)
mt9d115_write( 0x0990, 0x0002);   //      = 640

//zhaoqiang@wind-mobi.com chg 2012.04.05 start
//add read reg 0xa104 for change to still mode correctlly
//reviewed by yuanlan@wind-mobi.com
msleep(200);

while(i--)
{
	mt9d115_write( 0x098C, 0xA104 );
	data = mt9d115_read(0x0990);
	msleep(50);
	printk("%s:data=0x%x\n",__func__,data);
	if (data == 0x07)
		break;
}
printk("i=%d\n",i);
if(i == 0)
{
	mt9d115_write( 0x098C, 0xA115);
	mt9d115_write( 0x0990, 0x0002);
	mt9d115_write( 0x098C, 0xA103);
	mt9d115_write( 0x0990, 0x0002);
	msleep(200);
}
//zhaoqiang@wind-mobi.com chg 2012.04.05 end

	{
		UInt16 readout;
		readout=mt9d115_read(0x3400);
		readout &= 0xFFFD;
		mt9d115_write(0x3400,readout);
	}	

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
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
		pr_info("CAMDRV_SetWBMode()  called  scene_mode = 0x%x  ------\n",sensor);
	switch(scene_mode) {
		case CamSceneMode_Auto:
			//hanwei@wind-mobi.com add 2012.02.06 start 
			//implement scenemode function
			//reviewed by liubing@wind-mobi.com
			pr_info("CAMDRV_SetSceneMode() called for -------AUTO------\n");
			//[Normal mode 18.6FPS8.4FPS]                   //Frame rate >=10fps
			mt9d115_write( 0x098C, 0xA20C); 	        // MCU_ADDRESS [AE_MAX_INDEX]
			mt9d115_write( 0x0990, 0x0014); 	        // MCU_DATA_0 18
			mt9d115_write( 0x098C, 0xA215); 	        // MCU_ADDRESS [AE_INDEX_TH23]
			mt9d115_write( 0x0990, 0x0004);//c); 	        // MCU_DATA_0   15  for zte objective test  night mode dull 20120628
			mt9d115_write( 0x098C, 0xA20E); 	        // MCU_ADDRESS [AE_MAX_VIRTGAIN]	//Edison, 2011-6-13: 0xA0->0x60
			mt9d115_write( 0x0990, 0x0068); 	        // MCU_DATA_0 60
			mt9d115_write( 0x098C, 0x2212); 	        // MCU_ADDRESS [AE_MAX_DGAIN_AE1]
			mt9d115_write( 0x0990, 0x00c0); 	        // MCU_DATA_0 120

			mt9d115_write( 0x098C, 0xA103); 	        // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write( 0x0990, 0x0005); 	        // MCU_DATA_0
			msleep(100);

			break;
		case CamSceneMode_Night:
			pr_info("CAMDRV_SetSceneMode() called for --------Night------\n");
			//[Night mode 18.6FPS—4.2FPS]                    //Frame rate >=5fps
			mt9d115_write( 0x098C, 0xA20C); 	        // MCU_ADDRESS [AE_MAX_INDEX]
			mt9d115_write( 0x0990, 0x0022);      //18); 	        // MCU_DATA_0  for night mode dull 20120628
			mt9d115_write( 0x098C, 0xA215); 	        // MCU_ADDRESS [AE_INDEX_TH23]
			mt9d115_write( 0x0990, 0x0022); 	  //18      // MCU_DATA_0    for zte objective test night mode dull 20120628
			mt9d115_write( 0x098C, 0xA20E); 	        // MCU_ADDRESS [AE_MAX_VIRTGAIN]	//Edison, 2011-6-13: 0xA0->0x60
			mt9d115_write( 0x0990, 0x0060); 	        // MCU_DATA_0
			mt9d115_write( 0x098C, 0x2212); 	        // MCU_ADDRESS [AE_MAX_DGAIN_AE1]
			mt9d115_write( 0x0990, 0x0120); 	        // MCU_DATA_0

			mt9d115_write( 0x098C, 0xA103); 	        // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write( 0x0990, 0x0005); 	        // MCU_DATA_0
			msleep(100);
			break;
		default:
			pr_info("CAMDRV_SetSceneMode() not supported for   ------default------\n");
			mt9d115_write( 0x098C, 0xA20E); 	        // MCU_ADDRESS [AE_MAX_VIRTGAIN]
			mt9d115_write( 0x0990, 0x0060);  	        // MCU_DATA_0  20110113 yellow question 0x90
			mt9d115_write( 0x098C, 0x2212);  	        // MCU_ADDRESS [AE_MAX_DGAIN_AE1]
			mt9d115_write( 0x0990, 0x0080); 	        // MCU_DATA_0  20110113 yellow question 0xA0
			mt9d115_write( 0x098C, 0xA20C); 	        // MCU_ADDRESS [AE_MAX_INDEX]
			mt9d115_write( 0x0990, 0x0010);  	        // MCU_DATA_0  20110113 yellow question 0x0C   0x0007
			mt9d115_write( 0x098C, 0xA215);  	        // MCU_ADDRESS [AE_INDEX_TH23]  
			mt9d115_write( 0x0990, 0x000C);  	        // MCU_DATA_0  20110113 yellow question 0x0C  0x0007
			                                 
			mt9d115_write( 0x098C, 0xA103);  	         // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write( 0x0990, 0x0005);  	         // MCU_DATA_0
			msleep(100);
			break;
			//hanwei@wind-mobi.com add 2012.02.06 end
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
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetWBMode()  called  wb_mode = 0x%x \n",wb_mode);
	//hanwei@wind-mobi.com add 2012.02.06 start 
	//implement WBMode function
	//reviewed by liubing@wind-mobi.com
	switch (wb_mode)
	{
		case CamWB_Auto:	
			printk("CamWB_Auto\r\n");
			//[WB_auto]
			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x0059);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x00C8);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x0059);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x00A6);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x0000);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x007F);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0
			break;	
		case CamWB_Daylight:	//sunny	
			printk("CamWB_Daylight\r\n");	
			//[WB_daylight]
			mt9d115_write(0x098C, 0xA34A);  	// MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x00C2); 	        // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);  	// MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x00c4);  	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);  	// MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x005d);  	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);  	// MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x005f);  	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);  	// MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x007f);  	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);  	// MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x007F);  	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);  	// MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);  	// MCU_DATA_0
			break;	
	
		case CamWB_Fluorescent:			
			printk("CamWB_Fluorescent\r\n");
			//[WB_flourescant]
			////fix bug10225 fluorescent greenish 2012.04.12
			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x0086);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x0088);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x0092);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x0094);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x0000);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x0000);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0
			break;

		case CamWB_Cloudy:		
			printk("CamWB_Fluorescent\r\n");
			//[WB_flourescant]
			#if 0  //fix bug9267 fluorescent greenish 2012.04.12
			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x008C);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x008C);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x0081);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x0081);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x0066);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x0066);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0
			#else
			/*
			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x009C);    //0x0088	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x009C);    //0x008A	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x0052);    //0x0090	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x0052);    //0x0092	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x007F);    //0x0010	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x007F);    //0x0000	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0
			*/
			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x0090);    //0x0088	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x0090);    //0x008A	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x0070);    //0x0090	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x0070);    //0x0092	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x004F);    //0x0010	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x004F);    //0x0000	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0
			#endif
			
			break;
		case CamWB_Incandescent:	//home	
			printk("CamWB_Incandescent\r\n");
			//[WB_incandescent A LIGHT]


			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x0080);	//0x008C);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x0080);    //0x008C	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x00B0);    //0x0081	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x00B0);    //0x0081	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x0000);    //0x0066	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x0000);    //0x0066	// MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0
			break;	
		case CamWB_Twilight:
			printk("CamWB_Twilight\r\n");
			mt9d115_write(0x098C, 0xA34A);    // MCU_ADDRESS [AWB_GAIN_MIN]
			mt9d115_write(0x0990, 0x0080);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34B);    // MCU_ADDRESS [AWB_GAIN_MAX]
			mt9d115_write(0x0990, 0x0080);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34C);    // MCU_ADDRESS [AWB_GAINMIN_B]
			mt9d115_write(0x0990, 0x0086);	//A0);    //B0);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA34D);    // MCU_ADDRESS [AWB_GAINMAX_B]
			mt9d115_write(0x0990, 0x0086);	//A0);    //B0);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA351);    // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
			mt9d115_write(0x0990, 0x0000);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA352);    // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
			mt9d115_write(0x0990, 0x0000);    // MCU_DATA_0
			mt9d115_write(0x098C, 0xA103);    // MCU_ADDRESS [SEQ_CMD]
			mt9d115_write(0x0990, 0x0005);    // MCU_DATA_0

			break;
		case CamWB_Tungsten:
			printk("CamWB_Tungsten\r\n");
			//
			//
			break;
		default:
			printk("HAL_CAM_ERROR_ACTION_NOT_SUPPORTED\r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
	}
	//hanwei@wind-mobi.com add 2012.02.06 end
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
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com add 2012.02.06 start 
	//implement Antibanding function
	//reviewed by liubing@wind-mobi.com
	
	pr_info("CAMDRV_SetAntiBanding()  called\n");
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) {
		//[ANTIBANDING_SET_AUTO]
		printk("  case : CamAntiBanding AUTO \n");
		mt9d115_write(0x098C, 0xA118); 	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]          
		mt9d115_write(0x0990, 0x0001); 	// MCU_DATA_0                              
		mt9d115_write(0x098C, 0xA11E); 	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]         
		mt9d115_write(0x0990, 0x0001); 	// MCU_DATA_0                            
		mt9d115_write(0x098C, 0xA124); 	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]         
		mt9d115_write(0x0990, 0x0000); 	// MCU_DATA_0                                  
		mt9d115_write(0x098C, 0xA12A); 	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]          
		mt9d115_write(0x0990, 0x0001); 	// MCU_DATA_0  
		mt9d115_write(0x098C, 0xA404); 	// MCU_ADDRESS [FD_MODE]                      
		mt9d115_write(0x0990, 0x0060); 	// MCU_DATA_0       
		
		mt9d115_write(0x098C, 0xA103); 	                  
		mt9d115_write(0x0990, 0x0005); 	

	} else if (effect == CamAntiBanding50Hz) {
		printk("  case : CamAntiBanding50Hz \n");
		//[50HZ]
		mt9d115_write(0x098C, 0xA118); 	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]        
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                                 
		mt9d115_write(0x098C, 0xA11E); 	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]             
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                                 
		mt9d115_write(0x098C, 0xA124); 	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]             
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                                     
		mt9d115_write(0x098C, 0xA12A); 	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]            
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                                  
		mt9d115_write(0x098C, 0xA404); 	// MCU_ADDRESS [FD_MODE]                       
		mt9d115_write(0x0990, 0x00E0); 	// MCU_DATA_0   
		
		mt9d115_write(0x098C, 0xA103);          
		mt9d115_write(0x0990, 0x0005);

	} else if (effect == CamAntiBanding60Hz) {
		printk("  case : CamAntiBanding60Hz \n");
		//[60HZ]
		mt9d115_write(0x098C, 0xA118); 	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]           
		mt9d115_write(0x0990, 0x0002); 	// mt9d115_write(                                
		mt9d115_write(0x098C, 0xA11E); 	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]            
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                               
		mt9d115_write(0x098C, 0xA124); 	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]           
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                                   
		mt9d115_write(0x098C, 0xA12A); 	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]           
		mt9d115_write(0x0990, 0x0002); 	// MCU_DATA_0                                 
		mt9d115_write(0x098C, 0xA404); 	// MCU_ADDRESS [FD_MODE]                      
		mt9d115_write(0x0990, 0x00A0); 	// MCU_DATA_0    
		
		mt9d115_write(0x098C, 0xA103);           
		mt9d115_write(0x0990, 0x0005);

	} else {
		//[ANTIBANDING_SET_OFF]
		printk("  case : CamAntiBanding OFF \n");
		mt9d115_write(0x098C, 0xA118); 	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]          
		mt9d115_write(0x0990, 0x0000); 	// MCU_DATA_0                              
		mt9d115_write(0x098C, 0xA11E); 	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]         
		mt9d115_write(0x0990, 0x0000); 	// MCU_DATA_0                            
		mt9d115_write(0x098C, 0xA124); 	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]         
		mt9d115_write(0x0990, 0x0000); 	// MCU_DATA_0                                  
		mt9d115_write(0x098C, 0xA12A); 	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]          
		mt9d115_write(0x0990, 0x0000); 	// MCU_DATA_0  
		mt9d115_write(0x098C, 0xA404); 	// MCU_ADDRESS [FD_MODE]                      
		mt9d115_write(0x0990, 0x0060); 	// MCU_DATA_0    
		
		mt9d115_write(0x098C, 0xA103); 	                  
		mt9d115_write(0x0990, 0x0005); 
	}
	//hanwei@wind-mobi.com add 2012.02.06 end
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
{return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetFlashMode()  called\n");
	
	flash_switch=effect;
	if (effect == Flash_Off) {
		control_flash(0);
	} else if (effect == Flash_On) {/*used by camera flash, not torch  apk*/
		;//control_flash(1);
	} else if (effect == Torch_On) {/*used by  torch  to turn on flash */
		control_flash(1);
	} else if (effect == FlashLight_Auto) { /*close flash first ,*/
		control_flash(0);
	} else {
		printk(KERN_ERR"CAMDRV_SetFlashMode_Pri: unknow FlashMode %d ", effect);
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
{return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetFocusMode()  called effect 0x%x \n",effect);
	if (effect == CamFocusControlAuto) {

		mt9d115_write( 0x098E, 0x3003); 	// MCU_ADDRESS [AF_ALGO]
		mt9d115_write( 0x0990, 0x0002); 	// MCU_DATA_0		
		
	} else if (effect == CamFocusControlMacro) {

		mt9d115_write( 0x098E, 0x3003 );	// MCU_ADDRESS [AF_ALGO]
		mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0
		mt9d115_write( 0x098E, 0xB024 );	// MCU_ADDRESS [AF_BEST_POSITION]
		mt9d115_write( 0x0990, 0x00AF );	// MCU_DATA_0

	} else if (effect == CamFocusControlInfinity) {

		mt9d115_write( 0x098E, 0x3003 );	// MCU_ADDRESS [AF_ALGO]
		mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0
		mt9d115_write( 0x098E, 0xB024 );	// MCU_ADDRESS [AF_BEST_POSITION]
		mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0

	} else {

		printk(KERN_ERR" can not support effect focus 0x%x\r\n",effect);

	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetFocusMode(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	
	FOCUS_MODE=effect;
	
	return result;
}

static HAL_CAM_Result_en_t CAMDRV_TurnOffAF_Pri()
{
	
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	return result;
}

static HAL_CAM_Result_en_t CAMDRV_TurnOnAF_Pri()
{return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	switch (FOCUS_MODE)
	{
		case CamFocusControlAuto: /*AF trigger*/
			
			mt9d115_write( 0x098E, 0xB019 );	// MCU_ADDRESS [AF_PROGRESS]
			mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0

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

/****************************************************************************
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetZoom()
/
/ Description: This function will set the zoom value of camera
/ Notes:
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetZoom_Pri(CamZoom_t step,
					  CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetZoom()  called\n");
	if (step == CamZoom_1_0) {
		//TBD
	} else if (step == CamZoom_1_15) {
		//TBD
	} else if (step == CamZoom_1_6) {
		//TBD
	} else if (step == CamZoom_2_0) {
		//TBD
	} else {
		//TBD
	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetZoom(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
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
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetDigitalEffect_Pri()  called  DigitalEffect_value  = 0x%x \n",sensor);
	//hanwei@wind-mobi.com add 2012.02.06 start 
	//implement DigitalEffect  function
	//reviewed by liubing@wind-mobi.com
	switch (effect)
	{	
		case CamDigEffect_NoEffect:		
			printk(" CamDigEffect_NoEffect \r\n");
			mt9d115_write(0x098C, 0x2759); 	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_FX_SEPIA_SETTINGS]
			mt9d115_write(0x0990, 0x6440); 	// MCU_DATA_0 
			
			mt9d115_write(0x098C, 0x275B);           
			mt9d115_write(0x0990, 0x6440);                   
			mt9d115_write(0x098C, 0xA103);            
			mt9d115_write(0x0990, 0x0005);
			break;	
			
		case CamDigEffect_SepiaGreen:	//antique
			printk(" CamDigEffect_SepiaGreen \r\n");
			//[CAMERA_EFFECT_SEPIA] 
			mt9d115_write(0x098C, 0x2763); 	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_FX_SEPIA_SETTINGS]
			mt9d115_write(0x0990, 0xB023); 	// MCU_DATA_0 
			      
			mt9d115_write(0x098C, 0x2759);           
			mt9d115_write(0x0990, 0x6442);            
			mt9d115_write(0x098C, 0x275B);           
			mt9d115_write(0x0990, 0x6442);            
			mt9d115_write(0x098C, 0xA103);            
			mt9d115_write(0x0990, 0x0005);
			break;	
			
		case CamDigEffect_MonoChrome:	//B&W	
			printk(" CamDigEffect_MonoChrome \r\n");
			//[CAMERA_EFFECT_MONO] 
			mt9d115_write(0x098C, 0x2759);          
			mt9d115_write(0x0990, 0x6441);            
			mt9d115_write(0x098C, 0x275B);             
			mt9d115_write(0x0990, 0x6441);             
			mt9d115_write(0x098C, 0xA103);            
			mt9d115_write(0x0990, 0x0005); 
			break;	
			
		case CamDigEffect_NegColor:		//Negative
			printk(" CamDigEffect_NegColor \r\n");
			//[CAMERA_EFFECT_NEGATIVE]
			mt9d115_write(0x098C, 0x2759);           
			mt9d115_write(0x0990, 0x6443);          
			mt9d115_write(0x098C, 0x275B);           
			mt9d115_write(0x0990, 0x6443);           
			mt9d115_write(0x098C, 0xA103);           
			mt9d115_write(0x0990, 0x0005);
			break;
			
		case CamDigEffect_Auqa:
			printk(" CamDigEffect_Auqa \r\n");
			mt9d115_write(0x098C, 0x2763);           
			mt9d115_write(0x0990, 0xC3CE);          
			mt9d115_write(0x098C, 0x2759);           
			mt9d115_write(0x0990, 0x6442);           
			mt9d115_write(0x098C, 0x275B);           
			mt9d115_write(0x0990, 0x6442);
			mt9d115_write(0x098C, 0xA103);           
			mt9d115_write(0x0990, 0x0005);
			break;
			
		case CamDigEffect_SolarizeColor:
		case CamDigEffect_SolarizeMono:
			printk(" CamDigEffect_Solarize \r\n");
			mt9d115_write(0x098C, 0x2759);           
			mt9d115_write(0x0990, 0x6443);          
			mt9d115_write(0x098C, 0x275B);           
			mt9d115_write(0x0990, 0x6443);           
			mt9d115_write(0x098C, 0xA103);                   
			mt9d115_write(0x0990, 0x0005);
			break;
			
		case CamDigEffect_Posterize:
		case CamDigEffect_PosterizeMono:
			printk(" CamDigEffect_PosterizeMono \r\n");
			//
			//
			break;			
		default:
			printk(" HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
	}	
	//hanwei@wind-mobi.com add 2012.02.06 end
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

static HAL_CAM_Result_en_t Init_MT9D115(CamSensorSelect_t sensor)
{
		HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
		static ktime_t tm1,tm2;
		tm1 = ktime_get();
		
		//pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);
		
		CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;
		printk("\n-----------Init_MT9D115-------Start------\r\n");
		
		/*	
		   * Preview YUV 640x480  Frame rate 10~ 27.5fps
			  * Capture YUV 2048x1536	Frame Rate 8.45
			  * XMCLK 26MHz PCLK 64MHz
		   */
       #if defined (CONFIG_CAM_CSI2)
		   //printk(KERN_ERR"Init_MT9D115 CONFIG_CAM_CSI2 has been defined\n ");
	#endif 
		
		   checkCameraID(sensor);
		//reset MCLK=24M . PCLK=240;

		
		msleep(20);      //msleep(100);	//modify by zhaoqiang

mt9d115_write( 0x001A, 0x0051 );	// RESET_AND_MISC_CONTROL
msleep(2);
mt9d115_write( 0x001A, 0x0050 );	// RESET_AND_MISC_CONTROL
mt9d115_write( 0x001A, 0x0058 );	// RESET_AND_MISC_CONTROL
msleep(10);
mt9d115_write( 0x0014, 0x21F9 );	// PLL_CONTROL

//// For 12MHz, 15fps fixed.
//mt9d115_write( 0x0010, 0x0015 ); //PLL Dividers = 21  del for zte objective test recording slow 20120628
////
//// For 12MHz, 7.5fps fixed.
mt9d115_write( 0x0010, 0x0115);   //PLL Dividers = 21   add for zte objective test recording slow 20120628
////

mt9d115_write( 0x0012, 0x00F5 );	// PLL_P_DIVIDERS
mt9d115_write( 0x0014, 0x2545 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2547 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2447 );	// PLL_CONTROL
msleep(10);
mt9d115_write( 0x0014, 0x2047 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2046 );	// PLL_CONTROL
mt9d115_write( 0x0018, 0x402D );	// STANDBY_CONTROL
mt9d115_write( 0x0018, 0x402C );	// STANDBY_CONTROL
//  POLL  STANDBY_CONTROL::STANDBY_DONE =>  0x01, 0x00
msleep(10);		//modify by zhaoqiang

mt9d115_write( 0x98C, 0x2703	);   //Output Width (A)
mt9d115_write( 0x990, 0x0280	);   //      = 640
mt9d115_write( 0x98C, 0x2705	);   //Output Height (A)
mt9d115_write( 0x990, 0x01E0	);   //      = 480
mt9d115_write( 0x98C, 0x2707	);   //Output Width (B)
mt9d115_write( 0x990, 0x0640	);   //      = 1600
mt9d115_write( 0x98C, 0x2709	);   //Output Height (B)
mt9d115_write( 0x990, 0x04B0	);   //      = 1200
mt9d115_write( 0x98C, 0x270D	);   //Row Start (A)
mt9d115_write( 0x990, 0x000	  );   //      = 0
mt9d115_write( 0x98C, 0x270F	);   //Column Start (A)
mt9d115_write( 0x990, 0x000	  );   //      = 0
mt9d115_write( 0x98C, 0x2711	);   //Row End (A)
mt9d115_write( 0x990, 0x4BD	  );   //      = 1213
mt9d115_write( 0x98C, 0x2713	);   //Column End (A)
mt9d115_write( 0x990, 0x64D	  );   //      = 1613
mt9d115_write( 0x98C, 0x2715	);   //Row Speed (A)
mt9d115_write( 0x990, 0x0111	);   //      = 273
mt9d115_write( 0x98C, 0x2717	);   //Read Mode (A)
mt9d115_write( 0x990, 0x046C	);   //      = 1132
mt9d115_write( 0x98C, 0x2719	);   //sensor_fine_correction (A)
mt9d115_write( 0x990, 0x005A	);   //      = 90
mt9d115_write( 0x98C, 0x271B	);   //sensor_fine_IT_min (A)
mt9d115_write( 0x990, 0x01BE	);   //      = 446
mt9d115_write( 0x98C, 0x271D	);   //sensor_fine_IT_max_margin (A)
mt9d115_write( 0x990, 0x0131	);   //      = 305
mt9d115_write( 0x98C, 0x271F	);   //Frame Lines (A)
mt9d115_write( 0x990, 0x02B3	);   //      = 691
mt9d115_write( 0x98C, 0x2721	);   //Line Length (A)
mt9d115_write( 0x990, 0x0FD4	);   //      = 4052
mt9d115_write( 0x98C, 0x2723	);   //Row Start (B)
mt9d115_write( 0x990, 0x004	  );    //      = 4
mt9d115_write( 0x98C, 0x2725	);   //Column Start (B)
mt9d115_write( 0x990, 0x004	  );   //      = 4
mt9d115_write( 0x98C, 0x2727	);   //Row End (B)
mt9d115_write( 0x990, 0x4BB	  );    //      = 1211
mt9d115_write( 0x98C, 0x2729	);   //Column End (B)
mt9d115_write( 0x990, 0x64B	  );    //      = 1611
mt9d115_write( 0x98C, 0x272B	);   //Row Speed (B)
mt9d115_write( 0x990, 0x0111	);   //      = 273
mt9d115_write( 0x98C, 0x272D	);   //Read Mode (B)
mt9d115_write( 0x990, 0x0024	);   //      = 36
mt9d115_write( 0x98C, 0x272F	);   //sensor_fine_correction (B)
mt9d115_write( 0x990, 0x003A	);   //      = 58
mt9d115_write( 0x98C, 0x2731	);   //sensor_fine_IT_min (B)
mt9d115_write( 0x990, 0x00F6	);   //      = 246
mt9d115_write( 0x98C, 0x2733	);   //sensor_fine_IT_max_margin (B)
mt9d115_write( 0x990, 0x008B	);   //      = 139
mt9d115_write( 0x98C, 0x2735	);   //Frame Lines (B)
mt9d115_write( 0x990, 0x050D	);   //      = 1293
mt9d115_write( 0x98C, 0x2737	);   //Line Length (B)
mt9d115_write( 0x990, 0x0F04	);   //      = 3844
mt9d115_write( 0x98C, 0x2739	);   //Crop_X0 (A)
mt9d115_write( 0x990, 0x0000	);   //      = 0
mt9d115_write( 0x98C, 0x273B	);   //Crop_X1 (A)
mt9d115_write( 0x990, 0x031F	);   //      = 799
mt9d115_write( 0x98C, 0x273D	);   //Crop_Y0 (A)
mt9d115_write( 0x990, 0x0000	);   //      = 0
mt9d115_write( 0x98C, 0x273F	);   //Crop_Y1 (A)
mt9d115_write( 0x990, 0x0257	);   //      = 599
mt9d115_write( 0x98C, 0x2747	);   //Crop_X0 (B)
mt9d115_write( 0x990, 0x0000	);   //      = 0
mt9d115_write( 0x98C, 0x2749	);   //Crop_X1 (B)
mt9d115_write( 0x990, 0x063F	);   //      = 1599
mt9d115_write( 0x98C, 0x274B	);   //Crop_Y0 (B)
mt9d115_write( 0x990, 0x0000	);   //      = 0
mt9d115_write( 0x98C, 0x274D	);   //Crop_Y1 (B)
mt9d115_write( 0x990, 0x04AF	);   //      = 1199
mt9d115_write( 0x98C, 0x222D	);   //R9 Step
mt9d115_write( 0x990, 0x0056	);   //      = 86
mt9d115_write( 0x98C, 0xA408	);   //search_f1_50
mt9d115_write( 0x990, 0x14	  );   //      = 20
mt9d115_write( 0x98C, 0xA409	);   //search_f2_50
mt9d115_write( 0x990, 0x16	  );    //      = 22
mt9d115_write( 0x98C, 0xA40A	);   //search_f1_60
mt9d115_write( 0x990, 0x19	  );    //      = 25
mt9d115_write( 0x98C, 0xA40B	);   //search_f2_60
mt9d115_write( 0x990, 0x1B	  );   //      = 27
mt9d115_write( 0x98C, 0x2411	);   //R9_Step_60 (A)
mt9d115_write( 0x990, 0x0056	);   //      = 86
mt9d115_write( 0x98C, 0x2413	);   //R9_Step_50 (A)
mt9d115_write( 0x990, 0x0068	);   //      = 104
mt9d115_write( 0x98C, 0x2415	);   //R9_Step_60 (B)
mt9d115_write( 0x990, 0x005B	);   //      = 91
mt9d115_write( 0x98C, 0x2417	);   //R9_Step_50 (B)
mt9d115_write( 0x990, 0x006D	);   //      = 109
mt9d115_write( 0x98C, 0xA404	);   //FD Mode
mt9d115_write( 0x990, 0x10	  );   //      = 16
mt9d115_write( 0x98C, 0xA40D	);   //Stat_min
mt9d115_write( 0x990, 0x02	  );   //      = 2
mt9d115_write( 0x98C, 0xA40E	);   //Stat_max
mt9d115_write( 0x990, 0x03	  );   //      = 3
mt9d115_write( 0x98C, 0xA410	);   //Min_amplitude
mt9d115_write( 0x990, 0x0A	  );     //      = 10
mt9d115_write( 0x098C, 0xA117 );   	// MCU_ADDRESS [SEQ_PREVIEW_0_AE]
mt9d115_write( 0x0990, 0x0002 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA11D );   	// MCU_ADDRESS [SEQ_PREVIEW_1_AE]
mt9d115_write( 0x0990, 0x0002 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA129 );   	// MCU_ADDRESS [SEQ_PREVIEW_3_AE]
mt9d115_write( 0x0990, 0x0002 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA24F );   	// MCU_ADDRESS [AE_BASETARGET]
mt9d115_write( 0x0990, 0x0032 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA20C );   	// MCU_ADDRESS [AE_MAX_INDEX]
mt9d115_write( 0x0990, 0x0012 );   	// MCU_DATA_0 10		 //modify by zhaoqiang
mt9d115_write( 0x098C, 0xA216 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x0091 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA20E );   	// MCU_ADDRESS [AE_MAX_VIRTGAIN]
mt9d115_write( 0x0990, 0x0068 );   	// MCU_DATA_0 91		 //modify by zhaoqiang
mt9d115_write( 0x098C, 0x2212 );   	// MCU_ADDRESS [AE_MAX_DGAIN_AE1]
mt9d115_write( 0x0990, 0x00c0 );   	// MCU_DATA_0 a4 		//modify by zhaoqiang
mt9d115_write( 0x3210, 0x01B8 );   	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x098C, 0xAB36 );   	// MCU_ADDRESS [HG_CLUSTERDC_TH]
mt9d115_write( 0x0990, 0x0014 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B66 );   	// MCU_ADDRESS [HG_CLUSTER_DC_BM]
mt9d115_write( 0x0990, 0x2AF8 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB20 );   	// MCU_ADDRESS [HG_LL_SAT1]
mt9d115_write( 0x0990, 0x0080 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB24 );   	// MCU_ADDRESS [HG_LL_SAT2]
mt9d115_write( 0x0990, 0x0000 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB21 );   	// MCU_ADDRESS [HG_LL_INTERPTHRESH1]
mt9d115_write( 0x0990, 0x000A );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB25 );   	// MCU_ADDRESS [HG_LL_INTERPTHRESH2]
mt9d115_write( 0x0990, 0x002A );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB22 );   	// MCU_ADDRESS [HG_LL_APCORR1]
mt9d115_write( 0x0990, 0x0007 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB26 );   	// MCU_ADDRESS [HG_LL_APCORR2]
mt9d115_write( 0x0990, 0x0001 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB23 );   	// MCU_ADDRESS [HG_LL_APTHRESH1]
mt9d115_write( 0x0990, 0x0004 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB27 );   	// MCU_ADDRESS [HG_LL_APTHRESH2]
mt9d115_write( 0x0990, 0x0009 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B28 );   	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]
mt9d115_write( 0x0990, 0x0BB8 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B2A );   	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]
mt9d115_write( 0x0990, 0x2968 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2C );   	// MCU_ADDRESS [HG_NR_START_R]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB30 );   	// MCU_ADDRESS [HG_NR_STOP_R]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2D );   	// MCU_ADDRESS [HG_NR_START_G]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB31 );   	// MCU_ADDRESS [HG_NR_STOP_G]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2E );   	// MCU_ADDRESS [HG_NR_START_B]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB32 );   	// MCU_ADDRESS [HG_NR_STOP_B]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2F );   	// MCU_ADDRESS [HG_NR_START_OL]
mt9d115_write( 0x0990, 0x000A );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB33 );   	// MCU_ADDRESS [HG_NR_STOP_OL]
mt9d115_write( 0x0990, 0x0006 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB34 );   	// MCU_ADDRESS [HG_NR_GAINSTART]
mt9d115_write( 0x0990, 0x0020 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB35 );   	// MCU_ADDRESS [HG_NR_GAINSTOP]
mt9d115_write( 0x0990, 0x0091 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA765 );   	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_FILTER_MODE]
mt9d115_write( 0x0990, 0x0006 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB37 );   	// MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]
mt9d115_write( 0x0990, 0x0003 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B38 );   	// MCU_ADDRESS [HG_GAMMASTARTMORPH]
mt9d115_write( 0x0990, 0x2968 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B3A );   	// MCU_ADDRESS [HG_GAMMASTOPMORPH]
mt9d115_write( 0x0990, 0x2D50 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B62 );   	// MCU_ADDRESS [HG_FTB_START_BM]
mt9d115_write( 0x0990, 0xFFFE );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B64 );   	// MCU_ADDRESS [HG_FTB_STOP_BM]
mt9d115_write( 0x0990, 0xFFFF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4F );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
mt9d115_write( 0x0990, 0x0000 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB50 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
mt9d115_write( 0x0990, 0x0013 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB51 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
mt9d115_write( 0x0990, 0x0027 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB52 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
mt9d115_write( 0x0990, 0x0043 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB53 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
mt9d115_write( 0x0990, 0x0068 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB54 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
mt9d115_write( 0x0990, 0x0081 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB55 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
mt9d115_write( 0x0990, 0x0093 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB56 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
mt9d115_write( 0x0990, 0x00A3 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB57 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
mt9d115_write( 0x0990, 0x00B0 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB58 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
mt9d115_write( 0x0990, 0x00BC );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB59 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
mt9d115_write( 0x0990, 0x00C7 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5A );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
mt9d115_write( 0x0990, 0x00D1 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5B );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
mt9d115_write( 0x0990, 0x00DA );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5C );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
mt9d115_write( 0x0990, 0x00E2 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5D );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
mt9d115_write( 0x0990, 0x00E9 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5E );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
mt9d115_write( 0x0990, 0x00EF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5F );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
mt9d115_write( 0x0990, 0x00F4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB60 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
mt9d115_write( 0x0990, 0x00FA );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB61 );   	// MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
mt9d115_write( 0x0990, 0x00FF );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2306 );   	// MCU_ADDRESS [AWB_CCM_L_0]
mt9d115_write( 0x0990, 0x01D6 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2308 );   	// MCU_ADDRESS [AWB_CCM_L_1]
mt9d115_write( 0x0990, 0xFF89 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230A );   	// MCU_ADDRESS [AWB_CCM_L_2]
mt9d115_write( 0x0990, 0xFFA1 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230C );   	// MCU_ADDRESS [AWB_CCM_L_3]
mt9d115_write( 0x0990, 0xFF73 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230E );   	// MCU_ADDRESS [AWB_CCM_L_4]
mt9d115_write( 0x0990, 0x019C );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2310 );   	// MCU_ADDRESS [AWB_CCM_L_5]
mt9d115_write( 0x0990, 0xFFF1 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2312 );   	// MCU_ADDRESS [AWB_CCM_L_6]
mt9d115_write( 0x0990, 0xFFB0 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2314 );   	// MCU_ADDRESS [AWB_CCM_L_7]
mt9d115_write( 0x0990, 0xFF2D );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2316 );   	// MCU_ADDRESS [AWB_CCM_L_8]
mt9d115_write( 0x0990, 0x0223 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );   	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001C );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );   	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0048 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );   	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001C );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );   	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );   	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001E );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );   	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );   	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x0022 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );   	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );   	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x002C );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );   	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );   	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x0024 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );   	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231C );   	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9d115_write( 0x0990, 0xFFCD );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231E );   	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9d115_write( 0x0990, 0x0023 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2320 );   	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9d115_write( 0x0990, 0x0010 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2322 );   	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9d115_write( 0x0990, 0x0026 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2324 );   	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9d115_write( 0x0990, 0xFFE9 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2326 );   	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9d115_write( 0x0990, 0xFFF1 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2328 );   	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9d115_write( 0x0990, 0x003A );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232A );   	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9d115_write( 0x0990, 0x005D );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232C );   	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9d115_write( 0x0990, 0xFF69 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );   	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000C );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );   	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFE4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );   	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000C );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );   	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );   	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000A );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );   	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );   	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x0006 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );   	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );   	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0xFFFC );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );   	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );   	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x0004 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );   	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0x0415 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xF601 );   
mt9d115_write( 0x0992, 0x42C1 );   
mt9d115_write( 0x0994, 0x0326 );   
mt9d115_write( 0x0996, 0x11F6 );   
mt9d115_write( 0x0998, 0x0143 );   
mt9d115_write( 0x099A, 0xC104 );   
mt9d115_write( 0x099C, 0x260A );   
mt9d115_write( 0x099E, 0xCC04 );   
mt9d115_write( 0x098C, 0x0425 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x33BD );   
mt9d115_write( 0x0992, 0xA362 );   
mt9d115_write( 0x0994, 0xBD04 );   
mt9d115_write( 0x0996, 0x3339 );   
mt9d115_write( 0x0998, 0xC6FF );   
mt9d115_write( 0x099A, 0xF701 );   
mt9d115_write( 0x099C, 0x6439 );   
mt9d115_write( 0x099E, 0xFE01 );   
mt9d115_write( 0x098C, 0x0435 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x6918 );   
mt9d115_write( 0x0992, 0xCE03 );   
mt9d115_write( 0x0994, 0x25CC );   
mt9d115_write( 0x0996, 0x0013 );   
mt9d115_write( 0x0998, 0xBDC2 );   
mt9d115_write( 0x099A, 0xB8CC );   
mt9d115_write( 0x099C, 0x0489 );   
mt9d115_write( 0x099E, 0xFD03 );   
mt9d115_write( 0x098C, 0x0445 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x27CC );   
mt9d115_write( 0x0992, 0x0325 );   
mt9d115_write( 0x0994, 0xFD01 );   
mt9d115_write( 0x0996, 0x69FE );   
mt9d115_write( 0x0998, 0x02BD );   
mt9d115_write( 0x099A, 0x18CE );   
mt9d115_write( 0x099C, 0x0339 );   
mt9d115_write( 0x099E, 0xCC00 );   
mt9d115_write( 0x098C, 0x0455 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x11BD );   
mt9d115_write( 0x0992, 0xC2B8 );   
mt9d115_write( 0x0994, 0xCC04 );   
mt9d115_write( 0x0996, 0xC8FD );   
mt9d115_write( 0x0998, 0x0347 );   
mt9d115_write( 0x099A, 0xCC03 );   
mt9d115_write( 0x099C, 0x39FD );   
mt9d115_write( 0x099E, 0x02BD );   
mt9d115_write( 0x098C, 0x0465 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xDE00 );   
mt9d115_write( 0x0992, 0x18CE );   
mt9d115_write( 0x0994, 0x00C2 );   
mt9d115_write( 0x0996, 0xCC00 );   
mt9d115_write( 0x0998, 0x37BD );   
mt9d115_write( 0x099A, 0xC2B8 );   
mt9d115_write( 0x099C, 0xCC04 );   
mt9d115_write( 0x099E, 0xEFDD );   
mt9d115_write( 0x098C, 0x0475 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xE6CC );   
mt9d115_write( 0x0992, 0x00C2 );   
mt9d115_write( 0x0994, 0xDD00 );   
mt9d115_write( 0x0996, 0xC601 );   
mt9d115_write( 0x0998, 0xF701 );   
mt9d115_write( 0x099A, 0x64C6 );   
mt9d115_write( 0x099C, 0x03F7 );   
mt9d115_write( 0x099E, 0x0165 );   
mt9d115_write( 0x098C, 0x0485 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x7F01 );   
mt9d115_write( 0x0992, 0x6639 );   
mt9d115_write( 0x0994, 0x3C3C );   
mt9d115_write( 0x0996, 0x3C34 );   
mt9d115_write( 0x0998, 0xCC32 );   
mt9d115_write( 0x099A, 0x3EBD );   
mt9d115_write( 0x099C, 0xA558 );   
mt9d115_write( 0x099E, 0x30ED );   
mt9d115_write( 0x098C, 0x0495 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x04BD );   
mt9d115_write( 0x0992, 0xB2D7 );   
mt9d115_write( 0x0994, 0x30E7 );   
mt9d115_write( 0x0996, 0x06CC );   
mt9d115_write( 0x0998, 0x323E );   
mt9d115_write( 0x099A, 0xED00 );   
mt9d115_write( 0x099C, 0xEC04 );   
mt9d115_write( 0x099E, 0xBDA5 );   
mt9d115_write( 0x098C, 0x04A5 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x44CC );   
mt9d115_write( 0x0992, 0x3244 );   
mt9d115_write( 0x0994, 0xBDA5 );   
mt9d115_write( 0x0996, 0x585F );   
mt9d115_write( 0x0998, 0x30ED );   
mt9d115_write( 0x099A, 0x02CC );   
mt9d115_write( 0x099C, 0x3244 );   
mt9d115_write( 0x099E, 0xED00 );   
mt9d115_write( 0x098C, 0x04B5 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xF601 );   
mt9d115_write( 0x0992, 0xD54F );   
mt9d115_write( 0x0994, 0xEA03 );   
mt9d115_write( 0x0996, 0xAA02 );   
mt9d115_write( 0x0998, 0xBDA5 );   
mt9d115_write( 0x099A, 0x4430 );   
mt9d115_write( 0x099C, 0xE606 );   
mt9d115_write( 0x099E, 0x3838 );   
mt9d115_write( 0x098C, 0x04C5 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x3831 );   
mt9d115_write( 0x0992, 0x39BD );   
mt9d115_write( 0x0994, 0xD661 );   
mt9d115_write( 0x0996, 0xF602 );   
mt9d115_write( 0x0998, 0xF4C1 );   
mt9d115_write( 0x099A, 0x0126 );   
mt9d115_write( 0x099C, 0x0BFE );   
mt9d115_write( 0x099E, 0x02BD );   
mt9d115_write( 0x098C, 0x04D5 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xEE10 );   
mt9d115_write( 0x0992, 0xFC02 );   
mt9d115_write( 0x0994, 0xF5AD );   
mt9d115_write( 0x0996, 0x0039 );   
mt9d115_write( 0x0998, 0xF602 );   
mt9d115_write( 0x099A, 0xF4C1 );   
mt9d115_write( 0x099C, 0x0226 );   
mt9d115_write( 0x099E, 0x0AFE );   
mt9d115_write( 0x098C, 0x04E5 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x02BD );   
mt9d115_write( 0x0992, 0xEE10 );   
mt9d115_write( 0x0994, 0xFC02 );   
mt9d115_write( 0x0996, 0xF7AD );   
mt9d115_write( 0x0998, 0x0039 );   
mt9d115_write( 0x099A, 0x3CBD );   
mt9d115_write( 0x099C, 0xB059 );   
mt9d115_write( 0x099E, 0xCC00 );   
mt9d115_write( 0x098C, 0x04F5 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x28BD );   
mt9d115_write( 0x0992, 0xA558 );   
mt9d115_write( 0x0994, 0x8300 );   
mt9d115_write( 0x0996, 0x0027 );   
mt9d115_write( 0x0998, 0x0BCC );   
mt9d115_write( 0x099A, 0x0026 );   
mt9d115_write( 0x099C, 0x30ED );   
mt9d115_write( 0x099E, 0x00C6 );   
mt9d115_write( 0x098C, 0x0505 );   	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x03BD );   
mt9d115_write( 0x0992, 0xA544 );   
mt9d115_write( 0x0994, 0x3839 );   
mt9d115_write( 0x098C, 0x2006 );   	// MCU_ADDRESS [MON_ARG1]
mt9d115_write( 0x0990, 0x0415 );   	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA005 );   	// MCU_ADDRESS [MON_CMD]
mt9d115_write( 0x0990, 0x0001 );   	// MCU_DATA_0
msleep( 10);



mt9d115_write( 0x3404, 0x0080 ); 


//[95% LSC]
mt9d115_write( 0x3210, 0x01B0 ); 	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x364E, 0x0910 ); 	// P_GR_P0Q0
mt9d115_write( 0x3650, 0x088C ); 	// P_GR_P0Q1
mt9d115_write( 0x3652, 0x7830 ); 	// P_GR_P0Q2
mt9d115_write( 0x3654, 0xC5CB ); 	// P_GR_P0Q3
mt9d115_write( 0x3656, 0x5F91 ); 	// P_GR_P0Q4
mt9d115_write( 0x3658, 0x0190 ); 	// P_RD_P0Q0
mt9d115_write( 0x365A, 0xEBAB ); 	// P_RD_P0Q1
mt9d115_write( 0x365C, 0x4D90 ); 	// P_RD_P0Q2
mt9d115_write( 0x365E, 0x19ED ); 	// P_RD_P0Q3
mt9d115_write( 0x3660, 0x6132 ); 	// P_RD_P0Q4
mt9d115_write( 0x3662, 0x0150 ); 	// P_BL_P0Q0
mt9d115_write( 0x3664, 0x6BAA ); 	// P_BL_P0Q1
mt9d115_write( 0x3666, 0x2A30 ); 	// P_BL_P0Q2
mt9d115_write( 0x3668, 0x976A ); 	// P_BL_P0Q3
mt9d115_write( 0x366A, 0x4C12 ); 	// P_BL_P0Q4
mt9d115_write( 0x366C, 0x00D0 ); 	// P_GB_P0Q0
mt9d115_write( 0x366E, 0x88CB ); 	// P_GB_P0Q1
mt9d115_write( 0x3670, 0x04B1 ); 	// P_GB_P0Q2
mt9d115_write( 0x3672, 0x0C4E ); 	// P_GB_P0Q3
mt9d115_write( 0x3674, 0x3DF1 ); 	// P_GB_P0Q4
mt9d115_write( 0x3676, 0xC2EA ); 	// P_GR_P1Q0
mt9d115_write( 0x3678, 0xAFCE ); 	// P_GR_P1Q1
mt9d115_write( 0x367A, 0x6730 ); 	// P_GR_P1Q2
mt9d115_write( 0x367C, 0x5EF0 ); 	// P_GR_P1Q3
mt9d115_write( 0x367E, 0xBF71 ); 	// P_GR_P1Q4
mt9d115_write( 0x3680, 0x500C ); 	// P_RD_P1Q0
mt9d115_write( 0x3682, 0x484F ); 	// P_RD_P1Q1
mt9d115_write( 0x3684, 0x732F ); 	// P_RD_P1Q2
mt9d115_write( 0x3686, 0xFE31 ); 	// P_RD_P1Q3
mt9d115_write( 0x3688, 0xD4D1 ); 	// P_RD_P1Q4
mt9d115_write( 0x368A, 0xB74B ); 	// P_BL_P1Q0
mt9d115_write( 0x368C, 0xEE4D ); 	// P_BL_P1Q1
mt9d115_write( 0x368E, 0x2150 ); 	// P_BL_P1Q2
mt9d115_write( 0x3690, 0x3790 ); 	// P_BL_P1Q3
mt9d115_write( 0x3692, 0xCFF2 ); 	// P_BL_P1Q4
mt9d115_write( 0x3694, 0x952D ); 	// P_GB_P1Q0
mt9d115_write( 0x3696, 0x5B2F ); 	// P_GB_P1Q1
mt9d115_write( 0x3698, 0x32B0 ); 	// P_GB_P1Q2
mt9d115_write( 0x369A, 0x9AD1 ); 	// P_GB_P1Q3
mt9d115_write( 0x369C, 0xAED1 ); 	// P_GB_P1Q4
mt9d115_write( 0x369E, 0x72B2 ); 	// P_GR_P2Q0
mt9d115_write( 0x36A0, 0xFCD0 ); 	// P_GR_P2Q1
mt9d115_write( 0x36A2, 0xA2D3 ); 	// P_GR_P2Q2
mt9d115_write( 0x36A4, 0x3872 ); 	// P_GR_P2Q3
mt9d115_write( 0x36A6, 0x1056 ); 	// P_GR_P2Q4
mt9d115_write( 0x36A8, 0x51F2 ); 	// P_RD_P2Q0
mt9d115_write( 0x36AA, 0xD8CF ); 	// P_RD_P2Q1
mt9d115_write( 0x36AC, 0xB0D2 ); 	// P_RD_P2Q2
mt9d115_write( 0x36AE, 0x5D91 ); 	// P_RD_P2Q3
mt9d115_write( 0x36B0, 0x3075 ); 	// P_RD_P2Q4
mt9d115_write( 0x36B2, 0x44D2 ); 	// P_BL_P2Q0
mt9d115_write( 0x36B4, 0xA350 ); 	// P_BL_P2Q1
mt9d115_write( 0x36B6, 0xFAD0 ); 	// P_BL_P2Q2
mt9d115_write( 0x36B8, 0x1392 ); 	// P_BL_P2Q3
mt9d115_write( 0x36BA, 0x2D94 ); 	// P_BL_P2Q4
mt9d115_write( 0x36BC, 0x7812 ); 	// P_GB_P2Q0
mt9d115_write( 0x36BE, 0xC0D0 ); 	// P_GB_P2Q1
mt9d115_write( 0x36C0, 0xB7D3 ); 	// P_GB_P2Q2
mt9d115_write( 0x36C2, 0x0D73 ); 	// P_GB_P2Q3
mt9d115_write( 0x36C4, 0x0416 ); 	// P_GB_P2Q4
mt9d115_write( 0x36C6, 0x41F1 ); 	// P_GR_P3Q0
mt9d115_write( 0x36C8, 0xC1B1 ); 	// P_GR_P3Q1
mt9d115_write( 0x36CA, 0x78B4 ); 	// P_GR_P3Q2
mt9d115_write( 0x36CC, 0x3A33 ); 	// P_GR_P3Q3
mt9d115_write( 0x36CE, 0x8F57 ); 	// P_GR_P3Q4
mt9d115_write( 0x36D0, 0x4770 ); 	// P_RD_P3Q0
mt9d115_write( 0x36D2, 0x8EF2 ); 	// P_RD_P3Q1
mt9d115_write( 0x36D4, 0x0E95 ); 	// P_RD_P3Q2
mt9d115_write( 0x36D6, 0x2AB3 ); 	// P_RD_P3Q3
mt9d115_write( 0x36D8, 0xBEB7 ); 	// P_RD_P3Q4
mt9d115_write( 0x36DA, 0x790F ); 	// P_BL_P3Q0
mt9d115_write( 0x36DC, 0xEF10 ); 	// P_BL_P3Q1
mt9d115_write( 0x36DE, 0x62F4 ); 	// P_BL_P3Q2
mt9d115_write( 0x36E0, 0x3512 ); 	// P_BL_P3Q3
mt9d115_write( 0x36E2, 0xE016 ); 	// P_BL_P3Q4
mt9d115_write( 0x36E4, 0x5311 ); 	// P_GB_P3Q0
mt9d115_write( 0x36E6, 0xA9F2 ); 	// P_GB_P3Q1
mt9d115_write( 0x36E8, 0x5494 ); 	// P_GB_P3Q2
mt9d115_write( 0x36EA, 0xD18F ); 	// P_GB_P3Q3
mt9d115_write( 0x36EC, 0xE536 ); 	// P_GB_P3Q4
mt9d115_write( 0x36EE, 0xEF73 ); 	// P_GR_P4Q0
mt9d115_write( 0x36F0, 0xADD2 ); 	// P_GR_P4Q1
mt9d115_write( 0x36F2, 0x0F76 ); 	// P_GR_P4Q2
mt9d115_write( 0x36F4, 0x49D6 ); 	// P_GR_P4Q3
mt9d115_write( 0x36F6, 0xA359 ); 	// P_GR_P4Q4
mt9d115_write( 0x36F8, 0xAB31 ); 	// P_RD_P4Q0
mt9d115_write( 0x36FA, 0xC772 ); 	// P_RD_P4Q1
mt9d115_write( 0x36FC, 0x12B6 ); 	// P_RD_P4Q2
mt9d115_write( 0x36FE, 0x4BB5 ); 	// P_RD_P4Q3
mt9d115_write( 0x3700, 0xBF99 ); 	// P_RD_P4Q4
mt9d115_write( 0x3702, 0xEB50 ); 	// P_BL_P4Q0
mt9d115_write( 0x3704, 0xF911 ); 	// P_BL_P4Q1
mt9d115_write( 0x3706, 0xFC51 ); 	// P_BL_P4Q2
mt9d115_write( 0x3708, 0x78F4 ); 	// P_BL_P4Q3
mt9d115_write( 0x370A, 0xDFF7 ); 	// P_BL_P4Q4
mt9d115_write( 0x370C, 0x81D4 ); 	// P_GB_P4Q0
mt9d115_write( 0x370E, 0xA191 ); 	// P_GB_P4Q1
mt9d115_write( 0x3710, 0x0CD6 ); 	// P_GB_P4Q2
mt9d115_write( 0x3712, 0x6955 ); 	// P_GB_P4Q3
mt9d115_write( 0x3714, 0x96D9 ); 	// P_GB_P4Q4
mt9d115_write( 0x3644, 0x0338 ); 	// POLY_ORIGIN_C
mt9d115_write( 0x3642, 0x0240 ); 	// POLY_ORIGIN_R
mt9d115_write( 0x3210, 0x01B8 ); 	// COLOR_PIPELINE_CONTROL

    // Saturation
mt9d115_write( 0x098C, 0xAB20 ); // MCU_ADDRESS [HG_LL_SAT1]
mt9d115_write( 0x0990, 0x006C ); // MCU_DATA_0


    //AWB
mt9d115_write( 0x098C, 0xA34A ); // MCU_ADDRESS [AWB_GAIN_MIN]
mt9d115_write( 0x0990, 0x0059 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA34B ); // MCU_ADDRESS [AWB_GAIN_MAX]
mt9d115_write( 0x0990, 0x00B6 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA34C ); // MCU_ADDRESS [AWB_GAINMIN_B]
mt9d115_write( 0x0990, 0x0059 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA34D ); // MCU_ADDRESS [AWB_GAINMAX_B]
mt9d115_write( 0x0990, 0x00B5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA351 ); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
mt9d115_write( 0x0990, 0x0000 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA352 ); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
mt9d115_write( 0x0990, 0x007F ); // MCU_DATA_0
    
    // Gamma 0.40  Black 5  C 1.35
mt9d115_write( 0x098C, 0xAB4F ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
mt9d115_write( 0x0990, 0x0000 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB50 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
mt9d115_write( 0x0990, 0x000C ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB51 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
mt9d115_write( 0x0990, 0x0022 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB52 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
mt9d115_write( 0x0990, 0x003F ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB53 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
mt9d115_write( 0x0990, 0x0062 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB54 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
mt9d115_write( 0x0990, 0x007D ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB55 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
mt9d115_write( 0x0990, 0x0093 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB56 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
mt9d115_write( 0x0990, 0x00A5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB57 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
mt9d115_write( 0x0990, 0x00B3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB58 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
mt9d115_write( 0x0990, 0x00BF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB59 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
mt9d115_write( 0x0990, 0x00C9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5A ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
mt9d115_write( 0x0990, 0x00D3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5B ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
mt9d115_write( 0x0990, 0x00DB ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5C ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
mt9d115_write( 0x0990, 0x00E2 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5D ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
mt9d115_write( 0x0990, 0x00E9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5E ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
mt9d115_write( 0x0990, 0x00EF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5F ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
mt9d115_write( 0x0990, 0x00F5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB60 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
mt9d115_write( 0x0990, 0x00FA ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB61 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
mt9d115_write( 0x0990, 0x00FF ); // MCU_DATA_0
    
mt9d115_write( 0x098C, 0xAB3C ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_0]
mt9d115_write( 0x0990, 0x0000 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB3D ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_1]
mt9d115_write( 0x0990, 0x000C ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB3E ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_2]
mt9d115_write( 0x0990, 0x0022 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB3F ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_3]
mt9d115_write( 0x0990, 0x003F ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB40 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_4]
mt9d115_write( 0x0990, 0x0062 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB41 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_5]
mt9d115_write( 0x0990, 0x007D ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB42 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_6]
mt9d115_write( 0x0990, 0x0093 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB43 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_7]
mt9d115_write( 0x0990, 0x00A5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB44 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_8]
mt9d115_write( 0x0990, 0x00B3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB45 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_9]
mt9d115_write( 0x0990, 0x00BF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB46 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_10]
mt9d115_write( 0x0990, 0x00C9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB47 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_11]
mt9d115_write( 0x0990, 0x00D3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB48 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_12]
mt9d115_write( 0x0990, 0x00DB ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB49 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_13]
mt9d115_write( 0x0990, 0x00E2 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4A ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_14]
mt9d115_write( 0x0990, 0x00E9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4B ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_15]
mt9d115_write( 0x0990, 0x00EF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4C ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_16]
mt9d115_write( 0x0990, 0x00F5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4D ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_17]
mt9d115_write( 0x0990, 0x00FA ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4E ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_18]
mt9d115_write( 0x0990, 0x00FF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2C );              // MCU_ADDRESS [HG_NR_START_R]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2D );              // MCU_ADDRESS [HG_NR_START_G]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2E );              // MCU_ADDRESS [HG_NR_START_B]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2F );              // MCU_ADDRESS [HG_NR_START_OL]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
//Add 2012-5-2 16:29
mt9d115_write( 0x098C, 0xA366 ); 	// MCU_ADDRESS [AWB_KR_L]
mt9d115_write( 0x0990, 0x0078 ); 	// MCU_DATA_0

mt9d115_write( 0x098C, 0x2306 ); 	// MCU_ADDRESS [AWB_CCM_L_0]
mt9d115_write( 0x0990, 0x01B1 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231C ); 	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9d115_write( 0x0990, 0x0028 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2308 ); 	// MCU_ADDRESS [AWB_CCM_L_1]
mt9d115_write( 0x0990, 0xFFA2 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231E ); 	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9d115_write( 0x0990, 0xFFE5 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230A ); 	// MCU_ADDRESS [AWB_CCM_L_2]
mt9d115_write( 0x0990, 0xFFAD ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2320 ); 	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9d115_write( 0x0990, 0xFFF3 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230C ); 	// MCU_ADDRESS [AWB_CCM_L_3]
mt9d115_write( 0x0990, 0xFF87 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2322 ); 	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9d115_write( 0x0990, 0xFFF6 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230E ); 	// MCU_ADDRESS [AWB_CCM_L_4]
mt9d115_write( 0x0990, 0x0184 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2324 ); 	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9d115_write( 0x0990, 0x0026 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2310 ); 	// MCU_ADDRESS [AWB_CCM_L_5]
mt9d115_write( 0x0990, 0xFFF5 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2326 ); 	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9d115_write( 0x0990, 0xFFE4 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2312 ); 	// MCU_ADDRESS [AWB_CCM_L_6]
mt9d115_write( 0x0990, 0xFFBF ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2328 ); 	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9d115_write( 0x0990, 0x001C ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2314 ); 	// MCU_ADDRESS [AWB_CCM_L_7]
mt9d115_write( 0x0990, 0xFF4F ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232A ); 	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9d115_write( 0x0990, 0x0011 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2316 ); 	// MCU_ADDRESS [AWB_CCM_L_8]
mt9d115_write( 0x0990, 0x01F3 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232C ); 	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9d115_write( 0x0990, 0xFFD3 ); 	// MCU_DATA_0


//

mt9d115_write( 0x098C, 0xAB30);  // MCU_ADDRESS [HG_NR_STOP_R]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024
mt9d115_write( 0x098C, 0xAB31);  // MCU_ADDRESS [HG_NR_STOP_G]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024
mt9d115_write( 0x098C, 0xAB32);  // MCU_ADDRESS [HG_NR_STOP_B]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024
mt9d115_write( 0x098C, 0xAB33);  // MCU_ADDRESS [HG_NR_STOP_OL]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024 
mt9d115_write( 0x098C, 0xAB34);  // MCU_ADDRESS [HG_NR_GAINSTART] 
mt9d115_write( 0x0990, 0x000E);    // MCU_DATA_0 
mt9d115_write( 0x098C, 0xAB35);  // MCU_ADDRESS [HG_NR_GAINSTOP]
mt9d115_write( 0x0990, 0x0091);    // MCU_DATA_0

mt9d115_write( 0x0018, 0x0028 ); 	// STANDBY_CONTROL // 初始化中加这个位置加上这一句

mt9d115_write( 0x33F4, 0x005B );    // MCU_DATA_0 //default 4b

//  把50Hz的参数，加在这个位置！


///flicker


mt9d115_write( 0x3404, 0x0080 );



///flicker

//[50HZ]
mt9d115_write(0x098C, 0xA118 );	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]        
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                 
mt9d115_write(0x098C, 0xA11E );	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]             
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                 
mt9d115_write(0x098C, 0xA124 );	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]             
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                     
mt9d115_write(0x098C, 0xA12A );	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]            
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                  
mt9d115_write(0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]                       
mt9d115_write(0x0990, 0x00E0 );	// MCU_DATA_0                               
mt9d115_write(0x098C, 0xA103 );         
mt9d115_write(0x0990, 0x0005 );
msleep(10);		//modify by zhaoqiang
mt9d115_write( 0x3404, 0x0080 );

//[ANTIBANDING_SET_AUTO]
mt9d115_write(0x098C, 0xA118 );	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]          
mt9d115_write(0x0990, 0x0001 );	// MCU_DATA_0                              
mt9d115_write(0x098C, 0xA11E );	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]         
mt9d115_write(0x0990, 0x0001 );	// MCU_DATA_0                            
mt9d115_write(0x098C, 0xA124 );	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]         
mt9d115_write(0x0990, 0x0000 );	// MCU_DATA_0                                  
mt9d115_write(0x098C, 0xA12A );	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]          
mt9d115_write(0x0990, 0x0001 );	// MCU_DATA_0  
mt9d115_write(0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]                      
mt9d115_write(0x0990, 0x0060 );	// MCU_DATA_0

mt9d115_write( 0x098C, 0xA215 );	// MCU_ADDRESS [AE_INDEX_TH23]
mt9d115_write( 0x0990, 0x0004);//c );	// MCU_DATA_0	//0x04	 //modify by zhaoqiang chg for zte objective test recording slow 20120628
mt9d115_write( 0x098C, 0xA20C );	// MCU_ADDRESS [AE_MAX_INDEX]
mt9d115_write( 0x0990, 0x0006);//12 );	// MCU_DATA_0 04			 //modify by zhaoqiang chg for zte objective test recording slow 20120628

mt9d115_write( 0x3404, 0x0080 );

                                
//mt9d115_write(0x098C, 0xA103 );	                  
//mt9d115_write(0x0990, 0x0005 );	        
//  POLL  MON_PATCH_ID_0 =>  0x01
mt9d115_write( 0x0018, 0x0028 );	// STANDBY_CONTROL
msleep( 10);

/*order YUV for preview yuv422*/
mt9d115_write( 0x098C, 0x2755 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0002 );	// MCU_ADDRESS [SEQ_CMD]

/* order YUV for capture yuv422 */
mt9d115_write( 0x098C, 0x2757 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0002 );	// MCU_ADDRESS [SEQ_CMD]

/* flip */
/*
mt9d115_write( 0x098C, 0x2717 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x046E );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x098C, 0x272D );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0026 );	// MCU_ADDRESS [SEQ_CMD]
*/
mt9d115_write(0x098C, 0x272D);	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_B]
mt9d115_write(0x0990, 0x0027); 	// MCU_DATA_0

mt9d115_write(0x098C, 0x2717); 	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_A]
mt9d115_write(0x0990, 0x046f); 	// MCU_DATA_0
//  POLL  SEQ_STATE =>  0x03
mt9d115_write( 0x098C, 0xA103 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
	
msleep(200);			//modify by zhaoqiang
mt9d115_write( 0x098C, 0xA103 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0005 );	// MCU_DATA_0
//msleep(200);	//modify by zhaoqiang

//	mt9d115_write( 0x3404, 0x0080 );
//	mt9d115_write( 0x3404, 0x0080 );


	/*------- le you --------*/

	/* RGB565 */
	#if 0
	printk(KERN_ERR"----now RGB565-A   ----  \n");
	mt9d115_write( 0x098C, 0x2755 ); //REG= 0x098C, 0x2755   // MCU_ADDRESS [MODE_OUTPUT_FORMAT_A]
	mt9d115_write( 0x0990, 0x0020 ); //REG= 0x0990, 0x0020   // MCU_DATA_0
	mt9d115_write( 0x098C, 0xA103 ); //REG= 0x098C, 0xA103 // MCU_ADDRESS [SEQ_CMD]
	mt9d115_write( 0x0990, 0x0005 ); //REG= 0x0990, 0x0005   // MCU_DATA_0
	#endif 
	/*-----------end--------*/
	
	/*hanwei del-s just for debug code 20120329*/	
	#if 0
			//mt9d115_write( 0x3070, 0x0100 );	// MCU_DATA_0  leyou:  a single patten
			
			msleep(100);	//modify by zhaoqiang
	
			mt9d115_write( 0x098C, 0xA104 );
			msleep(20);	//modify by zhaoqiang
			printk(KERN_ERR"0x0990 read out : %d \n",mt9d115_read(0x0990));  /*expected to be 3 */
			
			printk(KERN_ERR"0x3400:read out :%d \n",mt9d115_read(0x3400));  /*0x3400*/
	#endif	
	/*hanwei del-e just for debug code 20120329 */	
				
			if (checkCameraID(sensor)) {
			result = HAL_CAM_ERROR_INTERNAL_ERROR;
		}
		tm2 = ktime_get();
		//pr_info("Exit Init Sec %d nsec %d\n",  tm2.tv.sec-tm1.tv.sec, tm2.tv.nsec-tm1.tv.nsec);
	printk("\n-----------Init_MT9D115-------End------\r\n");
		return result;
	}

static HAL_CAM_Result_en_t Init_MT9D115_floatFPS(CamSensorSelect_t sensor)
{
		HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
		static ktime_t tm1,tm2;
		tm1 = ktime_get();
		
		pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);
		
		CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;
		printk("Init Primary Sensor MT9D115: \r\n");
		
		/*	
		   * Preview YUV 640x480  Frame rate 10~ 27.5fps
			  * Capture YUV 2048x1536	Frame Rate 8.45
			  * XMCLK 26MHz PCLK 64MHz
		   */
       #if defined (CONFIG_CAM_CSI2)
		   printk(KERN_ERR"Init_MT9D115 CONFIG_CAM_CSI2 has been defined\n ");
	#endif 
		
		   checkCameraID(sensor);
		//reset MCLK=24M . PCLK=240;

		
msleep(100);	//Delay=100

mt9d115_write( 0x001A, 0x0051 );	// RESET_AND_MISC_CONTROL
msleep(2);												//DELAY=2
mt9d115_write( 0x001A, 0x0050 );	// RESET_AND_MISC_CONTROL
mt9d115_write( 0x001A, 0x0058 );	// RESET_AND_MISC_CONTROL
msleep(10);												//DELAY=10
mt9d115_write( 0x0014, 0x21F9 );	// PLL_CONTROL
mt9d115_write( 0x0010, 0x0115 );	// PLL_DIVIDERS
mt9d115_write( 0x0012, 0x00F5 );	// PLL_P_DIVIDERS
mt9d115_write( 0x0014, 0x2545 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2547 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2447 );	// PLL_CONTROL
msleep(10);												//DELAY=10                      );
mt9d115_write( 0x0014, 0x2047 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2046 );	// PLL_CONTROL
mt9d115_write( 0x0018, 0x402D );	// STANDBY_CONTROL
mt9d115_write( 0x0018, 0x402C );	// STANDBY_CONTROL
//  POLL  STANDBY_CONTROL::STANDBY_DONE =>  0x01, 0x00
msleep(50);	//DELAY=50

//800x600
//mt9d115_write( 0x098C, 0x2703 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
//mt9d115_write( 0x0990, 0x0320 );	// MCU_DATA_0
//mt9d115_write( 0x098C, 0x2705 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
//mt9d115_write( 0x0990, 0x0258 );	// MCU_DATA_0

//640x480
mt9d115_write( 0x098C, 0x2703 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
mt9d115_write( 0x0990, 0x0280 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2705 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
mt9d115_write( 0x0990, 0x01E0 );	// MCU_DATA_0

mt9d115_write( 0x098C, 0x2707 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_B]
mt9d115_write( 0x0990, 0x0640 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2709 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_B]
mt9d115_write( 0x0990, 0x04B0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x270D );	// MCU_ADDRESS [MODE_SENSOR_ROW_START_A]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x270F );	// MCU_ADDRESS [MODE_SENSOR_COL_START_A]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2711 );	// MCU_ADDRESS [MODE_SENSOR_ROW_END_A]
mt9d115_write( 0x0990, 0x04BD );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2713 );	// MCU_ADDRESS [MODE_SENSOR_COL_END_A]
mt9d115_write( 0x0990, 0x064D );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2715 );	// MCU_ADDRESS [MODE_SENSOR_ROW_SPEED_A]
mt9d115_write( 0x0990, 0x0111 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2717 );	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_A]
mt9d115_write( 0x0990, 0x046C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2719 );	// MCU_ADDRESS [MODE_SENSOR_FINE_CORRECTION_A]
mt9d115_write( 0x0990, 0x005A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x271B );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MIN_A]
mt9d115_write( 0x0990, 0x01BE );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x271D );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MAX_MARGIN_A]
mt9d115_write( 0x0990, 0x0131 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x271F );	// MCU_ADDRESS [MODE_SENSOR_FRAME_LENGTH_A]
mt9d115_write( 0x0990, 0x02BB );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2721 );	// MCU_ADDRESS [MODE_SENSOR_LINE_LENGTH_PCK_A]
mt9d115_write( 0x0990, 0x0888 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2723 );	// MCU_ADDRESS [MODE_SENSOR_ROW_START_B]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2725 );	// MCU_ADDRESS [MODE_SENSOR_COL_START_B]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2727 );	// MCU_ADDRESS [MODE_SENSOR_ROW_END_B]
mt9d115_write( 0x0990, 0x04BB );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2729 );	// MCU_ADDRESS [MODE_SENSOR_COL_END_B]
mt9d115_write( 0x0990, 0x064B );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x272B );	// MCU_ADDRESS [MODE_SENSOR_ROW_SPEED_B]
mt9d115_write( 0x0990, 0x0111 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x272D );	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_B]
mt9d115_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x272F );	// MCU_ADDRESS [MODE_SENSOR_FINE_CORRECTION_B]
mt9d115_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2731 );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MIN_B]
mt9d115_write( 0x0990, 0x00F6 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2733 );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MAX_MARGIN_B]
mt9d115_write( 0x0990, 0x008B );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2735 );	// MCU_ADDRESS [MODE_SENSOR_FRAME_LENGTH_B]
mt9d115_write( 0x0990, 0x0521 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2737 );	// MCU_ADDRESS [MODE_SENSOR_LINE_LENGTH_PCK_B]
mt9d115_write( 0x0990, 0x0888 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2739 );	// MCU_ADDRESS [MODE_CROP_X0_A]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x273B );	// MCU_ADDRESS [MODE_CROP_X1_A]
mt9d115_write( 0x0990, 0x031F );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x273D );	// MCU_ADDRESS [MODE_CROP_Y0_A]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x273F );	// MCU_ADDRESS [MODE_CROP_Y1_A]
mt9d115_write( 0x0990, 0x0257 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2747 );	// MCU_ADDRESS [MODE_CROP_X0_B]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2749 );	// MCU_ADDRESS [MODE_CROP_X1_B]
mt9d115_write( 0x0990, 0x063F );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x274B );	// MCU_ADDRESS [MODE_CROP_Y0_B]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x274D );	// MCU_ADDRESS [MODE_CROP_Y1_B]
mt9d115_write( 0x0990, 0x04AF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2222 );	// MCU_ADDRESS [AE_R9]
mt9d115_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA408 );	// MCU_ADDRESS [FD_SEARCH_F1_50]
mt9d115_write( 0x0990, 0x0026 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA409 );	// MCU_ADDRESS [FD_SEARCH_F2_50]
mt9d115_write( 0x0990, 0x0029 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40A );	// MCU_ADDRESS [FD_SEARCH_F1_60]
mt9d115_write( 0x0990, 0x002E );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40B );	// MCU_ADDRESS [FD_SEARCH_F2_60]
mt9d115_write( 0x0990, 0x0031 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2411 );	// MCU_ADDRESS [FD_R9_STEP_F60_A]
mt9d115_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2413 );	// MCU_ADDRESS [FD_R9_STEP_F50_A]
mt9d115_write( 0x0990, 0x00C0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2415 );	// MCU_ADDRESS [FD_R9_STEP_F60_B]
mt9d115_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2417 );	// MCU_ADDRESS [FD_R9_STEP_F50_B]
mt9d115_write( 0x0990, 0x00C0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]
mt9d115_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40D );	// MCU_ADDRESS [FD_STAT_MIN]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40E );	// MCU_ADDRESS [FD_STAT_MAX]
mt9d115_write( 0x0990, 0x0003 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA410 );	// MCU_ADDRESS [FD_MIN_AMPLITUDE]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA117 );	// MCU_ADDRESS [SEQ_PREVIEW_0_AE]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA11D );	// MCU_ADDRESS [SEQ_PREVIEW_1_AE]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA129 );	// MCU_ADDRESS [SEQ_PREVIEW_3_AE]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA24F );	// MCU_ADDRESS [AE_BASETARGET]
mt9d115_write( 0x0990, 0x0032 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA20C );	// MCU_ADDRESS [AE_MAX_INDEX]
mt9d115_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA216 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA20E );	// MCU_ADDRESS [AE_MAX_VIRTGAIN]
mt9d115_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2212 );	// MCU_ADDRESS [AE_MAX_DGAIN_AE1]
mt9d115_write( 0x0990, 0x00A4 );	// MCU_DATA_0
mt9d115_write( 0x3210, 0x01B8 );	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x098C, 0xAB36 );	// MCU_ADDRESS [HG_CLUSTERDC_TH]
mt9d115_write( 0x0990, 0x0014 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B66 );	// MCU_ADDRESS [HG_CLUSTER_DC_BM]
mt9d115_write( 0x0990, 0x2AF8 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB20 );	// MCU_ADDRESS [HG_LL_SAT1]
mt9d115_write( 0x0990, 0x0080 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB24 );	// MCU_ADDRESS [HG_LL_SAT2]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB21 );	// MCU_ADDRESS [HG_LL_INTERPTHRESH1]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB25 );	// MCU_ADDRESS [HG_LL_INTERPTHRESH2]
mt9d115_write( 0x0990, 0x002A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB22 );	// MCU_ADDRESS [HG_LL_APCORR1]
mt9d115_write( 0x0990, 0x0007 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB26 );	// MCU_ADDRESS [HG_LL_APCORR2]
mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB23 );	// MCU_ADDRESS [HG_LL_APTHRESH1]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB27 );	// MCU_ADDRESS [HG_LL_APTHRESH2]
mt9d115_write( 0x0990, 0x0009 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B28 );	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]
mt9d115_write( 0x0990, 0x0BB8 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B2A );	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]
mt9d115_write( 0x0990, 0x2968 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2C );	// MCU_ADDRESS [HG_NR_START_R]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB30 );	// MCU_ADDRESS [HG_NR_STOP_R]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2D );	// MCU_ADDRESS [HG_NR_START_G]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB31 );	// MCU_ADDRESS [HG_NR_STOP_G]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2E );	// MCU_ADDRESS [HG_NR_START_B]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB32 );	// MCU_ADDRESS [HG_NR_STOP_B]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2F );	// MCU_ADDRESS [HG_NR_START_OL]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB33 );	// MCU_ADDRESS [HG_NR_STOP_OL]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB34 );	// MCU_ADDRESS [HG_NR_GAINSTART]
mt9d115_write( 0x0990, 0x0020 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB35 );	// MCU_ADDRESS [HG_NR_GAINSTOP]
mt9d115_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA765 );	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_FILTER_MODE]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB37 );	// MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]
mt9d115_write( 0x0990, 0x0003 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B38 );	// MCU_ADDRESS [HG_GAMMASTARTMORPH]
mt9d115_write( 0x0990, 0x2968 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B3A );	// MCU_ADDRESS [HG_GAMMASTOPMORPH]
mt9d115_write( 0x0990, 0x2D50 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B62 );	// MCU_ADDRESS [HG_FTB_START_BM]
mt9d115_write( 0x0990, 0xFFFE );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B64 );	// MCU_ADDRESS [HG_FTB_STOP_BM]
mt9d115_write( 0x0990, 0xFFFF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4F );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB50 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
mt9d115_write( 0x0990, 0x0013 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB51 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
mt9d115_write( 0x0990, 0x0027 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB52 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
mt9d115_write( 0x0990, 0x0043 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB53 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
mt9d115_write( 0x0990, 0x0068 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB54 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
mt9d115_write( 0x0990, 0x0081 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB55 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
mt9d115_write( 0x0990, 0x0093 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB56 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
mt9d115_write( 0x0990, 0x00A3 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB57 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
mt9d115_write( 0x0990, 0x00B0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB58 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
mt9d115_write( 0x0990, 0x00BC );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB59 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
mt9d115_write( 0x0990, 0x00C7 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5A );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
mt9d115_write( 0x0990, 0x00D1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5B );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
mt9d115_write( 0x0990, 0x00DA );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5C );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
mt9d115_write( 0x0990, 0x00E2 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5D );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
mt9d115_write( 0x0990, 0x00E9 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5E );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
mt9d115_write( 0x0990, 0x00EF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5F );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
mt9d115_write( 0x0990, 0x00F4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB60 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
mt9d115_write( 0x0990, 0x00FA );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB61 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2306 );	// MCU_ADDRESS [AWB_CCM_L_0]
mt9d115_write( 0x0990, 0x01D6 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2308 );	// MCU_ADDRESS [AWB_CCM_L_1]
mt9d115_write( 0x0990, 0xFF89 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230A );	// MCU_ADDRESS [AWB_CCM_L_2]
mt9d115_write( 0x0990, 0xFFA1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230C );	// MCU_ADDRESS [AWB_CCM_L_3]
mt9d115_write( 0x0990, 0xFF73 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230E );	// MCU_ADDRESS [AWB_CCM_L_4]
mt9d115_write( 0x0990, 0x019C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2310 );	// MCU_ADDRESS [AWB_CCM_L_5]
mt9d115_write( 0x0990, 0xFFF1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2312 );	// MCU_ADDRESS [AWB_CCM_L_6]
mt9d115_write( 0x0990, 0xFFB0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2314 );	// MCU_ADDRESS [AWB_CCM_L_7]
mt9d115_write( 0x0990, 0xFF2D );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2316 );	// MCU_ADDRESS [AWB_CCM_L_8]
mt9d115_write( 0x0990, 0x0223 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0048 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001E );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x0022 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x002C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231C );	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9d115_write( 0x0990, 0xFFCD );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231E );	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9d115_write( 0x0990, 0x0023 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2320 );	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9d115_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2322 );	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9d115_write( 0x0990, 0x0026 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2324 );	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9d115_write( 0x0990, 0xFFE9 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2326 );	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9d115_write( 0x0990, 0xFFF1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2328 );	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9d115_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232A );	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9d115_write( 0x0990, 0x005D );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232C );	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9d115_write( 0x0990, 0xFF69 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFE4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0xFFFC );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x0415 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xF601 );
mt9d115_write( 0x0992, 0x42C1 );
mt9d115_write( 0x0994, 0x0326 );
mt9d115_write( 0x0996, 0x11F6 );
mt9d115_write( 0x0998, 0x0143 );
mt9d115_write( 0x099A, 0xC104 );
mt9d115_write( 0x099C, 0x260A );
mt9d115_write( 0x099E, 0xCC04 );
mt9d115_write( 0x098C, 0x0425 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x33BD );
mt9d115_write( 0x0992, 0xA362 );
mt9d115_write( 0x0994, 0xBD04 );
mt9d115_write( 0x0996, 0x3339 );
mt9d115_write( 0x0998, 0xC6FF );
mt9d115_write( 0x099A, 0xF701 );
mt9d115_write( 0x099C, 0x6439 );
mt9d115_write( 0x099E, 0xFE01 );
mt9d115_write( 0x098C, 0x0435 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x6918 );
mt9d115_write( 0x0992, 0xCE03 );
mt9d115_write( 0x0994, 0x25CC );
mt9d115_write( 0x0996, 0x0013 );
mt9d115_write( 0x0998, 0xBDC2 );
mt9d115_write( 0x099A, 0xB8CC );
mt9d115_write( 0x099C, 0x0489 );
mt9d115_write( 0x099E, 0xFD03 );
mt9d115_write( 0x098C, 0x0445 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x27CC );
mt9d115_write( 0x0992, 0x0325 );
mt9d115_write( 0x0994, 0xFD01 );
mt9d115_write( 0x0996, 0x69FE );
mt9d115_write( 0x0998, 0x02BD );
mt9d115_write( 0x099A, 0x18CE );
mt9d115_write( 0x099C, 0x0339 );
mt9d115_write( 0x099E, 0xCC00 );
mt9d115_write( 0x098C, 0x0455 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x11BD );
mt9d115_write( 0x0992, 0xC2B8 );
mt9d115_write( 0x0994, 0xCC04 );
mt9d115_write( 0x0996, 0xC8FD );
mt9d115_write( 0x0998, 0x0347 );
mt9d115_write( 0x099A, 0xCC03 );
mt9d115_write( 0x099C, 0x39FD );
mt9d115_write( 0x099E, 0x02BD );
mt9d115_write( 0x098C, 0x0465 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xDE00 );
mt9d115_write( 0x0992, 0x18CE );
mt9d115_write( 0x0994, 0x00C2 );
mt9d115_write( 0x0996, 0xCC00 );
mt9d115_write( 0x0998, 0x37BD );
mt9d115_write( 0x099A, 0xC2B8 );
mt9d115_write( 0x099C, 0xCC04 );
mt9d115_write( 0x099E, 0xEFDD );
mt9d115_write( 0x098C, 0x0475 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xE6CC );
mt9d115_write( 0x0992, 0x00C2 );
mt9d115_write( 0x0994, 0xDD00 );
mt9d115_write( 0x0996, 0xC601 );
mt9d115_write( 0x0998, 0xF701 );
mt9d115_write( 0x099A, 0x64C6 );
mt9d115_write( 0x099C, 0x03F7 );
mt9d115_write( 0x099E, 0x0165 );
mt9d115_write( 0x098C, 0x0485 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x7F01 );
mt9d115_write( 0x0992, 0x6639 );
mt9d115_write( 0x0994, 0x3C3C );
mt9d115_write( 0x0996, 0x3C34 );
mt9d115_write( 0x0998, 0xCC32 );
mt9d115_write( 0x099A, 0x3EBD );
mt9d115_write( 0x099C, 0xA558 );
mt9d115_write( 0x099E, 0x30ED );
mt9d115_write( 0x098C, 0x0495 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x04BD );
mt9d115_write( 0x0992, 0xB2D7 );
mt9d115_write( 0x0994, 0x30E7 );
mt9d115_write( 0x0996, 0x06CC );
mt9d115_write( 0x0998, 0x323E );
mt9d115_write( 0x099A, 0xED00 );
mt9d115_write( 0x099C, 0xEC04 );
mt9d115_write( 0x099E, 0xBDA5 );
mt9d115_write( 0x098C, 0x04A5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x44CC );
mt9d115_write( 0x0992, 0x3244 );
mt9d115_write( 0x0994, 0xBDA5 );
mt9d115_write( 0x0996, 0x585F );
mt9d115_write( 0x0998, 0x30ED );
mt9d115_write( 0x099A, 0x02CC );
mt9d115_write( 0x099C, 0x3244 );
mt9d115_write( 0x099E, 0xED00 );
mt9d115_write( 0x098C, 0x04B5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xF601 );
mt9d115_write( 0x0992, 0xD54F );
mt9d115_write( 0x0994, 0xEA03 );
mt9d115_write( 0x0996, 0xAA02 );
mt9d115_write( 0x0998, 0xBDA5 );
mt9d115_write( 0x099A, 0x4430 );
mt9d115_write( 0x099C, 0xE606 );
mt9d115_write( 0x099E, 0x3838 );
mt9d115_write( 0x098C, 0x04C5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x3831 );
mt9d115_write( 0x0992, 0x39BD );
mt9d115_write( 0x0994, 0xD661 );
mt9d115_write( 0x0996, 0xF602 );
mt9d115_write( 0x0998, 0xF4C1 );
mt9d115_write( 0x099A, 0x0126 );
mt9d115_write( 0x099C, 0x0BFE );
mt9d115_write( 0x099E, 0x02BD );
mt9d115_write( 0x098C, 0x04D5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xEE10 );
mt9d115_write( 0x0992, 0xFC02 );
mt9d115_write( 0x0994, 0xF5AD );
mt9d115_write( 0x0996, 0x0039 );
mt9d115_write( 0x0998, 0xF602 );
mt9d115_write( 0x099A, 0xF4C1 );
mt9d115_write( 0x099C, 0x0226 );
mt9d115_write( 0x099E, 0x0AFE );
mt9d115_write( 0x098C, 0x04E5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x02BD );
mt9d115_write( 0x0992, 0xEE10 );
mt9d115_write( 0x0994, 0xFC02 );
mt9d115_write( 0x0996, 0xF7AD );
mt9d115_write( 0x0998, 0x0039 );
mt9d115_write( 0x099A, 0x3CBD );
mt9d115_write( 0x099C, 0xB059 );
mt9d115_write( 0x099E, 0xCC00 );
mt9d115_write( 0x098C, 0x04F5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x28BD );
mt9d115_write( 0x0992, 0xA558 );
mt9d115_write( 0x0994, 0x8300 );
mt9d115_write( 0x0996, 0x0027 );
mt9d115_write( 0x0998, 0x0BCC );
mt9d115_write( 0x099A, 0x0026 );
mt9d115_write( 0x099C, 0x30ED );
mt9d115_write( 0x099E, 0x00C6 );
mt9d115_write( 0x098C, 0x0505 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x03BD );
mt9d115_write( 0x0992, 0xA544 );
mt9d115_write( 0x0994, 0x3839 );
mt9d115_write( 0x098C, 0x2006 );	// MCU_ADDRESS [MON_ARG1]
mt9d115_write( 0x0990, 0x0415 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA005 );	// MCU_ADDRESS [MON_CMD]
mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0
msleep(10);	//DELAY= 10

//[95% LSC]
mt9d115_write( 0x3210, 0x01B0 ); 	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x364E, 0x0910 ); 	// P_GR_P0Q0
mt9d115_write( 0x3650, 0x088C ); 	// P_GR_P0Q1
mt9d115_write( 0x3652, 0x7830 ); 	// P_GR_P0Q2
mt9d115_write( 0x3654, 0xC5CB ); 	// P_GR_P0Q3
mt9d115_write( 0x3656, 0x5F91 ); 	// P_GR_P0Q4
mt9d115_write( 0x3658, 0x0190 ); 	// P_RD_P0Q0
mt9d115_write( 0x365A, 0xEBAB ); 	// P_RD_P0Q1
mt9d115_write( 0x365C, 0x4D90 ); 	// P_RD_P0Q2
mt9d115_write( 0x365E, 0x19ED ); 	// P_RD_P0Q3
mt9d115_write( 0x3660, 0x6132 ); 	// P_RD_P0Q4
mt9d115_write( 0x3662, 0x0150 ); 	// P_BL_P0Q0
mt9d115_write( 0x3664, 0x6BAA ); 	// P_BL_P0Q1
mt9d115_write( 0x3666, 0x2A30 ); 	// P_BL_P0Q2
mt9d115_write( 0x3668, 0x976A ); 	// P_BL_P0Q3
mt9d115_write( 0x366A, 0x4C12 ); 	// P_BL_P0Q4
mt9d115_write( 0x366C, 0x00D0 ); 	// P_GB_P0Q0
mt9d115_write( 0x366E, 0x88CB ); 	// P_GB_P0Q1
mt9d115_write( 0x3670, 0x04B1 ); 	// P_GB_P0Q2
mt9d115_write( 0x3672, 0x0C4E ); 	// P_GB_P0Q3
mt9d115_write( 0x3674, 0x3DF1 ); 	// P_GB_P0Q4
mt9d115_write( 0x3676, 0xC2EA ); 	// P_GR_P1Q0
mt9d115_write( 0x3678, 0xAFCE ); 	// P_GR_P1Q1
mt9d115_write( 0x367A, 0x6730 ); 	// P_GR_P1Q2
mt9d115_write( 0x367C, 0x5EF0 ); 	// P_GR_P1Q3
mt9d115_write( 0x367E, 0xBF71 ); 	// P_GR_P1Q4
mt9d115_write( 0x3680, 0x500C ); 	// P_RD_P1Q0
mt9d115_write( 0x3682, 0x484F ); 	// P_RD_P1Q1
mt9d115_write( 0x3684, 0x732F ); 	// P_RD_P1Q2
mt9d115_write( 0x3686, 0xFE31 ); 	// P_RD_P1Q3
mt9d115_write( 0x3688, 0xD4D1 ); 	// P_RD_P1Q4
mt9d115_write( 0x368A, 0xB74B ); 	// P_BL_P1Q0
mt9d115_write( 0x368C, 0xEE4D ); 	// P_BL_P1Q1
mt9d115_write( 0x368E, 0x2150 ); 	// P_BL_P1Q2
mt9d115_write( 0x3690, 0x3790 ); 	// P_BL_P1Q3
mt9d115_write( 0x3692, 0xCFF2 ); 	// P_BL_P1Q4
mt9d115_write( 0x3694, 0x952D ); 	// P_GB_P1Q0
mt9d115_write( 0x3696, 0x5B2F ); 	// P_GB_P1Q1
mt9d115_write( 0x3698, 0x32B0 ); 	// P_GB_P1Q2
mt9d115_write( 0x369A, 0x9AD1 ); 	// P_GB_P1Q3
mt9d115_write( 0x369C, 0xAED1 ); 	// P_GB_P1Q4
mt9d115_write( 0x369E, 0x72B2 ); 	// P_GR_P2Q0
mt9d115_write( 0x36A0, 0xFCD0 ); 	// P_GR_P2Q1
mt9d115_write( 0x36A2, 0xA2D3 ); 	// P_GR_P2Q2
mt9d115_write( 0x36A4, 0x3872 ); 	// P_GR_P2Q3
mt9d115_write( 0x36A6, 0x1056 ); 	// P_GR_P2Q4
mt9d115_write( 0x36A8, 0x51F2 ); 	// P_RD_P2Q0
mt9d115_write( 0x36AA, 0xD8CF ); 	// P_RD_P2Q1
mt9d115_write( 0x36AC, 0xB0D2 ); 	// P_RD_P2Q2
mt9d115_write( 0x36AE, 0x5D91 ); 	// P_RD_P2Q3
mt9d115_write( 0x36B0, 0x3075 ); 	// P_RD_P2Q4
mt9d115_write( 0x36B2, 0x44D2 ); 	// P_BL_P2Q0
mt9d115_write( 0x36B4, 0xA350 ); 	// P_BL_P2Q1
mt9d115_write( 0x36B6, 0xFAD0 ); 	// P_BL_P2Q2
mt9d115_write( 0x36B8, 0x1392 ); 	// P_BL_P2Q3
mt9d115_write( 0x36BA, 0x2D94 ); 	// P_BL_P2Q4
mt9d115_write( 0x36BC, 0x7812 ); 	// P_GB_P2Q0
mt9d115_write( 0x36BE, 0xC0D0 ); 	// P_GB_P2Q1
mt9d115_write( 0x36C0, 0xB7D3 ); 	// P_GB_P2Q2
mt9d115_write( 0x36C2, 0x0D73 ); 	// P_GB_P2Q3
mt9d115_write( 0x36C4, 0x0416 ); 	// P_GB_P2Q4
mt9d115_write( 0x36C6, 0x41F1 ); 	// P_GR_P3Q0
mt9d115_write( 0x36C8, 0xC1B1 ); 	// P_GR_P3Q1
mt9d115_write( 0x36CA, 0x78B4 ); 	// P_GR_P3Q2
mt9d115_write( 0x36CC, 0x3A33 ); 	// P_GR_P3Q3
mt9d115_write( 0x36CE, 0x8F57 ); 	// P_GR_P3Q4
mt9d115_write( 0x36D0, 0x4770 ); 	// P_RD_P3Q0
mt9d115_write( 0x36D2, 0x8EF2 ); 	// P_RD_P3Q1
mt9d115_write( 0x36D4, 0x0E95 ); 	// P_RD_P3Q2
mt9d115_write( 0x36D6, 0x2AB3 ); 	// P_RD_P3Q3
mt9d115_write( 0x36D8, 0xBEB7 ); 	// P_RD_P3Q4
mt9d115_write( 0x36DA, 0x790F ); 	// P_BL_P3Q0
mt9d115_write( 0x36DC, 0xEF10 ); 	// P_BL_P3Q1
mt9d115_write( 0x36DE, 0x62F4 ); 	// P_BL_P3Q2
mt9d115_write( 0x36E0, 0x3512 ); 	// P_BL_P3Q3
mt9d115_write( 0x36E2, 0xE016 ); 	// P_BL_P3Q4
mt9d115_write( 0x36E4, 0x5311 ); 	// P_GB_P3Q0
mt9d115_write( 0x36E6, 0xA9F2 ); 	// P_GB_P3Q1
mt9d115_write( 0x36E8, 0x5494 ); 	// P_GB_P3Q2
mt9d115_write( 0x36EA, 0xD18F ); 	// P_GB_P3Q3
mt9d115_write( 0x36EC, 0xE536 ); 	// P_GB_P3Q4
mt9d115_write( 0x36EE, 0xEF73 ); 	// P_GR_P4Q0
mt9d115_write( 0x36F0, 0xADD2 ); 	// P_GR_P4Q1
mt9d115_write( 0x36F2, 0x0F76 ); 	// P_GR_P4Q2
mt9d115_write( 0x36F4, 0x49D6 ); 	// P_GR_P4Q3
mt9d115_write( 0x36F6, 0xA359 ); 	// P_GR_P4Q4
mt9d115_write( 0x36F8, 0xAB31 ); 	// P_RD_P4Q0
mt9d115_write( 0x36FA, 0xC772 ); 	// P_RD_P4Q1
mt9d115_write( 0x36FC, 0x12B6 ); 	// P_RD_P4Q2
mt9d115_write( 0x36FE, 0x4BB5 ); 	// P_RD_P4Q3
mt9d115_write( 0x3700, 0xBF99 ); 	// P_RD_P4Q4
mt9d115_write( 0x3702, 0xEB50 ); 	// P_BL_P4Q0
mt9d115_write( 0x3704, 0xF911 ); 	// P_BL_P4Q1
mt9d115_write( 0x3706, 0xFC51 ); 	// P_BL_P4Q2
mt9d115_write( 0x3708, 0x78F4 ); 	// P_BL_P4Q3
mt9d115_write( 0x370A, 0xDFF7 ); 	// P_BL_P4Q4
mt9d115_write( 0x370C, 0x81D4 ); 	// P_GB_P4Q0
mt9d115_write( 0x370E, 0xA191 ); 	// P_GB_P4Q1
mt9d115_write( 0x3710, 0x0CD6 ); 	// P_GB_P4Q2
mt9d115_write( 0x3712, 0x6955 ); 	// P_GB_P4Q3
mt9d115_write( 0x3714, 0x96D9 ); 	// P_GB_P4Q4
mt9d115_write( 0x3644, 0x0338 ); 	// POLY_ORIGIN_C
mt9d115_write( 0x3642, 0x0240 ); 	// POLY_ORIGIN_R
mt9d115_write( 0x3210, 0x01B8 ); 	// COLOR_PIPELINE_CONTROL

    // Saturation
mt9d115_write( 0x098C, 0xAB20 ); // MCU_ADDRESS [HG_LL_SAT1]
mt9d115_write( 0x0990, 0x006C ); // MCU_DATA_0


    //AWB
mt9d115_write( 0x098C, 0xA34A ); // MCU_ADDRESS [AWB_GAIN_MIN]
mt9d115_write( 0x0990, 0x0059 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA34B ); // MCU_ADDRESS [AWB_GAIN_MAX]
mt9d115_write( 0x0990, 0x00B6 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA34C ); // MCU_ADDRESS [AWB_GAINMIN_B]
mt9d115_write( 0x0990, 0x0059 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA34D ); // MCU_ADDRESS [AWB_GAINMAX_B]
mt9d115_write( 0x0990, 0x00B5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA351 ); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
mt9d115_write( 0x0990, 0x0000 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xA352 ); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
mt9d115_write( 0x0990, 0x007F ); // MCU_DATA_0
    
    // Gamma 0.40  Black 5  C 1.35
mt9d115_write( 0x098C, 0xAB4F ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
mt9d115_write( 0x0990, 0x0000 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB50 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
mt9d115_write( 0x0990, 0x000C ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB51 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
mt9d115_write( 0x0990, 0x0022 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB52 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
mt9d115_write( 0x0990, 0x003F ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB53 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
mt9d115_write( 0x0990, 0x0062 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB54 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
mt9d115_write( 0x0990, 0x007D ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB55 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
mt9d115_write( 0x0990, 0x0093 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB56 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
mt9d115_write( 0x0990, 0x00A5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB57 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
mt9d115_write( 0x0990, 0x00B3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB58 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
mt9d115_write( 0x0990, 0x00BF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB59 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
mt9d115_write( 0x0990, 0x00C9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5A ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
mt9d115_write( 0x0990, 0x00D3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5B ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
mt9d115_write( 0x0990, 0x00DB ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5C ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
mt9d115_write( 0x0990, 0x00E2 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5D ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
mt9d115_write( 0x0990, 0x00E9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5E ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
mt9d115_write( 0x0990, 0x00EF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5F ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
mt9d115_write( 0x0990, 0x00F5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB60 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
mt9d115_write( 0x0990, 0x00FA ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB61 ); // MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
mt9d115_write( 0x0990, 0x00FF ); // MCU_DATA_0
    
mt9d115_write( 0x098C, 0xAB3C ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_0]
mt9d115_write( 0x0990, 0x0000 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB3D ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_1]
mt9d115_write( 0x0990, 0x000C ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB3E ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_2]
mt9d115_write( 0x0990, 0x0022 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB3F ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_3]
mt9d115_write( 0x0990, 0x003F ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB40 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_4]
mt9d115_write( 0x0990, 0x0062 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB41 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_5]
mt9d115_write( 0x0990, 0x007D ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB42 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_6]
mt9d115_write( 0x0990, 0x0093 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB43 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_7]
mt9d115_write( 0x0990, 0x00A5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB44 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_8]
mt9d115_write( 0x0990, 0x00B3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB45 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_9]
mt9d115_write( 0x0990, 0x00BF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB46 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_10]
mt9d115_write( 0x0990, 0x00C9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB47 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_11]
mt9d115_write( 0x0990, 0x00D3 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB48 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_12]
mt9d115_write( 0x0990, 0x00DB ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB49 ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_13]
mt9d115_write( 0x0990, 0x00E2 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4A ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_14]
mt9d115_write( 0x0990, 0x00E9 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4B ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_15]
mt9d115_write( 0x0990, 0x00EF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4C ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_16]
mt9d115_write( 0x0990, 0x00F5 ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4D ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_17]
mt9d115_write( 0x0990, 0x00FA ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4E ); // MCU_ADDRESS [HG_GAMMA_TABLE_A_18]
mt9d115_write( 0x0990, 0x00FF ); // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2C );              // MCU_ADDRESS [HG_NR_START_R]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2D );              // MCU_ADDRESS [HG_NR_START_G]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2E );              // MCU_ADDRESS [HG_NR_START_B]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2F );              // MCU_ADDRESS [HG_NR_START_OL]
mt9d115_write( 0x0990, 0x0001 );    // MCU_DATA_0
//Add 2012-5-2 16:29
mt9d115_write( 0x098C, 0xA366 ); 	// MCU_ADDRESS [AWB_KR_L]
mt9d115_write( 0x0990, 0x0078 ); 	// MCU_DATA_0

mt9d115_write( 0x098C, 0x2306 ); 	// MCU_ADDRESS [AWB_CCM_L_0]
mt9d115_write( 0x0990, 0x01B1 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231C ); 	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9d115_write( 0x0990, 0x0028 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2308 ); 	// MCU_ADDRESS [AWB_CCM_L_1]
mt9d115_write( 0x0990, 0xFFA2 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231E ); 	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9d115_write( 0x0990, 0xFFE5 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230A ); 	// MCU_ADDRESS [AWB_CCM_L_2]
mt9d115_write( 0x0990, 0xFFAD ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2320 ); 	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9d115_write( 0x0990, 0xFFF3 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230C ); 	// MCU_ADDRESS [AWB_CCM_L_3]
mt9d115_write( 0x0990, 0xFF87 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2322 ); 	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9d115_write( 0x0990, 0xFFF6 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230E ); 	// MCU_ADDRESS [AWB_CCM_L_4]
mt9d115_write( 0x0990, 0x0184 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2324 ); 	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9d115_write( 0x0990, 0x0026 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2310 ); 	// MCU_ADDRESS [AWB_CCM_L_5]
mt9d115_write( 0x0990, 0xFFF5 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2326 ); 	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9d115_write( 0x0990, 0xFFE4 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2312 ); 	// MCU_ADDRESS [AWB_CCM_L_6]
mt9d115_write( 0x0990, 0xFFBF ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2328 ); 	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9d115_write( 0x0990, 0x001C ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2314 ); 	// MCU_ADDRESS [AWB_CCM_L_7]
mt9d115_write( 0x0990, 0xFF4F ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232A ); 	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9d115_write( 0x0990, 0x0011 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2316 ); 	// MCU_ADDRESS [AWB_CCM_L_8]
mt9d115_write( 0x0990, 0x01F3 ); 	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232C ); 	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9d115_write( 0x0990, 0xFFD3 ); 	// MCU_DATA_0


//

mt9d115_write( 0x098C, 0xAB30);  // MCU_ADDRESS [HG_NR_STOP_R]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024
mt9d115_write( 0x098C, 0xAB31);  // MCU_ADDRESS [HG_NR_STOP_G]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024
mt9d115_write( 0x098C, 0xAB32);  // MCU_ADDRESS [HG_NR_STOP_B]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024
mt9d115_write( 0x098C, 0xAB33);  // MCU_ADDRESS [HG_NR_STOP_OL]
mt9d115_write( 0x0990, 0x001E);    // MCU_DATA_0 //0X001E 20111024 
mt9d115_write( 0x098C, 0xAB34);  // MCU_ADDRESS [HG_NR_GAINSTART] 
mt9d115_write( 0x0990, 0x000E);    // MCU_DATA_0 
mt9d115_write( 0x098C, 0xAB35);  // MCU_ADDRESS [HG_NR_GAINSTOP]
mt9d115_write( 0x0990, 0x0091);    // MCU_DATA_0

mt9d115_write( 0x0018, 0x0028 ); 	// STANDBY_CONTROL // 初始化中加这个位置加上这一句

mt9d115_write( 0x33F4, 0x005B );    // MCU_DATA_0 //default 4b

//  把50Hz的参数，加在这个位置！

//[50HZ]
mt9d115_write(0x098C, 0xA118 );	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]        
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                 
mt9d115_write(0x098C, 0xA11E );	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]             
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                 
mt9d115_write(0x098C, 0xA124 );	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]             
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                     
mt9d115_write(0x098C, 0xA12A );	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]            
mt9d115_write(0x0990, 0x0002 );	// MCU_DATA_0                                  
mt9d115_write(0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]                       
mt9d115_write(0x0990, 0x00E0 );	// MCU_DATA_0                               
mt9d115_write(0x098C, 0xA103 );         
mt9d115_write(0x0990, 0x0005 );
msleep(50);	//delay=50
//[ANTIBANDING_SET_AUTO]
mt9d115_write(0x098C, 0xA118 );	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]          
mt9d115_write(0x0990, 0x0001 );	// MCU_DATA_0                              
mt9d115_write(0x098C, 0xA11E );	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]         
mt9d115_write(0x0990, 0x0001 );	// MCU_DATA_0                            
mt9d115_write(0x098C, 0xA124 );	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]         
mt9d115_write(0x0990, 0x0000 );	// MCU_DATA_0                                  
mt9d115_write(0x098C, 0xA12A );	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]          
mt9d115_write(0x0990, 0x0001 );	// MCU_DATA_0  
mt9d115_write(0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]                      
mt9d115_write(0x0990, 0x0060 );	// MCU_DATA_0                                
//mt9d115_write(0x098C, 0xA103 	                  
//mt9d115_write(0x0990, 0x0005 	        
//  POLL  MON_PATCH_ID_0 =>  0x01
mt9d115_write( 0x0018, 0x0028 );	// STANDBY_CONTROL
msleep(10);	//DELAY= 10
//  POLL  SEQ_STATE =>  0x03
mt9d115_write( 0x098C, 0xA103 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
	
	
	//mt9d115_write( 0x3070, 0x0100 );	// MCU_DATA_0  leyou:  a single patten
	
	/*added for debug only */	
			msleep(500);
			mt9d115_write( 0x098C, 0xA104 );
			msleep(50);
			printk(KERN_ERR"0x0990 read out : %d \n",mt9d115_read(0x0990));  /*expected to be 3 */
	/*---end ---*/
		
			printk(KERN_ERR"0x3400:read out :%d \n",mt9d115_read(0x3400));  /*0x3400*/
	
			if (checkCameraID(sensor)) {
			result = HAL_CAM_ERROR_INTERNAL_ERROR;
		}
		tm2 = ktime_get();
		pr_info("Exit Init Sec %d nsec %d\n",  tm2.tv.sec-tm1.tv.sec, tm2.tv.nsec-tm1.tv.nsec);
	
		return result;
	}


static HAL_CAM_Result_en_t Init_MT9D115_YuDe(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	static ktime_t tm1,tm2;
	tm1 = ktime_get();
	
	pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);
	
	CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;
	printk("Init Primary Sensor MT9T111: \r\n");
	
	/*  
	   * Preview YUV 640x480  Frame rate 10~ 27.5fps
          * Capture YUV 2048x1536	Frame Rate 8.45
          * XMCLK 26MHz	PCLK 64MHz
       */
       #if defined (CONFIG_CAM_CSI2)
	   printk(KERN_ERR"Init_MT9D115 CONFIG_CAM_CSI2 has been defined\n ");
	#endif 
	
       checkCameraID(sensor);
	//reset MCLK=24M . PCLK=240;
mt9d115_write( 0x001A, 0x0050 ); 	// RESET_AND_MISC_CONTROL
msleep(10);
mt9d115_write( 0x001A, 0x0058 );	// RESET_AND_MISC_CONTROL
msleep(20);
//pll settings
mt9d115_write( 0x0014, 0x21F9 );	// PLL_CONTROL
mt9d115_write( 0x0010, 0x0115 );	// PLL_DIVIDERS
mt9d115_write( 0x0012, 0x00F5 );	// PLL_P_DIVIDERS
mt9d115_write( 0x0014, 0x2545 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2547 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2447 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2047 );	// PLL_CONTROL
mt9d115_write( 0x0014, 0x2046 );	// PLL_CONTROL
mt9d115_write( 0x0018, 0x402D );	// STANDBY_CONTROL
msleep(50);                  
mt9d115_write( 0x0018, 0x402C );	// STANDBY_CONTROL
msleep(50);
//timing
mt9d115_write( 0x098C, 0x2703 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
mt9d115_write( 0x0990, 0x0640 );	// MCU_DATA_0   0640      mine:  0x0280
mt9d115_write( 0x098C, 0x2705 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
mt9d115_write( 0x0990, 0x04B0 );	// MCU_DATA_0   04b0      mine:  0x01E0
mt9d115_write( 0x098C, 0x2707 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_B]
mt9d115_write( 0x0990, 0x0640 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2709 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_B]
mt9d115_write( 0x0990, 0x04B0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x270D );	// MCU_ADDRESS [MODE_SENSOR_ROW_START_A]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x270F );	// MCU_ADDRESS [MODE_SENSOR_COL_START_A]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2711 );	// MCU_ADDRESS [MODE_SENSOR_ROW_END_A]
mt9d115_write( 0x0990, 0x04BB );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2713 );	// MCU_ADDRESS [MODE_SENSOR_COL_END_A]
mt9d115_write( 0x0990, 0x064B );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2715 );	// MCU_ADDRESS [MODE_SENSOR_ROW_SPEED_A]
mt9d115_write( 0x0990, 0x0111 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2717 );	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_A]
mt9d115_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2719 );	// MCU_ADDRESS [MODE_SENSOR_FINE_CORRECTION_A]
mt9d115_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x271B );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MIN_A]
mt9d115_write( 0x0990, 0x00F6 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x271D );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MAX_MARGIN_A]
mt9d115_write( 0x0990, 0x008B );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x271F );	// MCU_ADDRESS [MODE_SENSOR_FRAME_LENGTH_A]
mt9d115_write( 0x0990, 0x0521 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2721 );	// MCU_ADDRESS [MODE_SENSOR_LINE_LENGTH_PCK_A]
mt9d115_write( 0x0990, 0x0888 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2723 );	// MCU_ADDRESS [MODE_SENSOR_ROW_START_B]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2725 );	// MCU_ADDRESS [MODE_SENSOR_COL_START_B]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2727 );	// MCU_ADDRESS [MODE_SENSOR_ROW_END_B]
mt9d115_write( 0x0990, 0x04BB );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2729 );	// MCU_ADDRESS [MODE_SENSOR_COL_END_B]
mt9d115_write( 0x0990, 0x064B );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x272B );	// MCU_ADDRESS [MODE_SENSOR_ROW_SPEED_B]
mt9d115_write( 0x0990, 0x0111 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x272D );	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_B]
mt9d115_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x272F );	// MCU_ADDRESS [MODE_SENSOR_FINE_CORRECTION_B]
mt9d115_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2731 );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MIN_B]
mt9d115_write( 0x0990, 0x00F6 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2733 );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MAX_MARGIN_B]
mt9d115_write( 0x0990, 0x008B );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2735 );	// MCU_ADDRESS [MODE_SENSOR_FRAME_LENGTH_B]
mt9d115_write( 0x0990, 0x0521 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2737 );	// MCU_ADDRESS [MODE_SENSOR_LINE_LENGTH_PCK_B]
mt9d115_write( 0x0990, 0x0888 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2739 );	// MCU_ADDRESS [MODE_CROP_X0_A]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x273B );	// MCU_ADDRESS [MODE_CROP_X1_A]
mt9d115_write( 0x0990, 0x063F );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x273D );	// MCU_ADDRESS [MODE_CROP_Y0_A]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x273F );	// MCU_ADDRESS [MODE_CROP_Y1_A]
mt9d115_write( 0x0990, 0x04AF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2747 );	// MCU_ADDRESS [MODE_CROP_X0_B]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2749 );	// MCU_ADDRESS [MODE_CROP_X1_B]
mt9d115_write( 0x0990, 0x063F );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x274B );	// MCU_ADDRESS [MODE_CROP_Y0_B]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x274D );	// MCU_ADDRESS [MODE_CROP_Y1_B]
mt9d115_write( 0x0990, 0x04AF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2222 );	// MCU_ADDRESS [AE_R9]
mt9d115_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA408 );	// MCU_ADDRESS [FD_SEARCH_F1_50]
mt9d115_write( 0x0990, 0x0026 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA409 );	// MCU_ADDRESS [FD_SEARCH_F2_50]
mt9d115_write( 0x0990, 0x0029 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40A );	// MCU_ADDRESS [FD_SEARCH_F1_60]
mt9d115_write( 0x0990, 0x002E );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40B );	// MCU_ADDRESS [FD_SEARCH_F2_60]
mt9d115_write( 0x0990, 0x0031 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2411 );	// MCU_ADDRESS [FD_R9_STEP_F60_A]
mt9d115_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2413 );	// MCU_ADDRESS [FD_R9_STEP_F50_A]
mt9d115_write( 0x0990, 0x00C0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2415 );	// MCU_ADDRESS [FD_R9_STEP_F60_B]
mt9d115_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2417 );	// MCU_ADDRESS [FD_R9_STEP_F50_B]
mt9d115_write( 0x0990, 0x00C0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]
mt9d115_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40D );	// MCU_ADDRESS [FD_STAT_MIN]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA40E );	// MCU_ADDRESS [FD_STAT_MAX]
mt9d115_write( 0x0990, 0x0003 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA410 );	// MCU_ADDRESS [FD_MIN_AMPLITUDE]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
msleep(50);
mt9d115_write( 0x098C, 0xA103 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
msleep(100);
//AE
mt9d115_write( 0x098C, 0xA117 );	// MCU_ADDRESS [SEQ_PREVIEW_0_AE]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA11D );	// MCU_ADDRESS [SEQ_PREVIEW_1_AE]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA129 );	// MCU_ADDRESS [SEQ_PREVIEW_3_AE]
mt9d115_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA24F );	// MCU_ADDRESS [AE_BASETARGET]
mt9d115_write( 0x0990, 0x0032 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA20C );	// MCU_ADDRESS [AE_MAX_INDEX]
mt9d115_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA216 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA20E );	// MCU_ADDRESS [AE_MAX_VIRTGAIN]
mt9d115_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2212 );	// MCU_ADDRESS [AE_MAX_DGAIN_AE1]
mt9d115_write( 0x0990, 0x00A4 );	// MCU_DATA_0

//Gamma 
mt9d115_write( 0x3210, 0x01B8 );	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x098C, 0xAB36 );	// MCU_ADDRESS [RESERVED_HG_36]
mt9d115_write( 0x0990, 0x0014 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B66 );	// MCU_ADDRESS [HG_CLUSTER_DC_BM]
mt9d115_write( 0x0990, 0x2AF8 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB20 );	// MCU_ADDRESS [HG_LL_SAT1]
mt9d115_write( 0x0990, 0x0080 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB24 );	// MCU_ADDRESS [HG_LL_SAT2]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB21 );	// MCU_ADDRESS [HG_LL_INTERPTHRESH1]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB25 );	// MCU_ADDRESS [HG_LL_INTERPTHRESH2]
mt9d115_write( 0x0990, 0x002A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB22 );	// MCU_ADDRESS [HG_LL_APCORR1]
mt9d115_write( 0x0990, 0x0007 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB26 );	// MCU_ADDRESS [HG_LL_APCORR2]
mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB23 );	// MCU_ADDRESS [HG_LL_APTHRESH1]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB27 );	// MCU_ADDRESS [HG_LL_APTHRESH2]
mt9d115_write( 0x0990, 0x0009 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B28 );	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]
mt9d115_write( 0x0990, 0x0BB8 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B2A );	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]
mt9d115_write( 0x0990, 0x2968 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2C );	// MCU_ADDRESS [HG_NR_START_R]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB30 );	// MCU_ADDRESS [HG_NR_STOP_R]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2D );	// MCU_ADDRESS [HG_NR_START_G]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB31 );	// MCU_ADDRESS [HG_NR_STOP_G]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2E );	// MCU_ADDRESS [HG_NR_START_B]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB32 );	// MCU_ADDRESS [HG_NR_STOP_B]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB2F );	// MCU_ADDRESS [HG_NR_START_OL]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB33 );	// MCU_ADDRESS [HG_NR_STOP_OL]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB34 );	// MCU_ADDRESS [HG_NR_GAINSTART]
mt9d115_write( 0x0990, 0x0020 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB35 );	// MCU_ADDRESS [HG_NR_GAINSTOP]
mt9d115_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA765 );	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_FILTER_MODE]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB37 );	// MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]
mt9d115_write( 0x0990, 0x0003 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B38 );	// MCU_ADDRESS [HG_GAMMASTARTMORPH]
mt9d115_write( 0x0990, 0x2968 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B3A );	// MCU_ADDRESS [HG_GAMMASTOPMORPH]
mt9d115_write( 0x0990, 0x2D50 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B62 );	// MCU_ADDRESS [HG_FTB_START_BM]
mt9d115_write( 0x0990, 0xFFFE );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2B64 );	// MCU_ADDRESS [HG_FTB_STOP_BM]
mt9d115_write( 0x0990, 0xFFFF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB4F );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
mt9d115_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB50 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
mt9d115_write( 0x0990, 0x0013 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB51 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
mt9d115_write( 0x0990, 0x0027 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB52 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
mt9d115_write( 0x0990, 0x0043 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB53 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
mt9d115_write( 0x0990, 0x0068 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB54 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
mt9d115_write( 0x0990, 0x0081 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB55 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
mt9d115_write( 0x0990, 0x0093 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB56 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
mt9d115_write( 0x0990, 0x00A3 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB57 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
mt9d115_write( 0x0990, 0x00B0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB58 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
mt9d115_write( 0x0990, 0x00BC );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB59 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
mt9d115_write( 0x0990, 0x00C7 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5A );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
mt9d115_write( 0x0990, 0x00D1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5B );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
mt9d115_write( 0x0990, 0x00DA );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5C );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
mt9d115_write( 0x0990, 0x00E2 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5D );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
mt9d115_write( 0x0990, 0x00E9 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5E );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
mt9d115_write( 0x0990, 0x00EF );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB5F );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
mt9d115_write( 0x0990, 0x00F4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB60 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
mt9d115_write( 0x0990, 0x00FA );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xAB61 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
mt9d115_write( 0x0990, 0x00FF );	// MCU_DATA_0
                              
//AWB &CCM                    
mt9d115_write( 0x098C, 0x2306 );	// MCU_ADDRESS [AWB_CCM_L_0]
mt9d115_write( 0x0990, 0x01D6 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2308 );	// MCU_ADDRESS [AWB_CCM_L_1]
mt9d115_write( 0x0990, 0xFF89 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230A );	// MCU_ADDRESS [AWB_CCM_L_2]
mt9d115_write( 0x0990, 0xFFA1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230C );	// MCU_ADDRESS [AWB_CCM_L_3]
mt9d115_write( 0x0990, 0xFF73 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x230E );	// MCU_ADDRESS [AWB_CCM_L_4]
mt9d115_write( 0x0990, 0x019C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2310 );	// MCU_ADDRESS [AWB_CCM_L_5]
mt9d115_write( 0x0990, 0xFFF1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2312 );	// MCU_ADDRESS [AWB_CCM_L_6]
mt9d115_write( 0x0990, 0xFFB0 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2314 );	// MCU_ADDRESS [AWB_CCM_L_7]
mt9d115_write( 0x0990, 0xFF2D );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2316 );	// MCU_ADDRESS [AWB_CCM_L_8]
mt9d115_write( 0x0990, 0x0223 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0048 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x001E );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x0022 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x002C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9d115_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9d115_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231C );	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9d115_write( 0x0990, 0xFFCD );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x231E );	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9d115_write( 0x0990, 0x0023 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2320 );	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9d115_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2322 );	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9d115_write( 0x0990, 0x0026 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2324 );	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9d115_write( 0x0990, 0xFFE9 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2326 );	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9d115_write( 0x0990, 0xFFF1 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2328 );	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9d115_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232A );	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9d115_write( 0x0990, 0x005D );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232C );	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9d115_write( 0x0990, 0xFF69 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFE4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000C );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0xFFFC );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9d115_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9d115_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
                              
//patch                       
mt9d115_write( 0x098C, 0x0415 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xF601 );
mt9d115_write( 0x0992, 0x42C1 );
mt9d115_write( 0x0994, 0x0326 );
mt9d115_write( 0x0996, 0x11F6 );
mt9d115_write( 0x0998, 0x0143 );
mt9d115_write( 0x099A, 0xC104 );
mt9d115_write( 0x099C, 0x260A );
mt9d115_write( 0x099E, 0xCC04 );
mt9d115_write( 0x098C, 0x0425 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x33BD );
mt9d115_write( 0x0992, 0xA362 );
mt9d115_write( 0x0994, 0xBD04 );
mt9d115_write( 0x0996, 0x3339 );
mt9d115_write( 0x0998, 0xC6FF );
mt9d115_write( 0x099A, 0xF701 );
mt9d115_write( 0x099C, 0x6439 );
mt9d115_write( 0x099E, 0xFE01 );
mt9d115_write( 0x098C, 0x0435 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x6918 );
mt9d115_write( 0x0992, 0xCE03 );
mt9d115_write( 0x0994, 0x25CC );
mt9d115_write( 0x0996, 0x0013 );
mt9d115_write( 0x0998, 0xBDC2 );
mt9d115_write( 0x099A, 0xB8CC );
mt9d115_write( 0x099C, 0x0489 );
mt9d115_write( 0x099E, 0xFD03 );
mt9d115_write( 0x098C, 0x0445 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x27CC );
mt9d115_write( 0x0992, 0x0325 );
mt9d115_write( 0x0994, 0xFD01 );
mt9d115_write( 0x0996, 0x69FE );
mt9d115_write( 0x0998, 0x02BD );
mt9d115_write( 0x099A, 0x18CE );
mt9d115_write( 0x099C, 0x0339 );
mt9d115_write( 0x099E, 0xCC00 );
mt9d115_write( 0x098C, 0x0455 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x11BD );
mt9d115_write( 0x0992, 0xC2B8 );
mt9d115_write( 0x0994, 0xCC04 );
mt9d115_write( 0x0996, 0xC8FD );
mt9d115_write( 0x0998, 0x0347 );
mt9d115_write( 0x099A, 0xCC03 );
mt9d115_write( 0x099C, 0x39FD );
mt9d115_write( 0x099E, 0x02BD );
mt9d115_write( 0x098C, 0x0465 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xDE00 );
mt9d115_write( 0x0992, 0x18CE );
mt9d115_write( 0x0994, 0x00C2 );
mt9d115_write( 0x0996, 0xCC00 );
mt9d115_write( 0x0998, 0x37BD );
mt9d115_write( 0x099A, 0xC2B8 );
mt9d115_write( 0x099C, 0xCC04 );
mt9d115_write( 0x099E, 0xEFDD );
mt9d115_write( 0x098C, 0x0475 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xE6CC );
mt9d115_write( 0x0992, 0x00C2 );
mt9d115_write( 0x0994, 0xDD00 );
mt9d115_write( 0x0996, 0xC601 );
mt9d115_write( 0x0998, 0xF701 );
mt9d115_write( 0x099A, 0x64C6 );
mt9d115_write( 0x099C, 0x03F7 );
mt9d115_write( 0x099E, 0x0165 );
mt9d115_write( 0x098C, 0x0485 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x7F01 );
mt9d115_write( 0x0992, 0x6639 );
mt9d115_write( 0x0994, 0x3C3C );
mt9d115_write( 0x0996, 0x3C34 );
mt9d115_write( 0x0998, 0xCC32 );
mt9d115_write( 0x099A, 0x3EBD );
mt9d115_write( 0x099C, 0xA558 );
mt9d115_write( 0x099E, 0x30ED );
mt9d115_write( 0x098C, 0x0495 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x04BD );
mt9d115_write( 0x0992, 0xB2D7 );
mt9d115_write( 0x0994, 0x30E7 );
mt9d115_write( 0x0996, 0x06CC );
mt9d115_write( 0x0998, 0x323E );
mt9d115_write( 0x099A, 0xED00 );
mt9d115_write( 0x099C, 0xEC04 );
mt9d115_write( 0x099E, 0xBDA5 );
mt9d115_write( 0x098C, 0x04A5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x44CC );
mt9d115_write( 0x0992, 0x3244 );
mt9d115_write( 0x0994, 0xBDA5 );
mt9d115_write( 0x0996, 0x585F );
mt9d115_write( 0x0998, 0x30ED );
mt9d115_write( 0x099A, 0x02CC );
mt9d115_write( 0x099C, 0x3244 );
mt9d115_write( 0x099E, 0xED00 );
mt9d115_write( 0x098C, 0x04B5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xF601 );
mt9d115_write( 0x0992, 0xD54F );
mt9d115_write( 0x0994, 0xEA03 );
mt9d115_write( 0x0996, 0xAA02 );
mt9d115_write( 0x0998, 0xBDA5 );
mt9d115_write( 0x099A, 0x4430 );
mt9d115_write( 0x099C, 0xE606 );
mt9d115_write( 0x099E, 0x3838 );
mt9d115_write( 0x098C, 0x04C5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x3831 );
mt9d115_write( 0x0992, 0x39BD );
mt9d115_write( 0x0994, 0xD661 );
mt9d115_write( 0x0996, 0xF602 );
mt9d115_write( 0x0998, 0xF4C1 );
mt9d115_write( 0x099A, 0x0126 );
mt9d115_write( 0x099C, 0x0BFE );
mt9d115_write( 0x099E, 0x02BD );
mt9d115_write( 0x098C, 0x04D5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0xEE10 );
mt9d115_write( 0x0992, 0xFC02 );
mt9d115_write( 0x0994, 0xF5AD );
mt9d115_write( 0x0996, 0x0039 );
mt9d115_write( 0x0998, 0xF602 );
mt9d115_write( 0x099A, 0xF4C1 );
mt9d115_write( 0x099C, 0x0226 );
mt9d115_write( 0x099E, 0x0AFE );
mt9d115_write( 0x098C, 0x04E5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x02BD );
mt9d115_write( 0x0992, 0xEE10 );
mt9d115_write( 0x0994, 0xFC02 );
mt9d115_write( 0x0996, 0xF7AD );
mt9d115_write( 0x0998, 0x0039 );
mt9d115_write( 0x099A, 0x3CBD );
mt9d115_write( 0x099C, 0xB059 );
mt9d115_write( 0x099E, 0xCC00 );
mt9d115_write( 0x098C, 0x04F5 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x28BD );
mt9d115_write( 0x0992, 0xA558 );
mt9d115_write( 0x0994, 0x8300 );
mt9d115_write( 0x0996, 0x0027 );
mt9d115_write( 0x0998, 0x0BCC );
mt9d115_write( 0x099A, 0x0026 );
mt9d115_write( 0x099C, 0x30ED );
mt9d115_write( 0x099E, 0x00C6 );
mt9d115_write( 0x098C, 0x0505 );	// MCU_ADDRESS
mt9d115_write( 0x0990, 0x03BD );
mt9d115_write( 0x0992, 0xA544 );
mt9d115_write( 0x0994, 0x3839 );
mt9d115_write( 0x098C, 0x2006 );	// MCU_ADDRESS [MON_ARG1]
mt9d115_write( 0x0990, 0x0415 );	// MCU_DATA_0
mt9d115_write( 0x098C, 0xA005 );	// MCU_ADDRESS [MON_CMD]
mt9d115_write( 0x0990, 0x0001 );	// MCU_DATA_0
msleep(50);                 
//LSC              
#if 0
mt9d115_write( 0x3210, 0x01B0 );	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x364E, 0x0490 );	// P_GR_P0Q0
mt9d115_write( 0x3650, 0x5308 );	// P_GR_P0Q1
mt9d115_write( 0x3652, 0x04D2 );	// P_GR_P0Q2
mt9d115_write( 0x3654, 0xB06E );	// P_GR_P0Q3
mt9d115_write( 0x3656, 0xC432 );	// P_GR_P0Q4
mt9d115_write( 0x3658, 0x7E8F );	// P_RD_P0Q0
mt9d115_write( 0x365A, 0x024C );	// P_RD_P0Q1
mt9d115_write( 0x365C, 0x1232 );	// P_RD_P0Q2
mt9d115_write( 0x365E, 0xBBCF );	// P_RD_P0Q3
mt9d115_write( 0x3660, 0x96B2 );	// P_RD_P0Q4
mt9d115_write( 0x3662, 0x7EEF );	// P_BL_P0Q0
mt9d115_write( 0x3664, 0xC7A8 );	// P_BL_P0Q1
mt9d115_write( 0x3666, 0x7BD1 );	// P_BL_P0Q2
mt9d115_write( 0x3668, 0xC5AE );	// P_BL_P0Q3
mt9d115_write( 0x366A, 0xCD52 );	// P_BL_P0Q4
mt9d115_write( 0x366C, 0x7E8F );	// P_GB_P0Q0
mt9d115_write( 0x366E, 0x424B );	// P_GB_P0Q1
mt9d115_write( 0x3670, 0x01F2 );	// P_GB_P0Q2
mt9d115_write( 0x3672, 0xDACE );	// P_GB_P0Q3
mt9d115_write( 0x3674, 0xB7B2 );	// P_GB_P0Q4
mt9d115_write( 0x3676, 0x260D );	// P_GR_P1Q0
mt9d115_write( 0x3678, 0x8210 );	// P_GR_P1Q1
mt9d115_write( 0x367A, 0xD76C );	// P_GR_P1Q2
mt9d115_write( 0x367C, 0x78B1 );	// P_GR_P1Q3
mt9d115_write( 0x367E, 0xC811 );	// P_GR_P1Q4
mt9d115_write( 0x3680, 0x062E );	// P_RD_P1Q0
mt9d115_write( 0x3682, 0x564F );	// P_RD_P1Q1
mt9d115_write( 0x3684, 0x04CF );	// P_RD_P1Q2
mt9d115_write( 0x3686, 0x8CF2 );	// P_RD_P1Q3
mt9d115_write( 0x3688, 0xBE92 );	// P_RD_P1Q4
mt9d115_write( 0x368A, 0x298C );	// P_BL_P1Q0
mt9d115_write( 0x368C, 0xD2EF );	// P_BL_P1Q1
mt9d115_write( 0x368E, 0xA0AF );	// P_BL_P1Q2
mt9d115_write( 0x3690, 0x7371 );	// P_BL_P1Q3
mt9d115_write( 0x3692, 0xE5B0 );	// P_BL_P1Q4
mt9d115_write( 0x3694, 0x75CC );	// P_GB_P1Q0
mt9d115_write( 0x3696, 0x62AF );	// P_GB_P1Q1
mt9d115_write( 0x3698, 0xCCED );	// P_GB_P1Q2
mt9d115_write( 0x369A, 0x85F2 );	// P_GB_P1Q3
mt9d115_write( 0x369C, 0xB132 );	// P_GB_P1Q4
mt9d115_write( 0x369E, 0x0073 );	// P_GR_P2Q0
mt9d115_write( 0x36A0, 0x8FAE );	// P_GR_P2Q1
mt9d115_write( 0x36A2, 0xE9B3 );	// P_GR_P2Q2
mt9d115_write( 0x36A4, 0xD230 );	// P_GR_P2Q3
mt9d115_write( 0x36A6, 0x8E14 );	// P_GR_P2Q4
mt9d115_write( 0x36A8, 0x73B2 );	// P_RD_P2Q0
mt9d115_write( 0x36AA, 0xCD4C );	// P_RD_P2Q1
mt9d115_write( 0x36AC, 0x8631 );	// P_RD_P2Q2
mt9d115_write( 0x36AE, 0x8A52 );	// P_RD_P2Q3
mt9d115_write( 0x36B0, 0xBF96 );	// P_RD_P2Q4
mt9d115_write( 0x36B2, 0x4F52 );	// P_BL_P2Q0
mt9d115_write( 0x36B4, 0x832E );	// P_BL_P2Q1
mt9d115_write( 0x36B6, 0xC5D3 );	// P_BL_P2Q2
mt9d115_write( 0x36B8, 0xADAF );	// P_BL_P2Q3
mt9d115_write( 0x36BA, 0xB0B5 );	// P_BL_P2Q4
mt9d115_write( 0x36BC, 0x02B3 );	// P_GB_P2Q0
mt9d115_write( 0x36BE, 0x8E8D );	// P_GB_P2Q1
mt9d115_write( 0x36C0, 0xD5D3 );	// P_GB_P2Q2
mt9d115_write( 0x36C2, 0xE7B1 );	// P_GB_P2Q3
mt9d115_write( 0x36C4, 0x84D5 );	// P_GB_P2Q4
mt9d115_write( 0x36C6, 0xC24E );	// P_GR_P3Q0
mt9d115_write( 0x36C8, 0x48B1 );	// P_GR_P3Q1
mt9d115_write( 0x36CA, 0xA412 );	// P_GR_P3Q2
mt9d115_write( 0x36CC, 0x1A92 );	// P_GR_P3Q3
mt9d115_write( 0x36CE, 0x0BF6 );	// P_GR_P3Q4
mt9d115_write( 0x36D0, 0x858E );	// P_RD_P3Q0
mt9d115_write( 0x36D2, 0xB0B1 );	// P_RD_P3Q1
mt9d115_write( 0x36D4, 0xDFD3 );	// P_RD_P3Q2
mt9d115_write( 0x36D6, 0x12B2 );	// P_RD_P3Q3
mt9d115_write( 0x36D8, 0x5076 );	// P_RD_P3Q4
mt9d115_write( 0x36DA, 0xD1AF );	// P_BL_P3Q0
mt9d115_write( 0x36DC, 0x46B1 );	// P_BL_P3Q1
mt9d115_write( 0x36DE, 0x9CB3 );	// P_BL_P3Q2
mt9d115_write( 0x36E0, 0x8291 );	// P_BL_P3Q3
mt9d115_write( 0x36E2, 0x2696 );	// P_BL_P3Q4
mt9d115_write( 0x36E4, 0xB30D );	// P_GB_P3Q0
mt9d115_write( 0x36E6, 0xF491 );	// P_GB_P3Q1
mt9d115_write( 0x36E8, 0xE8F3 );	// P_GB_P3Q2
mt9d115_write( 0x36EA, 0x6012 );	// P_GB_P3Q3
mt9d115_write( 0x36EC, 0x0677 );	// P_GB_P3Q4
mt9d115_write( 0x36EE, 0xB654 );	// P_GR_P4Q0
mt9d115_write( 0x36F0, 0x94CF );	// P_GR_P4Q1
mt9d115_write( 0x36F2, 0xB317 );	// P_GR_P4Q2
mt9d115_write( 0x36F4, 0x24B5 );	// P_GR_P4Q3
mt9d115_write( 0x36F6, 0x307A );	// P_GR_P4Q4
mt9d115_write( 0x36F8, 0x8C53 );	// P_RD_P4Q0
mt9d115_write( 0x36FA, 0xFB70 );	// P_RD_P4Q1
mt9d115_write( 0x36FC, 0x8E98 );	// P_RD_P4Q2
mt9d115_write( 0x36FE, 0x7253 );	// P_RD_P4Q3
mt9d115_write( 0x3700, 0x6CFA );	// P_RD_P4Q4
mt9d115_write( 0x3702, 0xBF13 );	// P_BL_P4Q0
mt9d115_write( 0x3704, 0x4A50 );	// P_BL_P4Q1
mt9d115_write( 0x3706, 0xEA37 );	// P_BL_P4Q2
mt9d115_write( 0x3708, 0x95F0 );	// P_BL_P4Q3
mt9d115_write( 0x370A, 0x04FB );	// P_BL_P4Q4
mt9d115_write( 0x370C, 0xBAF4 );	// P_GB_P4Q0
mt9d115_write( 0x370E, 0xA511 );	// P_GB_P4Q1
mt9d115_write( 0x3710, 0xD377 );	// P_GB_P4Q2
mt9d115_write( 0x3712, 0x2B95 );	// P_GB_P4Q3
mt9d115_write( 0x3714, 0x4FFA );	// P_GB_P4Q4
mt9d115_write( 0x3644, 0x0320 );	// POLY_ORIGIN_C
mt9d115_write( 0x3642, 0x0264 );	// POLY_ORIGIN_R
mt9d115_write( 0x3210, 0x01B8 );	// COLOR_PIPELINE_CONTROL
#else
//[95% LSC]
mt9d115_write( 0x3210, 0x01B0 ); 	// COLOR_PIPELINE_CONTROL
mt9d115_write( 0x364E, 0x0910 ); 	// P_GR_P0Q0
mt9d115_write( 0x3650, 0x088C ); 	// P_GR_P0Q1
mt9d115_write( 0x3652, 0x7830 ); 	// P_GR_P0Q2
mt9d115_write( 0x3654, 0xC5CB ); 	// P_GR_P0Q3
mt9d115_write( 0x3656, 0x5F91 ); 	// P_GR_P0Q4
mt9d115_write( 0x3658, 0x0190 ); 	// P_RD_P0Q0
mt9d115_write( 0x365A, 0xEBAB ); 	// P_RD_P0Q1
mt9d115_write( 0x365C, 0x4D90 ); 	// P_RD_P0Q2
mt9d115_write( 0x365E, 0x19ED ); 	// P_RD_P0Q3
mt9d115_write( 0x3660, 0x6132 ); 	// P_RD_P0Q4
mt9d115_write( 0x3662, 0x0150 ); 	// P_BL_P0Q0
mt9d115_write( 0x3664, 0x6BAA ); 	// P_BL_P0Q1
mt9d115_write( 0x3666, 0x2A30 ); 	// P_BL_P0Q2
mt9d115_write( 0x3668, 0x976A ); 	// P_BL_P0Q3
mt9d115_write( 0x366A, 0x4C12 ); 	// P_BL_P0Q4
mt9d115_write( 0x366C, 0x00D0 ); 	// P_GB_P0Q0
mt9d115_write( 0x366E, 0x88CB ); 	// P_GB_P0Q1
mt9d115_write( 0x3670, 0x04B1 ); 	// P_GB_P0Q2
mt9d115_write( 0x3672, 0x0C4E ); 	// P_GB_P0Q3
mt9d115_write( 0x3674, 0x3DF1 ); 	// P_GB_P0Q4
mt9d115_write( 0x3676, 0xC2EA ); 	// P_GR_P1Q0
mt9d115_write( 0x3678, 0xAFCE ); 	// P_GR_P1Q1
mt9d115_write( 0x367A, 0x6730 ); 	// P_GR_P1Q2
mt9d115_write( 0x367C, 0x5EF0 ); 	// P_GR_P1Q3
mt9d115_write( 0x367E, 0xBF71 ); 	// P_GR_P1Q4
mt9d115_write( 0x3680, 0x500C ); 	// P_RD_P1Q0
mt9d115_write( 0x3682, 0x484F ); 	// P_RD_P1Q1
mt9d115_write( 0x3684, 0x732F ); 	// P_RD_P1Q2
mt9d115_write( 0x3686, 0xFE31 ); 	// P_RD_P1Q3
mt9d115_write( 0x3688, 0xD4D1 ); 	// P_RD_P1Q4
mt9d115_write( 0x368A, 0xB74B ); 	// P_BL_P1Q0
mt9d115_write( 0x368C, 0xEE4D ); 	// P_BL_P1Q1
mt9d115_write( 0x368E, 0x2150 ); 	// P_BL_P1Q2
mt9d115_write( 0x3690, 0x3790 ); 	// P_BL_P1Q3
mt9d115_write( 0x3692, 0xCFF2 ); 	// P_BL_P1Q4
mt9d115_write( 0x3694, 0x952D ); 	// P_GB_P1Q0
mt9d115_write( 0x3696, 0x5B2F ); 	// P_GB_P1Q1
mt9d115_write( 0x3698, 0x32B0 ); 	// P_GB_P1Q2
mt9d115_write( 0x369A, 0x9AD1 ); 	// P_GB_P1Q3
mt9d115_write( 0x369C, 0xAED1 ); 	// P_GB_P1Q4
mt9d115_write( 0x369E, 0x72B2 ); 	// P_GR_P2Q0
mt9d115_write( 0x36A0, 0xFCD0 ); 	// P_GR_P2Q1
mt9d115_write( 0x36A2, 0xA2D3 ); 	// P_GR_P2Q2
mt9d115_write( 0x36A4, 0x3872 ); 	// P_GR_P2Q3
mt9d115_write( 0x36A6, 0x1056 ); 	// P_GR_P2Q4
mt9d115_write( 0x36A8, 0x51F2 ); 	// P_RD_P2Q0
mt9d115_write( 0x36AA, 0xD8CF ); 	// P_RD_P2Q1
mt9d115_write( 0x36AC, 0xB0D2 ); 	// P_RD_P2Q2
mt9d115_write( 0x36AE, 0x5D91 ); 	// P_RD_P2Q3
mt9d115_write( 0x36B0, 0x3075 ); 	// P_RD_P2Q4
mt9d115_write( 0x36B2, 0x44D2 ); 	// P_BL_P2Q0
mt9d115_write( 0x36B4, 0xA350 ); 	// P_BL_P2Q1
mt9d115_write( 0x36B6, 0xFAD0 ); 	// P_BL_P2Q2
mt9d115_write( 0x36B8, 0x1392 ); 	// P_BL_P2Q3
mt9d115_write( 0x36BA, 0x2D94 ); 	// P_BL_P2Q4
mt9d115_write( 0x36BC, 0x7812 ); 	// P_GB_P2Q0
mt9d115_write( 0x36BE, 0xC0D0 ); 	// P_GB_P2Q1
mt9d115_write( 0x36C0, 0xB7D3 ); 	// P_GB_P2Q2
mt9d115_write( 0x36C2, 0x0D73 ); 	// P_GB_P2Q3
mt9d115_write( 0x36C4, 0x0416 ); 	// P_GB_P2Q4
mt9d115_write( 0x36C6, 0x41F1 ); 	// P_GR_P3Q0
mt9d115_write( 0x36C8, 0xC1B1 ); 	// P_GR_P3Q1
mt9d115_write( 0x36CA, 0x78B4 ); 	// P_GR_P3Q2
mt9d115_write( 0x36CC, 0x3A33 ); 	// P_GR_P3Q3
mt9d115_write( 0x36CE, 0x8F57 ); 	// P_GR_P3Q4
mt9d115_write( 0x36D0, 0x4770 ); 	// P_RD_P3Q0
mt9d115_write( 0x36D2, 0x8EF2 ); 	// P_RD_P3Q1
mt9d115_write( 0x36D4, 0x0E95 ); 	// P_RD_P3Q2
mt9d115_write( 0x36D6, 0x2AB3 ); 	// P_RD_P3Q3
mt9d115_write( 0x36D8, 0xBEB7 ); 	// P_RD_P3Q4
mt9d115_write( 0x36DA, 0x790F ); 	// P_BL_P3Q0
mt9d115_write( 0x36DC, 0xEF10 ); 	// P_BL_P3Q1
mt9d115_write( 0x36DE, 0x62F4 ); 	// P_BL_P3Q2
mt9d115_write( 0x36E0, 0x3512 ); 	// P_BL_P3Q3
mt9d115_write( 0x36E2, 0xE016 ); 	// P_BL_P3Q4
mt9d115_write( 0x36E4, 0x5311 ); 	// P_GB_P3Q0
mt9d115_write( 0x36E6, 0xA9F2 ); 	// P_GB_P3Q1
mt9d115_write( 0x36E8, 0x5494 ); 	// P_GB_P3Q2
mt9d115_write( 0x36EA, 0xD18F ); 	// P_GB_P3Q3
mt9d115_write( 0x36EC, 0xE536 ); 	// P_GB_P3Q4
mt9d115_write( 0x36EE, 0xEF73 ); 	// P_GR_P4Q0
mt9d115_write( 0x36F0, 0xADD2 ); 	// P_GR_P4Q1
mt9d115_write( 0x36F2, 0x0F76 ); 	// P_GR_P4Q2
mt9d115_write( 0x36F4, 0x49D6 ); 	// P_GR_P4Q3
mt9d115_write( 0x36F6, 0xA359 ); 	// P_GR_P4Q4
mt9d115_write( 0x36F8, 0xAB31 ); 	// P_RD_P4Q0
mt9d115_write( 0x36FA, 0xC772 ); 	// P_RD_P4Q1
mt9d115_write( 0x36FC, 0x12B6 ); 	// P_RD_P4Q2
mt9d115_write( 0x36FE, 0x4BB5 ); 	// P_RD_P4Q3
mt9d115_write( 0x3700, 0xBF99 ); 	// P_RD_P4Q4
mt9d115_write( 0x3702, 0xEB50 ); 	// P_BL_P4Q0
mt9d115_write( 0x3704, 0xF911 ); 	// P_BL_P4Q1
mt9d115_write( 0x3706, 0xFC51 ); 	// P_BL_P4Q2
mt9d115_write( 0x3708, 0x78F4 ); 	// P_BL_P4Q3
mt9d115_write( 0x370A, 0xDFF7 ); 	// P_BL_P4Q4
mt9d115_write( 0x370C, 0x81D4 ); 	// P_GB_P4Q0
mt9d115_write( 0x370E, 0xA191 ); 	// P_GB_P4Q1
mt9d115_write( 0x3710, 0x0CD6 ); 	// P_GB_P4Q2
mt9d115_write( 0x3712, 0x6955 ); 	// P_GB_P4Q3
mt9d115_write( 0x3714, 0x96D9 ); 	// P_GB_P4Q4
mt9d115_write( 0x3644, 0x0338 ); 	// POLY_ORIGIN_C
mt9d115_write( 0x3642, 0x0240 ); 	// POLY_ORIGIN_R
mt9d115_write( 0x3210, 0x01B8 ); 	// COLOR_PIPELINE_CONTROL

#endif

mt9d115_write( 0x0018, 0x0028 );	// STANDBY_CONTROL
msleep(50);              
mt9d115_write( 0x098C, 0xA103 );	// MCU_ADDRESS [SEQ_CMD]
mt9d115_write( 0x0990, 0x0006 );	// MCU_DATA_0
msleep(300);


//mt9d115_write( 0x3070, 0x0100 );	// MCU_DATA_0  leyou:  a single patten

/*added for debug only */	
		msleep(500);
		mt9d115_write( 0x098C, 0xA104 );
		msleep(50);
		printk(KERN_ERR"0x0990 read out : %d \n",mt9d115_read(0x0990));  /*expected to be 3 */
/*---end ---*/
	
	

		if (checkCameraID(sensor)) {
		result = HAL_CAM_ERROR_INTERNAL_ERROR;
	}
	tm2 = ktime_get();
	pr_info("Exit Init Sec %d nsec %d\n",  tm2.tv.sec-tm1.tv.sec, tm2.tv.nsec-tm1.tv.nsec);

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
    DRV_DisableCapture : CAMDRV_DisableCapture_Pri,
    DRV_DisablePreview : CAMDRV_DisablePreview_Pri,
    DRV_CfgStillnThumbCapture : CAMDRV_CfgStillnThumbCapture_Pri,
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
};

struct sens_methods *CAMDRV_primary_get(void)
{
	return &sens_meth_pri;
}
