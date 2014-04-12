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
*   @file   camdrv_hi253.c
*
*   @brief  This file is the lower level driver API of HI253(3M 2048*1536
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

#define CAMERA_IMAGE_INTERFACE  CSL_CAM_INTF_CPI
#define CAMERA_PHYS_INTF_PORT   CSL_CAM_PORT_AFE_1



//hanwei@wind-mobi.com add 2011.11.17 start
//reviewed by liubing@wind-mobi.com 
#define HI253_ID 0x92	   
#define HAL_CAM_PWDN 13 

#define HAL_CAM_RESET 56    /*gpio 56*/
#define FLASH_TRIGGER  30   /*gpio 30 for flash */
//hanwei@wind-mobi.com add 2011.11.17 end

#define HAL_CAM_VGA_STANDBY  9

#define HAL_CAM_VGA_RESET 10

int real_write=0;
//hanwei@wind-mobi.com add 2011.11.22 start
//reviewed by liubing@wind-mobi.com
UInt32 pv_exposure=0x015f00;//jerry debug
UInt32 cp_exposure=0;//jerry debug
UInt8 exposure_a = 0;
UInt8 exposure_b = 0;
UInt8 exposure_c = 0;
//hanwei@wind-mobi.com add 2011.11.22 end
CamFocusControlMode_t FOCUS_MODE;


/*****************************************************************************/
/* start of CAM configuration */
/*****************************************************************************/

/*****************************************************************************/
/*  Sensor Resolution Tables:												 */
/*****************************************************************************/
/* Resolutions & Sizes available for HI253 ISP/Sensor (QXGA max) */
static CamResolutionConfigure_t sSensorResInfo_HI253_st[] = {
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
//hanwei@wind-mobi.ocm 2011.11.02 config power on sequence  begin

	{GPIO_CNTRL, HAL_CAM_PWDN, GPIO_SetHigh},
	{MCLK_CNTRL, CamDrv_24MHz, CLK_TurnOn},

	{GPIO_CNTRL, HAL_CAM_PWDN, GPIO_SetLow},
	{PAUSE, 10, Nop_Cmd},

	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},
//hanwei@wind-mobi.ocm 2011.11.02 config power on sequence  end
};

/*---------Sensor Power Off*/
static CamSensorIntfCntrl_st_t CamPowerOffSeq[] = {
//hanwei@wind-mobi.ocm 2011.11.02 config power off sequence  begin

	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetLow},
	{PAUSE, 10, Nop_Cmd},
	{GPIO_CNTRL, HAL_CAM_PWDN, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
	{PAUSE, 10, Nop_Cmd},
//hanwei@wind-mobi.ocm 2011.11.02 config power off sequence  begin
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
	"HI253"
};

/*---------Sensor Primary Configuration CCIR656*/

//hanwei@wind-mobi.com 2011.11.02 Camera Vsync,Hsync,Clock config begin
static CamIntfConfig_CCIR656_st_t CamPrimaryCfg_CCIR656_st = {
	// Vsync, Hsync, Clock
	CSL_CAM_SYNC_EXTERNAL,				///< UInt32 sync_mode;				(default)CAM_SYNC_EXTERNAL:  Sync External or Embedded
	CSL_CAM_SYNC_DEFINES_ACTIVE,		///< UInt32 vsync_control;			(default)CAM_SYNC_DEFINES_ACTIVE:		VSYNCS determines active data
	CSL_CAM_SYNC_ACTIVE_LOW,			///< UInt32 vsync_polarity; 		   default)ACTIVE_LOW/ACTIVE_HIGH:		  Vsync active
	CSL_CAM_SYNC_DEFINES_ACTIVE,		///< UInt32 hsync_control;			(default)FALSE/TRUE:					HSYNCS determines active data
	CSL_CAM_SYNC_ACTIVE_HIGH,			///< UInt32 hsync_polarity; 		(default)ACTIVE_HIGH/ACTIVE_LOW:		Hsync active
	CSL_CAM_CLK_EDGE_POS,				///< UInt32 data_clock_sample;		(default)RISING_EDGE/FALLING_EDGE:		Pixel Clock Sample edge
	CSL_CAM_PIXEL_8BIT, 				///< UInt32 bus_width;				(default)CAM_BITWIDTH_8:				Camera bus width
	0,							///< UInt32 data_shift; 				   (default)0:							   data shift (+) left shift  (-) right shift
	CSL_CAM_FIELD_H_V,					///< UInt32 field_mode; 			(default)CAM_FIELD_H_V: 				field calculated
	CSL_CAM_INT_FRAME_END,				///< UInt32 data_intr_enable;		CAM_INTERRUPT_t:
	CSL_CAM_INT_FRAME_END,				///< UInt32 pkt_intr_enable;		CAM_INTERRUPT_t:

};
//hanwei@wind-mobi.com 2011.11.02 Camera Vsync,Hsync,Clock config end

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
static HAL_CAM_Result_en_t Init_Hi253(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt16 hi253_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t hi253_write(unsigned int sub_addr, UInt16 data);
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
			    ("Error in allocating %d bytes for HI253 sensor\n",
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

	result = Init_Hi253(sensor);
	printk("Init_Hi253 result =%d\r\n", result);
	return result;
}

static UInt16 CAMDRV_GetDeviceID_Pri(CamSensorSelect_t sensor)
{
	return hi253_read(0x04); //hanwei
}

static int checkCameraID(CamSensorSelect_t sensor)
{
	UInt16 devId = hi253_read(0x04); //hanwei

	if (devId == HI253_ID) {
		printk("Camera identified as HI253\r\n");
		return 0;
	} else {
		printk("Camera Id wrong. Expected 0x%x but got 0x%x\r\n",
			 HI253_ID, devId);
		return -1;
	}
}

static HAL_CAM_Result_en_t hi253_write(unsigned int sub_addr, UInt16 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	
	UInt8 write_data=data;
	result |= CAM_WriteI2c_Byte((UInt8)sub_addr, 1, &write_data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9t111_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
//hanwei@wind-mobi.com 2011.11.02 set write byte:8 end
	return result;
}

static UInt16 hi253_read(unsigned int sub_addr)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 data=0xFF;
	
	result |= CAM_ReadI2c_Byte((UInt8)sub_addr, 1,  &data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9t111_read(): ERROR: %d\r\n", result);
	}
	return data;
}

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format)
{	
	UInt32 x = 0, y = 0;
	UInt32 format = 0;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	printk(KERN_ERR"SensorSetPreviewMode image_resolution 0x%x image_format %d  ",image_resolution,image_format  );
	//hanwei@wind-mobi.com add 2011.11.22 start
	//reviewed by liubing@wind-mobi.com
	exposure_a=pv_exposure>>16;
	exposure_b=(pv_exposure>>8)&0x000000FF;
	exposure_c=pv_exposure&0x000000FF;
	  
	hi253_write(0x03, 0x00);
	hi253_write(0x12, 0x00);
	hi253_write(0x03, 0x20);
	hi253_write(0x83, exposure_a);
	hi253_write(0x84, exposure_b);
	hi253_write(0x85, exposure_c);
	//hanwei@wind-mobi.com add 2011.11.22 end
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
//hanwei@wind-mobi.com add 2011.11.04 start
//reviewed by liubing@wind-mobi.com
		//hi253_write(0x03, 0x00);  //hanwei del 2011.11.08 
		//hi253_write(0x01, 0xc1);  //hanwei del 2011.11.08 
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x21);
		hi253_write(0x03, 0x10);
		hi253_write(0x10, 0x03);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0xe0);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x08);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x04);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x08);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xe4);
		hi253_write(0x2c, 0x0a);
		hi253_write(0x2d, 0x00);
		hi253_write(0x2e, 0x0a);
		hi253_write(0x2f, 0x00);
		hi253_write(0x30, 0x44);
		//hi253_write(0x03, 0x00); //hanwei del 2011.11.08 
		//hi253_write(0x01, 0xc0); //hanwei del 2011.11.08 
//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_CIF:
		x = 352;
		y = 288;
		break;

	case CamImageSize_VGA:
		x = 640;
		y = 480;
//hanwei@wind-mobi.com add 2011.11.04 start
//reviewed by liubing@wind-mobi.com
		//hi253_write(0x03, 0x20);
		//hi253_write(0x10, 0x1c);
		//hi253_write(0x18, 0x38);

		//hi253_write(0x83, 0x01);
		//hi253_write(0x84, 0x5f);
		//hi253_write(0x85, 0x00);

		//hi253_write(0x03, 0x20);
		//hi253_write(0x10, 0x9c);
		//hi253_write(0x18, 0x30);		
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x11);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0xe0);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x04);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x04);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xe0);
		hi253_write(0x2c, 0x0a);
		hi253_write(0x2d, 0x00);
		hi253_write(0x2e, 0x0a);
		hi253_write(0x2f, 0x00);
		hi253_write(0x30, 0x44);
//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_SVGA:
		x = 800;
		y = 600;
		break;

	default:
		x = 320;
		y = 240;
//hanwei@wind-mobi.com add 2011.11.04 start
//reviewed by liubing@wind-mobi.com 
		hi253_write(0x03, 0x00); 
		hi253_write(0x01, 0xc1); 
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x21);
		hi253_write(0x03, 0x10);
		hi253_write(0x10, 0x03);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0xe0);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x08);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x04);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x08);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xe4);
		hi253_write(0x2c, 0x0a);
		hi253_write(0x2d, 0x00);
		hi253_write(0x2e, 0x0a);
		hi253_write(0x2f, 0x00);
		hi253_write(0x30, 0x44);
		hi253_write(0x03, 0x00); 
		hi253_write(0x01, 0xc0); 
//hanwei@wind-mobi.com add 2011.11.04 end
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

//hanwei@wind-mobi.com del 2011.11.04 start
//reviewed by liubing@wind-mobi.com
printk("####SensorSetPreviewMode end#####");
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
		hi253_write(0x03,0x10);
		hi253_write(0x12,(hi253_read(0x12)|0x10));//make sure the Yoffset control is opened.
		hi253_write(0x40,0x00);	
	}
	else if(value>0)
	{
		hi253_write(0x03,0x10);
		hi253_write(0x12,(hi253_read(0x12)|0x10));//make sure the Yoffset control is opened.
		hi253_write(0x40,0x40);	
	}
	else
	{
		hi253_write(0x03,0x10);
		hi253_write(0x12,(hi253_read(0x12)|0x10));//make sure the Yoffset control is opened.
		hi253_write(0x40,0xb0);
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
//hanwei@wind-mobi.com debug 2011.11.04 start
//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
//hanwei@wind-mobi.com debug 2011.11.04 end
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

			}
			else if(fps>=CamRate_10)
			{

			}
			else if(fps>=CamRate_5)
			{
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
	return HAL_CAM_SUCCESS;
	/*[Enable stream] */
	printk(KERN_ERR"hi253 CAMDRV_EnableVideoCapture \r\n");
//hanwei@wind-mobi.com debug 2011.11.04 start
//reviewed by liubing@wind-mobi.com
	//hi253_write(0x001A, 0x0218);
//hanwei@wind-mobi.com debug 2011.11.04 end
	msleep(300);
	return sCamI2cStatus;
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
	UInt16 readout;
	printk("CAMDRV_SetCamSleep(): hi253 enter soft standby mode \r\n");
//hanwei@wind-mobi del 2011.11.04 start
//reviewed by liubing@wind-mobi.com	
return 	HAL_CAM_SUCCESS;
//hanwei@wind-mobi.com del 2011.11.04 end	return result;
}

/*camera_disable for still mode */
static HAL_CAM_Result_en_t CAMDRV_DisableCapture_Pri(CamSensorSelect_t sensor)
{
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
/ Description: This function halts HI253 camera video
/
/ Notes:   camera_disable for preview mode , after this ,preview will not output data
/
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_DisablePreview_Pri(CamSensorSelect_t sensor)
{
HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
 
    return result;
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
		UInt32 x = 0, y = 0;
		UInt32 tx = 0, ty = 0;
		HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

		printk(KERN_ERR"***CAMDRV_CfgStillnThumbCapture():STARTS *** \r\n");

		//hanwei@wind-mobi.com add 2011.11.22 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x20);
		pv_exposure=(hi253_read(0x80)<<16)|(hi253_read(0x81)<<8)|(hi253_read(0x82));
		cp_exposure=pv_exposure/2;
		exposure_a=cp_exposure>>16;
		exposure_b=(cp_exposure>>8)&0x000000FF;
		exposure_c=cp_exposure&0x000000FF;
		hi253_write(0x03, 0x20);
		hi253_write(0x83, exposure_a);
		hi253_write(0x84, exposure_b);
		hi253_write(0x85, exposure_c);
		hi253_write(0x03, 0x00);
		hi253_write(0x12, 0x01);
		//hanwei@wind-mobi.com add 2011.11.22 end

		//hanwei@wind-mobi.com reset 2011.11.17 start
		//for enable camera flash light
		//reviewed by liubing@wind-mobi.com
			if(flash_switch==Flash_On)
				control_flash(1);
		//hanwei@wind-mobi.com reset 2011.11.17 end
			switch (stills_resolution) {
			case CamImageSize_QCIF:
		  
				x = 176;
				y = 144;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x21);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x03);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0x20);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x20);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x02);
		hi253_write(0x29, 0xe0);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0x20);
		hi253_write(0x2c, 0x10);
		hi253_write(0x2d, 0xaa);
		hi253_write(0x2e, 0x10);
		hi253_write(0x2f, 0xaa);
		hi253_write(0x30, 0x67);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_QVGA:
		x = 320;
		y = 240;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x21);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0xe0);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x00);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x00);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xe0);
		hi253_write(0x2c, 0x0a);
		hi253_write(0x2d, 0x00);
		hi253_write(0x2e, 0x0a);
		hi253_write(0x2f, 0x00);
		hi253_write(0x30, 0x44);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_CIF:
		x = 352;
		y = 288;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x11);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x03);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0x20);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x20);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x02);
		hi253_write(0x29, 0xe0);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0x20);
		hi253_write(0x2c, 0x10);
		hi253_write(0x2d, 0xaa);
		hi253_write(0x2e, 0x10);
		hi253_write(0x2f, 0xaa);
		hi253_write(0x30, 0x67);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_VGA:
		x = 640;
		y = 480;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com

		
		
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x11);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0xe0);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x04);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x04);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xe0);
		hi253_write(0x2c, 0x0a);
		hi253_write(0x2d, 0x00);
		hi253_write(0x2e, 0x0a);
		hi253_write(0x2f, 0x00);
		hi253_write(0x30, 0x44);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_SVGA:
		x = 800;
		y = 600;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x11);
		hi253_write(0x03, 0x18);
		hi253_write(0x10, 0x00);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_XGA:
		x = 1024;
		y = 768;
		printk(KERN_ERR"-------------coming here for 1024x768 resoluton---- ");
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03,0x00);
		hi253_write(0x10,0x00);

		hi253_write(0x03, 0x18);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x12, 0x20);

		hi253_write(0x20, 0x04);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x03);
		hi253_write(0x23, 0x00);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x00);
		hi253_write(0x28, 0x04);
		hi253_write(0x29, 0x00);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x2a, 0x03);
		hi253_write(0x2b, 0x00);
		hi253_write(0x2c, 0x0c);
		hi253_write(0x2d, 0x80);
		hi253_write(0x2e, 0x0c); 
		hi253_write(0x2f, 0x80);
		hi253_write(0x30, 0x60);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_SXGA:
		x = 1280;
		y = 1024;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03,0x00);
		hi253_write(0x10,0x00);

		hi253_write(0x03, 0x18);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x12, 0x20);

		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x54);
		hi253_write(0x22, 0x04);
		hi253_write(0x23, 0x00);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x2a);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x2a);		
		hi253_write(0x2a, 0x04);
		hi253_write(0x2b, 0x00);
		hi253_write(0x2c, 0x09);
		hi253_write(0x2d, 0x60);
		hi253_write(0x2e, 0x09); 
		hi253_write(0x2f, 0x62);
		hi253_write(0x30, 0x3d);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_UXGA:
		x = 1600;
		y = 1200;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x00);
		hi253_write(0x03, 0x18);
		hi253_write(0x10, 0x00);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	case CamImageSize_QXGA: /*the same with UXGA */
		x = 2048;
		y = 1536;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x00);
		hi253_write(0x03, 0x18);
		hi253_write(0x10, 0x00);
		//hanwei@wind-mobi.com add 2011.11.04 end
		break;

	default:
		x = 640;
		y = 480;
		//hanwei@wind-mobi.com add 2011.11.04 start
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x11);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x00);
		hi253_write(0x22, 0x01);
		hi253_write(0x23, 0xe0);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x00);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x00);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x00);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xe0);
		hi253_write(0x2c, 0x0a);
		hi253_write(0x2d, 0x00);
		hi253_write(0x2e, 0x0a);
		hi253_write(0x2f, 0x00);
		hi253_write(0x30, 0x44);
		break;
	}
	
                             
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk
		    ("CAMDRV_CfgStillnThumbCapture():Error sending capture mode \r\n");
		result = sCamI2cStatus;
	}

	printk(KERN_ERR"CAMDRV_CfgStillnThumbCapture(): stills_resolution = 0x%x, \
		 stills_format=0x%x \r\n", stills_resolution, stills_format);

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
	switch(scene_mode) {
		case CamSceneMode_Auto:
			printk("CamSceneMode_Auto \r\n");
			hi253_write(0x03, 0x00);
			hi253_write(0x90, 0x04);
			hi253_write(0x91, 0x04);
						
			hi253_write(0x03, 0x20);
			hi253_write(0xd4, 0x04);
			hi253_write(0xd5, 0x04);
					
			hi253_write(0x03, 0x20);
			hi253_write(0x10, 0x1c);
			hi253_write(0x18, 0x38);
			hi253_write(0x2a, 0xff); //0x03); //for fix//0xff);
			hi253_write(0x2b, 0x34); //0x35); //for fix//0x04); 
			hi253_write(0x30, 0x78); 
						
			hi253_write(0x83, 0x01);
			hi253_write(0x84, 0x5f);
			hi253_write(0x85, 0x00);
			
			hi253_write(0x88, 0x05);
			hi253_write(0x89, 0x7c);
			hi253_write(0x8a, 0x00);
			
			hi253_write(0x03, 0x20);
			hi253_write(0x10, 0x9c);
			hi253_write(0x18, 0x30);
			break;
		case CamSceneMode_Night:
			printk("CamSceneMode_Night \r\n");			
			hi253_write(0x03, 0x00);
			hi253_write(0x90, 0x0d);
			hi253_write(0x91, 0x0d);

			hi253_write(0x03, 0x02);
			hi253_write(0xd4, 0x0d);
			hi253_write(0xd5, 0x0d);

			hi253_write(0x03, 0x20);
			hi253_write(0x10, 0x1c);
			hi253_write(0x18, 0x38);
			hi253_write(0x2a, 0xff); //for variable//0x03); //for fix//0xff);
			hi253_write(0x2b, 0x34); //for variable//0x35); //for fix//0x04); 
			hi253_write(0x30, 0x78); 

			hi253_write(0x83, 0x05); //min5fps
			hi253_write(0x84, 0xf3);
			hi253_write(0x85, 0x70);

			hi253_write(0x88, 0x07); //min5fps
			hi253_write(0x89, 0x50);
			hi253_write(0x8a, 0x00);

			hi253_write(0x03, 0x20);
			hi253_write(0x10, 0x9c);
			hi253_write(0x18, 0x30);
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
{
	//hanwei@wind-mobi.com chg 2011.12.22 start
	//enable and set White Balance mode 
	//reviewed by liubing@wind-mobi.com
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetWBMode()  called  wb_mode = 0x%x \n",wb_mode);
	switch (wb_mode)
	{
		case CamWB_Auto:	
			printk("CamWB_Auto\r\n");
			hi253_write(0x03, 0x22);			
			hi253_write(0x11, 0x2e);				
			//hi253_write(0x80, 0x40);
			//hi253_write(0x81, 0x20);
			//hi253_write(0x82, 0x3E);
			hi253_write(0x83, 0x5e);
			hi253_write(0x84, 0x1e);
			hi253_write(0x85, 0x5e);
			hi253_write(0x86, 0x22);		
			break;	
		case CamWB_Daylight:	//sunny	
			printk("CamWB_Daylight\r\n");	
			hi253_write(0x03, 0x22);
			hi253_write(0x11, 0x28);		  
			hi253_write(0x80, 0x59);
			hi253_write(0x82, 0x29);
			hi253_write(0x83, 0x60);
			hi253_write(0x84, 0x50);
			hi253_write(0x85, 0x28);
			hi253_write(0x86, 0x20);	
			break;		
		case CamWB_Fluorescent:			
			printk("CamWB_Fluorescent\r\n");
			hi253_write(0x03, 0x22);
			hi253_write(0x11, 0x28);
			hi253_write(0x80, 0x41);
			hi253_write(0x82, 0x42);
			hi253_write(0x83, 0x4a);
			hi253_write(0x84, 0x38);
			hi253_write(0x85, 0x46);
			hi253_write(0x86, 0x40);
			break;
		case CamWB_Cloudy:		
			printk("CamWB_Cloudy\r\n");
			hi253_write(0x03, 0x22);
			hi253_write(0x11, 0x28);
			hi253_write(0x80, 0x51);//71); fix bug#6823
			hi253_write(0x82, 0x2b);
			hi253_write(0x83, 0x52);
			hi253_write(0x84, 0x50);
			hi253_write(0x85, 0x23);
			hi253_write(0x86, 0x20);		
			break;
		case CamWB_Incandescent:	//home	
			printk("CamWB_Incandescent\r\n");
			hi253_write(0x03, 0x22);
			hi253_write(0x11, 0x28);		  
			hi253_write(0x80, 0x29);
			hi253_write(0x82, 0x54);
			hi253_write(0x83, 0x2e);
			hi253_write(0x84, 0x23);
			hi253_write(0x85, 0x58);
			hi253_write(0x86, 0x4f);		
			break;	
		case CamWB_Twilight:
			printk("CamWB_Twilight\r\n");
			hi253_write(0x03, 0x22);			
			hi253_write(0x11, 0x2e);				
			hi253_write(0x80, 0x40);
			hi253_write(0x81, 0x20);
			hi253_write(0x82, 0x3E);
			hi253_write(0x83, 0x5e);
			hi253_write(0x84, 0x1e);
			hi253_write(0x85, 0x5e);
			hi253_write(0x86, 0x22);	
			break;
		case CamWB_Tungsten:
			printk("CamWB_Tungsten\r\n");
			hi253_write(0x03, 0x22);
			hi253_write(0x80, 0x24);
			hi253_write(0x81, 0x20);
			hi253_write(0x82, 0x58);
			hi253_write(0x83, 0x27);
			hi253_write(0x84, 0x22);
			hi253_write(0x85, 0x58);
			hi253_write(0x86, 0x52);
			break;
		default:
			printk("HAL_CAM_ERROR_ACTION_NOT_SUPPORTED\r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
	}
	//hanwei@wind-mobi.com chg 2011.12.22 end
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
	pr_info("CAMDRV_SetAntiBanding()  called\n");
	//hanwei@wind-mobi.com chg 2012.01.06 start
	//fix bug#6826 antibanding effect 
	//reviewed by liubing@wind-mobi.com
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) {
		hi253_write(0x03, 0x20);
		hi253_write(0x10, 0x1c);
		hi253_write(0x18, 0x38);
		hi253_write(0x83, 0x04);
		hi253_write(0x84, 0x92);
		hi253_write(0x85, 0x00);
		hi253_write(0x10, 0x9c);
		hi253_write(0x18, 0x30);

	} else if (effect == CamAntiBanding50Hz) {
	printk("    CamAntiBanding50Hz            \n");
		hi253_write(0x03, 0x20);
		hi253_write(0x10, 0x1c);
		hi253_write(0x18, 0x38);
		hi253_write(0x83, 0x04);
		hi253_write(0x84, 0x92);
		hi253_write(0x85, 0x00);
		hi253_write(0x10, 0x9c);
		hi253_write(0x18, 0x30);

	} else if (effect == CamAntiBanding60Hz) {
	printk("    CamAntiBanding60Hz            \n");
		hi253_write(0x03, 0x20);
		hi253_write(0x10, 0x1c);	
		hi253_write(0x18, 0x38);	
		hi253_write(0x83, 0x01);
		hi253_write(0x84, 0x84);
		hi253_write(0x85, 0x00);
		hi253_write(0x10, 0x8c);
		hi253_write(0x18, 0x30);

	} else {
	printk("    else       CamAntiBanding      \n");
		hi253_write(0x03, 0x20);
		hi253_write(0x10, 0x1c);
		hi253_write(0x18, 0x38);
		hi253_write(0x83, 0x04);
		hi253_write(0x84, 0x92);
		hi253_write(0x85, 0x00);
		hi253_write(0x10, 0x9c);
		hi253_write(0x18, 0x30);

	}
	//hanwei@wind-mobi.com chg 2012.01.06 end
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
	//hanwei@wind-mobi.com chg 2011.11.17 start
	//reviewed by liubing@wind-mobi.com 
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
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
	//hanwei@wind-mobi.com chg 2011.11.17 end
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
//hanwei@wind-mobi.com debug 2011.11.04 start
//reviewed by liubing@wind-mobi.com
return HAL_CAM_SUCCESS;
//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetFocusMode()  called effect 0x%x \n",effect);
	if (effect == CamFocusControlAuto) {

			
		
	} else if (effect == CamFocusControlMacro) {

		

	} else if (effect == CamFocusControlInfinity) {


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
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	switch (FOCUS_MODE)
	{
		case CamFocusControlAuto: /*AF trigger*/

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
	printk("hanwei:---debug:*******CAMDRV_SetDigitalEffect_Pri(): effect[0x%x] *****\r\n",effect);
#if 0
	if (effect == CamDigEffect_NoEffect) 
		else if (effect == CamDigEffect_MonoChrome) 
			else if ((effect == CamDigEffect_NegColor)
		   || (effect == CamDigEffect_NegMono)) 
		   else if ((effect == CamDigEffect_SolarizeColor)
		   || (effect == CamDigEffect_SolarizeMono)) 
		   else if (effect == CamDigEffect_SepiaGreen) 
		   	else if (effect == CamDigEffect_Auqa) 
				else 
#endif

switch (effect)
	{	
		case CamDigEffect_NoEffect:		
			printk(" CamDigEffect_NoEffect \r\n");
		  	hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x03);
			hi253_write(0x12, 0x30);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x80);
			hi253_write(0x45, 0x80);		
			break;	
			
		case CamDigEffect_SepiaGreen:	//antique
			printk(" CamDigEffect_SepiaGreen \r\n");
			hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x03);
			hi253_write(0x12, 0x33);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x22);
			hi253_write(0x45, 0xa2);	
			break;	
			
		case CamDigEffect_MonoChrome:	//B&W	
			printk(" CamDigEffect_MonoChrome \r\n");
			hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x03);
			hi253_write(0x12, 0xa3);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x80);
			hi253_write(0x45, 0x80);
			break;	
			
		case CamDigEffect_NegColor:		//Negative
			printk(" CamDigEffect_NegColor \r\n");
			hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x03);
			hi253_write(0x12, 0x38);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x80);
			hi253_write(0x45, 0x80);
			break;
			
		case CamDigEffect_Auqa:
			printk(" CamDigEffect_Auqa \r\n");
			hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x03);
			hi253_write(0x12, 0x33);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x30);
			hi253_write(0x45, 0x50);			
			break;
			
		case CamDigEffect_SolarizeColor:
		case CamDigEffect_SolarizeMono:
			printk(" CamDigEffect_Solarize \r\n");
			hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x0b);
			hi253_write(0x12, 0x38);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x80);
			hi253_write(0x45, 0x80);			
			break;
			
		case CamDigEffect_Posterize:
		case CamDigEffect_PosterizeMono:
			printk(" CamDigEffect_Posterise \r\n");
			hi253_write(0x03, 0x10);
			hi253_write(0x11, 0x0b);
			hi253_write(0x12, 0x30);
			hi253_write(0x13, 0x0a);
			hi253_write(0x44, 0x80);
			hi253_write(0x45, 0x80);			
			break;			
		default:
			printk(" HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
	}	

			
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}
//hanwei@wind-mobi.com 2011.11.02 init hi253 parameters begin

static HAL_CAM_Result_en_t Init_Hi253(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	ktime_t tm1;

	printk("!!!! HI253 Init_HI253 !!!!\r\n");
	
	int timeout;
	tm1 = ktime_get();
	pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;

	if (checkCameraID(sensor)) {
		return HAL_CAM_ERROR_INTERNAL_ERROR;
	}

	//HI253
		hi253_write(0x01, 0xf9); //sleep on
		hi253_write(0x08, 0x0f); //Hi-Z on
		hi253_write(0x01, 0xf8); //sleep off

		hi253_write(0x03, 0x00); // Dummy 750us START
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00); // Dummy 750us END

		hi253_write(0x0e, 0x03); //PLL On
		hi253_write(0x0e, 0x73); //PLLx2

		hi253_write(0x03, 0x00); // Dummy 750us START
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00); // Dummy 750us END

		hi253_write(0x0e, 0x00); //PLL off
		hi253_write(0x01, 0xf1); //sleep on
		hi253_write(0x08, 0x00); //Hi-Z off

		hi253_write(0x01, 0xf3);
		hi253_write(0x01, 0xf1);

	// PAGE 20
		hi253_write(0x03, 0x20); //page 20
		hi253_write(0x10, 0x1c); //ae off

	// PAGE 22
		hi253_write(0x03, 0x22); //page 22
		hi253_write(0x10, 0x69); //awb off


	//Initial Start
	/////// PAGE 0 START ///////
		hi253_write(0x03, 0x00);
		hi253_write(0x10, 0x11); // Sub1/2_Preview2 Mode_H binning
		hi253_write(0x11, 0x93);
		hi253_write(0x12, 0x00); //00

		hi253_write(0x0b, 0xaa); // ESD Check Register
		hi253_write(0x0c, 0xaa); // ESD Check Register
		hi253_write(0x0d, 0xaa); // ESD Check Register

		hi253_write(0x20, 0x00); // Windowing start point Y
		hi253_write(0x21, 0x04);
		hi253_write(0x22, 0x00); // Windowing start point X
		hi253_write(0x23, 0x07);

		hi253_write(0x24, 0x04);
		hi253_write(0x25, 0xb0);
		hi253_write(0x26, 0x06);
		hi253_write(0x27, 0x40); // WINROW END

		hi253_write(0x40, 0x01); //Hblank 408
		hi253_write(0x41, 0x98); 
		hi253_write(0x42, 0x00); //Vblank 20
		hi253_write(0x43, 0x14);

		hi253_write(0x45, 0x04);
		hi253_write(0x46, 0x18);
		hi253_write(0x47, 0xd8);

	//BLC
		hi253_write(0x80, 0x2e);
		hi253_write(0x81, 0x7e);
		hi253_write(0x82, 0x90);
		hi253_write(0x83, 0x00);
		hi253_write(0x84, 0x0c);
		hi253_write(0x85, 0x00);
		hi253_write(0x90, 0x0a); //BLC_TIME_TH_ON
		hi253_write(0x91, 0x0a); //BLC_TIME_TH_OFF 
		hi253_write(0x92, 0x58); //BLC_AG_TH_ON
		hi253_write(0x93, 0x50); //BLC_AG_TH_OFF
		hi253_write(0x94, 0x75);
		hi253_write(0x95, 0x70);
		hi253_write(0x96, 0xdc);
		hi253_write(0x97, 0xfe);
		hi253_write(0x98, 0x38);

	//OutDoor  BLC
		hi253_write(0x99, 0x43);
		hi253_write(0x9a, 0x43);
		hi253_write(0x9b, 0x43);
		hi253_write(0x9c, 0x43);

	//Dark BLC
		hi253_write(0xa0, 0x00);
		hi253_write(0xa2, 0x00);
		hi253_write(0xa4, 0x00);
		hi253_write(0xa6, 0x00);

	//Normal BLC
		hi253_write(0xa8, 0x43);
		hi253_write(0xaa, 0x43);
		hi253_write(0xac, 0x43);
		hi253_write(0xae, 0x43);

		hi253_write(0x03, 0x02); //Page 02
		hi253_write(0x10, 0x00); //Mode_test
		hi253_write(0x11, 0x00); //Mode_dead_test
		hi253_write(0x12, 0x03); //pwr_ctl_ctl1
		hi253_write(0x13, 0x03); //Mode_ana_test
		hi253_write(0x14, 0x00); //mode_memory
		hi253_write(0x16, 0x00); //dcdc_ctl1
		hi253_write(0x17, 0x8c); //dcdc_ctl2
		hi253_write(0x18, 0x4C); //analog_func1
		hi253_write(0x19, 0x00); //analog_func2
		hi253_write(0x1a, 0x39); //analog_func3
		hi253_write(0x1b, 0x00); //analog_func4
		hi253_write(0x1c, 0x09); //dcdc_ctl3
		hi253_write(0x1d, 0x40); //dcdc_ctl4
		hi253_write(0x1e, 0x30); //analog_func7
		hi253_write(0x1f, 0x10); //analog_func8
		hi253_write(0x20, 0x77); //pixel bias
		hi253_write(0x21, 0xde); //adc,asp bias
		hi253_write(0x22, 0xa7); //main,bus bias
		hi253_write(0x23, 0x30); //clamp
		hi253_write(0x24, 0x4a);		
		hi253_write(0x25, 0x10);		
		hi253_write(0x27, 0x3c);		
		hi253_write(0x28, 0x00);		
		hi253_write(0x29, 0x0c);		
		hi253_write(0x2a, 0x80);		
		hi253_write(0x2b, 0x80);		
		hi253_write(0x2c, 0x02);		
		hi253_write(0x2d, 0xa0);		
		hi253_write(0x2e, 0x11);		
		hi253_write(0x2f, 0xa1);		
		hi253_write(0x30, 0x05); //swap_ctl
		hi253_write(0x31, 0x99);		
		hi253_write(0x32, 0x00);		
		hi253_write(0x33, 0x00);		
		hi253_write(0x34, 0x22);		
		hi253_write(0x38, 0x88);		
		hi253_write(0x39, 0x88);		
		hi253_write(0x50, 0x20);		
		hi253_write(0x51, 0x00);		
		hi253_write(0x52, 0x01);		
		hi253_write(0x53, 0xc1);		
		hi253_write(0x54, 0x10);		
		hi253_write(0x55, 0x1c);		
		hi253_write(0x56, 0x11);		
		hi253_write(0x58, 0x10);		
		hi253_write(0x59, 0x0e);		
		hi253_write(0x5d, 0xa2);		
		hi253_write(0x5e, 0x5a);		
		hi253_write(0x60, 0x87);		
		hi253_write(0x61, 0x99);		
		hi253_write(0x62, 0x88);		
		hi253_write(0x63, 0x97);		
		hi253_write(0x64, 0x88);		
		hi253_write(0x65, 0x97);		
		hi253_write(0x67, 0x0c);		
		hi253_write(0x68, 0x0c);		
		hi253_write(0x69, 0x0c);		
		hi253_write(0x6a, 0xb4);		
		hi253_write(0x6b, 0xc4);		
		hi253_write(0x6c, 0xb5);		
		hi253_write(0x6d, 0xc2);		
		hi253_write(0x6e, 0xb5);		
		hi253_write(0x6f, 0xc0);		
		hi253_write(0x70, 0xb6);		
		hi253_write(0x71, 0xb8);		
		hi253_write(0x72, 0x89);		
		hi253_write(0x73, 0x96);		
		hi253_write(0x74, 0x89);		
		hi253_write(0x75, 0x96);		
		hi253_write(0x76, 0x89);		
		hi253_write(0x77, 0x96);		
		hi253_write(0x7c, 0x85);		
		hi253_write(0x7d, 0xaf);		
		hi253_write(0x80, 0x01);		
		hi253_write(0x81, 0x7f);		
		hi253_write(0x82, 0x13); //rx_on1_read
		hi253_write(0x83, 0x24);		
		hi253_write(0x84, 0x7D);		
		hi253_write(0x85, 0x81);		
		hi253_write(0x86, 0x7D);		
		hi253_write(0x87, 0x81);		
		hi253_write(0x88, 0xab);		
		hi253_write(0x89, 0xbc);		
		hi253_write(0x8a, 0xac);		
		hi253_write(0x8b, 0xba);		
		hi253_write(0x8c, 0xad);		
		hi253_write(0x8d, 0xb8);		
		hi253_write(0x8e, 0xae);		
		hi253_write(0x8f, 0xb2);		
		hi253_write(0x90, 0xb3);		
		hi253_write(0x91, 0xb7);		
		hi253_write(0x92, 0x48);		
		hi253_write(0x93, 0x54);		
		hi253_write(0x94, 0x7D);		
		hi253_write(0x95, 0x81);		
		hi253_write(0x96, 0x7D);		
		hi253_write(0x97, 0x81);		
		hi253_write(0xa0, 0x02);		
		hi253_write(0xa1, 0x7B);		
		hi253_write(0xa2, 0x02);		
		hi253_write(0xa3, 0x7B);		
		hi253_write(0xa4, 0x7B);		
		hi253_write(0xa5, 0x02);		
		hi253_write(0xa6, 0x7B);		
		hi253_write(0xa7, 0x02);		
		hi253_write(0xa8, 0x85);		
		hi253_write(0xa9, 0x8C);		
		hi253_write(0xaa, 0x85);		
		hi253_write(0xab, 0x8C);		
		hi253_write(0xac, 0x10); //Rx_pwr_off1_read
		hi253_write(0xad, 0x16); //Rx_pwr_on1_read
		hi253_write(0xae, 0x10); //Rx_pwr_off2_read
		hi253_write(0xaf, 0x16); //Rx_pwr_on1_read
		hi253_write(0xb0, 0x99);		
		hi253_write(0xb1, 0xA3);		
		hi253_write(0xb2, 0xA4);		
		hi253_write(0xb3, 0xAE);		
		hi253_write(0xb4, 0x9B);		
		hi253_write(0xb5, 0xA2);		
		hi253_write(0xb6, 0xA6);		
		hi253_write(0xb7, 0xAC);		
		hi253_write(0xb8, 0x9B);		
		hi253_write(0xb9, 0x9F);		
		hi253_write(0xba, 0xA6);		
		hi253_write(0xbb, 0xAA);		
		hi253_write(0xbc, 0x9B);		
		hi253_write(0xbd, 0x9F);		
		hi253_write(0xbe, 0xA6);		
		hi253_write(0xbf, 0xaa);		
		hi253_write(0xc4, 0x2c);		
		hi253_write(0xc5, 0x43);		
		hi253_write(0xc6, 0x63);		
		hi253_write(0xc7, 0x79);		
		hi253_write(0xc8, 0x2d);		
		hi253_write(0xc9, 0x42);		
		hi253_write(0xca, 0x2d);		
		hi253_write(0xcb, 0x42);		
		hi253_write(0xcc, 0x64);		
		hi253_write(0xcd, 0x78);		
		hi253_write(0xce, 0x64);		
		hi253_write(0xcf, 0x78);		
		hi253_write(0xd0, 0x0a);		
		hi253_write(0xd1, 0x09);		
		hi253_write(0xd2, 0x20);		
		hi253_write(0xd3, 0x00);	
		
		hi253_write(0xd4, 0x0a);		
		hi253_write(0xd5, 0x0a);		
		hi253_write(0xd6, 0xb8);		
		hi253_write(0xd7, 0xb0);
			
		hi253_write(0xe0, 0xc4);		
		hi253_write(0xe1, 0xc4);		
		hi253_write(0xe2, 0xc4);		
		hi253_write(0xe3, 0xc4);		
		hi253_write(0xe4, 0x00);		
		hi253_write(0xe8, 0x80);		
		hi253_write(0xe9, 0x40);		
		hi253_write(0xea, 0x7f);		
		hi253_write(0xf0, 0x01); //sram1_cfg
		hi253_write(0xf1, 0x01); //sram2_cfg
		hi253_write(0xf2, 0x01); //sram3_cfg
		hi253_write(0xf3, 0x01); //sram4_cfg
		hi253_write(0xf4, 0x01); //sram5_cfg

	/////// PAGE 3 ///////
		hi253_write(0x03, 0x03);
		hi253_write(0x10, 0x10);

	/////// PAGE 10 START ///////
		hi253_write(0x03, 0x10);
		hi253_write(0x10, 0x03); // CrYCbY // For Demoset 0x03 03
		hi253_write(0x12, 0x30);
		hi253_write(0x13, 0x0a);//add
		hi253_write(0x20, 0x00);

		hi253_write(0x30, 0x00);
		hi253_write(0x31, 0x00);
		hi253_write(0x32, 0x00);
		hi253_write(0x33, 0x00);

		hi253_write(0x34, 0x30);
		hi253_write(0x35, 0x00);
		hi253_write(0x36, 0x00);
		hi253_write(0x38, 0x00);
		hi253_write(0x3e, 0x58);
		hi253_write(0x3f, 0x00);

		hi253_write(0x40, 0x80); // YOFS
		hi253_write(0x41, 0x10); // DYOFS  //00
		hi253_write(0x48, 0x84); //80 contrast Jerry debug 11-12-6
		hi253_write(0x60, 0x67);
		hi253_write(0x61, 0x90); //7c saturation Jerry debug 11-12-6
		hi253_write(0x62, 0x80); //7c saturation Jerry debug 11-12-6
		hi253_write(0x63, 0x50); //Double_AG 50->30
		hi253_write(0x64, 0x41);

		hi253_write(0x66, 0x42);
		hi253_write(0x67, 0x20);

		hi253_write(0x6a, 0x80); //8a
		hi253_write(0x6b, 0x84); //74
		hi253_write(0x6c, 0x80); //7e //7a
		hi253_write(0x6d, 0x80); //8e

	//Don't touch//////////////////////////
	//	hi253_write(0x72, 0x84);
	//	hi253_write(0x76, 0x19);
	//	hi253_write(0x73, 0x70);
	//	hi253_write(0x74, 0x68);
	//	hi253_write(0x75, 0x60); // white protection ON
	//	hi253_write(0x77, 0x0e); //08 //0a
	//	hi253_write(0x78, 0x2a); //20
	//	hi253_write(0x79, 0x08);
	////////////////////////////////////////

	/////// PAGE 11 START ///////
		hi253_write(0x03, 0x11);
		hi253_write(0x10, 0x7f);
		hi253_write(0x11, 0x40);
		hi253_write(0x12, 0x0a); // Blue Max-Filter Delete
		hi253_write(0x13, 0xbb);

		hi253_write(0x26, 0x31); // Double_AG 31->20
		hi253_write(0x27, 0x34); // Double_AG 34->22
		hi253_write(0x28, 0x0f);
		hi253_write(0x29, 0x10);
		hi253_write(0x2b, 0x30);
		hi253_write(0x2c, 0x32);

	//Out2 D-LPF th
		hi253_write(0x30, 0x70);
		hi253_write(0x31, 0x10);
		hi253_write(0x32, 0x58);
		hi253_write(0x33, 0x09);
		hi253_write(0x34, 0x06);
		hi253_write(0x35, 0x03);

	//Out1 D-LPF th
		hi253_write(0x36, 0x70);
		hi253_write(0x37, 0x18);
		hi253_write(0x38, 0x58);
		hi253_write(0x39, 0x09);
		hi253_write(0x3a, 0x06);
		hi253_write(0x3b, 0x03);

	//Indoor D-LPF th
		hi253_write(0x3c, 0x50);
		hi253_write(0x3d, 0x18);
		hi253_write(0x3e, 0xa0); //80
		hi253_write(0x3f, 0x0a);
		hi253_write(0x40, 0x09);
		hi253_write(0x41, 0x06);

		hi253_write(0x42, 0x80);
		hi253_write(0x43, 0x18);
		hi253_write(0x44, 0xa0); //80
		hi253_write(0x45, 0x12);
		hi253_write(0x46, 0x10);
		hi253_write(0x47, 0x10);

		hi253_write(0x48, 0x90);
		hi253_write(0x49, 0x40);
		hi253_write(0x4a, 0x80);//80
		hi253_write(0x4b, 0x13);
		hi253_write(0x4c, 0x10);
		hi253_write(0x4d, 0x11);

		hi253_write(0x4e, 0x80);
		hi253_write(0x4f, 0x30);
		hi253_write(0x50, 0x80);
		hi253_write(0x51, 0x13);
		hi253_write(0x52, 0x10);
		hi253_write(0x53, 0x13);

		hi253_write(0x54, 0x11);
		hi253_write(0x55, 0x17);
		hi253_write(0x56, 0x20);
		hi253_write(0x57, 0x01);
		hi253_write(0x58, 0x00);
		hi253_write(0x59, 0x00);

		hi253_write(0x5a, 0x1b); //18
		hi253_write(0x5b, 0x00);
		hi253_write(0x5c, 0x00);

		hi253_write(0x60, 0x3f);
		hi253_write(0x62, 0x60);
		hi253_write(0x70, 0x06);

	/////// PAGE 12 START ///////
		hi253_write(0x03, 0x12);
		hi253_write(0x20, 0x0f);
		hi253_write(0x21, 0x0f);

		hi253_write(0x25, 0x00); //0x30

		hi253_write(0x28, 0x00);
		hi253_write(0x29, 0x00);
		hi253_write(0x2a, 0x00);

		hi253_write(0x30, 0x50);
		hi253_write(0x31, 0x18);
		hi253_write(0x32, 0x32);
		hi253_write(0x33, 0x40);
		hi253_write(0x34, 0x50);
		hi253_write(0x35, 0x70);
		hi253_write(0x36, 0xa0);

	//Out2 th
		hi253_write(0x40, 0xa0);
		hi253_write(0x41, 0x40);
		hi253_write(0x42, 0xa0);
		hi253_write(0x43, 0x90);
		hi253_write(0x44, 0x90);
		hi253_write(0x45, 0x80);

	//Out1 th
		hi253_write(0x46, 0xb0);
		hi253_write(0x47, 0x55);
		hi253_write(0x48, 0xa0);
		hi253_write(0x49, 0x90);
		hi253_write(0x4a, 0x90);
		hi253_write(0x4b, 0x80);

	//Indoor th
		hi253_write(0x4c, 0xb0);
		hi253_write(0x4d, 0x40);
		hi253_write(0x4e, 0x90);
		hi253_write(0x4f, 0x90);
		hi253_write(0x50, 0xa0);
		hi253_write(0x51, 0x80);

	//Dark1 th
		hi253_write(0x52, 0xb0);
		hi253_write(0x53, 0x60);
		hi253_write(0x54, 0xc0);
		hi253_write(0x55, 0xc0);
		hi253_write(0x56, 0xc0);
		hi253_write(0x57, 0xe0);//80

	//Dark2 th
		hi253_write(0x58, 0x90);
		hi253_write(0x59, 0x40);
		hi253_write(0x5a, 0xd0);
		hi253_write(0x5b, 0xd0);
		hi253_write(0x5c, 0xe0);
		hi253_write(0x5d, 0xe0);//80

	//Dark3 th
		hi253_write(0x5e, 0x88);
		hi253_write(0x5f, 0x40);
		hi253_write(0x60, 0xe0);
		hi253_write(0x61, 0xe0);
		hi253_write(0x62, 0xe0);
		hi253_write(0x63, 0xe0);//80

		hi253_write(0x70, 0x15);
		hi253_write(0x71, 0x01); //Don't Touch register

		hi253_write(0x72, 0x18);
		hi253_write(0x73, 0x01); //Don't Touch register
		hi253_write(0x90, 0x5d); //DPC
		hi253_write(0x91, 0x88);		
		hi253_write(0x98, 0x7d);		
		hi253_write(0x99, 0x28);		
		hi253_write(0x9A, 0x14);		
		hi253_write(0x9B, 0xc8);		
		hi253_write(0x9C, 0x02);		
		hi253_write(0x9D, 0x1e);		
		hi253_write(0x9E, 0x28);		
		hi253_write(0x9F, 0x07);		
		hi253_write(0xA0, 0x32);		
		hi253_write(0xA4, 0x04);		
		hi253_write(0xA5, 0x0e);		
		hi253_write(0xA6, 0x0c);		
		hi253_write(0xA7, 0x04);		
		hi253_write(0xA8, 0x3c);		

		hi253_write(0xAA, 0x14);		
		hi253_write(0xAB, 0x11);		
		hi253_write(0xAC, 0x0f);		
		hi253_write(0xAD, 0x16);		
		hi253_write(0xAE, 0x15);		
		hi253_write(0xAF, 0x14);		

		hi253_write(0xB1, 0xaa);		
		hi253_write(0xB2, 0x96);		
		hi253_write(0xB3, 0x28);		
	//	hi253_write(0xB6,read); only//dpc_flat_thres
	//	hi253_write(0xB7,read); only//dpc_grad_cnt
		hi253_write(0xB8, 0x78);		
		hi253_write(0xB9, 0xa0);		
		hi253_write(0xBA, 0xb4);		
		hi253_write(0xBB, 0x14);		
		hi253_write(0xBC, 0x14);		
		hi253_write(0xBD, 0x14);		
		hi253_write(0xBE, 0x64);		
		hi253_write(0xBF, 0x64);		
		hi253_write(0xC0, 0x64);		
		hi253_write(0xC1, 0x64);		
		hi253_write(0xC2, 0x04);		
		hi253_write(0xC3, 0x03);		
		hi253_write(0xC4, 0x0c);		
		hi253_write(0xC5, 0x30);		
		hi253_write(0xC6, 0x2a);		
		hi253_write(0xD0, 0x0c); //CI Option/CI DPC
		hi253_write(0xD1, 0x80);		
		hi253_write(0xD2, 0x67);		
		hi253_write(0xD3, 0x00);		
		hi253_write(0xD4, 0x00);		
		hi253_write(0xD5, 0x02);		
		hi253_write(0xD6, 0xff);		
		hi253_write(0xD7, 0x18);	

	/////// PAGE 13 START ///////
		hi253_write(0x03, 0x13);
	//Edge
		hi253_write(0x10, 0xcb);
		hi253_write(0x11, 0x7b);
		hi253_write(0x12, 0x07);//03);//07
		hi253_write(0x14, 0x00);

		hi253_write(0x20, 0x15);//15 Jerry debug 2011-12-06
		hi253_write(0x21, 0x13);//13 Jerry debug 2011-12-06
		hi253_write(0x22, 0x33);
		hi253_write(0x23, 0x05);
		hi253_write(0x24, 0x09);

		hi253_write(0x25, 0x0a);//0a //13 Jerry debug 2011-12-06

		hi253_write(0x26, 0x18);
		hi253_write(0x27, 0x30);
		hi253_write(0x29, 0x12);
		hi253_write(0x2a, 0x50);

	//Low clip th
		hi253_write(0x2b, 0x02);
		hi253_write(0x2c, 0x02);
		hi253_write(0x25, 0x06);//06 Jerry debug 2011-12-06
		hi253_write(0x2d, 0x0c);
		hi253_write(0x2e, 0x12);
		hi253_write(0x2f, 0x12);

	//Out2 Edge
		hi253_write(0x50, 0x18); //0x10 //0x16
		hi253_write(0x51, 0x1c); //0x14 //0x1a
		hi253_write(0x52, 0x1a); //0x12 //0x18
		hi253_write(0x53, 0x14); //0x0c //0x12
		hi253_write(0x54, 0x17); //0x0f //0x15
		hi253_write(0x55, 0x14); //0x0c //0x12

		//Out1 Edge          //Edge
		hi253_write(0x56, 0x18); //0x10 //0x16
		hi253_write(0x57, 0x1c); //0x13 //0x1a
		hi253_write(0x58, 0x1a); //0x12 //0x18
		hi253_write(0x59, 0x14); //0x0c //0x12
		hi253_write(0x5a, 0x17); //0x0f //0x15
		hi253_write(0x5b, 0x14); //0x0c //0x12

	//Indoor Edge
		hi253_write(0x5c, 0x0a);
		hi253_write(0x5d, 0x0b);
		hi253_write(0x5e, 0x0a);
		hi253_write(0x5f, 0x08);
		hi253_write(0x60, 0x09);
		hi253_write(0x61, 0x08);

	//Dark1 Edge
		hi253_write(0x62, 0x08);
		hi253_write(0x63, 0x08);
		hi253_write(0x64, 0x08);
		hi253_write(0x65, 0x06);
		hi253_write(0x66, 0x06);
		hi253_write(0x67, 0x06);

	//Dark2 Edge
		hi253_write(0x68, 0x07);
		hi253_write(0x69, 0x07);
		hi253_write(0x6a, 0x07);
		hi253_write(0x6b, 0x05);
		hi253_write(0x6c, 0x05);
		hi253_write(0x6d, 0x05);

	//Dark3 Edge
		hi253_write(0x6e, 0x07);
		hi253_write(0x6f, 0x07);
		hi253_write(0x70, 0x07);
		hi253_write(0x71, 0x05);
		hi253_write(0x72, 0x05);
		hi253_write(0x73, 0x05);

	//2DY
		hi253_write(0x80, 0xfd);
		hi253_write(0x81, 0x1f);
		hi253_write(0x82, 0x05);
		hi253_write(0x83, 0x31);

		hi253_write(0x90, 0x05);
		hi253_write(0x91, 0x05);
		hi253_write(0x92, 0x33);
		hi253_write(0x93, 0x30);
		hi253_write(0x94, 0x03);
		hi253_write(0x95, 0x14);
		hi253_write(0x97, 0x20);
		hi253_write(0x99, 0x20);

		hi253_write(0xa0, 0x01);
		hi253_write(0xa1, 0x02);
		hi253_write(0xa2, 0x01);
		hi253_write(0xa3, 0x02);
		hi253_write(0xa4, 0x05);
		hi253_write(0xa5, 0x05);
		hi253_write(0xa6, 0x07);
		hi253_write(0xa7, 0x08);
		hi253_write(0xa8, 0x07);
		hi253_write(0xa9, 0x08);
		hi253_write(0xaa, 0x07);
		hi253_write(0xab, 0x08);

	//Out2 
		hi253_write(0xb0, 0x22);
		hi253_write(0xb1, 0x2a);
		hi253_write(0xb2, 0x28);
		hi253_write(0xb3, 0x22);
		hi253_write(0xb4, 0x2a);
		hi253_write(0xb5, 0x28);

	//Out1 
		hi253_write(0xb6, 0x22);
		hi253_write(0xb7, 0x2a);
		hi253_write(0xb8, 0x28);
		hi253_write(0xb9, 0x22);
		hi253_write(0xba, 0x2a);
		hi253_write(0xbb, 0x28);

	//Indoor 
		hi253_write(0xbc, 0x25);
		hi253_write(0xbd, 0x2a);
		hi253_write(0xbe, 0x27);
		hi253_write(0xbf, 0x25);
		hi253_write(0xc0, 0x2a);
		hi253_write(0xc1, 0x27);

	//Dark1
		hi253_write(0xc2, 0x1e);
		hi253_write(0xc3, 0x24);
		hi253_write(0xc4, 0x20);
		hi253_write(0xc5, 0x1e);
		hi253_write(0xc6, 0x24);
		hi253_write(0xc7, 0x20);

	//Dark2
		hi253_write(0xc8, 0x18);
		hi253_write(0xc9, 0x20);
		hi253_write(0xca, 0x1e);
		hi253_write(0xcb, 0x18);
		hi253_write(0xcc, 0x20);
		hi253_write(0xcd, 0x1e);

	//Dark3 
		hi253_write(0xce, 0x18);
		hi253_write(0xcf, 0x20);
		hi253_write(0xd0, 0x1e);
		hi253_write(0xd1, 0x18);
		hi253_write(0xd2, 0x20);
		hi253_write(0xd3, 0x1e);

	/////// PAGE 14 START ///////
		hi253_write(0x03, 0x14);
		hi253_write(0x10, 0x11);

		hi253_write(0x14, 0x80); // GX
		hi253_write(0x15, 0x80); // GY
		hi253_write(0x16, 0x80); // RX
		hi253_write(0x17, 0x80); // RY
		hi253_write(0x18, 0x80); // BX
		hi253_write(0x19, 0x80); // BY

		hi253_write(0x20, 0x60); //X 
		hi253_write(0x21, 0x90); //Y
		hi253_write(0x22, 0x80);
		hi253_write(0x23, 0x73);
		hi253_write(0x24, 0x70);



		hi253_write(0x30, 0xc8);
		hi253_write(0x31, 0x2b);
		hi253_write(0x32, 0x00);
		hi253_write(0x33, 0x00);
		hi253_write(0x34, 0x90);

		hi253_write(0x40, 0x76); // 66 jerry debug 11-12-6
		hi253_write(0x50, 0x6e); // 4e jerry debug 11-12-6
		hi253_write(0x60, 0x6b); // 48 jerry debug 11-12-6
		hi253_write(0x70, 0x6e); // 4e jerry debug 11-12-6
	/////// PAGE 15 START ///////
		hi253_write(0x03, 0x15);
		hi253_write(0x10, 0x0f);

	//Rstep H 16
	//Rstep L 14
		hi253_write(0x14, 0x42); //CMCOFSGH_Day //4c
		hi253_write(0x15, 0x32); //CMCOFSGM_CWF //3c
		hi253_write(0x16, 0x24); //CMCOFSGL_A //2e
		hi253_write(0x17, 0x2f); //CMC SIGN

	//CMC_Default_CWF
		hi253_write(0x30, 0x8f);
		hi253_write(0x31, 0x59);
		hi253_write(0x32, 0x0a);
		hi253_write(0x33, 0x15);
		hi253_write(0x34, 0x5b);
		hi253_write(0x35, 0x06);
		hi253_write(0x36, 0x07);
		hi253_write(0x37, 0x40);
		hi253_write(0x38, 0x87); //86

	//CMC OFS L_A
		hi253_write(0x40, 0x92);
		hi253_write(0x41, 0x1b);
		hi253_write(0x42, 0x89);
		hi253_write(0x43, 0x81);
		hi253_write(0x44, 0x00);
		hi253_write(0x45, 0x01);
		hi253_write(0x46, 0x89);
		hi253_write(0x47, 0x9e);
		hi253_write(0x48, 0x28);

	//	hi253_write(0x40, 0x93);
	//	hi253_write(0x41, 0x1c);
	//	hi253_write(0x42, 0x89);
	//	hi253_write(0x43, 0x82);
	//	hi253_write(0x44, 0x01);
	//	hi253_write(0x45, 0x01);
	//	hi253_write(0x46, 0x8a);
	//	hi253_write(0x47, 0x9d);
	//	hi253_write(0x48, 0x28);

	//CMC POFS H_DAY
		hi253_write(0x50, 0x02);
		hi253_write(0x51, 0x82);
		hi253_write(0x52, 0x00);
		hi253_write(0x53, 0x07);
		hi253_write(0x54, 0x11);
		hi253_write(0x55, 0x98);
		hi253_write(0x56, 0x00);
		hi253_write(0x57, 0x0b);
		hi253_write(0x58, 0x8b);

		hi253_write(0x80, 0x03);
		hi253_write(0x85, 0x40);
		hi253_write(0x87, 0x02);
		hi253_write(0x88, 0x00);
		hi253_write(0x89, 0x00);
		hi253_write(0x8a, 0x00);

	/////// PAGE 16 START ///////
		hi253_write(0x03, 0x16);
		hi253_write(0x10, 0x31);
		hi253_write(0x18, 0x5e);// Double_AG 5e->37
		hi253_write(0x19, 0x5d);// Double_AG 5e->36
		hi253_write(0x1a, 0x0e);
		hi253_write(0x1b, 0x01);
		hi253_write(0x1c, 0xdc);
		hi253_write(0x1d, 0xfe);

	//GMA Default
		hi253_write(0x30, 0x00);
		hi253_write(0x31, 0x0a);
		hi253_write(0x32, 0x1f);
		hi253_write(0x33, 0x33);
		hi253_write(0x34, 0x53);
		hi253_write(0x35, 0x6c);
		hi253_write(0x36, 0x81);
		hi253_write(0x37, 0x94);
		hi253_write(0x38, 0xa4);
		hi253_write(0x39, 0xb3);
		hi253_write(0x3a, 0xc0);
		hi253_write(0x3b, 0xcb);
		hi253_write(0x3c, 0xd5);
		hi253_write(0x3d, 0xde);
		hi253_write(0x3e, 0xe6);
		hi253_write(0x3f, 0xee);
		hi253_write(0x40, 0xf5);
		hi253_write(0x41, 0xfc);
		hi253_write(0x42, 0xff);

		hi253_write(0x50, 0x00);
		hi253_write(0x51, 0x09);
		hi253_write(0x52, 0x1f);
		hi253_write(0x53, 0x37);
		hi253_write(0x54, 0x5b);
		hi253_write(0x55, 0x76);
		hi253_write(0x56, 0x8d);
		hi253_write(0x57, 0xa1);
		hi253_write(0x58, 0xb2);
		hi253_write(0x59, 0xbe);
		hi253_write(0x5a, 0xc9);
		hi253_write(0x5b, 0xd2);
		hi253_write(0x5c, 0xdb);
		hi253_write(0x5d, 0xe3);
		hi253_write(0x5e, 0xeb);
		hi253_write(0x5f, 0xf0);
		hi253_write(0x60, 0xf5);
		hi253_write(0x61, 0xf7);
		hi253_write(0x62, 0xf8);

		hi253_write(0x70, 0x00);
		hi253_write(0x71, 0x08);
		hi253_write(0x72, 0x17);
		hi253_write(0x73, 0x2f);
		hi253_write(0x74, 0x53);
		hi253_write(0x75, 0x6c);
		hi253_write(0x76, 0x81);
		hi253_write(0x77, 0x94);
		hi253_write(0x78, 0xa4);
		hi253_write(0x79, 0xb3);
		hi253_write(0x7a, 0xc0);
		hi253_write(0x7b, 0xcb);
		hi253_write(0x7c, 0xd5);
		hi253_write(0x7d, 0xde);
		hi253_write(0x7e, 0xe6);
		hi253_write(0x7f, 0xee);
		hi253_write(0x80, 0xf4);
		hi253_write(0x81, 0xfa);
		hi253_write(0x82, 0xff);
	/////// PAGE 17 START ///////
		hi253_write(0x03, 0x17);
		hi253_write(0x10, 0xf7);

	/////////PAGE 18 START ///////////
		hi253_write(0x03, 0x00); 
		hi253_write(0x10, 0x11);

		hi253_write(0x03, 0x18);
		hi253_write(0x12, 0x20);
		hi253_write(0x10, 0x07);
		hi253_write(0x11, 0x00);
		hi253_write(0x20, 0x05);
		hi253_write(0x21, 0x50);
		hi253_write(0x22, 0x02);
		hi253_write(0x23, 0x00);
		hi253_write(0x24, 0x00);
		hi253_write(0x25, 0x28);
		hi253_write(0x26, 0x00);
		hi253_write(0x27, 0x10);
		hi253_write(0x28, 0x05);
		hi253_write(0x29, 0x28);
		hi253_write(0x2a, 0x01);
		hi253_write(0x2b, 0xf0);
		hi253_write(0x2c, 0x09);
		hi253_write(0x2d, 0x60);
		hi253_write(0x2e, 0x09);
		hi253_write(0x2f, 0x69);
		hi253_write(0x30, 0x3e);
	//////////////////////////////

	/////// PAGE 20 START ///////
		hi253_write(0x03, 0x20);
		hi253_write(0x11, 0x1c);
		hi253_write(0x18, 0x30);
		hi253_write(0x1a, 0x08);
		hi253_write(0x20, 0x01); //05_lowtemp Y Mean off
		hi253_write(0x21, 0x30);
		hi253_write(0x22, 0x10);
		hi253_write(0x23, 0x00);
		hi253_write(0x24, 0x00); //Uniform Scene Off

		hi253_write(0x28, 0xe7);
		hi253_write(0x29, 0x0d); //20100305 ad->0d
		hi253_write(0x2a, 0xff);
		hi253_write(0x2b, 0xf4); //f4->Adaptive off

		hi253_write(0x2c, 0xc2);
		hi253_write(0x2d, 0xcf);  //ff->AE Speed option
		hi253_write(0x2e, 0x33);
		hi253_write(0x30, 0x78); //f8
		hi253_write(0x32, 0x03);
		hi253_write(0x33, 0x2e);
		hi253_write(0x34, 0x30);
		hi253_write(0x35, 0xd4);
		hi253_write(0x36, 0xfe);
		hi253_write(0x37, 0x32);
		hi253_write(0x38, 0x04);

		hi253_write(0x39, 0x22); //AE_escapeC10
		hi253_write(0x3a, 0xde); //AE_escapeC11

		hi253_write(0x3b, 0x22); //AE_escapeC1
		hi253_write(0x3c, 0xde); //AE_escapeC2

		hi253_write(0x50, 0x45);
		hi253_write(0x51, 0x88);

		hi253_write(0x56, 0x03);
		hi253_write(0x57, 0xf7);
		hi253_write(0x58, 0x14);
		hi253_write(0x59, 0x88);
		hi253_write(0x5a, 0x04);
		//hanwei@wind-mobi.com chg 2012.02.03 start
		//fix zet test yaoban
		//reviewed by liubing@wind-mobi.com
		hi253_write(0x60, 0x00);//55); // AEWGT1
		hi253_write(0x61, 0x00);//55); // AEWGT2
		hi253_write(0x62, 0x00);//6a); // AEWGT3
		hi253_write(0x63, 0x00);//a9); // AEWGT4
		hi253_write(0x64, 0x0f);//6a); // AEWGT5
		hi253_write(0x65, 0xf0);//a9); // AEWGT6
		hi253_write(0x66, 0x0f);//6a); // AEWGT7
		hi253_write(0x67, 0xf0);//a9); // AEWGT8
		hi253_write(0x68, 0x0f);//6b); // AEWGT9
		hi253_write(0x69, 0xf0);//e9); // AEWGT10
		hi253_write(0x6a, 0x0f);//6a); // AEWGT11
		hi253_write(0x6b, 0xf0);//a9); // AEWGT12
		hi253_write(0x6c, 0x00);//6a); // AEWGT13
		hi253_write(0x6d, 0x00);//a9); // AEWGT14
		hi253_write(0x6e, 0x00);//55); // AEWGT15
		hi253_write(0x6f, 0x00);//55); // AEWGT16
		//hanwei@wind-mobi.com chg 2012.02.03 end
		hi253_write(0x70, 0x76); //6e
		hi253_write(0x71, 0x00); //82(+8)->+0

	// haunting control
		hi253_write(0x76, 0x43);
		hi253_write(0x77, 0xe2); //04 //f2
		hi253_write(0x78, 0x23); //Yth1
		hi253_write(0x79, 0x42); //Yth2 //46
		hi253_write(0x7a, 0x23); //23
		hi253_write(0x7b, 0x22); //22
		hi253_write(0x7d, 0x23);

		hi253_write(0x83, 0x01); //EXP Normal 33.33 fps 
		hi253_write(0x84, 0x5f); 
		hi253_write(0x85, 0x00); 

		hi253_write(0x86, 0x02); //EXPMin 5859.38 fps
		hi253_write(0x87, 0x00); 

		hi253_write(0x88, 0x05); 
		hi253_write(0x89, 0x7c); 
		hi253_write(0x8a, 0x00); 

		hi253_write(0x8B, 0x75); //EXP100 
		hi253_write(0x8C, 0x00); 
		hi253_write(0x8D, 0x61); //EXP120 
		hi253_write(0x8E, 0x00); 

		hi253_write(0x9c, 0x18); 
		hi253_write(0x9d, 0x00); 
		hi253_write(0x9e, 0x02); //EXP Unit 
		hi253_write(0x9f, 0x00); 

	//AE_Middle Time option
	//	hi253_write(0xa0, 0x03);
	//	hi253_write(0xa1, 0xa9);
	//	hi253_write(0xa2, 0x80);

		hi253_write(0xb0, 0x18);
		hi253_write(0xb1, 0x14); //ADC 400->560
		hi253_write(0xb2, 0x90); //d0
		hi253_write(0xb3, 0x18);
		hi253_write(0xb4, 0x1a);
		hi253_write(0xb5, 0x44);
		hi253_write(0xb6, 0x2f);
		hi253_write(0xb7, 0x28);
		hi253_write(0xb8, 0x25);
		hi253_write(0xb9, 0x22);
		hi253_write(0xba, 0x21);
		hi253_write(0xbb, 0x20);
		hi253_write(0xbc, 0x1f);
		hi253_write(0xbd, 0x1f);

	//AE_Adaptive Time option
	//	hi253_write(0xc0, 0x10);
		hi253_write(0xc1, 0x1a);
		hi253_write(0xc2, 0x1a);
		hi253_write(0xc3, 0x1a);
		hi253_write(0xc4, 0x10);

		hi253_write(0xc8, 0x80);
		hi253_write(0xc9, 0x40);

	/////// PAGE 22 START ///////
		hi253_write(0x03, 0x22);
		hi253_write(0x10, 0xfd);
		hi253_write(0x11, 0x2e);
		hi253_write(0x19, 0x01); // Low On //
		hi253_write(0x20, 0x30);
		hi253_write(0x21, 0x80);
		hi253_write(0x24, 0x01);
	//	hi253_write(0x25, 0x00); //7f New Lock Cond & New light stable

		hi253_write(0x30, 0x80);
		hi253_write(0x31, 0x80);
		hi253_write(0x38, 0x11);
		hi253_write(0x39, 0x34);

		hi253_write(0x40, 0xf4);
		hi253_write(0x41, 0x55); //44
		hi253_write(0x42, 0x33); //43

		hi253_write(0x43, 0xf6);
		hi253_write(0x44, 0x55); //44
		hi253_write(0x45, 0x44); //33

		hi253_write(0x46, 0x00);
		hi253_write(0x50, 0xb2);
		hi253_write(0x51, 0x81);
		hi253_write(0x52, 0x98);

		hi253_write(0x80, 0x40); //3e
		hi253_write(0x81, 0x20);
		hi253_write(0x82, 0x36);

		hi253_write(0x83, 0x5e); //5e
		hi253_write(0x84, 0x1e); //24
		hi253_write(0x85, 0x5e); //54 //56 //5a
		hi253_write(0x86, 0x22); //24 //22

		hi253_write(0x87, 0x49);
		hi253_write(0x88, 0x39);
		hi253_write(0x89, 0x40); //38
		hi253_write(0x8a, 0x28); //2a

		hi253_write(0x8b, 0x41); //47
		hi253_write(0x8c, 0x39); 
		hi253_write(0x8d, 0x40); 
		hi253_write(0x8e, 0x28); //2c

		hi253_write(0x8f, 0x53); //4e
		hi253_write(0x90, 0x52); //4d
		hi253_write(0x91, 0x51); //4c
		hi253_write(0x92, 0x4e); //4a
		hi253_write(0x93, 0x4a); //46
		hi253_write(0x94, 0x45);
		hi253_write(0x95, 0x3d);
		hi253_write(0x96, 0x31);
		hi253_write(0x97, 0x28);
		hi253_write(0x98, 0x24);
		hi253_write(0x99, 0x20);
		hi253_write(0x9a, 0x20);

		hi253_write(0x9b, 0x77);
		hi253_write(0x9c, 0x77);
		hi253_write(0x9d, 0x48);
		hi253_write(0x9e, 0x38);
		hi253_write(0x9f, 0x30);

		hi253_write(0xa0, 0x60);
		hi253_write(0xa1, 0x34);
		hi253_write(0xa2, 0x6f);
		hi253_write(0xa3, 0xff);

		hi253_write(0xa4, 0x14); //1500fps
		hi253_write(0xa5, 0x2c); // 700fps
		hi253_write(0xa6, 0xcf);

		hi253_write(0xad, 0x40);
		hi253_write(0xae, 0x4a);

		hi253_write(0xaf, 0x28);  // low temp Rgain
		hi253_write(0xb0, 0x26);  // low temp Rgain

		hi253_write(0xb1, 0x00); //0x20 -> 0x00 0405 modify
		hi253_write(0xb4, 0xea);
		hi253_write(0xb8, 0xa0); //a2: b-2, R+2  //b4 B-3, R+4 lowtemp
		hi253_write(0xb9, 0x00);

	/////// PAGE 20 ///////
		hi253_write(0x03, 0x20);
		hi253_write(0x10, 0x8c);

	// PAGE 20
		hi253_write(0x03, 0x20); //page 20
		hi253_write(0x10, 0x9c); //ae off

	// PAGE 22
		hi253_write(0x03, 0x22); //page 22
		hi253_write(0x10, 0xe9); //awb off

	// PAGE 0
		hi253_write(0x03, 0x00);
		hi253_write(0x0e, 0x03); //PLL On
		hi253_write(0x0e, 0x73); //PLLx2

		hi253_write(0x03, 0x00); // Dummy 750us
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
		hi253_write(0x03, 0x00);
	   
		hi253_write(0x03, 0x00); // Page 0
		hi253_write(0x01, 0xf0); // Sleep Off
		hi253_write(0xff, 0xff);
		hi253_write(0xff, 0xff);

	tm1 = ktime_get();
	pr_info("Exit Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	return result;
}
//hanwei@wind-mobi.com 2011.11.02 init hi253 parameters end


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
