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

#define S5K5CA_ID 0x05CA

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
/* Resolutions & Sizes available for MT9T111 ISP/Sensor (QXGA max) */
static CamResolutionConfigure_t sSensorResInfo_MT9T111_st[] = {
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
    {GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetLow},
    //{PAUSE, 10, Nop_Cmd},
    
// -------Turn everything OFF   
    //{GPIO_CNTRL, HAL_CAM_RESET,    GPIO_SetLow},
    {MCLK_CNTRL, CamDrv_NO_CLK,    CLK_TurnOff},
 
    {GPIO_CNTRL, HAL_CAM_PWDN,       GPIO_SetLow},

    {PAUSE, 1, Nop_Cmd},
    //{PAUSE,      50,               Nop_Cmd},
// -------Disable Reset 
    //{GPIO_CNTRL, HAL_CAM_RESET,    GPIO_SetHigh},
// -------Enable Clock to Cameras @ Main clock speed
    {MCLK_CNTRL, CamDrv_12MHz,     CLK_TurnOn},  //Mclock
    {PAUSE,      1,                Nop_Cmd},
 
     {GPIO_CNTRL, HAL_CAM_PWDN,    GPIO_SetHigh},
    
    {PAUSE,      1,                Nop_Cmd},
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	{PAUSE,      10,                Nop_Cmd},
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
//songjinguo@wind-mobi.com 20120301 start
//fix standby larger current
//review by liubing
    //{GPIO_CNTRL, HAL_CAM_PWDN,       GPIO_SetLow},
    {GPIO_CNTRL, HAL_CAM_PWDN,       GPIO_SetHigh},
//songjinguo@wind-mobi.com 20120301 end    
    
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
static HAL_CAM_Result_en_t Init_Mt9t111(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt16 mt9t111_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t mt9t111_write(unsigned int sub_addr, UInt16 data);
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
			    ("Error in allocating %d bytes for MT9T111 sensor\n",
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

	result = Init_Mt9t111(sensor);
	printk("Init_Mt9t111 result =%d\r\n", result);
	return result;
}

static UInt16 CAMDRV_GetDeviceID_Pri(CamSensorSelect_t sensor)
{
	mt9t111_write(0x002C,0x0000);
	mt9t111_write(0x002E,0x0040);
	return mt9t111_read(0x0F12);
}

static int checkCameraID(CamSensorSelect_t sensor)
{
	UInt16 devId = CAMDRV_GetDeviceID_Pri(sensor);

	if (devId == S5K5CA_ID) {
		printk("Camera identified as S5K5CA  devId 0x%x \r\n", devId);
		return 0;
	} else {
		printk("Camera Id wrong. Expected 0x%x but got 0x%x\r\n",
			 S5K5CA_ID, devId);
		return -1;
	}
}

static HAL_CAM_Result_en_t mt9t111_write(unsigned int sub_addr, UInt16 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 bytes[2];
	bytes[0] = (data & 0xFF00) >> 8;
	bytes[1] = data & 0xFF;

	result |= CAM_WriteI2c(sub_addr, 2, bytes);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9t111_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
	return result;
}

static UInt16 mt9t111_read(unsigned int sub_addr)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt16 data;
	UInt16 temp;

	result |= CAM_ReadI2c(sub_addr, 2, (UInt8 *) &data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9t111_read(): ERROR: %d\r\n", result);
	}

	temp = data;
	data = ((temp & 0xFF) << 8) | ((temp & 0xFF00) >> 8);

	return data;
}

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format)
{
	printk("L400_Back_Camera >>>>>>SensorSetPreviewMode image_resolution 0x%x image_format %d  ",image_resolution,image_format  );
	
		mt9t111_write( 0x0028, 0x7000);                                                                                                                                                                                                                          
		mt9t111_write( 0x002A, 0x0254 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0000 );	// MCU_DATA_0
              mdelay(133);
		mt9t111_write( 0x002A, 0x0252); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0004); 	// MCU_DATA_0
		mdelay(200);                                                                                                                                                                                                                

		mt9t111_write( 0x002A, 0x023C);                                                                                                     
		mt9t111_write( 0x0F12, 0x0000);//0000 //REG_TC_GP_ActivePrevConfig                                                                                                                                                                                        
		mt9t111_write( 0x002A, 0x0240);                                                                                                     
		mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_PrevOpenAfterChange                                                                                                                                                                                          
		mt9t111_write( 0x002A, 0x0230);                                                                                                     
		mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_NewConfigSync                                                                                                                                                                                                 
		mt9t111_write( 0x002A, 0x023E);                                                                                                     
		mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_PrevConfigChanged                                                                                                                                                                                             
		mt9t111_write( 0x002A, 0x0220);                                                                                                     
		mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_EnablePreview                                                                                                                                                                                                 
		mt9t111_write( 0x0028, 0xD000);                                                                                                     
		mt9t111_write( 0x002A, 0xB0A0);                                                                                                     
		mt9t111_write( 0x0F12, 0x0000); // Clear cont. clock befor config change                                                                                                                                                                                  
		mt9t111_write( 0x0028, 0x7000);                                                                                                     
		mt9t111_write( 0x002A, 0x0222);                                                                                                     
		mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_EnablePreviewChanged 
		
		return HAL_CAM_SUCCESS;

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
		printk("CAMDRV_SetFrameRate_Pri    = 0  \r\n" );
		mt9t111_write(0xFCFC, 0xD000);
		mt9t111_write(0x0028, 0x7000);
		mt9t111_write(0x002A, 0x0F70);
		mt9t111_write(0x0F12, 0x003F); 
		mt9t111_write(0x002A, 0x020C);
		mt9t111_write(0x0F12, 0x0000);  
	}
	else if(value>0)
	{
		printk("CAMDRV_SetFrameRate_Pri    > 0  \r\n" );
		mt9t111_write(0xFCFC, 0xD000);
		mt9t111_write(0x0028, 0x7000);
		mt9t111_write(0x002A, 0x0F70);
		mt9t111_write(0x0F12, 0x0046); 
		mt9t111_write(0x002A, 0x020C);
		mt9t111_write(0x0F12, 0x0060);  
	}
	else
	{
		printk("CAMDRV_SetFrameRate_Pri    < 0  \r\n" );
		mt9t111_write(0xFCFC, 0xD000);
		mt9t111_write(0x0028, 0x7000);
		mt9t111_write(0x002A, 0x0F70);
		mt9t111_write(0x0F12, 0x002F); 
		mt9t111_write(0x002A, 0x020C);
		mt9t111_write(0x0F12, 0xFF94);  
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

	return HAL_CAM_SUCCESS;
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
				
				mt9t111_write(0x98E, 0x68AA);	//TX FIFO Watermark (A)
				mt9t111_write(0x990, 0x0218);	//		= 536
				mt9t111_write(0x98E, 0x6815);	//Max FD Zone 50 Hz
				mt9t111_write(0x990, 0x0007);	//		= 7
				mt9t111_write(0x98E, 0x6817 );//Max FD Zone 60 Hz
				mt9t111_write(0x990, 0x0008 );//	  = 8
				mt9t111_write(0x98E, 0x682D);	//AE Target FD Zone
				mt9t111_write(0x990, 0x0007);	//		= 7
				
				mt9t111_write( 0x098E, 0x8400); 	// MCU_ADDRESS [SEQ_CMD]
				mt9t111_write( 0x0990, 0x0006); 	// MCU_DATA_0
			}
			else if(fps>=CamRate_10)
			{
				mt9t111_write(0x98E, 0x68AA);	//TX FIFO Watermark (A)
				mt9t111_write(0x990, 0x026A);	//      = 618
				mt9t111_write(0x98E, 0x6815);	//Max FD Zone 50 Hz
				mt9t111_write(0x990, 0x000A);	//      = 10
				mt9t111_write(0x98E, 0x6817);	//Max FD Zone 60 Hz
				mt9t111_write(0x990, 0x000C);	//      = 12
				mt9t111_write(0x98E, 0x682D);	//AE Target FD Zone
				mt9t111_write(0x990, 0x000A);	//      = 10

				mt9t111_write( 0x098E, 0x8400); 	// MCU_ADDRESS [SEQ_CMD]
				mt9t111_write( 0x0990, 0x0006); 	// MCU_DATA_0
			}
			else if(fps>=CamRate_5)
			{
				mt9t111_write(0x98E, 0x68AA);	//TX FIFO Watermark (A)
				mt9t111_write(0x990, 0x0218);	//       = 536
				mt9t111_write(0x98E, 0x6815);	//Max FD Zone 50 Hz
				mt9t111_write(0x990, 0x0014);	//       = 20
				mt9t111_write(0x98E, 0x6817);	//Max FD Zone 60 Hz
				mt9t111_write(0x990, 0x0018);	//       = 24
				mt9t111_write(0x98E, 0x682D);	//AE Target FD Zone
				mt9t111_write(0x990, 0x0014);	//      = 20

				mt9t111_write( 0x098E, 0x8400); 	// MCU_ADDRESS [SEQ_CMD]
				mt9t111_write( 0x0990, 0x0006); 	// MCU_DATA_0
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
{return HAL_CAM_SUCCESS;
	/*[Enable stream] */
	printk(KERN_ERR"9t111 CAMDRV_EnableVideoCapture \r\n");
	mt9t111_write(0x001A, 0x0218);
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
	return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	UInt16 readout;
	printk("CAMDRV_SetCamSleep(): mt9t111 enter soft standby mode \r\n");
	
	readout=mt9t111_read(0x0028);	/* MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN] */
	readout&=0xFFFE;
	mt9t111_write(0x0028,readout);
	
	readout=mt9t111_read(0x0018);
	readout |= 0x1;
	mt9t111_write(0x0018,readout);
	msleep(3);
	
	readout=mt9t111_read(0x0018);
	if(!(readout&0x4000)){
		printk("failed: CAMDRV_SetCamSleep_Pri(): mt9t111 enter soft standby mode   \r\n");
		result=HAL_CAM_ERROR_OTHERS;
	}
	
	return result;
}

/*camera_disable for still mode */
static HAL_CAM_Result_en_t CAMDRV_DisableCapture_Pri(CamSensorSelect_t sensor)
{
	
	/*[Disable stream] */
	//printk(KERN_ERR"in 9t111 CAMDRV_DisableCapture \r\n");
	control_flash(0);

	/*[Preview on] */
	mt9t111_write(0x098E, 0xEC05);	/* MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN] */
	mt9t111_write(0x0990, 0x0005);	/* MCU_DATA_0 */
	mt9t111_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	mt9t111_write(0x0990, 0x0001);	/* MCU_DATA_0 */

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
	mt9t111_write(0x001A, 0x0018);

	printk("mt9t111 CAMDRV_DisablePreview(): \r\n");
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
	UInt32 x = 0, y = 0;
	UInt32 tx = 0, ty = 0;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	if(flash_switch==Flash_On)
		control_flash(1);

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

mt9t111_write( 0x0028, 0x7000);
mt9t111_write( 0x002A, 0x035C);   //Normal capture //
mt9t111_write( 0x0F12, 0x0000);   //REG_0TC_CCFG_uCaptureMode 2   7000035C	
mt9t111_write( 0x0F12, x);   //REG_0TC_CCFG_usWidth      2   7000035E 
mt9t111_write( 0x0F12, y);   //REG_0TC_CCFG_usHeight     2   70000360

mt9t111_write( 0x002A, 0x0208);
mt9t111_write( 0x0F12, 0x0001);   //REG_TC_IPRM_InitParamsUpdated//

mt9t111_write( 0x0028, 0x7000);
mt9t111_write( 0x002A, 0x0244);
mt9t111_write( 0x0F12, 0x0000); //REG_TC_GP_ActiveCapConfig

mt9t111_write( 0x002A, 0x0230);
mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_NewConfigSync

mt9t111_write( 0x002A, 0x0246);
mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_CapConfigChanged

mt9t111_write( 0x002A, 0x0224);
mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_EnableCapture

mt9t111_write( 0x0028, 0xD000);
mt9t111_write( 0x002A, 0xB0A0);
mt9t111_write( 0x0F12, 0x0000); // Clear cont. clock befor config change

mt9t111_write( 0x0028, 0x7000);
mt9t111_write( 0x002A, 0x0226);//0224
mt9t111_write( 0x0F12, 0x0001); //REG_TC_GP_EnableCaptureChanged

mdelay(200);

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
	pr_info("*******CAMDRV_SetSceneMode() called  ***** \n");
	
	switch(scene_mode) {

			pr_info("CAMDRV_SetSceneMode() called for AUTO\n");
			
			mt9t111_write(0xFCFC,0xD000);                                                                                                                                   
			mt9t111_write(0x0028,0x7000);                                                                                                                                                                                                                                                                 
			mt9t111_write(0x002A,0x0288);                                                                                                                                   
			mt9t111_write(0x0F12,0x0535);//REG_0TC_PCFG_usMaxFrTimeMsecMult10                                                                                              
			mt9t111_write(0x0F12,0x01F4);//REG_0TC_PCFG_usMinFrTimeMsecMult10                                                                                               
			                                                                                                                  
			mt9t111_write(0x002A,0x037A);                                                                                                                                   
			mt9t111_write(0x0F12,0x0535);//REG_0TC_CCFG_usMaxFrTimeMsecMult10                                                                                               
			mt9t111_write(0x0F12,0x0535);//REG_0TC_CCFG_usMinFrTimeMsecMult10                                                                                                                                                                                                      
			                                                                                                                  
			mt9t111_write(0x002A,0x0208);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);  //REG_TC_IPRM_InitParamsUpdated//                                                                                                
			                                                                                                                  	                                                                                                                  
			mt9t111_write(0x002A,0x0530);//lt_uMaxExp1//                                                                                                                     
			mt9t111_write(0x0F12,0x3415);                                                                                                                                   
			mt9t111_write(0x002A,0x0534);//lt_uMaxExp2//                                                                                                                     
			mt9t111_write(0x0F12,0x682A);                                                                                                                                   
			mt9t111_write(0x002A,0x167C);//lt_uMaxExp3//                                                                                                                     
			mt9t111_write(0x0F12,0x8235);                                                                                                                                   
			mt9t111_write(0x002A,0x1680);//lt_uMaxExp4//                                                                                                                     
			mt9t111_write(0x0F12,0x8235);                                                                                                                                   
			mt9t111_write(0x0F12,0x0000);                                                                                                                                   
			                                                                                                                  
			mt9t111_write(0x002A,0x0538);//It_uCapMaxExp1 //                                                                                                                 
			mt9t111_write(0x0F12,0x3415);                                                                                                                                   
			mt9t111_write(0x002A,0x053C);//It_uCapMaxExp2 //                                                                                                                 
			mt9t111_write(0x0F12,0x682A);                                                                                                                                   
			mt9t111_write(0x002A,0x1684);//It_uCapMaxExp3 //                                                                                                                 
			mt9t111_write(0x0F12,0x8235);                                                                                                                                   
			mt9t111_write(0x002A,0x1688);//It_uCapMaxExp4 //                                                                                                                 
			mt9t111_write(0x0F12,0xC350);                                                                                                                                   
			mt9t111_write(0x0F12,0x0000);                                                                                                                                   
			                                                                                                                  
			mt9t111_write(0x002A,0x0540);                                                                                                                                   
			mt9t111_write(0x0F12,0x01B3);//It_uMaxAnGain1 //                                                                                                                 
			mt9t111_write(0x0F12,0x0350);//It_uMaxAnGain2 //                                                                                                                 
			mt9t111_write(0x002A,0x168C);                                                                                                                                   
			mt9t111_write(0x0F12,0x0400);//It_uMaxAnGain3 //                                                                                                                 
			mt9t111_write(0x0F12,0x0710);//0800//It_uMaxAnGain4 //                                                                                                           
			                                                                                                                  
			mt9t111_write(0x002A,0x0544);                                                                                                                                   
			mt9t111_write(0x0F12,0x0100);//It_uMaxDigGain //                                                                                                                 
			                                                                                                                                                                                                                                               
			mt9t111_write(0x002A,0x023C);                                                                                                                                   
			mt9t111_write(0x0F12,0x0000); //night preview config 0//                                                                                                        
			mt9t111_write(0x002A,0x0240);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);                                                                                                                                   
			mt9t111_write(0x002A,0x0230);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);                                                                                                                                   
			mt9t111_write(0x002A,0x023E);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);                                                                                                                                   
			mt9t111_write(0x002A,0x0220);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);  
			
			mt9t111_write( 0x0028, 0xD000);                                                                                                     
			mt9t111_write( 0x002A, 0xB0A0);                                                                                                     
			mt9t111_write( 0x0F12, 0x0000); // Clear cont. clock befor config change                                                                                                                                                                                  
			mt9t111_write( 0x0028, 0x7000);                                                                                                     
			mt9t111_write( 0x002A, 0x0222);    
			mt9t111_write(0x0F12,0x0001);
			break;
		case CamSceneMode_Night:
			pr_info("CAMDRV_SetSceneMode() called for Night\n");

			mt9t111_write(0xFCFC,0xD000);                                                                                                                                   
			mt9t111_write(0x0028,0x7000);                                                                                                                                                                                                                                                                 
			mt9t111_write(0x002A,0x0288);                                                                                                                                   
			mt9t111_write(0x0F12,0x09C4);//REG_0TC_PCFG_usMaxFrTimeMsecMult10                                                                                              
			mt9t111_write(0x0F12,0x029A);//REG_0TC_PCFG_usMinFrTimeMsecMult10                                                                                               
			                                                                                                              
			mt9t111_write(0x002A,0x037A);                                                                                                                                   
			mt9t111_write(0x0F12,0x1388);//REG_0TC_CCFG_usMaxFrTimeMsecMult10                                                                                               
			mt9t111_write(0x0F12,0x1388);//REG_0TC_CCFG_usMinFrTimeMsecMult10                                                                                                                                                                                                      
			                                                                                                              
			mt9t111_write(0x002A,0x0208);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);  //REG_TC_IPRM_InitParamsUpdated//                                                                                                
			                                                                                                              
			                                                                                                              
			mt9t111_write(0x002A,0x0530);//lt_uMaxExp1//                                                                                                                     
			mt9t111_write(0x0F12,0x3415);                                                                                                                                   
			mt9t111_write(0x002A,0x0534);//lt_uMaxExp2//                                                                                                                     
			mt9t111_write(0x0F12,0x682A);                                                                                                                                   
			mt9t111_write(0x002A,0x167C);//lt_uMaxExp3//                                                                                                                     
			mt9t111_write(0x0F12,0x8235);                                                                                                                                   
			mt9t111_write(0x002A,0x1680);//lt_uMaxExp4//                                                                                                                     
			mt9t111_write(0x0F12,0x1A80);                                                                                                                                   
			mt9t111_write(0x0F12,0x0006);                                                                                                                                   
			                                                                                                              
			mt9t111_write(0x002A,0x0538);//It_uCapMaxExp1 //                                                                                                                 
			mt9t111_write(0x0F12,0x3415);                                                                                                                                   
			mt9t111_write(0x002A,0x053C);//It_uCapMaxExp2 //                                                                                                                 
			mt9t111_write(0x0F12,0x682A);                                                                                                                                   
			mt9t111_write(0x002A,0x1684);//It_uCapMaxExp3 //                                                                                                                 
			mt9t111_write(0x0F12,0x8235);                                                                                                                                   
			mt9t111_write(0x002A,0x1688);//It_uCapMaxExp4 //                                                                                                                 
			mt9t111_write(0x0F12,0x1A80);                                                                                                                                   
			mt9t111_write(0x0F12,0x0006);                                                                                                                                   
			                                                                                                              
			mt9t111_write(0x002A,0x0540);                                                                                                                                   
			mt9t111_write(0x0F12,0x0180);//It_uMaxAnGain1 //                                                                                                                 
			mt9t111_write(0x0F12,0x0250);//It_uMaxAnGain2 //                                                                                                                 
			mt9t111_write(0x002A,0x168C);                                                                                                                                   
			mt9t111_write(0x0F12,0x0340);//It_uMaxAnGain3 //                                                                                                                 
			mt9t111_write(0x0F12,0x0700);//0800//It_uMaxAnGain4 //                                                                                                           
			                                                                                                              
			mt9t111_write(0x002A,0x0544);                                                                                                                                   
			mt9t111_write(0x0F12,0x0100);//It_uMaxDigGain //                                                                                                                 
			                                                                                                                                                                                                                                           
			mt9t111_write(0x002A,0x023C);                                                                                                                                   
			mt9t111_write(0x0F12,0x0000); //night preview config 0//                                                                                                        
			mt9t111_write(0x002A,0x0240);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);                                                                                                                                   
			mt9t111_write(0x002A,0x0230);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);                                                                                                                                   
			mt9t111_write(0x002A,0x023E);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);                                                                                                                                   
			mt9t111_write(0x002A,0x0220);                                                                                                                                   
			mt9t111_write(0x0F12,0x0001);
			
			mt9t111_write( 0x0028, 0xD000);                                                                                                     
			mt9t111_write( 0x002A, 0xB0A0);                                                                                                     
			mt9t111_write( 0x0F12, 0x0000); // Clear cont. clock befor config change                                                                                                                                                                                  
			mt9t111_write( 0x0028, 0x7000);                                                                                                     
			mt9t111_write( 0x002A, 0x0222);    
			mt9t111_write(0x0F12,0x0001);
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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetWBMode()  called  wb_mode = 0x%x \n",wb_mode);

	switch (wb_mode)
	{
		case CamWB_Auto:	
			printk("CamWB_Auto\r\n");
			
			//[WB_auto]
			mt9t111_write(0xFCFC, 0xD000);                    
			mt9t111_write(0x0028, 0x7000);                    
			mt9t111_write(0x002A, 0x246E);                    
			mt9t111_write(0x0F12, 0x0001);  
			break;	
		case CamWB_Daylight:	//sunny	
			printk("CamWB_Daylight\r\n");	
			
			//[WB_daylight]
			mt9t111_write(0xFCFC, 0xD000);                    
			mt9t111_write(0x0028, 0x7000);                    
			mt9t111_write(0x002A, 0x246E);                    
			mt9t111_write(0x0F12, 0x0000);                    
			mt9t111_write(0x002A, 0x04A0);                    
			mt9t111_write(0x0F12, 0x0600);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0400);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0526);                    
			mt9t111_write(0x0F12, 0x0001);        
			break;		
		case CamWB_Fluorescent:			
			printk("CamWB_Fluorescent\r\n");
			
			//[WB_flourescant]
			mt9t111_write(0xFCFC, 0xD000);                    
			mt9t111_write(0x0028, 0x7000);                    
			mt9t111_write(0x002A, 0x246E);                    
			mt9t111_write(0x0F12, 0x0000);                    
			mt9t111_write(0x002A, 0x04A0);                    
			mt9t111_write(0x0F12, 0x0555);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0400);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x07F6);                    
			mt9t111_write(0x0F12, 0x0001);  
			break;
		case CamWB_Cloudy:		
			printk("CamWB_Cloudy\r\n");
			
			//[WB_cloudy]
			mt9t111_write(0xFCFC, 0xD000);                    
			mt9t111_write(0x0028, 0x7000);                    
			mt9t111_write(0x002A, 0x246E);                    
			mt9t111_write(0x0F12, 0x0000);                    
			mt9t111_write(0x002A, 0x04A0);                    
			mt9t111_write(0x0F12, 0x0700);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0400);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0485);                    
			mt9t111_write(0x0F12, 0x0001);   
			break;
		case CamWB_Incandescent:	//home	
			printk("CamWB_Incandescent\r\n");
			
			//[WB_incandescent A LIGHT]
			mt9t111_write(0xFCFC, 0xD000);                    
			mt9t111_write(0x0028, 0x7000);                    
			mt9t111_write(0x002A, 0x246E);                    
			mt9t111_write(0x0F12, 0x0000);                    
			mt9t111_write(0x002A, 0x04A0);                    
			mt9t111_write(0x0F12, 0x03E0);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0400);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0910);                    
			mt9t111_write(0x0F12, 0x0001);            
			break;	
		case CamWB_Twilight:
			printk("CamWB_Twilight\r\n");
			
			mt9t111_write(0xFCFC, 0xD000);                    
			mt9t111_write(0x0028, 0x7000);                    
			mt9t111_write(0x002A, 0x246E);                    
			mt9t111_write(0x0F12, 0x0000);                    
			mt9t111_write(0x002A, 0x04A0);                    
			mt9t111_write(0x0F12, 0x0555);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0400);                    
			mt9t111_write(0x0F12, 0x0001);                    
			mt9t111_write(0x0F12, 0x0810);   //0x07F6}                 
			mt9t111_write(0x0F12, 0x0001);  
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
	
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) {
		mt9t111_write(0xFCFC, 0xD000);                 
		mt9t111_write(0x0028, 0x7000);                 
		mt9t111_write(0x002A, 0x04D2);                 
		mt9t111_write(0x0F12, 0x067F);                 
		mt9t111_write(0x002A, 0x04BA);                
		mt9t111_write(0x0F12, 0x0001);                 
		mt9t111_write(0x0F12, 0x0001);

	} else if (effect == CamAntiBanding50Hz) {

		//Band 50Hz   
		mt9t111_write(0xFCFC, 0xD000);                
		mt9t111_write(0x0028, 0x7000);                 
		mt9t111_write(0x002A, 0x04D2);                
		mt9t111_write(0x0F12, 0x065F);                 
		mt9t111_write(0x002A, 0x04BA);                 
		mt9t111_write(0x0F12, 0x0001);                 
		mt9t111_write(0x0F12, 0x0001);

	} else if (effect == CamAntiBanding60Hz) {

		//Band 60Hz   
		mt9t111_write(0xFCFC, 0xD000);                 
		mt9t111_write(0x0028, 0x7000);                
		mt9t111_write(0x002A, 0x04D2);                 
		mt9t111_write(0x0F12, 0x065F);                
		mt9t111_write(0x002A, 0x04BA);                 
		mt9t111_write(0x0F12, 0x0002);                 
		mt9t111_write(0x0F12, 0x0001);

	} else {
	//Auto
		mt9t111_write(0xFCFC, 0xD000);                 
		mt9t111_write(0x0028, 0x7000);                 
		mt9t111_write(0x002A, 0x04D2);                 
		mt9t111_write(0x0F12, 0x067F);                 
		mt9t111_write(0x002A, 0x04BA);                
		mt9t111_write(0x0F12, 0x0001);                 
		mt9t111_write(0x0F12, 0x0001);
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
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	printk("CAMDRV_SetFocusMode()  called effect 0x%x \n",effect);
	if (effect == CamFocusControlAuto) {
		printk("L400_Back_Camera >>>>>>   CamFocusControlAuto    \n");
		
		mt9t111_write( 0xFCFC, 0xD000); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0028, 0x7000); 	// MCU_DATA_0
		mt9t111_write( 0x002A, 0x0254 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0000 );	// MCU_DATA_0
              mdelay(133);
		mt9t111_write( 0x002A, 0x0252); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0004); 	// MCU_DATA_0
		mdelay(200);
		mt9t111_write( 0x002A, 0x10D4 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x1000 );	// MCU_DATA_0
		
	} else if (effect == CamFocusControlMacro) {
	
		printk("L400_Back_Camera >>>>>>   CamFocusControlMacro    \n");
	
		mt9t111_write( 0xFCFC, 0xD000); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0028, 0x7000); 	// MCU_DATA_0
		mt9t111_write( 0x002A, 0x0254 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x00D0 );	// MCU_DATA_0
              mdelay(133);
		mt9t111_write( 0x002A, 0x0252); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0004); 	// MCU_DATA_0
		mdelay(200);
		mt9t111_write( 0x002A, 0x10D4 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x1040 );	// MCU_DATA_0
		mt9t111_write( 0x002A, 0x1066);	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x1300 );	// MCU_DATA_0

	} else if (effect == CamFocusControlInfinity) {
	
		printk("L400_Back_Camera >>>>>>   CamFocusControlInfinity    \n");
	
		mt9t111_write( 0xFCFC, 0xD000); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0028, 0x7000); 	// MCU_DATA_0
		mt9t111_write( 0x002A, 0x0254 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0000 );	// MCU_DATA_0
              mdelay(133);
		mt9t111_write( 0x002A, 0x0252); 	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0004); 	// MCU_DATA_0
		mdelay(200);
		mt9t111_write( 0x002A, 0x0252 );	// MCU_ADDRESS [AF_ALGO]
		mt9t111_write( 0x0F12, 0x0002);	// MCU_DATA_0


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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
		printk("L400_Back_Camera >>>>>>   CAMDRV_TurnOnAF_Pri      called \n");
		UInt16 afStatus ;
	switch (FOCUS_MODE)
	{
		
		case CamFocusControlAuto: /*AF trigger*/
			printk("L400_Back_Camera >>>>>>   CAMDRV_TurnOnAF_Pri      CamFocusControlAuto   called \n");
			mt9t111_write( 0xFCFC, 0xD000); 	// MCU_ADDRESS [AF_ALGO]
			mt9t111_write( 0x0028, 0x7000); 	// MCU_DATA_0
			mt9t111_write( 0x002A, 0x0252 );	// MCU_ADDRESS [AF_ALGO]
			mt9t111_write( 0x0F12, 0x0005 );	// MCU_DATA_0

			break;

		case CamFocusControlMacro:/*we need not doing trigger for this mode */
			printk("L400_Back_Camera >>>>>>   CAMDRV_TurnOnAF_Pri      CamFocusControlMacro   called \n");
			mt9t111_write( 0xFCFC, 0xD000); 	// MCU_ADDRESS [AF_ALGO]
			mt9t111_write( 0x0028, 0x7000); 	// MCU_DATA_0
			mt9t111_write( 0x002A, 0x0252 );	// MCU_ADDRESS [AF_ALGO]
			mt9t111_write( 0x0F12, 0x0005 );	// MCU_DATA_0
			break;

		case CamFocusControlInfinity:/*we need not doing trigger for this mode */
			printk("L400_Back_Camera >>>>>>   CAMDRV_TurnOnAF_Pri      CamFocusControlInfinity   called \n");
			msleep(100);
			break;

		default:
			printk(KERN_ERR"error in CAMDRV_TurnOnAF_Pri , can not support focus mode 0x%x \n",FOCUS_MODE);
				
	}
	#if 0
			mdelay(266);
			do{
				mdelay(100);
				mt9t111_write( 0x002C, 0x7000 );	
				mt9t111_write( 0x002E, 0x26FE );	
				afStatus = mt9t111_read(0x0F12);
			//	printk("L400_Back_Camera >>>>>>   CAMDRV_TurnOnAF_Pri      mt9t111_read(0F12) = 0x%x  \n",afStatus);
				}while((afStatus == 0002)||(afStatus == 0003));
			printk("L400_Back_Camera >>>>>>   CAMDRV_TurnOnAF_Pri      mt9t111_read(0F12) = 0x%x  \n",afStatus);
	#endif		

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
	
	switch (effect)
	{	
		case CamDigEffect_NoEffect:		
			printk(" CamDigEffect_NoEffect \r\n");
			//<CAMTUNING_EFFECT_OFF>
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x3286);
			mt9t111_write(0x0F12,0x0001); //Pre/Post gamma on
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0000); //Normal mode
			break;	
			
		case CamDigEffect_SepiaGreen:	//antique
			printk(" CamDigEffect_SepiaGreen \r\n");
			//<CAMTUNING_EFFECT_SEPIA>
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0004);
			break;	
			
		case CamDigEffect_MonoChrome:	//B&W	
			printk(" CamDigEffect_MonoChrome \r\n");
			//<CAMTUNING_EFFECT_MONO>
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0001);
			break;	
			
		case CamDigEffect_NegColor:		//Negative
			printk(" CamDigEffect_NegColor \r\n");
			//<CAMTUNING_EFFECT_NEGATIVE>
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0003);
			break;
			
		case CamDigEffect_Auqa:
			printk(" CamDigEffect_Auqa \r\n");
			//<CAMTUNING_EFFECT_AQUA>
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0005);
			break;
			
		case CamDigEffect_SolarizeColor:
		case CamDigEffect_SolarizeMono:
			printk(" CamDigEffect_Solarize \r\n");
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0002);
			break;
			
		case CamDigEffect_Posterize:
		case CamDigEffect_PosterizeMono:
			printk(" CamDigEffect_Posterise \r\n");
			//<CAMTUNING_EFFECT_SKETCH>
			mt9t111_write(0xFCFC,0xD000);
			mt9t111_write(0x0028,0x7000);
			mt9t111_write(0x002A,0x3286);
			mt9t111_write(0x0F12,0x0000); //Pre/Post gamma on(ฟ๘บน)
			mt9t111_write(0x002A,0x021E);
			mt9t111_write(0x0F12,0x0006); //Sketch mode
			break;			
		default:
			printk(" HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
	}	
	//L400_Back_Camera@wind-mobi.com add 2012.02.06 end
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

static HAL_CAM_Result_en_t Init_Mt9t111(CamSensorSelect_t sensor)
{
		HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
		static ktime_t tm1,tm2;
		tm1 = ktime_get();
		
		pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);
		
		CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;
		printk("Init Primary Sensor S5K5CA: \r\n");
		
		/*	
		   * Preview YUV 640x480  Frame rate 10~ 27.5fps
			  * Capture YUV 2048x1536	Frame Rate 8.45
			  * XMCLK 26MHz PCLK 64MHz
		   */
       #if defined (CONFIG_CAM_CSI2)
		   printk(KERN_ERR"Init_Mt9t111 CONFIG_CAM_CSI2 has been defined\n ");
	#endif 
		
		   checkCameraID(sensor);
		//reset MCLK=24M . PCLK=240;

		
		
		//********************************//
		////************************************/																															//
		////Ver 0.1 Nnalog Settings for 5CA EVT0 30FPS in Binning  including latest TnP for EVT1			  //
		////20091310 ded analog settings for 430LSB  long exposure mode only. Settings are for 32MHz Sys. CLK //
		////20091102 ded all calibration data  final settings for STW EVT1 module  SCLK 32MHz  PCLK 60 MHz.   //
		////20091104 anged the shading alpha &Near off														  //
		////20091104 anged awbb_GridEnable from 0001h to 0002h	//awbb_GridEnable							  //
		////aetarget4  gamma change for improving contrast													  //
		////20100113 w TnP updated//
		////20111122 SEMCO 5CA 1st model setting Base : MIPI Interface	AF GPIO Port 4->5
		////20111215 ZTE 1st setting matching (I2C device ID : 0x5A) Preview size : 1024 X 768				  //
		////20111219 ZTE Test setting release : Preview frame-rate modified and AE Metering mode changed.	  //
		
		mt9t111_write(0xFCFC , 0xD000); 	//Reset 								 //
		mt9t111_write(0x0010 , 0x0001);    //Clear host interrupt so main will wait //
		mt9t111_write(0x1030 , 0x0000);    //ARM go 								//
		mt9t111_write(0x0014 , 0x0001);    //Wait100mSec							//
		
		msleep(100);	//delayms(100);
		// It is necessary for delay time 100ms
		// Driving current

		/*  added for debug only 
		mt9t111_write(0x0028 , 0xD000); 
		mt9t111_write(0x002A , 0x1082); 
		mt9t111_write(0x0F12 , 0x0100); //0000 //set IO driving current 2mA for Gs500  0x0155
		mt9t111_write(0x0F12 , 0x0100); //0000 //set IO driving current                      0x0155
		mt9t111_write(0x0F12 , 0x1555); //1555 //set IO driving current
		mt9t111_write(0x0F12 , 0x05D5); //0510 //set IO driving current

		mt9t111_write(0x002A , 0xB0C8); 
		mt9t111_write(0x0F12 , 0x0080);  //0000  0x0040  0x0080
		*/
		
		mt9t111_write(0x0028 , 0x7000); 
		mt9t111_write(0x002A , 0x2CF8); 
		mt9t111_write(0x0F12 , 0xB510); 
		mt9t111_write(0x0F12 , 0x4827); 
		mt9t111_write(0x0F12 , 0x21C0); 
		mt9t111_write(0x0F12 , 0x8041); 
		mt9t111_write(0x0F12 , 0x4825); 
		mt9t111_write(0x0F12 , 0x4A26); 
		mt9t111_write(0x0F12 , 0x3020); 
		mt9t111_write(0x0F12 , 0x8382); 
		mt9t111_write(0x0F12 , 0x1D12); 
		mt9t111_write(0x0F12 , 0x83C2); 
		mt9t111_write(0x0F12 , 0x4822); 
		mt9t111_write(0x0F12 , 0x3040); 
		mt9t111_write(0x0F12 , 0x8041); 
		mt9t111_write(0x0F12 , 0x4821); 
		mt9t111_write(0x0F12 , 0x4922); 
		mt9t111_write(0x0F12 , 0x3060); 
		mt9t111_write(0x0F12 , 0x8381); 
		mt9t111_write(0x0F12 , 0x1D09); 
		mt9t111_write(0x0F12 , 0x83C1); 
		mt9t111_write(0x0F12 , 0x4821); 
		mt9t111_write(0x0F12 , 0x491D); 
		mt9t111_write(0x0F12 , 0x8802); 
		mt9t111_write(0x0F12 , 0x3980); 
		mt9t111_write(0x0F12 , 0x804A); 
		mt9t111_write(0x0F12 , 0x8842); 
		mt9t111_write(0x0F12 , 0x808A); 
		mt9t111_write(0x0F12 , 0x8882); 
		mt9t111_write(0x0F12 , 0x80CA); 
		mt9t111_write(0x0F12 , 0x88C2); 
		mt9t111_write(0x0F12 , 0x810A); 
		mt9t111_write(0x0F12 , 0x8902); 
		mt9t111_write(0x0F12 , 0x491C); 
		mt9t111_write(0x0F12 , 0x80CA); 
		mt9t111_write(0x0F12 , 0x8942); 
		mt9t111_write(0x0F12 , 0x814A); 
		mt9t111_write(0x0F12 , 0x8982); 
		mt9t111_write(0x0F12 , 0x830A); 
		mt9t111_write(0x0F12 , 0x89C2); 
		mt9t111_write(0x0F12 , 0x834A); 
		mt9t111_write(0x0F12 , 0x8A00); 
		mt9t111_write(0x0F12 , 0x4918); 
		mt9t111_write(0x0F12 , 0x8188); 
		mt9t111_write(0x0F12 , 0x4918); 
		mt9t111_write(0x0F12 , 0x4819); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xFA0E); 
		mt9t111_write(0x0F12 , 0x4918); 
		mt9t111_write(0x0F12 , 0x4819); 
		mt9t111_write(0x0F12 , 0x6341); 
		mt9t111_write(0x0F12 , 0x4919); 
		mt9t111_write(0x0F12 , 0x4819); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xFA07); 
		mt9t111_write(0x0F12 , 0x4816); 
		mt9t111_write(0x0F12 , 0x4918); 
		mt9t111_write(0x0F12 , 0x3840); 
		mt9t111_write(0x0F12 , 0x62C1); 
		mt9t111_write(0x0F12 , 0x4918); 
		mt9t111_write(0x0F12 , 0x3880); 
		mt9t111_write(0x0F12 , 0x63C1); 
		mt9t111_write(0x0F12 , 0x4917); 
		mt9t111_write(0x0F12 , 0x6301); 
		mt9t111_write(0x0F12 , 0x4917); 
		mt9t111_write(0x0F12 , 0x3040); 
		mt9t111_write(0x0F12 , 0x6181); 
		mt9t111_write(0x0F12 , 0x4917); 
		mt9t111_write(0x0F12 , 0x4817); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9F7); 
		mt9t111_write(0x0F12 , 0x4917); 
		mt9t111_write(0x0F12 , 0x4817); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9F3); 
		mt9t111_write(0x0F12 , 0x4917); 
		mt9t111_write(0x0F12 , 0x4817); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9EF); 
		mt9t111_write(0x0F12 , 0xBC10); 
		mt9t111_write(0x0F12 , 0xBC08); 
		mt9t111_write(0x0F12 , 0x4718); 
		mt9t111_write(0x0F12 , 0x1100); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x267C); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x2CE8); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x3274); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xF400); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0xF520); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x2DF1); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x89A9); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x2E43); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x0140); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x2E7D); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xB4F7); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x2F07); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x2F2B); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x2FD1); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x2FE5); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x2FB9); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x013D); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x0F12 , 0x306B); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x5823); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x30B9); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xD789); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0xB570); 
		mt9t111_write(0x0F12 , 0x6804); 
		mt9t111_write(0x0F12 , 0x6845); 
		mt9t111_write(0x0F12 , 0x6881); 
		mt9t111_write(0x0F12 , 0x6840); 
		mt9t111_write(0x0F12 , 0x2900); 
		mt9t111_write(0x0F12 , 0x6880); 
		mt9t111_write(0x0F12 , 0xD007); 
		mt9t111_write(0x0F12 , 0x49C3); 
		mt9t111_write(0x0F12 , 0x8949); 
		mt9t111_write(0x0F12 , 0x084A); 
		mt9t111_write(0x0F12 , 0x1880); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9BA); 
		mt9t111_write(0x0F12 , 0x80A0); 
		mt9t111_write(0x0F12 , 0xE000); 
		mt9t111_write(0x0F12 , 0x80A0); 
		mt9t111_write(0x0F12 , 0x88A0); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD010); 
		mt9t111_write(0x0F12 , 0x68A9); 
		mt9t111_write(0x0F12 , 0x6828); 
		mt9t111_write(0x0F12 , 0x084A); 
		mt9t111_write(0x0F12 , 0x1880); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9AE); 
		mt9t111_write(0x0F12 , 0x8020); 
		mt9t111_write(0x0F12 , 0x1D2D); 
		mt9t111_write(0x0F12 , 0xCD03); 
		mt9t111_write(0x0F12 , 0x084A); 
		mt9t111_write(0x0F12 , 0x1880); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9A7); 
		mt9t111_write(0x0F12 , 0x8060); 
		mt9t111_write(0x0F12 , 0xBC70); 
		mt9t111_write(0x0F12 , 0xBC08); 
		mt9t111_write(0x0F12 , 0x4718); 
		mt9t111_write(0x0F12 , 0x2000); 
		mt9t111_write(0x0F12 , 0x8060); 
		mt9t111_write(0x0F12 , 0x8020); 
		mt9t111_write(0x0F12 , 0xE7F8); 
		mt9t111_write(0x0F12 , 0xB510); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF9A2); 
		mt9t111_write(0x0F12 , 0x48B2); 
		mt9t111_write(0x0F12 , 0x8A40); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD00C); 
		mt9t111_write(0x0F12 , 0x48B1); 
		mt9t111_write(0x0F12 , 0x49B2); 
		mt9t111_write(0x0F12 , 0x8800); 
		mt9t111_write(0x0F12 , 0x4AB2); 
		mt9t111_write(0x0F12 , 0x2805); 
		mt9t111_write(0x0F12 , 0xD003); 
		mt9t111_write(0x0F12 , 0x4BB1); 
		mt9t111_write(0x0F12 , 0x795B); 
		mt9t111_write(0x0F12 , 0x2B00); 
		mt9t111_write(0x0F12 , 0xD005); 
		mt9t111_write(0x0F12 , 0x2001); 
		mt9t111_write(0x0F12 , 0x8008); 
		mt9t111_write(0x0F12 , 0x8010); 
		mt9t111_write(0x0F12 , 0xBC10); 
		mt9t111_write(0x0F12 , 0xBC08); 
		mt9t111_write(0x0F12 , 0x4718); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD1FA); 
		mt9t111_write(0x0F12 , 0x2000); 
		mt9t111_write(0x0F12 , 0x8008); 
		mt9t111_write(0x0F12 , 0x8010); 
		mt9t111_write(0x0F12 , 0xE7F6); 
		mt9t111_write(0x0F12 , 0xB5F8); 
		mt9t111_write(0x0F12 , 0x2407); 
		mt9t111_write(0x0F12 , 0x2C06); 
		mt9t111_write(0x0F12 , 0xD035); 
		mt9t111_write(0x0F12 , 0x2C07); 
		mt9t111_write(0x0F12 , 0xD033); 
		mt9t111_write(0x0F12 , 0x48A3); 
		mt9t111_write(0x0F12 , 0x8BC1); 
		mt9t111_write(0x0F12 , 0x2900); 
		mt9t111_write(0x0F12 , 0xD02A); 
		mt9t111_write(0x0F12 , 0x00A2); 
		mt9t111_write(0x0F12 , 0x1815); 
		mt9t111_write(0x0F12 , 0x4AA4); 
		mt9t111_write(0x0F12 , 0x6DEE); 
		mt9t111_write(0x0F12 , 0x8A92); 
		mt9t111_write(0x0F12 , 0x4296); 
		mt9t111_write(0x0F12 , 0xD923); 
		mt9t111_write(0x0F12 , 0x0028); 
		mt9t111_write(0x0F12 , 0x3080); 
		mt9t111_write(0x0F12 , 0x0007); 
		mt9t111_write(0x0F12 , 0x69C0); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF96B); 
		mt9t111_write(0x0F12 , 0x1C71); 
		mt9t111_write(0x0F12 , 0x0280); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF967); 
		mt9t111_write(0x0F12 , 0x0006); 
		mt9t111_write(0x0F12 , 0x4898); 
		mt9t111_write(0x0F12 , 0x0061); 
		mt9t111_write(0x0F12 , 0x1808); 
		mt9t111_write(0x0F12 , 0x8D80); 
		mt9t111_write(0x0F12 , 0x0A01); 
		mt9t111_write(0x0F12 , 0x0600); 
		mt9t111_write(0x0F12 , 0x0E00); 
		mt9t111_write(0x0F12 , 0x1A08); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF96A); 
		mt9t111_write(0x0F12 , 0x0002); 
		mt9t111_write(0x0F12 , 0x6DE9); 
		mt9t111_write(0x0F12 , 0x6FE8); 
		mt9t111_write(0x0F12 , 0x1A08); 
		mt9t111_write(0x0F12 , 0x4351); 
		mt9t111_write(0x0F12 , 0x0300); 
		mt9t111_write(0x0F12 , 0x1C49); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF953); 
		mt9t111_write(0x0F12 , 0x0401); 
		mt9t111_write(0x0F12 , 0x0430); 
		mt9t111_write(0x0F12 , 0x0C00); 
		mt9t111_write(0x0F12 , 0x4301); 
		mt9t111_write(0x0F12 , 0x61F9); 
		mt9t111_write(0x0F12 , 0xE004); 
		mt9t111_write(0x0F12 , 0x00A2); 
		mt9t111_write(0x0F12 , 0x4990); 
		mt9t111_write(0x0F12 , 0x1810); 
		mt9t111_write(0x0F12 , 0x3080); 
		mt9t111_write(0x0F12 , 0x61C1); 
		mt9t111_write(0x0F12 , 0x1E64); 
		mt9t111_write(0x0F12 , 0xD2C5); 
		mt9t111_write(0x0F12 , 0x2006); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF959); 
		mt9t111_write(0x0F12 , 0x2007); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF956); 
		mt9t111_write(0x0F12 , 0xBCF8); 
		mt9t111_write(0x0F12 , 0xBC08); 
		mt9t111_write(0x0F12 , 0x4718); 
		mt9t111_write(0x0F12 , 0xB510); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF958); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD00A); 
		mt9t111_write(0x0F12 , 0x4881); 
		mt9t111_write(0x0F12 , 0x8B81); 
		mt9t111_write(0x0F12 , 0x0089); 
		mt9t111_write(0x0F12 , 0x1808); 
		mt9t111_write(0x0F12 , 0x6DC1); 
		mt9t111_write(0x0F12 , 0x4883); 
		mt9t111_write(0x0F12 , 0x8A80); 
		mt9t111_write(0x0F12 , 0x4281); 
		mt9t111_write(0x0F12 , 0xD901); 
		mt9t111_write(0x0F12 , 0x2001); 
		mt9t111_write(0x0F12 , 0xE7A1); 
		mt9t111_write(0x0F12 , 0x2000); 
		mt9t111_write(0x0F12 , 0xE79F); 
		mt9t111_write(0x0F12 , 0xB5F8); 
		mt9t111_write(0x0F12 , 0x0004); 
		mt9t111_write(0x0F12 , 0x4F80); 
		mt9t111_write(0x0F12 , 0x227D); 
		mt9t111_write(0x0F12 , 0x8938); 
		mt9t111_write(0x0F12 , 0x0152); 
		mt9t111_write(0x0F12 , 0x4342); 
		mt9t111_write(0x0F12 , 0x487E); 
		mt9t111_write(0x0F12 , 0x9000); 
		mt9t111_write(0x0F12 , 0x8A01); 
		mt9t111_write(0x0F12 , 0x0848); 
		mt9t111_write(0x0F12 , 0x1810); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF91D); 
		mt9t111_write(0x0F12 , 0x210F); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF940); 
		mt9t111_write(0x0F12 , 0x497A); 
		mt9t111_write(0x0F12 , 0x8C49); 
		mt9t111_write(0x0F12 , 0x090E); 
		mt9t111_write(0x0F12 , 0x0136); 
		mt9t111_write(0x0F12 , 0x4306); 
		mt9t111_write(0x0F12 , 0x4979); 
		mt9t111_write(0x0F12 , 0x2C00); 
		mt9t111_write(0x0F12 , 0xD003); 
		mt9t111_write(0x0F12 , 0x2001); 
		mt9t111_write(0x0F12 , 0x0240); 
		mt9t111_write(0x0F12 , 0x4330); 
		mt9t111_write(0x0F12 , 0x8108); 
		mt9t111_write(0x0F12 , 0x4876); 
		mt9t111_write(0x0F12 , 0x2C00); 
		mt9t111_write(0x0F12 , 0x8D00); 
		mt9t111_write(0x0F12 , 0xD001); 
		mt9t111_write(0x0F12 , 0x2501); 
		mt9t111_write(0x0F12 , 0xE000); 
		mt9t111_write(0x0F12 , 0x2500); 
		mt9t111_write(0x0F12 , 0x4972); 
		mt9t111_write(0x0F12 , 0x4328); 
		mt9t111_write(0x0F12 , 0x8008); 
		mt9t111_write(0x0F12 , 0x207D); 
		mt9t111_write(0x0F12 , 0x00C0); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF92E); 
		mt9t111_write(0x0F12 , 0x2C00); 
		mt9t111_write(0x0F12 , 0x496E); 
		mt9t111_write(0x0F12 , 0x0328); 
		mt9t111_write(0x0F12 , 0x4330); 
		mt9t111_write(0x0F12 , 0x8108); 
		mt9t111_write(0x0F12 , 0x88F8); 
		mt9t111_write(0x0F12 , 0x2C00); 
		mt9t111_write(0x0F12 , 0x01AA); 
		mt9t111_write(0x0F12 , 0x4310); 
		mt9t111_write(0x0F12 , 0x8088); 
		mt9t111_write(0x0F12 , 0x9800); 
		mt9t111_write(0x0F12 , 0x8A01); 
		mt9t111_write(0x0F12 , 0x486A); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8F1); 
		mt9t111_write(0x0F12 , 0x496A); 
		mt9t111_write(0x0F12 , 0x8809); 
		mt9t111_write(0x0F12 , 0x4348); 
		mt9t111_write(0x0F12 , 0x0400); 
		mt9t111_write(0x0F12 , 0x0C00); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF918); 
		mt9t111_write(0x0F12 , 0x0020); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF91D); 
		mt9t111_write(0x0F12 , 0x4866); 
		mt9t111_write(0x0F12 , 0x7004); 
		mt9t111_write(0x0F12 , 0xE7A3); 
		mt9t111_write(0x0F12 , 0xB510); 
		mt9t111_write(0x0F12 , 0x0004); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF91E); 
		mt9t111_write(0x0F12 , 0x6020); 
		mt9t111_write(0x0F12 , 0x4963); 
		mt9t111_write(0x0F12 , 0x8B49); 
		mt9t111_write(0x0F12 , 0x0789); 
		mt9t111_write(0x0F12 , 0xD001); 
		mt9t111_write(0x0F12 , 0x0040); 
		mt9t111_write(0x0F12 , 0x6020); 
		mt9t111_write(0x0F12 , 0xE74C); 
		mt9t111_write(0x0F12 , 0xB510); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF91B); 
		mt9t111_write(0x0F12 , 0x485F); 
		mt9t111_write(0x0F12 , 0x8880); 
		mt9t111_write(0x0F12 , 0x0601); 
		mt9t111_write(0x0F12 , 0x4854); 
		mt9t111_write(0x0F12 , 0x1609); 
		mt9t111_write(0x0F12 , 0x8141); 
		mt9t111_write(0x0F12 , 0xE742); 
		mt9t111_write(0x0F12 , 0xB5F8); 
		mt9t111_write(0x0F12 , 0x000F); 
		mt9t111_write(0x0F12 , 0x4C55); 
		mt9t111_write(0x0F12 , 0x3420); 
		mt9t111_write(0x0F12 , 0x2500); 
		mt9t111_write(0x0F12 , 0x5765); 
		mt9t111_write(0x0F12 , 0x0039); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF913); 
		mt9t111_write(0x0F12 , 0x9000); 
		mt9t111_write(0x0F12 , 0x2600); 
		mt9t111_write(0x0F12 , 0x57A6); 
		mt9t111_write(0x0F12 , 0x4C4C); 
		mt9t111_write(0x0F12 , 0x42AE); 
		mt9t111_write(0x0F12 , 0xD01B); 
		mt9t111_write(0x0F12 , 0x4D54); 
		mt9t111_write(0x0F12 , 0x8AE8); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD013); 
		mt9t111_write(0x0F12 , 0x484D); 
		mt9t111_write(0x0F12 , 0x8A01); 
		mt9t111_write(0x0F12 , 0x8B80); 
		mt9t111_write(0x0F12 , 0x4378); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8B5); 
		mt9t111_write(0x0F12 , 0x89A9); 
		mt9t111_write(0x0F12 , 0x1A41); 
		mt9t111_write(0x0F12 , 0x484E); 
		mt9t111_write(0x0F12 , 0x3820); 
		mt9t111_write(0x0F12 , 0x8AC0); 
		mt9t111_write(0x0F12 , 0x4348); 
		mt9t111_write(0x0F12 , 0x17C1); 
		mt9t111_write(0x0F12 , 0x0D89); 
		mt9t111_write(0x0F12 , 0x1808); 
		mt9t111_write(0x0F12 , 0x1280); 
		mt9t111_write(0x0F12 , 0x8961); 
		mt9t111_write(0x0F12 , 0x1A08); 
		mt9t111_write(0x0F12 , 0x8160); 
		mt9t111_write(0x0F12 , 0xE003); 
		mt9t111_write(0x0F12 , 0x88A8); 
		mt9t111_write(0x0F12 , 0x0600); 
		mt9t111_write(0x0F12 , 0x1600); 
		mt9t111_write(0x0F12 , 0x8160); 
		mt9t111_write(0x0F12 , 0x200A); 
		mt9t111_write(0x0F12 , 0x5E20); 
		mt9t111_write(0x0F12 , 0x42B0); 
		mt9t111_write(0x0F12 , 0xD011); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8AB); 
		mt9t111_write(0x0F12 , 0x1D40); 
		mt9t111_write(0x0F12 , 0x00C3); 
		mt9t111_write(0x0F12 , 0x1A18); 
		mt9t111_write(0x0F12 , 0x214B); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF897); 
		mt9t111_write(0x0F12 , 0x211F); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8BA); 
		mt9t111_write(0x0F12 , 0x210A); 
		mt9t111_write(0x0F12 , 0x5E61); 
		mt9t111_write(0x0F12 , 0x0FC9); 
		mt9t111_write(0x0F12 , 0x0149); 
		mt9t111_write(0x0F12 , 0x4301); 
		mt9t111_write(0x0F12 , 0x483D); 
		mt9t111_write(0x0F12 , 0x81C1); 
		mt9t111_write(0x0F12 , 0x9800); 
		mt9t111_write(0x0F12 , 0xE74A); 
		mt9t111_write(0x0F12 , 0xB5F1); 
		mt9t111_write(0x0F12 , 0xB082); 
		mt9t111_write(0x0F12 , 0x2500); 
		mt9t111_write(0x0F12 , 0x483A); 
		mt9t111_write(0x0F12 , 0x9001); 
		mt9t111_write(0x0F12 , 0x2400); 
		mt9t111_write(0x0F12 , 0x2028); 
		mt9t111_write(0x0F12 , 0x4368); 
		mt9t111_write(0x0F12 , 0x4A39); 
		mt9t111_write(0x0F12 , 0x4925); 
		mt9t111_write(0x0F12 , 0x1887); 
		mt9t111_write(0x0F12 , 0x1840); 
		mt9t111_write(0x0F12 , 0x9000); 
		mt9t111_write(0x0F12 , 0x9800); 
		mt9t111_write(0x0F12 , 0x0066); 
		mt9t111_write(0x0F12 , 0x9A01); 
		mt9t111_write(0x0F12 , 0x1980); 
		mt9t111_write(0x0F12 , 0x218C); 
		mt9t111_write(0x0F12 , 0x5A09); 
		mt9t111_write(0x0F12 , 0x8A80); 
		mt9t111_write(0x0F12 , 0x8812); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8CA); 
		mt9t111_write(0x0F12 , 0x53B8); 
		mt9t111_write(0x0F12 , 0x1C64); 
		mt9t111_write(0x0F12 , 0x2C14); 
		mt9t111_write(0x0F12 , 0xDBF1); 
		mt9t111_write(0x0F12 , 0x1C6D); 
		mt9t111_write(0x0F12 , 0x2D03); 
		mt9t111_write(0x0F12 , 0xDBE6); 
		mt9t111_write(0x0F12 , 0x9802); 
		mt9t111_write(0x0F12 , 0x6800); 
		mt9t111_write(0x0F12 , 0x0600); 
		mt9t111_write(0x0F12 , 0x0E00); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8C5); 
		mt9t111_write(0x0F12 , 0xBCFE); 
		mt9t111_write(0x0F12 , 0xBC08); 
		mt9t111_write(0x0F12 , 0x4718); 
		mt9t111_write(0x0F12 , 0xB570); 
		mt9t111_write(0x0F12 , 0x6805); 
		mt9t111_write(0x0F12 , 0x2404); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8C5); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD103); 
		mt9t111_write(0x0F12 , 0xF000); 
		mt9t111_write(0x0F12 , 0xF8C9); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x2400); 
		mt9t111_write(0x0F12 , 0x3540); 
		mt9t111_write(0x0F12 , 0x88E8); 
		mt9t111_write(0x0F12 , 0x0500); 
		mt9t111_write(0x0F12 , 0xD403); 
		mt9t111_write(0x0F12 , 0x4822); 
		mt9t111_write(0x0F12 , 0x89C0); 
		mt9t111_write(0x0F12 , 0x2800); 
		mt9t111_write(0x0F12 , 0xD002); 
		mt9t111_write(0x0F12 , 0x2008); 
		mt9t111_write(0x0F12 , 0x4304); 
		mt9t111_write(0x0F12 , 0xE001); 
		mt9t111_write(0x0F12 , 0x2010); 
		mt9t111_write(0x0F12 , 0x4304); 
		mt9t111_write(0x0F12 , 0x481F); 
		mt9t111_write(0x0F12 , 0x8B80); 
		mt9t111_write(0x0F12 , 0x0700); 
		mt9t111_write(0x0F12 , 0x0F81); 
		mt9t111_write(0x0F12 , 0x2001); 
		mt9t111_write(0x0F12 , 0x2900); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x4304); 
		mt9t111_write(0x0F12 , 0x491C); 
		mt9t111_write(0x0F12 , 0x8B0A); 
		mt9t111_write(0x0F12 , 0x42A2); 
		mt9t111_write(0x0F12 , 0xD004); 
		mt9t111_write(0x0F12 , 0x0762); 
		mt9t111_write(0x0F12 , 0xD502); 
		mt9t111_write(0x0F12 , 0x4A19); 
		mt9t111_write(0x0F12 , 0x3220); 
		mt9t111_write(0x0F12 , 0x8110); 
		mt9t111_write(0x0F12 , 0x830C); 
		mt9t111_write(0x0F12 , 0xE691); 
		mt9t111_write(0x0F12 , 0x0C3C); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x3274); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x26E8); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x6100); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x6500); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x1A7C); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x1120); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xFFFF); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x3374); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x1D6C); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x167C); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xF400); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x2C2C); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x40A0); 
		mt9t111_write(0x0F12 , 0x00DD); 
		mt9t111_write(0x0F12 , 0xF520); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x2C29); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x1A54); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x1564); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xF2A0); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x2440); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x05A0); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x2894); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0x1224); 
		mt9t111_write(0x0F12 , 0x7000); 
		mt9t111_write(0x0F12 , 0xB000); 
		mt9t111_write(0x0F12 , 0xD000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x1A3F); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xF004); 
		mt9t111_write(0x0F12 , 0xE51F); 
		mt9t111_write(0x0F12 , 0x1F48); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x24BD); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x36DD); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xB4CF); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xB5D7); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x36ED); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xF53F); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xF5D9); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x013D); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xF5C9); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xFAA9); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x3723); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0x5823); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xD771); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x4778); 
		mt9t111_write(0x0F12 , 0x46C0); 
		mt9t111_write(0x0F12 , 0xC000); 
		mt9t111_write(0x0F12 , 0xE59F); 
		mt9t111_write(0x0F12 , 0xFF1C); 
		mt9t111_write(0x0F12 , 0xE12F); 
		mt9t111_write(0x0F12 , 0xD75B); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x8117); 
		mt9t111_write(0x0F12 , 0x0000); 
		
		// End T&P part
			
		//========================================================						
		// CIs/APs/An setting		 - 400LsB  sYsCLK 32MHz 							
		//========================================================						
		// This regis are for FACTORY ONLY. If you change it without prior notification 
		// YOU are REsIBLE for the FAILURE that will happen in the future.				
		//========================================================						
		
		mt9t111_write(0x002A , 0x157A); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x002A , 0x1578); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x002A , 0x1576); 
		mt9t111_write(0x0F12 , 0x0020); 
		mt9t111_write(0x002A , 0x1574); 
		mt9t111_write(0x0F12 , 0x0006); 
		mt9t111_write(0x002A , 0x156E); 
		mt9t111_write(0x0F12 , 0x0001);   //slope calibration tolerance in units of 1/256
		mt9t111_write(0x002A , 0x1568); 
		mt9t111_write(0x0F12 , 0x00FC); 
		
		//ADC control 
		mt9t111_write(0x002A , 0x155A); 
		mt9t111_write(0x0F12 , 0x01CC); 	// ADC sAT of 450mV for 10bit default in EVT1
		mt9t111_write(0x002A , 0x157E); 
		mt9t111_write(0x0F12 , 0x0C80); 	// 3200 Max. Reset ramp DCLK counts (default 2048 0x800)
		mt9t111_write(0x0F12 , 0x0578); 	// 1400 Max. Reset ramp DCLK counts for x3.5
		mt9t111_write(0x002A , 0x157C); 
		mt9t111_write(0x0F12 , 0x0190); 	// 400 Reset ramp for x1 in DCLK counts
		mt9t111_write(0x002A , 0x1570); 
		mt9t111_write(0x0F12 , 0x00A0); 	// 160 LsB
		mt9t111_write(0x0F12 , 0x0010); 	// reset threshold
		mt9t111_write(0x002A , 0x12C4); 
		mt9t111_write(0x0F12 , 0x006A); 	// 106 additional timing columns.
		mt9t111_write(0x002A , 0x12C8); 
		mt9t111_write(0x0F12 , 0x08AC); 	// 2220 ADC columns in normal mode including Hold & Latch
		mt9t111_write(0x0F12 , 0x0050); 	// 80 addition of ADC columns in Y-ave mode (default 244 0x74)
		
		mt9t111_write(0x002A , 0x1696); 	 // based on APs guidelines
		mt9t111_write(0x0F12 , 0x0000);    // based on APs guidelines
		mt9t111_write(0x0F12 , 0x0000);    // default. 1492 used for ADC dark characteristics
		mt9t111_write(0x0F12 , 0x00C6);    // default. 1492 used for ADC dark characteristics
		mt9t111_write(0x0F12 , 0x00C6); 
		
		mt9t111_write(0x002A , 0x1690);    // when set double sampling is activated - requires different set of pointers
		mt9t111_write(0x0F12 , 0x0001); 
		
		mt9t111_write(0x002A , 0x12B0);    // comp and pixel bias control 0xF40E - default for EVT1
		mt9t111_write(0x0F12 , 0x0055);    // comp and pixel bias control 0xF40E for binning mode
		mt9t111_write(0x0F12 , 0x005A); 
		
		mt9t111_write(0x002A , 0x337A);    // [7] - is used for rest-only mode (EVT0 value is 0xD and HW 0x6)
		mt9t111_write(0x0F12 , 0x0006); 
		mt9t111_write(0x0F12 , 0x0068); 
		mt9t111_write(0x002A , 0x169E); 
		mt9t111_write(0x0F12 , 0x0007); 
		mt9t111_write(0x002A , 0x0BF6); 
		mt9t111_write(0x0F12 , 0x0000); 
		
		
		mt9t111_write(0x002A , 0x327C); 
		mt9t111_write(0x0F12 , 0x1000); 
		mt9t111_write(0x0F12 , 0x6998); 
		mt9t111_write(0x0F12 , 0x0078); 
		mt9t111_write(0x0F12 , 0x04FE); 
		mt9t111_write(0x0F12 , 0x8800); 
		// Driving current
		mt9t111_write(0x002A , 0x3274); 
		mt9t111_write(0x0F12 , 0x0100); //0000 //set IO driving current 2mA for Gs500  0x0155
		mt9t111_write(0x0F12 , 0x0100); //0000 //set IO driving current                      0x0155
		mt9t111_write(0x0F12 , 0x1555); //1555 //set IO driving current
		mt9t111_write(0x0F12 , 0x05D5); //0510 //set IO driving current
		
		mt9t111_write(0x0028 , 0x7000); 
		mt9t111_write(0x002A , 0x0572); 
		mt9t111_write(0x0F12 , 0x0007); 	//#skl_usConfigStbySettings // Enable T&P code after HW stby + skip ZI part on HW wakeup.
		
		mt9t111_write(0x0028 , 0x7000); 
		mt9t111_write(0x002A , 0x12D2); 
		mt9t111_write(0x0F12 , 0x0003); 	 //senHal_pContSenModesRegsArray[0][0]2 700012D2
		mt9t111_write(0x0F12 , 0x0003);    //senHal_pContSenModesRegsArray[0][1]2 700012D4	
		mt9t111_write(0x0F12 , 0x0003);    //senHal_pContSenModesRegsArray[0][2]2 700012D6	
		mt9t111_write(0x0F12 , 0x0003);    //senHal_pContSenModesRegsArray[0][3]2 700012D8	
		mt9t111_write(0x0F12 , 0x0884);    //senHal_pContSenModesRegsArray[1][0]2 700012DA	
		mt9t111_write(0x0F12 , 0x08CF);    //senHal_pContSenModesRegsArray[1][1]2 700012DC	
		mt9t111_write(0x0F12 , 0x0500);    //senHal_pContSenModesRegsArray[1][2]2 700012DE	
		mt9t111_write(0x0F12 , 0x054B);    //senHal_pContSenModesRegsArray[1][3]2 700012E0	
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[2][0]2 700012E2	
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[2][1]2 700012E4	
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[2][2]2 700012E6	
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[2][3]2 700012E8	
		mt9t111_write(0x0F12 , 0x0885);    //senHal_pContSenModesRegsArray[3][0]2 700012EA	
		mt9t111_write(0x0F12 , 0x0467);    //senHal_pContSenModesRegsArray[3][1]2 700012EC	
		mt9t111_write(0x0F12 , 0x0501);    //senHal_pContSenModesRegsArray[3][2]2 700012EE	
		mt9t111_write(0x0F12 , 0x02A5);    //senHal_pContSenModesRegsArray[3][3]2 700012F0	
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[4][0]2 700012F2	
		mt9t111_write(0x0F12 , 0x046A);    //senHal_pContSenModesRegsArray[4][1]2 700012F4	
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[4][2]2 700012F6	
		mt9t111_write(0x0F12 , 0x02A8);    //senHal_pContSenModesRegsArray[4][3]2 700012F8	
		mt9t111_write(0x0F12 , 0x0885);    //senHal_pContSenModesRegsArray[5][0]2 700012FA	
		mt9t111_write(0x0F12 , 0x08D0);    //senHal_pContSenModesRegsArray[5][1]2 700012FC	
		mt9t111_write(0x0F12 , 0x0501);    //senHal_pContSenModesRegsArray[5][2]2 700012FE	
		mt9t111_write(0x0F12 , 0x054C);    //senHal_pContSenModesRegsArray[5][3]2 70001300	
		mt9t111_write(0x0F12 , 0x0006);    //senHal_pContSenModesRegsArray[6][0]2 70001302	
		mt9t111_write(0x0F12 , 0x0020);    //senHal_pContSenModesRegsArray[6][1]2 70001304	
		mt9t111_write(0x0F12 , 0x0006);    //senHal_pContSenModesRegsArray[6][2]2 70001306	
		mt9t111_write(0x0F12 , 0x0020);    //senHal_pContSenModesRegsArray[6][3]2 70001308	
		mt9t111_write(0x0F12 , 0x0881);    //senHal_pContSenModesRegsArray[7][0]2 7000130A	
		mt9t111_write(0x0F12 , 0x0463);    //senHal_pContSenModesRegsArray[7][1]2 7000130C	
		mt9t111_write(0x0F12 , 0x04FD);    //senHal_pContSenModesRegsArray[7][2]2 7000130E	
		mt9t111_write(0x0F12 , 0x02A1);    //senHal_pContSenModesRegsArray[7][3]2 70001310	
		mt9t111_write(0x0F12 , 0x0006);    //senHal_pContSenModesRegsArray[8][0]2 70001312	
		mt9t111_write(0x0F12 , 0x0489);    //senHal_pContSenModesRegsArray[8][1]2 70001314	
		mt9t111_write(0x0F12 , 0x0006);    //senHal_pContSenModesRegsArray[8][2]2 70001316	
		mt9t111_write(0x0F12 , 0x02C7);    //senHal_pContSenModesRegsArray[8][3]2 70001318	
		mt9t111_write(0x0F12 , 0x0881);    //senHal_pContSenModesRegsArray[9][0]2 7000131A	
		mt9t111_write(0x0F12 , 0x08CC);    //senHal_pContSenModesRegsArray[9][1]2 7000131C	
		mt9t111_write(0x0F12 , 0x04FD);    //senHal_pContSenModesRegsArray[9][2]2 7000131E	
		mt9t111_write(0x0F12 , 0x0548);    //senHal_pContSenModesRegsArray[9][3]2 70001320	
		mt9t111_write(0x0F12 , 0x03A2);    //senHal_pContSenModesRegsArray[10][0] 2 70001322
		mt9t111_write(0x0F12 , 0x01D3);    //senHal_pContSenModesRegsArray[10][1] 2 70001324
		mt9t111_write(0x0F12 , 0x01E0);    //senHal_pContSenModesRegsArray[10][2] 2 70001326
		mt9t111_write(0x0F12 , 0x00F2);    //senHal_pContSenModesRegsArray[10][3] 2 70001328
		mt9t111_write(0x0F12 , 0x03F2);    //senHal_pContSenModesRegsArray[11][0] 2 7000132A
		mt9t111_write(0x0F12 , 0x0223);    //senHal_pContSenModesRegsArray[11][1] 2 7000132C
		mt9t111_write(0x0F12 , 0x0230);    //senHal_pContSenModesRegsArray[11][2] 2 7000132E
		mt9t111_write(0x0F12 , 0x0142);    //senHal_pContSenModesRegsArray[11][3] 2 70001330
		mt9t111_write(0x0F12 , 0x03A2);    //senHal_pContSenModesRegsArray[12][0] 2 70001332
		mt9t111_write(0x0F12 , 0x063C);    //senHal_pContSenModesRegsArray[12][1] 2 70001334
		mt9t111_write(0x0F12 , 0x01E0);    //senHal_pContSenModesRegsArray[12][2] 2 70001336
		mt9t111_write(0x0F12 , 0x0399);    //senHal_pContSenModesRegsArray[12][3] 2 70001338
		mt9t111_write(0x0F12 , 0x03F2);    //senHal_pContSenModesRegsArray[13][0] 2 7000133A
		mt9t111_write(0x0F12 , 0x068C);    //senHal_pContSenModesRegsArray[13][1] 2 7000133C
		mt9t111_write(0x0F12 , 0x0230);    //senHal_pContSenModesRegsArray[13][2] 2 7000133E
		mt9t111_write(0x0F12 , 0x03E9);    //senHal_pContSenModesRegsArray[13][3] 2 70001340
		mt9t111_write(0x0F12 , 0x0002);    //senHal_pContSenModesRegsArray[14][0] 2 70001342
		mt9t111_write(0x0F12 , 0x0002);    //senHal_pContSenModesRegsArray[14][1] 2 70001344
		mt9t111_write(0x0F12 , 0x0002);    //senHal_pContSenModesRegsArray[14][2] 2 70001346
		mt9t111_write(0x0F12 , 0x0002);    //senHal_pContSenModesRegsArray[14][3] 2 70001348
		mt9t111_write(0x0F12 , 0x003C);    //senHal_pContSenModesRegsArray[15][0] 2 7000134A
		mt9t111_write(0x0F12 , 0x003C);    //senHal_pContSenModesRegsArray[15][1] 2 7000134C
		mt9t111_write(0x0F12 , 0x003C);    //senHal_pContSenModesRegsArray[15][2] 2 7000134E
		mt9t111_write(0x0F12 , 0x003C);    //senHal_pContSenModesRegsArray[15][3] 2 70001350
		mt9t111_write(0x0F12 , 0x01D3);    //senHal_pContSenModesRegsArray[16][0] 2 70001352
		mt9t111_write(0x0F12 , 0x01D3);    //senHal_pContSenModesRegsArray[16][1] 2 70001354
		mt9t111_write(0x0F12 , 0x00F2);    //senHal_pContSenModesRegsArray[16][2] 2 70001356
		mt9t111_write(0x0F12 , 0x00F2);    //senHal_pContSenModesRegsArray[16][3] 2 70001358
		mt9t111_write(0x0F12 , 0x020B);    //senHal_pContSenModesRegsArray[17][0] 2 7000135A
		mt9t111_write(0x0F12 , 0x024A);    //senHal_pContSenModesRegsArray[17][1] 2 7000135C
		mt9t111_write(0x0F12 , 0x012A);    //senHal_pContSenModesRegsArray[17][2] 2 7000135E
		mt9t111_write(0x0F12 , 0x0169);    //senHal_pContSenModesRegsArray[17][3] 2 70001360
		mt9t111_write(0x0F12 , 0x0002);    //senHal_pContSenModesRegsArray[18][0] 2 70001362
		mt9t111_write(0x0F12 , 0x046B);    //senHal_pContSenModesRegsArray[18][1] 2 70001364
		mt9t111_write(0x0F12 , 0x0002);    //senHal_pContSenModesRegsArray[18][2] 2 70001366
		mt9t111_write(0x0F12 , 0x02A9);    //senHal_pContSenModesRegsArray[18][3] 2 70001368
		mt9t111_write(0x0F12 , 0x0419);    //senHal_pContSenModesRegsArray[19][0] 2 7000136A
		mt9t111_write(0x0F12 , 0x04A5);    //senHal_pContSenModesRegsArray[19][1] 2 7000136C
		mt9t111_write(0x0F12 , 0x0257);    //senHal_pContSenModesRegsArray[19][2] 2 7000136E
		mt9t111_write(0x0F12 , 0x02E3);    //senHal_pContSenModesRegsArray[19][3] 2 70001370
		mt9t111_write(0x0F12 , 0x0630);    //senHal_pContSenModesRegsArray[20][0] 2 70001372
		mt9t111_write(0x0F12 , 0x063C);    //senHal_pContSenModesRegsArray[20][1] 2 70001374
		mt9t111_write(0x0F12 , 0x038D);    //senHal_pContSenModesRegsArray[20][2] 2 70001376
		mt9t111_write(0x0F12 , 0x0399);    //senHal_pContSenModesRegsArray[20][3] 2 70001378
		mt9t111_write(0x0F12 , 0x0668);    //senHal_pContSenModesRegsArray[21][0] 2 7000137A
		mt9t111_write(0x0F12 , 0x06B3);    //senHal_pContSenModesRegsArray[21][1] 2 7000137C
		mt9t111_write(0x0F12 , 0x03C5);    //senHal_pContSenModesRegsArray[21][2] 2 7000137E
		mt9t111_write(0x0F12 , 0x0410);    //senHal_pContSenModesRegsArray[21][3] 2 70001380
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[22][0] 2 70001382
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[22][1] 2 70001384
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[22][2] 2 70001386
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[22][3] 2 70001388
		mt9t111_write(0x0F12 , 0x03A2);    //senHal_pContSenModesRegsArray[23][0] 2 7000138A
		mt9t111_write(0x0F12 , 0x01D3);    //senHal_pContSenModesRegsArray[23][1] 2 7000138C
		mt9t111_write(0x0F12 , 0x01E0);    //senHal_pContSenModesRegsArray[23][2] 2 7000138E
		mt9t111_write(0x0F12 , 0x00F2);    //senHal_pContSenModesRegsArray[23][3] 2 70001390
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[24][0] 2 70001392
		mt9t111_write(0x0F12 , 0x0461);    //senHal_pContSenModesRegsArray[24][1] 2 70001394
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[24][2] 2 70001396
		mt9t111_write(0x0F12 , 0x029F);    //senHal_pContSenModesRegsArray[24][3] 2 70001398
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[25][0] 2 7000139A
		mt9t111_write(0x0F12 , 0x063C);    //senHal_pContSenModesRegsArray[25][1] 2 7000139C
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[25][2] 2 7000139E
		mt9t111_write(0x0F12 , 0x0399);    //senHal_pContSenModesRegsArray[25][3] 2 700013A0
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[26][0] 2 700013A2
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[26][1] 2 700013A4
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[26][2] 2 700013A6
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[26][3] 2 700013A8
		mt9t111_write(0x0F12 , 0x01D0);    //senHal_pContSenModesRegsArray[27][0] 2 700013AA
		mt9t111_write(0x0F12 , 0x01D0);    //senHal_pContSenModesRegsArray[27][1] 2 700013AC
		mt9t111_write(0x0F12 , 0x00EF);    //senHal_pContSenModesRegsArray[27][2] 2 700013AE
		mt9t111_write(0x0F12 , 0x00EF);    //senHal_pContSenModesRegsArray[27][3] 2 700013B0
		mt9t111_write(0x0F12 , 0x020C);    //senHal_pContSenModesRegsArray[28][0] 2 700013B2
		mt9t111_write(0x0F12 , 0x024B);    //senHal_pContSenModesRegsArray[28][1] 2 700013B4
		mt9t111_write(0x0F12 , 0x012B);    //senHal_pContSenModesRegsArray[28][2] 2 700013B6
		mt9t111_write(0x0F12 , 0x016A);    //senHal_pContSenModesRegsArray[28][3] 2 700013B8
		mt9t111_write(0x0F12 , 0x039F);    //senHal_pContSenModesRegsArray[29][0] 2 700013BA
		mt9t111_write(0x0F12 , 0x045E);    //senHal_pContSenModesRegsArray[29][1] 2 700013BC
		mt9t111_write(0x0F12 , 0x01DD);    //senHal_pContSenModesRegsArray[29][2] 2 700013BE
		mt9t111_write(0x0F12 , 0x029C);    //senHal_pContSenModesRegsArray[29][3] 2 700013C0
		mt9t111_write(0x0F12 , 0x041A);    //senHal_pContSenModesRegsArray[30][0] 2 700013C2
		mt9t111_write(0x0F12 , 0x04A6);    //senHal_pContSenModesRegsArray[30][1] 2 700013C4
		mt9t111_write(0x0F12 , 0x0258);    //senHal_pContSenModesRegsArray[30][2] 2 700013C6
		mt9t111_write(0x0F12 , 0x02E4);    //senHal_pContSenModesRegsArray[30][3] 2 700013C8
		mt9t111_write(0x0F12 , 0x062D);    //senHal_pContSenModesRegsArray[31][0] 2 700013CA
		mt9t111_write(0x0F12 , 0x0639);    //senHal_pContSenModesRegsArray[31][1] 2 700013CC
		mt9t111_write(0x0F12 , 0x038A);    //senHal_pContSenModesRegsArray[31][2] 2 700013CE
		mt9t111_write(0x0F12 , 0x0396);    //senHal_pContSenModesRegsArray[31][3] 2 700013D0
		mt9t111_write(0x0F12 , 0x0669);    //senHal_pContSenModesRegsArray[32][0] 2 700013D2
		mt9t111_write(0x0F12 , 0x06B4);    //senHal_pContSenModesRegsArray[32][1] 2 700013D4
		mt9t111_write(0x0F12 , 0x03C6);    //senHal_pContSenModesRegsArray[32][2] 2 700013D6
		mt9t111_write(0x0F12 , 0x0411);    //senHal_pContSenModesRegsArray[32][3] 2 700013D8
		mt9t111_write(0x0F12 , 0x087C);    //senHal_pContSenModesRegsArray[33][0] 2 700013DA
		mt9t111_write(0x0F12 , 0x08C7);    //senHal_pContSenModesRegsArray[33][1] 2 700013DC
		mt9t111_write(0x0F12 , 0x04F8);    //senHal_pContSenModesRegsArray[33][2] 2 700013DE
		mt9t111_write(0x0F12 , 0x0543);    //senHal_pContSenModesRegsArray[33][3] 2 700013E0
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[34][0] 2 700013E2
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[34][1] 2 700013E4
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[34][2] 2 700013E6
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[34][3] 2 700013E8
		mt9t111_write(0x0F12 , 0x01D0);    //senHal_pContSenModesRegsArray[35][0] 2 700013EA
		mt9t111_write(0x0F12 , 0x01D0);    //senHal_pContSenModesRegsArray[35][1] 2 700013EC
		mt9t111_write(0x0F12 , 0x00EF);    //senHal_pContSenModesRegsArray[35][2] 2 700013EE
		mt9t111_write(0x0F12 , 0x00EF);    //senHal_pContSenModesRegsArray[35][3] 2 700013F0
		mt9t111_write(0x0F12 , 0x020F);    //senHal_pContSenModesRegsArray[36][0] 2 700013F2
		mt9t111_write(0x0F12 , 0x024E);    //senHal_pContSenModesRegsArray[36][1] 2 700013F4
		mt9t111_write(0x0F12 , 0x012E);    //senHal_pContSenModesRegsArray[36][2] 2 700013F6
		mt9t111_write(0x0F12 , 0x016D);    //senHal_pContSenModesRegsArray[36][3] 2 700013F8
		mt9t111_write(0x0F12 , 0x039F);    //senHal_pContSenModesRegsArray[37][0] 2 700013FA
		mt9t111_write(0x0F12 , 0x045E);    //senHal_pContSenModesRegsArray[37][1] 2 700013FC
		mt9t111_write(0x0F12 , 0x01DD);    //senHal_pContSenModesRegsArray[37][2] 2 700013FE
		mt9t111_write(0x0F12 , 0x029C);    //senHal_pContSenModesRegsArray[37][3] 2 70001400
		mt9t111_write(0x0F12 , 0x041D);    //senHal_pContSenModesRegsArray[38][0] 2 70001402
		mt9t111_write(0x0F12 , 0x04A9);    //senHal_pContSenModesRegsArray[38][1] 2 70001404
		mt9t111_write(0x0F12 , 0x025B);    //senHal_pContSenModesRegsArray[38][2] 2 70001406
		mt9t111_write(0x0F12 , 0x02E7);    //senHal_pContSenModesRegsArray[38][3] 2 70001408
		mt9t111_write(0x0F12 , 0x062D);    //senHal_pContSenModesRegsArray[39][0] 2 7000140A
		mt9t111_write(0x0F12 , 0x0639);    //senHal_pContSenModesRegsArray[39][1] 2 7000140C
		mt9t111_write(0x0F12 , 0x038A);    //senHal_pContSenModesRegsArray[39][2] 2 7000140E
		mt9t111_write(0x0F12 , 0x0396);    //senHal_pContSenModesRegsArray[39][3] 2 70001410
		mt9t111_write(0x0F12 , 0x066C);    //senHal_pContSenModesRegsArray[40][0] 2 70001412
		mt9t111_write(0x0F12 , 0x06B7);    //senHal_pContSenModesRegsArray[40][1] 2 70001414
		mt9t111_write(0x0F12 , 0x03C9);    //senHal_pContSenModesRegsArray[40][2] 2 70001416
		mt9t111_write(0x0F12 , 0x0414);    //senHal_pContSenModesRegsArray[40][3] 2 70001418
		mt9t111_write(0x0F12 , 0x087C);    //senHal_pContSenModesRegsArray[41][0] 2 7000141A
		mt9t111_write(0x0F12 , 0x08C7);    //senHal_pContSenModesRegsArray[41][1] 2 7000141C
		mt9t111_write(0x0F12 , 0x04F8);    //senHal_pContSenModesRegsArray[41][2] 2 7000141E
		mt9t111_write(0x0F12 , 0x0543);    //senHal_pContSenModesRegsArray[41][3] 2 70001420
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[42][0] 2 70001422
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[42][1] 2 70001424
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[42][2] 2 70001426
		mt9t111_write(0x0F12 , 0x0040);    //senHal_pContSenModesRegsArray[42][3] 2 70001428
		mt9t111_write(0x0F12 , 0x01D0);    //senHal_pContSenModesRegsArray[43][0] 2 7000142A
		mt9t111_write(0x0F12 , 0x01D0);    //senHal_pContSenModesRegsArray[43][1] 2 7000142C
		mt9t111_write(0x0F12 , 0x00EF);    //senHal_pContSenModesRegsArray[43][2] 2 7000142E
		mt9t111_write(0x0F12 , 0x00EF);    //senHal_pContSenModesRegsArray[43][3] 2 70001430
		mt9t111_write(0x0F12 , 0x020F);    //senHal_pContSenModesRegsArray[44][0] 2 70001432
		mt9t111_write(0x0F12 , 0x024E);    //senHal_pContSenModesRegsArray[44][1] 2 70001434
		mt9t111_write(0x0F12 , 0x012E);    //senHal_pContSenModesRegsArray[44][2] 2 70001436
		mt9t111_write(0x0F12 , 0x016D);    //senHal_pContSenModesRegsArray[44][3] 2 70001438
		mt9t111_write(0x0F12 , 0x039F);    //senHal_pContSenModesRegsArray[45][0] 2 7000143A
		mt9t111_write(0x0F12 , 0x045E);    //senHal_pContSenModesRegsArray[45][1] 2 7000143C
		mt9t111_write(0x0F12 , 0x01DD);    //senHal_pContSenModesRegsArray[45][2] 2 7000143E
		mt9t111_write(0x0F12 , 0x029C);    //senHal_pContSenModesRegsArray[45][3] 2 70001440
		mt9t111_write(0x0F12 , 0x041D);    //senHal_pContSenModesRegsArray[46][0] 2 70001442
		mt9t111_write(0x0F12 , 0x04A9);    //senHal_pContSenModesRegsArray[46][1] 2 70001444
		mt9t111_write(0x0F12 , 0x025B);    //senHal_pContSenModesRegsArray[46][2] 2 70001446
		mt9t111_write(0x0F12 , 0x02E7);    //senHal_pContSenModesRegsArray[46][3] 2 70001448
		mt9t111_write(0x0F12 , 0x062D);    //senHal_pContSenModesRegsArray[47][0] 2 7000144A
		mt9t111_write(0x0F12 , 0x0639);    //senHal_pContSenModesRegsArray[47][1] 2 7000144C
		mt9t111_write(0x0F12 , 0x038A);    //senHal_pContSenModesRegsArray[47][2] 2 7000144E
		mt9t111_write(0x0F12 , 0x0396);    //senHal_pContSenModesRegsArray[47][3] 2 70001450
		mt9t111_write(0x0F12 , 0x066C);    //senHal_pContSenModesRegsArray[48][0] 2 70001452
		mt9t111_write(0x0F12 , 0x06B7);    //senHal_pContSenModesRegsArray[48][1] 2 70001454
		mt9t111_write(0x0F12 , 0x03C9);    //senHal_pContSenModesRegsArray[48][2] 2 70001456
		mt9t111_write(0x0F12 , 0x0414);    //senHal_pContSenModesRegsArray[48][3] 2 70001458
		mt9t111_write(0x0F12 , 0x087C);    //senHal_pContSenModesRegsArray[49][0] 2 7000145A
		mt9t111_write(0x0F12 , 0x08C7);    //senHal_pContSenModesRegsArray[49][1] 2 7000145C
		mt9t111_write(0x0F12 , 0x04F8);    //senHal_pContSenModesRegsArray[49][2] 2 7000145E
		mt9t111_write(0x0F12 , 0x0543);    //senHal_pContSenModesRegsArray[49][3] 2 70001460
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[50][0] 2 70001462
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[50][1] 2 70001464
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[50][2] 2 70001466
		mt9t111_write(0x0F12 , 0x003D);    //senHal_pContSenModesRegsArray[50][3] 2 70001468
		mt9t111_write(0x0F12 , 0x01D2);    //senHal_pContSenModesRegsArray[51][0] 2 7000146A
		mt9t111_write(0x0F12 , 0x01D2);    //senHal_pContSenModesRegsArray[51][1] 2 7000146C
		mt9t111_write(0x0F12 , 0x00F1);    //senHal_pContSenModesRegsArray[51][2] 2 7000146E
		mt9t111_write(0x0F12 , 0x00F1);    //senHal_pContSenModesRegsArray[51][3] 2 70001470
		mt9t111_write(0x0F12 , 0x020C);    //senHal_pContSenModesRegsArray[52][0] 2 70001472
		mt9t111_write(0x0F12 , 0x024B);    //senHal_pContSenModesRegsArray[52][1] 2 70001474
		mt9t111_write(0x0F12 , 0x012B);    //senHal_pContSenModesRegsArray[52][2] 2 70001476
		mt9t111_write(0x0F12 , 0x016A);    //senHal_pContSenModesRegsArray[52][3] 2 70001478
		mt9t111_write(0x0F12 , 0x03A1);    //senHal_pContSenModesRegsArray[53][0] 2 7000147A
		mt9t111_write(0x0F12 , 0x0460);    //senHal_pContSenModesRegsArray[53][1] 2 7000147C
		mt9t111_write(0x0F12 , 0x01DF);    //senHal_pContSenModesRegsArray[53][2] 2 7000147E
		mt9t111_write(0x0F12 , 0x029E);    //senHal_pContSenModesRegsArray[53][3] 2 70001480
		mt9t111_write(0x0F12 , 0x041A);    //senHal_pContSenModesRegsArray[54][0] 2 70001482
		mt9t111_write(0x0F12 , 0x04A6);    //senHal_pContSenModesRegsArray[54][1] 2 70001484
		mt9t111_write(0x0F12 , 0x0258);    //senHal_pContSenModesRegsArray[54][2] 2 70001486
		mt9t111_write(0x0F12 , 0x02E4);    //senHal_pContSenModesRegsArray[54][3] 2 70001488
		mt9t111_write(0x0F12 , 0x062F);    //senHal_pContSenModesRegsArray[55][0] 2 7000148A
		mt9t111_write(0x0F12 , 0x063B);    //senHal_pContSenModesRegsArray[55][1] 2 7000148C
		mt9t111_write(0x0F12 , 0x038C);    //senHal_pContSenModesRegsArray[55][2] 2 7000148E
		mt9t111_write(0x0F12 , 0x0398);    //senHal_pContSenModesRegsArray[55][3] 2 70001490
		mt9t111_write(0x0F12 , 0x0669);    //senHal_pContSenModesRegsArray[56][0] 2 70001492
		mt9t111_write(0x0F12 , 0x06B4);    //senHal_pContSenModesRegsArray[56][1] 2 70001494
		mt9t111_write(0x0F12 , 0x03C6);    //senHal_pContSenModesRegsArray[56][2] 2 70001496
		mt9t111_write(0x0F12 , 0x0411);    //senHal_pContSenModesRegsArray[56][3] 2 70001498
		mt9t111_write(0x0F12 , 0x087E);    //senHal_pContSenModesRegsArray[57][0] 2 7000149A
		mt9t111_write(0x0F12 , 0x08C9);    //senHal_pContSenModesRegsArray[57][1] 2 7000149C
		mt9t111_write(0x0F12 , 0x04FA);    //senHal_pContSenModesRegsArray[57][2] 2 7000149E
		mt9t111_write(0x0F12 , 0x0545);    //senHal_pContSenModesRegsArray[57][3] 2 700014A0
		mt9t111_write(0x0F12 , 0x03A2);    //senHal_pContSenModesRegsArray[58][0] 2 700014A2
		mt9t111_write(0x0F12 , 0x01D3);    //senHal_pContSenModesRegsArray[58][1] 2 700014A4
		mt9t111_write(0x0F12 , 0x01E0);    //senHal_pContSenModesRegsArray[58][2] 2 700014A6
		mt9t111_write(0x0F12 , 0x00F2);    //senHal_pContSenModesRegsArray[58][3] 2 700014A8
		mt9t111_write(0x0F12 , 0x03AF);    //senHal_pContSenModesRegsArray[59][0] 2 700014AA
		mt9t111_write(0x0F12 , 0x01E0);    //senHal_pContSenModesRegsArray[59][1] 2 700014AC
		mt9t111_write(0x0F12 , 0x01ED);    //senHal_pContSenModesRegsArray[59][2] 2 700014AE
		mt9t111_write(0x0F12 , 0x00FF);    //senHal_pContSenModesRegsArray[59][3] 2 700014B0
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[60][0] 2 700014B2
		mt9t111_write(0x0F12 , 0x0461);    //senHal_pContSenModesRegsArray[60][1] 2 700014B4
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[60][2] 2 700014B6
		mt9t111_write(0x0F12 , 0x029F);    //senHal_pContSenModesRegsArray[60][3] 2 700014B8
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[61][0] 2 700014BA
		mt9t111_write(0x0F12 , 0x046E);    //senHal_pContSenModesRegsArray[61][1] 2 700014BC
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[61][2] 2 700014BE
		mt9t111_write(0x0F12 , 0x02AC);    //senHal_pContSenModesRegsArray[61][3] 2 700014C0
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[62][0] 2 700014C2
		mt9t111_write(0x0F12 , 0x063C);    //senHal_pContSenModesRegsArray[62][1] 2 700014C4
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[62][2] 2 700014C6
		mt9t111_write(0x0F12 , 0x0399);    //senHal_pContSenModesRegsArray[62][3] 2 700014C8
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[63][0] 2 700014CA
		mt9t111_write(0x0F12 , 0x0649);    //senHal_pContSenModesRegsArray[63][1] 2 700014CC
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[63][2] 2 700014CE
		mt9t111_write(0x0F12 , 0x03A6);    //senHal_pContSenModesRegsArray[63][3] 2 700014D0
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[64][0] 2 700014D2
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[64][1] 2 700014D4
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[64][2] 2 700014D6
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[64][3] 2 700014D8
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[65][0] 2 700014DA
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[65][1] 2 700014DC
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[65][2] 2 700014DE
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[65][3] 2 700014E0
		mt9t111_write(0x0F12 , 0x03AA);    //senHal_pContSenModesRegsArray[66][0] 2 700014E2
		mt9t111_write(0x0F12 , 0x01DB);    //senHal_pContSenModesRegsArray[66][1] 2 700014E4
		mt9t111_write(0x0F12 , 0x01E8);    //senHal_pContSenModesRegsArray[66][2] 2 700014E6
		mt9t111_write(0x0F12 , 0x00FA);    //senHal_pContSenModesRegsArray[66][3] 2 700014E8
		mt9t111_write(0x0F12 , 0x03B7);    //senHal_pContSenModesRegsArray[67][0] 2 700014EA
		mt9t111_write(0x0F12 , 0x01E8);    //senHal_pContSenModesRegsArray[67][1] 2 700014EC
		mt9t111_write(0x0F12 , 0x01F5);    //senHal_pContSenModesRegsArray[67][2] 2 700014EE
		mt9t111_write(0x0F12 , 0x0107);    //senHal_pContSenModesRegsArray[67][3] 2 700014F0
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[68][0] 2 700014F2
		mt9t111_write(0x0F12 , 0x0469);    //senHal_pContSenModesRegsArray[68][1] 2 700014F4
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[68][2] 2 700014F6
		mt9t111_write(0x0F12 , 0x02A7);    //senHal_pContSenModesRegsArray[68][3] 2 700014F8
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[69][0] 2 700014FA
		mt9t111_write(0x0F12 , 0x0476);    //senHal_pContSenModesRegsArray[69][1] 2 700014FC
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[69][2] 2 700014FE
		mt9t111_write(0x0F12 , 0x02B4);    //senHal_pContSenModesRegsArray[69][3] 2 70001500
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[70][0] 2 70001502
		mt9t111_write(0x0F12 , 0x0644);    //senHal_pContSenModesRegsArray[70][1] 2 70001504
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[70][2] 2 70001506
		mt9t111_write(0x0F12 , 0x03A1);    //senHal_pContSenModesRegsArray[70][3] 2 70001508
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[71][0] 2 7000150A
		mt9t111_write(0x0F12 , 0x0651);    //senHal_pContSenModesRegsArray[71][1] 2 7000150C
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[71][2] 2 7000150E
		mt9t111_write(0x0F12 , 0x03AE);    //senHal_pContSenModesRegsArray[71][3] 2 70001510
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[72][0] 2 70001512
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[72][1] 2 70001514
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[72][2] 2 70001516
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[72][3] 2 70001518
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[73][0] 2 7000151A
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[73][1] 2 7000151C
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[73][2] 2 7000151E
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[73][3] 2 70001520
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[74][0] 2 70001522
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[74][1] 2 70001524
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[74][2] 2 70001526
		mt9t111_write(0x0F12 , 0x0001);    //senHal_pContSenModesRegsArray[74][3] 2 70001528
		mt9t111_write(0x0F12 , 0x000F);    //senHal_pContSenModesRegsArray[75][0] 2 7000152A
		mt9t111_write(0x0F12 , 0x000F);    //senHal_pContSenModesRegsArray[75][1] 2 7000152C
		mt9t111_write(0x0F12 , 0x000F);    //senHal_pContSenModesRegsArray[75][2] 2 7000152E
		mt9t111_write(0x0F12 , 0x000F);    //senHal_pContSenModesRegsArray[75][3] 2 70001530
		mt9t111_write(0x0F12 , 0x05AD);    //senHal_pContSenModesRegsArray[76][0] 2 70001532
		mt9t111_write(0x0F12 , 0x03DE);    //senHal_pContSenModesRegsArray[76][1] 2 70001534
		mt9t111_write(0x0F12 , 0x030A);    //senHal_pContSenModesRegsArray[76][2] 2 70001536
		mt9t111_write(0x0F12 , 0x021C);    //senHal_pContSenModesRegsArray[76][3] 2 70001538
		mt9t111_write(0x0F12 , 0x062F);    //senHal_pContSenModesRegsArray[77][0] 2 7000153A
		mt9t111_write(0x0F12 , 0x0460);    //senHal_pContSenModesRegsArray[77][1] 2 7000153C
		mt9t111_write(0x0F12 , 0x038C);    //senHal_pContSenModesRegsArray[77][2] 2 7000153E
		mt9t111_write(0x0F12 , 0x029E);    //senHal_pContSenModesRegsArray[77][3] 2 70001540
		mt9t111_write(0x0F12 , 0x07FC);    //senHal_pContSenModesRegsArray[78][0] 2 70001542
		mt9t111_write(0x0F12 , 0x0847);    //senHal_pContSenModesRegsArray[78][1] 2 70001544
		mt9t111_write(0x0F12 , 0x0478);    //senHal_pContSenModesRegsArray[78][2] 2 70001546
		mt9t111_write(0x0F12 , 0x04C3);    //senHal_pContSenModesRegsArray[78][3] 2 70001548
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[79][0] 2 7000154A
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[79][1] 2 7000154C
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[79][2] 2 7000154E
		mt9t111_write(0x0F12 , 0x0000);    //senHal_pContSenModesRegsArray[79][3] 2 70001550
		
		//============================================================ 
		// AF Interface setting --- for MCNEX MOTOR DRIVER IC 20110418( Robin )
		//============================================================ 
		mt9t111_write(0x002A , 0x01D4); 
		mt9t111_write(0x0F12 , 0x0000);  //REG_TC_IPRM_AuxGpios : 0 - no Flash
		mt9t111_write(0x002A , 0x01DE); 
		mt9t111_write(0x0F12 , 0x0003);  //REG_TC_IPRM_CM_Init_AfModeType : 3 - AFD_VCM_I2C
		mt9t111_write(0x0F12 , 0x0000);  //REG_TC_IPRM_CM_Init_PwmConfig1 : 0 - no PWM
		mt9t111_write(0x002A , 0x01E4); 
		mt9t111_write(0x0F12 , 0x0051);  //REG_TC_IPRM_CM_Init_GpioConfig1 : 4 -  GPIO4 //20110418( Robin )
		mt9t111_write(0x002A , 0x01E8); 
		mt9t111_write(0x0F12 , 0x200C);  //REG_TC_IPRM_CM_Init_Mi2cBits : MSCL - GPIO1 MSDA - GPIO2 Device ID (0C)//20110418( Robin )
		mt9t111_write(0x0F12 , 0x0190);  //REG_TC_IPRM_CM_Init_Mi2cRateKhz : MI2C Speed - 400KHz
		
		//============================================================ 
		// AF Parameter setting 
		//============================================================ 
		// AF Window Settings
		mt9t111_write(0x002A , 0x025A); 
		mt9t111_write(0x0F12 , 0x0080);  //#REG_TC_AF_FstWinStartX
		mt9t111_write(0x0F12 , 0x0090);  //#REG_TC_AF_FstWinStartY
		mt9t111_write(0x0F12 , 0x0300);  //#REG_TC_AF_FstWinSizeX
		mt9t111_write(0x0F12 , 0x0290);  //#REG_TC_AF_FstWinSizeY
		mt9t111_write(0x0F12 , 0x016C);  //#REG_TC_AF_ScndWinStartX
		mt9t111_write(0x0F12 , 0x0146);  //#REG_TC_AF_ScndWinStartY
		mt9t111_write(0x0F12 , 0x01e6);  //#REG_TC_AF_ScndWinSizeX
		mt9t111_write(0x0F12 , 0x0162);  //#REG_TC_AF_ScndWinSizeY
		mt9t111_write(0x0F12 , 0x0001);  //#REG_TC_AF_WinSizesUpdated
		
		// AF Setot Settings 
		mt9t111_write(0x002A , 0x0586); 
		mt9t111_write(0x0F12 , 0x00FF);  //#skl_af_StatOvlpExpFactor
		
		// AF Scene Settings 
		mt9t111_write(0x002A , 0x115E); 
		mt9t111_write(0x0F12 , 0x0003);  //#af_scene_usSaturatedScene
		
		 // AF Fine Search Settings 
		mt9t111_write(0x002A , 0x10D4); 
		mt9t111_write(0x0F12 , 0x1000);  //FineSearch Disable //#af_search_usSingleAfFlags 1000 jack 20110322
		mt9t111_write(0x002A , 0x10DE); 
		mt9t111_write(0x0F12 , 0x0004);  //#af_search_usFinePeakCount
		mt9t111_write(0x002A , 0x106C); 
		mt9t111_write(0x0F12 , 0x0402);  //#af_pos_usFineStepNumSize // 0202 -> 0402 (yeom)
		
		 // AF Peak Threshold Setting
		mt9t111_write(0x002A , 0x10CA);  //#af_search_usPeakThr
		mt9t111_write(0x0F12 , 0x00C0);  
							 
		
		 // AF Default Position 
		mt9t111_write(0x002A , 0x1060); 
		mt9t111_write(0x0F12 , 0x0000);  //#af_pos_usHomePos
		mt9t111_write(0x0F12 , 0x6C00);  //#af_pos_usLowConfPos
		
		 // AF LowConfThr Setting
		mt9t111_write(0x002A , 0x10F4);  //LowEdgeBoth GRAD
		mt9t111_write(0x0F12 , 0x0280); 
		mt9t111_write(0x002A , 0x1100);  //LowLight HPF
		mt9t111_write(0x0F12 , 0x03A0); 
		mt9t111_write(0x0F12 , 0x0320); 
		
		mt9t111_write(0x002A , 0x1134); 
		mt9t111_write(0x0F12 , 0x0030);  //af_stat_usMinStatVal
		
		// AF low Br Th
		mt9t111_write(0x002A , 0x1154);  //normBrThr
		mt9t111_write(0x0F12 , 0x0060); 
		
		// AF Policy
		mt9t111_write(0x002A , 0x10E2); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x002A , 0x1072); 
		mt9t111_write(0x0F12 , 0x003C);  //#af_pos_usCaptureFixedPo
		
		// AF Lens Position Table Settings 
		
		mt9t111_write(0x002A , 0x1074); 
		mt9t111_write(0x0F12 , 0x0012);  // 13 -> 12 (yeom)
		mt9t111_write(0x0F12 , 0x0015); 
		mt9t111_write(0x0F12 , 0x0021);  
		mt9t111_write(0x0F12 , 0x0031);  
		mt9t111_write(0x0F12 , 0x0035);  
		mt9t111_write(0x0F12 , 0x0036);  
		mt9t111_write(0x0F12 , 0x003a);  
		mt9t111_write(0x0F12 , 0x0040);  
		mt9t111_write(0x0F12 , 0x0042);  
		mt9t111_write(0x0F12 , 0x0045);  
		mt9t111_write(0x0F12 , 0x0047);  
		mt9t111_write(0x0F12 , 0x004a);  
		mt9t111_write(0x0F12 , 0x004f);  
		mt9t111_write(0x0F12 , 0x0050);  
		mt9t111_write(0x0F12 , 0x0051);  
		mt9t111_write(0x0F12 , 0x0055);  
		mt9t111_write(0x0F12 , 0x0060);  
		mt9t111_write(0x0F12 , 0x0065);  
		mt9t111_write(0x0F12 , 0x007B);  
		mt9t111_write(0x0F12 , 0x0087);  
							 // b3 deleted (yeom)
		
		mt9t111_write(0x002A , 0x1198); 
		mt9t111_write(0x0F12 , 0x8000); 
		mt9t111_write(0x0F12 , 0x0006);  
		mt9t111_write(0x0F12 , 0x3FF0); 
		mt9t111_write(0x0F12 , 0x03E8); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x0F12 , 0x0020); 
		mt9t111_write(0x0F12 , 0x0010); 
		mt9t111_write(0x0F12 , 0x0008); 
		mt9t111_write(0x0F12 , 0x0040);  
		mt9t111_write(0x0F12 , 0x0080); 
		mt9t111_write(0x0F12 , 0x00C0); 
		mt9t111_write(0x0F12 , 0x00E0); 
		
		mt9t111_write(0x002A , 0x0252); 
		mt9t111_write(0x0F12 , 0x0003);  //init 
		
		mt9t111_write(0x002A , 0x12B8);  //disable CINTR 0
		mt9t111_write(0x0F12 , 0x1000); 
		
		//============================================================
		// ISP-FE Setting
		//============================================================
		mt9t111_write(0x002A , 0x158A); 
		mt9t111_write(0x0F12 , 0xEAF0); 
		mt9t111_write(0x002A , 0x15C6); 
		mt9t111_write(0x0F12 , 0x0020); 
		mt9t111_write(0x0F12 , 0x0060); 
		mt9t111_write(0x002A , 0x15BC); 
		mt9t111_write(0x0F12 , 0x0200); 
		
		mt9t111_write(0x002A , 0x1608); 
		mt9t111_write(0x0F12 , 0x0100); 
		mt9t111_write(0x0F12 , 0x0100); 
		mt9t111_write(0x0F12 , 0x0100); 
		mt9t111_write(0x0F12 , 0x0100); 
		
		mt9t111_write(0x002A , 0x0F70); 
		mt9t111_write(0x0F12 , 0x003F); 	 //3E 40 36 //TVAR_ae_BrAve  //ae Target//
		mt9t111_write(0x002A , 0x0530); 
		mt9t111_write(0x0F12 , 0x3415);    //3A98 //3A98////lt_uMaxExp1 32 30ms  9~10ea// 15fps  //
		mt9t111_write(0x002A , 0x0534); 
		mt9t111_write(0x0F12 , 0x682A);    //68b0 //7EF4////lt_uMaxExp2 67 65ms 18~20ea // 7.5fps //
		mt9t111_write(0x002A , 0x167C); 
		mt9t111_write(0x0F12 , 0x8235);    //8340 //9C40//MaxExp3  83 80ms	24~25ea //
		mt9t111_write(0x002A , 0x1680); 
		mt9t111_write(0x0F12 , 0x8235);    // F424 //MaxExp4   125ms  38ea // 1219 0xc350-> 0x8235
		
		mt9t111_write(0x002A , 0x0538); 
		mt9t111_write(0x0F12 , 0x3415);    // 15fps //
		mt9t111_write(0x002A , 0x053C); 
		mt9t111_write(0x0F12 , 0x682A);    // 7.5fps //
		mt9t111_write(0x002A , 0x1684); 
		mt9t111_write(0x0F12 , 0x8235);    // CapMaxExp3 //
		mt9t111_write(0x002A , 0x1688); 
		mt9t111_write(0x0F12 , 0xC350);    // CapMaxExp4 //
		
		mt9t111_write(0x002A , 0x0540); 
		mt9t111_write(0x0F12 , 0x01B3); 	//0170//0150//lt_uMaxAnGain1_700lux// 1219 0x01E3->01B3 
		mt9t111_write(0x0F12 , 0x0350);    //0200//0400//lt_uMaxAnGain2_400lux//  1219 0x01E3->0350 
		mt9t111_write(0x002A , 0x168C); 
		mt9t111_write(0x0F12 , 0x0400);    //0300//MaxAnGain3_200lux// 1219 0x02E0->0x0400	 
		mt9t111_write(0x0F12 , 0x0710);    //MaxAnGain4 //
							 
		
		mt9t111_write(0x002A , 0x0544); 
		mt9t111_write(0x0F12 , 0x0100); 
		mt9t111_write(0x0F12 , 0x8000);    // Max Gain 8 //
		
		
		mt9t111_write(0x002A , 0x1694); 
		mt9t111_write(0x0F12 , 0x0001);    // expand forbidde zone //
		
		mt9t111_write(0x002A , 0x021A); 
		mt9t111_write(0x0F12 , 0x0000);    // MBR off //
		
		
		//==============================================//
		//AFC											//
		//==============================================//
		mt9t111_write(0x002A , 0x04d2); 
		mt9t111_write(0x0F12 , 0x065f); 	 // 065f : Manual AFC on   067f : Manual AFC off //
		mt9t111_write(0x002A , 0x04ba); 
		mt9t111_write(0x0F12 , 0x0001); 	 // 0002: 60hz	0001 : 50hz //
		mt9t111_write(0x0F12 , 0x0001); 	 // afc update command //
		
		mt9t111_write(0x002A , 0x06CE);    //JSK : AWB
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[0]  //H	  //R
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[1] //	//Gr
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[2] //	 //Gb
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[3] //	//B
		
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[4]  //A	  // 
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[5] // 
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[6] // 
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[7] // 
		
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[8]  //WW   // 
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[9] // 
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[10] //
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[11] //
		
		mt9t111_write(0x0F12 , 0x00D0);    //TVAR_ash_GAsalpha[16] //D50  
		mt9t111_write(0x0F12 , 0x00f8);    //TVAR_ash_GAsalpha[13] //
		mt9t111_write(0x0F12 , 0x00f8);    //TVAR_ash_GAsalpha[14] //
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[15] //
		
		mt9t111_write(0x0F12 , 0x00D8);    //TVAR_ash_GAsalpha[16] //D50  
		mt9t111_write(0x0F12 , 0x00f8);    //TVAR_ash_GAsalpha[17] //
		mt9t111_write(0x0F12 , 0x00f8);    //TVAR_ash_GAsalpha[18] //
		mt9t111_write(0x0F12 , 0x0110);    //TVAR_ash_GAsalpha[19] //
		
		mt9t111_write(0x0F12 , 0x00E0);    //TVAR_ash_GAsalpha[20] //D65  
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[21] //
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[22] //
		mt9t111_write(0x0F12 , 0x0118);    //TVAR_ash_GAsalpha[23] //
		
		mt9t111_write(0x0F12 , 0x00E0);    //TVAR_ash_GAsalpha[24] //D75  
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[25] //
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAsalpha[26] //
		mt9t111_write(0x0F12 , 0x0118);    //TVAR_ash_GAsalpha[27] //
		
		mt9t111_write(0x0F12 , 0x00E8); 	//TVAR_ash_GAS OutdoorAlpha[0] //
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAS OutdoorAlpha[1] //
		mt9t111_write(0x0F12 , 0x0100);    //TVAR_ash_GAS OutdoorAlpha[2] //
		mt9t111_write(0x0F12 , 0x0120);    //TVAR_ash_GAS OutdoorAlpha[3] //
		
		
		mt9t111_write(0x0F12 , 0x0036); 	//ash_GASBeta[0] //
		mt9t111_write(0x0F12 , 0x001F); 	//ash_GASBeta[1] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[2] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[3] //
		mt9t111_write(0x0F12 , 0x0036);    //ash_GASBeta[4] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GASBeta[5] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[6] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[7] //
		mt9t111_write(0x0F12 , 0x0036);    //ash_GASBeta[8] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GASBeta[9] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[10] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[11] //
		mt9t111_write(0x0F12 , 0x0010);    //ash_GASBeta[12] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GASBeta[13] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[14] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[15] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[16] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GASBeta[17] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[18] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[19] //
		mt9t111_write(0x0F12 , 0x0036);    //ash_GASBeta[20] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GASBeta[21] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[22] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[23] //
		mt9t111_write(0x0F12 , 0x0036);    //ash_GASBeta[24] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GASBeta[25] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GASBeta[26] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GASBeta[27] //
		
		mt9t111_write(0x0F12 , 0x0036); 	 //ash_GAS OutdoorBeta[0] //
		mt9t111_write(0x0F12 , 0x001F);    //ash_GAS OutdoorBeta[1] //
		mt9t111_write(0x0F12 , 0x0020);    //ash_GAS OutdoorBeta[2] //
		mt9t111_write(0x0F12 , 0x0000);    //ash_GAS OutdoorBeta[3] //
		
		mt9t111_write(0x002A , 0x075A); 	 //ash_bParabolicEstimation//
		mt9t111_write(0x0F12 , 0x0000);    //ash_uParabolicCenterX	 //
		mt9t111_write(0x0F12 , 0x0400);    //ash_uParabolicCenterY	 //
		mt9t111_write(0x0F12 , 0x0300);    //ash_uParabolicscalingA  //
		mt9t111_write(0x0F12 , 0x0010);    //ash_uParabolicscalingB  //
		mt9t111_write(0x0F12 , 0x0011); 
		
		mt9t111_write(0x002A , 0x347C); 
		mt9t111_write(0x0F12 , 0x020D); 	//01D9	//TVAR_ash_pGAS[0] //
		mt9t111_write(0x0F12 , 0x01C8); 	//019D	 //TVAR_ash_pGAS[1] //	
		mt9t111_write(0x0F12 , 0x0187); 	//015C	 //TVAR_ash_pGAS[2] //	
		mt9t111_write(0x0F12 , 0x014E); 	//0125	 //TVAR_ash_pGAS[3] //	
		mt9t111_write(0x0F12 , 0x0123); 	//00FE	 //TVAR_ash_pGAS[4] //	
		mt9t111_write(0x0F12 , 0x0103); 	//00E5	 //TVAR_ash_pGAS[5] //	
		mt9t111_write(0x0F12 , 0x00ED); 	//00DA	 //TVAR_ash_pGAS[6] //	
		mt9t111_write(0x0F12 , 0x00E7); 	//00E5	 //TVAR_ash_pGAS[7] //	
		mt9t111_write(0x0F12 , 0x00F1); 	//0100	 //TVAR_ash_pGAS[8] //	
		mt9t111_write(0x0F12 , 0x010B); 	//012D	 //TVAR_ash_pGAS[9] //	
		mt9t111_write(0x0F12 , 0x0133); 	//016B	 //TVAR_ash_pGAS[10] // 
		mt9t111_write(0x0F12 , 0x016A); 	//01B3	 //TVAR_ash_pGAS[11] // 
		mt9t111_write(0x0F12 , 0x01A6); 	//01F2	 //TVAR_ash_pGAS[12] // 
		mt9t111_write(0x0F12 , 0x01D2); 	//01A7	 //TVAR_ash_pGAS[13] // 
		mt9t111_write(0x0F12 , 0x0188); 	//0165	 //TVAR_ash_pGAS[14] // 
		mt9t111_write(0x0F12 , 0x0140); 	//011E	 //TVAR_ash_pGAS[15] // 
		mt9t111_write(0x0F12 , 0x0105); 	//00E3	 //TVAR_ash_pGAS[16] // 
		mt9t111_write(0x0F12 , 0x00D7); 	//00B6	 //TVAR_ash_pGAS[17] // 
		mt9t111_write(0x0F12 , 0x00B6); 	//009C	 //TVAR_ash_pGAS[18] // 
		mt9t111_write(0x0F12 , 0x00A0); 	//0092	 //TVAR_ash_pGAS[19] // 
		mt9t111_write(0x0F12 , 0x009C); 	//009D	 //TVAR_ash_pGAS[20] // 
		mt9t111_write(0x0F12 , 0x00A8); 	//00BB	 //TVAR_ash_pGAS[21] // 
		mt9t111_write(0x0F12 , 0x00CA); 	//00F0	 //TVAR_ash_pGAS[22] // 
		mt9t111_write(0x0F12 , 0x00F5); 	//0133	 //TVAR_ash_pGAS[23] // 
		mt9t111_write(0x0F12 , 0x012E); 	//0182	 //TVAR_ash_pGAS[24] // 
		mt9t111_write(0x0F12 , 0x0165); 	//01CD	 //TVAR_ash_pGAS[25] // 
		mt9t111_write(0x0F12 , 0x0197); 	//0170	 //TVAR_ash_pGAS[26] // 
		mt9t111_write(0x0F12 , 0x014E); 	//012A	 //TVAR_ash_pGAS[27] // 
		mt9t111_write(0x0F12 , 0x0102); 	//00DC	 //TVAR_ash_pGAS[28] // 
		mt9t111_write(0x0F12 , 0x00C3); 	//009A	 //TVAR_ash_pGAS[29] // 
		mt9t111_write(0x0F12 , 0x0091); 	//006E	 //TVAR_ash_pGAS[30] // 
		mt9t111_write(0x0F12 , 0x0070); 	//0053	 //TVAR_ash_pGAS[31] // 
		mt9t111_write(0x0F12 , 0x005B); 	//004A	 //TVAR_ash_pGAS[32] // 
		mt9t111_write(0x0F12 , 0x005A); 	//0055	 //TVAR_ash_pGAS[33] // 
		mt9t111_write(0x0F12 , 0x0069); 	//0076	 //TVAR_ash_pGAS[34] // 
		mt9t111_write(0x0F12 , 0x0090); 	//00AC	 //TVAR_ash_pGAS[35] // 
		mt9t111_write(0x0F12 , 0x00C0); 	//00F5	 //TVAR_ash_pGAS[36] // 
		mt9t111_write(0x0F12 , 0x0101); 	//0147	 //TVAR_ash_pGAS[37] // 
		mt9t111_write(0x0F12 , 0x013B); 	//0196	 //TVAR_ash_pGAS[38] // 
		mt9t111_write(0x0F12 , 0x015A); 	//014C	 //TVAR_ash_pGAS[39] // 
		mt9t111_write(0x0F12 , 0x010F); 	//0102	 //TVAR_ash_pGAS[40] // 
		mt9t111_write(0x0F12 , 0x00C2); 	//00B1	 //TVAR_ash_pGAS[41] // 
		mt9t111_write(0x0F12 , 0x0082); 	//006F	 //TVAR_ash_pGAS[42] // 
		mt9t111_write(0x0F12 , 0x0056); 	//0041	 //TVAR_ash_pGAS[43] // 
		mt9t111_write(0x0F12 , 0x0036); 	//0027	 //TVAR_ash_pGAS[44] // 
		mt9t111_write(0x0F12 , 0x0026); 	//001F	 //TVAR_ash_pGAS[45] // 
		mt9t111_write(0x0F12 , 0x0025); 	//002A	 //TVAR_ash_pGAS[46] // 
		mt9t111_write(0x0F12 , 0x0036); 	//004B	 //TVAR_ash_pGAS[47] // 
		mt9t111_write(0x0F12 , 0x005E); 	//0083	 //TVAR_ash_pGAS[48] // 
		mt9t111_write(0x0F12 , 0x0093); 	//00CE	 //TVAR_ash_pGAS[49] // 
		mt9t111_write(0x0F12 , 0x00D2); 	//0128	 //TVAR_ash_pGAS[50] // 
		mt9t111_write(0x0F12 , 0x0111); 	//0177	 //TVAR_ash_pGAS[51] // 
		mt9t111_write(0x0F12 , 0x013D); 	//0133	 //TVAR_ash_pGAS[52] // 
		mt9t111_write(0x0F12 , 0x00F2); 	//00E6	 //TVAR_ash_pGAS[53] // 
		mt9t111_write(0x0F12 , 0x00A6); 	//0094	 //TVAR_ash_pGAS[54] // 
		mt9t111_write(0x0F12 , 0x0064); 	//0052	 //TVAR_ash_pGAS[55] // 
		mt9t111_write(0x0F12 , 0x0035); 	//0025	 //TVAR_ash_pGAS[56] // 
		mt9t111_write(0x0F12 , 0x0017); 	//000C	 //TVAR_ash_pGAS[57] // 
		mt9t111_write(0x0F12 , 0x0008); 	//0004	 //TVAR_ash_pGAS[58] // 
		mt9t111_write(0x0F12 , 0x0009); 	//0010	 //TVAR_ash_pGAS[59] // 
		mt9t111_write(0x0F12 , 0x001B); 	//0030	 //TVAR_ash_pGAS[60] // 
		mt9t111_write(0x0F12 , 0x0040); 	//0069	 //TVAR_ash_pGAS[61] // 
		mt9t111_write(0x0F12 , 0x0076); 	//00B6	 //TVAR_ash_pGAS[62] // 
		mt9t111_write(0x0F12 , 0x00BC); 	//0112	 //TVAR_ash_pGAS[63] // 
		mt9t111_write(0x0F12 , 0x00FC); 	//0168	 //TVAR_ash_pGAS[64] // 
		mt9t111_write(0x0F12 , 0x0134); 	//012F	 //TVAR_ash_pGAS[65] // 
		mt9t111_write(0x0F12 , 0x00E6); 	//00E3	 //TVAR_ash_pGAS[66] // 
		mt9t111_write(0x0F12 , 0x0097); 	//008E	 //TVAR_ash_pGAS[67] // 
		mt9t111_write(0x0F12 , 0x0058); 	//004C	 //TVAR_ash_pGAS[68] // 
		mt9t111_write(0x0F12 , 0x0029); 	//0020	 //TVAR_ash_pGAS[69] // 
		mt9t111_write(0x0F12 , 0x000D); 	//0007	 //TVAR_ash_pGAS[70] // 
		mt9t111_write(0x0F12 , 0x0000); 	//0000	 //TVAR_ash_pGAS[71] // 
		mt9t111_write(0x0F12 , 0x0002); 	//000B	 //TVAR_ash_pGAS[72] // 
		mt9t111_write(0x0F12 , 0x0012); 	//002D	 //TVAR_ash_pGAS[73] // 
		mt9t111_write(0x0F12 , 0x0035); 	//0065	 //TVAR_ash_pGAS[74] // 
		mt9t111_write(0x0F12 , 0x0069); 	//00B4	 //TVAR_ash_pGAS[75] // 
		mt9t111_write(0x0F12 , 0x00B7); 	//0114	 //TVAR_ash_pGAS[76] // 
		mt9t111_write(0x0F12 , 0x00F9); 	//016B	 //TVAR_ash_pGAS[77] // 
		mt9t111_write(0x0F12 , 0x0136); 	//0138	 //TVAR_ash_pGAS[78] // 
		mt9t111_write(0x0F12 , 0x00E7); 	//00EB	 //TVAR_ash_pGAS[79] // 
		mt9t111_write(0x0F12 , 0x009A); 	//0099	 //TVAR_ash_pGAS[80] // 
		mt9t111_write(0x0F12 , 0x005C); 	//0058	 //TVAR_ash_pGAS[81] // 
		mt9t111_write(0x0F12 , 0x002D); 	//002B	 //TVAR_ash_pGAS[82] // 
		mt9t111_write(0x0F12 , 0x0012); 	//0012	 //TVAR_ash_pGAS[83] // 
		mt9t111_write(0x0F12 , 0x0006); 	//000B	 //TVAR_ash_pGAS[84] // 
		mt9t111_write(0x0F12 , 0x0008); 	//0017	 //TVAR_ash_pGAS[85] // 
		mt9t111_write(0x0F12 , 0x0018); 	//0039	 //TVAR_ash_pGAS[86] // 
		mt9t111_write(0x0F12 , 0x003B); 	//0074	 //TVAR_ash_pGAS[87] // 
		mt9t111_write(0x0F12 , 0x0070); 	//00C2	 //TVAR_ash_pGAS[88] // 
		mt9t111_write(0x0F12 , 0x00B7); 	//0121	 //TVAR_ash_pGAS[89] // 
		mt9t111_write(0x0F12 , 0x00FB); 	//0177	 //TVAR_ash_pGAS[90] // 
		mt9t111_write(0x0F12 , 0x014F); 	//0158	 //TVAR_ash_pGAS[91] // 
		mt9t111_write(0x0F12 , 0x00FC); 	//010C	 //TVAR_ash_pGAS[92] // 
		mt9t111_write(0x0F12 , 0x00AF); 	//00BC	 //TVAR_ash_pGAS[93] // 
		mt9t111_write(0x0F12 , 0x0072); 	//007B	 //TVAR_ash_pGAS[94] // 
		mt9t111_write(0x0F12 , 0x0044); 	//004E	 //TVAR_ash_pGAS[95] // 
		mt9t111_write(0x0F12 , 0x0029); 	//0034	 //TVAR_ash_pGAS[96] // 
		mt9t111_write(0x0F12 , 0x001A); 	//002D	 //TVAR_ash_pGAS[97] // 
		mt9t111_write(0x0F12 , 0x001D); 	//003B	 //TVAR_ash_pGAS[98] // 
		mt9t111_write(0x0F12 , 0x002D); 	//005E	 //TVAR_ash_pGAS[99] // 
		mt9t111_write(0x0F12 , 0x0051); 	//0099	 //TVAR_ash_pGAS[100] //
		mt9t111_write(0x0F12 , 0x0089); 	//00E7	 //TVAR_ash_pGAS[101] //
		mt9t111_write(0x0F12 , 0x00CD); 	//0145	 //TVAR_ash_pGAS[102] //
		mt9t111_write(0x0F12 , 0x010D); 	//0197	 //TVAR_ash_pGAS[103] //
		mt9t111_write(0x0F12 , 0x016C); 	//017E	 //TVAR_ash_pGAS[104] //
		mt9t111_write(0x0F12 , 0x0120); 	//0138	 //TVAR_ash_pGAS[105] //
		mt9t111_write(0x0F12 , 0x00D3); 	//00EB	 //TVAR_ash_pGAS[106] //
		mt9t111_write(0x0F12 , 0x0097); 	//00AC	 //TVAR_ash_pGAS[107] //
		mt9t111_write(0x0F12 , 0x006B); 	//0080	 //TVAR_ash_pGAS[108] //
		mt9t111_write(0x0F12 , 0x004C); 	//0067	 //TVAR_ash_pGAS[109] //
		mt9t111_write(0x0F12 , 0x003B); 	//0061	 //TVAR_ash_pGAS[110] //
		mt9t111_write(0x0F12 , 0x003D); 	//006E	 //TVAR_ash_pGAS[111] //
		mt9t111_write(0x0F12 , 0x0052); 	//0091	 //TVAR_ash_pGAS[112] //
		mt9t111_write(0x0F12 , 0x0078); 	//00CA	 //TVAR_ash_pGAS[113] //
		mt9t111_write(0x0F12 , 0x00A8); 	//0116	 //TVAR_ash_pGAS[114] //
		mt9t111_write(0x0F12 , 0x00EC); 	//016E	 //TVAR_ash_pGAS[115] //
		mt9t111_write(0x0F12 , 0x012D); 	//01C0	 //TVAR_ash_pGAS[116] //
		mt9t111_write(0x0F12 , 0x019F); 	//01BB	 //TVAR_ash_pGAS[117] //
		mt9t111_write(0x0F12 , 0x014F); 	//0177	 //TVAR_ash_pGAS[118] //
		mt9t111_write(0x0F12 , 0x0102); 	//0131	 //TVAR_ash_pGAS[119] //
		mt9t111_write(0x0F12 , 0x00CC); 	//00F7	 //TVAR_ash_pGAS[120] //
		mt9t111_write(0x0F12 , 0x00A1); 	//00CE	 //TVAR_ash_pGAS[121] //
		mt9t111_write(0x0F12 , 0x0087); 	//00B6	 //TVAR_ash_pGAS[122] //
		mt9t111_write(0x0F12 , 0x0074); 	//00AF	 //TVAR_ash_pGAS[123] //
		mt9t111_write(0x0F12 , 0x0076); 	//00BD	 //TVAR_ash_pGAS[124] //
		mt9t111_write(0x0F12 , 0x0087); 	//00DF	 //TVAR_ash_pGAS[125] //
		mt9t111_write(0x0F12 , 0x00AB); 	//0113	 //TVAR_ash_pGAS[126] //
		mt9t111_write(0x0F12 , 0x00DC); 	//0156	 //TVAR_ash_pGAS[127] //
		mt9t111_write(0x0F12 , 0x0119); 	//01AA	 //TVAR_ash_pGAS[128] //
		mt9t111_write(0x0F12 , 0x015A); 	//01F7	 //TVAR_ash_pGAS[129] //
		mt9t111_write(0x0F12 , 0x01C8); 	//01EF	 //TVAR_ash_pGAS[130] //
		mt9t111_write(0x0F12 , 0x0177); 	//01B2	 //TVAR_ash_pGAS[131] //
		mt9t111_write(0x0F12 , 0x0138); 	//0170	 //TVAR_ash_pGAS[132] //
		mt9t111_write(0x0F12 , 0x0107); 	//013E	 //TVAR_ash_pGAS[133] //
		mt9t111_write(0x0F12 , 0x00E3); 	//0119	 //TVAR_ash_pGAS[134] //
		mt9t111_write(0x0F12 , 0x00C7); 	//0103	 //TVAR_ash_pGAS[135] //
		mt9t111_write(0x0F12 , 0x00B6); 	//00FF	 //TVAR_ash_pGAS[136] //
		mt9t111_write(0x0F12 , 0x00B5); 	//010D	 //TVAR_ash_pGAS[137] //
		mt9t111_write(0x0F12 , 0x00C3); 	//012B	 //TVAR_ash_pGAS[138] //
		mt9t111_write(0x0F12 , 0x00E6); 	//0158	 //TVAR_ash_pGAS[139] //
		mt9t111_write(0x0F12 , 0x010D); 	//0194	 //TVAR_ash_pGAS[140] //
		mt9t111_write(0x0F12 , 0x014D); 	//01DA	 //TVAR_ash_pGAS[141] //
		mt9t111_write(0x0F12 , 0x018C); 	//0207	 //TVAR_ash_pGAS[142] //
		mt9t111_write(0x0F12 , 0x01A8); 	//0198	 //TVAR_ash_pGAS[143] //
		mt9t111_write(0x0F12 , 0x0167); 	//0156	 //TVAR_ash_pGAS[144] //
		mt9t111_write(0x0F12 , 0x0130); 	//011D	 //TVAR_ash_pGAS[145] //
		mt9t111_write(0x0F12 , 0x0103); 	//00EF	 //TVAR_ash_pGAS[146] //
		mt9t111_write(0x0F12 , 0x00DF); 	//00C9	 //TVAR_ash_pGAS[147] //
		mt9t111_write(0x0F12 , 0x00C4); 	//00B2	 //TVAR_ash_pGAS[148] //
		mt9t111_write(0x0F12 , 0x00B4); 	//00A9	 //TVAR_ash_pGAS[149] //
		mt9t111_write(0x0F12 , 0x00B0); 	//00B1	 //TVAR_ash_pGAS[150] //
		mt9t111_write(0x0F12 , 0x00B7); 	//00C5	 //TVAR_ash_pGAS[151] //
		mt9t111_write(0x0F12 , 0x00C9); 	//00E7	 //TVAR_ash_pGAS[152] //
		mt9t111_write(0x0F12 , 0x00E5); 	//0113	 //TVAR_ash_pGAS[153] //
		mt9t111_write(0x0F12 , 0x010E); 	//0152	 //TVAR_ash_pGAS[154] //
		mt9t111_write(0x0F12 , 0x0144); 	//018C	 //TVAR_ash_pGAS[155] //
		mt9t111_write(0x0F12 , 0x0173); 	//016E	 //TVAR_ash_pGAS[156] //
		mt9t111_write(0x0F12 , 0x0131); 	//0127	 //TVAR_ash_pGAS[157] //
		mt9t111_write(0x0F12 , 0x00F7); 	//00E8	 //TVAR_ash_pGAS[158] //
		mt9t111_write(0x0F12 , 0x00C7); 	//00B6	 //TVAR_ash_pGAS[159] //
		mt9t111_write(0x0F12 , 0x00A0); 	//0090	 //TVAR_ash_pGAS[160] //
		mt9t111_write(0x0F12 , 0x0084); 	//0078	 //TVAR_ash_pGAS[161] //
		mt9t111_write(0x0F12 , 0x0074); 	//0070	 //TVAR_ash_pGAS[162] //
		mt9t111_write(0x0F12 , 0x0072); 	//0078	 //TVAR_ash_pGAS[163] //
		mt9t111_write(0x0F12 , 0x007C); 	//008F	 //TVAR_ash_pGAS[164] //
		mt9t111_write(0x0F12 , 0x0092); 	//00B5	 //TVAR_ash_pGAS[165] //
		mt9t111_write(0x0F12 , 0x00B3); 	//00E7	 //TVAR_ash_pGAS[166] //
		mt9t111_write(0x0F12 , 0x00DD); 	//0126	 //TVAR_ash_pGAS[167] //
		mt9t111_write(0x0F12 , 0x010E); 	//016C	 //TVAR_ash_pGAS[168] //
		mt9t111_write(0x0F12 , 0x0148); 	//013D	 //TVAR_ash_pGAS[169] //
		mt9t111_write(0x0F12 , 0x0107); 	//00F6	 //TVAR_ash_pGAS[170] //
		mt9t111_write(0x0F12 , 0x00C9); 	//00B5	 //TVAR_ash_pGAS[171] //
		mt9t111_write(0x0F12 , 0x0096); 	//0080	 //TVAR_ash_pGAS[172] //
		mt9t111_write(0x0F12 , 0x006D); 	//0058	 //TVAR_ash_pGAS[173] //
		mt9t111_write(0x0F12 , 0x0051); 	//0040	 //TVAR_ash_pGAS[174] //
		mt9t111_write(0x0F12 , 0x0044); 	//0038	 //TVAR_ash_pGAS[175] //
		mt9t111_write(0x0F12 , 0x0042); 	//0042	 //TVAR_ash_pGAS[176] //
		mt9t111_write(0x0F12 , 0x004E); 	//005B	 //TVAR_ash_pGAS[177] //
		mt9t111_write(0x0F12 , 0x006A); 	//0083	 //TVAR_ash_pGAS[178] //
		mt9t111_write(0x0F12 , 0x008E); 	//00B9	 //TVAR_ash_pGAS[179] //
		mt9t111_write(0x0F12 , 0x00BE); 	//00F9	 //TVAR_ash_pGAS[180] //
		mt9t111_write(0x0F12 , 0x00EE); 	//0141	 //TVAR_ash_pGAS[181] //
		mt9t111_write(0x0F12 , 0x0114); 	//011E	 //TVAR_ash_pGAS[182] //
		mt9t111_write(0x0F12 , 0x00D4); 	//00D7	 //TVAR_ash_pGAS[183] //
		mt9t111_write(0x0F12 , 0x0098); 	//0094	 //TVAR_ash_pGAS[184] //
		mt9t111_write(0x0F12 , 0x0063); 	//005E	 //TVAR_ash_pGAS[185] //
		mt9t111_write(0x0F12 , 0x0040); 	//0035	 //TVAR_ash_pGAS[186] //
		mt9t111_write(0x0F12 , 0x0025); 	//001E	 //TVAR_ash_pGAS[187] //
		mt9t111_write(0x0F12 , 0x0018); 	//0017	 //TVAR_ash_pGAS[188] //
		mt9t111_write(0x0F12 , 0x0019); 	//0021	 //TVAR_ash_pGAS[189] //
		mt9t111_write(0x0F12 , 0x0027); 	//003C	 //TVAR_ash_pGAS[190] //
		mt9t111_write(0x0F12 , 0x0045); 	//0066	 //TVAR_ash_pGAS[191] //
		mt9t111_write(0x0F12 , 0x006C); 	//009F	 //TVAR_ash_pGAS[192] //
		mt9t111_write(0x0F12 , 0x009D); 	//00E2	 //TVAR_ash_pGAS[193] //
		mt9t111_write(0x0F12 , 0x00CE); 	//012A	 //TVAR_ash_pGAS[194] //
		mt9t111_write(0x0F12 , 0x00FB); 	//010A	 //TVAR_ash_pGAS[195] //
		mt9t111_write(0x0F12 , 0x00BA); 	//00C0	 //TVAR_ash_pGAS[196] //
		mt9t111_write(0x0F12 , 0x007F); 	//007B	 //TVAR_ash_pGAS[197] //
		mt9t111_write(0x0F12 , 0x004C); 	//0045	 //TVAR_ash_pGAS[198] //
		mt9t111_write(0x0F12 , 0x0028); 	//001F	 //TVAR_ash_pGAS[199] //
		mt9t111_write(0x0F12 , 0x000F); 	//0008	 //TVAR_ash_pGAS[200] //
		mt9t111_write(0x0F12 , 0x0004); 	//0002	 //TVAR_ash_pGAS[201] //
		mt9t111_write(0x0F12 , 0x0005); 	//000C	 //TVAR_ash_pGAS[202] //
		mt9t111_write(0x0F12 , 0x0014); 	//0027	 //TVAR_ash_pGAS[203] //
		mt9t111_write(0x0F12 , 0x002F); 	//0052	 //TVAR_ash_pGAS[204] //
		mt9t111_write(0x0F12 , 0x0058); 	//008C	 //TVAR_ash_pGAS[205] //
		mt9t111_write(0x0F12 , 0x008C); 	//00D2	 //TVAR_ash_pGAS[206] //
		mt9t111_write(0x0F12 , 0x00C1); 	//011E	 //TVAR_ash_pGAS[207] //
		mt9t111_write(0x0F12 , 0x00F3); 	//0106	 //TVAR_ash_pGAS[208] //
		mt9t111_write(0x0F12 , 0x00B2); 	//00BC	 //TVAR_ash_pGAS[209] //
		mt9t111_write(0x0F12 , 0x0075); 	//0077	 //TVAR_ash_pGAS[210] //
		mt9t111_write(0x0F12 , 0x0043); 	//0040	 //TVAR_ash_pGAS[211] //
		mt9t111_write(0x0F12 , 0x001F); 	//001A	 //TVAR_ash_pGAS[212] //
		mt9t111_write(0x0F12 , 0x0008); 	//0005	 //TVAR_ash_pGAS[213] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000	 //TVAR_ash_pGAS[214] //
		mt9t111_write(0x0F12 , 0x0003); 	//000A	 //TVAR_ash_pGAS[215] //
		mt9t111_write(0x0F12 , 0x000F); 	//0026	 //TVAR_ash_pGAS[216] //
		mt9t111_write(0x0F12 , 0x0029); 	//0052	 //TVAR_ash_pGAS[217] //
		mt9t111_write(0x0F12 , 0x0052); 	//008D	 //TVAR_ash_pGAS[218] //
		mt9t111_write(0x0F12 , 0x008B); 	//00D5	 //TVAR_ash_pGAS[219] //
		mt9t111_write(0x0F12 , 0x00BF); 	//0121	 //TVAR_ash_pGAS[220] //
		mt9t111_write(0x0F12 , 0x00F4); 	//010A	 //TVAR_ash_pGAS[221] //
		mt9t111_write(0x0F12 , 0x00B3); 	//00C1	 //TVAR_ash_pGAS[222] //
		mt9t111_write(0x0F12 , 0x0077); 	//007E	 //TVAR_ash_pGAS[223] //
		mt9t111_write(0x0F12 , 0x0048); 	//0049	 //TVAR_ash_pGAS[224] //
		mt9t111_write(0x0F12 , 0x0022); 	//0023	 //TVAR_ash_pGAS[225] //
		mt9t111_write(0x0F12 , 0x000C); 	//000E	 //TVAR_ash_pGAS[226] //
		mt9t111_write(0x0F12 , 0x0006); 	//0009	 //TVAR_ash_pGAS[227] //
		mt9t111_write(0x0F12 , 0x000A); 	//0015	 //TVAR_ash_pGAS[228] //
		mt9t111_write(0x0F12 , 0x0015); 	//0031	 //TVAR_ash_pGAS[229] //
		mt9t111_write(0x0F12 , 0x002E); 	//005D	 //TVAR_ash_pGAS[230] //
		mt9t111_write(0x0F12 , 0x0059); 	//0097	 //TVAR_ash_pGAS[231] //
		mt9t111_write(0x0F12 , 0x008C); 	//00DF	 //TVAR_ash_pGAS[232] //
		mt9t111_write(0x0F12 , 0x00C2); 	//0129	 //TVAR_ash_pGAS[233] //
		mt9t111_write(0x0F12 , 0x0108); 	//0121	 //TVAR_ash_pGAS[234] //
		mt9t111_write(0x0F12 , 0x00C4); 	//00DA	 //TVAR_ash_pGAS[235] //
		mt9t111_write(0x0F12 , 0x0087); 	//009A	 //TVAR_ash_pGAS[236] //
		mt9t111_write(0x0F12 , 0x0058); 	//0065	 //TVAR_ash_pGAS[237] //
		mt9t111_write(0x0F12 , 0x0032); 	//0040	 //TVAR_ash_pGAS[238] //
		mt9t111_write(0x0F12 , 0x001F); 	//002B	 //TVAR_ash_pGAS[239] //
		mt9t111_write(0x0F12 , 0x0015); 	//0026	 //TVAR_ash_pGAS[240] //
		mt9t111_write(0x0F12 , 0x0019); 	//0031	 //TVAR_ash_pGAS[241] //
		mt9t111_write(0x0F12 , 0x0026); 	//004E	 //TVAR_ash_pGAS[242] //
		mt9t111_write(0x0F12 , 0x003E); 	//007C	 //TVAR_ash_pGAS[243] //
		mt9t111_write(0x0F12 , 0x006B); 	//00B6	 //TVAR_ash_pGAS[244] //
		mt9t111_write(0x0F12 , 0x009D); 	//00FB	 //TVAR_ash_pGAS[245] //
		mt9t111_write(0x0F12 , 0x00D2); 	//0143	 //TVAR_ash_pGAS[246] //
		mt9t111_write(0x0F12 , 0x0122); 	//0140	 //TVAR_ash_pGAS[247] //
		mt9t111_write(0x0F12 , 0x00E1); 	//00FB	 //TVAR_ash_pGAS[248] //
		mt9t111_write(0x0F12 , 0x00A5); 	//00BD	 //TVAR_ash_pGAS[249] //
		mt9t111_write(0x0F12 , 0x0073); 	//008C	 //TVAR_ash_pGAS[250] //
		mt9t111_write(0x0F12 , 0x0053); 	//0068	 //TVAR_ash_pGAS[251] //
		mt9t111_write(0x0F12 , 0x003B); 	//0054	 //TVAR_ash_pGAS[252] //
		mt9t111_write(0x0F12 , 0x0031); 	//004F	 //TVAR_ash_pGAS[253] //
		mt9t111_write(0x0F12 , 0x0034); 	//005B	 //TVAR_ash_pGAS[254] //
		mt9t111_write(0x0F12 , 0x0042); 	//0077	 //TVAR_ash_pGAS[255] //
		mt9t111_write(0x0F12 , 0x005E); 	//00A2	 //TVAR_ash_pGAS[256] //
		mt9t111_write(0x0F12 , 0x0082); 	//00DA	 //TVAR_ash_pGAS[257] //
		mt9t111_write(0x0F12 , 0x00B4); 	//011C	 //TVAR_ash_pGAS[258] //
		mt9t111_write(0x0F12 , 0x00EA); 	//0163	 //TVAR_ash_pGAS[259] //
		mt9t111_write(0x0F12 , 0x014A); 	//0173	 //TVAR_ash_pGAS[260] //
		mt9t111_write(0x0F12 , 0x0104); 	//012D	 //TVAR_ash_pGAS[261] //
		mt9t111_write(0x0F12 , 0x00C4); 	//00F5	 //TVAR_ash_pGAS[262] //
		mt9t111_write(0x0F12 , 0x009B); 	//00C6	 //TVAR_ash_pGAS[263] //
		mt9t111_write(0x0F12 , 0x0079); 	//00A5	 //TVAR_ash_pGAS[264] //
		mt9t111_write(0x0F12 , 0x0068); 	//0092	 //TVAR_ash_pGAS[265] //
		mt9t111_write(0x0F12 , 0x005C); 	//008F	 //TVAR_ash_pGAS[266] //
		mt9t111_write(0x0F12 , 0x005F); 	//009B	 //TVAR_ash_pGAS[267] //
		mt9t111_write(0x0F12 , 0x006C); 	//00B5	 //TVAR_ash_pGAS[268] //
		mt9t111_write(0x0F12 , 0x0086); 	//00DC	 //TVAR_ash_pGAS[269] //
		mt9t111_write(0x0F12 , 0x00A9); 	//010C	 //TVAR_ash_pGAS[270] //
		mt9t111_write(0x0F12 , 0x00D5); 	//014C	 //TVAR_ash_pGAS[271] //
		mt9t111_write(0x0F12 , 0x010C); 	//0197	 //TVAR_ash_pGAS[272] //
		mt9t111_write(0x0F12 , 0x016B); 	//019F	 //TVAR_ash_pGAS[273] //
		mt9t111_write(0x0F12 , 0x0125); 	//015F	 //TVAR_ash_pGAS[274] //
		mt9t111_write(0x0F12 , 0x00F4); 	//0128	 //TVAR_ash_pGAS[275] //
		mt9t111_write(0x0F12 , 0x00CC); 	//00FF	 //TVAR_ash_pGAS[276] //
		mt9t111_write(0x0F12 , 0x00AE); 	//00E1	 //TVAR_ash_pGAS[277] //
		mt9t111_write(0x0F12 , 0x0099); 	//00D0	 //TVAR_ash_pGAS[278] //
		mt9t111_write(0x0F12 , 0x008E); 	//00CF	 //TVAR_ash_pGAS[279] //
		mt9t111_write(0x0F12 , 0x0090); 	//00DA	 //TVAR_ash_pGAS[280] //
		mt9t111_write(0x0F12 , 0x009C); 	//00F4	 //TVAR_ash_pGAS[281] //
		mt9t111_write(0x0F12 , 0x00B6); 	//0116	 //TVAR_ash_pGAS[282] //
		mt9t111_write(0x0F12 , 0x00D2); 	//0142	 //TVAR_ash_pGAS[283] //
		mt9t111_write(0x0F12 , 0x0103); 	//017A	 //TVAR_ash_pGAS[284] //
		mt9t111_write(0x0F12 , 0x0135); 	//01A8	 //TVAR_ash_pGAS[285] //
		mt9t111_write(0x0F12 , 0x0198); 	//0191	 //TVAR_ash_pGAS[286] //
		mt9t111_write(0x0F12 , 0x0154); 	//0152	 //TVAR_ash_pGAS[287] //
		mt9t111_write(0x0F12 , 0x011D); 	//0118	 //TVAR_ash_pGAS[288] //
		mt9t111_write(0x0F12 , 0x00F0); 	//00EB	 //TVAR_ash_pGAS[289] //
		mt9t111_write(0x0F12 , 0x00D3); 	//00C8	 //TVAR_ash_pGAS[290] //
		mt9t111_write(0x0F12 , 0x00BC); 	//00B4	 //TVAR_ash_pGAS[291] //
		mt9t111_write(0x0F12 , 0x00B1); 	//00AF	 //TVAR_ash_pGAS[292] //
		mt9t111_write(0x0F12 , 0x00B5); 	//00BB	 //TVAR_ash_pGAS[293] //
		mt9t111_write(0x0F12 , 0x00C2); 	//00D5	 //TVAR_ash_pGAS[294] //
		mt9t111_write(0x0F12 , 0x00D9); 	//00FE	 //TVAR_ash_pGAS[295] //
		mt9t111_write(0x0F12 , 0x00FE); 	//012E	 //TVAR_ash_pGAS[296] //
		mt9t111_write(0x0F12 , 0x012B); 	//016E	 //TVAR_ash_pGAS[297] //
		mt9t111_write(0x0F12 , 0x016E); 	//01AC	 //TVAR_ash_pGAS[298] //
		mt9t111_write(0x0F12 , 0x0164); 	//0166	 //TVAR_ash_pGAS[299] //
		mt9t111_write(0x0F12 , 0x0121); 	//0121	 //TVAR_ash_pGAS[300] //
		mt9t111_write(0x0F12 , 0x00E5); 	//00E3	 //TVAR_ash_pGAS[301] //
		mt9t111_write(0x0F12 , 0x00B8); 	//00B3	 //TVAR_ash_pGAS[302] //
		mt9t111_write(0x0F12 , 0x0096); 	//008E	 //TVAR_ash_pGAS[303] //
		mt9t111_write(0x0F12 , 0x007D); 	//0079	 //TVAR_ash_pGAS[304] //
		mt9t111_write(0x0F12 , 0x0071); 	//0074	 //TVAR_ash_pGAS[305] //
		mt9t111_write(0x0F12 , 0x0075); 	//0081	 //TVAR_ash_pGAS[306] //
		mt9t111_write(0x0F12 , 0x0083); 	//009D	 //TVAR_ash_pGAS[307] //
		mt9t111_write(0x0F12 , 0x009E); 	//00C7	 //TVAR_ash_pGAS[308] //
		mt9t111_write(0x0F12 , 0x00C5); 	//00FF	 //TVAR_ash_pGAS[309] //
		mt9t111_write(0x0F12 , 0x00F6); 	//013F	 //TVAR_ash_pGAS[310] //
		mt9t111_write(0x0F12 , 0x012B); 	//0188	 //TVAR_ash_pGAS[311] //
		mt9t111_write(0x0F12 , 0x0138); 	//0134	 //TVAR_ash_pGAS[312] //
		mt9t111_write(0x0F12 , 0x00F8); 	//00F1	 //TVAR_ash_pGAS[313] //
		mt9t111_write(0x0F12 , 0x00BB); 	//00B1	 //TVAR_ash_pGAS[314] //
		mt9t111_write(0x0F12 , 0x008A); 	//007C	 //TVAR_ash_pGAS[315] //
		mt9t111_write(0x0F12 , 0x0065); 	//0057	 //TVAR_ash_pGAS[316] //
		mt9t111_write(0x0F12 , 0x004E); 	//0041	 //TVAR_ash_pGAS[317] //
		mt9t111_write(0x0F12 , 0x0042); 	//003C	 //TVAR_ash_pGAS[318] //
		mt9t111_write(0x0F12 , 0x0044); 	//0048	 //TVAR_ash_pGAS[319] //
		mt9t111_write(0x0F12 , 0x0055); 	//0065	 //TVAR_ash_pGAS[320] //
		mt9t111_write(0x0F12 , 0x0074); 	//0091	 //TVAR_ash_pGAS[321] //
		mt9t111_write(0x0F12 , 0x009B); 	//00CA	 //TVAR_ash_pGAS[322] //
		mt9t111_write(0x0F12 , 0x00D0); 	//010C	 //TVAR_ash_pGAS[323] //
		mt9t111_write(0x0F12 , 0x0104); 	//0157	 //TVAR_ash_pGAS[324] //
		mt9t111_write(0x0F12 , 0x0109); 	//0118	 //TVAR_ash_pGAS[325] //
		mt9t111_write(0x0F12 , 0x00C8); 	//00D2	 //TVAR_ash_pGAS[326] //
		mt9t111_write(0x0F12 , 0x008D); 	//0091	 //TVAR_ash_pGAS[327] //
		mt9t111_write(0x0F12 , 0x005D); 	//005C	 //TVAR_ash_pGAS[328] //
		mt9t111_write(0x0F12 , 0x003C); 	//0035	 //TVAR_ash_pGAS[329] //
		mt9t111_write(0x0F12 , 0x0023); 	//001E	 //TVAR_ash_pGAS[330] //
		mt9t111_write(0x0F12 , 0x0019); 	//0019	 //TVAR_ash_pGAS[331] //
		mt9t111_write(0x0F12 , 0x001B); 	//0025	 //TVAR_ash_pGAS[332] //
		mt9t111_write(0x0F12 , 0x002C); 	//0043	 //TVAR_ash_pGAS[333] //
		mt9t111_write(0x0F12 , 0x004C); 	//0070	 //TVAR_ash_pGAS[334] //
		mt9t111_write(0x0F12 , 0x0076); 	//00A9	 //TVAR_ash_pGAS[335] //
		mt9t111_write(0x0F12 , 0x00AA); 	//00EE	 //TVAR_ash_pGAS[336] //
		mt9t111_write(0x0F12 , 0x00DE); 	//0136	 //TVAR_ash_pGAS[337] //
		mt9t111_write(0x0F12 , 0x00F8); 	//0105	 //TVAR_ash_pGAS[338] //
		mt9t111_write(0x0F12 , 0x00B9); 	//00BD	 //TVAR_ash_pGAS[339] //
		mt9t111_write(0x0F12 , 0x007D); 	//007B	 //TVAR_ash_pGAS[340] //
		mt9t111_write(0x0F12 , 0x004A); 	//0046	 //TVAR_ash_pGAS[341] //
		mt9t111_write(0x0F12 , 0x0028); 	//001F	 //TVAR_ash_pGAS[342] //
		mt9t111_write(0x0F12 , 0x000F); 	//0009	 //TVAR_ash_pGAS[343] //
		mt9t111_write(0x0F12 , 0x0004); 	//0003	 //TVAR_ash_pGAS[344] //
		mt9t111_write(0x0F12 , 0x0007); 	//000E	 //TVAR_ash_pGAS[345] //
		mt9t111_write(0x0F12 , 0x0018); 	//002B	 //TVAR_ash_pGAS[346] //
		mt9t111_write(0x0F12 , 0x0035); 	//0057	 //TVAR_ash_pGAS[347] //
		mt9t111_write(0x0F12 , 0x005E); 	//0091	 //TVAR_ash_pGAS[348] //
		mt9t111_write(0x0F12 , 0x0095); 	//00D7	 //TVAR_ash_pGAS[349] //
		mt9t111_write(0x0F12 , 0x00C9); 	//0121	 //TVAR_ash_pGAS[350] //
		mt9t111_write(0x0F12 , 0x00F5); 	//0105	 //TVAR_ash_pGAS[351] //
		mt9t111_write(0x0F12 , 0x00B6); 	//00BC	 //TVAR_ash_pGAS[352] //
		mt9t111_write(0x0F12 , 0x0078); 	//0078	 //TVAR_ash_pGAS[353] //
		mt9t111_write(0x0F12 , 0x0047); 	//0043	 //TVAR_ash_pGAS[354] //
		mt9t111_write(0x0F12 , 0x0021); 	//001D	 //TVAR_ash_pGAS[355] //
		mt9t111_write(0x0F12 , 0x000A); 	//0006	 //TVAR_ash_pGAS[356] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000	 //TVAR_ash_pGAS[357] //
		mt9t111_write(0x0F12 , 0x0004); 	//000B	 //TVAR_ash_pGAS[358] //
		mt9t111_write(0x0F12 , 0x0011); 	//0026	 //TVAR_ash_pGAS[359] //
		mt9t111_write(0x0F12 , 0x002B); 	//0052	 //TVAR_ash_pGAS[360] //
		mt9t111_write(0x0F12 , 0x0054); 	//008C	 //TVAR_ash_pGAS[361] //
		mt9t111_write(0x0F12 , 0x008F); 	//00D3	 //TVAR_ash_pGAS[362] //
		mt9t111_write(0x0F12 , 0x00C4); 	//011E	 //TVAR_ash_pGAS[363] //
		mt9t111_write(0x0F12 , 0x00FC); 	//010D	 //TVAR_ash_pGAS[364] //
		mt9t111_write(0x0F12 , 0x00BE); 	//00C5	 //TVAR_ash_pGAS[365] //
		mt9t111_write(0x0F12 , 0x0081); 	//0083	 //TVAR_ash_pGAS[366] //
		mt9t111_write(0x0F12 , 0x004F); 	//004E	 //TVAR_ash_pGAS[367] //
		mt9t111_write(0x0F12 , 0x0028); 	//0027	 //TVAR_ash_pGAS[368] //
		mt9t111_write(0x0F12 , 0x000F); 	//000F	 //TVAR_ash_pGAS[369] //
		mt9t111_write(0x0F12 , 0x0007); 	//0008	 //TVAR_ash_pGAS[370] //
		mt9t111_write(0x0F12 , 0x0009); 	//0012	 //TVAR_ash_pGAS[371] //
		mt9t111_write(0x0F12 , 0x0014); 	//002E	 //TVAR_ash_pGAS[372] //
		mt9t111_write(0x0F12 , 0x002E); 	//0059	 //TVAR_ash_pGAS[373] //
		mt9t111_write(0x0F12 , 0x0056); 	//0091	 //TVAR_ash_pGAS[374] //
		mt9t111_write(0x0F12 , 0x008B); 	//00D6	 //TVAR_ash_pGAS[375] //
		mt9t111_write(0x0F12 , 0x00C1); 	//0120	 //TVAR_ash_pGAS[376] //
		mt9t111_write(0x0F12 , 0x0115); 	//012A	 //TVAR_ash_pGAS[377] //
		mt9t111_write(0x0F12 , 0x00D4); 	//00E2	 //TVAR_ash_pGAS[378] //
		mt9t111_write(0x0F12 , 0x0097); 	//00A2	 //TVAR_ash_pGAS[379] //
		mt9t111_write(0x0F12 , 0x0064); 	//006D	 //TVAR_ash_pGAS[380] //
		mt9t111_write(0x0F12 , 0x003C); 	//0045	 //TVAR_ash_pGAS[381] //
		mt9t111_write(0x0F12 , 0x0024); 	//002D	 //TVAR_ash_pGAS[382] //
		mt9t111_write(0x0F12 , 0x0018); 	//0025	 //TVAR_ash_pGAS[383] //
		mt9t111_write(0x0F12 , 0x0018); 	//002E	 //TVAR_ash_pGAS[384] //
		mt9t111_write(0x0F12 , 0x0025); 	//0049	 //TVAR_ash_pGAS[385] //
		mt9t111_write(0x0F12 , 0x003B); 	//0073	 //TVAR_ash_pGAS[386] //
		mt9t111_write(0x0F12 , 0x0065); 	//00AA	 //TVAR_ash_pGAS[387] //
		mt9t111_write(0x0F12 , 0x0097); 	//00EC	 //TVAR_ash_pGAS[388] //
		mt9t111_write(0x0F12 , 0x00CD); 	//0135	 //TVAR_ash_pGAS[389] //
		mt9t111_write(0x0F12 , 0x0136); 	//0149	 //TVAR_ash_pGAS[390] //
		mt9t111_write(0x0F12 , 0x00F3); 	//0105	 //TVAR_ash_pGAS[391] //
		mt9t111_write(0x0F12 , 0x00B6); 	//00C9	 //TVAR_ash_pGAS[392] //
		mt9t111_write(0x0F12 , 0x0083); 	//0096	 //TVAR_ash_pGAS[393] //
		mt9t111_write(0x0F12 , 0x005E); 	//006F	 //TVAR_ash_pGAS[394] //
		mt9t111_write(0x0F12 , 0x0042); 	//0056	 //TVAR_ash_pGAS[395] //
		mt9t111_write(0x0F12 , 0x0032); 	//004E	 //TVAR_ash_pGAS[396] //
		mt9t111_write(0x0F12 , 0x0033); 	//0056	 //TVAR_ash_pGAS[397] //
		mt9t111_write(0x0F12 , 0x003F); 	//006F	 //TVAR_ash_pGAS[398] //
		mt9t111_write(0x0F12 , 0x0059); 	//0096	 //TVAR_ash_pGAS[399] //
		mt9t111_write(0x0F12 , 0x007A); 	//00CA	 //TVAR_ash_pGAS[400] //
		mt9t111_write(0x0F12 , 0x00AA); 	//0109	 //TVAR_ash_pGAS[401] //
		mt9t111_write(0x0F12 , 0x00DF); 	//0153	 //TVAR_ash_pGAS[402] //
		mt9t111_write(0x0F12 , 0x015E); 	//0181	 //TVAR_ash_pGAS[403] //
		mt9t111_write(0x0F12 , 0x0118); 	//013B	 //TVAR_ash_pGAS[404] //
		mt9t111_write(0x0F12 , 0x00DA); 	//0102	 //TVAR_ash_pGAS[405] //
		mt9t111_write(0x0F12 , 0x00AF); 	//00D2	 //TVAR_ash_pGAS[406] //
		mt9t111_write(0x0F12 , 0x0089); 	//00AE	 //TVAR_ash_pGAS[407] //
		mt9t111_write(0x0F12 , 0x0072); 	//0096	 //TVAR_ash_pGAS[408] //
		mt9t111_write(0x0F12 , 0x0060); 	//008D	 //TVAR_ash_pGAS[409] //
		mt9t111_write(0x0F12 , 0x005F); 	//0094	 //TVAR_ash_pGAS[410] //
		mt9t111_write(0x0F12 , 0x0069); 	//00AA	 //TVAR_ash_pGAS[411] //
		mt9t111_write(0x0F12 , 0x0080); 	//00CC	 //TVAR_ash_pGAS[412] //
		mt9t111_write(0x0F12 , 0x009F); 	//00F8	 //TVAR_ash_pGAS[413] //
		mt9t111_write(0x0F12 , 0x00C9); 	//0135	 //TVAR_ash_pGAS[414] //
		mt9t111_write(0x0F12 , 0x00FE); 	//0183	 //TVAR_ash_pGAS[415] //
		mt9t111_write(0x0F12 , 0x0183); 	//01B0	 //TVAR_ash_pGAS[416] //
		mt9t111_write(0x0F12 , 0x013B); 	//016F	 //TVAR_ash_pGAS[417] //
		mt9t111_write(0x0F12 , 0x010C); 	//0139	 //TVAR_ash_pGAS[418] //
		mt9t111_write(0x0F12 , 0x00E3); 	//010E	 //TVAR_ash_pGAS[419] //
		mt9t111_write(0x0F12 , 0x00C4); 	//00ED	 //TVAR_ash_pGAS[420] //
		mt9t111_write(0x0F12 , 0x00A8); 	//00D6	 //TVAR_ash_pGAS[421] //
		mt9t111_write(0x0F12 , 0x0096); 	//00CD	 //TVAR_ash_pGAS[422] //
		mt9t111_write(0x0F12 , 0x0091); 	//00D3	 //TVAR_ash_pGAS[423] //
		mt9t111_write(0x0F12 , 0x0099); 	//00E8	 //TVAR_ash_pGAS[424] //
		mt9t111_write(0x0F12 , 0x00AD); 	//0106	 //TVAR_ash_pGAS[425] //
		mt9t111_write(0x0F12 , 0x00C7); 	//012D	 //TVAR_ash_pGAS[426] //
		mt9t111_write(0x0F12 , 0x00F0); 	//0163	 //TVAR_ash_pGAS[427] //
		mt9t111_write(0x0F12 , 0x0126); 	//0195	 //TVAR_ash_pGAS[428] //
		mt9t111_write(0x0F12 , 0x0164); 	//0146	 //TVAR_ash_pGAS[429] //
		mt9t111_write(0x0F12 , 0x0124); 	//011B	 //TVAR_ash_pGAS[430] //
		mt9t111_write(0x0F12 , 0x00F4); 	//00EC	 //TVAR_ash_pGAS[431] //
		mt9t111_write(0x0F12 , 0x00CD); 	//00C6	 //TVAR_ash_pGAS[432] //
		mt9t111_write(0x0F12 , 0x00B5); 	//00AD	 //TVAR_ash_pGAS[433] //
		mt9t111_write(0x0F12 , 0x00A2); 	//009F	 //TVAR_ash_pGAS[434] //
		mt9t111_write(0x0F12 , 0x0099); 	//009C	 //TVAR_ash_pGAS[435] //
		mt9t111_write(0x0F12 , 0x009B); 	//00A6	 //TVAR_ash_pGAS[436] //
		mt9t111_write(0x0F12 , 0x00A7); 	//00BC	 //TVAR_ash_pGAS[437] //
		mt9t111_write(0x0F12 , 0x00BA); 	//00DF	 //TVAR_ash_pGAS[438] //
		mt9t111_write(0x0F12 , 0x00DA); 	//010B	 //TVAR_ash_pGAS[439] //
		mt9t111_write(0x0F12 , 0x0105); 	//0146	 //TVAR_ash_pGAS[440] //
		mt9t111_write(0x0F12 , 0x0139); 	//0176	 //TVAR_ash_pGAS[441] //
		mt9t111_write(0x0F12 , 0x0135); 	//0120	 //TVAR_ash_pGAS[442] //
		mt9t111_write(0x0F12 , 0x00FC); 	//00EC	 //TVAR_ash_pGAS[443] //
		mt9t111_write(0x0F12 , 0x00C8); 	//00BA	 //TVAR_ash_pGAS[444] //
		mt9t111_write(0x0F12 , 0x00A0); 	//0093	 //TVAR_ash_pGAS[445] //
		mt9t111_write(0x0F12 , 0x0084); 	//007A	 //TVAR_ash_pGAS[446] //
		mt9t111_write(0x0F12 , 0x0070); 	//006C	 //TVAR_ash_pGAS[447] //
		mt9t111_write(0x0F12 , 0x0067); 	//0069	 //TVAR_ash_pGAS[448] //
		mt9t111_write(0x0F12 , 0x0069); 	//0073	 //TVAR_ash_pGAS[449] //
		mt9t111_write(0x0F12 , 0x0075); 	//008B	 //TVAR_ash_pGAS[450] //
		mt9t111_write(0x0F12 , 0x008D); 	//00AD	 //TVAR_ash_pGAS[451] //
		mt9t111_write(0x0F12 , 0x00AA); 	//00DD	 //TVAR_ash_pGAS[452] //
		mt9t111_write(0x0F12 , 0x00D5); 	//0118	 //TVAR_ash_pGAS[453] //
		mt9t111_write(0x0F12 , 0x00FD); 	//0156	 //TVAR_ash_pGAS[454] //
		mt9t111_write(0x0F12 , 0x010B); 	//00F0	 //TVAR_ash_pGAS[455] //
		mt9t111_write(0x0F12 , 0x00D3); 	//00BF	 //TVAR_ash_pGAS[456] //
		mt9t111_write(0x0F12 , 0x009E); 	//0089	 //TVAR_ash_pGAS[457] //
		mt9t111_write(0x0F12 , 0x0075); 	//0062	 //TVAR_ash_pGAS[458] //
		mt9t111_write(0x0F12 , 0x0057); 	//0047	 //TVAR_ash_pGAS[459] //
		mt9t111_write(0x0F12 , 0x0045); 	//003A	 //TVAR_ash_pGAS[460] //
		mt9t111_write(0x0F12 , 0x003D); 	//0038	 //TVAR_ash_pGAS[461] //
		mt9t111_write(0x0F12 , 0x003E); 	//0041	 //TVAR_ash_pGAS[462] //
		mt9t111_write(0x0F12 , 0x004C); 	//0058	 //TVAR_ash_pGAS[463] //
		mt9t111_write(0x0F12 , 0x0064); 	//007C	 //TVAR_ash_pGAS[464] //
		mt9t111_write(0x0F12 , 0x0085); 	//00AC	 //TVAR_ash_pGAS[465] //
		mt9t111_write(0x0F12 , 0x00B0); 	//00E7	 //TVAR_ash_pGAS[466] //
		mt9t111_write(0x0F12 , 0x00D8); 	//0123	 //TVAR_ash_pGAS[467] //
		mt9t111_write(0x0F12 , 0x00D8); 	//00D5	 //TVAR_ash_pGAS[468] //
		mt9t111_write(0x0F12 , 0x00A5); 	//00A1	 //TVAR_ash_pGAS[469] //
		mt9t111_write(0x0F12 , 0x0072); 	//006C	 //TVAR_ash_pGAS[470] //
		mt9t111_write(0x0F12 , 0x004A); 	//0044	 //TVAR_ash_pGAS[471] //
		mt9t111_write(0x0F12 , 0x0031); 	//0029	 //TVAR_ash_pGAS[472] //
		mt9t111_write(0x0F12 , 0x0020); 	//001B	 //TVAR_ash_pGAS[473] //
		mt9t111_write(0x0F12 , 0x0017); 	//0018	 //TVAR_ash_pGAS[474] //
		mt9t111_write(0x0F12 , 0x001A); 	//0022	 //TVAR_ash_pGAS[475] //
		mt9t111_write(0x0F12 , 0x0028); 	//0039	 //TVAR_ash_pGAS[476] //
		mt9t111_write(0x0F12 , 0x0040); 	//005D	 //TVAR_ash_pGAS[477] //
		mt9t111_write(0x0F12 , 0x0061); 	//008E	 //TVAR_ash_pGAS[478] //
		mt9t111_write(0x0F12 , 0x008B); 	//00CA	 //TVAR_ash_pGAS[479] //
		mt9t111_write(0x0F12 , 0x00B3); 	//0105	 //TVAR_ash_pGAS[480] //
		mt9t111_write(0x0F12 , 0x00CA); 	//00C1	 //TVAR_ash_pGAS[481] //
		mt9t111_write(0x0F12 , 0x0096); 	//008C	 //TVAR_ash_pGAS[482] //
		mt9t111_write(0x0F12 , 0x0063); 	//0057	//TVAR_ash_pGAS[483] //
		mt9t111_write(0x0F12 , 0x003A); 	//002F	//TVAR_ash_pGAS[484] //
		mt9t111_write(0x0F12 , 0x001F); 	//0014	//TVAR_ash_pGAS[485] //
		mt9t111_write(0x0F12 , 0x000D); 	//0006	//TVAR_ash_pGAS[486] //
		mt9t111_write(0x0F12 , 0x0005); 	//0003	//TVAR_ash_pGAS[487] //
		mt9t111_write(0x0F12 , 0x0006); 	//000B	//TVAR_ash_pGAS[488] //
		mt9t111_write(0x0F12 , 0x0014); 	//0022	//TVAR_ash_pGAS[489] //
		mt9t111_write(0x0F12 , 0x002A); 	//0046	//TVAR_ash_pGAS[490] //
		mt9t111_write(0x0F12 , 0x004B); 	//0076	//TVAR_ash_pGAS[491] //
		mt9t111_write(0x0F12 , 0x0078); 	//00B4	//TVAR_ash_pGAS[492] //
		mt9t111_write(0x0F12 , 0x009F); 	//00F0	//TVAR_ash_pGAS[493] //
		mt9t111_write(0x0F12 , 0x00C3); 	//00C1	//TVAR_ash_pGAS[494] //
		mt9t111_write(0x0F12 , 0x0091); 	//008B	//TVAR_ash_pGAS[495] //
		mt9t111_write(0x0F12 , 0x005D); 	//0055	//TVAR_ash_pGAS[496] //
		mt9t111_write(0x0F12 , 0x0035); 	//002C	//TVAR_ash_pGAS[497] //
		mt9t111_write(0x0F12 , 0x0018); 	//0011	//TVAR_ash_pGAS[498] //
		mt9t111_write(0x0F12 , 0x0008); 	//0003	//TVAR_ash_pGAS[499] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000	//TVAR_ash_pGAS[500] //
		mt9t111_write(0x0F12 , 0x0002); 	//0007	//TVAR_ash_pGAS[501] //
		mt9t111_write(0x0F12 , 0x000C); 	//001D	//TVAR_ash_pGAS[502] //
		mt9t111_write(0x0F12 , 0x0020); 	//0040	//TVAR_ash_pGAS[503] //
		mt9t111_write(0x0F12 , 0x0041); 	//0070	//TVAR_ash_pGAS[504] //
		mt9t111_write(0x0F12 , 0x006E); 	//00AF	//TVAR_ash_pGAS[505] //
		mt9t111_write(0x0F12 , 0x0096); 	//00EC	//TVAR_ash_pGAS[506] //
		mt9t111_write(0x0F12 , 0x00C8); 	//00C6	//TVAR_ash_pGAS[507] //
		mt9t111_write(0x0F12 , 0x0097); 	//0092	//TVAR_ash_pGAS[508] //
		mt9t111_write(0x0F12 , 0x0063); 	//005D	//TVAR_ash_pGAS[509] //
		mt9t111_write(0x0F12 , 0x003C); 	//0035	//TVAR_ash_pGAS[510] //
		mt9t111_write(0x0F12 , 0x001D); 	//0019	//TVAR_ash_pGAS[511] //
		mt9t111_write(0x0F12 , 0x000B); 	//000A	//TVAR_ash_pGAS[512] //
		mt9t111_write(0x0F12 , 0x0005); 	//0006	//TVAR_ash_pGAS[513] //
		mt9t111_write(0x0F12 , 0x0006); 	//000E	//TVAR_ash_pGAS[514] //
		mt9t111_write(0x0F12 , 0x000D); 	//0024	//TVAR_ash_pGAS[515] //
		mt9t111_write(0x0F12 , 0x0020); 	//0046	//TVAR_ash_pGAS[516] //
		mt9t111_write(0x0F12 , 0x0040); 	//0074	//TVAR_ash_pGAS[517] //
		mt9t111_write(0x0F12 , 0x006A); 	//00B0	//TVAR_ash_pGAS[518] //
		mt9t111_write(0x0F12 , 0x0093); 	//00ED	//TVAR_ash_pGAS[519] //
		mt9t111_write(0x0F12 , 0x00E4); 	//00E0	//TVAR_ash_pGAS[520] //
		mt9t111_write(0x0F12 , 0x00A9); 	//00AA	//TVAR_ash_pGAS[521] //
		mt9t111_write(0x0F12 , 0x0078); 	//0078	//TVAR_ash_pGAS[522] //
		mt9t111_write(0x0F12 , 0x004F); 	//0050	//TVAR_ash_pGAS[523] //
		mt9t111_write(0x0F12 , 0x002F); 	//0034	//TVAR_ash_pGAS[524] //
		mt9t111_write(0x0F12 , 0x001C); 	//0024	//TVAR_ash_pGAS[525] //
		mt9t111_write(0x0F12 , 0x0012); 	//001F	//TVAR_ash_pGAS[526] //
		mt9t111_write(0x0F12 , 0x0012); 	//0026	//TVAR_ash_pGAS[527] //
		mt9t111_write(0x0F12 , 0x001B); 	//003A	//TVAR_ash_pGAS[528] //
		mt9t111_write(0x0F12 , 0x002B); 	//005D	//TVAR_ash_pGAS[529] //
		mt9t111_write(0x0F12 , 0x004C); 	//008B	//TVAR_ash_pGAS[530] //
		mt9t111_write(0x0F12 , 0x0072); 	//00C4	//TVAR_ash_pGAS[531] //
		mt9t111_write(0x0F12 , 0x009B); 	//00FE	//TVAR_ash_pGAS[532] //
		mt9t111_write(0x0F12 , 0x00FD); 	//00FE	//TVAR_ash_pGAS[533] //
		mt9t111_write(0x0F12 , 0x00C8); 	//00CB	//TVAR_ash_pGAS[534] //
		mt9t111_write(0x0F12 , 0x0093); 	//0099	//TVAR_ash_pGAS[535] //
		mt9t111_write(0x0F12 , 0x0069); 	//0074	//TVAR_ash_pGAS[536] //
		mt9t111_write(0x0F12 , 0x004A); 	//005A	//TVAR_ash_pGAS[537] //
		mt9t111_write(0x0F12 , 0x0035); 	//004B	//TVAR_ash_pGAS[538] //
		mt9t111_write(0x0F12 , 0x002A); 	//0046	//TVAR_ash_pGAS[539] //
		mt9t111_write(0x0F12 , 0x0027); 	//004A	//TVAR_ash_pGAS[540] //
		mt9t111_write(0x0F12 , 0x0032); 	//005C	//TVAR_ash_pGAS[541] //
		mt9t111_write(0x0F12 , 0x0044); 	//007A	//TVAR_ash_pGAS[542] //
		mt9t111_write(0x0F12 , 0x0060); 	//00A7	//TVAR_ash_pGAS[543] //
		mt9t111_write(0x0F12 , 0x0085); 	//00E0	//TVAR_ash_pGAS[544] //
		mt9t111_write(0x0F12 , 0x00AD); 	//0119	//TVAR_ash_pGAS[545] //
		mt9t111_write(0x0F12 , 0x0125); 	//0131	//TVAR_ash_pGAS[546] //
		mt9t111_write(0x0F12 , 0x00E9); 	//00FD	//TVAR_ash_pGAS[547] //
		mt9t111_write(0x0F12 , 0x00B3); 	//00CF	//TVAR_ash_pGAS[548] //
		mt9t111_write(0x0F12 , 0x0090); 	//00AC	//TVAR_ash_pGAS[549] //
		mt9t111_write(0x0F12 , 0x0072); 	//0092	//TVAR_ash_pGAS[550] //
		mt9t111_write(0x0F12 , 0x0061); 	//0084	//TVAR_ash_pGAS[551] //
		mt9t111_write(0x0F12 , 0x0052); 	//007F	//TVAR_ash_pGAS[552] //
		mt9t111_write(0x0F12 , 0x0050); 	//0085	//TVAR_ash_pGAS[553] //
		mt9t111_write(0x0F12 , 0x0058); 	//0094	//TVAR_ash_pGAS[554] //
		mt9t111_write(0x0F12 , 0x006A); 	//00AE	//TVAR_ash_pGAS[555] //
		mt9t111_write(0x0F12 , 0x0082); 	//00CF	//TVAR_ash_pGAS[556] //
		mt9t111_write(0x0F12 , 0x00A5); 	//0107	//TVAR_ash_pGAS[557] //
		mt9t111_write(0x0F12 , 0x00CF); 	//0148	//TVAR_ash_pGAS[558] //
		mt9t111_write(0x0F12 , 0x013D); 	//015F	//TVAR_ash_pGAS[559] //
		mt9t111_write(0x0F12 , 0x0107); 	//012F	//TVAR_ash_pGAS[560] //
		mt9t111_write(0x0F12 , 0x00D8); 	//0102	//TVAR_ash_pGAS[561] //
		mt9t111_write(0x0F12 , 0x00BC); 	//00E3	//TVAR_ash_pGAS[562] //
		mt9t111_write(0x0F12 , 0x00A1); 	//00CA	//TVAR_ash_pGAS[563] //
		mt9t111_write(0x0F12 , 0x0090); 	//00BD	//TVAR_ash_pGAS[564] //
		mt9t111_write(0x0F12 , 0x0083); 	//00B8	//TVAR_ash_pGAS[565] //
		mt9t111_write(0x0F12 , 0x007E); 	//00BC	//TVAR_ash_pGAS[566] //
		mt9t111_write(0x0F12 , 0x0081); 	//00CB	//TVAR_ash_pGAS[567] //
		mt9t111_write(0x0F12 , 0x0093); 	//00E4	//TVAR_ash_pGAS[568] //
		mt9t111_write(0x0F12 , 0x00A6); 	//0104	//TVAR_ash_pGAS[569] //
		mt9t111_write(0x0F12 , 0x00CB); 	//012E	//TVAR_ash_pGAS[570] //
		mt9t111_write(0x0F12 , 0x00F1); 	//0148	//TVAR_ash_pGAS[571] //
		
		mt9t111_write(0x002A , 0x074E); 
		mt9t111_write(0x0F12 , 0x0001); 	//ash_bLumaMode//
		mt9t111_write(0x002A , 0x0D30); 
		mt9t111_write(0x0F12 , 0x02A8); 	//awbb_GLocu   //
		mt9t111_write(0x0F12 , 0x0347); 	//awbb_GLocuSB //
		
		mt9t111_write(0x002A , 0x06B8); 
		mt9t111_write(0x0F12 , 0x00C0); 	//TVAR_ash_AwbashCord[0] //
		mt9t111_write(0x0F12 , 0x00E0); 	//TVAR_ash_AwbashCord[1] //
		mt9t111_write(0x0F12 , 0x0120); 	//TVAR_ash_AwbashCord[2] //
		mt9t111_write(0x0F12 , 0x0124); 	//TVAR_ash_AwbashCord[3] //
		mt9t111_write(0x0F12 , 0x0156); 	//TVAR_ash_AwbashCord[4] //
		mt9t111_write(0x0F12 , 0x017F); 	//TVAR_ash_AwbashCord[5] //
		mt9t111_write(0x0F12 , 0x018F); 	//TVAR_ash_AwbashCord[6] //
		
		mt9t111_write(0x002A , 0x0664); 
		mt9t111_write(0x0F12 , 0x013E); 	//seti_uContrastCenter //
		
		mt9t111_write(0x002A , 0x06C6); 
		mt9t111_write(0x0F12 , 0x010B); 	//ash_CGrasalphaS[0] //
		mt9t111_write(0x0F12 , 0x0103); 	//ash_CGrasalphaS[1] //
		mt9t111_write(0x0F12 , 0x00FC); 	//ash_CGrasalphaS[2] //
		mt9t111_write(0x0F12 , 0x010C); 	//ash_CGrasalphaS[3] //
					
		mt9t111_write(0x002A , 0x0C48);  //jsk Change	
		mt9t111_write(0x0F12 , 0x03C8); 		//03C8	//03C9	//awbb_IndoorGrZones_m_BGrid[0] //	  
		mt9t111_write(0x0F12 , 0x03DE); 		//03DE	//03DE	//awbb_IndoorGrZones_m_BGrid[1] // 
		mt9t111_write(0x0F12 , 0x0372); 		//0372	//0372	//awbb_IndoorGrZones_m_BGrid[2] // 
		mt9t111_write(0x0F12 , 0x03EA); 		//03EA	//03EA	//awbb_IndoorGrZones_m_BGrid[3] // 
		mt9t111_write(0x0F12 , 0x0336); 		//0336	//0336	//awbb_IndoorGrZones_m_BGrid[4] // 
		mt9t111_write(0x0F12 , 0x03DE); 		//03DE	//03DE	//awbb_IndoorGrZones_m_BGrid[5] // 
		mt9t111_write(0x0F12 , 0x0302); 		//0302	//0302	//awbb_IndoorGrZones_m_BGrid[6] // 
		mt9t111_write(0x0F12 , 0x03A2); 		//03A2	//03A2	//awbb_IndoorGrZones_m_BGrid[7] // 
		mt9t111_write(0x0F12 , 0x02C8); 		//02C8	//02c8	//awbb_IndoorGrZones_m_BGrid[8] // 
		mt9t111_write(0x0F12 , 0x036C); 		//0368	//0368	//awbb_IndoorGrZones_m_BGrid[9] // 
		mt9t111_write(0x0F12 , 0x0292); 		//0292	//0292	//awbb_IndoorGrZones_m_BGrid[10] //
		mt9t111_write(0x0F12 , 0x0340); 		//033A	//033A	//awbb_IndoorGrZones_m_BGrid[11] //
		mt9t111_write(0x0F12 , 0x0276); 		//0276	//0262	//awbb_IndoorGrZones_m_BGrid[12] //
		mt9t111_write(0x0F12 , 0x0318); 		//030A	//0306	//awbb_IndoorGrZones_m_BGrid[13] //
		mt9t111_write(0x0F12 , 0x025A); 		//025A	//0250	//awbb_IndoorGrZones_m_BGrid[14] //
		mt9t111_write(0x0F12 , 0x02F4); 		//02E4	//02C2	//awbb_IndoorGrZones_m_BGrid[15] //
		mt9t111_write(0x0F12 , 0x0246); 		//0246	//023A	//awbb_IndoorGrZones_m_BGrid[16] //
		mt9t111_write(0x0F12 , 0x02D6); 		//02C0	//02A2	//awbb_IndoorGrZones_m_BGrid[17] //
		mt9t111_write(0x0F12 , 0x0232); 		//0232	//0228	//awbb_IndoorGrZones_m_BGrid[18] //
		mt9t111_write(0x0F12 , 0x02B6); 		//029E	//0298	//awbb_IndoorGrZones_m_BGrid[19] //
		mt9t111_write(0x0F12 , 0x021E); 		//021E	//0210	//awbb_IndoorGrZones_m_BGrid[20] //
		mt9t111_write(0x0F12 , 0x0298); 		//0288	//029C	//awbb_IndoorGrZones_m_BGrid[21] //
		mt9t111_write(0x0F12 , 0x0208); 		//0208	//01FE	//awbb_IndoorGrZones_m_BGrid[22] //
		mt9t111_write(0x0F12 , 0x027E); 		//026C	//0292	//awbb_IndoorGrZones_m_BGrid[23] //
		mt9t111_write(0x0F12 , 0x01EE); 		//01EE	//01EE	//awbb_IndoorGrZones_m_BGrid[24] //
		mt9t111_write(0x0F12 , 0x0264); 		//025C	//0278	//awbb_IndoorGrZones_m_BGrid[25] //
		mt9t111_write(0x0F12 , 0x01F0); 		//01F0	//01F2	//awbb_IndoorGrZones_m_BGrid[26] //
		mt9t111_write(0x0F12 , 0x0248); 		//0248	//0268	//awbb_IndoorGrZones_m_BGrid[27] //
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0200	//awbb_IndoorGrZones_m_BGrid[28] //
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0246	//awbb_IndoorGrZones_m_BGrid[29] //
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[30] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[31] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[32] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[33] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[34] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[35] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[36] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[37] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[38] // 
		mt9t111_write(0x0F12 , 0x0000); 		//0000	//0000	//awbb_IndoorGrZones_m_BGrid[39] //
		
		mt9t111_write(0x0F12 , 0x0005);    //awbb_IndoorGrZones_m_Gridstep //
		
		
		
		mt9t111_write(0x002A , 0x0C9C); 
		mt9t111_write(0x0F12 , 0x000E); 
		mt9t111_write(0x002A , 0x0CA0); 
		mt9t111_write(0x0F12 , 0x0108); 	//awbb_IndoorGrZones_m_Boffs //
							 
		mt9t111_write(0x002A , 0x0CE0); 
		mt9t111_write(0x0F12 , 0x03D4); 	//awbb_LowBrGrZones_m_BGrid[0] //
		mt9t111_write(0x0F12 , 0x043E); 	//awbb_LowBrGrZones_m_BGrid[1] //
		mt9t111_write(0x0F12 , 0x035C); 	//awbb_LowBrGrZones_m_BGrid[2] //
		mt9t111_write(0x0F12 , 0x0438); 	//awbb_LowBrGrZones_m_BGrid[3] //
		mt9t111_write(0x0F12 , 0x02F0); 	//awbb_LowBrGrZones_m_BGrid[4] //
		mt9t111_write(0x0F12 , 0x042D); 	//awbb_LowBrGrZones_m_BGrid[5] //
		mt9t111_write(0x0F12 , 0x029A); 	//awbb_LowBrGrZones_m_BGrid[6] //
		mt9t111_write(0x0F12 , 0x03EF); 	//awbb_LowBrGrZones_m_BGrid[7] //
		mt9t111_write(0x0F12 , 0x025E); 	//awbb_LowBrGrZones_m_BGrid[8] //
		mt9t111_write(0x0F12 , 0x0395); 	//awbb_LowBrGrZones_m_BGrid[9] //
		mt9t111_write(0x0F12 , 0x022E); 	//awbb_LowBrGrZones_m_BGrid[10] //
		mt9t111_write(0x0F12 , 0x0346); 	//awbb_LowBrGrZones_m_BGrid[11] //
		mt9t111_write(0x0F12 , 0x0200); 	//awbb_LowBrGrZones_m_BGrid[12] //
		mt9t111_write(0x0F12 , 0x02F6); 	//awbb_LowBrGrZones_m_BGrid[13] //
		mt9t111_write(0x0F12 , 0x01CE); 	//awbb_LowBrGrZones_m_BGrid[14] //
		mt9t111_write(0x0F12 , 0x02C8); 	//awbb_LowBrGrZones_m_BGrid[15] //
		mt9t111_write(0x0F12 , 0x01BB); 	//awbb_LowBrGrZones_m_BGrid[16] //
		mt9t111_write(0x0F12 , 0x0287); 	//awbb_LowBrGrZones_m_BGrid[17] //
		mt9t111_write(0x0F12 , 0x01E2); 	//awbb_LowBrGrZones_m_BGrid[18] //
		mt9t111_write(0x0F12 , 0x0239); 	//awbb_LowBrGrZones_m_BGrid[19] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_LowBrGrZones_m_BGrid[20] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_LowBrGrZones_m_BGrid[21] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_LowBrGrZones_m_BGrid[22] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_LowBrGrZones_m_BGrid[23] //
		
		mt9t111_write(0x0F12 , 0x0006); 	//awbb_LowBrGrZones_m_Gridstep //
		mt9t111_write(0x002A , 0x0D18); 
		mt9t111_write(0x0F12 , 0x00AE); 	//awbb_LowBrGrZones_m_Boff //
		
		mt9t111_write(0x002A , 0x0CA4); 
		mt9t111_write(0x0F12 , 0x026E); 	//026E//0294//0286//02C2//awbb_OutdoorGrZones_m_BGrid[0] //    
		mt9t111_write(0x0F12 , 0x02A4); 	//02A4//02CB//02BD//02E0//awbb_OutdoorGrZones_m_BGrid[1] //
		mt9t111_write(0x0F12 , 0x0262); 	//0262//027A//026C//0278//awbb_OutdoorGrZones_m_BGrid[2] //
		mt9t111_write(0x0F12 , 0x02A8); 	//02A8//02D7//02C9//02BC//awbb_OutdoorGrZones_m_BGrid[3] //
		mt9t111_write(0x0F12 , 0x0256); 	//0256//0266//0258//025A//awbb_OutdoorGrZones_m_BGrid[4] //
		mt9t111_write(0x0F12 , 0x02AE); 	//02AE//02BF//02B1//02A2//awbb_OutdoorGrZones_m_BGrid[5] //
		mt9t111_write(0x0F12 , 0x0248); 	//0248//0252//0244//024A//awbb_OutdoorGrZones_m_BGrid[6] //
		mt9t111_write(0x0F12 , 0x02A4); 	//02A4//02A8//029A//0288//awbb_OutdoorGrZones_m_BGrid[7] //
		mt9t111_write(0x0F12 , 0x023E); 	//023E//023E//0230//0240//awbb_OutdoorGrZones_m_BGrid[8] //
		mt9t111_write(0x0F12 , 0x029A); 	//029A//028F//0281//0278//awbb_OutdoorGrZones_m_BGrid[9] //
		mt9t111_write(0x0F12 , 0x023A); 	//023A//0239//022B//023E//awbb_OutdoorGrZones_m_BGrid[10] //
		mt9t111_write(0x0F12 , 0x0290); 	//0290//027A//026C//0254//awbb_OutdoorGrZones_m_BGrid[11] //
		mt9t111_write(0x0F12 , 0x023A); 	//023A//024A//023C//0000//awbb_OutdoorGrZones_m_BGrid[12] //
		mt9t111_write(0x0F12 , 0x027E); 	//027E//0260//0252//0000//awbb_OutdoorGrZones_m_BGrid[13] //
		mt9t111_write(0x0F12 , 0x0244); 	//0244//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[14] //
		mt9t111_write(0x0F12 , 0x0266); 	//0266//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[15] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[16] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[17] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[18] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[19] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[20] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[21] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[22] //
		mt9t111_write(0x0F12 , 0x0000); 	//0000//0000//0000//0000//awbb_OutdoorGrZones_m_BGrid[23] //
		
		mt9t111_write(0x0F12 , 0x0004); 	//awbb_OutdoorGrZones_m_Gridstep //
		mt9t111_write(0x002A , 0x0CD8); 
		mt9t111_write(0x0F12 , 0x0008); 
		mt9t111_write(0x002A , 0x0CDC); 
		mt9t111_write(0x0F12 , 0x0204); 	//awbb_OutdoorGrZones_m_Boff //
		mt9t111_write(0x002A , 0x0D1C); 
		mt9t111_write(0x0F12 , 0x037C); 	//awbb_CrclLowT_R_c //
		mt9t111_write(0x002A , 0x0D20); 
		mt9t111_write(0x0F12 , 0x0157); 	//awbb_CrclLowT_B_c //
		mt9t111_write(0x002A , 0x0D24); 
		mt9t111_write(0x0F12 , 0x3EB8); 	//awbb_CrclLowT_Rad_c //
		
		
		mt9t111_write(0x002A , 0x0D2C); 
		mt9t111_write(0x0F12 , 0x013D); 	//awbb_IntcR //
		mt9t111_write(0x0F12 , 0x011E); 	//awbb_IntcB //
		mt9t111_write(0x002A , 0x0D46); 
		mt9t111_write(0x0F12 , 0x04C0); 	//0554//055D//0396//04A2//awbb_MvEq_RBthresh //
		
		mt9t111_write(0x002A , 0x0D28); 	//wp outdoor
		mt9t111_write(0x0F12 , 0x0270); 
		mt9t111_write(0x0F12 , 0x0240); 
		
		
		mt9t111_write(0x002A , 0x0D5C); 
		mt9t111_write(0x0F12 , 0x7FFF); 
		mt9t111_write(0x0F12 , 0x0050); 
		
		mt9t111_write(0x002A , 0x2316); 
		mt9t111_write(0x0F12 , 0x0006); 
		
		mt9t111_write(0x002A , 0x0E44); 
		mt9t111_write(0x0F12 , 0x0633);   //0525
		mt9t111_write(0x0F12 , 0x0400); 
		mt9t111_write(0x0F12 , 0x0599);   //078C  
		
#if 0
		mt9t111_write(0x002A  0x0D44); 
		mt9t111_write(0x0F12  0x0020);	 //awb speed
		
		
		mt9t111_write(0x002A  0x0DC4); 
		mt9t111_write(0x0F12  0x0000);	 //awb speed
		
		mt9t111_write(0x002A  0x0DCA); 
		mt9t111_write(0x0F12  0x0000);	 //awb speed
#endif
		
		mt9t111_write(0x002A , 0x0E36); 
		mt9t111_write(0x0F12 , 0x0028); 	 //R OFFSET
		mt9t111_write(0x0F12 , 0xFFD8); 	 //B OFFSET
		mt9t111_write(0x0F12 , 0x0000); 	 //G OFFSET
		
		
		mt9t111_write(0x002A , 0x0DD4); //jsk awb
		mt9t111_write(0x0F12 , 0x000A); 	//awbb_GridCorr_R[0] // 
		mt9t111_write(0x0F12 , 0x000A); 	//awbb_GridCorr_R[1] // 
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_R[2] // 
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_R[3] // 
		mt9t111_write(0x0F12 , 0xFFD8); 	//awbb_GridCorr_R[4] // 
		mt9t111_write(0x0F12 , 0xFFD8); 	//awbb_GridCorr_R[5] // 
			  
		mt9t111_write(0x0F12 , 0x000A); 	//awbb_GridCorr_R[6] // 
		mt9t111_write(0x0F12 , 0x000A); 	//awbb_GridCorr_R[7] // 
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_R[8] // 
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_R[9] // 
		mt9t111_write(0x0F12 , 0xFFD8); 	//awbb_GridCorr_R[10] //
		mt9t111_write(0x0F12 , 0xFFD8); 	//awbb_GridCorr_R[11] //
			  
		mt9t111_write(0x0F12 , 0x000A); 	//awbb_GridCorr_R[12] //
		mt9t111_write(0x0F12 , 0x000A); 	//awbb_GridCorr_R[13] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_R[14] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_R[15] //
		mt9t111_write(0x0F12 , 0xFFD8); 	//awbb_GridCorr_R[16] //
		mt9t111_write(0x0F12 , 0xFFD8); 	//awbb_GridCorr_R[17] //
			  
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[0] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[1] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[2] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[3] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[4] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[5] //FE60 FFC0
		  
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[6] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[7] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[8] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[9] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[10] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[11] //FE60 FFC0
		  
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[12] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[13] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[14] //
		mt9t111_write(0x0F12 , 0x0000); 	//awbb_GridCorr_B[15] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[16] //
		mt9t111_write(0x0F12 , 0x0050); 	//awbb_GridCorr_B[17] //FE60 FFC0
		
		mt9t111_write(0x0F12 , 0x02D9); 	//awbb_GridConst_1[0] //
		mt9t111_write(0x0F12 , 0x0357); 	//awbb_GridConst_1[1] //
		mt9t111_write(0x0F12 , 0x03D1); 	//awbb_GridConst_1[2] //
		
		
		mt9t111_write(0x0F12 , 0x0DF6); 	//0E4F//0DE9//0DE9//awbb_GridConst_2[0] //
		mt9t111_write(0x0F12 , 0x0ED8); 	//0EDD//0EDD//0EDD//awbb_GridConst_2[1] //
		mt9t111_write(0x0F12 , 0x0F51); 	//0F42//0F42//0F42//awbb_GridConst_2[2] //
		mt9t111_write(0x0F12 , 0x0F5C); 	//0F4E//0F4E//0F54//awbb_GridConst_2[3] //
		mt9t111_write(0x0F12 , 0x0F8F); 	//0F99//0F99//0FAE//awbb_GridConst_2[4] //
		mt9t111_write(0x0F12 , 0x1006); 	//1006//1006//1011//awbb_GridConst_2[5] //
		
		mt9t111_write(0x0F12 , 0x00AC); 	//00BA//awbb_GridCoeff_R_1
		mt9t111_write(0x0F12 , 0x00BD); 	//00AF//awbb_GridCoeff_B_1
		mt9t111_write(0x0F12 , 0x0049); 	//0049//awbb_GridCoeff_R_2
		mt9t111_write(0x0F12 , 0x00F5); 	//00F5//awbb_GridCoeff_B_2
		
		mt9t111_write(0x002A , 0x0E4A); 
		mt9t111_write(0x0F12 , 0x0002); 	//awbb_GridEnable//
		
		mt9t111_write(0x002A , 0x051A); 
		mt9t111_write(0x0F12 , 0x010E); 	//lt_uLimitHigh//
		mt9t111_write(0x0F12 , 0x00F5); 	//lt_uLimitLow// 
		
		
		mt9t111_write(0x002A , 0x0F76); 
		mt9t111_write(0x0F12 , 0x0007); 	//ae_statmode BLC off : 0x0F  on : 0x0D//  illumType On : 07   Off : 0F
		
		mt9t111_write(0x002A , 0x1034); 
		mt9t111_write(0x0F12 , 0x00C0); 	//saRR_IllumType[0] //
		mt9t111_write(0x0F12 , 0x00E0); 	//saRR_IllumType[1] //
		mt9t111_write(0x0F12 , 0x0104); 	//saRR_IllumType[2] //
		mt9t111_write(0x0F12 , 0x0129); 	//saRR_IllumType[3] //
		mt9t111_write(0x0F12 , 0x0156); 	//saRR_IllumType[4] //
		mt9t111_write(0x0F12 , 0x017F); 	//saRR_IllumType[5] //
		mt9t111_write(0x0F12 , 0x018F); 	//saRR_IllumType[6] //
		
		
		mt9t111_write(0x0F12 , 0x0120); 	//saRR_IllumTypeF[0] //
		mt9t111_write(0x0F12 , 0x0120); 	//saRR_IllumTypeF[1] //
		mt9t111_write(0x0F12 , 0x0120); 	//saRR_IllumTypeF[2] //
		mt9t111_write(0x0F12 , 0x0100); 	//saRR_IllumTypeF[3] //
		mt9t111_write(0x0F12 , 0x0100); 	//saRR_IllumTypeF[4] //
		mt9t111_write(0x0F12 , 0x0100); 	//saRR_IllumTypeF[5] //
		mt9t111_write(0x0F12 , 0x0100); 	//saRR_IllumTypeF[6] //
		
		
		
		mt9t111_write(0x002A , 0x3288);  //  saRR_usDualGammaLutRGBIndoor  //
		mt9t111_write(0x0F12 , 0x0000);  //  saRR_usDualGammaLutRGBIndoor[0] //[0] //
		mt9t111_write(0x0F12 , 0x0008);  //  saRR_usDualGammaLutRGBIndoor[0] //[1] //
		mt9t111_write(0x0F12 , 0x0013);  //  saRR_usDualGammaLutRGBIndoor[0] //[2] //
		mt9t111_write(0x0F12 , 0x002C);  //  saRR_usDualGammaLutRGBIndoor[0] //[3] //
		mt9t111_write(0x0F12 , 0x0062);  //  saRR_usDualGammaLutRGBIndoor[0] //[4] //
		mt9t111_write(0x0F12 , 0x00CD);  //  saRR_usDualGammaLutRGBIndoor[0] //[5] //
		mt9t111_write(0x0F12 , 0x0129);  //  saRR_usDualGammaLutRGBIndoor[0] //[6] //
		mt9t111_write(0x0F12 , 0x0151);  //  saRR_usDualGammaLutRGBIndoor[0] //[7] //
		mt9t111_write(0x0F12 , 0x0174);  //  saRR_usDualGammaLutRGBIndoor[0] //[8] //
		mt9t111_write(0x0F12 , 0x01AA);  //  saRR_usDualGammaLutRGBIndoor[0] //[9] //
		mt9t111_write(0x0F12 , 0x01D7);  //  saRR_usDualGammaLutRGBIndoor[0] //[10] //
		mt9t111_write(0x0F12 , 0x01FE);  //  saRR_usDualGammaLutRGBIndoor[0] //[11] //
		mt9t111_write(0x0F12 , 0x0221);  //  saRR_usDualGammaLutRGBIndoor[0] //[12] //
		mt9t111_write(0x0F12 , 0x025D);  //  saRR_usDualGammaLutRGBIndoor[0] //[13] //
		mt9t111_write(0x0F12 , 0x0291);  //  saRR_usDualGammaLutRGBIndoor[0] //[14] //
		mt9t111_write(0x0F12 , 0x02EB);  //  saRR_usDualGammaLutRGBIndoor[0] //[15] //
		mt9t111_write(0x0F12 , 0x033A);  //  saRR_usDualGammaLutRGBIndoor[0] //[16] //
		mt9t111_write(0x0F12 , 0x0380);  //  saRR_usDualGammaLutRGBIndoor[0] //[17] //
		mt9t111_write(0x0F12 , 0x03C2);  //  saRR_usDualGammaLutRGBIndoor[0] //[18] //
		mt9t111_write(0x0F12 , 0x03FF);  //  saRR_usDualGammaLutRGBIndoor[0] //[19] //
		mt9t111_write(0x0F12 , 0x0000);  //  saRR_usDualGammaLutRGBIndoor[1] //[0] //
		mt9t111_write(0x0F12 , 0x0008);  //  saRR_usDualGammaLutRGBIndoor[1] //[1] //
		mt9t111_write(0x0F12 , 0x0013);  //  saRR_usDualGammaLutRGBIndoor[1] //[2] //
		mt9t111_write(0x0F12 , 0x002C);  //  saRR_usDualGammaLutRGBIndoor[1] //[3] //
		mt9t111_write(0x0F12 , 0x0062);  //  saRR_usDualGammaLutRGBIndoor[1] //[4] //
		mt9t111_write(0x0F12 , 0x00CD);  //  saRR_usDualGammaLutRGBIndoor[1] //[5] //
		mt9t111_write(0x0F12 , 0x0129);  //  saRR_usDualGammaLutRGBIndoor[1] //[6] //
		mt9t111_write(0x0F12 , 0x0151);  //  saRR_usDualGammaLutRGBIndoor[1] //[7] //
		mt9t111_write(0x0F12 , 0x0174);  //  saRR_usDualGammaLutRGBIndoor[1] //[8] //
		mt9t111_write(0x0F12 , 0x01AA);  //  saRR_usDualGammaLutRGBIndoor[1] //[9] //
		mt9t111_write(0x0F12 , 0x01D7);  //  saRR_usDualGammaLutRGBIndoor[1] //[10] //
		mt9t111_write(0x0F12 , 0x01FE);  //  saRR_usDualGammaLutRGBIndoor[1] //[11] //
		mt9t111_write(0x0F12 , 0x0221);  //  saRR_usDualGammaLutRGBIndoor[1] //[12] //
		mt9t111_write(0x0F12 , 0x025D);  //  saRR_usDualGammaLutRGBIndoor[1] //[13] //
		mt9t111_write(0x0F12 , 0x0291);  //  saRR_usDualGammaLutRGBIndoor[1] //[14] //
		mt9t111_write(0x0F12 , 0x02EB);  //  saRR_usDualGammaLutRGBIndoor[1] //[15] //
		mt9t111_write(0x0F12 , 0x033A);  //  saRR_usDualGammaLutRGBIndoor[1] //[16] //
		mt9t111_write(0x0F12 , 0x0380);  //  saRR_usDualGammaLutRGBIndoor[1] //[17] //
		mt9t111_write(0x0F12 , 0x03C2);  //  saRR_usDualGammaLutRGBIndoor[1] //[18] //
		mt9t111_write(0x0F12 , 0x03FF);  //  saRR_usDualGammaLutRGBIndoor[1] //[19] //
		mt9t111_write(0x0F12 , 0x0000);  //  saRR_usDualGammaLutRGBIndoor[2] //[0] //
		mt9t111_write(0x0F12 , 0x0008);  //  saRR_usDualGammaLutRGBIndoor[2] //[1] //
		mt9t111_write(0x0F12 , 0x0013);  //  saRR_usDualGammaLutRGBIndoor[2] //[2] //
		mt9t111_write(0x0F12 , 0x002C);  //  saRR_usDualGammaLutRGBIndoor[2] //[3] //
		mt9t111_write(0x0F12 , 0x0062);  //  saRR_usDualGammaLutRGBIndoor[2] //[4] //
		mt9t111_write(0x0F12 , 0x00CD);  //  saRR_usDualGammaLutRGBIndoor[2] //[5] //
		mt9t111_write(0x0F12 , 0x0129);  //  saRR_usDualGammaLutRGBIndoor[2] //[6] //
		mt9t111_write(0x0F12 , 0x0151);  //  saRR_usDualGammaLutRGBIndoor[2] //[7] //
		mt9t111_write(0x0F12 , 0x0174);  //  saRR_usDualGammaLutRGBIndoor[2] //[8] //
		mt9t111_write(0x0F12 , 0x01AA);  //  saRR_usDualGammaLutRGBIndoor[2] //[9] //
		mt9t111_write(0x0F12 , 0x01D7);  //  saRR_usDualGammaLutRGBIndoor[2] //[10] //
		mt9t111_write(0x0F12 , 0x01FE);  //  saRR_usDualGammaLutRGBIndoor[2] //[11] //
		mt9t111_write(0x0F12 , 0x0221);  //  saRR_usDualGammaLutRGBIndoor[2] //[12] //
		mt9t111_write(0x0F12 , 0x025D);  //  saRR_usDualGammaLutRGBIndoor[2] //[13] //
		mt9t111_write(0x0F12 , 0x0291);  //  saRR_usDualGammaLutRGBIndoor[2] //[14] //
		mt9t111_write(0x0F12 , 0x02EB);  //  saRR_usDualGammaLutRGBIndoor[2] //[15] //
		mt9t111_write(0x0F12 , 0x033A);  //  saRR_usDualGammaLutRGBIndoor[2] //[16] //
		mt9t111_write(0x0F12 , 0x0380);  //  saRR_usDualGammaLutRGBIndoor[2] //[17] //
		mt9t111_write(0x0F12 , 0x03C2);  //  saRR_usDualGammaLutRGBIndoor[2] //[18] //
		mt9t111_write(0x0F12 , 0x03FF);  //  saRR_usDualGammaLutRGBIndoor[2] //[19] //
		
		
		mt9t111_write(0x0F12 , 0x0000); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[0] //
		mt9t111_write(0x0F12 , 0x0008); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[1] //
		mt9t111_write(0x0F12 , 0x0013); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[2] //
		mt9t111_write(0x0F12 , 0x002C); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[3] //
		mt9t111_write(0x0F12 , 0x0062); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[4] //
		mt9t111_write(0x0F12 , 0x00CD); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[5] //
		mt9t111_write(0x0F12 , 0x0129); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[6] //
		mt9t111_write(0x0F12 , 0x0151); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[7] //
		mt9t111_write(0x0F12 , 0x0174); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[8] //
		mt9t111_write(0x0F12 , 0x01AA); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[9] //
		mt9t111_write(0x0F12 , 0x01D7); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[10] //
		mt9t111_write(0x0F12 , 0x01FE); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[11] //
		mt9t111_write(0x0F12 , 0x0221); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[12] //
		mt9t111_write(0x0F12 , 0x025D); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[13] //
		mt9t111_write(0x0F12 , 0x0291); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[14] //
		mt9t111_write(0x0F12 , 0x02EB); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[15] //
		mt9t111_write(0x0F12 , 0x033A); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[16] //
		mt9t111_write(0x0F12 , 0x0380); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[17] //
		mt9t111_write(0x0F12 , 0x03C2); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[18] //
		mt9t111_write(0x0F12 , 0x03FF); 	//	saRR_usDualGammaLutRGBOutdoor[0] //[19] //
		mt9t111_write(0x0F12 , 0x0000); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[0] //
		mt9t111_write(0x0F12 , 0x0008); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[1] //
		mt9t111_write(0x0F12 , 0x0013); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[2] //
		mt9t111_write(0x0F12 , 0x002C); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[3] //
		mt9t111_write(0x0F12 , 0x0062); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[4] //
		mt9t111_write(0x0F12 , 0x00CD); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[5] //
		mt9t111_write(0x0F12 , 0x0129); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[6] //
		mt9t111_write(0x0F12 , 0x0151); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[7] //
		mt9t111_write(0x0F12 , 0x0174); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[8] //
		mt9t111_write(0x0F12 , 0x01AA); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[9] //
		mt9t111_write(0x0F12 , 0x01D7); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[10] //
		mt9t111_write(0x0F12 , 0x01FE); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[11] //
		mt9t111_write(0x0F12 , 0x0221); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[12] //
		mt9t111_write(0x0F12 , 0x025D); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[13] //
		mt9t111_write(0x0F12 , 0x0291); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[14] //
		mt9t111_write(0x0F12 , 0x02EB); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[15] //
		mt9t111_write(0x0F12 , 0x033A); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[16] //
		mt9t111_write(0x0F12 , 0x0380); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[17] //
		mt9t111_write(0x0F12 , 0x03C2); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[18] //
		mt9t111_write(0x0F12 , 0x03FF); 	//	saRR_usDualGammaLutRGBOutdoor[1] //[19] //
		mt9t111_write(0x0F12 , 0x0000); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[0] //
		mt9t111_write(0x0F12 , 0x0008); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[1] //
		mt9t111_write(0x0F12 , 0x0013); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[2] //
		mt9t111_write(0x0F12 , 0x002C); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[3] //
		mt9t111_write(0x0F12 , 0x0062); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[4] //
		mt9t111_write(0x0F12 , 0x00CD); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[5] //
		mt9t111_write(0x0F12 , 0x0129); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[6] //
		mt9t111_write(0x0F12 , 0x0151); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[7] //
		mt9t111_write(0x0F12 , 0x0174); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[8] //
		mt9t111_write(0x0F12 , 0x01AA); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[9] //
		mt9t111_write(0x0F12 , 0x01D7); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[10] //
		mt9t111_write(0x0F12 , 0x01FE); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[11] //
		mt9t111_write(0x0F12 , 0x0221); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[12] //
		mt9t111_write(0x0F12 , 0x025D); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[13] //
		mt9t111_write(0x0F12 , 0x0291); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[14] //
		mt9t111_write(0x0F12 , 0x02EB); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[15] //
		mt9t111_write(0x0F12 , 0x033A); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[16] //
		mt9t111_write(0x0F12 , 0x0380); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[17] //
		mt9t111_write(0x0F12 , 0x03C2); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[18] //
		mt9t111_write(0x0F12 , 0x03FF); 	//	saRR_usDualGammaLutRGBOutdoor[2] //[19] //
		
		
		mt9t111_write(0x002A , 0x06A6); 
		mt9t111_write(0x0F12 , 0x00C0); 	//saRR_AwbCcmCord[0] //
		mt9t111_write(0x0F12 , 0x00E0); 	//saRR_AwbCcmCord[1] //
		mt9t111_write(0x0F12 , 0x0110); 	//saRR_AwbCcmCord[2] //
		mt9t111_write(0x0F12 , 0x0139); 	//saRR_AwbCcmCord[3] //
		mt9t111_write(0x0F12 , 0x0166); 	//saRR_AwbCcmCord[4] //
		mt9t111_write(0x0F12 , 0x019F); 	//saRR_AwbCcmCord[5] //
		
		mt9t111_write(0x002A , 0x33A4); 
		mt9t111_write(0x0F12 , 0x0206); 	//0163	 //TVAR_wbt_pBaseCcmS[0] //
		mt9t111_write(0x0F12 , 0xFF14); 	//FF77	 //TVAR_wbt_pBaseCcmS[1] //
		mt9t111_write(0x0F12 , 0xFF82); 	//FFBF	 //TVAR_wbt_pBaseCcmS[2] //
		mt9t111_write(0x0F12 , 0xFF61); 	//FEC0	 //TVAR_wbt_pBaseCcmS[3] //
		mt9t111_write(0x0F12 , 0x030B); 	//0111	 //TVAR_wbt_pBaseCcmS[4] //
		mt9t111_write(0x0F12 , 0xFDF4); 	//FEAB	 //TVAR_wbt_pBaseCcmS[5] //
		mt9t111_write(0x0F12 , 0xFF98); 	//FFA2	 //TVAR_wbt_pBaseCcmS[6] //
		mt9t111_write(0x0F12 , 0x0021); 	//FFB9	 //TVAR_wbt_pBaseCcmS[7] //
		mt9t111_write(0x0F12 , 0x02D9); 	//023A	 //TVAR_wbt_pBaseCcmS[8] //
		mt9t111_write(0x0F12 , 0x0324); 	//014F	 //TVAR_wbt_pBaseCcmS[9] //
		mt9t111_write(0x0F12 , 0x0103); 	//0096	 //TVAR_wbt_pBaseCcmS[10] //
		mt9t111_write(0x0F12 , 0xFDE6); 	//FF06	 //TVAR_wbt_pBaseCcmS[11] //
		mt9t111_write(0x0F12 , 0x0124); 	//01CF	 //TVAR_wbt_pBaseCcmS[12] //
		mt9t111_write(0x0F12 , 0xFF30); 	//FF54	 //TVAR_wbt_pBaseCcmS[13] //
		mt9t111_write(0x0F12 , 0x01E6); 	//01CD	 //TVAR_wbt_pBaseCcmS[14] //
		mt9t111_write(0x0F12 , 0xFE70); 	//FEE7	 //TVAR_wbt_pBaseCcmS[15] //
		mt9t111_write(0x0F12 , 0x01F8); 	//0106	 //TVAR_wbt_pBaseCcmS[16] //
		mt9t111_write(0x0F12 , 0x0145); 	//00F3	 //TVAR_wbt_pBaseCcmS[17] //
		
		mt9t111_write(0x0F12 , 0x0206); //0163);	//TVAR_wbt_pBaseCcmS[18] //
		mt9t111_write(0x0F12 , 0xFF14); //FF77);	//TVAR_wbt_pBaseCcmS[19] //
		mt9t111_write(0x0F12 , 0xFF82); //FFBF);	//TVAR_wbt_pBaseCcmS[20] //
		mt9t111_write(0x0F12 , 0xFF61); //FEC0);	//TVAR_wbt_pBaseCcmS[21] //
		mt9t111_write(0x0F12 , 0x030B); //0111);	//TVAR_wbt_pBaseCcmS[22] //
		mt9t111_write(0x0F12 , 0xFDF4); //FEAB);	//TVAR_wbt_pBaseCcmS[23] //
		mt9t111_write(0x0F12 , 0xFF98); //FFA2);	//TVAR_wbt_pBaseCcmS[24] //
		mt9t111_write(0x0F12 , 0x0021); //FFB9);	//TVAR_wbt_pBaseCcmS[25] //
		mt9t111_write(0x0F12 , 0x02D9); //023A);	//TVAR_wbt_pBaseCcmS[26] //
		mt9t111_write(0x0F12 , 0x0324); //014F);	//TVAR_wbt_pBaseCcmS[27] //
		mt9t111_write(0x0F12 , 0x0103); //0096);	//TVAR_wbt_pBaseCcmS[28] //
		mt9t111_write(0x0F12 , 0xFDE6); //FF06);	//TVAR_wbt_pBaseCcmS[29] //
		mt9t111_write(0x0F12 , 0x0124); //01CF);	//TVAR_wbt_pBaseCcmS[30] //
		mt9t111_write(0x0F12 , 0xFF30); //FF54);	//TVAR_wbt_pBaseCcmS[31] //
		mt9t111_write(0x0F12 , 0x01E6); //01CD);	//TVAR_wbt_pBaseCcmS[32] //
		mt9t111_write(0x0F12 , 0xFE70); //FEE7);	//TVAR_wbt_pBaseCcmS[33] //
		mt9t111_write(0x0F12 , 0x01F8); //0106);	//TVAR_wbt_pBaseCcmS[34] //
		mt9t111_write(0x0F12 , 0x0145); //00F3);	//TVAR_wbt_pBaseCcmS[35] //
		
		mt9t111_write(0x0F12 , 0x01E9);   //0163   //TVAR_wbt_pBaseCcmS[36] //
		mt9t111_write(0x0F12 , 0xFF7D);   //FF77   //TVAR_wbt_pBaseCcmS[37] //
		mt9t111_write(0x0F12 , 0x0012);   //FFBF   //TVAR_wbt_pBaseCcmS[38] //
		mt9t111_write(0x0F12 , 0xFEDA);   //FEC0   //TVAR_wbt_pBaseCcmS[39] //
		mt9t111_write(0x0F12 , 0x00BD);   //0111   //TVAR_wbt_pBaseCcmS[40] //
		mt9t111_write(0x0F12 , 0xFF0F);   //FEAB   //TVAR_wbt_pBaseCcmS[41] //
		mt9t111_write(0x0F12 , 0xFFA4);   //FFA2   //TVAR_wbt_pBaseCcmS[42] //
		mt9t111_write(0x0F12 , 0xFFA2);   //FFB9   //TVAR_wbt_pBaseCcmS[43] //
		mt9t111_write(0x0F12 , 0x0254);   //023A   //TVAR_wbt_pBaseCcmS[44] //
		mt9t111_write(0x0F12 , 0x015C);   //014F   //TVAR_wbt_pBaseCcmS[45] //
		mt9t111_write(0x0F12 , 0x0102);   //0096   //TVAR_wbt_pBaseCcmS[46] //
		mt9t111_write(0x0F12 , 0xFE36);   //FF06   //TVAR_wbt_pBaseCcmS[47] //
		mt9t111_write(0x0F12 , 0x021B);   //01CF   //TVAR_wbt_pBaseCcmS[48] //
		mt9t111_write(0x0F12 , 0xFF4B);   //FF54   //TVAR_wbt_pBaseCcmS[49] //
		mt9t111_write(0x0F12 , 0x01D5);   //01CD   //TVAR_wbt_pBaseCcmS[50] //
		mt9t111_write(0x0F12 , 0xFE5E);   //FEE7   //TVAR_wbt_pBaseCcmS[51] //
		mt9t111_write(0x0F12 , 0x0199);   //0106   //TVAR_wbt_pBaseCcmS[52] //
		mt9t111_write(0x0F12 , 0x0135);   //00F3   //TVAR_wbt_pBaseCcmS[53] //
		
		mt9t111_write(0x0F12 , 0x01E0);   //01E7  //01FA   //TVAR_wbt_pBaseCcmS[54] //
		mt9t111_write(0x0F12 , 0xFFBC);   //FFAE  //FF9B   //TVAR_wbt_pBaseCcmS[55] //
		mt9t111_write(0x0F12 , 0xFFF9);   //0001  //FFFF   //TVAR_wbt_pBaseCcmS[56] //
		mt9t111_write(0x0F12 , 0xFEBD);   //FECD  //FE9F   //TVAR_wbt_pBaseCcmS[57] //
		mt9t111_write(0x0F12 , 0x0103);   //00E6  //010F   //TVAR_wbt_pBaseCcmS[58] //
		mt9t111_write(0x0F12 , 0xFEE3);   //FEEF  //FEF5   //TVAR_wbt_pBaseCcmS[59] //
		mt9t111_write(0x0F12 , 0xFFD2);   //FFA6  //FFD2   //TVAR_wbt_pBaseCcmS[60] //
		mt9t111_write(0x0F12 , 0x0015);   //FFF9  //0015   //TVAR_wbt_pBaseCcmS[61] //
		mt9t111_write(0x0F12 , 0x01A1);   //01E9  //01A1   //TVAR_wbt_pBaseCcmS[62] //
		mt9t111_write(0x0F12 , 0x00F8);   //0104  //0111   //TVAR_wbt_pBaseCcmS[63] //
		mt9t111_write(0x0F12 , 0x0091);   //00A6  //009D   //TVAR_wbt_pBaseCcmS[64] //
		mt9t111_write(0x0F12 , 0xFEF0);   //FECF  //FECB   //TVAR_wbt_pBaseCcmS[65] //
		mt9t111_write(0x0F12 , 0x01FC);   //01E1  //01FC   //TVAR_wbt_pBaseCcmS[66] //
		mt9t111_write(0x0F12 , 0xFF99);   //FF96  //FF99   //TVAR_wbt_pBaseCcmS[67] //
		mt9t111_write(0x0F12 , 0x01A9);   //01C7  //01A9   //TVAR_wbt_pBaseCcmS[68] //
		mt9t111_write(0x0F12 , 0xFF26);   //FF2B  //FF26   //TVAR_wbt_pBaseCcmS[69] //
		mt9t111_write(0x0F12 , 0x012B);   //0140  //012B   //TVAR_wbt_pBaseCcmS[70] //
		mt9t111_write(0x0F12 , 0x00DF);   //00C5  //00DF   //TVAR_wbt_pBaseCcmS[71] //
		
		mt9t111_write(0x0F12 , 0x01BE);   //01C6  //01E2   //TVAR_wbt_pBaseCcmS[72] //
		mt9t111_write(0x0F12 , 0xFFAF);   //FFA5  //FF9A   //TVAR_wbt_pBaseCcmS[73] //
		mt9t111_write(0x0F12 , 0x001E);   //FFF7  //FFE7   //TVAR_wbt_pBaseCcmS[74] //
		mt9t111_write(0x0F12 , 0xFECF);   //FF23  //FE9F   //TVAR_wbt_pBaseCcmS[75] //
		mt9t111_write(0x0F12 , 0x00C1);   //0198  //010F   //TVAR_wbt_pBaseCcmS[76] //
		mt9t111_write(0x0F12 , 0xFF14);   //FF33  //FEF5   //TVAR_wbt_pBaseCcmS[77] //
		mt9t111_write(0x0F12 , 0x0000);   //FFA8  //FFD2   //TVAR_wbt_pBaseCcmS[78] //
		mt9t111_write(0x0F12 , 0xFFE4);   //FFDF  //FFFE   //TVAR_wbt_pBaseCcmS[79] //
		mt9t111_write(0x0F12 , 0x01A8);   //01FF  //01B7   //TVAR_wbt_pBaseCcmS[80] //
		mt9t111_write(0x0F12 , 0x00B6);   //00D7  //00E8   //TVAR_wbt_pBaseCcmS[81] //
		mt9t111_write(0x0F12 , 0x00B1);   //00D4  //0095   //TVAR_wbt_pBaseCcmS[82] //
		mt9t111_write(0x0F12 , 0xFF16);   //FF37  //FF0D   //TVAR_wbt_pBaseCcmS[83] //
		mt9t111_write(0x0F12 , 0x01FA);   //0158  //0182   //TVAR_wbt_pBaseCcmS[84] //
		mt9t111_write(0x0F12 , 0xFF52);   //FF1F  //FF29   //TVAR_wbt_pBaseCcmS[85] //
		mt9t111_write(0x0F12 , 0x01A3);   //017A  //0146   //TVAR_wbt_pBaseCcmS[86] //
		mt9t111_write(0x0F12 , 0xFEC7);   //FF3A  //FF26   //TVAR_wbt_pBaseCcmS[87] //
		mt9t111_write(0x0F12 , 0x0195);   //0163  //012B   //TVAR_wbt_pBaseCcmS[88] //
		mt9t111_write(0x0F12 , 0x00D8);   //0093  //00DF   //TVAR_wbt_pBaseCcmS[89] //
		
		mt9t111_write(0x0F12 , 0x01BE);   //01C6  //01E2   //TVAR_wbt_pBaseCcmS[90] //
		mt9t111_write(0x0F12 , 0xFFAF);   //FFA5  //FF9A   //TVAR_wbt_pBaseCcmS[91] //
		mt9t111_write(0x0F12 , 0x001E);   //FFF7  //FFE7   //TVAR_wbt_pBaseCcmS[92] //
		mt9t111_write(0x0F12 , 0xFECF);   //FF23  //FE9F   //TVAR_wbt_pBaseCcmS[93] //
		mt9t111_write(0x0F12 , 0x00C1);   //0198  //010F   //TVAR_wbt_pBaseCcmS[94] //
		mt9t111_write(0x0F12 , 0xFF14);   //FF33  //FEF5   //TVAR_wbt_pBaseCcmS[95] //
		mt9t111_write(0x0F12 , 0x0000);   //FFA8  //FFD2   //TVAR_wbt_pBaseCcmS[96] //
		mt9t111_write(0x0F12 , 0xFFE4);   //FFDF  //FFFE   //TVAR_wbt_pBaseCcmS[97] //
		mt9t111_write(0x0F12 , 0x01A8);   //01FF  //01B7   //TVAR_wbt_pBaseCcmS[98] //
		mt9t111_write(0x0F12 , 0x00B6);   //00D7  //00E8   //TVAR_wbt_pBaseCcmS[99] //
		mt9t111_write(0x0F12 , 0x00B1);   //00D4  //0095   //TVAR_wbt_pBaseCcmS[100] //
		mt9t111_write(0x0F12 , 0xFF16);   //FF37  //FF0D   //TVAR_wbt_pBaseCcmS[101] //
		mt9t111_write(0x0F12 , 0x01FA);   //0158  //0182   //TVAR_wbt_pBaseCcmS[102] //
		mt9t111_write(0x0F12 , 0xFF52);   //FF1F  //FF29   //TVAR_wbt_pBaseCcmS[103] //
		mt9t111_write(0x0F12 , 0x01A3);   //017A  //0146   //TVAR_wbt_pBaseCcmS[104] //
		mt9t111_write(0x0F12 , 0xFEC7);   //FF3A  //FF26   //TVAR_wbt_pBaseCcmS[105] //
		mt9t111_write(0x0F12 , 0x0195);   //0163  //012B   //TVAR_wbt_pBaseCcmS[106] //
		mt9t111_write(0x0F12 , 0x00D8);   //0093  //00DF   //TVAR_wbt_pBaseCcmS[107] //
		
		mt9t111_write(0x002A , 0x3380);  //12
		mt9t111_write(0x0F12 , 0x019A);    //01A0 //0204  //01FA  //0223   //0223  //01F3  //01F3  //TVAR_wbt_pOutdoorCcm[0] //
		mt9t111_write(0x0F12 , 0xFFCC);    //FFC8 //FF8E  //FF94  //FF7C   //FF7C  //FFA4  //FFA4  //TVAR_wbt_pOutdoorCcm[1] //
		mt9t111_write(0x0F12 , 0xFFFF);    //FFFC //FFD2  //FFD6  //FFC5   //FFC5  //FFE4  //FFE4  //TVAR_wbt_pOutdoorCcm[2] //
		mt9t111_write(0x0F12 , 0xFEB3);    //FEAA //FE3D  //FE3D  //FE3D   //FE3D  //FE3D  //FE23  //TVAR_wbt_pOutdoorCcm[3] //
		mt9t111_write(0x0F12 , 0x00F3);    //0102 //0158  //0158  //0158   //0158  //0158  //017D  //TVAR_wbt_pOutdoorCcm[4] //
		mt9t111_write(0x0F12 , 0xFEF3);    //FEED //FF03  //FF03  //FF03   //FF03  //FF03  //FEF9  //TVAR_wbt_pOutdoorCcm[5] //
		mt9t111_write(0x0F12 , 0xFFD1); 	 //FF99  //FF99  //FF9F   //FF9F  //FF9F  //FF9F  //TVAR_wbt_pOutdoorCcm[6] //
		mt9t111_write(0x0F12 , 0xFFEC); 	 //0018  //0018  //0011   //0011  //0011  //0011  //TVAR_wbt_pOutdoorCcm[7] //
		mt9t111_write(0x0F12 , 0x0228); 	 //0235  //0235  //0237   //0237  //0237  //0237  //TVAR_wbt_pOutdoorCcm[8] //
		mt9t111_write(0x0F12 , 0x012F); 	 //0101  //0101  //00EB   //00D1  //012A  //0143  //TVAR_wbt_pOutdoorCcm[9] //
		mt9t111_write(0x0F12 , 0x00BE); 	 //0116  //0116  //012A   //0125  //00CA  //00F6  //TVAR_wbt_pOutdoorCcm[10] //
		mt9t111_write(0x0F12 , 0xFF2A); 	 //FF00  //FF00  //FF02   //FEF5  //FEF6  //FEB1  //TVAR_wbt_pOutdoorCcm[11] //
		mt9t111_write(0x0F12 , 0x0179); 	 //018C  //018C  //01C5   //01C5  //01C5  //01C5  //TVAR_wbt_pOutdoorCcm[12] //
		mt9t111_write(0x0F12 , 0xFF61); 	 //FF66  //FF66  //FF80   //FF80  //FF80  //FF80  //TVAR_wbt_pOutdoorCcm[13] //
		mt9t111_write(0x0F12 , 0x017F); 	 //0167  //0167  //019D   //019D  //019D  //019D  //TVAR_wbt_pOutdoorCcm[14] //
		mt9t111_write(0x0F12 , 0xFEF4); 	 //FE7A  //FE7A  //FE7A   //FE7A  //FE7A  //FE7A  //TVAR_wbt_pOutdoorCcm[15] //
		mt9t111_write(0x0F12 , 0x01BE); 	 //0179  //0179  //0179   //0179  //0179  //0179  //TVAR_wbt_pOutdoorCcm[16] //
		mt9t111_write(0x0F12 , 0x00BB); 	 //0179  //0179  //0179   //0179  //0179  //0179  //TVAR_wbt_pOutdoorCcm[17] //
		
		
		
		mt9t111_write(0x002A , 0x0764); 
		mt9t111_write(0x0F12 , 0x0049); 	//afit_uNoiseIndInDoor[0] //
		mt9t111_write(0x0F12 , 0x005F); 	//afit_uNoiseIndInDoor[1] //
		mt9t111_write(0x0F12 , 0x00CB); 	//afit_uNoiseIndInDoor[2] // 203 //
		mt9t111_write(0x0F12 , 0x01E0); 	//afit_uNoiseIndInDoor[3] // Indoor_NB below 1500 _Noise index 300-400d //
		mt9t111_write(0x0F12 , 0x0220); 	//afit_uNoiseIndInDoor[4] // DNP NB 4600 _ Noisenidex :560d-230h //
		
		
		mt9t111_write(0x002A , 0x07C4); 
		mt9t111_write(0x0F12 , 0x0034); 	//700007C4 //TVAR_afit_pBaseValS[0] // AFIT16_BRIGHTNESS
		mt9t111_write(0x0F12 , 0x0000); 	//700007C6 //TVAR_afit_pBaseValS[1] // AFIT16_CONTRAST
		mt9t111_write(0x0F12 , 0x0020); 	//700007C8 //TVAR_afit_pBaseValS[2] // AFIT16_SATURATION
		mt9t111_write(0x0F12 , 0xFFD6); 	//700007CA //TVAR_afit_pBaseValS[3] // AFIT16_SHARP_BLUR
		mt9t111_write(0x0F12 , 0x0000); 	//700007CC //TVAR_afit_pBaseValS[4] // AFIT16_GLAMOUR
		mt9t111_write(0x0F12 , 0x00C1); 	//700007CE //TVAR_afit_pBaseValS[5] // AFIT16_sddd8a_edge_high
		mt9t111_write(0x0F12 , 0x03FF); 	//700007D0 //TVAR_afit_pBaseValS[6] // AFIT16_Demosaicing_iSatVal
		mt9t111_write(0x0F12 , 0x009C); 	//700007D2 //TVAR_afit_pBaseValS[7] // AFIT16_Sharpening_iReduceEdgeThresh
		mt9t111_write(0x0F12 , 0x0251); 	//700007D4 //TVAR_afit_pBaseValS[8] // AFIT16_demsharpmix1_iRGBOffset
		mt9t111_write(0x0F12 , 0x03FF); 	//700007D6 //TVAR_afit_pBaseValS[9] // AFIT16_demsharpmix1_iDemClamp
		mt9t111_write(0x0F12 , 0x000C); 	//700007D8 //TVAR_afit_pBaseValS[10] //AFIT16_demsharpmix1_iLowThreshold
		mt9t111_write(0x0F12 , 0x0010); 	//700007DA //TVAR_afit_pBaseValS[11] //AFIT16_demsharpmix1_iHighThreshold
		mt9t111_write(0x0F12 , 0x012C); 	//700007DC //TVAR_afit_pBaseValS[12] //AFIT16_demsharpmix1_iLowBright		  
		mt9t111_write(0x0F12 , 0x03E8); 	//700007DE //TVAR_afit_pBaseValS[13] //AFIT16_demsharpmix1_iHighBright		  
		mt9t111_write(0x0F12 , 0x0046); 	//700007E0 //TVAR_afit_pBaseValS[14] //AFIT16_demsharpmix1_iLowSat			  
		mt9t111_write(0x0F12 , 0x005A); 	//700007E2 //TVAR_afit_pBaseValS[15] //AFIT16_demsharpmix1_iHighSat 		  
		mt9t111_write(0x0F12 , 0x0070); 	//700007E4 //TVAR_afit_pBaseValS[16] //AFIT16_demsharpmix1_iTune			  
		mt9t111_write(0x0F12 , 0x0000); 	//700007E6 //TVAR_afit_pBaseValS[17] //AFIT16_demsharpmix1_iHystThLow		  
		mt9t111_write(0x0F12 , 0x0000); 	//700007E8 //TVAR_afit_pBaseValS[18] //AFIT16_demsharpmix1_iHystThHigh		  
		mt9t111_write(0x0F12 , 0x01AA); 	//700007EA //TVAR_afit_pBaseValS[19] //AFIT16_demsharpmix1_iHystCenter		  
		mt9t111_write(0x0F12 , 0x003C); 	//700007EC //TVAR_afit_pBaseValS[20] //AFIT16_YUV422_DENOISE_iUVLowThresh	  
		mt9t111_write(0x0F12 , 0x003C); 	//700007EE //TVAR_afit_pBaseValS[21] //AFIT16_YUV422_DENOISE_iUVHighThresh	  
		mt9t111_write(0x0F12 , 0x0000); 	//700007F0 //TVAR_afit_pBaseValS[22] //AFIT16_YUV422_DENOISE_iYLowThresh	  
		mt9t111_write(0x0F12 , 0x0000); 	//700007F2 //TVAR_afit_pBaseValS[23] //AFIT16_YUV422_DENOISE_iYHighThresh	  
		mt9t111_write(0x0F12 , 0x003E); 	//700007F4 //TVAR_afit_pBaseValS[24] //AFIT16_Sharpening_iLowSharpClamp 	  
		mt9t111_write(0x0F12 , 0x0008); 	//700007F6 //TVAR_afit_pBaseValS[25] //AFIT16_Sharpening_iHighSharpClamp	  
		mt9t111_write(0x0F12 , 0x003C); 	//700007F8 //TVAR_afit_pBaseValS[26] //AFIT16_Sharpening_iLowSharpClamp_Bin   
		mt9t111_write(0x0F12 , 0x001E); 	//700007FA //TVAR_afit_pBaseValS[27] //AFIT16_Sharpening_iHighSharpClamp_Bin  
		mt9t111_write(0x0F12 , 0x003C); 	//700007FC //TVAR_afit_pBaseValS[28] //AFIT16_Sharpening_iLowSharpClamp_sBin  
		mt9t111_write(0x0F12 , 0x001E); 	//700007FE //TVAR_afit_pBaseValS[29] //AFIT16_Sharpening_iHighSharpClamp_sBin 
		mt9t111_write(0x0F12 , 0x0A24); 	//70000800 //TVAR_afit_pBaseValS[30] //AFIT8_sddd8a_edge_low [7:0]	  AFIT8_sddd8a_repl_thresh [15:8]		 
		mt9t111_write(0x0F12 , 0x1701); 	//70000802 //TVAR_afit_pBaseValS[31] //AFIT8_sddd8a_repl_force [7:0]   AFIT8_sddd8a_sat_level [15:8]		 
		mt9t111_write(0x0F12 , 0x0229); 	//70000804 //TVAR_afit_pBaseValS[32] //AFIT8_sddd8a_sat_thr[7:0]   AFIT8_sddd8a_sat_mpl [15:8]				 
		mt9t111_write(0x0F12 , 0x1403); 	//70000806 //TVAR_afit_pBaseValS[33] //AFIT8_sddd8a_sat_noise[7:0]	 AFIT8_sddd8a_iMaxSlopeAllowed [15:8]	 
		mt9t111_write(0x0F12 , 0x0000); 	//70000808 //TVAR_afit_pBaseValS[34] //AFIT8_sddd8a_iHotThreshHigh[7:0]   AFIT8_sddd8a_iHotThreshLow [15:8]  
		mt9t111_write(0x0F12 , 0x0000); 	//7000080A //TVAR_afit_pBaseValS[35] //AFIT8_sddd8a_iColdThreshHigh[7:0]   AFIT8_sddd8a_iColdThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0000); 	//7000080C //TVAR_afit_pBaseValS[36] //AFIT8_sddd8a_AddNoisePower1[7:0]   AFIT8_sddd8a_AddNoisePower2 [15:8] 
		mt9t111_write(0x0F12 , 0x00FF); 	//7000080E //TVAR_afit_pBaseValS[37] //AFIT8_sddd8a_iSatSat[7:0]	AFIT8_sddd8a_iRadialTune [15:8] 		 
		mt9t111_write(0x0F12 , 0x045A); 	//70000810 //TVAR_afit_pBaseValS[38] //AFIT8_sddd8a_iRadialLimit [7:0]	  AFIT8_sddd8a_iRadialPower [15:8]
		mt9t111_write(0x0F12 , 0x1414); 	//70000812 //TVAR_afit_pBaseValS[39] //AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0]	AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
		mt9t111_write(0x0F12 , 0x0301); 	//70000814 //TVAR_afit_pBaseValS[40] //AFIT8_sddd8a_iLowSlopeThresh[7:0]	AFIT8_sddd8a_iHighSlopeThresh [15:8]		
		mt9t111_write(0x0F12 , 0xFF07); 	//70000816 //TVAR_afit_pBaseValS[41] //AFIT8_sddd8a_iSquaresRounding [7:0]	  AFIT8_Demosaicing_iCentGrad [15:8]		
		mt9t111_write(0x0F12 , 0x081E); 	//70000818 //TVAR_afit_pBaseValS[42] //AFIT8_Demosaicing_iMonochrom [7:0]	 AFIT8_Demosaicing_iDecisionThresh [15:8]	
		mt9t111_write(0x0F12 , 0x0A14); 	//7000081A //TVAR_afit_pBaseValS[43] //AFIT8_Demosaicing_iDesatThresh [7:0]    AFIT8_Demosaicing_iEnhThresh [15:8]		
		mt9t111_write(0x0F12 , 0x0F0F); 	//7000081C //TVAR_afit_pBaseValS[44] //AFIT8_Demosaicing_iGRDenoiseVal [7:0]	AFIT8_Demosaicing_iGBDenoiseVal [15:8]	
		mt9t111_write(0x0F12 , 0x0A00); 	//7000081E //TVAR_afit_pBaseValS[45] //AFIT8_Demosaicing_iNearGrayDesat[7:0]	AFIT8_Demosaicing_iDFD_ReduceCoeff [15:8]
		mt9t111_write(0x0F12 , 0x0032); 	//70000820 //TVAR_afit_pBaseValS[46] //AFIT8_Sharpening_iMSharpen [7:0]    AFIT8_Sharpening_iMShThresh [15:8]			
		mt9t111_write(0x0F12 , 0x000E); 	//70000822 //TVAR_afit_pBaseValS[47] //AFIT8_Sharpening_iWSharpen [7:0]    AFIT8_Sharpening_iWShThresh [15:8]			
		mt9t111_write(0x0F12 , 0x0002); 	//70000824 //TVAR_afit_pBaseValS[48] //AFIT8_Sharpening_nSharpWidth [7:0]	 AFIT8_Sharpening_iReduceNegative [15:8]	
		mt9t111_write(0x0F12 , 0x00FF); 	//70000826 //TVAR_afit_pBaseValS[49] //AFIT8_Sharpening_iShDespeckle [7:0]	 AFIT8_demsharpmix1_iRGBMultiplier [15:8]	
		mt9t111_write(0x0F12 , 0x1102); 	//70000828 //TVAR_afit_pBaseValS[50] //AFIT8_demsharpmix1_iFilterPower [7:0]   AFIT8_demsharpmix1_iBCoeff [15:8]		
		mt9t111_write(0x0F12 , 0x001B); 	//7000082A //TVAR_afit_pBaseValS[51] //AFIT8_demsharpmix1_iGCoeff [7:0]    AFIT8_demsharpmix1_iWideMult [15:8]			
		mt9t111_write(0x0F12 , 0x0900); 	//7000082C //TVAR_afit_pBaseValS[52] //AFIT8_demsharpmix1_iNarrMult [7:0]	 AFIT8_demsharpmix1_iHystFalloff [15:8] 	
		mt9t111_write(0x0F12 , 0x0600); 	//7000082E //TVAR_afit_pBaseValS[53] //AFIT8_demsharpmix1_iHystMinMult [7:0]	AFIT8_demsharpmix1_iHystWidth [15:8]	
		mt9t111_write(0x0F12 , 0x0504); 	//70000830 //TVAR_afit_pBaseValS[54] //AFIT8_demsharpmix1_iHystFallLow [7:0]	AFIT8_demsharpmix1_iHystFallHigh [15:8] 
		mt9t111_write(0x0F12 , 0x0306); 	//70000832 //TVAR_afit_pBaseValS[55] //AFIT8_demsharpmix1_iHystTune [7:0]	* AFIT8_YUV422_DENOISE_iUVSupport [15:8]	
		mt9t111_write(0x0F12 , 0x4603); 	//70000834 //TVAR_afit_pBaseValS[56] //AFIT8_YUV422_DENOISE_iYSupport [7:0]    AFIT8_byr_cgras_iShadingPower [15:8] 	
		mt9t111_write(0x0F12 , 0x0480); 	//70000836 //TVAR_afit_pBaseValS[57] //AFIT8_RGBGamma2_iLinearity [7:0]   AFIT8_RGBGamma2_iDarkReduce [15:8]			
		mt9t111_write(0x0F12 , 0x103C); 	//70000838 //TVAR_afit_pBaseValS[58] //AFIT8_ccm_oscar_iSaturation[7:0]    AFIT8_RGB2YUV_iYOffset [15:8]				
		mt9t111_write(0x0F12 , 0x0080); 	//7000083A //TVAR_afit_pBaseValS[59] //AFIT8_RGB2YUV_iRGBGain [7:0]    AFIT8_RGB2YUV_iSaturation [15:8] 				
		mt9t111_write(0x0F12 , 0x0101); 	//7000083C //TVAR_afit_pBaseValS[60] //AFIT8_sddd8a_iClustThresh_H [7:0]   AFIT8_sddd8a_iClustThresh_C [15:8]			
		mt9t111_write(0x0F12 , 0x0707); 	//7000083E //TVAR_afit_pBaseValS[61] //AFIT8_sddd8a_iClustMulT_H [7:0]	  AFIT8_sddd8a_iClustMulT_C [15:8]				
		mt9t111_write(0x0F12 , 0x4601); 	//70000840 //TVAR_afit_pBaseValS[62] //AFIT8_sddd8a_nClustLevel_H [7:0]    AFIT8_sddd8a_DispTH_Low [15:8]				
		mt9t111_write(0x0F12 , 0x8144); 	//70000842 //TVAR_afit_pBaseValS[63] //AFIT8_sddd8a_DispTH_High [7:0]	 AFIT8_sddd8a_iDenThreshLow [15:8]				
		mt9t111_write(0x0F12 , 0x5058); 	//70000844 //TVAR_afit_pBaseValS[64] //AFIT8_sddd8a_iDenThreshHigh[7:0]    AFIT8_Demosaicing_iEdgeDesat [15:8]			
		mt9t111_write(0x0F12 , 0x0500); 	//70000846 //TVAR_afit_pBaseValS[65] //AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]    AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
		mt9t111_write(0x0F12 , 0x0003); 	//70000848 //TVAR_afit_pBaseValS[66] //AFIT8_Demosaicing_iEdgeDesatLimit[7:0]	AFIT8_Demosaicing_iDemSharpenLow [15:8]
		mt9t111_write(0x0F12 , 0x5400); 	//7000084A //TVAR_afit_pBaseValS[67] //AFIT8_Demosaicing_iDemSharpenHigh[7:0]	 AFIT8_Demosaicing_iDemSharpThresh [15:8]
		mt9t111_write(0x0F12 , 0x0714); 	//7000084C //TVAR_afit_pBaseValS[68] //AFIT8_Demosaicing_iDemShLowLimit [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
		mt9t111_write(0x0F12 , 0x32FF); 	//7000084E //TVAR_afit_pBaseValS[69] //AFIT8_Demosaicing_iDemBlurLow[7:0]	 AFIT8_Demosaicing_iDemBlurHigh [15:8]			   
		mt9t111_write(0x0F12 , 0x5A04); 	//70000850 //TVAR_afit_pBaseValS[70] //AFIT8_Demosaicing_iDemBlurRange[7:0]    AFIT8_Sharpening_iLowSharpPower [15:8]		   
		mt9t111_write(0x0F12 , 0x201E); 	//70000852 //TVAR_afit_pBaseValS[71] //AFIT8_Sharpening_iHighSharpPower[7:0]	AFIT8_Sharpening_iLowShDenoise [15:8]		   
		mt9t111_write(0x0F12 , 0x4012); 	//70000854 //TVAR_afit_pBaseValS[72] //AFIT8_Sharpening_iHighShDenoise [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult [15:8]	   
		mt9t111_write(0x0F12 , 0x0204); 	//70000856 //TVAR_afit_pBaseValS[73] //AFIT8_Sharpening_iReduceEdgeSlope [7:0]	 AFIT8_demsharpmix1_iWideFiltReduce [15:8]	   
		mt9t111_write(0x0F12 , 0x1403); 	//70000858 //TVAR_afit_pBaseValS[74] //AFIT8_demsharpmix1_iNarrFiltReduce [7:0]   AFIT8_sddd8a_iClustThresh_H_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0114); 	//7000085A //TVAR_afit_pBaseValS[75] //AFIT8_sddd8a_iClustThresh_C_Bin [7:0]	AFIT8_sddd8a_iClustMulT_H_Bin [15:8]		   
		mt9t111_write(0x0F12 , 0x0101); 	//7000085C //TVAR_afit_pBaseValS[76] //AFIT8_sddd8a_iClustMulT_C_Bin [7:0]	  AFIT8_sddd8a_nClustLevel_H_Bin [15:8] 		   
		mt9t111_write(0x0F12 , 0x4446); 	//7000085E //TVAR_afit_pBaseValS[77] //AFIT8_sddd8a_DispTH_Low_Bin [7:0]	AFIT8_sddd8a_DispTH_High_Bin [15:8] 			   
		mt9t111_write(0x0F12 , 0x646E); 	//70000860 //TVAR_afit_pBaseValS[78] //AFIT8_sddd8a_iDenThreshLow_Bin [7:0]    AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]		   
		mt9t111_write(0x0F12 , 0x0028); 	//70000862 //TVAR_afit_pBaseValS[79] //AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]	AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]  
		mt9t111_write(0x0F12 , 0x030A); 	//70000864 //TVAR_afit_pBaseValS[80] //AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]   AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0000); 	//70000866 //TVAR_afit_pBaseValS[81] //AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]	AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]   
		mt9t111_write(0x0F12 , 0x141E); 	//70000868 //TVAR_afit_pBaseValS[82] //AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]	 AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]   
		mt9t111_write(0x0F12 , 0xFF07); 	//7000086A //TVAR_afit_pBaseValS[83] //AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]   AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0432); 	//7000086C //TVAR_afit_pBaseValS[84] //AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]   AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0000); 	//7000086E //TVAR_afit_pBaseValS[85] //AFIT8_Sharpening_iLowSharpPower_Bin [7:0]   AFIT8_Sharpening_iHighSharpPower_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0F0F); 	//70000870 //TVAR_afit_pBaseValS[86] //AFIT8_Sharpening_iLowShDenoise_Bin [7:0]   AFIT8_Sharpening_iHighShDenoise_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0440); 	//70000872 //TVAR_afit_pBaseValS[87] //AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]   AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0302); 	//70000874 //TVAR_afit_pBaseValS[88] //AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]   AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
		mt9t111_write(0x0F12 , 0x1414); 	//70000876 //TVAR_afit_pBaseValS[89] //AFIT8_sddd8a_iClustThresh_H_sBin[7:0]	AFIT8_sddd8a_iClustThresh_C_sBin [15:8] 		   
		mt9t111_write(0x0F12 , 0x0101); 	//70000878 //TVAR_afit_pBaseValS[90] //AFIT8_sddd8a_iClustMulT_H_sBin [7:0]    AFIT8_sddd8a_iClustMulT_C_sBin [15:8]			   
		mt9t111_write(0x0F12 , 0x4601); 	//7000087A //TVAR_afit_pBaseValS[91] //AFIT8_sddd8a_nClustLevel_H_sBin [7:0]	AFIT8_sddd8a_DispTH_Low_sBin [15:8] 			   
		mt9t111_write(0x0F12 , 0x6E44); 	//7000087C //TVAR_afit_pBaseValS[92] //AFIT8_sddd8a_DispTH_High_sBin [7:0]	  AFIT8_sddd8a_iDenThreshLow_sBin [15:8]			   
		mt9t111_write(0x0F12 , 0x2864); 	//7000087E //TVAR_afit_pBaseValS[93] //AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]	AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]		   
		mt9t111_write(0x0F12 , 0x0A00); 	//70000880 //TVAR_afit_pBaseValS[94] //AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
		mt9t111_write(0x0F12 , 0x0003); 	//70000882 //TVAR_afit_pBaseValS[95] //AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]   AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]	  
		mt9t111_write(0x0F12 , 0x1E00); 	//70000884 //TVAR_afit_pBaseValS[96] //AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]   AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]   
		mt9t111_write(0x0F12 , 0x0714); 	//70000886 //TVAR_afit_pBaseValS[97] //AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
		mt9t111_write(0x0F12 , 0x32FF); 	//70000888 //TVAR_afit_pBaseValS[98] //AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]   AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]			
		mt9t111_write(0x0F12 , 0x0004); 	//7000088A //TVAR_afit_pBaseValS[99] //AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]	AFIT8_Sharpening_iLowSharpPower_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x0F00); 	//7000088C //TVAR_afit_pBaseValS[100] /AFIT8_Sharpening_iHighSharpPower_sBin [7:0]	 AFIT8_Sharpening_iLowShDenoise_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x400F); 	//7000088E //TVAR_afit_pBaseValS[101] /AFIT8_Sharpening_iHighShDenoise_sBin [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8] 	
		mt9t111_write(0x0F12 , 0x0204); 	//70000890 //TVAR_afit_pBaseValS[102] /AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]   AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]	
		mt9t111_write(0x0F12 , 0x0003); 	//70000892 //TVAR_afit_pBaseValS[103] /AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
		mt9t111_write(0x0F12 , 0x0000); 	//70000894 //TVAR_afit_pBaseValS[104] /AFIT16_BRIGHTNESS				   
		mt9t111_write(0x0F12 , 0x0000); 	//70000896 //TVAR_afit_pBaseValS[105] /AFIT16_CONTRAST					   
		mt9t111_write(0x0F12 , 0x0020); 	//70000898 //TVAR_afit_pBaseValS[106] /AFIT16_SATURATION				   
		mt9t111_write(0x0F12 , 0xFFD6); 	//7000089A //TVAR_afit_pBaseValS[107] /AFIT16_SHARP_BLUR				   
		mt9t111_write(0x0F12 , 0x0000); 	//7000089C //TVAR_afit_pBaseValS[108] /AFIT16_GLAMOUR					   
		mt9t111_write(0x0F12 , 0x00C1); 	//7000089E //TVAR_afit_pBaseValS[109] /AFIT16_sddd8a_edge_high			   
		mt9t111_write(0x0F12 , 0x03FF); 	//700008A0 //TVAR_afit_pBaseValS[110] /AFIT16_Demosaicing_iSatVal		   
		mt9t111_write(0x0F12 , 0x009C); 	//700008A2 //TVAR_afit_pBaseValS[111] /AFIT16_Sharpening_iReduceEdgeThresh 
		mt9t111_write(0x0F12 , 0x0251); 	//700008A4 //TVAR_afit_pBaseValS[112] /AFIT16_demsharpmix1_iRGBOffset	   
		mt9t111_write(0x0F12 , 0x03FF); 	//700008A6 //TVAR_afit_pBaseValS[113] /AFIT16_demsharpmix1_iDemClamp	   
		mt9t111_write(0x0F12 , 0x000C); 	//700008A8 //TVAR_afit_pBaseValS[114] /AFIT16_demsharpmix1_iLowThreshold   
		mt9t111_write(0x0F12 , 0x0010); 	//700008AA //TVAR_afit_pBaseValS[115] /AFIT16_demsharpmix1_iHighThreshold  
		mt9t111_write(0x0F12 , 0x012C); 	//700008AC //TVAR_afit_pBaseValS[116] /AFIT16_demsharpmix1_iLowBright	   
		mt9t111_write(0x0F12 , 0x03E8); 	//700008AE //TVAR_afit_pBaseValS[117] /AFIT16_demsharpmix1_iHighBright	   
		mt9t111_write(0x0F12 , 0x0046); 	//700008B0 //TVAR_afit_pBaseValS[118] /AFIT16_demsharpmix1_iLowSat		   
		mt9t111_write(0x0F12 , 0x005A); 	//700008B2 //TVAR_afit_pBaseValS[119] /AFIT16_demsharpmix1_iHighSat 	   
		mt9t111_write(0x0F12 , 0x0070); 	//700008B4 //TVAR_afit_pBaseValS[120] /AFIT16_demsharpmix1_iTune		   
		mt9t111_write(0x0F12 , 0x0000); 	//700008B6 //TVAR_afit_pBaseValS[121] /AFIT16_demsharpmix1_iHystThLow	   
		mt9t111_write(0x0F12 , 0x0000); 	//700008B8 //TVAR_afit_pBaseValS[122] /AFIT16_demsharpmix1_iHystThHigh	   
		mt9t111_write(0x0F12 , 0x01AE); 	//700008BA //TVAR_afit_pBaseValS[123] /AFIT16_demsharpmix1_iHystCenter	   
		mt9t111_write(0x0F12 , 0x001E); 	//700008BC //TVAR_afit_pBaseValS[124] /AFIT16_YUV422_DENOISE_iUVLowThresh  
		mt9t111_write(0x0F12 , 0x001E); 	//700008BE //TVAR_afit_pBaseValS[125] /AFIT16_YUV422_DENOISE_iUVHighThresh 
		mt9t111_write(0x0F12 , 0x0000); 	//700008C0 //TVAR_afit_pBaseValS[126] /AFIT16_YUV422_DENOISE_iYLowThresh   
		mt9t111_write(0x0F12 , 0x0000); 	//700008C2 //TVAR_afit_pBaseValS[127] /AFIT16_YUV422_DENOISE_iYHighThresh  
		mt9t111_write(0x0F12 , 0x003E); 	//700008C4 //TVAR_afit_pBaseValS[128] /AFIT16_Sharpening_iLowSharpClamp    
		mt9t111_write(0x0F12 , 0x0008); 	//700008C6 //TVAR_afit_pBaseValS[129] /AFIT16_Sharpening_iHighSharpClamp   
		mt9t111_write(0x0F12 , 0x003C); 	//700008C8 //TVAR_afit_pBaseValS[130] /AFIT16_Sharpening_iLowSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x001E); 	//700008CA //TVAR_afit_pBaseValS[131] /AFIT16_Sharpening_iHighSharpClamp_Bin 
		mt9t111_write(0x0F12 , 0x003C); 	//700008CC //TVAR_afit_pBaseValS[132] /AFIT16_Sharpening_iLowSharpClamp_sBin 
		mt9t111_write(0x0F12 , 0x001E); 	//700008CE //TVAR_afit_pBaseValS[133] /AFIT16_Sharpening_iHighSharpClamp_sBin
		mt9t111_write(0x0F12 , 0x0A24); 	//700008D0 //TVAR_afit_pBaseValS[134] /AFIT8_sddd8a_edge_low [7:0]	  AFIT8_sddd8a_repl_thresh [15:8]
		mt9t111_write(0x0F12 , 0x1701); 	//700008D2 //TVAR_afit_pBaseValS[135] /AFIT8_sddd8a_repl_force [7:0]   AFIT8_sddd8a_sat_level [15:8] 
		mt9t111_write(0x0F12 , 0x0229); 	//700008D4 //TVAR_afit_pBaseValS[136] /AFIT8_sddd8a_sat_thr[7:0]   AFIT8_sddd8a_sat_mpl [15:8]		 
		mt9t111_write(0x0F12 , 0x1403); 	//700008D6 //TVAR_afit_pBaseValS[137] /AFIT8_sddd8a_sat_noise[7:0]	 AFIT8_sddd8a_iMaxSlopeAllowed [15:8]  
		mt9t111_write(0x0F12 , 0x0000); 	//700008D8 //TVAR_afit_pBaseValS[138] /AFIT8_sddd8a_iHotThreshHigh[7:0]   AFIT8_sddd8a_iHotThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0000); 	//700008DA //TVAR_afit_pBaseValS[139] /AFIT8_sddd8a_iColdThreshHigh[7:0]   AFIT8_sddd8a_iColdThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0000); 	//700008DC //TVAR_afit_pBaseValS[140] /AFIT8_sddd8a_AddNoisePower1[7:0]   AFIT8_sddd8a_AddNoisePower2 [15:8] 
		mt9t111_write(0x0F12 , 0x00FF); 	//700008DE //TVAR_afit_pBaseValS[141] /AFIT8_sddd8a_iSatSat[7:0]	AFIT8_sddd8a_iRadialTune [15:8] 		 
		mt9t111_write(0x0F12 , 0x045A); 	//700008E0 //TVAR_afit_pBaseValS[142] /AFIT8_sddd8a_iRadialLimit [7:0]	  AFIT8_sddd8a_iRadialPower [15:8]	 
		mt9t111_write(0x0F12 , 0x1414); 	//700008E2 //TVAR_afit_pBaseValS[143] /AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0]	AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
		mt9t111_write(0x0F12 , 0x0301); 	//700008E4 //TVAR_afit_pBaseValS[144] /AFIT8_sddd8a_iLowSlopeThresh[7:0]	AFIT8_sddd8a_iHighSlopeThresh [15:8]		
		mt9t111_write(0x0F12 , 0xFF07); 	//700008E6 //TVAR_afit_pBaseValS[145] /AFIT8_sddd8a_iSquaresRounding [7:0]	  AFIT8_Demosaicing_iCentGrad [15:8]		
		mt9t111_write(0x0F12 , 0x081E); 	//700008E8 //TVAR_afit_pBaseValS[146] /AFIT8_Demosaicing_iMonochrom [7:0]	 AFIT8_Demosaicing_iDecisionThresh [15:8]	
		mt9t111_write(0x0F12 , 0x0A14); 	//700008EA //TVAR_afit_pBaseValS[147] /AFIT8_Demosaicing_iDesatThresh [7:0]    AFIT8_Demosaicing_iEnhThresh [15:8]		
		mt9t111_write(0x0F12 , 0x0F0F); 	//700008EC //TVAR_afit_pBaseValS[148] /AFIT8_Demosaicing_iGRDenoiseVal [7:0]	AFIT8_Demosaicing_iGBDenoiseVal [15:8]	
		mt9t111_write(0x0F12 , 0x0A00); 	//700008EE //TVAR_afit_pBaseValS[149] /AFIT8_Demosaicing_iNearGrayDesat[7:0]	AFIT8_Demosaicing_iDFD_ReduceCoeff [15:8]
		mt9t111_write(0x0F12 , 0x0032); 	//700008F0 //TVAR_afit_pBaseValS[150] /AFIT8_Sharpening_iMSharpen [7:0]    AFIT8_Sharpening_iMShThresh [15:8] 
		mt9t111_write(0x0F12 , 0x000E); 	//700008F2 //TVAR_afit_pBaseValS[151] /AFIT8_Sharpening_iWSharpen [7:0]    AFIT8_Sharpening_iWShThresh [15:8] 
		mt9t111_write(0x0F12 , 0x0002); 	//700008F4 //TVAR_afit_pBaseValS[152] /AFIT8_Sharpening_nSharpWidth [7:0]	 AFIT8_Sharpening_iReduceNegative [15:8] 
		mt9t111_write(0x0F12 , 0x00FF); 	//700008F6 //TVAR_afit_pBaseValS[153] /AFIT8_Sharpening_iShDespeckle [7:0]	 AFIT8_demsharpmix1_iRGBMultiplier [15:8]
		mt9t111_write(0x0F12 , 0x1102); 	//700008F8 //TVAR_afit_pBaseValS[154] /AFIT8_demsharpmix1_iFilterPower [7:0]   AFIT8_demsharpmix1_iBCoeff [15:8]	 
		mt9t111_write(0x0F12 , 0x001B); 	//700008FA //TVAR_afit_pBaseValS[155] /AFIT8_demsharpmix1_iGCoeff [7:0]    AFIT8_demsharpmix1_iWideMult [15:8]		 
		mt9t111_write(0x0F12 , 0x0900); 	//700008FC //TVAR_afit_pBaseValS[156] /AFIT8_demsharpmix1_iNarrMult [7:0]	 AFIT8_demsharpmix1_iHystFalloff [15:8]  
		mt9t111_write(0x0F12 , 0x0600); 	//700008FE //TVAR_afit_pBaseValS[157] /AFIT8_demsharpmix1_iHystMinMult [7:0]	AFIT8_demsharpmix1_iHystWidth [15:8] 
		mt9t111_write(0x0F12 , 0x0504); 	//70000900 //TVAR_afit_pBaseValS[158] /AFIT8_demsharpmix1_iHystFallLow [7:0]	AFIT8_demsharpmix1_iHystFallHigh [15:8]
		mt9t111_write(0x0F12 , 0x0306); 	//70000902 //TVAR_afit_pBaseValS[159] /AFIT8_demsharpmix1_iHystTune [7:0]	* AFIT8_YUV422_DENOISE_iUVSupport [15:8] 
		mt9t111_write(0x0F12 , 0x4603); 	//70000904 //TVAR_afit_pBaseValS[160] /AFIT8_YUV422_DENOISE_iYSupport [7:0]    AFIT8_byr_cgras_iShadingPower [15:8]  
		mt9t111_write(0x0F12 , 0x0480); 	//70000906 //TVAR_afit_pBaseValS[161] /AFIT8_RGBGamma2_iLinearity [7:0]   AFIT8_RGBGamma2_iDarkReduce [15:8]		 
		mt9t111_write(0x0F12 , 0x1046); 	//70000908 //TVAR_afit_pBaseValS[162] /AFIT8_ccm_oscar_iSaturation[7:0]    AFIT8_RGB2YUV_iYOffset [15:8]			 
		mt9t111_write(0x0F12 , 0x0080); 	//7000090A //TVAR_afit_pBaseValS[163] /AFIT8_RGB2YUV_iRGBGain [7:0]    AFIT8_RGB2YUV_iSaturation [15:8] 			 
		mt9t111_write(0x0F12 , 0x0101); 	//7000090C //TVAR_afit_pBaseValS[164] /AFIT8_sddd8a_iClustThresh_H [7:0]   AFIT8_sddd8a_iClustThresh_C [15:8]		 
		mt9t111_write(0x0F12 , 0x0707); 	//7000090E //TVAR_afit_pBaseValS[165] /AFIT8_sddd8a_iClustMulT_H [7:0]	  AFIT8_sddd8a_iClustMulT_C [15:8]			 
		mt9t111_write(0x0F12 , 0x1E01); 	//70000910 //TVAR_afit_pBaseValS[166] /AFIT8_sddd8a_nClustLevel_H [7:0]    AFIT8_sddd8a_DispTH_Low [15:8]			 
		mt9t111_write(0x0F12 , 0x811E); 	//70000912 //TVAR_afit_pBaseValS[167] /AFIT8_sddd8a_DispTH_High [7:0]	 AFIT8_sddd8a_iDenThreshLow [15:8]			 
		mt9t111_write(0x0F12 , 0x5058); 	//70000914 //TVAR_afit_pBaseValS[168] /AFIT8_sddd8a_iDenThreshHigh[7:0]    AFIT8_Demosaicing_iEdgeDesat [15:8]		 
		mt9t111_write(0x0F12 , 0x0500); 	//70000916 //TVAR_afit_pBaseValS[169] /AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]    AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8] 
		mt9t111_write(0x0F12 , 0x0004); 	//70000918 //TVAR_afit_pBaseValS[170] /AFIT8_Demosaicing_iEdgeDesatLimit[7:0]	AFIT8_Demosaicing_iDemSharpenLow [15:8] 	  
		mt9t111_write(0x0F12 , 0x3C0A); 	//7000091A //TVAR_afit_pBaseValS[171] /AFIT8_Demosaicing_iDemSharpenHigh[7:0]	 AFIT8_Demosaicing_iDemSharpThresh [15:8]	  
		mt9t111_write(0x0F12 , 0x0714); 	//7000091C //TVAR_afit_pBaseValS[172] /AFIT8_Demosaicing_iDemShLowLimit [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
		mt9t111_write(0x0F12 , 0x3214); 	//7000091E //TVAR_afit_pBaseValS[173] /AFIT8_Demosaicing_iDemBlurLow[7:0]	 AFIT8_Demosaicing_iDemBlurHigh [15:8]			   
		mt9t111_write(0x0F12 , 0x5A03); 	//70000920 //TVAR_afit_pBaseValS[174] /AFIT8_Demosaicing_iDemBlurRange[7:0]    AFIT8_Sharpening_iLowSharpPower [15:8]		   
		mt9t111_write(0x0F12 , 0x121E); 	//70000922 //TVAR_afit_pBaseValS[175] /AFIT8_Sharpening_iHighSharpPower[7:0]	AFIT8_Sharpening_iLowShDenoise [15:8]		   
		mt9t111_write(0x0F12 , 0x4012); 	//70000924 //TVAR_afit_pBaseValS[176] /AFIT8_Sharpening_iHighShDenoise [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult [15:8]	   
		mt9t111_write(0x0F12 , 0x0604); 	//70000926 //TVAR_afit_pBaseValS[177] /AFIT8_Sharpening_iReduceEdgeSlope [7:0]	 AFIT8_demsharpmix1_iWideFiltReduce [15:8]	   
		mt9t111_write(0x0F12 , 0x1E06); 	//70000928 //TVAR_afit_pBaseValS[178] /AFIT8_demsharpmix1_iNarrFiltReduce [7:0]   AFIT8_sddd8a_iClustThresh_H_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x011E); 	//7000092A //TVAR_afit_pBaseValS[179] /AFIT8_sddd8a_iClustThresh_C_Bin [7:0]	AFIT8_sddd8a_iClustMulT_H_Bin [15:8]		   
		mt9t111_write(0x0F12 , 0x0101); 	//7000092C //TVAR_afit_pBaseValS[180] /AFIT8_sddd8a_iClustMulT_C_Bin [7:0]	  AFIT8_sddd8a_nClustLevel_H_Bin [15:8] 		   
		mt9t111_write(0x0F12 , 0x3A3C); 	//7000092E //TVAR_afit_pBaseValS[181] /AFIT8_sddd8a_DispTH_Low_Bin [7:0]	AFIT8_sddd8a_DispTH_High_Bin [15:8] 			   
		mt9t111_write(0x0F12 , 0x585A); 	//70000930 //TVAR_afit_pBaseValS[182] /AFIT8_sddd8a_iDenThreshLow_Bin [7:0]    AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]		   
		mt9t111_write(0x0F12 , 0x0028); 	//70000932 //TVAR_afit_pBaseValS[183] /AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]	AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]  
		mt9t111_write(0x0F12 , 0x030A); 	//70000934 //TVAR_afit_pBaseValS[184] /AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]   AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0000); 	//70000936 //TVAR_afit_pBaseValS[185] /AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]	AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]   
		mt9t111_write(0x0F12 , 0x141E); 	//70000938 //TVAR_afit_pBaseValS[186] /AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]	 AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]   
		mt9t111_write(0x0F12 , 0xFF07); 	//7000093A //TVAR_afit_pBaseValS[187] /AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]   AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0432); 	//7000093C //TVAR_afit_pBaseValS[188] /AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]   AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0000); 	//7000093E //TVAR_afit_pBaseValS[189] /AFIT8_Sharpening_iLowSharpPower_Bin [7:0]   AFIT8_Sharpening_iHighSharpPower_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0F0F); 	//70000940 //TVAR_afit_pBaseValS[190] /AFIT8_Sharpening_iLowShDenoise_Bin [7:0]   AFIT8_Sharpening_iHighShDenoise_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0440); 	//70000942 //TVAR_afit_pBaseValS[191] /AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]   AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0302); 	//70000944 //TVAR_afit_pBaseValS[192] /AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]   AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
		mt9t111_write(0x0F12 , 0x1E1E); 	//70000946 //TVAR_afit_pBaseValS[193] /AFIT8_sddd8a_iClustThresh_H_sBin[7:0]	AFIT8_sddd8a_iClustThresh_C_sBin [15:8] 		   
		mt9t111_write(0x0F12 , 0x0101); 	//70000948 //TVAR_afit_pBaseValS[194] /AFIT8_sddd8a_iClustMulT_H_sBin [7:0]    AFIT8_sddd8a_iClustMulT_C_sBin [15:8]			   
		mt9t111_write(0x0F12 , 0x3C01); 	//7000094A //TVAR_afit_pBaseValS[195] /AFIT8_sddd8a_nClustLevel_H_sBin [7:0]	AFIT8_sddd8a_DispTH_Low_sBin [15:8] 			   
		mt9t111_write(0x0F12 , 0x5A3A); 	//7000094C //TVAR_afit_pBaseValS[196] /AFIT8_sddd8a_DispTH_High_sBin [7:0]	  AFIT8_sddd8a_iDenThreshLow_sBin [15:8]			   
		mt9t111_write(0x0F12 , 0x2858); 	//7000094E //TVAR_afit_pBaseValS[197] /AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]	AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]		   
		mt9t111_write(0x0F12 , 0x0A00); 	//70000950 //TVAR_afit_pBaseValS[198] /AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
		mt9t111_write(0x0F12 , 0x0003); 	//70000952 //TVAR_afit_pBaseValS[199] /AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]   AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]	  
		mt9t111_write(0x0F12 , 0x1E00); 	//70000954 //TVAR_afit_pBaseValS[200] /AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]   AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]   
		mt9t111_write(0x0F12 , 0x0714); 	//70000956 //TVAR_afit_pBaseValS[201] /AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
		mt9t111_write(0x0F12 , 0x32FF); 	//70000958 //TVAR_afit_pBaseValS[202] /AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]   AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]			
		mt9t111_write(0x0F12 , 0x0004); 	//7000095A //TVAR_afit_pBaseValS[203] /AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]	AFIT8_Sharpening_iLowSharpPower_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x0F00); 	//7000095C //TVAR_afit_pBaseValS[204] /AFIT8_Sharpening_iHighSharpPower_sBin [7:0]	 AFIT8_Sharpening_iLowShDenoise_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x400F); 	//7000095E //TVAR_afit_pBaseValS[205] /AFIT8_Sharpening_iHighShDenoise_sBin [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8] 	
		mt9t111_write(0x0F12 , 0x0204); 	//70000960 //TVAR_afit_pBaseValS[206] /AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]   AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]	
		mt9t111_write(0x0F12 , 0x0003); 	//70000962 //TVAR_afit_pBaseValS[207] /AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
		mt9t111_write(0x0F12 , 0x0000); 	//70000964 //TVAR_afit_pBaseValS[208] /AFIT16_BRIGHTNESS  
		mt9t111_write(0x0F12 , 0x0000); 	//70000966 //TVAR_afit_pBaseValS[209] /AFIT16_CONTRAST	  
		mt9t111_write(0x0F12 , 0x0020); 	//70000968 //TVAR_afit_pBaseValS[210] /AFIT16_SATURATION  
		mt9t111_write(0x0F12 , 0x0000); 	//7000096A //TVAR_afit_pBaseValS[211] /AFIT16_SHARP_BLUR  
		mt9t111_write(0x0F12 , 0x0000); 	//7000096C //TVAR_afit_pBaseValS[212] /AFIT16_GLAMOUR	  
		mt9t111_write(0x0F12 , 0x00C1); 	//7000096E //TVAR_afit_pBaseValS[213] /AFIT16_sddd8a_edge_high			  
		mt9t111_write(0x0F12 , 0x03FF); 	//70000970 //TVAR_afit_pBaseValS[214] /AFIT16_Demosaicing_iSatVal		  
		mt9t111_write(0x0F12 , 0x009C); 	//70000972 //TVAR_afit_pBaseValS[215] /AFIT16_Sharpening_iReduceEdgeThresh
		mt9t111_write(0x0F12 , 0x0251); 	//70000974 //TVAR_afit_pBaseValS[216] /AFIT16_demsharpmix1_iRGBOffset	  
		mt9t111_write(0x0F12 , 0x03FF); 	//70000976 //TVAR_afit_pBaseValS[217] /AFIT16_demsharpmix1_iDemClamp	  
		mt9t111_write(0x0F12 , 0x000C); 	//70000978 //TVAR_afit_pBaseValS[218] /AFIT16_demsharpmix1_iLowThreshold  
		mt9t111_write(0x0F12 , 0x0010); 	//7000097A //TVAR_afit_pBaseValS[219] /AFIT16_demsharpmix1_iHighThreshold 
		mt9t111_write(0x0F12 , 0x012C); 	//7000097C //TVAR_afit_pBaseValS[220] /AFIT16_demsharpmix1_iLowBright	  
		mt9t111_write(0x0F12 , 0x03E8); 	//7000097E //TVAR_afit_pBaseValS[221] /AFIT16_demsharpmix1_iHighBright	  
		mt9t111_write(0x0F12 , 0x0046); 	//70000980 //TVAR_afit_pBaseValS[222] /AFIT16_demsharpmix1_iLowSat		  
		mt9t111_write(0x0F12 , 0x005A); 	//70000982 //TVAR_afit_pBaseValS[223] /AFIT16_demsharpmix1_iHighSat 	  
		mt9t111_write(0x0F12 , 0x0070); 	//70000984 //TVAR_afit_pBaseValS[224] /AFIT16_demsharpmix1_iTune		  
		mt9t111_write(0x0F12 , 0x0000); 	//70000986 //TVAR_afit_pBaseValS[225] /AFIT16_demsharpmix1_iHystThLow	  
		mt9t111_write(0x0F12 , 0x0000); 	//70000988 //TVAR_afit_pBaseValS[226] /AFIT16_demsharpmix1_iHystThHigh	  
		mt9t111_write(0x0F12 , 0x0226); 	//7000098A //TVAR_afit_pBaseValS[227] /AFIT16_demsharpmix1_iHystCenter	  
		mt9t111_write(0x0F12 , 0x001E); 	//7000098C //TVAR_afit_pBaseValS[228] /AFIT16_YUV422_DENOISE_iUVLowThresh 
		mt9t111_write(0x0F12 , 0x001E); 	//7000098E //TVAR_afit_pBaseValS[229] /AFIT16_YUV422_DENOISE_iUVHighThresh
		mt9t111_write(0x0F12 , 0x0000); 	//70000990 //TVAR_afit_pBaseValS[230] /AFIT16_YUV422_DENOISE_iYLowThresh   
		mt9t111_write(0x0F12 , 0x0000); 	//70000992 //TVAR_afit_pBaseValS[231] /AFIT16_YUV422_DENOISE_iYHighThresh  
		mt9t111_write(0x0F12 , 0x004E); 	//70000994 //TVAR_afit_pBaseValS[232] /AFIT16_Sharpening_iLowSharpClamp    
		mt9t111_write(0x0F12 , 0x0000); 	//70000996 //TVAR_afit_pBaseValS[233] /AFIT16_Sharpening_iHighSharpClamp   
		mt9t111_write(0x0F12 , 0x003C); 	//70000998 //TVAR_afit_pBaseValS[234] /AFIT16_Sharpening_iLowSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x001E); 	//7000099A //TVAR_afit_pBaseValS[235] /AFIT16_Sharpening_iHighSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x003C); 	//7000099C //TVAR_afit_pBaseValS[236] /AFIT16_Sharpening_iLowSharpClamp_sBin
		mt9t111_write(0x0F12 , 0x001E); 	//7000099E //TVAR_afit_pBaseValS[237] /AFIT16_Sharpening_iHighSharpClamp_sBin 
		mt9t111_write(0x0F12 , 0x0A24); 	//700009A0 //TVAR_afit_pBaseValS[238] /AFIT8_sddd8a_edge_low [7:0]	  AFIT8_sddd8a_repl_thresh [15:8]
		mt9t111_write(0x0F12 , 0x1701); 	//700009A2 //TVAR_afit_pBaseValS[239] /AFIT8_sddd8a_repl_force [7:0]   AFIT8_sddd8a_sat_level [15:8] 
		mt9t111_write(0x0F12 , 0x0229); 	//700009A4 //TVAR_afit_pBaseValS[240] /AFIT8_sddd8a_sat_thr[7:0]   AFIT8_sddd8a_sat_mpl [15:8]		 
		mt9t111_write(0x0F12 , 0x1403); 	//700009A6 //TVAR_afit_pBaseValS[241] /AFIT8_sddd8a_sat_noise[7:0]	 AFIT8_sddd8a_iMaxSlopeAllowed [15:8]	 
		mt9t111_write(0x0F12 , 0x0000); 	//700009A8 //TVAR_afit_pBaseValS[242] /AFIT8_sddd8a_iHotThreshHigh[7:0]   AFIT8_sddd8a_iHotThreshLow [15:8]  
		mt9t111_write(0x0F12 , 0x0000); 	//700009AA //TVAR_afit_pBaseValS[243] /AFIT8_sddd8a_iColdThreshHigh[7:0]   AFIT8_sddd8a_iColdThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0906); 	//700009AC //TVAR_afit_pBaseValS[244] /AFIT8_sddd8a_AddNoisePower1[7:0]   AFIT8_sddd8a_AddNoisePower2 [15:8] 
		mt9t111_write(0x0F12 , 0x00FF); 	//700009AE //TVAR_afit_pBaseValS[245] /AFIT8_sddd8a_iSatSat[7:0]	AFIT8_sddd8a_iRadialTune [15:8] 		 
		mt9t111_write(0x0F12 , 0x045A); 	//700009B0 //TVAR_afit_pBaseValS[246] /AFIT8_sddd8a_iRadialLimit [7:0]	  AFIT8_sddd8a_iRadialPower [15:8]	 
		mt9t111_write(0x0F12 , 0x1414); 	//700009B2 //TVAR_afit_pBaseValS[247] /AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0]	AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
		mt9t111_write(0x0F12 , 0x0301); 	//700009B4 //TVAR_afit_pBaseValS[248] /AFIT8_sddd8a_iLowSlopeThresh[7:0]	AFIT8_sddd8a_iHighSlopeThresh [15:8]		
		mt9t111_write(0x0F12 , 0xFF07); 	//700009B6 //TVAR_afit_pBaseValS[249] /AFIT8_sddd8a_iSquaresRounding [7:0]	  AFIT8_Demosaicing_iCentGrad [15:8]		
		mt9t111_write(0x0F12 , 0x081E); 	//700009B8 //TVAR_afit_pBaseValS[250] /AFIT8_Demosaicing_iMonochrom [7:0]	 AFIT8_Demosaicing_iDecisionThresh [15:8]	
		mt9t111_write(0x0F12 , 0x0A14); 	//700009BA //TVAR_afit_pBaseValS[251] /AFIT8_Demosaicing_iDesatThresh [7:0]    AFIT8_Demosaicing_iEnhThresh [15:8]		
		mt9t111_write(0x0F12 , 0x0F0F); 	//700009BC //TVAR_afit_pBaseValS[252] /AFIT8_Demosaicing_iGRDenoiseVal [7:0]	AFIT8_Demosaicing_iGBDenoiseVal [15:8]	
		mt9t111_write(0x0F12 , 0x0A00);    //05 //0A05	//700009BE //TVAR_afit_pBaseValS[253] /AFIT8_Demosaicing_iNearGrayDesat[7:0]	AFIT8_Demosaicing_iDFD_ReduceCoeff [15:8]
		mt9t111_write(0x0F12 , 0x0090); 	//700009C0 //TVAR_afit_pBaseValS[254] /AFIT8_Sharpening_iMSharpen [7:0]    AFIT8_Sharpening_iMShThresh [15:8]			 
		mt9t111_write(0x0F12 , 0x000A); 	//700009C2 //TVAR_afit_pBaseValS[255] /AFIT8_Sharpening_iWSharpen [7:0]    AFIT8_Sharpening_iWShThresh [15:8]			 
		mt9t111_write(0x0F12 , 0x0002); 	//700009C4 //TVAR_afit_pBaseValS[256] /AFIT8_Sharpening_nSharpWidth [7:0]	 AFIT8_Sharpening_iReduceNegative [15:8]	 
		mt9t111_write(0x0F12 , 0x00FF); 	//700009C6 //TVAR_afit_pBaseValS[257] /AFIT8_Sharpening_iShDespeckle [7:0]	 AFIT8_demsharpmix1_iRGBMultiplier [15:8]	 
		mt9t111_write(0x0F12 , 0x1102); 	//700009C8 //TVAR_afit_pBaseValS[258] /AFIT8_demsharpmix1_iFilterPower [7:0]   AFIT8_demsharpmix1_iBCoeff [15:8]		 
		mt9t111_write(0x0F12 , 0x001B); 	//700009CA //TVAR_afit_pBaseValS[259] /AFIT8_demsharpmix1_iGCoeff [7:0]    AFIT8_demsharpmix1_iWideMult [15:8]			 
		mt9t111_write(0x0F12 , 0x0900); 	//700009CC //TVAR_afit_pBaseValS[260] /AFIT8_demsharpmix1_iNarrMult [7:0]	 AFIT8_demsharpmix1_iHystFalloff [15:8] 	 
		mt9t111_write(0x0F12 , 0x0600); 	//700009CE //TVAR_afit_pBaseValS[261] /AFIT8_demsharpmix1_iHystMinMult [7:0]	AFIT8_demsharpmix1_iHystWidth [15:8]	 
		mt9t111_write(0x0F12 , 0x0504); 	//700009D0 //TVAR_afit_pBaseValS[262] /AFIT8_demsharpmix1_iHystFallLow [7:0]	AFIT8_demsharpmix1_iHystFallHigh [15:8]  
		mt9t111_write(0x0F12 , 0x0306); 	//700009D2 //TVAR_afit_pBaseValS[263] /AFIT8_demsharpmix1_iHystTune [7:0]	* AFIT8_YUV422_DENOISE_iUVSupport [15:8]	 
		mt9t111_write(0x0F12 , 0x4602); 	//700009D4 //TVAR_afit_pBaseValS[264] /AFIT8_YUV422_DENOISE_iYSupport [7:0]    AFIT8_byr_cgras_iShadingPower [15:8] 	 
		mt9t111_write(0x0F12 , 0x0880); 	//700009D6 //TVAR_afit_pBaseValS[265] /AFIT8_RGBGamma2_iLinearity [7:0]   AFIT8_RGBGamma2_iDarkReduce [15:8]			 
		mt9t111_write(0x0F12 , 0x0080); 	//700009D8 //TVAR_afit_pBaseValS[266] /AFIT8_ccm_oscar_iSaturation[7:0]    AFIT8_RGB2YUV_iYOffset [15:8]				 
		mt9t111_write(0x0F12 , 0x0080); 	//700009DA //TVAR_afit_pBaseValS[267] /AFIT8_RGB2YUV_iRGBGain [7:0]    AFIT8_RGB2YUV_iSaturation [15:8] 				 
		mt9t111_write(0x0F12 , 0x0101); 	//700009DC //TVAR_afit_pBaseValS[268] /AFIT8_sddd8a_iClustThresh_H [7:0]   AFIT8_sddd8a_iClustThresh_C [15:8]			 
		mt9t111_write(0x0F12 , 0x0707); 	//700009DE //TVAR_afit_pBaseValS[269] /AFIT8_sddd8a_iClustMulT_H [7:0]	  AFIT8_sddd8a_iClustMulT_C [15:8]				 
		mt9t111_write(0x0F12 , 0x1E01); 	//700009E0 //TVAR_afit_pBaseValS[270] /AFIT8_sddd8a_nClustLevel_H [7:0]    AFIT8_sddd8a_DispTH_Low [15:8]				 
		mt9t111_write(0x0F12 , 0x3C1E); 	//700009E2 //TVAR_afit_pBaseValS[271] /AFIT8_sddd8a_DispTH_High [7:0]	 AFIT8_sddd8a_iDenThreshLow [15:8]				 
		mt9t111_write(0x0F12 , 0x5028); 	//700009E4 //TVAR_afit_pBaseValS[272] /AFIT8_sddd8a_iDenThreshHigh[7:0]    AFIT8_Demosaicing_iEdgeDesat [15:8]			 
		mt9t111_write(0x0F12 , 0x0500); 	//700009E6 //TVAR_afit_pBaseValS[273] /AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]    AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
		mt9t111_write(0x0F12 , 0x1A04); 	//700009E8 //TVAR_afit_pBaseValS[274] /AFIT8_Demosaicing_iEdgeDesatLimit[7:0]	AFIT8_Demosaicing_iDemSharpenLow [15:8] 	 
		mt9t111_write(0x0F12 , 0x280A); 	//700009EA //TVAR_afit_pBaseValS[275] /AFIT8_Demosaicing_iDemSharpenHigh[7:0]	 AFIT8_Demosaicing_iDemSharpThresh [15:8]	 
		mt9t111_write(0x0F12 , 0x080C); 	//700009EC //TVAR_afit_pBaseValS[276] /AFIT8_Demosaicing_iDemShLowLimit [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
		mt9t111_write(0x0F12 , 0x1414); 	//700009EE //TVAR_afit_pBaseValS[277] /AFIT8_Demosaicing_iDemBlurLow[7:0]	 AFIT8_Demosaicing_iDemBlurHigh [15:8]			   
		mt9t111_write(0x0F12 , 0x6A03); 	//700009F0 //TVAR_afit_pBaseValS[278] /AFIT8_Demosaicing_iDemBlurRange[7:0]    AFIT8_Sharpening_iLowSharpPower [15:8]		   
		mt9t111_write(0x0F12 , 0x121E); 	//700009F2 //TVAR_afit_pBaseValS[279] /AFIT8_Sharpening_iHighSharpPower[7:0]	AFIT8_Sharpening_iLowShDenoise [15:8]		   
		mt9t111_write(0x0F12 , 0x4012); 	//700009F4 //TVAR_afit_pBaseValS[280] /AFIT8_Sharpening_iHighShDenoise [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult [15:8]	   
		mt9t111_write(0x0F12 , 0x0604); 	//700009F6 //TVAR_afit_pBaseValS[281] /AFIT8_Sharpening_iReduceEdgeSlope [7:0]	 AFIT8_demsharpmix1_iWideFiltReduce [15:8]	   
		mt9t111_write(0x0F12 , 0x2806); 	//700009F8 //TVAR_afit_pBaseValS[282] /AFIT8_demsharpmix1_iNarrFiltReduce [7:0]   AFIT8_sddd8a_iClustThresh_H_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0128); 	//700009FA //TVAR_afit_pBaseValS[283] /AFIT8_sddd8a_iClustThresh_C_Bin [7:0]	AFIT8_sddd8a_iClustMulT_H_Bin [15:8]		   
		mt9t111_write(0x0F12 , 0x0101); 	//700009FC //TVAR_afit_pBaseValS[284] /AFIT8_sddd8a_iClustMulT_C_Bin [7:0]	  AFIT8_sddd8a_nClustLevel_H_Bin [15:8] 		   
		mt9t111_write(0x0F12 , 0x2224); 	//700009FE //TVAR_afit_pBaseValS[285] /AFIT8_sddd8a_DispTH_Low_Bin [7:0]	AFIT8_sddd8a_DispTH_High_Bin [15:8] 			   
		mt9t111_write(0x0F12 , 0x3236); 	//70000A00 //TVAR_afit_pBaseValS[286] /AFIT8_sddd8a_iDenThreshLow_Bin [7:0]    AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]		   
		mt9t111_write(0x0F12 , 0x0028); 	//70000A02 //TVAR_afit_pBaseValS[287] /AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]	AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x030A); 	//70000A04 //TVAR_afit_pBaseValS[288] /AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]   AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0410); 	//70000A06 //TVAR_afit_pBaseValS[289] /AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]	AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]  
		mt9t111_write(0x0F12 , 0x141E); 	//70000A08 //TVAR_afit_pBaseValS[290] /AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]	 AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]  
		mt9t111_write(0x0F12 , 0xFF07); 	//70000A0A //TVAR_afit_pBaseValS[291] /AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]   AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0432); 	//70000A0C //TVAR_afit_pBaseValS[292] /AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]   AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x4050); 	//70000A0E //TVAR_afit_pBaseValS[293] /AFIT8_Sharpening_iLowSharpPower_Bin [7:0]   AFIT8_Sharpening_iHighSharpPower_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x0F0F); 	//70000A10 //TVAR_afit_pBaseValS[294] /AFIT8_Sharpening_iLowShDenoise_Bin [7:0]   AFIT8_Sharpening_iHighShDenoise_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x0440); 	//70000A12 //TVAR_afit_pBaseValS[295] /AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]   AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0302); 	//70000A14 //TVAR_afit_pBaseValS[296] /AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]   AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
		mt9t111_write(0x0F12 , 0x2828); 	//70000A16 //TVAR_afit_pBaseValS[297] /AFIT8_sddd8a_iClustThresh_H_sBin[7:0]	AFIT8_sddd8a_iClustThresh_C_sBin [15:8] 		  
		mt9t111_write(0x0F12 , 0x0101); 	//70000A18 //TVAR_afit_pBaseValS[298] /AFIT8_sddd8a_iClustMulT_H_sBin [7:0]    AFIT8_sddd8a_iClustMulT_C_sBin [15:8]			  
		mt9t111_write(0x0F12 , 0x2401); 	//70000A1A //TVAR_afit_pBaseValS[299] /AFIT8_sddd8a_nClustLevel_H_sBin [7:0]	AFIT8_sddd8a_DispTH_Low_sBin [15:8] 			  
		mt9t111_write(0x0F12 , 0x3622); 	//70000A1C //TVAR_afit_pBaseValS[300] /AFIT8_sddd8a_DispTH_High_sBin [7:0]	  AFIT8_sddd8a_iDenThreshLow_sBin [15:8]			  
		mt9t111_write(0x0F12 , 0x2832); 	//70000A1E //TVAR_afit_pBaseValS[301] /AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]	AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]		  
		mt9t111_write(0x0F12 , 0x0A00); 	//70000A20 //TVAR_afit_pBaseValS[302] /AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
		mt9t111_write(0x0F12 , 0x1003); 	//70000A22 //TVAR_afit_pBaseValS[303] /AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]   AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]	  
		mt9t111_write(0x0F12 , 0x1E04); 	//70000A24 //TVAR_afit_pBaseValS[304] /AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]   AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]   
		mt9t111_write(0x0F12 , 0x0714); 	//70000A26 //TVAR_afit_pBaseValS[305] /AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
		mt9t111_write(0x0F12 , 0x32FF); 	//70000A28 //TVAR_afit_pBaseValS[306] /AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]   AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]			
		mt9t111_write(0x0F12 , 0x5004); 	//70000A2A //TVAR_afit_pBaseValS[307] /AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]	AFIT8_Sharpening_iLowSharpPower_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x0F40); 	//70000A2C //TVAR_afit_pBaseValS[308] /AFIT8_Sharpening_iHighSharpPower_sBin [7:0]	 AFIT8_Sharpening_iLowShDenoise_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x400F); 	//70000A2E //TVAR_afit_pBaseValS[309] /AFIT8_Sharpening_iHighShDenoise_sBin [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8] 	
		mt9t111_write(0x0F12 , 0x0204); 	//70000A30 //TVAR_afit_pBaseValS[310] /AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]   AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]	
		mt9t111_write(0x0F12 , 0x0003); 	//70000A32 //TVAR_afit_pBaseValS[311] /AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
		mt9t111_write(0x0F12 , 0x0000); 	//70000A34 //TVAR_afit_pBaseValS[312] /AFIT16_BRIGHTNESS 
		mt9t111_write(0x0F12 , 0x0000); 	//70000A36 //TVAR_afit_pBaseValS[313] /AFIT16_CONTRAST 
		mt9t111_write(0x0F12 , 0x0020); 	//70000A38 //TVAR_afit_pBaseValS[314] /AFIT16_SATURATION	  
		mt9t111_write(0x0F12 , 0x0000); 	//70000A3A //TVAR_afit_pBaseValS[315] /AFIT16_SHARP_BLUR 
		mt9t111_write(0x0F12 , 0x0000); 	//70000A3C //TVAR_afit_pBaseValS[316] /AFIT16_GLAMOUR	 
		mt9t111_write(0x0F12 , 0x00C1); 	//70000A3E //TVAR_afit_pBaseValS[317] /AFIT16_sddd8a_edge_high			  
		mt9t111_write(0x0F12 , 0x03FF); 	//70000A40 //TVAR_afit_pBaseValS[318] /AFIT16_Demosaicing_iSatVal		  
		mt9t111_write(0x0F12 , 0x009C); 	//70000A42 //TVAR_afit_pBaseValS[319] /AFIT16_Sharpening_iReduceEdgeThresh
		mt9t111_write(0x0F12 , 0x0251); 	//70000A44 //TVAR_afit_pBaseValS[320] /AFIT16_demsharpmix1_iRGBOffset	  
		mt9t111_write(0x0F12 , 0x03FF); 	//70000A46 //TVAR_afit_pBaseValS[321] /AFIT16_demsharpmix1_iDemClamp	  
		mt9t111_write(0x0F12 , 0x000C); 	//70000A48 //TVAR_afit_pBaseValS[322] /AFIT16_demsharpmix1_iLowThreshold  
		mt9t111_write(0x0F12 , 0x0010); 	//70000A4A //TVAR_afit_pBaseValS[323] /AFIT16_demsharpmix1_iHighThreshold 
		mt9t111_write(0x0F12 , 0x00C8); 	//70000A4C //TVAR_afit_pBaseValS[324] /AFIT16_demsharpmix1_iLowBright	  
		mt9t111_write(0x0F12 , 0x03E8); 	//70000A4E //TVAR_afit_pBaseValS[325] /AFIT16_demsharpmix1_iHighBright	  
		mt9t111_write(0x0F12 , 0x0046); 	//70000A50 //TVAR_afit_pBaseValS[326] /AFIT16_demsharpmix1_iLowSat		  
		mt9t111_write(0x0F12 , 0x0050); 	//70000A52 //TVAR_afit_pBaseValS[327] /AFIT16_demsharpmix1_iHighSat 	  
		mt9t111_write(0x0F12 , 0x0070); 	//70000A54 //TVAR_afit_pBaseValS[328] /AFIT16_demsharpmix1_iTune		  
		mt9t111_write(0x0F12 , 0x0000); 	//70000A56 //TVAR_afit_pBaseValS[329] /AFIT16_demsharpmix1_iHystThLow	  
		mt9t111_write(0x0F12 , 0x0000); 	//70000A58 //TVAR_afit_pBaseValS[330] /AFIT16_demsharpmix1_iHystThHigh	  
		mt9t111_write(0x0F12 , 0x0226); 	//70000A5A //TVAR_afit_pBaseValS[331] /AFIT16_demsharpmix1_iHystCenter	  
		mt9t111_write(0x0F12 , 0x0014); 	//70000A5C //TVAR_afit_pBaseValS[332] /AFIT16_YUV422_DENOISE_iUVLowThresh 
		mt9t111_write(0x0F12 , 0x0014); 	//70000A5E //TVAR_afit_pBaseValS[333] /AFIT16_YUV422_DENOISE_iUVHighThresh
		mt9t111_write(0x0F12 , 0x0000); 	//70000A60 //TVAR_afit_pBaseValS[334] /AFIT16_YUV422_DENOISE_iYLowThresh  
		mt9t111_write(0x0F12 , 0x0000); 	//70000A62 //TVAR_afit_pBaseValS[335] /AFIT16_YUV422_DENOISE_iYHighThresh 
		mt9t111_write(0x0F12 , 0x004E); 	//70000A64 //TVAR_afit_pBaseValS[336] /AFIT16_Sharpening_iLowSharpClamp   
		mt9t111_write(0x0F12 , 0x0000); 	//70000A66 //TVAR_afit_pBaseValS[337] /AFIT16_Sharpening_iHighSharpClamp  
		mt9t111_write(0x0F12 , 0x002D); 	//70000A68 //TVAR_afit_pBaseValS[338] /AFIT16_Sharpening_iLowSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x0019); 	//70000A6A //TVAR_afit_pBaseValS[339] /AFIT16_Sharpening_iHighSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x002D); 	//70000A6C //TVAR_afit_pBaseValS[340] /AFIT16_Sharpening_iLowSharpClamp_sBin
		mt9t111_write(0x0F12 , 0x0019); 	//70000A6E //TVAR_afit_pBaseValS[341] /AFIT16_Sharpening_iHighSharpClamp_sBin
		mt9t111_write(0x0F12 , 0x0A24); 	//70000A70 //TVAR_afit_pBaseValS[342] /AFIT8_sddd8a_edge_low [7:0]	  AFIT8_sddd8a_repl_thresh [15:8]
		mt9t111_write(0x0F12 , 0x1701); 	//70000A72 //TVAR_afit_pBaseValS[343] /AFIT8_sddd8a_repl_force [7:0]   AFIT8_sddd8a_sat_level [15:8] 
		mt9t111_write(0x0F12 , 0x0229); 	//70000A74 //TVAR_afit_pBaseValS[344] /AFIT8_sddd8a_sat_thr[7:0]   AFIT8_sddd8a_sat_mpl [15:8]		 
		mt9t111_write(0x0F12 , 0x1403); 	//70000A76 //TVAR_afit_pBaseValS[345] /AFIT8_sddd8a_sat_noise[7:0]	 AFIT8_sddd8a_iMaxSlopeAllowed [15:8]	 
		mt9t111_write(0x0F12 , 0x0000); 	//70000A78 //TVAR_afit_pBaseValS[346] /AFIT8_sddd8a_iHotThreshHigh[7:0]   AFIT8_sddd8a_iHotThreshLow [15:8]  
		mt9t111_write(0x0F12 , 0x0000); 	//70000A7A //TVAR_afit_pBaseValS[347] /AFIT8_sddd8a_iColdThreshHigh[7:0]   AFIT8_sddd8a_iColdThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0906); 	//70000A7C //TVAR_afit_pBaseValS[348] /AFIT8_sddd8a_AddNoisePower1[7:0]   AFIT8_sddd8a_AddNoisePower2 [15:8] 
		mt9t111_write(0x0F12 , 0x00FF); 	//70000A7E //TVAR_afit_pBaseValS[349] /AFIT8_sddd8a_iSatSat[7:0]	AFIT8_sddd8a_iRadialTune [15:8] 		 
		mt9t111_write(0x0F12 , 0x045A); 	//70000A80 //TVAR_afit_pBaseValS[350] /AFIT8_sddd8a_iRadialLimit [7:0]	  AFIT8_sddd8a_iRadialPower [15:8]	 
		mt9t111_write(0x0F12 , 0x1414); 	//70000A82 //TVAR_afit_pBaseValS[351] /AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0]	AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
		mt9t111_write(0x0F12 , 0x0301); 	//70000A84 //TVAR_afit_pBaseValS[352] /AFIT8_sddd8a_iLowSlopeThresh[7:0]	AFIT8_sddd8a_iHighSlopeThresh [15:8]		
		mt9t111_write(0x0F12 , 0xFF07); 	//70000A86 //TVAR_afit_pBaseValS[353] /AFIT8_sddd8a_iSquaresRounding [7:0]	  AFIT8_Demosaicing_iCentGrad [15:8]		
		mt9t111_write(0x0F12 , 0x081E); 	//70000A88 //TVAR_afit_pBaseValS[354] /AFIT8_Demosaicing_iMonochrom [7:0]	 AFIT8_Demosaicing_iDecisionThresh [15:8]	
		mt9t111_write(0x0F12 , 0x0A14); 	//70000A8A //TVAR_afit_pBaseValS[355] /AFIT8_Demosaicing_iDesatThresh [7:0]    AFIT8_Demosaicing_iEnhThresh [15:8]		
		mt9t111_write(0x0F12 , 0x0F0F); 	//70000A8C //TVAR_afit_pBaseValS[356] /AFIT8_Demosaicing_iGRDenoiseVal [7:0]	AFIT8_Demosaicing_iGBDenoiseVal [15:8]	
		mt9t111_write(0x0F12 , 0x0A01);    //0A05 //0A05	//70000A8E //TVAR_afit_pBaseValS[357] /AFIT8_Demosaicing_iNearGrayDesat[7:0]	AFIT8_Demosaicing_iDFD_ReduceCoeff [15:8]
		mt9t111_write(0x0F12 , 0x0090); 	//70000A90 //TVAR_afit_pBaseValS[358] /AFIT8_Sharpening_iMSharpen [7:0]    AFIT8_Sharpening_iMShThresh [15:8] 
		mt9t111_write(0x0F12 , 0x000A); 	//70000A92 //TVAR_afit_pBaseValS[359] /AFIT8_Sharpening_iWSharpen [7:0]    AFIT8_Sharpening_iWShThresh [15:8] 
		mt9t111_write(0x0F12 , 0x0001); 	//70000A94 //TVAR_afit_pBaseValS[360] /AFIT8_Sharpening_nSharpWidth [7:0]	 AFIT8_Sharpening_iReduceNegative [15:8] 
		mt9t111_write(0x0F12 , 0x00FF); 	//70000A96 //TVAR_afit_pBaseValS[361] /AFIT8_Sharpening_iShDespeckle [7:0]	 AFIT8_demsharpmix1_iRGBMultiplier [15:8]
		mt9t111_write(0x0F12 , 0x1002); 	//70000A98 //TVAR_afit_pBaseValS[362] /AFIT8_demsharpmix1_iFilterPower [7:0]   AFIT8_demsharpmix1_iBCoeff [15:8]	 
		mt9t111_write(0x0F12 , 0x001E); 	//70000A9A //TVAR_afit_pBaseValS[363] /AFIT8_demsharpmix1_iGCoeff [7:0]    AFIT8_demsharpmix1_iWideMult [15:8]		 
		mt9t111_write(0x0F12 , 0x0900); 	//70000A9C //TVAR_afit_pBaseValS[364] /AFIT8_demsharpmix1_iNarrMult [7:0]	 AFIT8_demsharpmix1_iHystFalloff [15:8]  
		mt9t111_write(0x0F12 , 0x0600); 	//70000A9E //TVAR_afit_pBaseValS[365] /AFIT8_demsharpmix1_iHystMinMult [7:0]	AFIT8_demsharpmix1_iHystWidth [15:8] 
		mt9t111_write(0x0F12 , 0x0504); 	//70000AA0 //TVAR_afit_pBaseValS[366] /AFIT8_demsharpmix1_iHystFallLow [7:0]	AFIT8_demsharpmix1_iHystFallHigh [15:8]
		mt9t111_write(0x0F12 , 0x0307); 	//70000AA2 //TVAR_afit_pBaseValS[367] /AFIT8_demsharpmix1_iHystTune [7:0]	* AFIT8_YUV422_DENOISE_iUVSupport [15:8] 
		mt9t111_write(0x0F12 , 0x5002); 	//70000AA4 //TVAR_afit_pBaseValS[368] /AFIT8_YUV422_DENOISE_iYSupport [7:0]    AFIT8_byr_cgras_iShadingPower [15:8]  
		mt9t111_write(0x0F12 , 0x0080); 	//70000AA6 //TVAR_afit_pBaseValS[369] /AFIT8_RGBGamma2_iLinearity [7:0]   AFIT8_RGBGamma2_iDarkReduce [15:8]		 
		mt9t111_write(0x0F12 , 0x0080); 	//70000AA8 //TVAR_afit_pBaseValS[370] /AFIT8_ccm_oscar_iSaturation[7:0]    AFIT8_RGB2YUV_iYOffset [15:8]			 
		mt9t111_write(0x0F12 , 0x0080); 	//70000AAA //TVAR_afit_pBaseValS[371] /AFIT8_RGB2YUV_iRGBGain [7:0]    AFIT8_RGB2YUV_iSaturation [15:8] 			 
		mt9t111_write(0x0F12 , 0x0101); 	//70000AAC //TVAR_afit_pBaseValS[372] /AFIT8_sddd8a_iClustThresh_H [7:0]   AFIT8_sddd8a_iClustThresh_C [15:8]		 
		mt9t111_write(0x0F12 , 0x0707); 	//70000AAE //TVAR_afit_pBaseValS[373] /AFIT8_sddd8a_iClustMulT_H [7:0]	  AFIT8_sddd8a_iClustMulT_C [15:8]			 
		mt9t111_write(0x0F12 , 0x1E01); 	//70000AB0 //TVAR_afit_pBaseValS[374] /AFIT8_sddd8a_nClustLevel_H [7:0]    AFIT8_sddd8a_DispTH_Low [15:8]			 
		mt9t111_write(0x0F12 , 0x2A1E); 	//70000AB2 //TVAR_afit_pBaseValS[375] /AFIT8_sddd8a_DispTH_High [7:0]	 AFIT8_sddd8a_iDenThreshLow [15:8]			 
		mt9t111_write(0x0F12 , 0x5020); 	//70000AB4 //TVAR_afit_pBaseValS[376] /AFIT8_sddd8a_iDenThreshHigh[7:0]    AFIT8_Demosaicing_iEdgeDesat [15:8]		 
		mt9t111_write(0x0F12 , 0x0500); 	//70000AB6 //TVAR_afit_pBaseValS[377] /AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]    AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8] 
		mt9t111_write(0x0F12 , 0x1A04); 	//70000AB8 //TVAR_afit_pBaseValS[378] /AFIT8_Demosaicing_iEdgeDesatLimit[7:0]	AFIT8_Demosaicing_iDemSharpenLow [15:8] 	  
		mt9t111_write(0x0F12 , 0x280A); 	//70000ABA //TVAR_afit_pBaseValS[379] /AFIT8_Demosaicing_iDemSharpenHigh[7:0]	 AFIT8_Demosaicing_iDemSharpThresh [15:8]	  
		mt9t111_write(0x0F12 , 0x080C); 	//70000ABC //TVAR_afit_pBaseValS[380] /AFIT8_Demosaicing_iDemShLowLimit [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
		mt9t111_write(0x0F12 , 0x1414); 	//70000ABE //TVAR_afit_pBaseValS[381] /AFIT8_Demosaicing_iDemBlurLow[7:0]	 AFIT8_Demosaicing_iDemBlurHigh [15:8]			  
		mt9t111_write(0x0F12 , 0x6A03); 	//70000AC0 //TVAR_afit_pBaseValS[382] /AFIT8_Demosaicing_iDemBlurRange[7:0]    AFIT8_Sharpening_iLowSharpPower [15:8]		  
		mt9t111_write(0x0F12 , 0x121E); 	//70000AC2 //TVAR_afit_pBaseValS[383] /AFIT8_Sharpening_iHighSharpPower[7:0]	AFIT8_Sharpening_iLowShDenoise [15:8]		  
		mt9t111_write(0x0F12 , 0x4012); 	//70000AC4 //TVAR_afit_pBaseValS[384] /AFIT8_Sharpening_iHighShDenoise [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult [15:8]	  
		mt9t111_write(0x0F12 , 0x0604); 	//70000AC6 //TVAR_afit_pBaseValS[385] /AFIT8_Sharpening_iReduceEdgeSlope [7:0]	 AFIT8_demsharpmix1_iWideFiltReduce [15:8]	  
		mt9t111_write(0x0F12 , 0x3C06); 	//70000AC8 //TVAR_afit_pBaseValS[386] /AFIT8_demsharpmix1_iNarrFiltReduce [7:0]   AFIT8_sddd8a_iClustThresh_H_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x013C); 	//70000ACA //TVAR_afit_pBaseValS[387] /AFIT8_sddd8a_iClustThresh_C_Bin [7:0]	AFIT8_sddd8a_iClustMulT_H_Bin [15:8]		  
		mt9t111_write(0x0F12 , 0x0101); 	//70000ACC //TVAR_afit_pBaseValS[388] /AFIT8_sddd8a_iClustMulT_C_Bin [7:0]	  AFIT8_sddd8a_nClustLevel_H_Bin [15:8] 		  
		mt9t111_write(0x0F12 , 0x1C1E); 	//70000ACE //TVAR_afit_pBaseValS[389] /AFIT8_sddd8a_DispTH_Low_Bin [7:0]	AFIT8_sddd8a_DispTH_High_Bin [15:8] 			  
		mt9t111_write(0x0F12 , 0x1E22); 	//70000AD0 //TVAR_afit_pBaseValS[390] /AFIT8_sddd8a_iDenThreshLow_Bin [7:0]    AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]		  
		mt9t111_write(0x0F12 , 0x0028); 	//70000AD2 //TVAR_afit_pBaseValS[391] /AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]	AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8] 
		mt9t111_write(0x0F12 , 0x030A); 	//70000AD4 //TVAR_afit_pBaseValS[392] /AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]   AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0214); 	//70000AD6 //TVAR_afit_pBaseValS[393] /AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]	AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]   
		mt9t111_write(0x0F12 , 0x0E14); 	//70000AD8 //TVAR_afit_pBaseValS[394] /AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]	 AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]   
		mt9t111_write(0x0F12 , 0xFF06); 	//70000ADA //TVAR_afit_pBaseValS[395] /AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]   AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0432); 	//70000ADC //TVAR_afit_pBaseValS[396] /AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]   AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x4052); 	//70000ADE //TVAR_afit_pBaseValS[397] /AFIT8_Sharpening_iLowSharpPower_Bin [7:0]   AFIT8_Sharpening_iHighSharpPower_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x150C); 	//70000AE0 //TVAR_afit_pBaseValS[398] /AFIT8_Sharpening_iLowShDenoise_Bin [7:0]   AFIT8_Sharpening_iHighShDenoise_Bin [15:8]	   
		mt9t111_write(0x0F12 , 0x0440); 	//70000AE2 //TVAR_afit_pBaseValS[399] /AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]   AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
		mt9t111_write(0x0F12 , 0x0302); 	//70000AE4 //TVAR_afit_pBaseValS[400] /AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]   AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
		mt9t111_write(0x0F12 , 0x3C3C); 	//70000AE6 //TVAR_afit_pBaseValS[401] /AFIT8_sddd8a_iClustThresh_H_sBin[7:0]	AFIT8_sddd8a_iClustThresh_C_sBin [15:8] 		   
		mt9t111_write(0x0F12 , 0x0101); 	//70000AE8 //TVAR_afit_pBaseValS[402] /AFIT8_sddd8a_iClustMulT_H_sBin [7:0]    AFIT8_sddd8a_iClustMulT_C_sBin [15:8]			   
		mt9t111_write(0x0F12 , 0x1E01); 	//70000AEA //TVAR_afit_pBaseValS[403] /AFIT8_sddd8a_nClustLevel_H_sBin [7:0]	AFIT8_sddd8a_DispTH_Low_sBin [15:8] 			   
		mt9t111_write(0x0F12 , 0x221C); 	//70000AEC //TVAR_afit_pBaseValS[404] /AFIT8_sddd8a_DispTH_High_sBin [7:0]	  AFIT8_sddd8a_iDenThreshLow_sBin [15:8]			   
		mt9t111_write(0x0F12 , 0x281E); 	//70000AEE //TVAR_afit_pBaseValS[405] /AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]	AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]		   
		mt9t111_write(0x0F12 , 0x0A00); 	//70000AF0 //TVAR_afit_pBaseValS[406] /AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8] 
		mt9t111_write(0x0F12 , 0x1403); 	//70000AF2 //TVAR_afit_pBaseValS[407] /AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]   AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]	   
		mt9t111_write(0x0F12 , 0x1402); 	//70000AF4 //TVAR_afit_pBaseValS[408] /AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]   AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]    
		mt9t111_write(0x0F12 , 0x060E); 	//70000AF6 //TVAR_afit_pBaseValS[409] /AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
		mt9t111_write(0x0F12 , 0x32FF); 	//70000AF8 //TVAR_afit_pBaseValS[410] /AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]   AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]		   
		mt9t111_write(0x0F12 , 0x5204); 	//70000AFA //TVAR_afit_pBaseValS[411] /AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]	AFIT8_Sharpening_iLowSharpPower_sBin [15:8] 	   
		mt9t111_write(0x0F12 , 0x0C40); 	//70000AFC //TVAR_afit_pBaseValS[412] /AFIT8_Sharpening_iHighSharpPower_sBin [7:0]	 AFIT8_Sharpening_iLowShDenoise_sBin [15:8] 	   
		mt9t111_write(0x0F12 , 0x4015); 	//70000AFE //TVAR_afit_pBaseValS[413] /AFIT8_Sharpening_iHighShDenoise_sBin [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8]    
		mt9t111_write(0x0F12 , 0x0204); 	//70000B00 //TVAR_afit_pBaseValS[414] /AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]   AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]   
		mt9t111_write(0x0F12 , 0x0003); 	//70000B02 //TVAR_afit_pBaseValS[415] /AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
		mt9t111_write(0x0F12 , 0x0000); 	//70000B04 //TVAR_afit_pBaseValS[416] /AFIT16_BRIGHTNESS 
		mt9t111_write(0x0F12 , 0x0000); 	//70000B06 //TVAR_afit_pBaseValS[417] /AFIT16_CONTRAST	 
		mt9t111_write(0x0F12 , 0x0020); 	//70000A38 //TVAR_afit_pBaseValS[314] /AFIT16_SATURATION	 
		mt9t111_write(0x0F12 , 0x0000); 	//70000B0A //TVAR_afit_pBaseValS[419] /AFIT16_SHARP_BLUR 
		mt9t111_write(0x0F12 , 0x0000); 	//70000B0C //TVAR_afit_pBaseValS[420] /AFIT16_GLAMOUR	 
		mt9t111_write(0x0F12 , 0x00C1); 	//70000B0E //TVAR_afit_pBaseValS[421] /AFIT16_sddd8a_edge_high			   
		mt9t111_write(0x0F12 , 0x03FF); 	//70000B10 //TVAR_afit_pBaseValS[422] /AFIT16_Demosaicing_iSatVal		   
		mt9t111_write(0x0F12 , 0x009C); 	//70000B12 //TVAR_afit_pBaseValS[423] /AFIT16_Sharpening_iReduceEdgeThresh 
		mt9t111_write(0x0F12 , 0x0251); 	//70000B14 //TVAR_afit_pBaseValS[424] /AFIT16_demsharpmix1_iRGBOffset	   
		mt9t111_write(0x0F12 , 0x03FF); 	//70000B16 //TVAR_afit_pBaseValS[425] /AFIT16_demsharpmix1_iDemClamp	   
		mt9t111_write(0x0F12 , 0x000C); 	//70000B18 //TVAR_afit_pBaseValS[426] /AFIT16_demsharpmix1_iLowThreshold   
		mt9t111_write(0x0F12 , 0x0010); 	//70000B1A //TVAR_afit_pBaseValS[427] /AFIT16_demsharpmix1_iHighThreshold  
		mt9t111_write(0x0F12 , 0x0032); 	//70000B1C //TVAR_afit_pBaseValS[428] /AFIT16_demsharpmix1_iLowBright	   
		mt9t111_write(0x0F12 , 0x028A); 	//70000B1E //TVAR_afit_pBaseValS[429] /AFIT16_demsharpmix1_iHighBright	   
		mt9t111_write(0x0F12 , 0x0032); 	//70000B20 //TVAR_afit_pBaseValS[430] /AFIT16_demsharpmix1_iLowSat		   
		mt9t111_write(0x0F12 , 0x01F4); 	//70000B22 //TVAR_afit_pBaseValS[431] /AFIT16_demsharpmix1_iHighSat 	   
		mt9t111_write(0x0F12 , 0x0070); 	//70000B24 //TVAR_afit_pBaseValS[432] /AFIT16_demsharpmix1_iTune		   
		mt9t111_write(0x0F12 , 0x0000); 	//70000B26 //TVAR_afit_pBaseValS[433] /AFIT16_demsharpmix1_iHystThLow	   
		mt9t111_write(0x0F12 , 0x0000); 	//70000B28 //TVAR_afit_pBaseValS[434] /AFIT16_demsharpmix1_iHystThHigh	   
		mt9t111_write(0x0F12 , 0x01AA); 	//70000B2A //TVAR_afit_pBaseValS[435] /AFIT16_demsharpmix1_iHystCenter	   
		mt9t111_write(0x0F12 , 0x003C); 	//70000B2C //TVAR_afit_pBaseValS[436] /AFIT16_YUV422_DENOISE_iUVLowThresh  
		mt9t111_write(0x0F12 , 0x0050); 	//70000B2E //TVAR_afit_pBaseValS[437] /AFIT16_YUV422_DENOISE_iUVHighThresh 
		mt9t111_write(0x0F12 , 0x0000); 	//70000B30 //TVAR_afit_pBaseValS[438] /AFIT16_YUV422_DENOISE_iYLowThresh   
		mt9t111_write(0x0F12 , 0x0000); 	//70000B32 //TVAR_afit_pBaseValS[439] /AFIT16_YUV422_DENOISE_iYHighThresh  
		mt9t111_write(0x0F12 , 0x0044); 	//70000B34 //TVAR_afit_pBaseValS[440] /AFIT16_Sharpening_iLowSharpClamp    
		mt9t111_write(0x0F12 , 0x0014); 	//70000B36 //TVAR_afit_pBaseValS[441] /AFIT16_Sharpening_iHighSharpClamp   
		mt9t111_write(0x0F12 , 0x0046); 	//70000B38 //TVAR_afit_pBaseValS[442] /AFIT16_Sharpening_iLowSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x0019); 	//70000B3A //TVAR_afit_pBaseValS[443] /AFIT16_Sharpening_iHighSharpClamp_Bin
		mt9t111_write(0x0F12 , 0x0046); 	//70000B3C //TVAR_afit_pBaseValS[444] /AFIT16_Sharpening_iLowSharpClamp_sBin
		mt9t111_write(0x0F12 , 0x0019); 	//70000B3E //TVAR_afit_pBaseValS[445] /AFIT16_Sharpening_iHighSharpClamp_sBin
		mt9t111_write(0x0F12 , 0x0A24); 	//70000B40 //TVAR_afit_pBaseValS[446] /AFIT8_sddd8a_edge_low [7:0]	  AFIT8_sddd8a_repl_thresh [15:8]	   
		mt9t111_write(0x0F12 , 0x1701); 	//70000B42 //TVAR_afit_pBaseValS[447] /AFIT8_sddd8a_repl_force [7:0]   AFIT8_sddd8a_sat_level [15:8]	   
		mt9t111_write(0x0F12 , 0x0229); 	//70000B44 //TVAR_afit_pBaseValS[448] /AFIT8_sddd8a_sat_thr[7:0]   AFIT8_sddd8a_sat_mpl [15:8]			   
		mt9t111_write(0x0F12 , 0x0503); 	//70000B46 //TVAR_afit_pBaseValS[449] /AFIT8_sddd8a_sat_noise[7:0]	 AFIT8_sddd8a_iMaxSlopeAllowed [15:8]  
		mt9t111_write(0x0F12 , 0x080F); 	//70000B48 //TVAR_afit_pBaseValS[450] /AFIT8_sddd8a_iHotThreshHigh[7:0]   AFIT8_sddd8a_iHotThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0808); 	//70000B4A //TVAR_afit_pBaseValS[451] /AFIT8_sddd8a_iColdThreshHigh[7:0]   AFIT8_sddd8a_iColdThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x0000); 	//70000B4C //TVAR_afit_pBaseValS[452] /AFIT8_sddd8a_AddNoisePower1[7:0]   AFIT8_sddd8a_AddNoisePower2 [15:8]
		mt9t111_write(0x0F12 , 0x00FF); 	//70000B4E //TVAR_afit_pBaseValS[453] /AFIT8_sddd8a_iSatSat[7:0]	AFIT8_sddd8a_iRadialTune [15:8] 	   
		mt9t111_write(0x0F12 , 0x012D); 	//70000B50 //TVAR_afit_pBaseValS[454] /AFIT8_sddd8a_iRadialLimit [7:0]	  AFIT8_sddd8a_iRadialPower [15:8] 
		mt9t111_write(0x0F12 , 0x1414); 	//70000B52 //TVAR_afit_pBaseValS[455] /AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0]	AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
		mt9t111_write(0x0F12 , 0x0301); 	//70000B54 //TVAR_afit_pBaseValS[456] /AFIT8_sddd8a_iLowSlopeThresh[7:0]	AFIT8_sddd8a_iHighSlopeThresh [15:8]
		mt9t111_write(0x0F12 , 0xFF07); 	//70000B56 //TVAR_afit_pBaseValS[457] /AFIT8_sddd8a_iSquaresRounding [7:0]	  AFIT8_Demosaicing_iCentGrad [15:8]
		mt9t111_write(0x0F12 , 0x061E); 	//70000B58 //TVAR_afit_pBaseValS[458] /AFIT8_Demosaicing_iMonochrom [7:0]	 AFIT8_Demosaicing_iDecisionThresh [15:8]
		mt9t111_write(0x0F12 , 0x0A1E); 	//70000B5A //TVAR_afit_pBaseValS[459] /AFIT8_Demosaicing_iDesatThresh [7:0]    AFIT8_Demosaicing_iEnhThresh [15:8]	 
		mt9t111_write(0x0F12 , 0x0606); 	//70000B5C //TVAR_afit_pBaseValS[460] /AFIT8_Demosaicing_iGRDenoiseVal [7:0]	AFIT8_Demosaicing_iGBDenoiseVal [15:8]
		mt9t111_write(0x0F12 , 0x0A03);    //0A05 //0A05	//70000B5E //TVAR_afit_pBaseValS[461] /AFIT8_Demosaicing_iNearGrayDesat[7:0]	AFIT8_Demosaicing_iDFD_ReduceCoeff [15:8] 
		mt9t111_write(0x0F12 , 0x378B); 	//70000B60 //TVAR_afit_pBaseValS[462] /AFIT8_Sharpening_iMSharpen [7:0]    AFIT8_Sharpening_iMShThresh [15:8]			  
		mt9t111_write(0x0F12 , 0x1028); 	//70000B62 //TVAR_afit_pBaseValS[463] /AFIT8_Sharpening_iWSharpen [7:0]    AFIT8_Sharpening_iWShThresh [15:8]			  
		mt9t111_write(0x0F12 , 0x0001); 	//70000B64 //TVAR_afit_pBaseValS[464] /AFIT8_Sharpening_nSharpWidth [7:0]	 AFIT8_Sharpening_iReduceNegative [15:8]	  
		mt9t111_write(0x0F12 , 0x00FF); 	//70000B66 //TVAR_afit_pBaseValS[465] /AFIT8_Sharpening_iShDespeckle [7:0]	 AFIT8_demsharpmix1_iRGBMultiplier [15:8]	  
		mt9t111_write(0x0F12 , 0x1002); 	//70000B68 //TVAR_afit_pBaseValS[466] /AFIT8_demsharpmix1_iFilterPower [7:0]   AFIT8_demsharpmix1_iBCoeff [15:8]  
		mt9t111_write(0x0F12 , 0x001E); 	//70000B6A //TVAR_afit_pBaseValS[467] /AFIT8_demsharpmix1_iGCoeff [7:0]    AFIT8_demsharpmix1_iWideMult [15:8]	  
		mt9t111_write(0x0F12 , 0x0900); 	//70000B6C //TVAR_afit_pBaseValS[468] /AFIT8_demsharpmix1_iNarrMult [7:0]	 AFIT8_demsharpmix1_iHystFalloff [15:8] 
		mt9t111_write(0x0F12 , 0x0600); 	//70000B6E //TVAR_afit_pBaseValS[469] /AFIT8_demsharpmix1_iHystMinMult [7:0]	AFIT8_demsharpmix1_iHystWidth [15:8]
		mt9t111_write(0x0F12 , 0x0504); 	//70000B70 //TVAR_afit_pBaseValS[470] /AFIT8_demsharpmix1_iHystFallLow [7:0]	AFIT8_demsharpmix1_iHystFallHigh [15:8]
		mt9t111_write(0x0F12 , 0x0307); 	//70000B72 //TVAR_afit_pBaseValS[471] /AFIT8_demsharpmix1_iHystTune [7:0]	* AFIT8_YUV422_DENOISE_iUVSupport [15:8]   
		mt9t111_write(0x0F12 , 0x5001); 	//70000B74 //TVAR_afit_pBaseValS[472] /AFIT8_YUV422_DENOISE_iYSupport [7:0]    AFIT8_byr_cgras_iShadingPower [15:8]    
		mt9t111_write(0x0F12 , 0x0080); 	//70000B76 //TVAR_afit_pBaseValS[473] /AFIT8_RGBGamma2_iLinearity [7:0]   AFIT8_RGBGamma2_iDarkReduce [15:8]		   
		mt9t111_write(0x0F12 , 0x0080); 	//70000B78 //TVAR_afit_pBaseValS[474] /AFIT8_ccm_oscar_iSaturation[7:0]    AFIT8_RGB2YUV_iYOffset [15:8]			   
		mt9t111_write(0x0F12 , 0x0080); 	//70000B7A //TVAR_afit_pBaseValS[475] /AFIT8_RGB2YUV_iRGBGain [7:0]    AFIT8_RGB2YUV_iSaturation [15:8] 			   
		mt9t111_write(0x0F12 , 0x5050); 	//70000B7C //TVAR_afit_pBaseValS[476] /AFIT8_sddd8a_iClustThresh_H [7:0]   AFIT8_sddd8a_iClustThresh_C [15:8]		   
		mt9t111_write(0x0F12 , 0x0101); 	//70000B7E //TVAR_afit_pBaseValS[477] /AFIT8_sddd8a_iClustMulT_H [7:0]	  AFIT8_sddd8a_iClustMulT_C [15:8]			   
		mt9t111_write(0x0F12 , 0x3201); 	//70000B80 //TVAR_afit_pBaseValS[478] /AFIT8_sddd8a_nClustLevel_H [7:0]    AFIT8_sddd8a_DispTH_Low [15:8]			   
		mt9t111_write(0x0F12 , 0x1832); 	//70000B82 //TVAR_afit_pBaseValS[479] /AFIT8_sddd8a_DispTH_High [7:0]	 AFIT8_sddd8a_iDenThreshLow [15:8]
		mt9t111_write(0x0F12 , 0x210C); 	//70000B84 //TVAR_afit_pBaseValS[480] /AFIT8_sddd8a_iDenThreshHigh[7:0]    AFIT8_Demosaicing_iEdgeDesat [15:8]
		mt9t111_write(0x0F12 , 0x0A00); 	//70000B86 //TVAR_afit_pBaseValS[481] /AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]    AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
		mt9t111_write(0x0F12 , 0x1E04); 	//70000B88 //TVAR_afit_pBaseValS[482] /AFIT8_Demosaicing_iEdgeDesatLimit[7:0]	AFIT8_Demosaicing_iDemSharpenLow [15:8]
		mt9t111_write(0x0F12 , 0x0A08); 	//70000B8A //TVAR_afit_pBaseValS[483] /AFIT8_Demosaicing_iDemSharpenHigh[7:0]	 AFIT8_Demosaicing_iDemSharpThresh [15:8]
		mt9t111_write(0x0F12 , 0x070C); 	//70000B8C //TVAR_afit_pBaseValS[484] /AFIT8_Demosaicing_iDemShLowLimit [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
		mt9t111_write(0x0F12 , 0x3264); 	//70000B8E //TVAR_afit_pBaseValS[485] /AFIT8_Demosaicing_iDemBlurLow[7:0]	 AFIT8_Demosaicing_iDemBlurHigh [15:8]		  
		mt9t111_write(0x0F12 , 0x5A02); 	//70000B90 //TVAR_afit_pBaseValS[486] /AFIT8_Demosaicing_iDemBlurRange[7:0]    AFIT8_Sharpening_iLowSharpPower [15:8]	  
		mt9t111_write(0x0F12 , 0x1040); 	//70000B92 //TVAR_afit_pBaseValS[487] /AFIT8_Sharpening_iHighSharpPower[7:0]	AFIT8_Sharpening_iLowShDenoise [15:8]	  
		mt9t111_write(0x0F12 , 0x4012); 	//70000B94 //TVAR_afit_pBaseValS[488] /AFIT8_Sharpening_iHighShDenoise [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult [15:8]
		mt9t111_write(0x0F12 , 0x0604); 	//70000B96 //TVAR_afit_pBaseValS[489] /AFIT8_Sharpening_iReduceEdgeSlope [7:0]	 AFIT8_demsharpmix1_iWideFiltReduce [15:8]
		mt9t111_write(0x0F12 , 0x4606); 	//70000B98 //TVAR_afit_pBaseValS[490] /AFIT8_demsharpmix1_iNarrFiltReduce [7:0]   AFIT8_sddd8a_iClustThresh_H_Bin [15:8]  
		mt9t111_write(0x0F12 , 0x0146); 	//70000B9A //TVAR_afit_pBaseValS[491] /AFIT8_sddd8a_iClustThresh_C_Bin [7:0]	AFIT8_sddd8a_iClustMulT_H_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x0101); 	//70000B9C //TVAR_afit_pBaseValS[492] /AFIT8_sddd8a_iClustMulT_C_Bin [7:0]	  AFIT8_sddd8a_nClustLevel_H_Bin [15:8] 	  
		mt9t111_write(0x0F12 , 0x1C18); 	//70000B9E //TVAR_afit_pBaseValS[493] /AFIT8_sddd8a_DispTH_Low_Bin [7:0]	AFIT8_sddd8a_DispTH_High_Bin [15:8] 		  
		mt9t111_write(0x0F12 , 0x1819); 	//70000BA0 //TVAR_afit_pBaseValS[494] /AFIT8_sddd8a_iDenThreshLow_Bin [7:0]    AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]	  
		mt9t111_write(0x0F12 , 0x0028); 	//70000BA2 //TVAR_afit_pBaseValS[495] /AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]	AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]		
		mt9t111_write(0x0F12 , 0x030A); 	//70000BA4 //TVAR_afit_pBaseValS[496] /AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]   AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8] 
		mt9t111_write(0x0F12 , 0x0514); 	//70000BA6 //TVAR_afit_pBaseValS[497] /AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]	AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]	
		mt9t111_write(0x0F12 , 0x0C14); 	//70000BA8 //TVAR_afit_pBaseValS[498] /AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]	 AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]	
		mt9t111_write(0x0F12 , 0xFF05); 	//70000BAA //TVAR_afit_pBaseValS[499] /AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]   AFIT8_Demosaicing_iDemBlurLow_Bin [15:8] 
		mt9t111_write(0x0F12 , 0x0432); 	//70000BAC //TVAR_afit_pBaseValS[500] /AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]   AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]		
		mt9t111_write(0x0F12 , 0x4052); 	//70000BAE //TVAR_afit_pBaseValS[501] /AFIT8_Sharpening_iLowSharpPower_Bin [7:0]   AFIT8_Sharpening_iHighSharpPower_Bin [15:8]		
		mt9t111_write(0x0F12 , 0x1514); 	//70000BB0 //TVAR_afit_pBaseValS[502] /AFIT8_Sharpening_iLowShDenoise_Bin [7:0]   AFIT8_Sharpening_iHighShDenoise_Bin [15:8]		
		mt9t111_write(0x0F12 , 0x0440); 	//70000BB2 //TVAR_afit_pBaseValS[503] /AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]   AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8] 
		mt9t111_write(0x0F12 , 0x0302); 	//70000BB4 //TVAR_afit_pBaseValS[504] /AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]   AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8] 
		mt9t111_write(0x0F12 , 0x4646); 	//70000BB6 //TVAR_afit_pBaseValS[505] /AFIT8_sddd8a_iClustThresh_H_sBin[7:0]	AFIT8_sddd8a_iClustThresh_C_sBin [15:8] 			
		mt9t111_write(0x0F12 , 0x0101); 	//70000BB8 //TVAR_afit_pBaseValS[506] /AFIT8_sddd8a_iClustMulT_H_sBin [7:0]    AFIT8_sddd8a_iClustMulT_C_sBin [15:8]				
		mt9t111_write(0x0F12 , 0x1801); 	//70000BBA //TVAR_afit_pBaseValS[507] /AFIT8_sddd8a_nClustLevel_H_sBin [7:0]	AFIT8_sddd8a_DispTH_Low_sBin [15:8] 				
		mt9t111_write(0x0F12 , 0x191C); 	//70000BBC //TVAR_afit_pBaseValS[508] /AFIT8_sddd8a_DispTH_High_sBin [7:0]	  AFIT8_sddd8a_iDenThreshLow_sBin [15:8]				
		mt9t111_write(0x0F12 , 0x2818); 	//70000BBE //TVAR_afit_pBaseValS[509] /AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]	AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]			
		mt9t111_write(0x0F12 , 0x0A00); 	//70000BC0 //TVAR_afit_pBaseValS[510] /AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
		mt9t111_write(0x0F12 , 0x1403); 	//70000BC2 //TVAR_afit_pBaseValS[511] /AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]   AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]	  
		mt9t111_write(0x0F12 , 0x1405); 	//70000BC4 //TVAR_afit_pBaseValS[512] /AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]   AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]   
		mt9t111_write(0x0F12 , 0x050C); 	//70000BC6 //TVAR_afit_pBaseValS[513] /AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]	 AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
		mt9t111_write(0x0F12 , 0x32FF); 	//70000BC8 //TVAR_afit_pBaseValS[514] /AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]   AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]			
		mt9t111_write(0x0F12 , 0x5204); 	//70000BCA //TVAR_afit_pBaseValS[515] /AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]	AFIT8_Sharpening_iLowSharpPower_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x1440); 	//70000BCC //TVAR_afit_pBaseValS[516] /AFIT8_Sharpening_iHighSharpPower_sBin [7:0]	 AFIT8_Sharpening_iLowShDenoise_sBin [15:8] 		
		mt9t111_write(0x0F12 , 0x4015); 	//70000BCE //TVAR_afit_pBaseValS[517] /AFIT8_Sharpening_iHighShDenoise_sBin [7:0]	AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8] 	
		mt9t111_write(0x0F12 , 0x0204); 	//70000BD0 //TVAR_afit_pBaseValS[518] /AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]   AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]	
		mt9t111_write(0x0F12 , 0x0003); 	//70000BD2 //TVAR_afit_pBaseValS[519] /AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]													
		
		mt9t111_write(0x0F12 , 0x7DFA); 	//afit_pConstBaseValS[0] 
		mt9t111_write(0x0F12 , 0xFFBD); 	//afit_pConstBaseValS[1] 
		mt9t111_write(0x0F12 , 0x26FE); 	//afit_pConstBaseValS[2] 
		mt9t111_write(0x0F12 , 0xF7BC); 	//afit_pConstBaseValS[3] 
		mt9t111_write(0x0F12 , 0x7E06); 	//afit_pConstBaseValS[4] 
		mt9t111_write(0x0F12 , 0x00D3); 	//afit_pConstBaseValS[5] 
		
		
		mt9t111_write(0x002A , 0x2CE8); 
		mt9t111_write(0x0F12 , 0x0007); 	//Modify LSB to control AWBB_YThreshLow //
		mt9t111_write(0x0F12 , 0x00E2); 	//
		mt9t111_write(0x0F12 , 0x0005); 	//Modify LSB to control AWBB_YThreshLowBrLow//
		mt9t111_write(0x0F12 , 0x00E2); 	//
		
		
		mt9t111_write(0x002A , 0x0F7E);   //ae weight// 1219 all 0101
		mt9t111_write(0x0F12 , 0x0101);   //0000  //0000
		mt9t111_write(0x0F12 , 0x0101);   //0000  //0000
		mt9t111_write(0x0F12 , 0x0101);   //0000  //0000
		mt9t111_write(0x0F12 , 0x0101);   //0000  //0000
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0201  //0201
		mt9t111_write(0x0F12 , 0x0101);   //0102  //0102
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0101
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0201
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0202  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0102
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0201
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0202
		mt9t111_write(0x0F12 , 0x0101);   //0101  //0102
		
		
		mt9t111_write(0x002A , 0x01CC); 
		mt9t111_write(0x0F12 , 0x5DC0); 	//REG_TC_IPRM_InClockLSBs//input clock=24MHz
		mt9t111_write(0x0F12 , 0x0000);   //REG_TC_IPRM_InClockMSBs
		mt9t111_write(0x002A , 0x01EE); 
		mt9t111_write(0x0F12 , 0x0000); 	//REG_TC_IPRM_UseNPviClocks
		mt9t111_write(0x0F12 , 0x0003);  //#REG_TC_IPRM_UseNMiPiClocks
		
		mt9t111_write(0x002A , 0x01F6); 
		mt9t111_write(0x0F12 , 0x2328); 	//REG_TC_IPRM_OpClk4KHz_0						2	700001F6
		mt9t111_write(0x0F12 , 0x32A8); 
		mt9t111_write(0x0F12 , 0x32E8); 
		
		mt9t111_write(0x0F12 , 0x1F40); 	  //REG_TC_IPRM_OpClk4KHz_1 					2	700001FC
		mt9t111_write(0x0F12 , 0x2ea0); 	//REG_TC_IPRM_MinOutRate4KHz_1					2	700001FE
		mt9t111_write(0x0F12 , 0x2f00); 	//REG_TC_IPRM_MaxOutRate4KHz_1					2	70000200
		
		mt9t111_write(0x0F12 , 0x0BB8); 	//REG_TC_IPRM_OpClk4KHz_2						2	70000202
		mt9t111_write(0x0F12 , 0x05DC); 	//REG_TC_IPRM_MinOutRate4KHz_2					2	70000204
		mt9t111_write(0x0F12 , 0x1770); 	//REG_TC_IPRM_MaxOutRate4KHz_2					2	70000206
		
		mt9t111_write(0x002A , 0x0208); 
		mt9t111_write(0x0F12 , 0x0001); 
		
		msleep(100); //delayms(100);
		
		//AWB
		
		mt9t111_write(0x002A , 0x0D44); 
		mt9t111_write(0x0F12 , 0x0020); // awbb_MinNumOfChromaClassifyPatches
		
		mt9t111_write(0x002A , 0x020E); 
		mt9t111_write(0x0F12 , 0x0015); //Cont	
		
		mt9t111_write(0x0028 , 0x7000); 
		mt9t111_write(0x002A , 0x026C);    //Normal preview 15fps
		mt9t111_write(0x0F12 , 0x0280); 	//REG_0TC_PCFG_usWidth
		mt9t111_write(0x0F12 , 0x01E0); 	//REG_0TC_PCFG_usHeight
		mt9t111_write(0x0F12 , 0x0005);    //REG_0TC_PCFG_Format						2	70000270	//
		mt9t111_write(0x0F12 , 0x32E8); 
		mt9t111_write(0x0F12 , 0x32a8); 
		mt9t111_write(0x0F12 , 0x0100);    //REG_0TC_PCFG_OutClkPerPix88				2	70000276	//
		mt9t111_write(0x0F12 , 0x0800);    //REG_0TC_PCFG_uMaxBpp88 					2	70000278	//
		mt9t111_write(0x0F12 , 0x0040);//old: 0x0050 	 //REG_0TC_PCFG_PVIMask 2 92->96->52->5A->5C						2	7000027A	//92  (1) PCLK inversion  (4)1b_C first (5) UV First
		mt9t111_write(0x0F12 , 0x0010);    //REG_0TC_PCFG_OIFMask						2	7000027C	//
		mt9t111_write(0x0F12 , 0x01E0);    //REG_0TC_PCFG_usJpegPacketSize				2	7000027E	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_usJpegTotalPackets			2	70000280	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_uClockInd 					2	70000282	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_usFrTimeType					2	70000284	//
		mt9t111_write(0x0F12 , 0x0001);    //REG_0TC_PCFG_FrRateQualityType 			2	70000286	//
		mt9t111_write(0x0F12 , 0x0535);    //REG_0TC_PCFG_usMaxFrTimeMsecMult10 		2	70000288	//
		mt9t111_write(0x0F12 , 0x01F4);    //REG_0TC_PCFG_usMinFrTimeMsecMult10 		2	7000028A	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_bSmearOutput					2	7000028C	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_sSaturation					2	7000028E	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_sSharpBlur					2	70000290	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_sColorTemp					2	70000292	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_uDeviceGammaIndex 			2	70000294	//
		mt9t111_write(0x0F12 , 0x0001);//REG_0TC_PCFG_uPrevMirror		 0x0003			2	70000296	//
		mt9t111_write(0x0F12 , 0x0001);    //REG_0TC_PCFG_uCaptureMirror	0x0003			2	70000298	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_PCFG_uRotation 					2	7000029A	//
		
		mt9t111_write(0x0F12 , 0x0400); 	//REG_1TC_PCFG_usWidth
		mt9t111_write(0x0F12 , 0x0300); 	//REG_1TC_PCFG_usHeight 				  
		mt9t111_write(0x0F12 , 0x0005);    //REG_1TC_PCFG_Format						2	700002A0	//
		mt9t111_write(0x0F12 , 0x32E8);    //REG_1TC_PCFG_usMaxOut4KHzRate				2	700002A2	//
		mt9t111_write(0x0F12 , 0x32a8);    //REG_1TC_PCFG_usMinOut4KHzRate				2	700002A4	//
		mt9t111_write(0x0F12 , 0x0100);    //REG_1TC_PCFG_OutClkPerPix88				2	700002A6	//
		mt9t111_write(0x0F12 , 0x0800);    //REG_1TC_PCFG_uMaxBpp88 					2	700002A8	//
		mt9t111_write(0x0F12 , 0x0050); //0092	 //REG_1TC_PCFG_PVIMask 						2	700002AA	//
		mt9t111_write(0x0F12 , 0x0010);    //REG_1TC_PCFG_OIFMask						2	700002AC	//
		mt9t111_write(0x0F12 , 0x01E0);    //REG_1TC_PCFG_usJpegPacketSize				2	700002AE	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_usJpegTotalPackets			2	700002B0	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_uClockInd 					2	700002B2	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_usFrTimeType					2	700002B4	//
		mt9t111_write(0x0F12 , 0x0001);    //REG_1TC_PCFG_FrRateQualityType 			2	700002B6	//
		mt9t111_write(0x0F12 , 0x02B0);    //REG_1TC_PCFG_usMaxFrTimeMsecMult10 		2	700002B8	//
		mt9t111_write(0x0F12 , 0x02B0);    //REG_1TC_PCFG_usMinFrTimeMsecMult10 		2	700002BA	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_bSmearOutput					2	700002BC	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_sSaturation					2	700002BE	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_sSharpBlur					2	700002C0	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_sColorTemp					2	700002C2	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_uDeviceGammaIndex 			2	700002C4	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_uPrevMirror					2	700002C6	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_uCaptureMirror				2	700002C8	//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_PCFG_uRotation 					2	700002CA	//
		
		
		mt9t111_write(0x002A , 0x035C);    //Normal capture //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_uCaptureMode					2	7000035C						// 
		mt9t111_write(0x0F12 , 0x0800);    //REG_0TC_CCFG_usWidth						2	7000035E		  //
		mt9t111_write(0x0F12 , 0x0600);    //REG_0TC_CCFG_usHeight						2	70000360		  //
		mt9t111_write(0x0F12 , 0x0005);    //REG_0TC_CCFG_Format						2	70000362		  //
		mt9t111_write(0x0F12 , 0x32E8); 	
		mt9t111_write(0x0F12 , 0x32a8); 	
		mt9t111_write(0x0F12 , 0x0100);    //REG_0TC_CCFG_OutClkPerPix88				2	70000368		  //
		mt9t111_write(0x0F12 , 0x0800);    //REG_0TC_CCFG_uMaxBpp88 					2	7000036A		  //
		mt9t111_write(0x0F12 , 0x0040); //0092	 //REG_0TC_CCFG_PVIMask 						2	7000036C		  //
		mt9t111_write(0x0F12 , 0x0010);    //REG_0TC_CCFG_OIFMask						2	7000036E		  //
		mt9t111_write(0x0F12 , 0x01E0);    //REG_0TC_CCFG_usJpegPacketSize				2	70000370		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_usJpegTotalPackets			2	70000372		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_uClockInd 					2	70000374		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_usFrTimeType					2	70000376		  //
		mt9t111_write(0x0F12 , 0x0002);    //REG_0TC_CCFG_FrRateQualityType 			2	70000378		  //
		mt9t111_write(0x0F12 , 0x0535);    //REG_0TC_CCFG_usMaxFrTimeMsecMult10 		2	7000037A		  //
		mt9t111_write(0x0F12 , 0x0535);    //REG_0TC_CCFG_usMinFrTimeMsecMult10 		2	7000037C		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_bSmearOutput					2	7000037E		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_sSaturation					2	70000380		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_sSharpBlur					2	70000382		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_sColorTemp					2	70000384		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_0TC_CCFG_uDeviceGammaIndex 			2	70000386		  //
		
		// Low_lux capture//
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_uCaptureMode					2	70000388						//
		mt9t111_write(0x0F12 , 0x0800);    //REG_1TC_CCFG_usWidth						2	7000038A		  //
		mt9t111_write(0x0F12 , 0x0600);    //REG_1TC_CCFG_usHeight						2	7000038C		  //
		mt9t111_write(0x0F12 , 0x0005);    //REG_1TC_CCFG_Format						2	7000038E		  //
		mt9t111_write(0x0F12 , 0x32E8);    //REG_1TC_CCFG_usMaxOut4KHzRate				2	70000390		  //
		mt9t111_write(0x0F12 , 0x32a8);    //REG_1TC_CCFG_usMinOut4KHzRate				2	70000392		  //
		mt9t111_write(0x0F12 , 0x0100);    //REG_1TC_CCFG_OutClkPerPix88				2	70000394		  //
		mt9t111_write(0x0F12 , 0x0800);    //REG_1TC_CCFG_uMaxBpp88 					2	70000396		  //
		mt9t111_write(0x0F12 , 0x0050); //0092	 //REG_1TC_CCFG_PVIMask 						2	70000398		  //
		mt9t111_write(0x0F12 , 0x0010);    //REG_1TC_CCFG_OIFMask						2	7000039A		  //
		mt9t111_write(0x0F12 , 0x01E0);    //REG_1TC_CCFG_usJpegPacketSize				2	7000039C		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_usJpegTotalPackets			2	7000039E		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_uClockInd 					2	700003A0		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_usFrTimeType					2	700003A2		  //
		mt9t111_write(0x0F12 , 0x0002);    //REG_1TC_CCFG_FrRateQualityType 			2	700003A4		  //
		mt9t111_write(0x0F12 , 0x07D0);    //REG_1TC_CCFG_usMaxFrTimeMsecMult10 		2	700003A6		  //
		mt9t111_write(0x0F12 , 0x0535);    //REG_1TC_CCFG_usMinFrTimeMsecMult10 		2	700003A8		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_bSmearOutput					2	700003AA		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_sSaturation					2	700003AC		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_sSharpBlur					2	700003AE		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_sColorTemp					2	700003B0		  //
		mt9t111_write(0x0F12 , 0x0000);    //REG_1TC_CCFG_uDeviceGammaIndex 			2	700003B2		  //
		
		mt9t111_write(0x002A , 0x0208); 
		mt9t111_write(0x0F12 , 0x0001);    //REG_TC_IPRM_InitParamsUpdated//
		
		mt9t111_write(0x002A , 0x023C); 
		mt9t111_write(0x0F12 , 0x0000); 	 //32MHz Sys Clock//
		mt9t111_write(0x002A , 0x0244); 
		mt9t111_write(0x0F12 , 0x0000); 
		mt9t111_write(0x002A , 0x0240); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x002A , 0x0230); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x002A , 0x023E); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x002A , 0x0246); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x002A , 0x0220); 
		mt9t111_write(0x0F12 , 0x0001); 
		mt9t111_write(0x0F12 , 0x0001); 
		
		mt9t111_write(0x1000 , 0x0001); 
		
		msleep(100); //delayms(100);
		
		mt9t111_write(0x002A , 0x0DA0); 
		mt9t111_write(0x0F12 , 0x0005); 
		
		mt9t111_write(0x002A , 0x0D88); 
		mt9t111_write(0x0F12 , 0x0038); 
		mt9t111_write(0x0F12 , 0x0074); 
		mt9t111_write(0x0F12 , 0xFFF1); 
		mt9t111_write(0x0F12 , 0x00BF); 
		mt9t111_write(0x0F12 , 0xFF9B); 
		mt9t111_write(0x0F12 , 0x00DB); 
		mt9t111_write(0x0F12 , 0xFF56); 
		mt9t111_write(0x0F12 , 0x00F0); 
		mt9t111_write(0x0F12 , 0xFEFF); 
		mt9t111_write(0x0F12 , 0x010F); 
		mt9t111_write(0x0F12 , 0x0E74); 
		mt9t111_write(0x002A , 0x0DA8); 
		mt9t111_write(0x0F12 , 0x0BB8); 	//NB 3000
		mt9t111_write(0x002A , 0x0DA4); 
		mt9t111_write(0x0F12 , 0x274E); 
		
		mt9t111_write(0x002A , 0x0DCA); 
		mt9t111_write(0x0F12 , 0x0060); 
		
		mt9t111_write(0x002A , 0x3286); 
		mt9t111_write(0x0F12 , 0x0001);    //Pre/Post gamma on(??)
		
		mt9t111_write(0x002A , 0x032C); 
		mt9t111_write(0x0F12 , 0xAAAA);    //ESD Check 
		
		
		mt9t111_write(0x002A , 0x032E); 
		
		mt9t111_write(0x0F12 , 0xFFFE); 	//HighLux over this NB
		mt9t111_write(0x0F12 , 0x0000); 
		
		mt9t111_write(0x0F12 , 0x0020);	   //LowLux under this NB
		mt9t111_write(0x0F12 , 0x0000); 
		
/*for debug only */
		mt9t111_write(0x002C , 0x7000);	   //LowLux under this NB
		mt9t111_write(0x002E , 0x026C);

		printk("-----1: 0x%x \n ",mt9t111_read(0x0F12));
			printk("-----2:0x%x \n ",mt9t111_read(0x0F12));
				printk("-----3:0x%x \n ",mt9t111_read(0x0F12));
					printk("-----4:0x%x \n ",mt9t111_read(0x0F12));

		mt9t111_write(0x002C , 0x7000);	   //LowLux under this NB
		mt9t111_write(0x002E , 0x16B6);

		printk("-----5: 0x%x \n ",mt9t111_read(0x0F12));
/*debug end */
		return result;
}

static HAL_CAM_Result_en_t Init_Mt9t111_floatFPS(CamSensorSelect_t sensor)
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
			  * XMCLK 26MHz PCLK 64MHz
		   */
       #if defined (CONFIG_CAM_CSI2)
		   printk(KERN_ERR"Init_Mt9t111 CONFIG_CAM_CSI2 has been defined\n ");
	#endif 
		
		   checkCameraID(sensor);
		//reset MCLK=24M . PCLK=240;

		
msleep(100);	//Delay=100

mt9t111_write( 0x001A, 0x0051 );	// RESET_AND_MISC_CONTROL
msleep(2);												//DELAY=2
mt9t111_write( 0x001A, 0x0050 );	// RESET_AND_MISC_CONTROL
mt9t111_write( 0x001A, 0x0058 );	// RESET_AND_MISC_CONTROL
msleep(10);												//DELAY=10
mt9t111_write( 0x0014, 0x21F9 );	// PLL_CONTROL
mt9t111_write( 0x0010, 0x0115 );	// PLL_DIVIDERS
mt9t111_write( 0x0012, 0x00F5 );	// PLL_P_DIVIDERS
mt9t111_write( 0x0014, 0x2545 );	// PLL_CONTROL
mt9t111_write( 0x0014, 0x2547 );	// PLL_CONTROL
mt9t111_write( 0x0014, 0x2447 );	// PLL_CONTROL
msleep(10);												//DELAY=10                      );
mt9t111_write( 0x0014, 0x2047 );	// PLL_CONTROL
mt9t111_write( 0x0014, 0x2046 );	// PLL_CONTROL
mt9t111_write( 0x0018, 0x402D );	// STANDBY_CONTROL
mt9t111_write( 0x0018, 0x402C );	// STANDBY_CONTROL
//  POLL  STANDBY_CONTROL::STANDBY_DONE =>  0x01, 0x00
msleep(50);	//DELAY=50

//800x600
//mt9t111_write( 0x098C, 0x2703 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
//mt9t111_write( 0x0990, 0x0320 );	// MCU_DATA_0
//mt9t111_write( 0x098C, 0x2705 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
//mt9t111_write( 0x0990, 0x0258 );	// MCU_DATA_0

//640x480
mt9t111_write( 0x098C, 0x2703 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
mt9t111_write( 0x0990, 0x0280 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2705 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
mt9t111_write( 0x0990, 0x01E0 );	// MCU_DATA_0

mt9t111_write( 0x098C, 0x2707 );	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_B]
mt9t111_write( 0x0990, 0x0640 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2709 );	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_B]
mt9t111_write( 0x0990, 0x04B0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x270D );	// MCU_ADDRESS [MODE_SENSOR_ROW_START_A]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x270F );	// MCU_ADDRESS [MODE_SENSOR_COL_START_A]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2711 );	// MCU_ADDRESS [MODE_SENSOR_ROW_END_A]
mt9t111_write( 0x0990, 0x04BD );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2713 );	// MCU_ADDRESS [MODE_SENSOR_COL_END_A]
mt9t111_write( 0x0990, 0x064D );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2715 );	// MCU_ADDRESS [MODE_SENSOR_ROW_SPEED_A]
mt9t111_write( 0x0990, 0x0111 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2717 );	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_A]
mt9t111_write( 0x0990, 0x046C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2719 );	// MCU_ADDRESS [MODE_SENSOR_FINE_CORRECTION_A]
mt9t111_write( 0x0990, 0x005A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x271B );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MIN_A]
mt9t111_write( 0x0990, 0x01BE );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x271D );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MAX_MARGIN_A]
mt9t111_write( 0x0990, 0x0131 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x271F );	// MCU_ADDRESS [MODE_SENSOR_FRAME_LENGTH_A]
mt9t111_write( 0x0990, 0x02BB );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2721 );	// MCU_ADDRESS [MODE_SENSOR_LINE_LENGTH_PCK_A]
mt9t111_write( 0x0990, 0x0888 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2723 );	// MCU_ADDRESS [MODE_SENSOR_ROW_START_B]
mt9t111_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2725 );	// MCU_ADDRESS [MODE_SENSOR_COL_START_B]
mt9t111_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2727 );	// MCU_ADDRESS [MODE_SENSOR_ROW_END_B]
mt9t111_write( 0x0990, 0x04BB );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2729 );	// MCU_ADDRESS [MODE_SENSOR_COL_END_B]
mt9t111_write( 0x0990, 0x064B );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x272B );	// MCU_ADDRESS [MODE_SENSOR_ROW_SPEED_B]
mt9t111_write( 0x0990, 0x0111 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x272D );	// MCU_ADDRESS [MODE_SENSOR_READ_MODE_B]
mt9t111_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x272F );	// MCU_ADDRESS [MODE_SENSOR_FINE_CORRECTION_B]
mt9t111_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2731 );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MIN_B]
mt9t111_write( 0x0990, 0x00F6 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2733 );	// MCU_ADDRESS [MODE_SENSOR_FINE_IT_MAX_MARGIN_B]
mt9t111_write( 0x0990, 0x008B );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2735 );	// MCU_ADDRESS [MODE_SENSOR_FRAME_LENGTH_B]
mt9t111_write( 0x0990, 0x0521 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2737 );	// MCU_ADDRESS [MODE_SENSOR_LINE_LENGTH_PCK_B]
mt9t111_write( 0x0990, 0x0888 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2739 );	// MCU_ADDRESS [MODE_CROP_X0_A]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x273B );	// MCU_ADDRESS [MODE_CROP_X1_A]
mt9t111_write( 0x0990, 0x031F );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x273D );	// MCU_ADDRESS [MODE_CROP_Y0_A]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x273F );	// MCU_ADDRESS [MODE_CROP_Y1_A]
mt9t111_write( 0x0990, 0x0257 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2747 );	// MCU_ADDRESS [MODE_CROP_X0_B]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2749 );	// MCU_ADDRESS [MODE_CROP_X1_B]
mt9t111_write( 0x0990, 0x063F );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x274B );	// MCU_ADDRESS [MODE_CROP_Y0_B]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x274D );	// MCU_ADDRESS [MODE_CROP_Y1_B]
mt9t111_write( 0x0990, 0x04AF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2222 );	// MCU_ADDRESS [AE_R9]
mt9t111_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA408 );	// MCU_ADDRESS [FD_SEARCH_F1_50]
mt9t111_write( 0x0990, 0x0026 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA409 );	// MCU_ADDRESS [FD_SEARCH_F2_50]
mt9t111_write( 0x0990, 0x0029 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA40A );	// MCU_ADDRESS [FD_SEARCH_F1_60]
mt9t111_write( 0x0990, 0x002E );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA40B );	// MCU_ADDRESS [FD_SEARCH_F2_60]
mt9t111_write( 0x0990, 0x0031 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2411 );	// MCU_ADDRESS [FD_R9_STEP_F60_A]
mt9t111_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2413 );	// MCU_ADDRESS [FD_R9_STEP_F50_A]
mt9t111_write( 0x0990, 0x00C0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2415 );	// MCU_ADDRESS [FD_R9_STEP_F60_B]
mt9t111_write( 0x0990, 0x00A0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2417 );	// MCU_ADDRESS [FD_R9_STEP_F50_B]
mt9t111_write( 0x0990, 0x00C0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]
mt9t111_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA40D );	// MCU_ADDRESS [FD_STAT_MIN]
mt9t111_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA40E );	// MCU_ADDRESS [FD_STAT_MAX]
mt9t111_write( 0x0990, 0x0003 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA410 );	// MCU_ADDRESS [FD_MIN_AMPLITUDE]
mt9t111_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA117 );	// MCU_ADDRESS [SEQ_PREVIEW_0_AE]
mt9t111_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA11D );	// MCU_ADDRESS [SEQ_PREVIEW_1_AE]
mt9t111_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA129 );	// MCU_ADDRESS [SEQ_PREVIEW_3_AE]
mt9t111_write( 0x0990, 0x0002 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA24F );	// MCU_ADDRESS [AE_BASETARGET]
mt9t111_write( 0x0990, 0x0032 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA20C );	// MCU_ADDRESS [AE_MAX_INDEX]
mt9t111_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA216 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA20E );	// MCU_ADDRESS [AE_MAX_VIRTGAIN]
mt9t111_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2212 );	// MCU_ADDRESS [AE_MAX_DGAIN_AE1]
mt9t111_write( 0x0990, 0x00A4 );	// MCU_DATA_0
mt9t111_write( 0x3210, 0x01B8 );	// COLOR_PIPELINE_CONTROL
mt9t111_write( 0x098C, 0xAB36 );	// MCU_ADDRESS [HG_CLUSTERDC_TH]
mt9t111_write( 0x0990, 0x0014 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B66 );	// MCU_ADDRESS [HG_CLUSTER_DC_BM]
mt9t111_write( 0x0990, 0x2AF8 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB20 );	// MCU_ADDRESS [HG_LL_SAT1]
mt9t111_write( 0x0990, 0x0080 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB24 );	// MCU_ADDRESS [HG_LL_SAT2]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB21 );	// MCU_ADDRESS [HG_LL_INTERPTHRESH1]
mt9t111_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB25 );	// MCU_ADDRESS [HG_LL_INTERPTHRESH2]
mt9t111_write( 0x0990, 0x002A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB22 );	// MCU_ADDRESS [HG_LL_APCORR1]
mt9t111_write( 0x0990, 0x0007 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB26 );	// MCU_ADDRESS [HG_LL_APCORR2]
mt9t111_write( 0x0990, 0x0001 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB23 );	// MCU_ADDRESS [HG_LL_APTHRESH1]
mt9t111_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB27 );	// MCU_ADDRESS [HG_LL_APTHRESH2]
mt9t111_write( 0x0990, 0x0009 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B28 );	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]
mt9t111_write( 0x0990, 0x0BB8 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B2A );	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]
mt9t111_write( 0x0990, 0x2968 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB2C );	// MCU_ADDRESS [HG_NR_START_R]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB30 );	// MCU_ADDRESS [HG_NR_STOP_R]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB2D );	// MCU_ADDRESS [HG_NR_START_G]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB31 );	// MCU_ADDRESS [HG_NR_STOP_G]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB2E );	// MCU_ADDRESS [HG_NR_START_B]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB32 );	// MCU_ADDRESS [HG_NR_STOP_B]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB2F );	// MCU_ADDRESS [HG_NR_START_OL]
mt9t111_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB33 );	// MCU_ADDRESS [HG_NR_STOP_OL]
mt9t111_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB34 );	// MCU_ADDRESS [HG_NR_GAINSTART]
mt9t111_write( 0x0990, 0x0020 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB35 );	// MCU_ADDRESS [HG_NR_GAINSTOP]
mt9t111_write( 0x0990, 0x0091 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA765 );	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_FILTER_MODE]
mt9t111_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB37 );	// MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]
mt9t111_write( 0x0990, 0x0003 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B38 );	// MCU_ADDRESS [HG_GAMMASTARTMORPH]
mt9t111_write( 0x0990, 0x2968 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B3A );	// MCU_ADDRESS [HG_GAMMASTOPMORPH]
mt9t111_write( 0x0990, 0x2D50 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B62 );	// MCU_ADDRESS [HG_FTB_START_BM]
mt9t111_write( 0x0990, 0xFFFE );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2B64 );	// MCU_ADDRESS [HG_FTB_STOP_BM]
mt9t111_write( 0x0990, 0xFFFF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB4F );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB50 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
mt9t111_write( 0x0990, 0x0013 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB51 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
mt9t111_write( 0x0990, 0x0027 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB52 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
mt9t111_write( 0x0990, 0x0043 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB53 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
mt9t111_write( 0x0990, 0x0068 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB54 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
mt9t111_write( 0x0990, 0x0081 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB55 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
mt9t111_write( 0x0990, 0x0093 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB56 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
mt9t111_write( 0x0990, 0x00A3 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB57 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
mt9t111_write( 0x0990, 0x00B0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB58 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
mt9t111_write( 0x0990, 0x00BC );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB59 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
mt9t111_write( 0x0990, 0x00C7 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB5A );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
mt9t111_write( 0x0990, 0x00D1 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB5B );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
mt9t111_write( 0x0990, 0x00DA );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB5C );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
mt9t111_write( 0x0990, 0x00E2 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB5D );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
mt9t111_write( 0x0990, 0x00E9 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB5E );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
mt9t111_write( 0x0990, 0x00EF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB5F );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
mt9t111_write( 0x0990, 0x00F4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB60 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
mt9t111_write( 0x0990, 0x00FA );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB61 );	// MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
mt9t111_write( 0x0990, 0x00FF );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2306 );	// MCU_ADDRESS [AWB_CCM_L_0]
mt9t111_write( 0x0990, 0x01D6 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2308 );	// MCU_ADDRESS [AWB_CCM_L_1]
mt9t111_write( 0x0990, 0xFF89 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x230A );	// MCU_ADDRESS [AWB_CCM_L_2]
mt9t111_write( 0x0990, 0xFFA1 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x230C );	// MCU_ADDRESS [AWB_CCM_L_3]
mt9t111_write( 0x0990, 0xFF73 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x230E );	// MCU_ADDRESS [AWB_CCM_L_4]
mt9t111_write( 0x0990, 0x019C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2310 );	// MCU_ADDRESS [AWB_CCM_L_5]
mt9t111_write( 0x0990, 0xFFF1 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2312 );	// MCU_ADDRESS [AWB_CCM_L_6]
mt9t111_write( 0x0990, 0xFFB0 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2314 );	// MCU_ADDRESS [AWB_CCM_L_7]
mt9t111_write( 0x0990, 0xFF2D );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2316 );	// MCU_ADDRESS [AWB_CCM_L_8]
mt9t111_write( 0x0990, 0x0223 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9t111_write( 0x0990, 0x001C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9t111_write( 0x0990, 0x0048 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9t111_write( 0x0990, 0x001C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9t111_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9t111_write( 0x0990, 0x001E );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9t111_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9t111_write( 0x0990, 0x0022 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9t111_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9t111_write( 0x0990, 0x002C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9t111_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2318 );	// MCU_ADDRESS [AWB_CCM_L_9]
mt9t111_write( 0x0990, 0x0024 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231A );	// MCU_ADDRESS [AWB_CCM_L_10]
mt9t111_write( 0x0990, 0x0038 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231C );	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9t111_write( 0x0990, 0xFFCD );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231E );	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9t111_write( 0x0990, 0x0023 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2320 );	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9t111_write( 0x0990, 0x0010 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2322 );	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9t111_write( 0x0990, 0x0026 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2324 );	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9t111_write( 0x0990, 0xFFE9 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2326 );	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9t111_write( 0x0990, 0xFFF1 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2328 );	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9t111_write( 0x0990, 0x003A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232A );	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9t111_write( 0x0990, 0x005D );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232C );	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9t111_write( 0x0990, 0xFF69 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9t111_write( 0x0990, 0x000C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9t111_write( 0x0990, 0xFFE4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9t111_write( 0x0990, 0x000C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9t111_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9t111_write( 0x0990, 0x000A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9t111_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9t111_write( 0x0990, 0x0006 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9t111_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9t111_write( 0x0990, 0xFFFC );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9t111_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232E );	// MCU_ADDRESS [AWB_CCM_RL_9]
mt9t111_write( 0x0990, 0x0004 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2330 );	// MCU_ADDRESS [AWB_CCM_RL_10]
mt9t111_write( 0x0990, 0xFFF4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x0415 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0xF601 );
mt9t111_write( 0x0992, 0x42C1 );
mt9t111_write( 0x0994, 0x0326 );
mt9t111_write( 0x0996, 0x11F6 );
mt9t111_write( 0x0998, 0x0143 );
mt9t111_write( 0x099A, 0xC104 );
mt9t111_write( 0x099C, 0x260A );
mt9t111_write( 0x099E, 0xCC04 );
mt9t111_write( 0x098C, 0x0425 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x33BD );
mt9t111_write( 0x0992, 0xA362 );
mt9t111_write( 0x0994, 0xBD04 );
mt9t111_write( 0x0996, 0x3339 );
mt9t111_write( 0x0998, 0xC6FF );
mt9t111_write( 0x099A, 0xF701 );
mt9t111_write( 0x099C, 0x6439 );
mt9t111_write( 0x099E, 0xFE01 );
mt9t111_write( 0x098C, 0x0435 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x6918 );
mt9t111_write( 0x0992, 0xCE03 );
mt9t111_write( 0x0994, 0x25CC );
mt9t111_write( 0x0996, 0x0013 );
mt9t111_write( 0x0998, 0xBDC2 );
mt9t111_write( 0x099A, 0xB8CC );
mt9t111_write( 0x099C, 0x0489 );
mt9t111_write( 0x099E, 0xFD03 );
mt9t111_write( 0x098C, 0x0445 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x27CC );
mt9t111_write( 0x0992, 0x0325 );
mt9t111_write( 0x0994, 0xFD01 );
mt9t111_write( 0x0996, 0x69FE );
mt9t111_write( 0x0998, 0x02BD );
mt9t111_write( 0x099A, 0x18CE );
mt9t111_write( 0x099C, 0x0339 );
mt9t111_write( 0x099E, 0xCC00 );
mt9t111_write( 0x098C, 0x0455 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x11BD );
mt9t111_write( 0x0992, 0xC2B8 );
mt9t111_write( 0x0994, 0xCC04 );
mt9t111_write( 0x0996, 0xC8FD );
mt9t111_write( 0x0998, 0x0347 );
mt9t111_write( 0x099A, 0xCC03 );
mt9t111_write( 0x099C, 0x39FD );
mt9t111_write( 0x099E, 0x02BD );
mt9t111_write( 0x098C, 0x0465 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0xDE00 );
mt9t111_write( 0x0992, 0x18CE );
mt9t111_write( 0x0994, 0x00C2 );
mt9t111_write( 0x0996, 0xCC00 );
mt9t111_write( 0x0998, 0x37BD );
mt9t111_write( 0x099A, 0xC2B8 );
mt9t111_write( 0x099C, 0xCC04 );
mt9t111_write( 0x099E, 0xEFDD );
mt9t111_write( 0x098C, 0x0475 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0xE6CC );
mt9t111_write( 0x0992, 0x00C2 );
mt9t111_write( 0x0994, 0xDD00 );
mt9t111_write( 0x0996, 0xC601 );
mt9t111_write( 0x0998, 0xF701 );
mt9t111_write( 0x099A, 0x64C6 );
mt9t111_write( 0x099C, 0x03F7 );
mt9t111_write( 0x099E, 0x0165 );
mt9t111_write( 0x098C, 0x0485 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x7F01 );
mt9t111_write( 0x0992, 0x6639 );
mt9t111_write( 0x0994, 0x3C3C );
mt9t111_write( 0x0996, 0x3C34 );
mt9t111_write( 0x0998, 0xCC32 );
mt9t111_write( 0x099A, 0x3EBD );
mt9t111_write( 0x099C, 0xA558 );
mt9t111_write( 0x099E, 0x30ED );
mt9t111_write( 0x098C, 0x0495 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x04BD );
mt9t111_write( 0x0992, 0xB2D7 );
mt9t111_write( 0x0994, 0x30E7 );
mt9t111_write( 0x0996, 0x06CC );
mt9t111_write( 0x0998, 0x323E );
mt9t111_write( 0x099A, 0xED00 );
mt9t111_write( 0x099C, 0xEC04 );
mt9t111_write( 0x099E, 0xBDA5 );
mt9t111_write( 0x098C, 0x04A5 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x44CC );
mt9t111_write( 0x0992, 0x3244 );
mt9t111_write( 0x0994, 0xBDA5 );
mt9t111_write( 0x0996, 0x585F );
mt9t111_write( 0x0998, 0x30ED );
mt9t111_write( 0x099A, 0x02CC );
mt9t111_write( 0x099C, 0x3244 );
mt9t111_write( 0x099E, 0xED00 );
mt9t111_write( 0x098C, 0x04B5 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0xF601 );
mt9t111_write( 0x0992, 0xD54F );
mt9t111_write( 0x0994, 0xEA03 );
mt9t111_write( 0x0996, 0xAA02 );
mt9t111_write( 0x0998, 0xBDA5 );
mt9t111_write( 0x099A, 0x4430 );
mt9t111_write( 0x099C, 0xE606 );
mt9t111_write( 0x099E, 0x3838 );
mt9t111_write( 0x098C, 0x04C5 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x3831 );
mt9t111_write( 0x0992, 0x39BD );
mt9t111_write( 0x0994, 0xD661 );
mt9t111_write( 0x0996, 0xF602 );
mt9t111_write( 0x0998, 0xF4C1 );
mt9t111_write( 0x099A, 0x0126 );
mt9t111_write( 0x099C, 0x0BFE );
mt9t111_write( 0x099E, 0x02BD );
mt9t111_write( 0x098C, 0x04D5 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0xEE10 );
mt9t111_write( 0x0992, 0xFC02 );
mt9t111_write( 0x0994, 0xF5AD );
mt9t111_write( 0x0996, 0x0039 );
mt9t111_write( 0x0998, 0xF602 );
mt9t111_write( 0x099A, 0xF4C1 );
mt9t111_write( 0x099C, 0x0226 );
mt9t111_write( 0x099E, 0x0AFE );
mt9t111_write( 0x098C, 0x04E5 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x02BD );
mt9t111_write( 0x0992, 0xEE10 );
mt9t111_write( 0x0994, 0xFC02 );
mt9t111_write( 0x0996, 0xF7AD );
mt9t111_write( 0x0998, 0x0039 );
mt9t111_write( 0x099A, 0x3CBD );
mt9t111_write( 0x099C, 0xB059 );
mt9t111_write( 0x099E, 0xCC00 );
mt9t111_write( 0x098C, 0x04F5 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x28BD );
mt9t111_write( 0x0992, 0xA558 );
mt9t111_write( 0x0994, 0x8300 );
mt9t111_write( 0x0996, 0x0027 );
mt9t111_write( 0x0998, 0x0BCC );
mt9t111_write( 0x099A, 0x0026 );
mt9t111_write( 0x099C, 0x30ED );
mt9t111_write( 0x099E, 0x00C6 );
mt9t111_write( 0x098C, 0x0505 );	// MCU_ADDRESS
mt9t111_write( 0x0990, 0x03BD );
mt9t111_write( 0x0992, 0xA544 );
mt9t111_write( 0x0994, 0x3839 );
mt9t111_write( 0x098C, 0x2006 );	// MCU_ADDRESS [MON_ARG1]
mt9t111_write( 0x0990, 0x0415 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA005 );	// MCU_ADDRESS [MON_CMD]
mt9t111_write( 0x0990, 0x0001 );	// MCU_DATA_0
msleep(10);	//DELAY= 10

// Edison, fine tune for test. 2011-7-7
//[A fine tune]
mt9t111_write( 0x098C, 0x2306 );	// MCU_ADDRESS [AWB_CCM_L_0]
mt9t111_write( 0x0990, 0x0163 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231C );	// MCU_ADDRESS [AWB_CCM_RL_0]
mt9t111_write( 0x0990, 0x0040 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2308 );	// MCU_ADDRESS [AWB_CCM_L_1]
mt9t111_write( 0x0990, 0xFFD8 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x231E );	// MCU_ADDRESS [AWB_CCM_RL_1]
mt9t111_write( 0x0990, 0xFFD4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x230A );	// MCU_ADDRESS [AWB_CCM_L_2]
mt9t111_write( 0x0990, 0xFFC5 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2320 );	// MCU_ADDRESS [AWB_CCM_RL_2]
mt9t111_write( 0x0990, 0xFFEC );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x230C );	// MCU_ADDRESS [AWB_CCM_L_3]
mt9t111_write( 0x0990, 0xFFB2 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2322 );	// MCU_ADDRESS [AWB_CCM_RL_3]
mt9t111_write( 0x0990, 0xFFE7 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x230E );	// MCU_ADDRESS [AWB_CCM_L_4]
mt9t111_write( 0x0990, 0x0150 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2324 );	// MCU_ADDRESS [AWB_CCM_RL_4]
mt9t111_write( 0x0990, 0x0035 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2310 );	// MCU_ADDRESS [AWB_CCM_L_5]
mt9t111_write( 0x0990, 0xFFFE );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2326 );	// MCU_ADDRESS [AWB_CCM_RL_5]
mt9t111_write( 0x0990, 0xFFE4 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2312 );	// MCU_ADDRESS [AWB_CCM_L_6]
mt9t111_write( 0x0990, 0xFFDD );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2328 );	// MCU_ADDRESS [AWB_CCM_RL_6]
mt9t111_write( 0x0990, 0x000D );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2314 );	// MCU_ADDRESS [AWB_CCM_L_7]
mt9t111_write( 0x0990, 0xFF95 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232A );	// MCU_ADDRESS [AWB_CCM_RL_7]
mt9t111_write( 0x0990, 0xFFF5 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x2316 );	// MCU_ADDRESS [AWB_CCM_L_8]
mt9t111_write( 0x0990, 0x018C );	// MCU_DATA_0
mt9t111_write( 0x098C, 0x232C );	// MCU_ADDRESS [AWB_CCM_RL_8]
mt9t111_write( 0x0990, 0x0000 );	// MCU_DATA_0

mt9t111_write( 0x098C, 0xA366 );	// MCU_ADDRESS [AWB_KR_L]
mt9t111_write( 0x0990, 0x0080 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA368 );	// MCU_ADDRESS [AWB_KB_L]
mt9t111_write( 0x0990, 0x008B );	// MCU_DATA_0

//[Day light]
mt9t111_write( 0x098C, 0xA369 );	// MCU_ADDRESS [AWB_KR_R]
mt9t111_write( 0x0990, 0x007A );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xAB20 );	// MCU_ADDRESS [HG_LL_SAT1]
mt9t111_write( 0x0990, 0x00B0 );	// MCU_DATA_0

// for CWF
mt9t111_write( 0x098C, 0xA36E );	// MCU_ADDRESS [AWB_EDGETH_MAX]
mt9t111_write( 0x0990, 0x0028 );	// MCU_DATA_0
mt9t111_write( 0x098C, 0xA363 );	// MCU_ADDRESS [AWB_TG_MIN0]
mt9t111_write( 0x0990, 0x00CF );	// MCU_DATA_0
// End finetune. 2011-7-7

///flicker

//[50HZ]
mt9t111_write(0x098C, 0xA118 );	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]        
mt9t111_write(0x0990, 0x0002 );	// MCU_DATA_0                                 
mt9t111_write(0x098C, 0xA11E );	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]             
mt9t111_write(0x0990, 0x0002 );	// MCU_DATA_0                                 
mt9t111_write(0x098C, 0xA124 );	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]             
mt9t111_write(0x0990, 0x0002 );	// MCU_DATA_0                                     
mt9t111_write(0x098C, 0xA12A );	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]            
mt9t111_write(0x0990, 0x0002 );	// MCU_DATA_0                                  
mt9t111_write(0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]                       
mt9t111_write(0x0990, 0x00E0 );	// MCU_DATA_0                               
mt9t111_write(0x098C, 0xA103 );         
mt9t111_write(0x0990, 0x0005 );
msleep(50);	//delay=50
//[ANTIBANDING_SET_AUTO]
mt9t111_write(0x098C, 0xA118 );	// MCU_ADDRESS [SEQ_PREVIEW_0_FD]          
mt9t111_write(0x0990, 0x0001 );	// MCU_DATA_0                              
mt9t111_write(0x098C, 0xA11E );	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]         
mt9t111_write(0x0990, 0x0001 );	// MCU_DATA_0                            
mt9t111_write(0x098C, 0xA124 );	// MCU_ADDRESS [SEQ_PREVIEW_2_FD]         
mt9t111_write(0x0990, 0x0000 );	// MCU_DATA_0                                  
mt9t111_write(0x098C, 0xA12A );	// MCU_ADDRESS [SEQ_PREVIEW_3_FD]          
mt9t111_write(0x0990, 0x0001 );	// MCU_DATA_0  
mt9t111_write(0x098C, 0xA404 );	// MCU_ADDRESS [FD_MODE]                      
mt9t111_write(0x0990, 0x0060 );	// MCU_DATA_0                                
//mt9t111_write(0x098C, 0xA103 	                  
//mt9t111_write(0x0990, 0x0005 	        
//  POLL  MON_PATCH_ID_0 =>  0x01
mt9t111_write( 0x0018, 0x0028 );	// STANDBY_CONTROL
msleep(10);	//DELAY= 10
//  POLL  SEQ_STATE =>  0x03
mt9t111_write( 0x098C, 0xA103 );	// MCU_ADDRESS [SEQ_CMD]
mt9t111_write( 0x0990, 0x0006 );	// MCU_DATA_0
	
	
	//mt9t111_write( 0x3070, 0x0100 );	// MCU_DATA_0  leyou:  a single patten
	
	/*added for debug only */	
			msleep(500);
			mt9t111_write( 0x098C, 0xA104 );
			msleep(50);
			printk(KERN_ERR"0x0990 read out : %d \n",mt9t111_read(0x0990));  /*expected to be 3 */
	/*---end ---*/
		
			printk(KERN_ERR"0x3400:read out :%d \n",mt9t111_read(0x3400));  /*0x3400*/
	
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
