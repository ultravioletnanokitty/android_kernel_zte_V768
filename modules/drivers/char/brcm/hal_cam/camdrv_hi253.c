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

#include <mach/reg_camera.h>
#include <mach/reg_lcd.h>

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

#include "hal_cam.h"
#include "hal_cam_drv.h"
#include "cam_cntrl_2153.h"
#include "camdrv_dev.h"

//#define MT9T111_ID 0x2680
#define HI253_ID 0x92

//#define HAL_CAM_RESET 22
#define HAL_CAM_RESET 56 //hanwei 2011.11.01
#define HAL_CAM_PWDN 13 //hanwei 2011.11.01

/* Start of Embedded Image (default value) */
#define SOEI	0xBEFF

/* Start of Status Information */
#define SOSI	0xBCFF

#ifndef ALLOC_TN_BUFFER
#define ALLOC_TN_BUFFER 1
#define TN_BUFFER_SIZE (320 * 240 * 2)	/* for storing thumbnail data. */
int tn_width, tn_height;
int jpeg_width, jpeg_height;

static char *thumbnail_data;
static char *jpeg_header_data;
static char *jpeg_raw_data;

struct DataFormat {
	/*Copy of the variable */
	unsigned short CONFIG_JPEG_OB_TX_CONTROL_VAR;  
	/*Copy of the variable */
	unsigned short CONFIG_JPEG_OB_SPOOF_CONTROL_VAR;
	/*Spoof Width used when the spoof mode is enabled */
	unsigned int spoof_width;
	/*Spoof Height used when the spoof mode is enabled */
	unsigned int spoof_heigth;
	/*Status used to define if thumbnail is embedded */
	unsigned char IS_TN_ENABLED;
	/*Thumbnail width used when the thumbnail is enabled */
	unsigned int thumbnail_width;
	/*Thumbnail height used when the thumbnail is enabled */
	unsigned int thumbnail_height;
	/*Load the offset of the last valid address */
	unsigned int JpegSize;
	/*Tell which Qscale to use. */
	unsigned short JpegStatus;
	/*0 for Monochrome, 2 for YUV422 and 3 for YUV420 */
	unsigned char JpegFormat;
	/*Jpeg Resolution */
	unsigned short JpegXsize;
	/*Jpeg resolution */
	unsigned short JpegYsize;
};

const unsigned char JFIFHeaderSize = 18;
const unsigned char LumaQtableSize = 64;
const unsigned char LumaHuffTableSizeDC = 33;
const unsigned char LumaHuffTableSizeAC = 183;
const unsigned char QTableHeaderSize = 4;
const unsigned char ChromaQtableSize = 64;
const unsigned char ChromaHuffTableSizeAC = 183;
const unsigned char ChromaHuffTableSizeDC = 33;
const unsigned char RestartIntervalSize = 6;
unsigned char StartOfFrameSize = 19;
const unsigned char StartOfScanSize = 14;
const unsigned char StartOfScanSizeM = 10;

const unsigned char JFIFHeader[18] = { 0xFF,	/* APP0 marker */
	0xE0,
	0x00,			/* length */
	0x10,
	0x4A,			/* JFIF identifier */
	0x46,
	0x49,
	0x46,
	0x00,
	0x01,			/* JFIF version */
	0x02,
	0x00,			/* units */
	0x00,			/* X density */
	0x01,
	0x00,			/* Y density */
	0x01,
	0x00,			/* X thumbnail */
	0x00			/* Y thumbnail */
};

unsigned char QTableHeader[4] = {
	0xff, 0xDB,		/*Qtable marker */
	0x00, 0x84,		/*Qtable size for 2 tables (only one for monochrome) */
};

const unsigned char JPEG_StdQuantTblY_ZZ[64] = {
	16, 11, 12, 14, 12, 10, 16, 14,
	13, 14, 18, 17, 16, 19, 24, 40,
	26, 24, 22, 22, 24, 49, 35, 37,
	29, 40, 58, 51, 61, 60, 57, 51,
	56, 55, 64, 72, 92, 78, 64, 68,
	87, 69, 55, 56, 80, 109, 81, 87,
	95, 98, 103, 104, 103, 62, 77, 113,
	121, 112, 100, 120, 92, 101, 103, 99
};

const unsigned char JPEG_StdQuantTblC_ZZ[64] = {
	17, 18, 18, 24, 21, 24, 47, 26,
	26, 47, 99, 66, 56, 66, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99
};

unsigned char quant_JPEG_Hdr[2 /*luma/chroma */][64] =
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x10, 0x0f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x10,
	0x00, 0x00, 0x10, 0x10, 0x13, 0x0c, 0x0e, 0x10,
	0x00, 0x00, 0x10, 0x10, 0x11, 0x13, 0x13, 0x13,
	0x00, 0x00, 0x00, 0x00, 0x04, 0x05, 0x09, 0x05,
	0x00, 0x00, 0x10, 0x00, 0x0b, 0x0c, 0x13, 0x13,
	0x00, 0x00, 0x10, 0x10, 0x13, 0x13, 0x13, 0x13,
	0x00, 0x00, 0x10, 0x13, 0x13, 0x13, 0x13, 0x13,
	0x00, 0x00, 0x10, 0x13, 0x13, 0x13, 0x13, 0x13,
	0x00, 0x00, 0x10, 0x13, 0x13, 0x13, 0x13, 0x13,
	0x00, 0x00, 0x10, 0x13, 0x13, 0x13, 0x13, 0x13,
	0x00, 0x00, 0x10, 0x13, 0x13, 0x13, 0x13, 0x13
};

const unsigned char LumaHuffTableDC[33] = {
	0xff, 0xc4,
	0x00, 0x1f,
	/* Luma DC Table */
	0x00,
	0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b
};

const unsigned char LumaHuffTableAC[183] = {
	0xff, 0xc4,
	0x00, 0xb5,
	/* Luma AC Table */
	0x10,
	0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
	0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d,
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

const unsigned char ChromaHuffTableDC[33] = {
	0xff, 0xc4,
	0x00, 0x1f,
	/* Chroma DC Table */
	0x01,
	0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b
};

const unsigned char ChromaHuffTableAC[183] = {
	0xff, 0xc4,
	0x00, 0xb5,
	/* Chroma AC Table */
	0x11,
	0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
	0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

/* DRI : Restart Interval = 0 MCU */
const unsigned char RestartInterval[6] = {
	0xff, 0xdd,
	0x00, 0x04,
	0x00, 0x00
};

const unsigned char StartOfFrame[19] = {
	0xff, 0xC0,		/*SOF Marker */
	0x00, 0x11,		/*SOF length */
	0x08,			/*Number of bits per pixel */
	0x06, 0x00, 0x08, 0x00,	/*Image resolution Y and X */
	0x03,			/*Number of components */
	0x00, 0x21, 0x00,	/* Y parameter: 0x21 for YUV422 0x22 for
				   YUV420 and related Qtable */
	0x01, 0x11, 0x01,	/* Cb parameter and related Qtable */
	0x02, 0x11, 0x01	/* Cr parameter and related Qtable */
};

const unsigned char StartOfScanM[10] = {
	0xff, 0xDA,		/*SOS Marker */
	0x00, 0x08,		/*SOS Length */
	0x01,			/*Number of components */
	0x00, 0x00,		/*Y parameter and Huffman table selector */
	0x00, 0x3F,
	0x00
};

const unsigned char StartOfScan[14] = {
	0xff, 0xDA,		/*SOS Marker */
	0x00, 0x0C,		/*SOS Length */
	0x03,			/*Number of components */
	0x00, 0x00,		/*Y parameter and Huffman table selector */
	0x01, 0x11,		/*Cb parameter and Huffman table selector */
	0x02, 0x11,		/*Cr parameter and Huffman table selector */
	0x00, 0x3F,
	0x00
};

#endif
/*****************************************************************************/
/* start of CAM configuration */
/*****************************************************************************/
/* Sensor select */
typedef enum {
	SensorHI253,
	SensorUnused
} CamSensor_t;

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

/*---------Sensor Power On */   //hanwei
static CamSensorIntfCntrl_st_t CamPowerOnSeq[] = {
#if 0
	{PAUSE, 1, Nop_Cmd},

	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},

/* -------Turn everything OFF   */
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetLow},
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},

/* -------Enable Clock to Cameras @ Main clock speed*/
	{MCLK_CNTRL, CamDrv_26MHz, CLK_TurnOn},
	{PAUSE, 10, Nop_Cmd},

/* -------Raise Reset to ISP*/
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd}
#else
       {PAUSE, 1, Nop_Cmd},

	{GPIO_CNTRL, HAL_CAM_PWDN, GPIO_SetHigh},
	{MCLK_CNTRL, CamDrv_24MHz, CLK_TurnOn},

	{GPIO_CNTRL, HAL_CAM_PWDN, GPIO_SetLow},
	{PAUSE, 10, Nop_Cmd},
		
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},
	
#endif

};

/*---------Sensor Power Off*/
static CamSensorIntfCntrl_st_t CamPowerOffSeq[] = {
	/* No Hardware Standby available. */
	{PAUSE, 50, Nop_Cmd},
/* -------Lower Reset to ISP*/
	{GPIO_CNTRL, HAL_CAM_RESET, GPIO_SetLow},
/* -------Disable Clock to Cameras*/
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
};

/** Primary Sensor Configuration and Capabilities  */
static HAL_CAM_IntConfigCaps_st_t CamPrimaryCfgCap_st = {
	/* CamSensorOutputCaps_st_t */
	{
	 CamDataFmtYCbCr |	/*< UInt32 formats;   CamDataFmt_t bit masked */
	 CamDataFmtJPEG,
	 2048,			/*< UInt32 max_width;   Maximum width resolution */
	 1536,			/*< UInt32 max_height;  Maximum height resolution */
	 CamImageSize_VGA,	/*< UInt32 ViewFinderResolutions;  ViewFinder
				   Resolutions (Maximum Resolution for now) */
	 CamImageSize_VGA,	/*< UInt32 VideoResolutions;  Video Resolutions
				   (Maximum Resolution for now) */
	 CamImageSize_QXGA,	/*< UInt32 StillsResolutions;  Stills Resolutions
				   (Maximum Resolution for now) */
	 1,			/*< UInt32 pre_frame_video;  frames to throw out for
				   ViewFinder/Video capture (total= pre_frame_video+1 */
	 3,			/*< UInt32 pre_frame_still;  frames to throw out for
				   Stills capture (total= pre_frame_still+1 */
	 TRUE,			/*< Boolean JpegCapable;     Sensor Jpeg
				   Capable: TRUE/FALSE: */
	 TRUE,			/*< Boolean StillnThumbCapable;
				   Sensor Still and Thumbnail Capable: TRUE/FALSE: */
	 TRUE			/*< Boolean VideonViewfinderCapable;
				   Sensor Video and Viewfinder Capable: TRUE/FALSE: */
	 },

	{
	 CamFocusControlOff,	/*< CamFocusControlMode_t
				   default_setting=CamFocusControlOff; */
	 CamFocusControlOff,	/*< CamFocusControlMode_t cur_setting; */
	 CamFocusControlOff	/*< UInt32 settings;  Settings Allowed:
				   CamFocusControlMode_t bit masked */
	 },

	/*< Digital Zoom Settings & Capabilities:
	   CamDigitalZoomMode_st_t digital_zoom_st; */
	{
	 CamZoom_1_0,		/*< CamZoom_t default_setting;
				   default=CamZoom_1_0:  Values allowed  CamZoom_t */
	 CamZoom_1_0,		/*< CamZoom_t cur_setting;  CamZoom_t */
	 CamZoom_1_0,		/*< CamZoom_t max_zoom;
				   Max Zoom Allowed(256/max_zoom = *zoom) */
	 FALSE			/*< Boolean capable;  Sensor capable: TRUE/FALSE: */
	 },

	/*< Sensor ESD Settings & Capabilities:  CamESD_st_t esd_st; */
	{
	 0x01,			/*< UInt8 ESDTimer;  Periodic timer to retrieve
				   the camera status (ms) */
	 FALSE			/*< Boolean capable;  TRUE/FALSE: */
	 },

	/*< Sensor version string */
	"HI253"
};

/*---------Sensor Primary Configuration CCIR656*/
static CamIntfConfig_CCIR656_st_t CamPrimaryCfg_CCIR656_st = {
/* Vsync, Hsync, Clock */
	TRUE,			/*[00] Boolean ext_sync_enable;
				   (default)TRUE: CCIR-656 with external VSYNC/HSYNC
				   FALSE: Embedded VSYNC/HSYNC */
	TRUE,			/*[01] Boolean hsync_control;
				   (default)FALSE: FALSE=all HSYNCS
				   TRUE: HSYNCS only during valid VSYNC  */
	SyncFallingEdge,	/*[02] UInt32 vsync_irq_trigger;
				   (default)SyncRisingEdge/SyncFallingEdge:
				   Vsync Irq trigger    */
	SyncActiveHigh,		/*[03] UInt32 vsync_polarity;
				   (default)SyncActiveLow/SyncActiveHigh:
				   Vsync active  */
	SyncFallingEdge,	/*[04] UInt32 hsync_irq_trigger;
				   (default)SyncRisingEdge/SyncFallingEdge:
				   Hsync Irq trigger */
	SyncActiveHigh,		/*[05] UInt32 hsync_polarity;
				   (default)SyncActiveLow/SyncActiveHigh:
				   Hsync active    */
	SyncFallingEdge		/*[06] UInt32 data_clock_sample;
				   (default)SyncRisingEdge/SyncFallingEdge:
				   Pixel Clock Sample edge */
};

/*---------Sensor Primary Configuration YCbCr Input*/
static CamIntfConfig_YCbCr_st_t CamPrimaryCfg_YCbCr_st = {
/* YCbCr(YUV422) Input format = YCbCr=YUV= Y0 U0 Y1 V0  Y2 U2 Y3 V2 ....*/
	TRUE,			/*[00] Boolean yuv_full_range;
				   (default)FALSE: CROPPED YUV=16-240
				   TRUE: FULL RANGE YUV= 1-254  */
	SensorYCSeq_YCbYCr,	/*[01] CamSensorYCbCrSeq_t sensor_yc_seq;
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
	&CamPrimaryCfg_YCbCr_st,	/* *sensor_config_ycbcr; */
	&CamPrimaryCfg_I2C_st,	/* *sensor_config_i2c; */
	&CamPrimaryCfg_IOCR_st,	/* *sensor_config_iocr; */
	&CamPrimaryCfg_Jpeg_st,	/* *sensor_config_jpeg; */
	NULL,			/* *sensor_config_interleave_video; */
	&CamPrimaryCfg_InterLeaveStills_st,	/**sensor_config_interleave_stills; */
	&CamPrimaryCfg_PktMarkerInfo_st	/* *sensor_config_pkt_marker_info; */
};

/* I2C transaction result */
static HAL_CAM_Result_en_t sCamI2cStatus = HAL_CAM_SUCCESS;

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution,
		     CamDataFmt_t image_format);
static HAL_CAM_Result_en_t Init_Hi253(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt16 hi253_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t hi253_write(unsigned int sub_addr, UInt16 data);
HAL_CAM_Result_en_t CAMDRV_SetFrameRate(CamRates_t fps,
					CamSensorSelect_t sensor);
HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode(CamImageSize_t image_resolution,
					       CamDataFmt_t image_format,
					       CamSensorSelect_t sensor);
HAL_CAM_Result_en_t CAMDRV_SetZoom(CamZoom_t step,CamSensorSelect_t sensor);
/*****************************************************************************
*
* Function Name:   CAMDRV_GetIntfConfigTbl
*
* Description: Return Camera Sensor Interface Configuration
*
* Notes:
*
*****************************************************************************/
CamIntfConfig_st_t *CAMDRV_GetIntfConfig(CamSensorSelect_t nSensor)
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
CamSensorIntfCntrl_st_t *CAMDRV_GetIntfSeqSel(CamSensorSelect_t nSensor,
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
			pr_debug("SensorPwrUp Sequence\r\n");
			*pLength = sizeof(CamPowerOnSeq);
			power_seq = CamPowerOnSeq;
		}
		break;

	case SensorInitPwrDn:	/* Camera Init Power Down (Unused) */
	case SensorPwrDn:	/* Both off */
		if ((nSensor == CamSensorPrimary)
		    || (nSensor == CamSensorSecondary)) {
			pr_debug("SensorPwrDn Sequence\r\n");
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
HAL_CAM_Result_en_t CAMDRV_Supp_Init(CamSensorSelect_t sensor)
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
HAL_CAM_Result_en_t CAMDRV_Wakeup(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	result = Init_Hi253(sensor);
	pr_debug("Init_Hi253 result =%d\r\n", result);
	return result;
}

UInt16 CAMDRV_GetDeviceID(CamSensorSelect_t sensor)
{
	//return hi253_read(0x00); //hanwei 2011.11.01
	return hi253_read(0x04); //hanwei 2011.11.01
}

static int checkCameraID(CamSensorSelect_t sensor)
{
	//UInt16 devId = CAMDRV_GetDeviceID(sensor);
	printk(KERN_ERR "hanwei :*******checkCameraID ****** \n");  
	UInt16 devId = hi253_read(0x04); //hanwei 2011.11.01
	pr_debug("hanwei:  *********Camera  HI253 devId = 0x% **********\r\n",devId);

	if (devId == HI253_ID) {
		pr_debug("Camera identified as HI253\r\n");
		return 0;
	} else {
		pr_debug("Camera Id wrong. Expected 0x%x but got 0x%x\r\n",
			 HI253_ID, devId);
		return -1;
	}
}

static HAL_CAM_Result_en_t hi253_write(unsigned int sub_addr, UInt16 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 bytes[2];
	bytes[0] = (data & 0xFF00) >> 8;
	bytes[1] = data & 0xFF;

	result |= CAM_WriteI2c(sub_addr, 2, bytes);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("hi253_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
	return result;
}

static UInt16 hi253_read(unsigned int sub_addr)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt16 data;
	UInt16 temp;

	result |= CAM_ReadI2c(sub_addr, 2, (UInt8 *) &data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("hi253_read(): ERROR: %d\r\n", result);
	}

	temp = data;
	data = ((temp & 0xFF) << 8) | ((temp & 0xFF00) >> 8);

	return data;
}

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format)
{
	UInt32 x = 0, y = 0;
	UInt32 format = 0;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

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

	/* Set preview resolution */
	/* [Preview XxY] */
	hi253_write(0x098E, 0x6800);	/* MCU_ADDRESS [PRI_A_IMAGE_WIDTH] */
	hi253_write(0x0990, x);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x6802);	/* MCU_ADDRESS [PRI_A_IMAGE_HEIGHT] */
	hi253_write(0x0990, y);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */

	/* [Format Selection] */
	hi253_write(0x098E, 0x6807);	/* MCU_ADDRESS [PRI_A_OUTPUT_FORMAT] */
	hi253_write(0x0990, format);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x6809);	/* MCU_ADDRESS[PRI_A_OUTPUT_FORMAT_ORDER] */
	hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0xE88E);	/* MCU_ADDRESS[PRI_A_CONFIG_JPEG_JP_MODE] */
	hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */

	if (image_format == CamDataFmtYCbCr) {
		UInt32 output_order = 2;	/* Switch low, high bytes. Y and C. */

		/* [Set Output Format Order] */
		/*MCU_ADDRESS[PRI_A_CONFIG_JPEG_JP_MODE] */
		hi253_write(0x098E, 0x6809);
		hi253_write(0x0990, output_order);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
		hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */

		pr_debug
		    ("SensorSetPreviewMode(): Output Format Order = 0x%x\r\n",
		     output_order);
	}

	pr_debug("SensorSetPreviewMode(): Resolution:0x%x, Format:0x%x r\n",
		 image_resolution, image_format);

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug
		    ("SensorSetPreviewMode(): Error[%d] sending preview mode  r\n",
		     sCamI2cStatus);
		result = sCamI2cStatus;
	}
	/*[Enable stream] */
	hi253_write(0x001A, 0x0218);

	return result;

}

/** The CAMDRV_GetResolutionAvailable returns the pointer to the sensor
	output size of the image width and height requested
    @param [in] width
		width of resolution requested from Sensor.
    @param [in] height
		height of resolution requested from Sensor.
    @param [in] mode
		Capture mode for resolution requested.
    @param [in] sensor
		Sensor for which resolution is requested.
    @param [out] *sensor_size
		Actual size of requested resolution.

    @return HAL_CAM_Result_en_t
		status returned.
 */
HAL_CAM_Result_en_t CAMDRV_GetResolutionAvailable(UInt32 width,
						  UInt32 height,
						  CamCaptureMode_t mode,
						  CamSensorSelect_t sensor,
						  HAL_CAM_ResolutionSize_st_t *
						  sensor_size)
{
	HAL_CAM_Result_en_t result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
	UInt16 index, res_size;
	CamResolutionConfigure_t *res_ptr;
	CamResolutionConfigure_t *best_match_ptr;

/* Resolution default settings */
	sensor_size->size = CamImageSize_INVALID;
	sensor_size->resX = 0;
	sensor_size->resY = 0;

/* Init Resolution table pointer */
	if (sensor == CamSensorPrimary) {
		res_size =
		    sizeof(sSensorResInfo_HI253_st) /
		    sizeof(CamResolutionConfigure_t);
		res_ptr = &sSensorResInfo_HI253_st[0];
	} else {
		pr_debug("CAMDRV_GetResolution(): ERROR:  Sensor Failed \r\n");
		result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
		return result;
	}

/* Search Resolution table for best match for requested width & height
	based on capture mode */
	best_match_ptr = res_ptr;
	for (index = 0; index < res_size; index++) {
		if ((mode == CamCaptureVideo)
		    || (mode == CamCaptureVideonViewFinder)) {
			if (res_ptr->previewIndex != -1) {
				best_match_ptr = res_ptr;
			}

			if ((width <= res_ptr->resX)
			    && (height <= res_ptr->resY)
			    && (res_ptr->previewIndex != -1)) {
				sensor_size->size =
				    (CamImageSize_t) res_ptr->previewIndex;
				sensor_size->resX = res_ptr->resX;
				sensor_size->resY = res_ptr->resY;
				result = HAL_CAM_SUCCESS;
				pr_debug
				    ("CAMDRV_GetResolutionAvailable(): Resolution: \
						size index=%d width=%d height=%d \r\n",
				     res_ptr->previewIndex, res_ptr->resX, res_ptr->resY);
				return result;
			}
		} else if ((mode == CamCaptureStill)
			   || (mode == CamCaptureStillnThumb)) {
			if (res_ptr->captureIndex != -1) {
				best_match_ptr = res_ptr;
			}

			if ((width <= res_ptr->resX)
			    && (height <= res_ptr->resY)
			    && (res_ptr->captureIndex != -1)) {
				sensor_size->size =
				    (CamImageSize_t) res_ptr->previewIndex;
				sensor_size->resX = res_ptr->resX;
				sensor_size->resY = res_ptr->resY;
				result = HAL_CAM_SUCCESS;
				pr_debug
				    ("CAMDRV_GetResolutionAvailable(): Resolution: \
					 size index=%d width=%d height=%d \r\n",
				     res_ptr->captureIndex, res_ptr->resX, res_ptr->resY);
				return result;
			}
		} else {
			pr_debug
			    ("CAMDRV_GetResolutionAvailable(): ERROR: Mode Failed\r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
		}
		/* Increment table pointer */
		res_ptr++;
	}
/* Resolution best match settings */
	pr_debug("CAMDRV_GetResolutionAvailable(): Best Match used \r\n");
	sensor_size->size = (CamImageSize_t) best_match_ptr->previewIndex;
	sensor_size->resX = best_match_ptr->resX;
	sensor_size->resY = best_match_ptr->resY;
	return result;
}

/** The CAMDRV_GetResolution returns the sensor output size of the
	image resolution requested
    @param [in] size
		Image size requested from Sensor.
    @param [in] mode
		Capture mode for resolution requested.
    @param [in] sensor
		Sensor for which resolution is requested.
    @param [out] *sensor_size
		Actual size of requested resolution.

    @return HAL_CAM_Result_en_t
		status returned.
 */
HAL_CAM_Result_en_t CAMDRV_GetResolution(CamImageSize_t size,
					 CamCaptureMode_t mode,
					 CamSensorSelect_t sensor,
					 HAL_CAM_ResolutionSize_st_t *
					 sensor_size)
{
	HAL_CAM_Result_en_t result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
	UInt16 index, res_size;
	CamResolutionConfigure_t *res_ptr;

	pr_debug("CAMDRV_GetResolution(): size=0x%x, mode=%d, sensor=%d\r\n",
		 size, mode, sensor);

/* Resolution default settings */
	sensor_size->size = CamImageSize_INVALID;
	sensor_size->resX = 0;
	sensor_size->resY = 0;

/* Init Resolution table pointer */
	if (sensor == CamSensorPrimary) {
		res_size =
		    sizeof(sSensorResInfo_HI253_st) /
		    sizeof(CamResolutionConfigure_t);
		res_ptr = &sSensorResInfo_HI253_st[0];
	} else {
		pr_debug("CAMDRV_GetResolution(): ERROR:  Sensor Failed \r\n");
		result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
		return result;
	}

/* Search Resolution table for requested resolution based on capture mode
	(or largest resolution available) */
	for (index = 0; index < res_size; index++) {
		pr_debug
		    ("CAMDRV_GetResolution(): Resolution: size index=%d width=%d \
			height=%d \r\n",
		     index, res_ptr->resX, res_ptr->resY);
		if ((mode == CamCaptureVideo)
		    || (mode == CamCaptureVideonViewFinder)) {
			if (size == (CamImageSize_t) res_ptr->previewIndex) {
				sensor_size->size =
				    (CamImageSize_t) res_ptr->previewIndex;
				sensor_size->resX = res_ptr->resX;
				sensor_size->resY = res_ptr->resY;
				result = HAL_CAM_SUCCESS;
				return result;
			} else if (res_ptr->previewIndex != -1) {
				sensor_size->size =
				    (CamImageSize_t) res_ptr->previewIndex;
				sensor_size->resX = res_ptr->resX;
				sensor_size->resY = res_ptr->resY;
			}
		} else if ((mode == CamCaptureStill)
			   || (mode == CamCaptureStillnThumb)) {
			if (size == (CamImageSize_t) res_ptr->captureIndex) {
				sensor_size->size =
				    (CamImageSize_t) res_ptr->captureIndex;
				sensor_size->resX = res_ptr->resX;
				sensor_size->resY = res_ptr->resY;
				result = HAL_CAM_SUCCESS;
				return result;
			} else if (res_ptr->captureIndex != -1) {
				sensor_size->size =
				    (CamImageSize_t) res_ptr->captureIndex;
				sensor_size->resX = res_ptr->resX;
				sensor_size->resY = res_ptr->resY;
			}
		} else {
			pr_debug
			    ("CAMDRV_GetResolution(): ERROR:  Mode Failed \r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
		}
		/* Increment table pointer */
		res_ptr++;
	}
	pr_debug("CAMDRV_GetResolution(): ERROR:  Resolution Failed \r\n");
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
HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode(CamImageSize_t image_resolution,
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
* Function Name:   HAL_CAM_Result_en_t CAMDRV_SetFrameRate(CamRates_t fps)
*
* Description: This function sets the frame rate of the Camera Sensor
*
* Notes:    15 or 30 fps are supported.
*
****************************************************************************/
HAL_CAM_Result_en_t CAMDRV_SetFrameRate(CamRates_t fps,
					CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
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
			switch(fps)
			{
				case CamRate_5:
				//PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				//fps
				hi253_write(0x98E, 0x6815 	); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x0012 	); //	   = 3
				hi253_write(0x98E, 0x6817 	); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x0012 	); //	   = 3
				hi253_write(0x98E, 0x682D 	); //AE Target FD Zone
				hi253_write(0x990, 0x0012 	); //	   = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0
				break;
				
				case CamRate_10:             
				//PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				//fps
				hi253_write(0x98E, 0x6815 ); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x000a ); //	   = 3
				hi253_write(0x98E, 0x6817 ); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x000a ); //	   = 3
				hi253_write(0x98E, 0x682D ); //AE Target FD Zone
				hi253_write(0x990, 0x000a); //	  = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0

				break;
				
				case CamRate_15:            // 15 fps
				//PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				//fps
				hi253_write(0x98E, 0x6815 ); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x0007 ); //	   = 3
				hi253_write(0x98E, 0x6817 ); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x0007 ); //	   = 3
				hi253_write(0x98E, 0x682D ); //AE Target FD Zone
				hi253_write(0x990, 0x0007); //	  = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0


				break;
				
				case CamRate_20:             
				//PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				//fps
				hi253_write(0x98E, 0x6815 ); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x0005 ); //	   = 3
				hi253_write(0x98E, 0x6817 ); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x0005 ); //	   = 3
				hi253_write(0x98E, 0x682D ); //AE Target FD Zone
				hi253_write(0x990, 0x0005); //	  = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0


				break;
				
				case CamRate_25:           // 25 fps
				//PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				//fps
				hi253_write(0x98E, 0x6815 ); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x0004 ); //	   = 3
				hi253_write(0x98E, 0x6817 ); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x0004 ); //	   = 3
				hi253_write(0x98E, 0x682D ); //AE Target FD Zone
				hi253_write(0x990, 0x0004); //	  = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0


				break;
				
				case CamRate_30:           // 30 fps
				 //PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				 //fps
				hi253_write(0x98E, 0x6815 ); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x0003 ); //	   = 3
				hi253_write(0x98E, 0x6817 ); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x0003 ); //	   = 3
				hi253_write(0x98E, 0x682D ); //AE Target FD Zone
				hi253_write(0x990, 0x0003 ); //	   = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0


				break;
				
				default:                                        // Maximum Clock Rate = 26Mhz
				 //PLL
				hi253_write(0x0010, 0x0C78 ); //PLL Dividers = 3192
				hi253_write(0x0012, 0x0070 ); //PLL P Dividers = 112
				hi253_write(0x002A, 0x7788 ); //PLL P Dividers 4-5-6 = 30600
				 //fps
				hi253_write(0x98E, 0x6815 ); //Max FD Zone 50 Hz
				hi253_write(0x990, 0x0003 ); //	   = 3
				hi253_write(0x98E, 0x6817 ); //Max FD Zone 60 Hz
				hi253_write(0x990, 0x0003 ); //	   = 3
				hi253_write(0x98E, 0x682D ); //AE Target FD Zone
				hi253_write(0x990, 0x0003 ); //	   = 3
				hi253_write(0x098E, 0x8400	); // MCU_ADDRESS
				hi253_write(0x0990, 0x0006	); // MCU_DATA_0
				result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
				 pr_debug("CAMDRV_SetFrameRate(): Error HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
				break;
			}
		}       // else (if (ImageSettingsConfig_st.sensor_framerate->cur_setting == CamRate_Auto))
	}       // else (if (fps <= CamRate_Auto))

	 if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		 pr_debug("CAMDRV_SetFrameRate(): Error[%d] \r\n",
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
/ Notes:
/
****************************************************************************/
HAL_CAM_Result_en_t CAMDRV_EnableVideoCapture(CamSensorSelect_t sensor)
{
	/*[Enable stream] */
	hi253_write(0x001A, 0x0218);
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
HAL_CAM_Result_en_t CAMDRV_SetCamSleep(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	pr_debug("CAMDRV_SetCamSleep(): \r\n");

	/* To be implemented. */
	return result;
}

static void _touchJpegQtable(int qtable, int qscale)
{
	int i;

	/* Quantization table Luma */
	for (i = 0; i < 64; ++i) {
		unsigned int q = (JPEG_StdQuantTblY_ZZ[i] * qscale + 16) >> 5;
		q = (q < 1) ? 1 : (q > 255) ? 255 : q;
		quant_JPEG_Hdr[0][i] = (unsigned char)q;
	}

	/* Quantization table Chroma */
	for (i = 0; i < 64; ++i) {
		unsigned int q = (JPEG_StdQuantTblC_ZZ[i] * qscale + 16) >> 5;
		q = (q < 1) ? 1 : (q > 255) ? 255 : q;
		quant_JPEG_Hdr[1][i] = (unsigned char)q;
	}
}

void CAMDRV_StoreBaseAddress(void *virt_ptr)
{
	jpeg_header_data = virt_ptr;
}

UInt32 CAMDRV_GetJpegSize(CamSensorSelect_t sensor, void *data)
{
	/* In this function, prepare the jpeg, thumbnail data and also
	   compute the JPEG data size. */
	struct DataFormat JpegDataFormat;
	int tn_counter = 0;
	int jpeg_size = 0;
	int tn_size = 0;
	int i = 0;
	int Counter = 0;
	bool marker_found = false;
	int frame_length = 0;
	unsigned char qsel = 0;
	u16 *buffer = (u16 *) data;
	/* pointer to loop thro DMA buffer to find if data is valid and to find the
	   length of the thumbnail + JPEG.
	   Also, to extract thumbnail and JPEG.
	 */
	u8 *tn_ptr = thumbnail_data;	/* pointer to loop for thumbnail data. */
	u8 *write_ptr = data;	/* pointer which will have raw JPEG data. */
	jpeg_raw_data = data;
	/* pointer to store the JPEG raw data.
	   This will be copied to user buffer in JPEG ioctl.
	 */

	/* scan the entire loop to search for sosi and eosi. */
	while (tn_counter <= (1200 * 1360)) {
		u8 *ptr;
		if (*buffer == SOSI) {
			buffer++;
			ptr = (u8 *) buffer;
			for (i = 0; i < 3; i++) {
				frame_length |= *ptr++;
				frame_length <<= 8;
			}
			frame_length |= *ptr++;
			pr_info
			    ("************* FRAME LENGTH 0x%x ***************\n ",
			     frame_length);
			marker_found = true;
			break;
		} else {
			buffer++;
			tn_counter += 2;
		}
	}
	if (!marker_found) {
		pr_info("******** No valid data *******");
		return 0;
	}
	/* extract the thumbnail */
	tn_counter = 0;
	buffer = (u16 *) data;

	while (tn_counter < tn_height) {
		if (*buffer == SOEI) {
			buffer++;	/* remove the start code. */
			for (i = 0; i < tn_width; i++) {
				tn_ptr[(2 * i) + 1] = (buffer[i] & 0x00FF);
				tn_ptr[2 * i] = (buffer[i] & 0xFF00) >> 8;
			}
			tn_counter++;
			/*printk(KERN_INFO"0x%x 0x%x 0x%x 0x%x\n\n",op_buffer[0],
			   op_buffer[1],op_buffer[2],op_buffer[3]); */
			tn_ptr += tn_width * 2;
			buffer += tn_width;
			tn_size += tn_width * 2;
			buffer++;
		} else {
			buffer++;
		}
		if ((*buffer == SOSI) || (tn_size >= tn_width * tn_height * 2))
			break;

	}
	pr_info("Number of extracted thumbnail lines is %d \n", tn_counter);
	pr_info("Thumbnail size is %d\n", tn_size);

	if (tn_counter != tn_height) {
		pr_info("Thumbnail Shorter than expected \n");
		return 0;
	}

	buffer = (u16 *) data;	/*To loop thro JPEG data to extract the jpeg data */
	tn_counter = 0;

	while (tn_counter < frame_length) {
		u8 *copy_ptr;
		if (*buffer == SOEI) {
			buffer++;	/* to skip soei */
			buffer += tn_width;	/* to skip the tn data */
			buffer++;	/* to skip the eoei */
			tn_counter += 4 + (tn_width * 2);
		} else {
			copy_ptr = (u8 *) buffer;
			*write_ptr++ = *copy_ptr++;
			*write_ptr++ = *copy_ptr++;
			jpeg_size += 2;
			buffer++;
			tn_counter += 2;
		}
	}
	/* Now we have the JPEG raw data in contigous memory in write_ptr. */

	*write_ptr++ = 0xFF;
	*write_ptr = 0xD9;

	jpeg_size += 2;
	JpegDataFormat.spoof_width = jpeg_width;
	JpegDataFormat.spoof_heigth = jpeg_height;
	JpegDataFormat.IS_TN_ENABLED = 0;
	JpegDataFormat.CONFIG_JPEG_OB_TX_CONTROL_VAR = 1;
	JpegDataFormat.CONFIG_JPEG_OB_SPOOF_CONTROL_VAR = 0;
	JpegDataFormat.JpegFormat = 2;
	JpegDataFormat.JpegXsize = jpeg_width;	/*2048; */
	JpegDataFormat.JpegYsize = jpeg_height;	/*1536; */
	JpegDataFormat.thumbnail_height = tn_width;
	JpegDataFormat.thumbnail_width = tn_height;
	JpegDataFormat.JpegStatus = 0x80;
	JpegDataFormat.JpegSize = jpeg_size;	/* size of raw JPEG data. */
	qsel = (JpegDataFormat.JpegStatus & 0x600) >> 9;

	/*Store Jpeg Header */
	jpeg_header_data[0] = 0xFF;
	jpeg_header_data[1] = 0xD8;
	Counter = 2;

	for (i = 0; i < JFIFHeaderSize; i++) {
		jpeg_header_data[Counter] = JFIFHeader[i];
		Counter++;
	}

	/*Store Qtable Header */
	for (i = 0; i < QTableHeaderSize; i++) {
		jpeg_header_data[Counter] = QTableHeader[i];
		Counter++;
	}
	/*Store Luma QTable */
	jpeg_header_data[Counter] = 0x00;
	Counter++;

	/*Check which  which Qscale value to use */
	if (qsel == 0) {	/*Use Qscale1 */
		_touchJpegQtable(qsel, 0x08);
	} else {		/*Use Qscale2 */
		_touchJpegQtable(qsel, 0x0C);
	}

	/*Store Q Tables */
	for (i = 0; i < 64; i++) {
		jpeg_header_data[Counter] = quant_JPEG_Hdr[0][i];
		Counter++;
	}

	/*If color then store the Chroma table */
	if (JpegDataFormat.JpegFormat != 0) {
		jpeg_header_data[Counter] = 0x01;
		Counter++;
		for (i = 0; i < 64; i++) {
			jpeg_header_data[Counter] = quant_JPEG_Hdr[1][i];
			Counter++;
		}
	}
	/*STORE HUFFMAN TABLES */
	for (i = 0; i < LumaHuffTableSizeDC; i++) {
		jpeg_header_data[Counter] = LumaHuffTableDC[i];
		Counter++;
	}
	for (i = 0; i < LumaHuffTableSizeAC; i++) {
		jpeg_header_data[Counter] = LumaHuffTableAC[i];
		Counter++;
	}

	/*if color store Chroma */
	if (JpegDataFormat.JpegFormat != 0) {
		for (i = 0; i < ChromaHuffTableSizeDC; i++) {
			jpeg_header_data[Counter] = ChromaHuffTableDC[i];
			Counter++;
		}
		for (i = 0; i < ChromaHuffTableSizeAC; i++) {
			jpeg_header_data[Counter] = ChromaHuffTableAC[i];
			Counter++;
		}
	}

	/*end If */
	/*Store Restart Interval */
	for (i = 0; i < RestartIntervalSize; i++) {
		jpeg_header_data[Counter] = RestartInterval[i];
		Counter++;
	}

	for (i = 0; i < StartOfFrameSize; i++) {
		jpeg_header_data[Counter] = StartOfFrame[i];
		Counter++;
	}

	/*Update Snapshot resolution */
	if (JpegDataFormat.JpegFormat == 0) {
		jpeg_header_data[Counter - 8] = (JpegDataFormat.JpegYsize >> 8);
		jpeg_header_data[Counter - 7] =
		    (JpegDataFormat.JpegYsize & 0xFF);
		jpeg_header_data[Counter - 6] = (JpegDataFormat.JpegXsize >> 8);
		jpeg_header_data[Counter - 5] =
		    (JpegDataFormat.JpegXsize & 0xFF);
	} else {		/*Color SOF */
		jpeg_header_data[Counter - 14] =
		    (JpegDataFormat.JpegYsize >> 8);
		jpeg_header_data[Counter - 13] =
		    (JpegDataFormat.JpegYsize & 0xFF);
		jpeg_header_data[Counter - 12] =
		    (JpegDataFormat.JpegXsize >> 8);
		jpeg_header_data[Counter - 11] =
		    (JpegDataFormat.JpegXsize & 0xFF);
	}

	/*Store Start of Scan */
	if (JpegDataFormat.JpegFormat == 0) {
		for (i = 0; i < StartOfScanSizeM; i++) {
			jpeg_header_data[Counter] = StartOfScanM[i];
			Counter++;
		}
	} else {		/*Color */
		for (i = 0; i < StartOfScanSize; i++) {
			jpeg_header_data[Counter] = StartOfScan[i];
			Counter++;
		}
	}
	pr_info("Counter now is %d", Counter);

	/* Now we have JPEG header data in 1st 1K of DMA memory. The JPEG raw data
	   is afterwards. We have to move the data up to form a contigous memory.
	 */
	memcpy((jpeg_header_data + Counter), jpeg_raw_data,
	       JpegDataFormat.JpegSize);

	/*memmove(op_buffer, (op_buffer + 1024 - Counter),
	   JpegDataFormat.JpegSize); */
	Counter += JpegDataFormat.JpegSize;
	pr_info("Size of JPEG data is %d\n", Counter);
	return Counter;
}

enum {
	STATE_INIT,
	STATE_JPEG,
	STATE_THUMB,
	STATE_STATUS,
	STATE_EXIT
};

UInt16 *CAMDRV_GetJpeg(short *buf)
{
	/*return (tn_buffer + (tn_width * tn_height * 2)); */
	return (UInt16 *) jpeg_header_data;
}

UInt8 *CAMDRV_GetThumbnail(void *buf, UInt32 offset)
{
	/*return (UInt8 *)tn_buffer; give the starting address of
	   1.5MB which contains thumbnail */
	return (UInt8 *) thumbnail_data;
}

HAL_CAM_Result_en_t CAMDRV_DisableCapture(CamSensorSelect_t sensor)
{

	/*[Disable stream] */
	hi253_write(0x001A, 0x0018);

	/*[Preview on] */
	hi253_write(0x098E, 0xEC05);	/* MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN] */
	hi253_write(0x0990, 0x0005);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */

	pr_debug("CAMDRV_DisableCapture(): \r\n");
	return sCamI2cStatus;
}

/****************************************************************************
/
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_DisablePreview(void)
/
/ Description: This function halts HI253 camera video
/
/ Notes:
/
****************************************************************************/
HAL_CAM_Result_en_t CAMDRV_DisablePreview(CamSensorSelect_t sensor)
{
	/* [Disable stream] */
	hi253_write(0x001A, 0x0018);

	pr_debug("CAMDRV_DisablePreview(): \r\n");
	return sCamI2cStatus;
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
HAL_CAM_Result_en_t CAMDRV_CfgStillnThumbCapture(CamImageSize_t
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

	pr_debug("***************************** \
		CAMDRV_CfgStillnThumbCapture():STARTS ************************* \r\n");

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

	switch (thumb_resolution) {

	case CamImageSize_QCIF:

		tx = 176;
		ty = 144;
		break;

	case CamImageSize_QVGA:
		tx = 320;
		ty = 240;
		break;

	case CamImageSize_CIF:
		tx = 352;
		ty = 288;
		break;

	case CamImageSize_VGA:
		tx = 640;
		ty = 480;
		break;

	default:
		tx = 176;
		ty = 144;
		break;
	}

	pr_debug("CAMDRV_CfgStillnThumbCapture():Operating in JPEG mode \r\n");

	tn_width = tx;
	tn_height = ty;
	jpeg_width = x;
	jpeg_height = y;

#if 1
	/***************** We assume we will not use JPEG mode at all.
					Only YCbCr is supported here.****************/
	/* enable for thumbnail */
	/* [Thumbnail tx x ty] */
	hi253_write(0x098E, 0x6C97);	/*MCU_ADDRESS[PRI_B_CONFIG_JPEG_TN_WIDTH] */
	hi253_write(0x0990, tx);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x6C99);	/*MCU_ADDRESS[PRI_B_CONFIG_JPEG_TN_HEIGHT] */
	hi253_write(0x0990, ty);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0005);	/* MCU_DATA_0 */

	/* [Enable Thumbnail] */
	hi253_write(0x098E, 0xEC8E);	/*MCU_ADDRESS[PRI_B_CONFIG_JPEG_JP_MODE] */
	hi253_write(0x0990, 0x0003);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0005);	/* MCU_DATA_0 */

#if 0
	/* [Disable Thumbnail] */
	hi253_write(0x098E, 0xEC8E);	/*MCU_ADDRESS [PRI_B_CONFIG_JPEG_JP_MODE] */
	hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0005);	/* MCU_DATA_0 */
#endif

#endif
	/* [Capture XxY] */
	hi253_write(0x098E, 0x6C00);	/* MCU_ADDRESS [PRI_B_IMAGE_WIDTH] */
	hi253_write(0x0990, x);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x6C02);	/* MCU_ADDRESS [PRI_B_IMAGE_HEIGHT] */
	hi253_write(0x0990, y);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */

	/*JPEG  SPOOF   full height ,only EOI */
	hi253_write(0x098E, 0xEC8E);	/*JPEG (B) */
	hi253_write(0x0990, 0x003);	/*      = 0     0x00 last  06182010 */
	hi253_write(0x098E, 0x6C07);	/* MCU_ADDRESS [PRI_B_OUTPUT_FORMAT] */
	hi253_write(0x0990, 0x0001);	/* MCU_DATA_0  JPEG422 */
	hi253_write(0x098E, 0x6C09);	/*MCU_ADDRESS[PRI_B_OUTPUT_FORMAT_ORDER] */
	hi253_write(0x0990, 0x0000);	/* MCU_DATA_0  JPEG422 */
	hi253_write(0x098E, 0xEC8E);	/*MCU_ADDRESS[PRI_B_CONFIG_JPEG_JP_MODE] */

	hi253_write(0x0990, 0x0003);	/* MCU_DATA_0  JPEG422 */
	/* need to know should we make it 1 or 3 ? vinay */

	hi253_write(0x098E, 0xEC8F);	/* MCU_ADDRESS [PRI_B_CONFIG_JPEG_FORMAT] */
	hi253_write(0x0990, 0x0002);	/* MCU_DATA_0  JPEG422 */
	/* MCU_ADDRESS[PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR] */
	hi253_write(0x098E, 0x6CA0);
	hi253_write(0x0990, 0x082D);	/* MCU_DATA_0  spoof full height */
#if 1
	/* MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_CONTROL_VAR] */
	hi253_write(0x098E, 0xEC9F);
	hi253_write(0x0990, 0x0001);	/* MCU_DATA_0  ignore spoof full height */
#endif
	hi253_write(0x098E, 0x6C90);	/* MCU_ADDRESS [PRI_B_CONFIG_JPEG_CONFIG] */
	hi253_write(0x0990, 0x8434);	/* MCU_DATA_0  EOI only */
	/* MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_WIDTH_VAR] */
	hi253_write(0x098E, 0x6C9B);
	/* MCU_DATA_0  1000 SPOOF size */
	hi253_write(0x0990, CamPrimaryCfg_Jpeg_st.jpeg_packet_size_bytes);
	/* MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_HEIGHT_VAR] */
	hi253_write(0x098E, 0x6C9D);
	/* MCU_DATA_0  900    SPOOF size */
	hi253_write(0x0990, CamPrimaryCfg_Jpeg_st.jpeg_max_packets);
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */

	/* Jpeg422 */
	/* [Capture on] */
	hi253_write(0x098E, 0xEC05);	/* MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN] */
	hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0xEC05);	/* MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN] */
	hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
	hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
	hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */

	/* [Enable stream] */
	hi253_write(0x001A, 0x0218);

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug
		    ("CAMDRV_CfgStillnThumbCapture():Error sending capture mode \r\n");
		result = sCamI2cStatus;
	}

	pr_debug("CAMDRV_CfgStillnThumbCapture(): stills_resolution = 0x%x, \
		 stills_format=0x%x \r\n", stills_resolution, stills_format);
	pr_debug("CAMDRV_CfgStillnThumbCapture(): thumb_resolution = 0x%x, \
		 thumb_format=0x%x \r\n", thumb_resolution, thumb_format);

	/* msleep(5000); */

	return result;
}

/****************************************************************************
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetSceneMode(
/					CamSceneMode_t scene_mode)
/
/ Description: This function will set the scene mode of camera
/ Notes:
****************************************************************************/

HAL_CAM_Result_en_t CAMDRV_SetSceneMode(CamSceneMode_t scene_mode,
					CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	switch(scene_mode) {
		case CamSceneMode_Auto:
			pr_info("CAMDRV_SetSceneMode() called for AUTO\n");
			hi253_write(0x098E, 0x6815);	// MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_50HZ]
			hi253_write(0x0990, 0x006);   // MCU_DATA_0
			hi253_write(0x098E, 0x6817);  // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_60HZ]
			hi253_write(0x0990, 0x006);   // MCU_DATA_0
			hi253_write(0x098E, 0x682D); 	// MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_FDZONE]
			hi253_write(0x0990, 0x0003);  // MCU_DATA_0
			hi253_write(0x098E, 0x682F);  // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_AGAIN]
			hi253_write(0x0990, 0x0100);  // MCU_DATA_0
			hi253_write(0x098E, 0x6839);  // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_AGAIN]
			hi253_write(0x0990, 0x012C);  // MCU_DATA_0
			hi253_write(0x098E, 0x6835);  // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_DGAIN]
			hi253_write(0x0990, 0x00F0);  // MCU_DATA_0
			break;
		case CamSceneMode_Night:
			pr_info("CAMDRV_SetSceneMode() called for Night\n");
			hi253_write(0x098E, 0x6815);	// MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_50HZ]
			hi253_write(0x0990, 0x0018);  // MCU_DATA_0
			hi253_write(0x098E, 0x6817);  // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_60HZ]
			hi253_write(0x0990, 0x0018);  // MCU_DATA_0
			hi253_write(0x098E, 0x682D); 	// MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_FDZONE]
			hi253_write(0x0990, 0x0006);  // MCU_DATA_0
			hi253_write(0x098E, 0x682F);  // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_AGAIN]
			hi253_write(0x0990, 0x0100);  // MCU_DATA_0
			hi253_write(0x098E, 0x6839);  // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_AGAIN]
			hi253_write(0x0990, 0x012C);  // MCU_DATA_0
			hi253_write(0x098E, 0x6835);  // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_DGAIN]
			hi253_write(0x0990, 0x00F0);  // MCU_DATA_0
			break;
		default:
			pr_info("CAMDRV_SetSceneMode() not supported for %d\n", scene_mode);
			break;
	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_SetSceneMode(): Error[%d] \r\n",
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

HAL_CAM_Result_en_t CAMDRV_SetWBMode(CamWB_WBMode_t wb_mode,
				     CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetWBMode()  called\n");
	if (wb_mode == CamWB_Auto) {

		hi253_write(0x098E, 0x8002);	/* MCU_ADDRESS [MON_MODE] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x007F);
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x098E, 0xE84A);
		hi253_write(0x0990, 0x0080);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x098E, 0xE84C);
		hi253_write(0x0990, 0x0080);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x098E, 0xE84D);
		hi253_write(0x0990, 0x0078);	/* 85   MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x098E, 0xE84F);
		hi253_write(0x0990, 0x007E);	/* 81    MCU_DATA_0 */

	} else if (wb_mode == CamWB_Incandescent) {

		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x0003);
		hi253_write(0x098E, 0xAC33);	/* MCU_ADDRESS [AWB_CCMPOSITION] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x098E, 0xE84A);
		hi253_write(0x0990, 0x0079);	/* 6C   MCU_DATA_0  */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x098E, 0xE84C);
		hi253_write(0x0990, 0x00DB);	/* 9B   MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x098E, 0xE84D);
		hi253_write(0x0990, 0x0079);	/* 6C   MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x098E, 0xE84F);
		hi253_write(0x0990, 0x00DB);	/* 9B   MCU_DATA_0 */

	} else if ((wb_mode == CamWB_DaylightFluorescent)
		   || (wb_mode == CamWB_WarmFluorescent)) {

		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x0003);
		hi253_write(0x098E, 0xAC33);	/* MCU_ADDRESS [AWB_CCMPOSITION] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x098E, 0xE84A);
		hi253_write(0x0990, 0x0075);	/* 79 6C      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x098E, 0xE84C);
		hi253_write(0x0990, 0x0099);	/* DB 9B      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x098E, 0xE84D);
		hi253_write(0x0990, 0x0075);	/* 79 6C      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x098E, 0xE84F);
		hi253_write(0x0990, 0x0099);	/* DB 9B      MCU_DATA_0 */

	} else if (wb_mode == CamWB_Daylight) {

		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x0003);
		hi253_write(0x098E, 0xAC33);	/* MCU_ADDRESS [AWB_CCMPOSITION] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x098E, 0xE84A);
		hi253_write(0x0990, 0x008E);	/* 79 6C      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x098E, 0xE84C);
		hi253_write(0x0990, 0x005C);	/* DB 9B      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x098E, 0xE84D);
		hi253_write(0x0990, 0x008E);	/* 79 6C      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x098E, 0xE84F);
		hi253_write(0x0990, 0x005C);	/* DB 9B      MCU_DATA_0 */

	} else if (wb_mode == CamWB_Cloudy) {

		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x0003);
		hi253_write(0x098E, 0xAC33);	/* MCU_ADDRESS [AWB_CCMPOSITION] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x098E, 0xE84A);
		hi253_write(0x0990, 0x0098);	/* 79 6C      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x098E, 0xE84C);
		hi253_write(0x0990, 0x004C);	/* DB 9B      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x098E, 0xE84D);
		hi253_write(0x0990, 0x0098);	/* 79 6C      MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x098E, 0xE84F);
		hi253_write(0x0990, 0x004C);	/* DB 9B      MCU_DATA_0 */

	} else if (wb_mode == CamWB_Twilight) {

		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0057);	/* 6F */
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x0059);	/* 6F */
		hi253_write(0x098E, 0xAC33);	/* MCU_ADDRESS [AWB_CCMPOSITION] */
		hi253_write(0x0990, 0x0058);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84A);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x0990, 0x009F);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84B);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_G_L] */
		hi253_write(0x0990, 0x0083);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84C);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x0990, 0x0004);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84D);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x0990, 0x00A5);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84E);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_G_R] */
		hi253_write(0x0990, 0x005E);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84F);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x0990, 0x0083);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xEC4A);	/*MCU_ADDRESS[PRI_B_CONFIG_AWB_K_R_L] */
		hi253_write(0x0990, 0x009F);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xEC4B);	/*MCU_ADDRESS[PRI_B_CONFIG_AWB_K_G_L] */
		hi253_write(0x0990, 0x0083);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xEC4C);	/*MCU_ADDRESS[PRI_B_CONFIG_AWB_K_B_L] */
		hi253_write(0x0990, 0x0004);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xEC4D);	/*MCU_ADDRESS[PRI_B_CONFIG_AWB_K_R_R] */
		hi253_write(0x0990, 0x00A5);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xEC4E);	/*MCU_ADDRESS[PRI_B_CONFIG_AWB_K_G_R] */
		hi253_write(0x0990, 0x005E);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xEC4F);	/*MCU_ADDRESS[PRI_B_CONFIG_AWB_K_B_R] */
		hi253_write(0x0990, 0x0083);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */
	} else {
		printk("Am here in wb:%d\n", wb_mode);
		hi253_write(0x098E, 0x8002);	/* MCU_ADDRESS [MON_MODE] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xC8F2);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xC8F3);
		hi253_write(0x0990, 0x007F);
		hi253_write(0x098E, 0xE84A);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_R_L] */
		hi253_write(0x0990, 0x0080);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84C);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_B_L] */
		hi253_write(0x0990, 0x0080);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xE84D);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_R_R] */
		hi253_write(0x0990, 0x0078);	/* 85   MCU_DATA_0 */
		hi253_write(0x098E, 0xE84F);	/*MCU_ADDRESS[PRI_A_CONFIG_AWB_K_B_R] */
		hi253_write(0x0990, 0x007E);	/* 81   MCU_DATA_0 */
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_SetWBMode(): Error[%d] \r\n", sCamI2cStatus);
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

HAL_CAM_Result_en_t CAMDRV_SetAntiBanding(CamAntiBanding_t effect,
					  CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetAntiBanding()  called\n");
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) {
		hi253_write(0x098E, 0xA005);	/* MCU_ADDRESS [FD_FDPERIOD_SELECT] */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_B_CONFIG_FD_ALGO_RUN] */
		hi253_write(0x098E, 0x6C11);
		hi253_write(0x0990, 0x0003);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_FD_ALGO_RUN] */
		hi253_write(0x098E, 0x6811);
		hi253_write(0x0990, 0x0003);	/* MCU_DATA_0 */

	} else if (effect == CamAntiBanding50Hz) {

		hi253_write(0x098E, 0xA005);	/* MCU_ADDRESS [FD_FDPERIOD_SELECT] */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 =>'0'=60Hz, '1'=50Hz */
		hi253_write(0x098E, 0x6C11);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_FD_ALGO_RUN] */
		hi253_write(0x098E, 0x6811);
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */

	} else if (effect == CamAntiBanding60Hz) {

		hi253_write(0x098E, 0xA005);	/* MCU_ADDRESS [FD_FDPERIOD_SELECT] */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 =>'0'=60Hz, '1'=50Hz */
		hi253_write(0x098E, 0x6C11);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_FD_ALGO_RUN] */
		hi253_write(0x098E, 0x6811);
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */

	} else {
		hi253_write(0x098E, 0xA005);	/* MCU_ADDRESS [FD_FDPERIOD_SELECT] */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_B_CONFIG_FD_ALGO_RUN] */
		hi253_write(0x098E, 0x6C11);
		hi253_write(0x0990, 0x0003);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_FD_ALGO_RUN] */
		hi253_write(0x098E, 0x6811);
		hi253_write(0x0990, 0x0003);	/* MCU_DATA_0 */
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_SetAntiBanding(): Error[%d] \r\n",
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

HAL_CAM_Result_en_t CAMDRV_SetFlashMode(FlashLedState_t effect,
					CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetFlashMode()  called\n");
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
		pr_debug("CAMDRV_SetFlashMode(): Error[%d] \r\n",
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

HAL_CAM_Result_en_t CAMDRV_SetFocusMode(CamFocusControlMode_t effect,
					CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetFocusMode()  called\n");
	if (effect == CamFocusControlAuto) {

		hi253_write(0x098E, 0xB03A);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0032);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB04A);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0064);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB048);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB041);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x002A);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB042);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0004);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB043);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0014);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB044);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB045);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0014);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB046);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x3003);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0010);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB019);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */

	} else if (effect == CamFocusControlMacro) {

		hi253_write(0x098E, 0x3003);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB024);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0000);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x3003);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* AF_ALGO */
		hi253_write(0x098E, 0xB019);	/* MCU_ADDRESS [AF_PROGRESS] */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */

	} else if (effect == CamFocusControlInfinity) {

		hi253_write(0x098E, 0x3003);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB024);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0xFFFF);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x3003);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* AF_ALGO */
		hi253_write(0x098E, 0xB019);	/* MCU_ADDRESS [AF_PROGRESS] */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */

	} else {

		hi253_write(0x098E, 0xB03A);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0032);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB04A);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0064);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB048);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB041);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x002A);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB042);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0004);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB043);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0014);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB044);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB045);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0014);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB046);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x3003);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0010);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0xB019);	/* MCU_ADDRESS */
		hi253_write(0x0990, 0x0001);	/* MCU_DATA_0 */

	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_SetFocusMode(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}

HAL_CAM_Result_en_t CAMDRV_TurnOffAF()
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_TurnOffAF() called\n");
	hi253_write(0x0604, 0x0F00);    /* MCU_ADDRESS */
	hi253_write(0x0606, 0x0F00);    /* MCU_DATA_0 */

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_TurnOffAF(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}

HAL_CAM_Result_en_t CAMDRV_TurnOnAF()
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	/* AF DriverIC power enable */
	pr_info("CAMDRV_TurnOnAF() called\n");
	hi253_write(0x0604, 0x0F01);    /* MCU_ADDRESS */
	hi253_write(0x0606, 0x0F00);    /* MCU_DATA_0 */

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_TurnOnAF(): Error[%d] \r\n",
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
HAL_CAM_Result_en_t CAMDRV_SetJpegQuality(CamJpegQuality_t effect,
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
		pr_debug("CAMDRV_SetJpegQuality(): Error[%d] \r\n",
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
HAL_CAM_Result_en_t CAMDRV_SetZoom(CamZoom_t step,
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
		pr_debug("CAMDRV_SetZoom(): Error[%d] \r\n",
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
HAL_CAM_Result_en_t CAMDRV_SetDigitalEffect(CamDigEffect_t effect,
					    CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	if (effect == CamDigEffect_NoEffect) {

		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0x8400);
		hi253_write(0x0990, 0x0006);

	} else if (effect == CamDigEffect_MonoChrome) {

		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0001);
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0001);
		hi253_write(0x098E, 0x8400);
		hi253_write(0x0990, 0x0006);

	} else if ((effect == CamDigEffect_NegColor)
		   || (effect == CamDigEffect_NegMono)) {
		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0003);
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0003);
		hi253_write(0x098E, 0x8400);
		hi253_write(0x0990, 0x0006);

	} else if ((effect == CamDigEffect_SolarizeColor)
		   || (effect == CamDigEffect_SolarizeMono)) {
		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0004);
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0004);
		hi253_write(0x098E, 0x8400);
		hi253_write(0x0990, 0x0006);

	} else if (effect == CamDigEffect_SepiaGreen) {

		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0002);
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0002);
		hi253_write(0x098E, 0xE885);
		hi253_write(0x0990, 0x0037);
		hi253_write(0x098E, 0xEC85);
		hi253_write(0x0990, 0x0037);
		hi253_write(0x098E, 0xE886);
		hi253_write(0x0990, 0x00BE);
		hi253_write(0x098E, 0xEC86);
		hi253_write(0x0990, 0x00BE);
		hi253_write(0x098E, 0x8400);
		hi253_write(0x0990, 0x0006);

	} else if (effect == CamDigEffect_Auqa) {

		/* MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX] */
		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX] */
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0002);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR] */
		hi253_write(0x098E, 0xE885);
		hi253_write(0x0990, 0x008C);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR] */
		hi253_write(0x098E, 0xEC85);
		hi253_write(0x0990, 0x008C);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB] */
		hi253_write(0x098E, 0xE886);
		hi253_write(0x0990, 0x0042);	/* MCU_DATA_0 */
		/* MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CB] */
		hi253_write(0x098E, 0xEC86);
		hi253_write(0x0990, 0x0042);	/* MCU_DATA_0 */
		hi253_write(0x098E, 0x8400);	/* MCU_ADDRESS [SEQ_CMD] */
		hi253_write(0x0990, 0x0006);	/* MCU_DATA_0 */

	} else {
		hi253_write(0x098E, 0xE883);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0xEC83);
		hi253_write(0x0990, 0x0000);
		hi253_write(0x098E, 0x8400);
		hi253_write(0x0990, 0x0006);

	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		pr_debug("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

/**
*  This function will perform specific action from defined list,
*  copy of parameters passed thru parm structure.
*
*/
HAL_CAM_Result_en_t CAMDRV_ActionCtrl(HAL_CAM_Action_en_t action,
				      void *data, void *callback)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	pr_debug("HAL_CAMDRV_Ctrl():  action=%d \r\n", action);

	switch (action) {
#if 0
/* Operation Control Settings */
		/* < Get Jpeg Max size (bytes),use HAL_CAM_Action_param_st_t to get */
	case ACTION_CAM_GetJpegMaxSize:
		result =
		    CAMDRV_GetJpegMaxSize(((HAL_CAM_Action_param_st_t *) data)->
					  param,
					  ((HAL_CAM_Action_param_st_t *) data)->
					  sensor);
		break;

		/* < Get closest sensor resolution from passed pixel width and height,
		   use HAL_CAM_Action_image_size_st_t to get CamImageSize_t */
	case ACTION_CAM_GetImageSize:
		result =
		    CAMDRV_GetResolutionAvailable(((HAL_CAM_Action_image_size_st_t *)
						   data)->width,
						  ((HAL_CAM_Action_image_size_st_t *)
						   data)->height,
						  ((HAL_CAM_Action_image_size_st_t *)
						   data)->mode,
						  ((HAL_CAM_Action_image_size_st_t *) data)->sensor, ((HAL_CAM_Action_image_size_st_t *) data)->sensor_size);
		break;
		/* < Get matching image size from passed pixel width and height,
		   use HAL_CAM_Action_image_size_st_t to get CamImageSize_t */
	case ACTION_CAM_GetSensorResolution:
		result =
		    CAMDRV_GetResolution(((HAL_CAM_Action_resolution_st_t *)
					  data)->size,
					 ((HAL_CAM_Action_resolution_st_t *)
					  data)->mode,
					 ((HAL_CAM_Action_resolution_st_t *)
					  data)->sensor,
					 ((HAL_CAM_Action_resolution_st_t *)
					  data)->sensor_size);
		break;
		/* < Set required picture frame, use (CamRates_t)
		   HAL_CAM_Action_param_st_t to set */
	case ACTION_CAM_SetFrameRate:
		result = CAMDRV_SetFrameRate((CamRates_t)
					     (((HAL_CAM_Action_param_st_t *)
					       data)->param),
					     ((HAL_CAM_Action_param_st_t *)
					      data)->sensor);
		break;
#endif
/* Unsupported Actions in Camera Device Driver
	Return:  HAL_CAM_ERROR_ACTION_NOT_SUPPORTED */
	default:
		pr_debug("HAL_CAMDRV_Ctrl(): Invalid Action \r\n");
		result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
		break;
	}
	return result;
}

ktime_t tm1;

static HAL_CAM_Result_en_t Init_Hi253(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	ktime_t tm1;

	printk("hanwei:***** HI253 Init_HI253 *****\r\n");
	
	int timeout;
	tm1 = ktime_get();
	pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	CamSensorCfg_st.sensor_config_caps = &CamPrimaryCfgCap_st;

	if (checkCameraID(sensor)) {
		return HAL_CAM_ERROR_INTERNAL_ERROR;
	}

	//HI253
	hi253_write(0x01, 0x79); //sleep on
	hi253_write(0x08, 0x0f); //Hi-Z on
	hi253_write(0x01, 0x78); //sleep off

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
	hi253_write(0x01, 0x71); //sleep on
	hi253_write(0x08, 0x00); //Hi-Z off

	hi253_write(0x01, 0x73);
	hi253_write(0x01, 0x71);

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
	hi253_write(0x11, 0x90);
	hi253_write(0x12, 0x00);

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
	hi253_write(0x90, 0x0c); //jerry 8-29 debug
	hi253_write(0x91, 0x0c); //jerry 8-29 debug
	hi253_write(0x92, 0xd8); //jerry 8-29 debug
	hi253_write(0x93, 0xd0); //jerry 8-29 debug
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
		
	hi253_write(0xd4, 0x0c);		
	hi253_write(0xd5, 0x0c);		
	hi253_write(0xd6, 0xd8);//jerry 8-29 debug
	hi253_write(0xd7, 0xd0);//jerry 8-29 debug
			
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
	hi253_write(0x10, 0x03); // CrYCbY // For Demoset 0x03
	hi253_write(0x12, 0x30);
	hi253_write(0x13, 0x0a); // contrast on
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

	hi253_write(0x40, 0x80); 
	hi253_write(0x41, 0x00); 
	hi253_write(0x48, 0x80); // Contrast

	hi253_write(0x60, 0x67);
	hi253_write(0x61, 0x90);
	hi253_write(0x62, 0x80); 
	hi253_write(0x63, 0x50); //Double_AG 50->30
	hi253_write(0x64, 0x41);

	hi253_write(0x66, 0x42);
	hi253_write(0x67, 0x20);
 
	hi253_write(0x6a, 0x80);
	hi253_write(0x6b, 0x84);
	hi253_write(0x6c, 0x80);
	hi253_write(0x6d, 0x80);

	//Don't touch//////////////////////////
	//(0x72, 0x84);
	//(0x76, 0x19);
	//(0x73, 0x70);
	//(0x74, 0x68);
	//(0x75, 0x60); // white protection ON
	//(0x77, 0x0e); //08 //0a
	//(0x78, 0x2a); //20
	//(0x79, 0x08);
	////////////////////////////////////////

	/////// PAGE 11 START ///////
	hi253_write(0x03, 0x11);
	hi253_write(0x10, 0x6f);	//0x7f);
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
	hi253_write(0x3c, 0x80);
	hi253_write(0x3d, 0x18);
	hi253_write(0x3e, 0xa0);//a0 jerry debug
 	hi253_write(0x3f, 0x0c);//0c jerry debug
	hi253_write(0x40, 0x09);//09 jerry debug
	hi253_write(0x41, 0x06);//06 jerry debug

	hi253_write(0x42, 0x80);
	hi253_write(0x43, 0x18);
	hi253_write(0x44, 0xa0); //80
	hi253_write(0x45, 0x12);
	hi253_write(0x46, 0x10);
	hi253_write(0x47, 0x10);

	hi253_write(0x48, 0x90);
	hi253_write(0x49, 0x40);
	hi253_write(0x4a, 0x80);
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

	hi253_write(0x5a,0x10);	// 0x1f); //18
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
	hi253_write(0x51, 0x80);// jerry debug
	//hi253_write(0x4e, 0xd0);
	//hi253_write(0x4f, 0xd0);
	//hi253_write(0x50, 0xd0);
	//hi253_write(0x51, 0xd0);

	//Dark1 th
	hi253_write(0x52, 0xb0);
	hi253_write(0x53, 0x60);
	hi253_write(0x54, 0xc0);
	hi253_write(0x55, 0xc0);
	hi253_write(0x56, 0xc0);
	hi253_write(0x57, 0x80);

	//Dark2 th
	hi253_write(0x58, 0x90);
	hi253_write(0x59, 0x40);
	hi253_write(0x5a, 0xd0);
	hi253_write(0x5b, 0xd0);
	hi253_write(0x5c, 0xe0);
	hi253_write(0x5d, 0x80);

	//Dark3 th
	hi253_write(0x5e, 0x88);
	hi253_write(0x5f, 0x40);
	hi253_write(0x60, 0xe0);
	hi253_write(0x61, 0xe0);
	hi253_write(0x62, 0xe0);
	hi253_write(0x63, 0x80);

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
	//(0xB6,read); only//dpc_flat_thres
	//(0xB7,read); only//dpc_grad_cnt
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
	hi253_write(0x10, 0xca);//cb jerry debug
	hi253_write(0x11, 0x7b);
	hi253_write(0x12, 0x03);//07 jerry debug
	hi253_write(0x14, 0x00);

	hi253_write(0x20, 0x15);
	hi253_write(0x21, 0x13);
	hi253_write(0x22, 0x33);
	hi253_write(0x23, 0x05);
	hi253_write(0x24, 0x09);

	hi253_write(0x25, 0x0a);

	hi253_write(0x26, 0x18);
	hi253_write(0x27, 0x30);
	hi253_write(0x29, 0x12);
	hi253_write(0x2a, 0x50);

	//Low clip th
	hi253_write(0x2b, 0x00); //Out2 02
	hi253_write(0x2c, 0x00); //Out1 02 //01
	hi253_write(0x25, 0x06);
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
	hi253_write(0x80, 0xfc);//fd jerry debug
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

	hi253_write(0x20, 0x60); //X 60 //a0
	hi253_write(0x21, 0x80); //Y

	hi253_write(0x22, 0x80);
	hi253_write(0x23, 0x80);
	hi253_write(0x24, 0x80);

	hi253_write(0x30, 0xc8);
	hi253_write(0x31, 0x2b);
	hi253_write(0x32, 0x00);
	hi253_write(0x33, 0x00);
	hi253_write(0x34, 0x90);

	//hi253_write(0x40, 0x4a); 
	//hi253_write(0x50, 0x43);	
	//hi253_write(0x60, 0x43);
	//hi253_write(0x70, 0x43);

	hi253_write(0x40, 0x48); 
	hi253_write(0x50, 0x34);	
	hi253_write(0x60, 0x2b);
	hi253_write(0x70, 0x34);

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

	//(0x40, 0x93);
	//(0x41, 0x1c);
	//(0x42, 0x89);
	//(0x43, 0x82);
	//(0x44, 0x01);
	//(0x45, 0x01);
	//(0x46, 0x8a);
	//(0x47, 0x9d);
	//(0x48, 0x28);

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
	hi253_write(0x71, 0x07);	// 0x08);
	hi253_write(0x72, 0x0c);
	hi253_write(0x73, 0x18);
	hi253_write(0x74, 0x31);
	hi253_write(0x75, 0x4d);
	hi253_write(0x76, 0x69);
	hi253_write(0x77, 0x83);
	hi253_write(0x78, 0x9b);
	hi253_write(0x79, 0xb1);
	hi253_write(0x7a, 0xc3);
	hi253_write(0x7b, 0xd2);
	hi253_write(0x7c, 0xde);
	hi253_write(0x7d, 0xe8);
	hi253_write(0x7e, 0xf0);
	hi253_write(0x7f, 0xf5);
	hi253_write(0x80, 0xfa);
	hi253_write(0x81, 0xfd);
	hi253_write(0x82, 0xff);

	/////// PAGE 17 START ///////
	hi253_write(0x03, 0x17);
	hi253_write(0x10, 0xf7);

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
	hi253_write(0x2b, 0x04); //f4->Adaptive off

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

	//New Weight For Samsung
	#if 1
	hi253_write(0x60, 0x00);
	hi253_write(0x61, 0x00);
	hi253_write(0x62, 0x00);
	hi253_write(0x63, 0x00);
	hi253_write(0x64, 0x0f);
	hi253_write(0x65, 0xf0);
	hi253_write(0x66, 0x0f);
	hi253_write(0x67, 0xf0);
	hi253_write(0x68, 0x0f);
	hi253_write(0x69, 0xf0);
	hi253_write(0x6a, 0x0f);
	hi253_write(0x6b, 0xf0);
	hi253_write(0x6c, 0x00);
	hi253_write(0x6d, 0x00);
	hi253_write(0x6e, 0x00);
	hi253_write(0x6f, 0x00);

	#else
	hi253_write(0x60, 0xaa);
	hi253_write(0x61, 0xaa);
	hi253_write(0x62, 0xaa);
	hi253_write(0x63, 0xaa);
	hi253_write(0x64, 0xaa);
	hi253_write(0x65, 0xaa);
	hi253_write(0x66, 0xab);
	hi253_write(0x67, 0xEa);
	hi253_write(0x68, 0xab);
	hi253_write(0x69, 0xEa);
	hi253_write(0x6a, 0xaa);
	hi253_write(0x6b, 0xaa);
	hi253_write(0x6c, 0xaa);
	hi253_write(0x6d, 0xaa);
	hi253_write(0x6e, 0xaa);
	hi253_write(0x6f, 0xaa);
	#endif
/*
	hi253_write(0x60, 0x55); // AEWGT1
	hi253_write(0x61, 0x55); // AEWGT2
	hi253_write(0x62, 0x6a); // AEWGT3
	hi253_write(0x63, 0xa9); // AEWGT4
	hi253_write(0x64, 0x6a); // AEWGT5
	hi253_write(0x65, 0xa9); // AEWGT6
	hi253_write(0x66, 0x6a); // AEWGT7
	hi253_write(0x67, 0xa9); // AEWGT8
	hi253_write(0x68, 0x6b); // AEWGT9
	hi253_write(0x69, 0xe9); // AEWGT10
	hi253_write(0x6a, 0x6a); // AEWGT11
	hi253_write(0x6b, 0xa9); // AEWGT12
	hi253_write(0x6c, 0x6a); // AEWGT13
	hi253_write(0x6d, 0xa9); // AEWGT14
	hi253_write(0x6e, 0x55); // AEWGT15
	hi253_write(0x6f, 0x55); // AEWGT16
*/
	//hanwei@wind-mobi.com for camera watermark correction 20111014 begin
	/*
	hi253_write(0x70, 0x86);//7e //shuiyin test jerry debug 2011-10-11
	hi253_write(0x71, 0x00); //82(+8)->+0
	*/
	hi253_write(0x70, 0x76);//7e //shuiyin test jerry debug 2011-10-11
	hi253_write(0x71, 0x00);
	// haunting control
	//hanwei@wind-mobi.com for camera watermark correction 20111014 begin
	/*
	hi253_write(0x76, 0x53);//53//shuiyin test jerry debug 2011-10-11
	hi253_write(0x77, 0x05);//05 //shuiyin test jerry debug 2011-10-11
	hi253_write(0x78, 0x24); //24//shuiyin test jerry debug 2011-10-11
	hi253_write(0x79, 0x50); //4 //4B//shuiyin test jerry debug 2011-10-11
	*/
	hi253_write(0x76, 0x43);	
	hi253_write(0x77, 0x04);	
	hi253_write(0x78, 0x23); 	
	hi253_write(0x79, 0x46);
	//hanwei@wind-mobi.com for camera watermark correction 20111014 end
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

	//hanwei@wind-mobi.com for camera watermark correction 20111014 begin
	//hi253_write(0x9c, 0x1c); // 1a//shuiyin test jerry debug 2011-10-11
	hi253_write(0x9c, 0x18); 
	//hanwei@wind-mobi.com for camera watermark correction 20111014 end
	hi253_write(0x9d, 0x00); 
	hi253_write(0x9e, 0x02); //EXP Unit 
	hi253_write(0x9f, 0x00); 

	//AE_Middle Time option
	//(0xa0, 0x03);
	//(0xa1, 0xa9);
	//(0xa2, 0x80);

	hi253_write(0xb0, 0x18);
	hi253_write(0xb1, 0x14); //ADC 400->560
	hi253_write(0xb2, 0xe0); 
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
	hi253_write(0xc0, 0x14);
	hi253_write(0xc1, 0x1f);
	hi253_write(0xc2, 0x1f);
	hi253_write(0xc3, 0x18);
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
	//(0x25, 0x00); //7f New Lock Cond & New light stable

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
	hi253_write(0x82, 0x3e);

	hi253_write(0x83, 0x5e); //5e
	hi253_write(0x84, 0x1e); //24
	hi253_write(0x85, 0x5e); //54 //56 //5a
	hi253_write(0x86, 0x22); 

	hi253_write(0x87, 0x49);
	hi253_write(0x88, 0x39);
	hi253_write(0x89, 0x37); //38
	hi253_write(0x8a, 0x28); //2a

	hi253_write(0x8b, 0x41); //47
	hi253_write(0x8c, 0x39); 
	hi253_write(0x8d, 0x34); 
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
	hi253_write(0x01, 0xc0); // Sleep Off 0xf8->0x50 for solve green line issue

	hi253_write(0xff, 0xff);
	hi253_write(0xff, 0xff);

	tm1 = ktime_get();
	pr_info("Exit Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	return result;

}

struct sens_methods sens_meth = {
    DRV_GetIntfConfig: CAMDRV_GetIntfConfig,
    DRV_GetIntfSeqSel : CAMDRV_GetIntfSeqSel,
    DRV_Wakeup : CAMDRV_Wakeup,
    DRV_GetResolution : CAMDRV_GetResolution,
    DRV_SetVideoCaptureMode : CAMDRV_SetVideoCaptureMode,
    DRV_SetFrameRate : CAMDRV_SetFrameRate,
    DRV_EnableVideoCapture : CAMDRV_EnableVideoCapture,
    DRV_SetCamSleep : CAMDRV_SetCamSleep,
    DRV_GetJpegSize : CAMDRV_GetJpegSize,
    DRV_GetJpeg : CAMDRV_GetJpeg,
    DRV_GetThumbnail : CAMDRV_GetThumbnail,
    DRV_DisableCapture : CAMDRV_DisableCapture,
    DRV_DisablePreview : CAMDRV_DisablePreview,
    DRV_CfgStillnThumbCapture : CAMDRV_CfgStillnThumbCapture,
    DRV_StoreBaseAddress : CAMDRV_StoreBaseAddress,
    DRV_SetSceneMode : CAMDRV_SetSceneMode,
    DRV_SetWBMode : CAMDRV_SetWBMode,
    DRV_SetAntiBanding : CAMDRV_SetAntiBanding,
    DRV_SetFocusMode : CAMDRV_SetFocusMode,
    DRV_SetDigitalEffect : CAMDRV_SetDigitalEffect,
    DRV_SetFlashMode : CAMDRV_SetFlashMode,
    DRV_SetJpegQuality : CAMDRV_SetJpegQuality,
    DRV_TurnOnAF : CAMDRV_TurnOnAF,
    DRV_TurnOffAF : CAMDRV_TurnOffAF,
	DRV_SetZoom : CAMDRV_SetZoom,

};

struct sens_methods *CAMDRV_primary_get(void)
{
	return &sens_meth;
}
