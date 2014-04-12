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
*   @file   camdrv_ov7692.c
*
*   @brief  This file is the lower level driver API of ov7692 sensor.
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

#define OV7692_ID 0x7692 
UInt32 temp;

//hanwei@wind-mobi.com 2011.11.02 Device id address begin
#define DeviceId_Addr_H		0x300A		//high	0A
#define DeviceId_Addr_L	       0x300B		//low	0B
//hanwei@wind-mobi.com 2011.11.02 Device id address end

#define HAL_SECOND_CAM_RESET 56  /*it should be low when using second sensor*/

//hanwei@wind-mobi.com 2011.11.02 second camera power down pin begin
#define HAL_Second_CAM_PWDN  9   

//hanwei@wind-mobi.com 2011.11.02 second camera power down pin end

typedef struct{
	UInt8 SubAddr;
	UInt8 Value;
}OV7692Reg_t;


/*---------Sensor Power On */
static CamSensorIntfCntrl_st_t CamPowerOnSeq[] = 
{
//hanwei@wind-mobi.com add 2011.11.04 start
//reviewed by liubing@wind-mobi.com
	{PAUSE, 1, Nop_Cmd},	

	#if 0
	{GPIO_CNTRL, HAL_CAM_VGA_STANDBY, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},/*for vga camera shutdown*/
	
	{GPIO_CNTRL, HAL_CAM_VGA_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd}, /*for vga camera shutdown*/
	#endif
	
	//{GPIO_CNTRL, HAL_SECOND_CAM_RESET, GPIO_SetLow},  
	//{PAUSE, 10, Nop_Cmd},     

	/* -------Turn everything OFF   */
	//{GPIO_CNTRL, HAL_SECOND_CAM_RESET, GPIO_SetLow},	
	{GPIO_CNTRL, HAL_Second_CAM_PWDN, GPIO_SetHigh},
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},

	{PAUSE, 5, Nop_Cmd},
	/* -------Enable Clock to Cameras @ Main clock speed*/
	{GPIO_CNTRL, HAL_Second_CAM_PWDN, GPIO_SetLow},
	{PAUSE, 5, Nop_Cmd},
	
	{MCLK_CNTRL, CamDrv_24MHz, CLK_TurnOn},
	{PAUSE, 5, Nop_Cmd},    

	/* -------Raise Reset to ISP*/
	//{GPIO_CNTRL, HAL_SECOND_CAM_RESET, GPIO_SetHigh},	 
	//{PAUSE, 10, Nop_Cmd}
//hanwei@wind-mobi.com add 2011.11.04 end

};

/*---------Sensor Power Off*/
static CamSensorIntfCntrl_st_t CamPowerOffSeq[] = 
{
//hanwei@wind-mobi.com add 2011.11.04 start
//reviewed by liubing@wind-mobi.com
	/* No Hardware Standby available. */
	{PAUSE, 50, Nop_Cmd},
/* -------Lower Reset to ISP*/
	//{GPIO_CNTRL, HAL_SECOND_CAM_RESET, GPIO_SetLow},
	
	{GPIO_CNTRL, HAL_Second_CAM_PWDN, GPIO_SetHigh},
	{PAUSE, 5, Nop_Cmd},
/* -------Disable Clock to Cameras*/
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
/*--let primary camera enter soft standby mode , reset should be high-- */
	//{GPIO_CNTRL, HAL_SECOND_CAM_RESET, GPIO_SetHigh},  
//hanwei@wind-mobi.com add 2011.11.04 end
};

/** Secondary Sensor Configuration and Capabilities  */
static HAL_CAM_ConfigCaps_st_t CamSecondaryCfgCap_st = {
	// CAMDRV_DATA_MODE_S *video_mode
	{
        640,                           // unsigned short        max_width;                //Maximum width resolution
        480,                           // unsigned short        max_height;                //Maximum height resolution
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

        640,                           // unsigned short max_width;   Maximum width resolution
        480,                           // unsigned short max_height;  Maximum height resolution
        0,                              // UInt32                data_size;                //Minimum amount of data sent by the camera
        15,                             // UInt32                framerate_lo_absolute;  //Minimum possible framerate u24.8 format
        15,                             // UInt32                framerate_hi_absolute;  //Maximum possible framerate u24.8 format
        CAMDRV_TRANSFORM_NONE,          // CAMDRV_TRANSFORM_T    transform;            //Possible transformations in this mode / user requested transformations
        CAMDRV_IMAGE_YUV422,              // CAMDRV_IMAGE_TYPE_T    format;                //Image format of the frame.
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
	"OV7692"
};

/*---------Sensor Secondary Configuration CCIR656*/
//hanwei@wind-mobi.com add 2011.11.04 start
//reviewed by liubing@wind-mobi.com
static CamIntfConfig_CCIR656_st_t CamSecondaryCfg_CCIR656_st = {
	// Vsync, Hsync, Clock
	CSL_CAM_SYNC_EXTERNAL,				///< UInt32 sync_mode;				(default)CAM_SYNC_EXTERNAL:  Sync External or Embedded
	CSL_CAM_SYNC_DEFINES_ACTIVE,		///< UInt32 vsync_control;			(default)CAM_SYNC_DEFINES_ACTIVE:		VSYNCS determines active data
  	CSL_CAM_SYNC_ACTIVE_LOW, //	CSL_CAM_SYNC_ACTIVE_HIGH,			///< UInt32 vsync_polarity; 		   default)ACTIVE_LOW/ACTIVE_HIGH:		  Vsync active
	CSL_CAM_SYNC_DEFINES_ACTIVE,		///< UInt32 hsync_control;			(default)FALSE/TRUE:					HSYNCS determines active data
	CSL_CAM_SYNC_ACTIVE_HIGH,			///< UInt32 hsync_polarity; 		(default)ACTIVE_HIGH/ACTIVE_LOW:		Hsync active
	CSL_CAM_CLK_EDGE_POS,				///< UInt32 data_clock_sample;		(default)RISING_EDGE/FALLING_EDGE:		Pixel Clock Sample edge
	CSL_CAM_PIXEL_8BIT, 				///< UInt32 bus_width;				(default)CAM_BITWIDTH_8:				Camera bus width
	0,							///< UInt32 data_shift; 				   (default)0:							   data shift (+) left shift  (-) right shift
	CSL_CAM_FIELD_H_V,					///< UInt32 field_mode; 			(default)CAM_FIELD_H_V: 				field calculated
	CSL_CAM_INT_FRAME_END,				///< UInt32 data_intr_enable;		CAM_INTERRUPT_t:
	CSL_CAM_INT_FRAME_END,				///< UInt32 pkt_intr_enable;		CAM_INTERRUPT_t:
//hanwei@wind-mobi.com add 2011.11.04 end

};

CamIntfConfig_CCP_CSI_st_t  CamSecondaryCfg_CCP_CSI_st =
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

/*---------Sensor Secondary Configuration YCbCr Input*/
static CamIntfConfig_YCbCr_st_t CamSecondaryCfg_YCbCr_st = {
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

/*---------Sensor Secondary Configuration I2C */
static CamIntfConfig_I2C_st_t CamSecondaryCfg_I2C_st = {
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

/*---------Sensor Secondary Configuration IOCR */
static CamIntfConfig_IOCR_st_t CamSecondaryCfg_IOCR_st = {
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

/*---------Sensor Secondary Configuration */
static CamIntfConfig_st_t CamSensorCfg_st = {
	&CamSecondaryCfgCap_st,	/* *sensor_config_caps; */
	&CamSecondaryCfg_CCIR656_st,	/* *sensor_config_ccir656; */
	&CamSecondaryCfg_CCP_CSI_st,
	&CamSecondaryCfg_YCbCr_st,	/* *sensor_config_ycbcr; */
	NULL,	                     /* *sensor_config_i2c; */
	&CamSecondaryCfg_IOCR_st,	/* *sensor_config_iocr; */
	NULL,	/* *sensor_config_jpeg; */
	NULL,			/* *sensor_config_interleave_video; */
	NULL,	/**sensor_config_interleave_stills; */
	NULL	/* *sensor_config_pkt_marker_info; */
};
/// hanwei  #########################

static OV7692Reg_t sSensor_init_st[]=
{
	//hanwei@wind-mobi.com add 2011.11.04 start 
	//reviewed by liubing@wind-mobi.com
#if 1
	{0x12, 0x00},//12},//0x80},
	{0x0c ,0x96},//add flip 
	{0x0e ,0x08},
	{0x0e ,0x08},
	{0x69 ,0x52},
	{0x1e ,0xb3},
	{0x48 ,0x42},
	{0xff ,0x01},
	{0xb5 ,0x30},
	{0xff ,0x00},
	{0x16 ,0x03},
	{0x62 ,0x10},
	//{0x12 ,0x00},
	{0x17 ,0x65},
	{0x18 ,0xa4},
	{0x19 ,0x0a},
	{0x1a ,0xf6},
	{0x3e ,0x20},
	{0x64 ,0x11},
	{0x67 ,0x20},
	{0x81 ,0x3f},
	{0xcc ,0x02},
	{0xcd ,0x80},
	{0xce ,0x01},
	{0xcf ,0xe0},
	{0xc8 ,0x02},
	{0xc9 ,0x80},
	{0xca ,0x01},
	{0xcb ,0xe0},
	{0xd0 ,0x48},
	{0x82 ,0x03},
	{0x0e ,0x00},
	{0x70 ,0x00},
	{0x71 ,0x34},
	{0x74 ,0x28},
	{0x75 ,0x98},
	{0x76 ,0x00},
	{0x77 ,0x64},
	{0x78 ,0x01},
	{0x79 ,0xc2},
	{0x7a ,0x4e},
	{0x7b ,0x1f},
	{0x7c ,0x00},
	{0x11 ,0x02}, //0x01  jacky debug 2011.11.29
	{0x20 ,0x00},
	{0x21 ,0x57},
	{0x50 ,0x4d},
	{0x51 ,0x40},
	{0x4c ,0x7d},
	{0x0e ,0x00},
	{0x80 ,0x7f},  /*error -------------*/
	{0x85 ,0x00}, /*eror  --------------*/
	{0x86 ,0x00},
	{0x87 ,0x00},
	{0x88 ,0x00},
	{0x89 ,0x2a},
	{0x8a ,0x22},/*eror  --------------*/
	{0x8b ,0x20},/*eror  --------------*/
	{0xbb ,0xab},
	{0xbc ,0x84},
	{0xbd ,0x27},
	{0xbe ,0x0e},
	{0xbf ,0xb8},
	{0xc0 ,0xc5},
	{0xc1 ,0x1e},
	{0xb7 ,0x05},
	{0xb8 ,0x09},
	{0xb9 ,0x00},
	{0xba ,0x18},
	{0x5a ,0x1f},
	{0x5b ,0x9f},
	{0x5c ,0x69},
	{0x5d ,0x42},
	{0x24 ,0x78},
	{0x25 ,0x68},
	{0x26 ,0xb3},
	{0xa3 ,0x0b},
	{0xa4 ,0x15},
	{0xa5 ,0x29},
	{0xa6 ,0x4a},
	{0xa7 ,0x58},
	{0xa8 ,0x65},
	{0xa9 ,0x70},
	{0xaa ,0x7b},
	{0xab ,0x85},
	{0xac ,0x8e},
	{0xad ,0xa0},
	{0xae ,0xb0},
	{0xaf ,0xcb},
	{0xb0 ,0xe1},
	{0xb1 ,0xf1},
	{0xb2 ,0x14},
	{0x8e ,0x92},
	{0x96 ,0xff},
	{0x97 ,0x00},
	{0x14 ,0x3b},
	{0x0e ,0x00},
	//////////////////////

	{0x15 ,0xb8},
	//;lens cor
	{0x85 ,0x90}, 
	{0x86 ,0x10}, 
	{0x87 ,0x00}, 
	{0x88 ,0x10}, 
	{0x89 ,0x18}, 
	{0x8a ,0x10}, 
	{0x8b ,0x14}, 
	//;cmx     
	{0xbb ,0x80}, 
	{0xbc ,0x62}, 
	{0xbd ,0x1e}, 
	{0xbe ,0x26}, 
	{0xbf ,0x7b}, 
	{0xc0 ,0xac}, 
	{0xc1 ,0x1e}, 
	//;edge/denoise/exposure   
	{0xb7 ,0x02}, 
	{0xb8 ,0x0b}, 
	{0xb9 ,0x00}, 
	{0xba ,0x18}, 
	{0x5A ,0x4A}, 
	{0x5B ,0x9F}, 
	{0x5C ,0x48}, 
	{0x5d ,0x32}, 
	{0x24 ,0x88}, 
	{0x25 ,0x78}, 
	{0x26 ,0xb3}, 
	//;gamma   
	//hanwei@wind-mobi.com chg 2011.11.29 start
	//reviewed by liubing@wind-mobi.com
	/*
	{0xa3 ,0x08}, 
	{0xa4 ,0x15}, 
	{0xa5 ,0x24}, 
	{0xa6 ,0x45}, 
	{0xa7 ,0x55}, 
	{0xa8 ,0x6a}, 
	{0xa9 ,0x78}, 
	{0xaa ,0x87}, 
	{0xab ,0x96}, 
	{0xac ,0xa3}, 
	{0xad ,0xb4}, 
	{0xae ,0xc3}, 
	{0xaf ,0xd6}, 
	{0xb0 ,0xe6}, 
	{0xb1 ,0xf2}, 
	{0xb2 ,0x12}, 
	*/
	//hanwei@wind-mobi.com chg 2011.11.29 end
	{0xa3 ,0x0e}, 
	{0xa4 ,0x1a}, 
	{0xa5 ,0x31}, 
	{0xa6 ,0x5a}, 
	{0xa7 ,0x69}, 
	{0xa8 ,0x75}, 
	{0xa9 ,0x7e}, 
	{0xaa ,0x88}, 
	{0xab ,0x8f}, 
	{0xac ,0x96}, 
	{0xad ,0xa3}, 
	{0xae ,0xaf}, 
	{0xaf ,0xc4}, 
	{0xb0 ,0xd7}, 
	{0xb1 ,0xe8}, 
	{0xb2 ,0x20}, 	
	//;awb     
	{0x8e ,0x92}, 
	{0x96 ,0xff}, 
	{0x97 ,0x00}, 
	{0x8c ,0x5d}, 
	{0x8d ,0x11}, 
	{0x8e ,0x12}, 
	{0x8f ,0x11}, 
	{0x90 ,0x50}, 
	{0x91 ,0x22}, 
	{0x92 ,0xd1}, 
	{0x93 ,0xa7}, 
	{0x94 ,0x23}, 
	//hanwei@wind-mobi.com chg 2011.11.29 start
	//reviewed by liubing@wind-mobi.com
	#if 0
	{0x24 ,0x88},  
	{0x25 ,0x78},  
	{0x26 ,0xc4}, 
	#else
	{0x24 ,0x70},   //88    //jacky debug 
	{0x25 ,0x60},   //78
	{0x26 ,0x94}, 
	#endif
	//hanwei@wind-mobi.com chg 2011.11.29 end
	{0x95 ,0x3b}, 
	{0x96 ,0xff}, 
	{0x97 ,0x00}, 
	{0x98 ,0x4a}, 
	{0x99 ,0x46}, 
	{0x9a ,0x3d}, 
	{0x9b ,0x3a}, 
	{0x9c ,0xf0}, 
	{0x9d ,0xf0}, 
	{0x9e ,0xf0}, 
	{0x9f ,0xff}, 
	{0xa0 ,0x56}, 
	{0xa1 ,0x55}, 
	{0xa2 ,0x13}, 
	{0x50 ,0x4d}, 
	{0x51 ,0x3f}, 
	{0x21 ,0x57}, 
	{0x20 ,0x00}, 
	{0x14 ,0x29}, 
	{0x68 ,0xb0}, 
	{0xd2 ,0x07},
	{0x81 ,0x3f}, 
	{0xd3 ,0x18}, 
	{0xd2 ,0x07}, 
	{0xdc ,0x00},

	////////////////////////////
#else

	{0x12 ,0x80},
	{0x0c ,0x46},
	{0x48 ,0x42},
	{0x41 ,0x43},
	{0x4c ,0x73},
	{0x81 ,0xef},
	////////
	{0x21 ,0x44},
	{0x16 ,0x03},
	{0x39 ,0x80},
	{0x1e ,0xb1},
	{0x12 ,0x00},
	{0x82 ,0x03},
	///////
	{0xd0 ,0x48},
	{0x80 ,0x7f},
	{0x3e ,0x30},
	{0x22 ,0x00},
	{0x17 ,0x69},
	{0x18 ,0xa4},
	//////
	{0x19 ,0x0c},
	{0x1a ,0xf6},
	{0xc8 ,0x02},
	{0xc9 ,0x80},
	{0xca ,0x01},
	{0xcb ,0xe0},
	/////
	{0xcc ,0x02},
	{0xcd ,0x80},
	{0xce ,0x01},
	{0xcf ,0xe0},
	{0x85 ,0x90},
	{0x86 ,0x10},
	//////
	{0x87 ,0x00},
	{0x88 ,0x10},
	{0x89 ,0x18},
	{0x8a ,0x10},
	{0x8b ,0x14},
	             
	{0xBB ,0xac},
	{0xBC ,0xae},
	{0xBD ,0x02},
	{0xBE ,0x1f},
	{0xBF ,0x93},
	{0xC0 ,0xb1},
	{0xC1 ,0x1a},
	             
	{0xb7 ,0x02},
	{0xb8 ,0x0b},
	{0xb9 ,0x00},
	{0xba ,0x18},
	{0x5A ,0x4A},
	{0x5B ,0x9F},
	///////
	{0x5C ,0x48},
	{0x5d ,0x32},
	{0x24 ,0x88},
	{0x25 ,0x78},
	{0x26 ,0xb3},
	             
	{0xa3 ,0x04},
	{0xa4 ,0x0c},
	{0xa5 ,0x23},
	{0xa6 ,0x55},
	{0xa7 ,0x69},
	{0xa8 ,0x78},

	//////
	{0xa9 ,0x80},
	{0xaa ,0x88},
	{0xab ,0x90},
	{0xac ,0x97},
	{0xad ,0xa4},
	{0xae ,0xb0},
	////
	{0xaf ,0xc5},
	{0xb0 ,0xd7},
	{0xb1 ,0xe8},
	{0xb2 ,0x20},
	             
	{0x8e ,0x92},
	{0x96 ,0xff},
	{0x97 ,0x00},
	{0x8c ,0x5d},
	{0x8d ,0x11},
	{0x8e ,0x12},

	{0x8f ,0x11},
	{0x90 ,0x50},
	{0x91 ,0x22},
	{0x92 ,0xd1},
	{0x93 ,0xa7},
	{0x94 ,0x23},
	/////
	{0x24 ,0x88},
	{0x25 ,0x78},
	{0x26 ,0xc4},
	{0x95 ,0x3b},
	{0x96 ,0xff},
	{0x97 ,0x00},
	/////
	{0x98 ,0x4a},
	{0x99 ,0x46},
	{0x9a ,0x3d},
	{0x9b ,0x3a},
	{0x9c ,0xf0},
	{0x9d ,0xf0},
	////
	{0x9e ,0xf0},
	{0x9f ,0xff},
	{0xa0 ,0x56},
	{0xa1 ,0x55},
	{0xa2 ,0x13},
	{0x50 ,0x4d},

	{0x51 ,0x3f},
	{0x21 ,0x57},
	{0x20 ,0x00},
	{0x14 ,0x21},
	{0x15 ,0x98},
	{0x13 ,0xf7},
	{0x11 ,0x01},
	{0x68 ,0xb0},
	{0xd2 ,0x07},
#endif
	////////////////////////////
};
//hanwei@wind-mobi.com add 2011.11.04 end

/* I2C transaction result */
static HAL_CAM_Result_en_t sCamI2cStatus = HAL_CAM_SUCCESS;

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format);

static HAL_CAM_Result_en_t Init_OV7692(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt8 ov7692_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t ov7692_write(unsigned int sub_addr, UInt8 data);


/*****************************************************************************
*
* Function Name:   CAMDRV_GetIntfConfigTbl
*
* Description: Return Camera Sensor Interface Configuration
*
* Notes:
*
*****************************************************************************/
static CamIntfConfig_st_t *CAMDRV_GetIntfConfig_Sec(CamSensorSelect_t nSensor)
{

/* ---------Default to no configuration Table */
	CamIntfConfig_st_t *config_tbl = NULL;
 	pr_err("ov7692 CAMDRV_GetIntfConfig ");

	switch (nSensor) {
	case CamSensorPrimary:	/* Primary Sensor Configuration */
	default:
		CamSensorCfg_st.sensor_config_caps = NULL;
		break;
	case CamSensorSecondary:	/* Secondary Sensor Configuration */
		CamSensorCfg_st.sensor_config_caps = &CamSecondaryCfgCap_st;
		break;
	}
	config_tbl = &CamSensorCfg_st;

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
static CamSensorIntfCntrl_st_t *CAMDRV_GetIntfSeqSel_Sec(CamSensorSelect_t nSensor,
					      CamSensorSeqSel_t nSeqSel,
					      UInt32 *pLength)
{

/* ---------Default to no Sequence  */
	CamSensorIntfCntrl_st_t *power_seq = NULL;
	*pLength = 0;
 	pr_err("ov7692 CAMDRV_WakeupCAMDRV_Wakeup nSeqSel:%d",nSeqSel);

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
static HAL_CAM_Result_en_t CAMDRV_Wakeup_Sec(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
 	pr_err("ov7692 CAMDRV_Wakeup  ");

	result = Init_OV7692(sensor);
	printk("Init_OV7692 result =%d\r\n", result);
	return result;
}

static UInt16 CAMDRV_GetDeviceID_Sec(CamSensorSelect_t sensor)
{
//hanwei@wind-mobi.com 2011.11.02 get Device Id begin
	return ( (ov7692_read(DeviceId_Addr_H)<<8 )|(ov7692_read(DeviceId_Addr_L)) );
//hanwei@wind-mobi.com 2011.11.02 get Device Id end
}

static int checkCameraID(CamSensorSelect_t sensor)
{
	UInt16 devId = CAMDRV_GetDeviceID_Sec(sensor);
	pr_debug("hanwei:  *********Camera  OV7692 devId = 0x% **********\r\n",devId);

	if (devId == OV7692_ID) {
		printk("Camera identified as OV7692\r\n");
		return 0;
	} else {
		printk("Camera Id wrong. Expected 0x%x but got 0x%x\r\n",
			 OV7692_ID, devId);
		return -1;
	}
}

/*this one we should modufi it*/
static HAL_CAM_Result_en_t ov7692_write(unsigned int sub_addr, UInt8 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 write_data=data;
	

	result |= CAM_WriteI2c_Byte((UInt8)sub_addr, 1, &write_data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("ov7692_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
	return result;
}

static UInt8 ov7692_read(unsigned int sub_addr)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 data=0xFF;
	
	result |= CAM_ReadI2c_Byte((UInt8)sub_addr, 1,  &data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("ov7692_read(): ERROR: %d\r\n", result);
	}

	return data;
}

//hanwei@wind-mobi.com add 2011.11.04 start 
//reviewed by liubing@wind-mobi.com
static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
 	pr_err("ov7692 SensorSetPreviewMode image_resolution 0x%x image_format %d  ",image_resolution,image_format  );

	if(image_resolution == CamImageSize_QVGA){
		//hanwei@wind-mobi.com add 2011.11.09 start
		//reviewed by liubing@wind-mobi.com
		ov7692_write(0x16,0x03);
		ov7692_write(0x17,0x69);
		ov7692_write(0x18,0xa4);
		ov7692_write(0x19,0x06);
		ov7692_write(0x1a,0xf6);
		ov7692_write(0x22,0x10);
		
		ov7692_write(0xc8,0x02);
		ov7692_write(0xc9,0x80);
		ov7692_write(0xca,0x01);
		ov7692_write(0xcb,0xe0);
		
		ov7692_write(0xcc,0x01);//320
		ov7692_write(0xcd,0x40);
		ov7692_write(0xce,0x00);//240
		ov7692_write(0xcf,0xf0);
		//hanwei@wind-mobi.com add 2011.11.09 end
		}
	else if(image_resolution == CamImageSize_VGA)
	{	
		ov7692_write(0x16, 0x03);
		ov7692_write(0x17, 0x69);
		ov7692_write(0x18, 0xa4);
		ov7692_write(0x19, 0x0c);
		ov7692_write(0x1a, 0xf6);
		ov7692_write(0x22, 0x00);
		
		ov7692_write(0xc8, 0x02);
		ov7692_write(0xc9, 0x80);
		ov7692_write(0xca, 0x01);
		ov7692_write(0xcb, 0xe0);
		
		ov7692_write(0xcc, 0x02); //640
		ov7692_write(0xcd, 0x80);
		ov7692_write(0xce, 0x01); //480
		ov7692_write(0xcf, 0xe0);
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk
		    ("SensorSetPreviewMode(): Error[%d] sending preview mode  r\n",
		     sCamI2cStatus);
		result = sCamI2cStatus;
	}
	
	return result;
}
//hanwei@wind-mobi.com add 2011.11.04 end


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
static HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode_Sec(CamImageSize_t image_resolution,
					       CamDataFmt_t image_format,
					       CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_SetVideoCaptureMode \r\n");

	/* --------Set up Camera Isp for Output Resolution & Format */
	result = SensorSetPreviewMode(image_resolution, image_format);
	return result;
}

/****************************************************************************
*
* Function Name:   HAL_CAM_Result_en_t CAMDRV_SetExposure_Sec(CamRates_t fps)
*
* Description: This function sets the Exposure compensation
*
* Notes:   
*
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetExposure_Sec(int value)
{ 
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	
	printk("ov7692  CAMDRV_SetExposure_Sec%d \r\n",value );
	//hanwei@wind-mobi.com add 2012.04 start
	//fix bug#6931 enable exposure mode
	//reviewed by liubing@wind-mobi.com
	UInt32 temp_m ;
	temp_m = ov7692_read(0xd2); 
	temp_m |= 0x07;
	//hanwei@wind-mobi.com add 2012.04 end
		ov7692_write(0x24, 0x88);
		ov7692_write(0x25, 0x78);  	 
		ov7692_write(0x26, 0xa5);
        if(value == 0)
	{
		ov7692_write(0x81, 0x3f);
		ov7692_write(0xd3, 0x00);  	 
		ov7692_write(0xd2, temp_m);  //chg by hanwei 2012.01.04
		ov7692_write(0xdc, 0x00);
        }
	else if(value > 0)
	{
		ov7692_write(0x81, 0x3f);
		ov7692_write(0xd3, 0x18);
		ov7692_write(0xd2, temp_m);//chg by hanwei 2012.01.04
		ov7692_write(0xdc, 0x00);
	}
	else
	{
		ov7692_write(0x81, 0x3f);
		ov7692_write(0xd3, 0x18);
		ov7692_write(0xd2, temp_m); //chg by hanwei 2012.01.04
		ov7692_write(0xdc, 0x08);
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
static HAL_CAM_Result_en_t CAMDRV_SetFrameRate_Sec(CamRates_t fps,
					CamSensorSelect_t sensor)
{ 
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
		return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	pr_err("ov7692 CAMDRV_SetFrameRate %d \r\n",fps );
 
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
				 ov7692_write(0xCA,0x97);
				 ov7692_write(0xCE,0x63);
				 ov7692_write(0xCF,0x06);
				 ov7692_write(0x1E,0x18);				 
				
				break;
				
				case CamRate_10:             
				 ov7692_write(0xCA,0x97);
				 ov7692_write(0xCE,0x63);
				 ov7692_write(0xCF,0x06);
				 ov7692_write(0x1E,0x30);				 

				break;
				
				case CamRate_15:            // 15 fps
				 ov7692_write(0xCA,0x97);
				 ov7692_write(0xCE,0x63);
				 ov7692_write(0xCF,0x06);
				 ov7692_write(0x1E,0x48);				 
				
				break;
				
				case CamRate_20:             
				 ov7692_write(0xCA,0x97);
				 ov7692_write(0xCE,0x63);
				 ov7692_write(0xCF,0x06);
				 ov7692_write(0x1E,0x60);				 
				
				break;
				
				case CamRate_25:           // 25 fps
				 ov7692_write(0xCA,0x97);
				 ov7692_write(0xCE,0x63);
				 ov7692_write(0xCF,0x06);
				 ov7692_write(0x1E,0x76);				 
			
				break;
				
				case CamRate_30:           // 30 fps
				 ov7692_write(0xCA,0x97);
				 ov7692_write(0xCE,0x63);
				 ov7692_write(0xCF,0x06);
				 ov7692_write(0x1E,0x8E);
				
				break;
				
				default:                                        // Maximum Clock Rate = 26Mhz
				
				result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
				 printk("CAMDRV_SetFrameRate(): Error HAL_CAM_ERROR_ACTION_NOT_SUPPORTED \r\n");
				break;
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
/ Notes:
/
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_EnableVideoCapture_Sec(CamSensorSelect_t sensor)
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	/*[Enable stream] */
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_EnableVideoCapture \r\n");
	ov7692_write(0xC2, 0x80);
	ov7692_write(0xDE, 0x80);
	
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		 printk("CAMDRV_SetFrameRate(): Error[%d] \r\n",
			  sCamI2cStatus);
		 result = sCamI2cStatus;
        }
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
static HAL_CAM_Result_en_t CAMDRV_SetCamSleep_Sec(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	pr_err("ov7692 CAMDRV_SetCamSleep \r\n");

	/* To be implemented. */
	return result;
}

static HAL_CAM_Result_en_t CAMDRV_DisableCapture_Sec(CamSensorSelect_t sensor)
{
	/*[Disable stream] */
	pr_err("ov7692 CAMDRV_DisableCapture \r\n");

	return sCamI2cStatus;
}

/****************************************************************************
/
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_DisablePreview(void)
/
/ Description: This function halts MT9M111 camera video
/
/ Notes:
/
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_DisablePreview_Sec(CamSensorSelect_t sensor)
{
	pr_err("ov7692 CAMDRV_DisablePreview \r\n");
	
	return sCamI2cStatus;
}

/****************************************************************************
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetSceneMode(
/					CamSceneMode_t scene_mode)
/
/ Description: This function will set the scene mode of camera
/ Notes:
****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetSceneMode_Sec(CamSceneMode_t scene_mode,
					CamSensorSelect_t sensor)
{
//hanwei@wind-mobi.com debug 2011.11.04 start
//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	pr_err("ov7692 CAMDRV_SetSceneMode \r\n");

	switch(scene_mode) {
		case CamSceneMode_Auto:
			pr_info("CAMDRV_SetSceneMode() called for AUTO\n");
			
			break;
		case CamSceneMode_Night:
			pr_info("CAMDRV_SetSceneMode() called for Night\n");
			
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

static HAL_CAM_Result_en_t CAMDRV_SetWBMode_Sec(CamWB_WBMode_t wb_mode,
				     CamSensorSelect_t sensor)
{
	
	//hanwei@wind-mobi.com 2012.01.11 start
	//fix bug#6924 correct effect :WB 
	//reviewed by liubing@wind-mobi.com
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("--------CAMDRV_SetWBMode wb_mode:%d ---------\r\n", wb_mode);
	
	if (wb_mode == CamWB_Auto) 
	{
		ov7692_write(0x13,0xf7);
	}
	else if (wb_mode == CamWB_Incandescent) 
	{
		ov7692_write(0x13,0xf5);
		ov7692_write(0x01,0x66);
		ov7692_write(0x02,0x55);
		ov7692_write(0x03,0x50);
	} 

	else if (wb_mode == CamWB_WarmFluorescent)
	{
		ov7692_write(0xDE,0x80);
		ov7692_write(0x73,0x50);
		ov7692_write(0x74,0x00);
		ov7692_write(0x75,0x00);
		ov7692_write(0x76,0x30);
	} 
	else if (wb_mode == CamWB_Daylight) 
	{
		
		ov7692_write(0x13,0xf5);
		ov7692_write(0x01,0x3a); //60
		ov7692_write(0x02,0x76); //70
		ov7692_write(0x03,0x50); //70

	} 
	else if (wb_mode == CamWB_Cloudy) 
		{
		
		ov7692_write(0x13,0xf5);
		ov7692_write(0x01,0x38); //63
		ov7692_write(0x02,0x78); //80
		ov7692_write(0x03,0x50); //70

	} else if (wb_mode == CamWB_Twilight) {
		ov7692_write(0x13,0xf5);
		ov7692_write(0x01,0x60); //63
		ov7692_write(0x02,0x55); //75
		ov7692_write(0x03,0x50);


	}else if(wb_mode == CamWB_Fluorescent){
		
		ov7692_write(0x13,0xf5);
		ov7692_write(0x01,0x60);
		ov7692_write(0x03,0x50);
	       ov7692_write(0x02,0x50); //75

	}
	else {
		printk("Am here in wb:%d\n", wb_mode);
		;
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetWBMode(): Error[%d] \r\n", sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
	//hanwei@wind-mobi.com 2012.01.11 end
}

/****************************************************************************
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetAntiBanding(
/					CamAntiBanding_t effect)
/
/ Description: This function will set the antibanding effect of camera
/ Notes:
****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetAntiBanding_Sec(CamAntiBanding_t effect,
					  CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_SetAntiBanding effect:%d \r\n", effect);
	//hanwei@wind-mobi.com chg 2011.12.28 start
	//add antibanding option
	//reviewed by liubing@wind-mobi.com
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) 
	{
		ov7692_write(0xD7,0x08);

	} else if (effect == CamAntiBanding50Hz)
	{
		ov7692_write(0x13,0xef);
		ov7692_write(0x50,0xa6);
	} else if (effect == CamAntiBanding60Hz) 
	{
		ov7692_write(0x13,0xef);
		ov7692_write(0x50,0x8a);
	} else 
	{
		;	
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetAntiBanding(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	//hanwei@wind-mobi.com chg 2011.12.28 end
	return HAL_CAM_SUCCESS;

	//return result;
}

/****************************************************************************
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetFlashMode(
					FlashLedState_t effect)
/
/ Description: This function will set the flash mode of camera
/ Notes:
****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetFlashMode_Sec(FlashLedState_t effect,
					CamSensorSelect_t sensor)
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
		return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_SetFlashMode \r\n");
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
/ Description: This function will set the focus mode of camera, we not support it
/ Notes:
****************************************************************************/

static HAL_CAM_Result_en_t CAMDRV_SetFocusMode_Sec(CamFocusControlMode_t effect,
					CamSensorSelect_t sensor)
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_SetFocusMode \r\n");
	if (effect == CamFocusControlAuto) {


	} else if (effect == CamFocusControlMacro) {


	} else if (effect == CamFocusControlInfinity) {

		
	} else {


	}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetFocusMode(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}


static HAL_CAM_Result_en_t CAMDRV_TurnOffAF_Sec()
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_TurnOffAF \r\n");
	
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_TurnOffAF(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}

static HAL_CAM_Result_en_t CAMDRV_TurnOnAF_Sec()
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_TurnOnAF \r\n");
	/* AF DriverIC power enable */
	pr_info("CAMDRV_TurnOnAF() called\n");
	
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_TurnOnAF(): Error[%d] \r\n",
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
static HAL_CAM_Result_en_t CAMDRV_SetZoom_Sec(CamZoom_t step,
					  CamSensorSelect_t sensor)
{
	//hanwei@wind-mobi.com debug 2011.11.04 start
	//reviewed by liubing@wind-mobi.com
	return HAL_CAM_SUCCESS;
	//hanwei@wind-mobi.com debug 2011.11.04 end
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_SetZoom \r\n");
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


static HAL_CAM_Result_en_t CAMDRV_CfgStillnThumbCapture_Sec(CamImageSize_t
						 stills_resolution,
						 CamDataFmt_t stills_format,
						 CamImageSize_t
						 thumb_resolution,
						 CamDataFmt_t thumb_format,
						 CamSensorSelect_t sensor)
{

	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
 	pr_err("ov7692 CAMDRV_CfgStillnThumbCapture stills_resolution 0x%x stills_format %d  ",stills_resolution,stills_format);
        UInt16 aec_ag = 0;
        UInt16 aec_dg = 0;
        UInt16 aec_es = 0;
	if(stills_resolution == CamImageSize_QVGA)
		{
		//hanwei@wind-mobi.com add 2011.11.08 start
		//reviewed by liubing@wind-mobi.com
		//ov7692_write(0x22, 0x43);
			ov7692_write(0x16,0x03);
			ov7692_write(0x17,0x69);
			ov7692_write(0x18,0xa4);
			ov7692_write(0x19,0x06);
			ov7692_write(0x1a,0xf6);
			ov7692_write(0x22,0x10);
			ov7692_write(0xc8,0x02);
			ov7692_write(0xc9,0x80);
			
			ov7692_write(0xca,0x01);//00);
			ov7692_write(0xcb,0xe0);//f0);		
			ov7692_write(0xcc,0x01);
			ov7692_write(0xcd,0x40);
			ov7692_write(0xce,0x00);
			ov7692_write(0xcf,0xf0);
		//hanwei@wind-mobi.com add 2011.11.08 end
		}
	else 	if(stills_resolution == CamImageSize_VGA)
		{
			ov7692_write(0x16,0x03);
			ov7692_write(0x17,0x69);
			ov7692_write(0x18,0xa4);
			ov7692_write(0x19,0x0c);
			ov7692_write(0x1a,0xf6);
			ov7692_write(0x22,0x00);
			ov7692_write(0xc8,0x02);
			ov7692_write(0xc9,0x80);
			ov7692_write(0xca,0x01);
			ov7692_write(0xcb,0xe0);
			
			ov7692_write(0xcc,0x02);
			ov7692_write(0xcd,0x80);
			ov7692_write(0xce,0x01);
			ov7692_write(0xcf,0xe0);
		//add-e hanwei
		}
       /*
	ov7692_write(0xDE, 0xC0);
	ov7692_write(0xC2, 0xC0);
	aec_ag = ov7692_read(0x05);
	aec_ag = (aec_ag<<8)|ov7692_read(0x04);
              
	aec_dg = ov7692_read(0x06);
                
	aec_es = ov7692_read(0x08);
	aec_es = (aec_es<<8)|ov7692_read(0x07);

	ov7692_write(0xC2, 0x40);
	ov7692_write(0xC6, aec_ag&0xFF);
	ov7692_write(0xC7, (aec_ag>>8)&0xF);
	ov7692_write(0xC5, aec_dg&0xFF);
	ov7692_write(0xC3, aec_es&0xFF);
	ov7692_write(0xC4, (aec_es>>8)&0x3F);
	
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk
		    ("CAMDRV_CfgStillnThumbCapture(): Error[%d] sending preview mode  r\n",
		     sCamI2cStatus);
		result = sCamI2cStatus;
	}
	*/
      
	return result;
}


static HAL_CAM_Result_en_t CAMDRV_SetJpegQuality_Sec(CamJpegQuality_t effect,
					  CamSensorSelect_t sensor)
{
	pr_err("ov7692 CAMDRV_StoreBaseAddress \r\n");
	return HAL_CAM_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////
/****************************************************************************
/ Function Name:   HAL_CAM_Result_en_t CAMDRV_SetDigitalEffect(
/					CamDigEffect_t effect)
/
/ Description: This function will set the digital effect of camera
/ Notes:
****************************************************************************/
static HAL_CAM_Result_en_t CAMDRV_SetDigitalEffect_Sec(CamDigEffect_t effect,
					    CamSensorSelect_t sensor)
{



	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("ov7692 CAMDRV_SetDigitalEffect effect:0x%x \r\n", effect);
	if (effect == CamDigEffect_NoEffect) {   //auto
		//Mono: 02
		temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x00);
		ov7692_write(0xd2,0x00);
		ov7692_write(0xda,0x80);
		ov7692_write(0xdb,0x80);
	} 
	else if (effect == CamDigEffect_MonoChrome)
	{
	       //Mono: 02
	       temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x00);
		ov7692_write(0xd2,0x18);
		ov7692_write(0xda,0x80);
		ov7692_write(0xdb,0x80);
	}
	else if ((effect == CamDigEffect_NegColor) 				      //Negative: 08
		   || (effect == CamDigEffect_NegMono)) 
       {
      
	       temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x80);
		ov7692_write(0xd2,0x00);
		ov7692_write(0xda,0x80);
		ov7692_write(0xdb,0x80);
	} 
	else if ((effect == CamDigEffect_Blue_Tint)				//Solarize: 200 //Bluish
		   || (effect == CamDigEffect_SolarizeMono))
	{
	 
	       temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x00);
		ov7692_write(0xd2,0x1e);
		ov7692_write(0xda,0xa0);
		ov7692_write(0xdb,0x40);
	}
	else if (effect == CamDigEffect_SepiaGreen) 					//Sepia: 10
	{
	 
	       temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x00);
		ov7692_write(0xd2,0x1e);
		ov7692_write(0xda,0x40);
		ov7692_write(0xdb,0xa0);
	} 
	else if (effect == CamDigEffect_Auqa)//hanwei
	{
	
		       temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x00);										//Auqa: 2000000 Greenish
		ov7692_write(0xd2,0x1e);
		ov7692_write(0xda,0x60);
		ov7692_write(0xdb,0x60);
	} else if(effect == CamDigEffect_Red_Tint) //hanwei
	{
		//Posterize: 20 //Redish
		temp = ov7692_read(0x81);
		temp |= 0x20;
		ov7692_write(0x81,temp);
		ov7692_write(0x28,0x00);
		ov7692_write(0xd2,0x18);
		ov7692_write(0xda,0x80);
		ov7692_write(0xdb,0xc0);
	}else{
		;
		}
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

ktime_t tm1;

//hanwei@wind-mobi.com add 2011.11.02 start
//reviewed by liubing@wind-mobi.com
//init second camera OV7692 
static HAL_CAM_Result_en_t Init_OV7692(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	OV7692Reg_t *reg;
	int timeout;
	tm1 = ktime_get();
	int num,i ;
	printk(KERN_ERR "#############Init_OV7692  Start #########\r\n");

	num=ARRAY_SIZE(sSensor_init_st);
	
	pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	CamSensorCfg_st.sensor_config_caps = &CamSecondaryCfgCap_st ;
	printk("Init Secondary Sensor Init_OV7692: \r\n");
	
	printk(KERN_ERR"-- Init_OV7692 ID 0x%x 0x%x \r\n",ov7692_read(DeviceId_Addr_H),ov7692_read(DeviceId_Addr_L));

	for(i=0;i<num;i++)
	{
		reg=&(sSensor_init_st[i]);
		ov7692_write(reg->SubAddr, reg->Value);
		if(reg->SubAddr==0x12 && reg->Value==0x80 )
			msleep(2);
	}

	if (checkCameraID(sensor)) {
		;//result = HAL_CAM_ERROR_INTERNAL_ERROR;
	}
	
	tm1 = ktime_get();
	pr_err("debug:#############Init_OV7692  End ###################\r\n");
	
	printk(KERN_ERR"Exit Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);
}
//hanwei@wind-mobi.com 2011.11.02 end

struct sens_methods sens_meth_sec = {
    DRV_GetIntfConfig: CAMDRV_GetIntfConfig_Sec,
    DRV_GetIntfSeqSel : CAMDRV_GetIntfSeqSel_Sec,
    DRV_Wakeup : CAMDRV_Wakeup_Sec,
    DRV_SetVideoCaptureMode : CAMDRV_SetVideoCaptureMode_Sec,
    DRV_SetFrameRate : CAMDRV_SetFrameRate_Sec,
    DRV_EnableVideoCapture : CAMDRV_EnableVideoCapture_Sec,
    DRV_SetCamSleep : CAMDRV_SetCamSleep_Sec,
    DRV_DisableCapture : CAMDRV_DisableCapture_Sec,
    DRV_DisablePreview : CAMDRV_DisablePreview_Sec,
    DRV_CfgStillnThumbCapture : CAMDRV_CfgStillnThumbCapture_Sec,
    DRV_SetSceneMode : CAMDRV_SetSceneMode_Sec,
    DRV_SetWBMode : CAMDRV_SetWBMode_Sec,
    DRV_SetAntiBanding : CAMDRV_SetAntiBanding_Sec,
    DRV_SetFocusMode : CAMDRV_SetFocusMode_Sec,
    DRV_SetDigitalEffect : CAMDRV_SetDigitalEffect_Sec,
    DRV_SetFlashMode : CAMDRV_SetFlashMode_Sec,
    DRV_SetJpegQuality : CAMDRV_SetJpegQuality_Sec,//dummy
    DRV_TurnOnAF : CAMDRV_TurnOnAF_Sec,
    DRV_TurnOffAF : CAMDRV_TurnOffAF_Sec,
    DRV_SetExposure : CAMDRV_SetExposure_Sec,
};

struct sens_methods *CAMDRV_secondary_get(void)
{
	return &sens_meth_sec;
}

