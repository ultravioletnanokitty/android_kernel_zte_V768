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
*   @file   camdrv_tcm9001.c
*
*   @brief  This file is the lower level driver API of tcm9001 sensor.
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

#define TCM9001_ID 0x4810

#define OV7690_ID 0x7691

UInt32 temp;
#define HAL_PRIMARY_CAM_RESET 56  /*it should be low when using second sensor*/

//#define HAL_Second_CAM_RESET 10  no reset ping  
#define HAL_Second_CAM_PWDN 9

typedef struct{
	UInt8 SubAddr;
	UInt8 Value;
}TCM9001Reg_t;


/*---------Sensor Power On */
static CamSensorIntfCntrl_st_t CamPowerOnSeq[] = 
{
	{PAUSE, 1, Nop_Cmd},	

	#if 0
	{GPIO_CNTRL, HAL_CAM_VGA_STANDBY, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd},/*for vga camera shutdown*/
	
	{GPIO_CNTRL, HAL_CAM_VGA_RESET, GPIO_SetHigh},
	{PAUSE, 10, Nop_Cmd}, /*for vga camera shutdown*/
	#endif
	
	{GPIO_CNTRL, HAL_PRIMARY_CAM_RESET, GPIO_SetLow},  
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
	
	{PAUSE, 10, Nop_Cmd},

	/* -------Turn everything OFF   */
	
	{GPIO_CNTRL, HAL_Second_CAM_PWDN, GPIO_SetLow},

	/* -------Enable Clock to Cameras @ Main clock speed*/
	
	{PAUSE, 10, Nop_Cmd},
	
	{MCLK_CNTRL, CamDrv_24MHz, CLK_TurnOn},
	{PAUSE, 10, Nop_Cmd},

	/* -------Raise Reset to ISP*/
	
	
};

/*---------Sensor Power Off*/
static CamSensorIntfCntrl_st_t CamPowerOffSeq[] = 
{
	/* No Hardware Standby available. */
	{PAUSE, 5, Nop_Cmd},
//hanwei@wind-mobi.com chg 2012.02.27 start
//for fix big current 
//reviewed by liubing@wind-mobi.com
	//{GPIO_CNTRL, HAL_Second_CAM_PWDN, GPIO_SetLow},
	{GPIO_CNTRL, HAL_Second_CAM_PWDN, GPIO_SetHigh},
//hanwei@wind-mobi.com chg 2012.02.27 start
/* -------Disable Clock to Cameras*/
	{MCLK_CNTRL, CamDrv_NO_CLK, CLK_TurnOff},
/*--let primary camera enter soft standby mode , reset should be high-- */
	//{GPIO_CNTRL, HAL_PRIMARY_CAM_RESET, GPIO_SetHigh},  
	{GPIO_CNTRL, HAL_PRIMARY_CAM_RESET, GPIO_SetLow}, 
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
	"TCM9001"
};

/*---------Sensor Secondary Configuration CCIR656*/
static CamIntfConfig_CCIR656_st_t CamSecondaryCfg_CCIR656_st = {
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



/* I2C transaction result */
static HAL_CAM_Result_en_t sCamI2cStatus = HAL_CAM_SUCCESS;

static HAL_CAM_Result_en_t
SensorSetPreviewMode(CamImageSize_t image_resolution, CamDataFmt_t image_format);

static HAL_CAM_Result_en_t Init_OV7690(CamSensorSelect_t sensor);
static int checkCameraID(CamSensorSelect_t sensor);
static UInt8 ov7690_read(unsigned int sub_addr);
static HAL_CAM_Result_en_t ov7690_write(unsigned int sub_addr, UInt8 data);


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
 	pr_err("tcm9001 CAMDRV_GetIntfConfig ");

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
 	pr_err("tcm9001 CAMDRV_WakeupCAMDRV_Wakeup nSeqSel:%d",nSeqSel);

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
 	pr_err("tcm9001 CAMDRV_Wakeup  ");

	result = Init_OV7690(sensor);
	printk("Init_OV7690 result =%d\r\n", result);
	return result;
}

static UInt16 CAMDRV_GetDeviceID_Sec(CamSensorSelect_t sensor)
{
	return ( (ov7690_read(0x00)<<8 )|(ov7690_read(0x01)) );
}

static int checkCameraID(CamSensorSelect_t sensor)
{
	UInt16 devId = CAMDRV_GetDeviceID_Sec(sensor);

	if (devId == TCM9001_ID) {
		printk("Camera identified as TCM9001\r\n");
		return 0;
	} else {
		printk("Camera Id wrong. Expected 0x%x but got 0x%x\r\n",
			 TCM9001_ID, devId);
		return -1;
	}
}

/*this one we should modufi it*/
static HAL_CAM_Result_en_t ov7690_write(unsigned int sub_addr, UInt8 data)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	sCamI2cStatus = HAL_CAM_SUCCESS;
	UInt8 write_data=data;
	

	result |= CAM_WriteI2c_Byte((UInt8)sub_addr, 1, &write_data);
	if (result != HAL_CAM_SUCCESS) {
		sCamI2cStatus = result;
		pr_info("mt9t111_write(): ERROR: at addr:0x%x with value: 0x%x\n", sub_addr, data);
	}
	return result;
}

static UInt8 ov7690_read(unsigned int sub_addr)
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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
 	pr_err("tcm9001 SensorSetPreviewMode image_resolution 0x%x image_format %d  ",image_resolution,image_format  );

	if(image_resolution == CamImageSize_QVGA){
		ov7690_write(0x16,0x03);
		ov7690_write(0x17,0x69);
		ov7690_write(0x18,0xa4);
		ov7690_write(0x19,0x06);
		ov7690_write(0x1a,0xf6);
		ov7690_write(0x22,0x10);
		
		ov7690_write(0xc8,0x02);
		ov7690_write(0xc9,0x80);
		ov7690_write(0xca,0x01);
		ov7690_write(0xcb,0xe0);
		
		ov7690_write(0xcc,0x01);//320
		ov7690_write(0xcd,0x40);
		ov7690_write(0xce,0x00);//240
		ov7690_write(0xcf,0xf0);}
	else 	if(image_resolution == CamImageSize_VGA){
		ov7690_write(0x16, 0x03);
		ov7690_write(0x17, 0x69);
		ov7690_write(0x18, 0xa4);
		ov7690_write(0x19, 0x0c);
		ov7690_write(0x1a, 0xf6);
		ov7690_write(0x22, 0x00);
		
		ov7690_write(0xc8, 0x02);
		ov7690_write(0xc9, 0x80);
		ov7690_write(0xca, 0x01);
		ov7690_write(0xcb, 0xe0);
		
		ov7690_write(0xcc, 0x02); //640
		ov7690_write(0xcd, 0x80);
		ov7690_write(0xce, 0x01); //480
		ov7690_write(0xcf, 0xe0);
		}


		mdelay(1000);	//fix bug#10190 for when back camera switch to front camera ,front camera instant greenish 2012.04.25
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk
		    ("SensorSetPreviewMode(): Error[%d] sending preview mode  r\n",
		     sCamI2cStatus);
		result = sCamI2cStatus;
	}
	
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
static HAL_CAM_Result_en_t CAMDRV_SetVideoCaptureMode_Sec(CamImageSize_t image_resolution,
					       CamDataFmt_t image_format,
					       CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("tcm9001 CAMDRV_SetVideoCaptureMode \r\n");

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
	printk("OV7690  CAMDRV_SetExposure_Sec%d \r\n",value );
        if(value == 0){
		ov7690_write(0x24, 0x78);
		ov7690_write(0x25, 0x68);
		ov7690_write(0x26, 0xB4);
        	}
	else if(value > 0){
		ov7690_write(0x24, 0xb8);
		ov7690_write(0x25, 0xb0);
		ov7690_write(0x26, 0xf8);
		}
	else{
		ov7690_write(0x24, 0x30);
		ov7690_write(0x25, 0x28);
		ov7690_write(0x26, 0x61);
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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	return result;

	pr_err("tcm9001 CAMDRV_SetFrameRate %d \r\n",fps );
 
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
				 ov7690_write(0xCA,0x97);
				 ov7690_write(0xCE,0x63);
				 ov7690_write(0xCF,0x06);
				 ov7690_write(0x1E,0x18);				 
				
				break;
				
				case CamRate_10:             
				 ov7690_write(0xCA,0x97);
				 ov7690_write(0xCE,0x63);
				 ov7690_write(0xCF,0x06);
				 ov7690_write(0x1E,0x30);				 

				break;
				
				case CamRate_15:            // 15 fps
				 ov7690_write(0xCA,0x97);
				 ov7690_write(0xCE,0x63);
				 ov7690_write(0xCF,0x06);
				 ov7690_write(0x1E,0x48);				 
				
				break;
				
				case CamRate_20:             
				 ov7690_write(0xCA,0x97);
				 ov7690_write(0xCE,0x63);
				 ov7690_write(0xCF,0x06);
				 ov7690_write(0x1E,0x60);				 
				
				break;
				
				case CamRate_25:           // 25 fps
				 ov7690_write(0xCA,0x97);
				 ov7690_write(0xCE,0x63);
				 ov7690_write(0xCF,0x06);
				 ov7690_write(0x1E,0x76);				 
			
				break;
				
				case CamRate_30:           // 30 fps
				 ov7690_write(0xCA,0x97);
				 ov7690_write(0xCE,0x63);
				 ov7690_write(0xCF,0x06);
				 ov7690_write(0x1E,0x8E);
				
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
	/*[Enable stream] */
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	return result;
	
	pr_err("tcm9001 CAMDRV_EnableVideoCapture \r\n");
	ov7690_write(0xC2, 0x80);
	ov7690_write(0xDE, 0x80);
	
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

	pr_err("tcm9001 CAMDRV_SetCamSleep \r\n");

	/* To be implemented. */
	return result;
}

static HAL_CAM_Result_en_t CAMDRV_DisableCapture_Sec(CamSensorSelect_t sensor)
{
	/*[Disable stream] */
	pr_err("tcm9001 CAMDRV_DisableCapture \r\n");

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
	pr_err("tcm9001 CAMDRV_DisablePreview \r\n");
	
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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;

	pr_err("tcm9001 CAMDRV_SetSceneMode \r\n");
	return result;

	switch(scene_mode) {
		case CamSceneMode_Auto:
			pr_info("CAMDRV_SetSceneMode() called for AUTO\n");
			ov7690_write(0x15, 0x14);
			ov7690_write(0x2D, 0x00);
			ov7690_write(0x2E, 0x00);
			break;
		case CamSceneMode_Night:
			pr_info("CAMDRV_SetSceneMode() called for Night\n");
			ov7690_write(0x15, 0x34);
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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_info("CAMDRV_SetWBMode()  called  wb_mode = 0x%x \n",wb_mode);
	//return result;
	switch (wb_mode)
	{
		case CamWB_Auto:	//normal
			//printk("-----------CamWB_Auto-----------\r\n");
			ov7690_write(0x13,0xf7);
			//ov7690_write(0x15,0xa4);//00);
			break;	
		case CamWB_Daylight:		//sunny	
			//printk("-----------CamWB_Daylight---------\r\n");	
			ov7690_write(0x13,0xf5);
			ov7690_write(0x01,0x5a);
			ov7690_write(0x02,0x5c);
			//ov7690_write(0x15,0x00);
			break;		
		case CamWB_Fluorescent:		//home	
			//printk("-------------CamWB_Fluorescent------------\r\n");
			ov7690_write(0x13,0xf5);
			ov7690_write(0x01,0x66); ///96
			ov7690_write(0x02,0x40);
			//ov7690_write(0x15,0x00);
			break;
		case CamWB_Cloudy:		//cloudy
			//printk("------------CamWB_Cloudy------------\r\n");
			ov7690_write(0x13,0xf5);
			ov7690_write(0x01,0x58);
			ov7690_write(0x02,0x60);
			//ov7690_write(0x15,0x00);
			break;
		case CamWB_Incandescent:	//office	
			//printk("--------------CamWB_Incandescent---------\r\n");
			ov7690_write(0x13,0xf5);
			ov7690_write(0x01,0x64); //84
			ov7690_write(0x02,0x4c);
			//ov7690_write(0x15,0x00);
			break;	
		case CamWB_Twilight:
			printk("CamWB_Twilight\r\n");
			;
			break;
		case CamWB_Tungsten:
			printk("CamWB_Tungsten\r\n");
			;
			break;
		default:
			printk("HAL_CAM_ERROR_ACTION_NOT_SUPPORTED\r\n");
			result = HAL_CAM_ERROR_ACTION_NOT_SUPPORTED;
			return result;
	}
#if 1
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetWBMode(): Error[%d] \r\n", sCamI2cStatus);
		result = sCamI2cStatus;
	}
#endif
	return result;
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
	pr_err("tcm9001 CAMDRV_SetAntiBanding effect:%d \r\n", effect);
	return result;
	if ((effect == CamAntiBandingAuto) || (effect == CamAntiBandingOff)) 
	{
		ov7690_write(0xD7,0x08);

	} else if (effect == CamAntiBanding50Hz)
	{
		ov7690_write(0xD7,0x00);
		ov7690_write(0xD4,0x0A);		
	} else if (effect == CamAntiBanding60Hz) 
	{
		ov7690_write(0xD7,0x00);
		ov7690_write(0xD7,0x8A);
	} else 
	{
		
	}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetAntiBanding(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("tcm9001 CAMDRV_SetFlashMode \r\n");
	return result;
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
	return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("tcm9001 CAMDRV_SetFocusMode \r\n");
	
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
	//return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("tcm9001 CAMDRV_TurnOffAF \r\n");
	return result;
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_TurnOffAF(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}
	return result;
}

static HAL_CAM_Result_en_t CAMDRV_TurnOnAF_Sec()
{//return HAL_CAM_SUCCESS;
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("tcm9001 CAMDRV_TurnOnAF \r\n");
	/* AF DriverIC power enable */
	pr_info("CAMDRV_TurnOnAF() called\n");
	return result;
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
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	pr_err("tcm9001 CAMDRV_SetZoom \r\n");
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
 	pr_err("OV7690 CAMDRV_CfgStillnThumbCapture stills_resolution 0x%x stills_format %d  ",stills_resolution,stills_format);

	if(stills_resolution == CamImageSize_QVGA){
		//printk("------CamImageSize_QVGA  called ------\n");
			ov7690_write(0x16,0x03);
			ov7690_write(0x17,0x69);
			ov7690_write(0x18,0xa4);
			ov7690_write(0x19,0x06);
			ov7690_write(0x1a,0xf6);
			ov7690_write(0x22,0x10);
			ov7690_write(0xc8,0x02);
			ov7690_write(0xc9,0x80);
			
			ov7690_write(0xca,0x01);//00);
			ov7690_write(0xcb,0xe0);//f0);		
			ov7690_write(0xcc,0x01);
			ov7690_write(0xcd,0x40);
			ov7690_write(0xce,0x00);
			ov7690_write(0xcf,0xf0);
	}else if(stills_resolution == CamImageSize_VGA){
		//printk("------CamImageSize_VGA  called ------");
			ov7690_write(0x16,0x03);
			ov7690_write(0x17,0x69);
			ov7690_write(0x18,0xa4);
			ov7690_write(0x19,0x0c);
			ov7690_write(0x1a,0xf6);
			ov7690_write(0x22,0x00);
			ov7690_write(0xc8,0x02);
			ov7690_write(0xc9,0x80);
			ov7690_write(0xca,0x01);
			ov7690_write(0xcb,0xe0);
			
			ov7690_write(0xcc,0x02);
			ov7690_write(0xcd,0x80);
			ov7690_write(0xce,0x01);
			ov7690_write(0xcf,0xe0);
	}
      
	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk
		    ("CAMDRV_CfgStillnThumbCapture(): Error[%d] sending preview mode  r\n",
		     sCamI2cStatus);
		result = sCamI2cStatus;
	}
	
	return result;
}


static HAL_CAM_Result_en_t CAMDRV_SetJpegQuality_Sec(CamJpegQuality_t effect,
					  CamSensorSelect_t sensor)
{
	pr_err("tcm9001 CAMDRV_StoreBaseAddress \r\n");
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
	
	pr_err("-------OV7690 CAMDRV_SetDigitalEffect effect:%d -------\r\n", effect);
	switch(effect){
		case CamDigEffect_NoEffect:	//normal
			//printk("YUDE>>>>>----  CamDigEffect_NoEffect  = 0x%x ------\r\n" ,CamDigEffect_NoEffect);
			temp = ov7690_read(0x81);
			temp &= 0xff;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp &= 0xe7;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x80);
			ov7690_write(0xdb,0x80);
			break;
		case CamDigEffect_MonoChrome://mono
			//printk("YUDE>>>>>----  CamDigEffect_MonoChrome  = 0x%x ------\r\n" ,CamDigEffect_MonoChrome);
			temp = ov7690_read(0x81);
			temp |= 0x20;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp |= 0x18;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x80);
			ov7690_write(0xdb,0x80);
			break;
		case CamDigEffect_NegColor:	//negative
			//printk("YUDE>>>>>----  CamDigEffect_NegColor  = 0x%x ------\r\n" ,CamDigEffect_NegColor);
			temp = ov7690_read(0x81);
			temp |= 0x20;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp |= 0x80;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp &= 0xe7;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x80);
			ov7690_write(0xdb,0x80);
			break;
		case CamDigEffect_Antique:		//antique
			//printk("YUDE>>>>>----  CamDigEffect_Antique  = 0x%x ------\r\n" ,CamDigEffect_Antique);
			temp = ov7690_read(0x81);
			temp |= 0x20;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp |= 0x18;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x40);
			ov7690_write(0xdb,0xa0);
			break;
		case CamDigEffect_Red_Tint:	//redish
			//printk("YUDE>>>>>----  CamDigEffect_Red_Tint  = 0x%x ------\r\n" ,CamDigEffect_Red_Tint);
			temp = ov7690_read(0x81);
			temp |= 0x20;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp |= 0x18;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x80);
			ov7690_write(0xdb,0xc0);
			break;
		case CamDigEffect_Green_Tint:	//greenish
			//printk("YUDE>>>>>----  CamDigEffect_Green_Tint  = 0x%x ------\r\n" ,CamDigEffect_Green_Tint);
			temp = ov7690_read(0x81);
			temp |= 0x20;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp |= 0x18;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x60);
			ov7690_write(0xdb,0x60);
			break;
		case CamDigEffect_Blue_Tint:	//bluenish
			//printk("YUDE>>>>>----  CamDigEffect_Blue_Tint  = 0x%x ------\r\n" ,CamDigEffect_Blue_Tint);
			temp = ov7690_read(0x81);
			temp |= 0x20;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp |= 0x18;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0xa0);
			ov7690_write(0xdb,0x40);
			break;
		default:
			printk("YUDE>>>>>----  default  = 0x%x ------\r\n" ,CamDigEffect_NoEffect);
			temp = ov7690_read(0x81);
			temp &= 0xff;
			ov7690_write(0x81,temp);
			temp = ov7690_read(0x28);

			temp &= 0x7f;
			ov7690_write(0x28,temp);

			temp = ov7690_read(0xd2);
			temp &= 0xe7;
			ov7690_write(0xd2,temp);
			
			ov7690_write(0xda,0x80);
			ov7690_write(0xdb,0x80);
			break;

		}

	if (sCamI2cStatus != HAL_CAM_SUCCESS) {
		printk("CAMDRV_SetDigitalEffect(): Error[%d] \r\n",
			 sCamI2cStatus);
		result = sCamI2cStatus;
	}

	return result;
}

//ktime_t tm1;
static HAL_CAM_Result_en_t Init_OV7690(CamSensorSelect_t sensor)
{
	HAL_CAM_Result_en_t result = HAL_CAM_SUCCESS;
	//tm1 = ktime_get();

	//pr_info("Entry Init Sec %d nsec %d\n", tm1.tv.sec, tm1.tv.nsec);

	CamSensorCfg_st.sensor_config_caps = &CamSecondaryCfgCap_st ;
	printk("Init Secondary Sensor OV7690: \r\n");

	printk(KERN_ERR"-- OV7690 ID 0x%x 0x%x \r\n",ov7690_read(0x00),ov7690_read(0x01));
	
	//printk(KERN_ERR"-- OV7690 ID 0x%x 0x%x \r\n",ov7690_read(0x00),ov7690_read(0x01));
		ov7690_write(0x12, 0x80);
		mdelay(2);
		ov7690_write(0x28, 0x02);
		//ov7690_write(0x0c,0xc6);
		ov7690_write(0x0c,0x16); //2012.05.15 hanwei fix for front camera inversion
		ov7690_write(0x48,0x44);
		ov7690_write(0x49,0x0c);
		ov7690_write(0x41,0x43);
		ov7690_write(0x81,0xff);
		ov7690_write(0x21,0x67);
		ov7690_write(0x16,0x03);
		ov7690_write(0x39,0x80);
		//;;===Format===;;
		ov7690_write(0x11,0x01);
		ov7690_write(0x12,0x00);
		ov7690_write(0x82,0x03);
		ov7690_write(0xd0,0x48);
		ov7690_write(0x80,0x7f);
		ov7690_write(0x3e,0x30);
		ov7690_write(0x22,0x00);
		ov7690_write(0x2a,0x30);
		ov7690_write(0x2b,0x34);
		ov7690_write(0x2c,0x00);
		ov7690_write(0x15,0xa4);
		//;;===Resolution===;;
		ov7690_write(0x17,0x69);
		ov7690_write(0x18,0xa4);
		ov7690_write(0x19,0x0c);
		ov7690_write(0x1a,0xf6);
		ov7690_write(0xc8,0x02);
		ov7690_write(0xc9,0x80);
		ov7690_write(0xca,0x01);
		ov7690_write(0xcb,0xe0);
		ov7690_write(0xcc,0x02);
		ov7690_write(0xcd,0x80);
		ov7690_write(0xce,0x01);
		ov7690_write(0xcf,0xe0);
		//;;===Lens Correction==;
		ov7690_write(0x85,0x10);
		ov7690_write(0x86,0x00);
		ov7690_write(0x87,0x1b);
		ov7690_write(0x88,0xAF);
		ov7690_write(0x89,0x21);
		ov7690_write(0x8a,0x20);
		ov7690_write(0x8b,0x20);
		//Ron add-s for fonrt camera greenish 20120428
		//;cmx     
		ov7690_write(0xbb ,0x80); 
		ov7690_write(0xbc ,0x62);  
		ov7690_write(0xbd ,0x1e);  
		ov7690_write(0xbe ,0x26); 
		ov7690_write(0xbf ,0x7b); 
		ov7690_write(0xc0 ,0xac); 
		ov7690_write(0xc1 ,0x1e); 
		//;edge/denoise/exposure 
		ov7690_write(0xb7 ,0x02); 
		ov7690_write(0xb8 ,0x0b); 
		ov7690_write(0xb9 ,0x00); 
		ov7690_write(0xba ,0x18); 
		ov7690_write(0x5A ,0x4A); 
		ov7690_write(0x5B ,0x9F); 
		ov7690_write(0x5C ,0x48); 
		ov7690_write(0x5d ,0x32); 
		ov7690_write(0x24 ,0x88); 
		ov7690_write(0x25 ,0x78); 
		ov7690_write(0x26 ,0xb3); 
		//Ron add-e for fonrt camera greenish 20120428
	       /*
		//;;====Color Matrix======
		ov7690_write(0xbb,0xac);
		ov7690_write(0xbc,0xae);
		ov7690_write(0xbd,0x02);
		ov7690_write(0xbe,0x1f);
		ov7690_write(0xbf,0x93);
		ov7690_write(0xc0,0xb1);
		ov7690_write(0xc1,0x1A);
		//;;===Edge + Denoise====;
		ov7690_write(0xb4,0x16);
		ov7690_write(0xb7,0x06);
		ov7690_write(0xb8,0x06);
		ov7690_write(0xb9,0x02);
		ov7690_write(0xba,0x08);
		//;;====AEC/AGC target===
		ov7690_write(0x24,0x86);
		ov7690_write(0x25,0x76);
		ov7690_write(0x26,0xB4);
		*/
		//;;=====UV adjust======;
		ov7690_write(0x81,0xff);
		ov7690_write(0x5A,0x10);
		ov7690_write(0x5B,0xA1);
		ov7690_write(0x5C,0x3a);
		ov7690_write(0x5d,0x20);
		//;;====Gamma====;;
		ov7690_write(0xa3,0x04);
		ov7690_write(0xa4,0x09);
		ov7690_write(0xa5,0x18);
		ov7690_write(0xa6,0x38);
		ov7690_write(0xa7,0x47);
		ov7690_write(0xa8,0x56);
		ov7690_write(0xa9,0x66);
		ov7690_write(0xaa,0x74);
		ov7690_write(0xab,0x7f);
		ov7690_write(0xac,0x89);
		ov7690_write(0xad,0x9a);
		ov7690_write(0xae,0xa9);
		ov7690_write(0xaf,0xc4);
		ov7690_write(0xb0,0xdb);
		ov7690_write(0xb1,0xee);
		ov7690_write(0xb2,0x18);
		/*
		//;;==Advance AWB==;;
		ov7690_write(0x8c, 0x5c);
		ov7690_write(0x8d, 0x11);
		ov7690_write(0x8e, 0x12);
		ov7690_write(0x8f, 0x19);
		ov7690_write(0x90, 0x50);
		ov7690_write(0x91, 0x20);
		ov7690_write(0x92, 0x99);
		ov7690_write(0x93, 0x91);
		ov7690_write(0x94, 0x0f);
		ov7690_write(0x95, 0x13);
		ov7690_write(0x96, 0xff);
		ov7690_write(0x97, 0x00);
		ov7690_write(0x98, 0x38);
		ov7690_write(0x99, 0x33);
		ov7690_write(0x9a, 0x4f);
		ov7690_write(0x9b, 0x43);
		ov7690_write(0x9c, 0xf0);
		ov7690_write(0x9d, 0xf0);
		ov7690_write(0x9e, 0xf0);
		ov7690_write(0x9f, 0xff);
		ov7690_write(0xa0, 0x60);
		ov7690_write(0xa1, 0x5a);
		ov7690_write(0xa2, 0x10);
		ov7690_write(0x14,0x21);
		ov7690_write(0xd8,0x40);
		ov7690_write(0xd9,0x40);
		ov7690_write(0xd2,0x00);
		ov7690_write(0x50,0x6e);
		ov7690_write(0x51,0x5c);
		ov7690_write(0x13,0xf7);
              */
		//;awb     
	 	ov7690_write(0x8e ,0x92);
	 	ov7690_write(0x96 ,0xff); 
	 	ov7690_write(0x97 ,0x00); 
	 	ov7690_write(0x8c ,0x5d); 
	 	ov7690_write(0x8d ,0x11);
	 	ov7690_write(0x8e ,0x12); 
	 	ov7690_write(0x8f ,0x11);
	 	ov7690_write(0x90 ,0x50);
	 	ov7690_write(0x91 ,0x22);
	 	ov7690_write(0x92 ,0xd1);
	 	ov7690_write(0x93 ,0xa7);
	 	ov7690_write(0x94 ,0x23);

	return result;
	
	
}

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

