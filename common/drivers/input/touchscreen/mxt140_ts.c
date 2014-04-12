/*
 * MXT140 Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */


#include <linux/device.h>

//xiongbiao@wind-mobi.com 2011.12.15 begin
#define DEBUG   
#define DEBUG_PEN_STATUS
#define CONFIG_TOUCHSCREEN_MXT140_MULTITOUCH

#define USE_BRCM_WORK_QUEUE
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/mxt140_ts.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h>

#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//maxiaohui tp compatible
#define BOARD_VIRTUALKEY_MXT140
//xiongbiao@wind-mobi.com 2011.12.15 begin
//modify 
/* Version */
#define MXT140_VER_20			0x20
#define MXT140_VER_21			0x21
#define MXT140_VER_22			0x22
//xiongbiao@wind-mobi.com 2011.12.15 end

/* Slave addresses */
#define MXT140_APP_LOW		0x4a
#define MXT140_APP_HIGH		0x4b
#define MXT140_BOOT_LOW		0x24
#define MXT140_BOOT_HIGH		0x25

/* Firmware */
#define MXT140_FW_NAME		"mxt140.fw"

/* Registers */
#define MXT140_FAMILY_ID		0x00
#define MXT140_VARIANT_ID		0x01
#define MXT140_VERSION		0x02
#define MXT140_BUILD			0x03
#define MXT140_MATRIX_X_SIZE		0x04
#define MXT140_MATRIX_Y_SIZE		0x05
#define MXT140_OBJECT_NUM		0x06
#define MXT140_OBJECT_START		0x07

#define MXT140_OBJECT_SIZE		6

/* Object types */
#define MXT140_DEBUG_DIAGNOSTIC	37
#define MXT140_GEN_MESSAGE		5
#define MXT140_GEN_COMMAND		6
#define MXT140_GEN_POWER		7
#define MXT140_GEN_ACQUIRE		8
#define MXT140_TOUCH_MULTI		9
#define MXT140_TOUCH_KEYARRAY		15
#define MXT140_TOUCH_PROXIMITY	23
#define MXT140_PROCI_GRIPFACE		20
#define MXT140_PROCG_NOISE		22
#define MXT140_PROCI_ONETOUCH		24
#define MXT140_PROCI_TWOTOUCH		27
#define MXT140_SPT_COMMSCONFIG	18	/* firmware ver 21 over */
#define MXT140_SPT_GPIOPWM		19
#define MXT140_SPT_SELFTEST		25
#define MXT140_SPT_CTECONFIG		28
#define MXT140_SPT_USERDATA		38	/* firmware ver 21 over */

/* MXT140_GEN_COMMAND field */
#define MXT140_COMMAND_RESET		0
#define MXT140_COMMAND_BACKUPNV	1
#define MXT140_COMMAND_CALIBRATE	2
#define MXT140_COMMAND_REPORTALL	3
#define MXT140_COMMAND_DIAGNOSTIC	5

/* MXT140_GEN_POWER field */
#define MXT140_POWER_IDLEACQINT	0
#define MXT140_POWER_ACTVACQINT	1
#define MXT140_POWER_ACTV2IDLETO	2

/* MXT140_GEN_ACQUIRE field */
#define MXT140_ACQUIRE_CHRGTIME	0
#define MXT140_ACQUIRE_TCHDRIFT	2
#define MXT140_ACQUIRE_DRIFTST	3
#define MXT140_ACQUIRE_TCHAUTOCAL	4
#define MXT140_ACQUIRE_SYNC		5
#define MXT140_ACQUIRE_ATCHCALST	6
#define MXT140_ACQUIRE_ATCHCALSTHR	7

/* MXT140_TOUCH_MULTI field */
#define MXT140_TOUCH_CTRL		0
#define MXT140_TOUCH_XORIGIN		1
#define MXT140_TOUCH_YORIGIN		2
#define MXT140_TOUCH_XSIZE		3
#define MXT140_TOUCH_YSIZE		4
#define MXT140_TOUCH_BLEN		6
#define MXT140_TOUCH_TCHTHR		7
#define MXT140_TOUCH_TCHDI		8
#define MXT140_TOUCH_ORIENT		9
#define MXT140_TOUCH_MOVHYSTI		11
#define MXT140_TOUCH_MOVHYSTN		12
#define MXT140_TOUCH_NUMTOUCH		14
#define MXT140_TOUCH_MRGHYST		15
#define MXT140_TOUCH_MRGTHR		16
#define MXT140_TOUCH_AMPHYST		17
#define MXT140_TOUCH_XRANGE_LSB	18
#define MXT140_TOUCH_XRANGE_MSB	19
#define MXT140_TOUCH_YRANGE_LSB	20
#define MXT140_TOUCH_YRANGE_MSB	21
#define MXT140_TOUCH_XLOCLIP		22
#define MXT140_TOUCH_XHICLIP		23
#define MXT140_TOUCH_YLOCLIP		24
#define MXT140_TOUCH_YHICLIP		25
#define MXT140_TOUCH_XEDGECTRL	26
#define MXT140_TOUCH_XEDGEDIST	27
#define MXT140_TOUCH_YEDGECTRL	28
#define MXT140_TOUCH_YEDGEDIST	29
#define MXT140_TOUCH_JUMPLIMIT	30	/* firmware ver 22 over */

/* MXT140_PROCI_GRIPFACE field */
#define MXT140_GRIPFACE_CTRL		0
#define MXT140_GRIPFACE_XLOGRIP	1
#define MXT140_GRIPFACE_XHIGRIP	2
#define MXT140_GRIPFACE_YLOGRIP	3
#define MXT140_GRIPFACE_YHIGRIP	4
#define MXT140_GRIPFACE_MAXTCHS	5
#define MXT140_GRIPFACE_SZTHR1	7
#define MXT140_GRIPFACE_SZTHR2	8
#define MXT140_GRIPFACE_SHPTHR1	9
#define MXT140_GRIPFACE_SHPTHR2	10
#define MXT140_GRIPFACE_SUPEXTTO	11

/* MXT140_PROCI_NOISE field */
#define MXT140_NOISE_CTRL		0
#define MXT140_NOISE_OUTFLEN		1
#define MXT140_NOISE_GCAFUL_LSB	3
#define MXT140_NOISE_GCAFUL_MSB	4
#define MXT140_NOISE_GCAFLL_LSB	5
#define MXT140_NOISE_GCAFLL_MSB	6
#define MXT140_NOISE_ACTVGCAFVALID	7
#define MXT140_NOISE_NOISETHR		8
#define MXT140_NOISE_FREQHOPSCALE	10
#define MXT140_NOISE_FREQ0		11
#define MXT140_NOISE_FREQ1		12
#define MXT140_NOISE_FREQ2		13
#define MXT140_NOISE_FREQ3		14
#define MXT140_NOISE_FREQ4		15
#define MXT140_NOISE_IDLEGCAFVALID	16

/* MXT140_SPT_COMMSCONFIG */
#define MXT140_COMMS_CTRL		0
#define MXT140_COMMS_CMD		1

/* MXT140_SPT_CTECONFIG field */
#define MXT140_CTE_CTRL		0
#define MXT140_CTE_CMD		1
#define MXT140_CTE_MODE		2
#define MXT140_CTE_IDLEGCAFDEPTH	3
#define MXT140_CTE_ACTVGCAFDEPTH	4
#define MXT140_CTE_VOLTAGE		5	/* firmware ver 21 over */

#define MXT140_VOLTAGE_DEFAULT	2700000
#define MXT140_VOLTAGE_STEP		10000

/* Define for MXT140_GEN_COMMAND */
#define MXT140_BOOT_VALUE		0xa5
#define MXT140_BACKUP_VALUE		0x55
#define MXT140_BACKUP_TIME		25	/* msec */
#define MXT140_RESET_TIME		65	/* msec */

#define MXT140_FWRESET_TIME		175	/* msec */

/* Command to unlock bootloader */
#define MXT140_UNLOCK_CMD_MSB		0xaa
#define MXT140_UNLOCK_CMD_LSB		0xdc

/* Bootloader mode status */
#define MXT140_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT140_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT140_FRAME_CRC_CHECK	0x02
#define MXT140_FRAME_CRC_FAIL		0x03
#define MXT140_FRAME_CRC_PASS		0x04
#define MXT140_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define MXT140_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT140_SUPPRESS		(1 << 1)
#define MXT140_AMP			(1 << 2)
#define MXT140_VECTOR			(1 << 3)
#define MXT140_MOVE			(1 << 4)
#define MXT140_RELEASE		(1 << 5)
#define MXT140_PRESS			(1 << 6)
#define MXT140_DETECT			(1 << 7)

/* Touchscreen absolute values */
#define MXT140_MAX_XC			320
#define MXT140_MAX_YC			480
#define MXT140_MAX_AREA		0xff

#define MXT140_MAX_FINGER		10

/* Initial register values recommended from chip vendor */
static const u8 init_vals_ver_20[] = {
	/* MXT140_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_GEN_POWER(7) */
	0x20, 0xff, 0x32,
	/* MXT140_GEN_ACQUIRE(8) */
	0x08, 0x05, 0x05, 0x00, 0x00, 0x00, 0x05, 0x14,
	/* MXT140_TOUCH_MULTI(9) */
	0x00, 0x00, 0x00, 0x11, 0x0a, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x01, 0x01, 0x0e, 0x0a, 0x0a, 0x0a, 0x0a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x64,
	/* MXT140_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* MXT140_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00,
	/* MXT140_PROCI_GRIPFACE(20) */
	0x00, 0x64, 0x64, 0x64, 0x64, 0x00, 0x00, 0x1e, 0x14, 0x04,
	0x1e, 0x00,
	/* MXT140_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x19, 0x00, 0xe7, 0xff, 0x04, 0x32, 0x00,
	0x01, 0x0a, 0x0f, 0x14, 0x00, 0x00, 0xe8,
	/* MXT140_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
	/* MXT140_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x00, 0x04, 0x08,
};

//xiongbiao@wind-mobi.com 2011.12.15 begin
//modify for setup
static const u8 init_vals_ver_21[] = {
	/* MXT140_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_GEN_POWER(7) */
	0x20, 0x10, 0x32, // idle, act, idle ->active 
	/* MXT140_GEN_ACQUIRE(8):wrong */
	0x06, 0x00, 0x0a, 0x0a, 0x00, 0x00, 0xff, 0x01, 0x00, 0x00,
	/* MXT140_TOUCH_MULTI(9) */
	0x83, 0x00, 0x02, 0x0f, 0x09, 0x00, 0x10, 0x2a, 0x02, 0x01,
	0x00, 0x06, 0x02, 0x00, 0x05, 0x0a, 0x0a, 0x0a, 0x07, 0x02,  //tp work area
	0x3f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00,
	0x0f, 0x05,
	/* MXT140_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* MXT140_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_GRIPFACE(20) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x28, 0x04,
	0x0f, 0x0a,
	/* MXT140_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x14, 0x00,
	0x00, 0x05, 0x0a, 0x0b, 0x13, 0x15, 0x08,
	/* MXT140_TOUCH_PROXIMITY(23) :wrong*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_TWOTOUCH(27) */
	/*0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
	/* MXT140_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x02, 0x10, 0x10, 0x1e,
};
//xiongbiao@wind-mobi.com 2011.12.15 end

static const u8 init_vals_ver_22[] = {
	/* MXT140_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* [SPT_USERDATA_T38 INSTANCE 0] */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_GEN_POWER(7) */
	/*
	 * IDLEACQINT (Idle Acquisition Interval) - 60ms
	 * ACTVACQINT (Active Acquisition Interval) - 37ms
	 * ACTV2IDLETO (Active to Idle time out) - 60ms
	 *
	 * Configuring the ACTVACQINT to 50ms in conjunction with
	 * making the controller operate on Mode1 (MXT140_SPT_COMMSCONFIG)
	 * brought down the number of interrupts sent to the Host.
	 * This value seems optimal with respect to sytem load when multi-touch
	 * is enabled.
	 */
	0x3C, 0x1B, 0x3C,
	/* MXT140_GEN_ACQUIRE(8) */
	0x0a, 0x00, 0x05, 0x00, 0x00, 0x00, 0x09, 0x23,
	/* MXT140_TOUCH_MULTI(9) */
	/*
	 * Touch Threshold TCHTHR - 30 (typical 30 to 80)
	 * Lower the Threshold value the more sensitive the touch is.
	 * Programming the least value recommended in the Data sheet
	 */
	0x83, 0x00, 0x00, 0x0f, 0x0b, 0x00, 0x21, 0x46, 0x02, 0x07,
	0x00, 0x01, 0x01, 0x00, 0x0a, 0x0a, 0x0a, 0x0a, 0xFF, 0x03,
	0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* MXT140_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* MXT140_SPT_COMMSCONFIG(18) */
	0x01, 0x00,
	/* MXT140_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_GRIPFACE(20) */
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x28, 0x04,
	0x0f, 0x0a,
	/* MXT140_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x2d, 0x00,
	0x00, 0x05, 0x0f, 0x19, 0x23, 0x2b, 0x03,
	/* MXT140_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* MXT140_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT140_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x00, 0x10, 0x20, 0x3c,
};

struct mxt140_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt140_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt140_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct mxt140_finger {
	int status;
	int x;
	int y;
	int area;
};

/* Each client has this additional data */
struct mxt140_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt140_platform_data *pdata;
	struct mxt140_object *object_table;
	struct mxt140_info info;
	struct mxt140_finger finger[MXT140_MAX_FINGER];
	unsigned int irq;
	int pen_down;
	struct early_suspend suspend_desc;
 #ifdef VKEY_SUPPORT
        struct ts_printkey_data *pk_data;
        u16 pk_num;
#endif

#ifdef USE_BRCM_WORK_QUEUE
	struct delayed_work work;
#endif

};
#ifdef  VKEY_SUPPORT
static unsigned int ctr_menu = 1;
static void mxt140_send_key_upevent(struct input_dev *input, unsigned int code)
{

        input_report_key(input, code, 0);
        input_sync(input);
}
static void _mxt140_send_key_event(struct input_dev *input, unsigned int code)
{

        input_report_key(input, code, 1);
        input_sync(input);
        //input_report_key(input, code, 0);
        //input_sync(input);
}

static void mxt140_send_key_event(struct input_dev *input, unsigned int code)
{

       //if (code != 59)
       {
            _mxt140_send_key_event(input, code);
       }
#if 0
       else
       {
           printk("[WYQ] key menu %d\n",ctr_menu);
          if(ctr_menu < 3){
              input_report_key(input, code, 1);
              input_sync(input);
              
          }
          else if(ctr_menu < 5){
             input_report_key(input, code, 0);
              input_sync(input);
              if(ctr_menu == 5)
                  ctr_menu = 1;
          }
          ctr_menu ++;
       
       }
#endif
}
#endif

static bool mxt140_object_readable(unsigned int type)
{
	switch (type) {
	case MXT140_GEN_MESSAGE:
	case MXT140_GEN_COMMAND:
	case MXT140_GEN_POWER:
	case MXT140_GEN_ACQUIRE:
	case MXT140_TOUCH_MULTI:
	case MXT140_TOUCH_KEYARRAY:
	case MXT140_TOUCH_PROXIMITY:
	case MXT140_PROCI_GRIPFACE:
	case MXT140_PROCG_NOISE:
	case MXT140_PROCI_ONETOUCH:
	/*case MXT140_PROCI_TWOTOUCH:*/
	case MXT140_SPT_COMMSCONFIG:
	case MXT140_SPT_GPIOPWM:
	case MXT140_SPT_SELFTEST:
	case MXT140_SPT_CTECONFIG:
	case MXT140_SPT_USERDATA:
		return true;
	default:
		return false;
	}
}

static bool mxt140_object_writable(unsigned int type)
{
	switch (type) {
	case MXT140_GEN_COMMAND:
	case MXT140_GEN_POWER:
	case MXT140_GEN_ACQUIRE:
	case MXT140_TOUCH_MULTI:
	case MXT140_TOUCH_KEYARRAY:
	case MXT140_SPT_GPIOPWM:
	case MXT140_PROCI_GRIPFACE:
	case MXT140_PROCG_NOISE:
    case MXT140_TOUCH_PROXIMITY:
	case MXT140_PROCI_ONETOUCH:
//xiongbiao@wind-mobi.com 2011.12.15 begin
	/*case MXT140_PROCI_TWOTOUCH:*/
//xiongbiao@wind-mobi.com 2011.12.15 end	
	case MXT140_SPT_SELFTEST:
	case MXT140_SPT_CTECONFIG:
		return true;
	default:
		return false;
	}
}

static void mxt140_dump_message(struct device *dev,
				  struct mxt140_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
}

static int mxt140_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT140_WAITING_BOOTLOAD_CMD:
	case MXT140_WAITING_FRAME_DATA:
		val &= ~MXT140_BOOT_STATUS_MASK;
		break;
	case MXT140_FRAME_CRC_PASS:
		if (val == MXT140_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt140_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT140_UNLOCK_CMD_LSB;
	buf[1] = MXT140_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt140_fw_write(struct i2c_client *client,
			     const u8 * data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt140_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt140_read_reg(struct i2c_client *client, u16 reg, u8 * val)
{
	return __mxt140_read_reg(client, reg, 1, val);
}

static int mxt140_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt140_read_object_table(struct i2c_client *client,
				      u16 reg, u8 * object_buf)
{
	return __mxt140_read_reg(client, reg, MXT140_OBJECT_SIZE,
				   object_buf);
}

static struct mxt140_object *mxt140_get_object(struct mxt140_data *data,
						   u8 type)
{
	struct mxt140_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt140_read_message(struct mxt140_data *data,
				 struct mxt140_message *message)
{
	struct mxt140_object *object;
	u16 reg;

	object = mxt140_get_object(data, MXT140_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt140_read_reg(data->client, reg,
				   sizeof(struct mxt140_message), message);
}

static int mxt140_read_object(struct mxt140_data *data,
				u8 type, u8 offset, u8 * val)
{
	struct mxt140_object *object;
	u16 reg;

	object = mxt140_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt140_read_reg(data->client, reg + offset, 1, val);
}

static int mxt140_write_object(struct mxt140_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt140_object *object;
	u16 reg;

	object = mxt140_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt140_write_reg(data->client, reg + offset, val);
}

static void mxt140_input_report(struct mxt140_data *data, int single_id)
{
	struct mxt140_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;
	
//xiongbiao@wind-mobi.com 2011.12.15 begin
#if defined (CONFIG_TOUCHSCREEN_MXT140_MULTITOUCH)
	for (id = 0; id < MXT140_MAX_FINGER; id++)
	{
		if (!finger[id].status)
			continue;
		/*
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
				 finger[id].status != MXT140_RELEASE ?
				 finger[id].area : 0);
		input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
		*/
#ifdef  VKEY_SUPPORT
        int vkey = 1,vkey2 = 1;
        u16 usCount;

        for (usCount = 0; usCount < 4;    usCount++)
        {
			//printk("[WYQ] id[%d] iVTP x =%d, y=%d\n",id,finger[id].x,finger[id].y);
            if ((finger[id].y > (data->pk_data + usCount)->up)
                    && (finger[id].y < (data->pk_data + usCount)->down)
                    && (finger[id].x > (data->pk_data + usCount)->left)
                    && (finger[id].x < (data->pk_data + usCount)->right))
            {      
                  mxt140_send_key_event(input_dev,(data->pk_data+usCount)->code);
                  //printk(" Send Printkey %d  \r\n", (data->pk_data+usCount)->code);
                  data->pen_down = true;
                        
                  vkey = 0;
                  vkey2 = 1;

              }
         }
         if(vkey)
		 {
                
            vkey = 1;
#endif
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
			//input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(input_dev);

			if (finger[id].status == MXT140_RELEASE)
				finger[id].status = 0;
			else
				finger_num++;
	#ifdef VKEY_SUPPORT
          }
	#endif
	}
	input_sync(input_dev);
#else
	if (status != MXT140_RELEASE)
	{
		finger_num = 1;
		input_report_key(input_dev, BTN_TOUCH, finger_num > 0);
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		input_sync(input_dev);
	}
#endif



	if (status != MXT140_RELEASE)
	{
		data->pen_down = 1;
	} 
	else 
	{
#ifdef  VKEY_SUPPORT
		u16 usCount;

		for (usCount = 0; usCount < 4;    usCount++)
        {
                     
            if ((finger[id].y > (data->pk_data + usCount)->up)
                  && (finger[id].y < (data->pk_data + usCount)->down)
                  && (finger[id].x > (data->pk_data + usCount)->left)
                  && (finger[id].x < (data->pk_data + usCount)->right))
             {
                   if(vkey2)
                   {      
                         mxt140_send_key_upevent(input_dev,(data->pk_data+usCount)->code);
                         // printk("[up] Send Printkey %d  \r\n", (data->pk_data+usCount)->code);
                         data->pen_down = false;

                    }

               }
           }
          if(vkey2 !=1 )
		  {
#endif
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(input_dev);
		data->pen_down = 0;
#ifdef VKEY_SUPPORT
          }
          else
               vkey2 = 0;
#endif
	}
//xiongbiao@wind-mobi.com 2011.12.15 end


}

static void mxt140_input_touchevent(struct mxt140_data *data,
				      struct mxt140_message *message, int id)
{
	struct mxt140_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;

	/* Check the touch is present on the screen */
	if (!(status & MXT140_DETECT)) {
		if (status & MXT140_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT140_RELEASE;
			mxt140_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT140_PRESS | MXT140_MOVE)))
		return;
	

	x = (message->message[1] << 2) | ((message->message[3] & ~0x3f) >> 6);
	y = (message->message[2] << 2) | ((message->message[3] & ~0xf3) >> 2);
	area = message->message[4];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT140_MOVE ? "moved" : "pressed", x, y, area);
	//printk("[%d] %s x: %d, y: %d, area: %d\n", id,
	//	status & MXT140_MOVE ? "moved" : "pressed", x, y, area);  //open for debug

	finger[id].status = status & MXT140_MOVE ?
	    MXT140_MOVE : MXT140_PRESS;
	
//xiongbiao@wind-mobi.com 2011.12.15 start
#if 1
	finger[id].x = x;
	finger[id].y = y;
#else
#if defined(CONFIG_ENABLE_HVGA) && defined(CONFIG_BCM_LCD_NT35582)
	/*HVGA on WVGA panel*/
	finger[id].x = (x - 40 - 1024 * (480 - 320) / 2 / 480) * 480 / 320;
	finger[id].y = (y - 40 - 1024 * (800 - 480) / 2 / 800) * 800 / 480;
#elif defined(CONFIG_ENABLE_QVGA)
#if defined(CONFIG_BCM_LCD_R61581)
	/* QVGA on HVGA panel*/
	finger[id].x = (x - 40 - 1024 * (320 - 240) / 2 / 320) * 320 / 240;
	finger[id].y = (y - 40 - 1024 * (480 - 320) / 2 / 480) * 480 / 320;
#elif defined(CONFIG_BCM_LCD_NT35582)
	/*QVGA on HVGA panel*/
	finger[id].x = (x - 40 - 1024 * (480 - 240) / 2 / 480) * 480 / 240;
	finger[id].y = (y - 40 - 1024 * (800 - 320) / 2 / 800) * 800 / 320;
#endif
#else
	#if defined (CONFIG_TOUCHSCREEN_MXT140_MULTITOUCH)

		finger[id].x = (x - 10 ) * 1024 / (1024 - 10);
		finger[id].y = (y - 50) * 1024 / (1024 - 50);
	#else
		finger[id].x = (x + 50) * 1024 / (1024 + 50);
		finger[id].y = (y + 50) * 1024 / (1024 + 50);
	#endif
#endif
#endif
//xiongbiao@wind-mobi.com 2011.12.15 end

	finger[id].area = area;

	mxt140_input_report(data, id);
}

#ifndef USE_BRCM_WORK_QUEUE
static irqreturn_t mxt140_interrupt(int irq, void *dev_id)
{
	struct mxt140_data *data = dev_id;
	struct mxt140_message message;
	struct mxt140_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

	do {
		if (mxt140_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT140_TOUCH_MULTI */
		object = mxt140_get_object(data, MXT140_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt140_input_touchevent(data, &message, id);
		else
			mxt140_dump_message(dev, &message);
	} while (reportid != 0xff);

end:
	return IRQ_HANDLED;
}
#endif

#ifdef USE_BRCM_WORK_QUEUE

#define TS_POLL_DELAY	(0)	/* ms delay between irq & sample */
#define to_delayed_work(_work) container_of(_work, struct delayed_work, work)

static void mxt140_read_input(struct mxt140_data *data);

static void mxt140_touch_work(struct work_struct *work)
{
	struct mxt140_data *p_data =
	    container_of(to_delayed_work(work), struct mxt140_data, work);

	/* Peform the actual read and send the events to input sub-system */
	mxt140_read_input(p_data);

	/* Re-enable the interrupts from the device */
//	enable_irq(p_data->irq);
}

static void mxt140_read_input(struct mxt140_data *data)
{
	/* Implement what is implemented in the threaded IRQ implementation of the original driver */
	struct mxt140_message message;
	struct mxt140_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

	do {
		if (mxt140_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}
		reportid = message.reportid;

		/* whether reportid is thing of MXT140_TOUCH_MULTI */
		object = mxt140_get_object(data, MXT140_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt140_input_touchevent(data, &message, id);
		else {
			mxt140_dump_message(dev, &message);
		}
	} while (reportid != 0xff);
end:
	return;
}

static irqreturn_t mxt140_touch_interrupt(int irq, void *dev_id)
{
	struct mxt140_data *p_data = dev_id;

	/*
	 * Disable further interrupts from the device
	 * Note that, we assume that the device is configured
	 * to operate in  Mode1 where the interrupt line is
	 * pulled low until the last available valid message is
	 * read from the device. Since the irq is configured as
	 * edge sensitive for falling edge, we explicitly disable
	 * further interrupts from the device until we finish
	 * processing all the messages from the bottom half and
	 * re-enable it from the bottom half.
	 */
//	disable_irq_nosync(p_data->irq);
	schedule_delayed_work(&p_data->work, msecs_to_jiffies(TS_POLL_DELAY));
	return IRQ_HANDLED;
}
#endif

static int mxt140_check_reg_init(struct mxt140_data *data)
{
	struct mxt140_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j;
	u8 version = data->info.version;
	u8 *init_vals;

	switch (version) {
	case MXT140_VER_20:
		init_vals = (u8 *) init_vals_ver_20;
		break;
	case MXT140_VER_21:
		init_vals = (u8 *) init_vals_ver_21;
		break;
	case MXT140_VER_22:
		init_vals = (u8 *) init_vals_ver_22;
		break;
	default:
		dev_err(dev, "Firmware version %d doesn't support\n", version);
		return -EINVAL;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt140_object_writable(object->type))
			continue;

		for (j = 0; j < object->size + 1; j++)
			mxt140_write_object(data, object->type, j,
					      init_vals[index + j]);

		index += object->size + 1;
	}

	return 0;
}

static int mxt140_check_matrix_size(struct mxt140_data *data)
{
	const struct mxt140_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int mode = -1;
	int error;
	u8 val;

	dev_dbg(dev, "Number of X lines: %d\n", pdata->x_line);
	dev_dbg(dev, "Number of Y lines: %d\n", pdata->y_line);

	switch (pdata->x_line) {
	case 0 ... 15:
		if (pdata->y_line <= 14)
			mode = 0;
		break;
	case 16:
		if (pdata->y_line <= 12)
			mode = 1;
		if (pdata->y_line == 13 || pdata->y_line == 14)
			mode = 0;
		break;
	case 17:
		if (pdata->y_line <= 11)
			mode = 2;
		if (pdata->y_line == 12 || pdata->y_line == 13)
			mode = 1;
		break;
	case 18:
		if (pdata->y_line <= 10)
			mode = 3;
		if (pdata->y_line == 11 || pdata->y_line == 12)
			mode = 2;
		break;
	case 19:
		if (pdata->y_line <= 9)
			mode = 4;
		if (pdata->y_line == 10 || pdata->y_line == 11)
			mode = 3;
		break;
	case 20:
		mode = 4;
	}

	if (mode < 0) {
		dev_err(dev, "Invalid X/Y lines\n");
		return -EINVAL;
	}

	error = mxt140_read_object(data, MXT140_SPT_CTECONFIG,
				     MXT140_CTE_MODE, &val);
	if (error)
		return error;

	if (mode == val)
		return 0;

	/* Change the CTE configuration */
	mxt140_write_object(data, MXT140_SPT_CTECONFIG,
			      MXT140_CTE_CTRL, 1);
//xiongbiao@wind-mobi.com 2011.12.15 start
//rewrite for setup
	//mxt140_write_object(data, MXT140_SPT_CTECONFIG,
			      //MXT140_CTE_MODE, mode);
//xiongbiao@wind-mobi.com 2011.12.15 end			  
	mxt140_write_object(data, MXT140_SPT_CTECONFIG,
			      MXT140_CTE_CTRL, 0);

	return 0;
}

static int mxt140_make_highchg(struct mxt140_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt140_message message;
	int count = 20;
	int error;

	/* Read dummy message to make high CHG pin */
	do {
		error =
		    mxt140_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != 0xff && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

static void mxt140_handle_pdata(struct mxt140_data *data)
{
	const struct mxt140_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	mxt140_write_object(data, MXT140_TOUCH_MULTI, MXT140_TOUCH_XSIZE,
			      pdata->x_line);
	mxt140_write_object(data, MXT140_TOUCH_MULTI, MXT140_TOUCH_YSIZE,
			      pdata->y_line);

	/* Set touchscreen orient */
	mxt140_write_object(data, MXT140_TOUCH_MULTI, MXT140_TOUCH_ORIENT,
			      pdata->orient);

	/* Set touchscreen burst length */
	mxt140_write_object(data, MXT140_TOUCH_MULTI,
			      MXT140_TOUCH_BLEN, pdata->blen);

	/* Set touchscreen threshold */
	mxt140_write_object(data, MXT140_TOUCH_MULTI,
			      MXT140_TOUCH_TCHTHR, pdata->threshold);

	/* Set touchscreen resolution */
	mxt140_write_object(data, MXT140_TOUCH_MULTI,
			      MXT140_TOUCH_XRANGE_LSB,
			      (pdata->x_size - 1) & 0xff);
	mxt140_write_object(data, MXT140_TOUCH_MULTI,
			      MXT140_TOUCH_XRANGE_MSB,
			      (pdata->x_size - 1) >> 8);
	mxt140_write_object(data, MXT140_TOUCH_MULTI,
			      MXT140_TOUCH_YRANGE_LSB,
			      (pdata->y_size - 1) & 0xff);
	mxt140_write_object(data, MXT140_TOUCH_MULTI,
			      MXT140_TOUCH_YRANGE_MSB,
			      (pdata->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (data->info.version >= MXT140_VER_21 && pdata->voltage) {
		if (pdata->voltage < MXT140_VOLTAGE_DEFAULT) {
			voltage = (MXT140_VOLTAGE_DEFAULT - pdata->voltage) /
			    MXT140_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (pdata->voltage - MXT140_VOLTAGE_DEFAULT) /
			    MXT140_VOLTAGE_STEP;

		mxt140_write_object(data, MXT140_SPT_CTECONFIG,
				      MXT140_CTE_VOLTAGE, voltage);
	}
}


/* heweimao@wind-mobi.com 2012.09.11 begin */
static int read_tpid(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int ret;

    ret = sprintf(buf, "%s", "mxt140");

    return ret;
}
/* heweimao@wind-mobi.com 2012.09.11end */

/* heweimao@wind-mobi.com 2012.09.11 begin */
static struct proc_dir_entry *TP_pdir;
static struct proc_dir_entry *TPid_point;
/* heweimao@wind-mobi.com 2012.09.11 end */


static int mxt140_get_info(struct mxt140_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt140_info *info = &data->info;
	int error;
	u8 val;

	error = mxt140_read_reg(client, MXT140_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt140_read_reg(client, MXT140_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt140_read_reg(client, MXT140_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt140_read_reg(client, MXT140_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = mxt140_read_reg(client, MXT140_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt140_get_object_table(struct mxt140_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT140_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt140_object *object = data->object_table + i;

		reg = MXT140_OBJECT_START + MXT140_OBJECT_SIZE * i;
		error = mxt140_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
			    (object->instances + 1);
			object->max_reportid = reportid;
		}
	}
	return 0;
}

static int mxt140_initialize(struct mxt140_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt140_info *info = &data->info;
	int error;
	u8 val;

	error = mxt140_get_info(data);
	if (error)
		return error;

/* heweimao@wind-mobi.com 2012.09.11  begin */
    TP_pdir = proc_mkdir("TPdir", NULL);
    if (TP_pdir == NULL)
        pr_err("create /proc/TPdir error!\n");

    TPid_point = create_proc_read_entry("TP_id",0444,TP_pdir,read_tpid,NULL);
    if (TPid_point == NULL) {
        pr_err("create /proc/lcddir error!\n");
        remove_proc_entry("TPdir",NULL);
    }
/*  heweimao@wind-mobi.com 2012.09.11 end */

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt140_data), GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt140_get_object_table(data);
	if (error)
		return error;

	/* Check register init values */
	error = mxt140_check_reg_init(data);
	if (error)
		return error;

	/* Check X/Y matrix size */
	error = mxt140_check_matrix_size(data);
	if (error)
		return error;

    error = mxt140_make_highchg(data);
	if(error)
	{
         printk(KERN_INFO"[WYQ] mxt140_make_highchg error\n");
		return error;
	}
//xiongbiao@wind-mobi.com 2011.12.15 start
//rewrite for setup
	//mxt140_handle_pdata(data);
//xiongbiao@wind-mobi.com 2011.12.15 end

	/* Backup to memory */
	mxt140_write_object(data, MXT140_GEN_COMMAND,
			      MXT140_COMMAND_BACKUPNV, MXT140_BACKUP_VALUE);
	msleep(MXT140_BACKUP_TIME);

	/* Soft reset */
	mxt140_write_object(data, MXT140_GEN_COMMAND,
			      MXT140_COMMAND_RESET, 1);
	msleep(MXT140_RESET_TIME);

	/* Update matrix size at info struct */
	error = mxt140_read_reg(client, MXT140_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;

	error = mxt140_read_reg(client, MXT140_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;

	dev_info(&client->dev,
		 "Family ID: %d Variant ID: %d Version: %d Build: %d\n",
		 info->family_id, info->variant_id, info->version, info->build);

	dev_info(&client->dev,
		 "Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
		 info->matrix_xsize, info->matrix_ysize, info->object_num);

	return 0;
}

static ssize_t mxt140_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt140_data *data = dev_get_drvdata(dev);
	struct mxt140_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += sprintf(buf + count,
				 "Object Table Element %d(Type %d)\n",
				 i + 1, object->type);

		if (!mxt140_object_readable(object->type)) {
			count += sprintf(buf + count, "\n");
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			error = mxt140_read_object(data,
						     object->type, j, &val);
			if (error)
				return error;

			count += sprintf(buf + count,
					 "  Byte %d: 0x%x (%d)\n", j, val, val);
		}

		count += sprintf(buf + count, "\n");
	}

	return count;
}

static int mxt140_load_fw(struct device *dev, const char *fn)
{
	struct mxt140_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	mxt140_write_object(data, MXT140_GEN_COMMAND,
			      MXT140_COMMAND_RESET, MXT140_BOOT_VALUE);
	msleep(MXT140_RESET_TIME);

	/* Change to slave address of bootloader */
	if (client->addr == MXT140_APP_LOW)
		client->addr = MXT140_BOOT_LOW;
	else
		client->addr = MXT140_BOOT_HIGH;

	ret = mxt140_check_bootloader(client, MXT140_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt140_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt140_check_bootloader(client,
						MXT140_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		mxt140_fw_write(client, fw->data + pos, frame_size);

		ret = mxt140_check_bootloader(client,
						MXT140_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == MXT140_BOOT_LOW)
		client->addr = MXT140_APP_LOW;
	else
		client->addr = MXT140_APP_HIGH;

	return ret;
}

static ssize_t mxt140_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt140_data *data = dev_get_drvdata(dev);
	unsigned int version;
	int error;

	if (sscanf(buf, "%u", &version) != 1) {
		dev_err(dev, "Invalid values\n");
		return -EINVAL;
	}

	if (data->info.version < MXT140_VER_21 || version < MXT140_VER_21) {
		dev_err(dev, "FW update supported starting with version 21\n");
		return -EINVAL;
	}

	disable_irq(data->irq);

	error = mxt140_load_fw(dev, MXT140_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(MXT140_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

		mxt140_initialize(data);
	}

	enable_irq(data->irq);

	return count;
}

static DEVICE_ATTR(object, 0444, mxt140_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, mxt140_update_fw_store);

#if defined(DEBUG_PEN_STATUS)
static ssize_t pen_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mxt140_data *p_data =
	    (struct mxt140_data *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", p_data->pen_down);
}

static DEVICE_ATTR(pen_status, 0644, pen_status_show, NULL);
#endif

static struct attribute *mxt140_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
#if defined(DEBUG_PEN_STATUS)
	&dev_attr_pen_status.attr,
#endif
	NULL
};

static const struct attribute_group mxt140_attr_group = {
	.attrs = mxt140_attrs,
};

static void mxt140_start(struct mxt140_data *data)
{
	/* Touch enable */

	/*
	 * Writing 0x03 does not enable the SCANEN bit.
	 * which is optimized for not notifying the Host with unnecessary
	 * messages.
	 * The data sheet says that, if there are some issues in detecting
	 * multiple touches try, enabling this bit i.e write 0x83
	 */
	mxt140_write_object(data,
			      MXT140_TOUCH_MULTI, MXT140_TOUCH_CTRL, 0x03);
}

static void mxt140_stop(struct mxt140_data *data)
{
	/* Touch disable */
	mxt140_write_object(data,
			      MXT140_TOUCH_MULTI, MXT140_TOUCH_CTRL, 0);
}

static int mxt140_input_open(struct input_dev *dev)
{
	struct mxt140_data *data = input_get_drvdata(dev);

	mxt140_start(data);

	return 0;
}

static void mxt140_input_close(struct input_dev *dev)
{
	struct mxt140_data *data = input_get_drvdata(dev);

	mxt140_stop(data);
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int mxt140_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxt140_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		mxt140_stop(data);

		mxt140_write_object(data, MXT140_GEN_POWER,
				MXT140_POWER_IDLEACQINT, 0);
		mxt140_write_object(data, MXT140_GEN_POWER,
				MXT140_POWER_ACTVACQINT, 0);
	}
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int mxt140_resume(struct i2c_client *client)
{
	struct mxt140_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		mxt140_write_object(data, MXT140_GEN_POWER,
				MXT140_POWER_IDLEACQINT, init_vals_ver_22[14]);
		mxt140_write_object(data, MXT140_GEN_POWER,
				MXT140_POWER_ACTVACQINT, init_vals_ver_22[15]);
		mxt140_start(data);

	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#else
#define mxt140_suspend	NULL
#define mxt140_resume		NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt140_early_suspend(struct early_suspend *desc)
{
	struct mxt140_data *data = container_of(desc, struct mxt140_data, suspend_desc);
	pm_message_t mesg = {
		.event = PM_EVENT_SUSPEND,
		};
	mxt140_suspend(data->client, mesg);
}

static void mxt140_late_resume(struct early_suspend *desc)
{
	struct mxt140_data *data = container_of(desc, struct mxt140_data, suspend_desc);
	mxt140_resume(data->client);
}
#endif
#if defined(BOARD_VIRTUALKEY_MXT140)
static ssize_t acar_virtual_keys_show(struct kobject *kobj,
                                               struct kobj_attribute *attr, char *buf)
{
       return sprintf(buf,
               __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":40:500:60:60"
          ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":120:500:60:60"
          ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":200:500:60:60"
          ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":280:500:60:60"
          "\n");


}

static struct kobj_attribute acar_virtual_keys_attr = {
       .attr = {
               .name = "virtualkeys.mxt140_ts",
               .mode = S_IRUGO,
       },
       .show = &acar_virtual_keys_show,
};

static struct attribute *acar_properties_attrs[] = {
       &acar_virtual_keys_attr.attr,
       NULL
};

static struct attribute_group acar_properties_attr_group = {
       .attrs = acar_properties_attrs,
};
#endif

static int __devinit mxt140_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct mxt140_data *data;
	struct input_dev *input_dev;
	int error;

	if (!client->dev.platform_data)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt140_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
#ifdef VKEY_SUPPORT
    int iCount;
    /* code ,left ,right ,up ,down */
    struct ts_printkey_data pk_data_tlb[] = {
        {59, 0, 79, 481, 579},
        {KEY_HOME, 80, 159, 481, 579},
        {KEY_BACK, 160, 239, 481, 579},
        {KEY_SEARCH, 240, 319, 481, 579}
    };
#endif
	input_dev->name = "mxt140_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt140_input_open;
	input_dev->close = mxt140_input_close;

//xiongbiao@wind-mobi.com 2011.12.15 start
//modify
#if 0
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
#else
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) /*| BIT(ABS_PRESSURE)*/;
#endif	
#if 0

	 int acar_virtualkeys[] = {
		 KEY_MENU ,KEY_HOME, KEY_BACK, KEY_SEARCH
	};

	/* register virtualkeys capability if support */
	if(acar_virtualkeys) {
		int i = 0;
		while(acar_virtualkeys[i] > 0) {
			__set_bit(acar_virtualkeys[i++], input_dev->keybit);
		}
	}
#endif
//xiongbiao@wind-mobi.com 2011.12.15 end


	data->client = client;
	data->input_dev = input_dev;
	data->pdata = client->dev.platform_data;
	data->irq = client->irq;
#ifdef VKEY_SUPPORT
    data->pk_data = (struct ts_printkey_data *)kmalloc(sizeof(pk_data_tlb), GFP_KERNEL);
    if (!data->pk_data)
    {
           error = -ENOMEM;
           goto err_free_mem;
    }

    memcpy(data->pk_data, pk_data_tlb, sizeof(pk_data_tlb));

    data->pk_num = 4;/*sizeof(pk_data_tlb) / sizeof(struct ts_printkey_data);*/

    for (iCount = 0; iCount < 4; iCount++)
    {
          input_dev->keybit[BIT_WORD(data->pk_data[iCount].code)] =
             BIT_MASK(data->pk_data[iCount].code);
    }
#endif

//xiongbiao@wind-mobi.com 2011.12.15 start
#if 0
//xiongbiao
	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     data->pdata->x_min, data->pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     data->pdata->y_min, data->pdata->y_size, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, data->pdata->max_area, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->pdata->y_max, 0, 0);
#else
//	input_set_abs_params(input_dev, ABS_X, 0, data->pdata->x_size, 0, 0);
//	input_set_abs_params(input_dev, ABS_Y, 0, data->pdata->y_size, 0, 0);
//	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	

	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 320, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 480, 0, 0);	
#endif
//xiongbiao@wind-mobi.com 2011.12.15 end

	input_set_drvdata(input_dev, data);

	i2c_set_clientdata(client, data);

	if (data->pdata->platform_init)
		data->pdata->platform_init();

	error = mxt140_initialize(data);
	if (error)
		goto err_free_object;

	data->pen_down = 0;

#ifdef USE_BRCM_WORK_QUEUE
	INIT_DELAYED_WORK(&data->work, mxt140_touch_work);

	error = request_irq(client->irq, mxt140_touch_interrupt,
			    IRQF_TRIGGER_FALLING, "mxt140_ts", data);

	if (error < 0) {
		printk("mxt140_touch: Failed to register interrupt \r\n");
		goto err_free_object;
	}
#else
	error = request_threaded_irq(client->irq, NULL, mxt140_interrupt,
				     IRQF_TRIGGER_FALLING,
				     client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
#endif

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt140_attr_group);
	if (error)
		goto err_unregister_device;
//xiongbiao@wind-mobi.com 2011.12.15 start
//add
     mxt140_start(data);
//xiongbiao@wind-mobi.com 2011.12.15 end 
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	data->suspend_desc.suspend = mxt140_early_suspend,
	data->suspend_desc.resume = mxt140_late_resume,
	register_early_suspend(&data->suspend_desc);
#endif

{
    struct kobject *properties_kobj;
    int reply;
    properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		reply = sysfs_create_group(properties_kobj,
					 &acar_properties_attr_group);
	if (!properties_kobj || reply)
		pr_err("failed to create board_properties\n");

}

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
       if (data->pdata->platform_exit)
               data->pdata->platform_exit();
#ifdef VKEY_SUPPORT
    if (data->pk_data)
    {
         kfree(data->pk_data);
    }
#endif
	kfree(data);

	return error;
}

static int __devexit mxt140_remove(struct i2c_client *client)
{
	struct mxt140_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &mxt140_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

void mxt140_shutdown(struct i2c_client *client)
{
	struct mxt140_data *data = i2c_get_clientdata(client);
	/* Soft reset  controller here to prevent it from
	 *getting into an un-responsive state */
	mxt140_write_object(data, MXT140_GEN_COMMAND,
			      MXT140_COMMAND_RESET, 1);
	msleep(MXT140_RESET_TIME);

}

static const struct i2c_device_id mxt140_id[] = {
	{"mxt140_ts", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mxt140_id);

static struct i2c_driver mxt140_driver = {
	.driver = {
		   .name = "mxt140_ts",
		   .owner = THIS_MODULE,
		   },
	.probe = mxt140_probe,
	.remove = __devexit_p(mxt140_remove),
	.shutdown = mxt140_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = mxt140_suspend,
	.resume = mxt140_resume,
#endif
	.id_table = mxt140_id,
};

static int __init mxt140_init(void)
{
	return i2c_add_driver(&mxt140_driver);
}

static void __exit mxt140_exit(void)
{
	i2c_del_driver(&mxt140_driver);
}

module_init(mxt140_init);
module_exit(mxt140_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("MXT140 Touchscreen driver");
MODULE_LICENSE("GPL");
