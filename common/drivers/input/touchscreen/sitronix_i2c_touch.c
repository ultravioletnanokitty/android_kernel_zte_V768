/*
 * drivers/input/touchscreen/sitronix_i2c_touch.c
 *
 * Touchscreen driver for Sitronix (I2C bus)
 *
 * Copyright (C) 2011 Sitronix Technology Co., Ltd.
 *	Rudy Huang <rudy_huang@sitronix.com.tw>
 */
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */
#define SITRONIX_FW_UPGRADE_FEATURE

#include <linux/module.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif // CONFIG_HAS_EARLYSUSPEND
#ifdef SITRONIX_FW_UPGRADE_FEATURE
#include <linux/cdev.h>
#include <asm/uaccess.h>
#endif // SITRONIX_FW_UPGRADE_FEATURE
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sitronix_i2c_touch.h>

#define DRIVER_AUTHOR           "Sitronix, Inc."
#define DRIVER_NAME             "sitronix"
#define DRIVER_DESC             "Sitronix I2C touch"
#define DRIVER_DATE             "20120105"
#define DRIVER_MAJOR            2
#define DRIVER_MINOR         	6
#define DRIVER_PATCHLEVEL       0

MODULE_AUTHOR("Rudy Huang <rudy_huang@sitronix.com.tw>");
MODULE_DESCRIPTION("Sitronix I2C multitouch panels");
MODULE_LICENSE("GPL");

#ifdef SITRONIX_SENSOR_KEY
#define SITRONIX_NUMBER_SENSOR_KEY 4
int sitronix_sensor_key[SITRONIX_NUMBER_SENSOR_KEY] = {
	KEY_MENU, // bit 2
	KEY_HOME, // bit 1
	KEY_BACK, // bit 0
	KEY_SEARCH,
};
#endif // SITRONIX_SENSOR_KEY

#ifdef SITRONIX_TOUCH_KEY
#define SITRONIX_NUMBER_TOUCH_KEY 4

#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY 
#define SITRONIX_TOUCH_RESOLUTION_X 480 /* max of X value in display area */
#define SITRONIX_TOUCH_RESOLUTION_Y 854 /* max of Y value in display area */
#define SITRONIX_TOUCH_GAP_Y	10  /* Gap between bottom of display and top of touch key */
#define SITRONIX_TOUCH_MAX_Y 915  /* resolution of y axis of touch ic */ 
struct sitronix_AA_key sitronix_key_array[SITRONIX_NUMBER_TOUCH_KEY] = {
	{15, 105, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_MENU}, /* MENU */
	{135, 225, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_HOME},
	{255, 345, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_BACK}, /* KEY_EXIT */
	{375, 465, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_SEARCH},
};
#else
#define SCALE_KEY_HIGH_Y 15
struct sitronix_AA_key sitronix_key_array[SITRONIX_NUMBER_TOUCH_KEY] = {
	{0, 0, 0, 0, KEY_MENU}, /* MENU */
	{0, 0, 0, 0, KEY_HOME},
	{0, 0, 0, 0, KEY_BACK}, /* KEY_EXIT */
	{0, 0, 0, 0, KEY_SEARCH},
};
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#endif // SITRONIX_TOUCH_KEY
struct sitronix_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
#if defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
 	struct input_dev *keyevent_input;
#endif // defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	int (*get_int_status)(void);
	void (*reset_ic)(void);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif // CONFIG_HAS_EARLYSUSPEND
	uint8_t fw_revision[4];
	int resolution_x;
	int resolution_y;
	uint8_t max_touches;
	uint8_t touch_protocol_type;
	uint8_t pixel_length;
	int suspend_state;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sitronix_ts_early_suspend(struct early_suspend *h);
static void sitronix_ts_late_resume(struct early_suspend *h);
#endif // CONFIG_HAS_EARLYSUSPEND

static MTD_STRUCTURE sitronix_ts_gMTDPreStructure[SITRONIX_MAX_SUPPORTED_POINT]={{0}};

static struct sitronix_ts_data *sitronix_ts_gpts = NULL;
static int Is_Sensor_key = 0;

#ifdef SITRONIX_FW_UPGRADE_FEATURE
int      sitronix_release(struct inode *, struct file *);
int      sitronix_open(struct inode *, struct file *);
ssize_t  sitronix_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
ssize_t  sitronix_read(struct file *file, char *buf, size_t count, loff_t *ppos);
long	 sitronix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static struct cdev sitronix_cdev;
static struct class *sitronix_class;
static int sitronix_major = 0;

int  sitronix_open(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(sitronix_open);

int  sitronix_release(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(sitronix_release);

ssize_t  sitronix_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int ret;
	char *tmp;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}
	UpgradeMsg("writing %zu bytes.\n", count);

	ret = i2c_master_send(sitronix_ts_gpts->client, tmp, count);
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(sitronix_write);

ssize_t  sitronix_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *tmp;
	int ret;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

	UpgradeMsg("reading %zu bytes.\n", count);

	ret = i2c_master_recv(sitronix_ts_gpts->client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf,tmp,count)?-EFAULT:ret;
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(sitronix_read);

long	 sitronix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	uint8_t temp[4];

	if (_IOC_TYPE(cmd) != SMT_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SMT_IOC_MAXNR) return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user *)arg,\
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ,(void __user *)arg,\
				  _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch(cmd) {
		case IOCTL_SMT_GET_DRIVER_REVISION:
			UpgradeMsg("IOCTL_SMT_GET_DRIVER_REVISION\n");
			temp[0] = SITRONIX_TOUCH_DRIVER_VERSION;
			if(copy_to_user((uint8_t __user *)arg, &temp[0], 1)){
				UpgradeMsg("fail to get driver version\n");
				retval = -EFAULT;
			}
			break;
		case IOCTL_SMT_GET_FW_REVISION:
			UpgradeMsg("IOCTL_SMT_GET_FW_REVISION\n");
			if(copy_to_user((uint8_t __user *)arg, &sitronix_ts_gpts->fw_revision[0], 4))
					retval = -EFAULT;
			break;
		case IOCTL_SMT_ENABLE_IRQ:
			UpgradeMsg("IOCTL_SMT_ENABLE_IRQ\n");
			enable_irq(sitronix_ts_gpts->client->irq);
			break;
		case IOCTL_SMT_DISABLE_IRQ:
			UpgradeMsg("IOCTL_SMT_DISABLE_IRQ\n");
			disable_irq_nosync(sitronix_ts_gpts->client->irq);
			break;
		default:
			retval = -ENOTTY;
	}

	return retval;
}
EXPORT_SYMBOL(sitronix_ioctl);
#endif // SITRONIX_FW_UPGRADE_FEATURE
static int sitronix_get_fw_version(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];

	buffer[0] = FIRMWARE_VERSION;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("send fw version command error (%d)\n", ret);
		return ret;
	}
	ret = i2c_master_recv(ts->client, buffer, 1);
	if (ret < 0){
		printk("read fw version error (%d)\n", ret);
		return ret;
	}else{
		DbgMsg("fw version = 0x%x \n", buffer[0]); 
	}
	if((buffer[0] & 0xe0) == 0x20)	
		Is_Sensor_key = 1;
	
	return 0;
}

static int sitronix_get_fw_revision(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[4];

	buffer[0] = FIRMWARE_REVISION_3;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("send fw revision command error (%d)\n", ret);
		return ret;
	}
	ret = i2c_master_recv(ts->client, buffer, 4);
	if (ret < 0){
		printk("read fw revision error (%d)\n", ret);
		return ret;
	}else{
		memcpy(ts->fw_revision, buffer, 4);
		DbgMsg("fw revision (hex) = %x %x %x %x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	}
	return 0;
}
static int sitronix_get_max_touches(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];

	buffer[0] = MAX_NUM_TOUCHES;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("send max touches command error (%d)\n", ret);
		return ret;
	}
	ret = i2c_master_recv(ts->client, buffer, 1);
	if (ret < 0){
		printk("read max touches error (%d)\n", ret);
		return ret;
	}else{
		ts->max_touches = buffer[0];
		DbgMsg("max touches = %d \n",ts->max_touches); 
	}
	return 0;
}

static int sitronix_get_protocol_type(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];

	buffer[0] = I2C_PROTOCOL;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("send i2c protocol command error (%d)\n", ret);
		return ret;
	}
	ret = i2c_master_recv(ts->client, buffer, 1);
	if (ret < 0){
		printk("read i2c protocol error (%d)\n", ret);
		return ret;
	}else{
		ts->touch_protocol_type = buffer[0] & I2C_PROTOCOL_BMSK;
		if(ts->touch_protocol_type == SITRONIX_A_TYPE)
			ts->pixel_length = PIXEL_DATA_LENGTH_A;
		else if(ts->touch_protocol_type == SITRONIX_B_TYPE)
			ts->pixel_length = PIXEL_DATA_LENGTH_B;
		else
			ts->pixel_length = PIXEL_DATA_LENGTH_A;	
		DbgMsg("i2c protocol = %d \n", ts->touch_protocol_type);
	}
	return 0;
}

static int sitronix_get_resolution(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[3];

	buffer[0] = XY_RESOLUTION_HIGH;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("send resolution command error (%d)\n", ret);
		return ret;
	}
	ret = i2c_master_recv(ts->client, buffer, 3);
	if (ret < 0){
		printk("read resolution error (%d)\n", ret);
		return ret;
	}else{
		ts->resolution_x = ((buffer[0] & (X_RES_H_BMSK << X_RES_H_SHFT)) << 4) | buffer[1]; 
		ts->resolution_y = ((buffer[0] & Y_RES_H_BMSK) << 8) | buffer[2];
		DbgMsg("resolution = %d x %d\n", ts->resolution_x, ts->resolution_y);
	}
	return 0;
}

static int sitronix_ts_set_powerdown_bit(struct sitronix_ts_data *ts, int value)
{
	int ret = 0;
	uint8_t buffer[2];

	DbgMsg("%s, value = %d\n", __FUNCTION__, value);
	buffer[0] = DEVICE_CONTROL_REG;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("send device control command error (%d)\n", ret);
		return ret;
	}
		
	ret = i2c_master_recv(ts->client, buffer, 1);
	if (ret < 0){
		printk("read device control status error (%d)\n", ret);
		return ret;
	}else{
		DbgMsg("dev status = %d \n", buffer[0]);
	}
	
	if(value == 0)
		buffer[0] &= 0xfd;
	else
		buffer[0] |= 0x2;
		
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0){
		printk("write power down error (%d)\n", ret);
		return ret;
	}
	
	return 0;
}

static void sitronix_ts_get_touch_info(struct sitronix_ts_data *ts)
{
	sitronix_get_resolution(ts);
	sitronix_get_fw_revision(ts);
	if((ts->fw_revision[0] == 0) && (ts->fw_revision[1] == 0)){
		ts->touch_protocol_type = SITRONIX_B_TYPE;
		ts->pixel_length = PIXEL_DATA_LENGTH_B;	
		ts->max_touches = 2;
	}else{
		sitronix_get_protocol_type(ts);
		if(ts->touch_protocol_type == SITRONIX_B_TYPE){
			ts->max_touches = 2;
		}else
			sitronix_get_max_touches(ts);
	}
}

static void sitronix_ts_work_func(struct work_struct *work)
{
	int i;
#ifdef SITRONIX_TOUCH_KEY
	int j;
#endif // SITRONIX_TOUCH_KEY
	int ret;
	struct sitronix_ts_data *ts = container_of(work, struct sitronix_ts_data, work);
	uint8_t buffer[2+ SITRONIX_MAX_SUPPORTED_POINT * PIXEL_DATA_LENGTH_A];
	//uint8_t PixelCount = 0;
	static MTD_STRUCTURE MTDStructure[SITRONIX_MAX_SUPPORTED_POINT]={{0}};

	DbgMsg("%s\n",  __FUNCTION__);
	if(ts->get_int_status)
		if(ts->get_int_status())
			goto exit_invalid_data;

#if 0
	// get finger count
	buffer[0] = FINGERS;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0)
		printk("send finger command error (%d)\n", ret);
	ret = i2c_master_recv(ts->client, buffer, 1);
	if (ret < 0) {
		printk("read finger error (%d)\n", ret);
		goto exit_invalid_data;
	}else{
		PixelCount = buffer[0] & FINGERS_BMSK ;
		printk("fingers = %d\n", PixelCount);
	}
#endif
#ifdef SITRONIX_SENSOR_KEY
if(Is_Sensor_key == 1){
	buffer[0] = KEYS_REG;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0)
		printk("send key command error (%d)\n", ret);
	ret = i2c_master_recv(ts->client, buffer, 1);
	if (ret < 0) {
		printk("read key error (%d)\n", ret);
		goto exit_invalid_data;
	}else{
		DbgMsg("key = 0x%x\n", buffer[0]);
	}
	for(i = 0; i < SITRONIX_NUMBER_SENSOR_KEY; i++){
		if(buffer[0] & (1 << i)){
			DbgMsg("key[%d] down\n", i);
			input_report_key(ts->keyevent_input, sitronix_sensor_key[i], 1);
		}else{
			DbgMsg("key[%d] up\n", i);
			input_report_key(ts->keyevent_input, sitronix_sensor_key[i], 0);
		}
	}
}
#endif // SITRONIX_SENSOR_KEY
	buffer[0] = XY0_COORD_H;
	ret = i2c_master_send(ts->client, buffer, 1);
	if (ret < 0)
		printk("send coordination command error (%d)\n", ret);

	ret = i2c_master_recv(ts->client, buffer, ts->max_touches * ts->pixel_length);
	if (ret < 0) {
		printk("read coordination error (%d)\n", ret);
		goto exit_invalid_data;
	}

	for(i = 0; i < ts->max_touches; i++){
		MTDStructure[i].Pixel_X = ((buffer[ts->pixel_length * i] & (X_COORD_H_BMSK << X_COORD_H_SHFT)) << 4) |  (buffer[ts->pixel_length * i + X_COORD_L]);
		MTDStructure[i].Pixel_Y = ((buffer[ts->pixel_length * i] & Y_COORD_H_BMSK) << 8) |  (buffer[ts->pixel_length * i + Y_COORD_L]);
//#ifndef SITRONIX_TOUCH_KEY
if(Is_Sensor_key == 1){
		if((buffer[ts->pixel_length * i] >> X_COORD_VALID_SHFT) == 1)
			MTDStructure[i].Current_Pressed_area = AREA_DISPLAY;
		else
			MTDStructure[i].Current_Pressed_area = AREA_NONE;
//#else
}else{
		if((buffer[ts->pixel_length * i] >> X_COORD_VALID_SHFT) == 1){
			MTDStructure[i].Current_Pressed_area = AREA_INVALID;
#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
			if((MTDStructure[i].Pixel_X < ts->resolution_x) && (MTDStructure[i].Pixel_Y < sitronix_key_array[0].y_low)){
#else
            if((MTDStructure[i].Pixel_X < ts->resolution_x) && (MTDStructure[i].Pixel_Y < (ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y))){
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
				MTDStructure[i].Current_Pressed_area = AREA_DISPLAY;
			}else{
				for(j = 0; j < SITRONIX_NUMBER_TOUCH_KEY; j++){
					if((MTDStructure[i].Pixel_X >= sitronix_key_array[j].x_low) && 
					(MTDStructure[i].Pixel_X <= sitronix_key_array[j].x_high) && 
					(MTDStructure[i].Pixel_Y >= sitronix_key_array[j].y_low) && 
					(MTDStructure[i].Pixel_Y <= sitronix_key_array[j].y_high)){
						MTDStructure[i].Current_Pressed_area = AREA_KEY;
						MTDStructure[i].Current_key_index = j;
						break;
					}
				}
			}
		}else{
			MTDStructure[i].Current_Pressed_area = AREA_NONE;
		}
//#endif // SITRONIX_TOUCH_KEY
}
		
	}
	for(i = 0; i < ts->max_touches; i++){
//#ifndef SITRONIX_TOUCH_KEY
if(Is_Sensor_key == 1){
		input_report_abs(ts->input_dev,  ABS_MT_TRACKING_ID, i);
		input_report_abs(ts->input_dev,  ABS_MT_POSITION_X, MTDStructure[i].Pixel_X);
		input_report_abs(ts->input_dev,  ABS_MT_POSITION_Y, MTDStructure[i].Pixel_Y);
		
		if(MTDStructure[i].Current_Pressed_area == AREA_DISPLAY){
			input_report_abs(ts->input_dev,  ABS_MT_TOUCH_MAJOR, 1);
			DbgMsg("[%d](%d, %d)+\n", i, MTDStructure[i].Pixel_X, MTDStructure[i].Pixel_Y);
			input_mt_sync(ts->input_dev);
		//}else if((MTDStructure[i].Current_Pressed_area == AREA_NONE)&&(sitronix_ts_gMTDPreStructure[i].Current_Pressed_area == AREA_DISPLAY)){
		}else if(MTDStructure[i].Current_Pressed_area == AREA_NONE){
			input_report_abs(ts->input_dev,  ABS_MT_TOUCH_MAJOR, 0);
			DbgMsg("[%d](%d, %d)-\n", i, MTDStructure[i].Pixel_X, MTDStructure[i].Pixel_Y);
			input_mt_sync(ts->input_dev);
		}
		memcpy(&sitronix_ts_gMTDPreStructure[i], &MTDStructure[i], sizeof(MTD_STRUCTURE));
//#else
}else{
		if(sitronix_ts_gMTDPreStructure[i].First_Pressed_area == AREA_NONE){
			if(MTDStructure[i].Current_Pressed_area == AREA_DISPLAY){
				input_report_abs(ts->input_dev,  ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input_dev,  ABS_MT_POSITION_X, MTDStructure[i].Pixel_X);
				input_report_abs(ts->input_dev,  ABS_MT_POSITION_Y, MTDStructure[i].Pixel_Y);
				input_report_abs(ts->input_dev,  ABS_MT_TOUCH_MAJOR, 1);
				input_mt_sync(ts->input_dev);
				sitronix_ts_gMTDPreStructure[i].First_Pressed_area = AREA_DISPLAY;
				DbgMsg("[%d](%d, %d)\n", i, MTDStructure[i].Pixel_X, MTDStructure[i].Pixel_Y);
			}else if(MTDStructure[i].Current_Pressed_area == AREA_KEY){
				sitronix_ts_gMTDPreStructure[i].First_Pressed_area = AREA_KEY;
				sitronix_ts_gMTDPreStructure[i].First_key_index = MTDStructure[i].Current_key_index;
				input_report_key(ts->keyevent_input, sitronix_key_array[MTDStructure[i].Current_key_index].code, 1);
				DbgMsg("key [%d] down\n", MTDStructure[i].Current_key_index);
			}
		}else if(sitronix_ts_gMTDPreStructure[i].First_Pressed_area == AREA_DISPLAY){
			if(MTDStructure[i].Current_Pressed_area == AREA_DISPLAY){
				input_report_abs(ts->input_dev,  ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input_dev,  ABS_MT_POSITION_X, MTDStructure[i].Pixel_X);
				input_report_abs(ts->input_dev,  ABS_MT_POSITION_Y, MTDStructure[i].Pixel_Y);
				input_report_abs(ts->input_dev,  ABS_MT_TOUCH_MAJOR, 1);
				input_mt_sync(ts->input_dev);
				DbgMsg("[%d](%d, %d)+\n", i, MTDStructure[i].Pixel_X, MTDStructure[i].Pixel_Y);
			}else if(MTDStructure[i].Current_Pressed_area == AREA_NONE){ 
				input_report_abs(ts->input_dev,  ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input_dev,  ABS_MT_POSITION_X, MTDStructure[i].Pixel_X);
				input_report_abs(ts->input_dev,  ABS_MT_POSITION_Y, MTDStructure[i].Pixel_Y);
				input_report_abs(ts->input_dev,  ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(ts->input_dev);
				DbgMsg("[%d](%d, %d)-\n", i, MTDStructure[i].Pixel_X, MTDStructure[i].Pixel_Y);
				sitronix_ts_gMTDPreStructure[i].First_Pressed_area = AREA_NONE;
			}
		}else if(sitronix_ts_gMTDPreStructure[i].First_Pressed_area == AREA_KEY){
			if(MTDStructure[i].Current_Pressed_area == AREA_KEY){ 
				if(sitronix_ts_gMTDPreStructure[i].First_key_index == MTDStructure[i].Current_key_index){
					input_report_key(ts->keyevent_input, sitronix_key_array[sitronix_ts_gMTDPreStructure[i].First_key_index].code, 1);
					DbgMsg("key [%d] down+\n", MTDStructure[i].Current_key_index);
				}
			}else if(MTDStructure[i].Current_Pressed_area == AREA_DISPLAY){ 
				input_report_key(ts->keyevent_input, sitronix_key_array[MTDStructure[i].Current_key_index].code, 0);
				DbgMsg("key [%d] up\n", MTDStructure[i].Current_key_index);
				//sitronix_ts_gMTDPreStructure[i].First_Pressed_area = AREA_NONE;
			}else if(MTDStructure[i].Current_Pressed_area == AREA_NONE){ 
				input_report_key(ts->keyevent_input, sitronix_key_array[MTDStructure[i].Current_key_index].code, 0);
				DbgMsg("key [%d] up\n", MTDStructure[i].Current_key_index);
				sitronix_ts_gMTDPreStructure[i].First_Pressed_area = AREA_NONE;
			}
		}
		//sitronix_ts_gMTDPreStructure[i].Pixel_X = MTDStructure[i].Pixel_X;
		//sitronix_ts_gMTDPreStructure[i].Pixel_Y = MTDStructure[i].Pixel_Y;
		//sitronix_ts_gMTDPreStructure[i].Current_Pressed_area = MTDStructure[i].Current_Pressed_area;
		//sitronix_ts_gMTDPreStructure[i].Current_key_index = MTDStructure[i].Current_key_index;
//#endif // SITRONIX_TOUCH_KEY
}
	}
	input_sync(ts->input_dev);

exit_invalid_data:
#ifdef SITRONIX_LEVEL_TRIGGERED
	if (ts->use_irq)
		enable_irq(ts->client->irq);
#endif // SITRONIX_LEVEL_TRIGGERED
;
}


static irqreturn_t sitronix_ts_irq_handler(int irq, void *dev_id)
{
	struct sitronix_ts_data *ts = dev_id;

	if(!ts->suspend_state){
#ifdef SITRONIX_LEVEL_TRIGGERED
		disable_irq_nosync(ts->client->irq);
#endif // SITRONIX_LEVEL_TRIGGERED
		schedule_work(&ts->work);
	}
	return IRQ_HANDLED;
}

static int sitronix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#if defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	int i;
#endif // defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	struct sitronix_ts_data *ts;
	int ret = 0;
	uint16_t max_x = 0, max_y = 0;
	struct sitronix_i2c_touch_platform_data *pdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, sitronix_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (pdata->init_irq){
		ret = pdata->init_irq();
		if (ret < 0) {
			dev_err(&client->dev, "failed to initialize IRQ#%d: "
			"%d\n", client->irq, ret);
			goto err_init_irq_failed;
		}
	}
	if(pdata->get_int_status)
		ts->get_int_status = pdata->get_int_status;
	if(pdata->reset_ic){
		ts->reset_ic = pdata->reset_ic;
		pdata->reset_ic();
		mdelay(SITRONIX_TS_CHANGE_MODE_DELAY);
	}
	sitronix_ts_gpts = kzalloc(sizeof(*ts), GFP_KERNEL);

	ret =sitronix_get_fw_version(ts);
	if (ret < 0) {
		dev_err(&client->dev, "failed to sitronix_get_fw_version: %d\n", ret);
		goto err_init_irq_failed;
	}
	sitronix_ts_get_touch_info(ts);
	memcpy(sitronix_ts_gpts, ts, sizeof(*ts));

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL){
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sitronix-i2c-touch-mt";
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

#if defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	ts->keyevent_input = input_allocate_device();
	if (ts->keyevent_input == NULL){
		printk("Can not allocate memory for key input device.");
		goto err_input_dev_alloc_failed;
	}
	ts->keyevent_input->name  = "sitronix-i2c-touch-key";
	set_bit(EV_KEY, ts->keyevent_input->evbit);
#endif // defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
#if defined(SITRONIX_SENSOR_KEY)
if(Is_Sensor_key == 1){
	for(i = 0; i < SITRONIX_NUMBER_SENSOR_KEY; i++){
		set_bit(sitronix_sensor_key[i], ts->keyevent_input->keybit);
	}
}
#endif // defined(SITRONIX_SENSOR_KEY)

//#ifndef SITRONIX_TOUCH_KEY
if(Is_Sensor_key == 1){
	max_x = ts->resolution_x;
	max_y = ts->resolution_y;
//#else
}else{
#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
	for(i = 0; i < SITRONIX_NUMBER_TOUCH_KEY; i++){
		set_bit(sitronix_key_array[i].code, ts->keyevent_input->keybit);
	}
	max_x = SITRONIX_TOUCH_RESOLUTION_X;
	max_y = SITRONIX_TOUCH_RESOLUTION_Y;
#else
	for(i = 0; i < SITRONIX_NUMBER_TOUCH_KEY; i++){
		sitronix_key_array[i].x_low = ((ts->resolution_x / SITRONIX_NUMBER_TOUCH_KEY ) * i ) + 15;
		sitronix_key_array[i].x_high = ((ts->resolution_x / SITRONIX_NUMBER_TOUCH_KEY ) * (i + 1)) - 15;
		sitronix_key_array[i].y_low = ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y;
		sitronix_key_array[i].y_high = ts->resolution_y;
		DbgMsg("key[%d] %d, %d, %d, %d\n", i, sitronix_key_array[i].x_low, sitronix_key_array[i].x_high, sitronix_key_array[i].y_low, sitronix_key_array[i].y_high);
		set_bit(sitronix_key_array[i].code, ts->keyevent_input->keybit);
       
	}
	max_x = ts->resolution_x;
	max_y = ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y;
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
//#endif // SITRONIX_TOUCH_KEY
}
#if defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	if(input_register_device(ts->keyevent_input)){
		printk("Can not register key input device.");
		goto err_input_register_device_failed;
	}
#endif // defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)

	__set_bit(ABS_X, ts->input_dev->absbit);
	__set_bit(ABS_Y, ts->input_dev->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	__set_bit(ABS_MT_TOOL_TYPE, ts->input_dev->absbit);
	__set_bit(ABS_MT_BLOB_ID, ts->input_dev->absbit);
	__set_bit(ABS_MT_TRACKING_ID, ts->input_dev->absbit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0,  255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touches, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	
	if(input_register_device(ts->input_dev)){
		printk("Can not register input device.");
		goto err_input_register_device_failed;
	}

	ts->suspend_state = 0;
	if (client->irq){
#ifdef SITRONIX_LEVEL_TRIGGERED
		ret = request_irq(client->irq, sitronix_ts_irq_handler, IRQF_TRIGGER_LOW | IRQF_DISABLED, client->name, ts);
#else
		ret = request_irq(client->irq, sitronix_ts_irq_handler, IRQF_TRIGGER_FALLING | IRQF_DISABLED, client->name, ts);
#endif // SITRONIX_LEVEL_TRIGGERED
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
        ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        ts->early_suspend.suspend = sitronix_ts_early_suspend;
        ts->early_suspend.resume = sitronix_ts_late_resume;
        register_early_suspend(&ts->early_suspend);
#endif // CONFIG_HAS_EARLYSUSPEND
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);
#if defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	input_free_device(ts->keyevent_input);
#endif // defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
err_input_dev_alloc_failed:
err_init_irq_failed:
	if (pdata->exit_ic){
		pdata->exit_ic();
	}
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int sitronix_ts_remove(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);
	struct sitronix_i2c_touch_platform_data *pdata=client->dev.platform_data;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif // CONFIG_HAS_EARLYSUSPEND
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
#if defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	input_unregister_device(ts->keyevent_input);
#endif // defined(SITRONIX_SENSOR_KEY) || defined (SITRONIX_TOUCH_KEY)
	if (pdata->exit_ic){
		pdata->exit_ic();
	}

	kfree(ts);
	return 0;
}

static int sitronix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);

	DbgMsg("%s\n", __FUNCTION__);
	disable_irq(client->irq);
	ts->suspend_state = 1;

	ret = sitronix_ts_set_powerdown_bit(ts, 1);
	DbgMsg("%s return\n", __FUNCTION__);

	return 0;
}

static int sitronix_ts_resume(struct i2c_client *client)
{
	int ret;
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);

	DbgMsg("%s\n", __FUNCTION__);

	ret = sitronix_ts_set_powerdown_bit(ts, 0);
	ts->suspend_state = 0;

	if (ts->use_irq)
		enable_irq(client->irq);
	DbgMsg("%s return\n", __FUNCTION__);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sitronix_ts_early_suspend(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void sitronix_ts_late_resume(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_resume(ts->client);
}
#endif // CONFIG_HAS_EARLYSUSPEND

static const struct i2c_device_id sitronix_ts_id[] = {
	{ SITRONIX_I2C_TOUCH_DRV_NAME, 0 },
	{ }
};

static struct i2c_driver sitronix_ts_driver = {
	.probe		= sitronix_ts_probe,
	.remove		= sitronix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= sitronix_ts_suspend,
	.resume		= sitronix_ts_resume,
#endif // CONFIG_HAS_EARLYSUSPEND
	.id_table	= sitronix_ts_id,
	.driver = {
		.name	= SITRONIX_I2C_TOUCH_DRV_NAME,
	},
};

#ifdef SITRONIX_FW_UPGRADE_FEATURE
static struct file_operations nc_fops = {
	.owner =        THIS_MODULE,
	.write		= sitronix_write,
	.read		= sitronix_read,
	.open		= sitronix_open,
	.unlocked_ioctl = sitronix_ioctl,
	.release	= sitronix_release,
};
#endif // SITRONIX_FW_UPGRADE_FEATURE

static int __devinit sitronix_ts_init(void)
{
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	int result;
	int err = 0;
	dev_t devno = MKDEV(sitronix_major, 0);
	result  = alloc_chrdev_region(&devno, 0, 1, SITRONIX_I2C_TOUCH_DEV_NAME);
	if(result < 0){
		printk("fail to allocate chrdev (%d) \n", result);
		return 0;
	}
	sitronix_major = MAJOR(devno);
        cdev_init(&sitronix_cdev, &nc_fops);
	sitronix_cdev.owner = THIS_MODULE;
	sitronix_cdev.ops = &nc_fops;
        err =  cdev_add(&sitronix_cdev, devno, 1);
	if(err){
		printk("fail to add cdev (%d) \n", err);
		return 0;
	}

	sitronix_class = class_create(THIS_MODULE, SITRONIX_I2C_TOUCH_DEV_NAME);
	if (IS_ERR(sitronix_class)) {
		result = PTR_ERR(sitronix_class);
		unregister_chrdev(sitronix_major, SITRONIX_I2C_TOUCH_DEV_NAME);
		printk("fail to create class (%d) \n", result);
		return result;
	}
	device_create(sitronix_class, NULL, MKDEV(sitronix_major, 0), NULL, SITRONIX_I2C_TOUCH_DEV_NAME);
#endif // SITRONIX_FW_UPGRADE_FEATURE
	return i2c_add_driver(&sitronix_ts_driver);
}

static void __exit sitronix_ts_exit(void)
{
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	dev_t dev_id = MKDEV(sitronix_major, 0);
#endif // SITRONIX_FW_UPGRADE_FEATURE
	i2c_del_driver(&sitronix_ts_driver);
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	cdev_del(&sitronix_cdev); 

	device_destroy(sitronix_class, dev_id); //delete device node under /dev
	class_destroy(sitronix_class); //delete class created by us
	unregister_chrdev_region(dev_id, 1);
#endif // SITRONIX_FW_UPGRADE_FEATURE
}

module_init(sitronix_ts_init);
module_exit(sitronix_ts_exit);

MODULE_DESCRIPTION("Sitronix Multi-Touch Driver");
MODULE_LICENSE("GPL");
