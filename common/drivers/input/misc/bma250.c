/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/input/misc/bma250.c
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/bma250.h>
#include <linux/delay.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* uncomment to enable interrupt based event generation */
/* #define BMA250_ACCL_IRQ_MODE */
#define ACCL_NAME			"bma250_accl"
#define ACCL_VENDORID			0x0001

#define LOW_G_THRES			5
#define LOW_G_DUR			50
#define HIGH_G_THRES			10
#define HIGH_G_DUR			1
#define ANY_MOTION_THRES		1
#define ANY_MOTION_CT			1
 
//modify for  test by jerayli  12-02-21


#define BMA250_CHIP_ID_REG                      0x00
#define BMA250_VERSION_REG                      0x01
#define BMA250_X_AXIS_LSB_REG                   0x02
#define BMA250_X_AXIS_MSB_REG                   0x03
#define BMA250_Y_AXIS_LSB_REG                   0x04
#define BMA250_Y_AXIS_MSB_REG                   0x05
#define BMA250_Z_AXIS_LSB_REG                   0x06
#define BMA250_Z_AXIS_MSB_REG                   0x07

#define BMA250_ACC_X_LSB__POS           6
#define BMA250_ACC_X_LSB__LEN           2
#define BMA250_ACC_X_LSB__MSK           0xC0
#define BMA250_ACC_X_LSB__REG           BMA250_X_AXIS_LSB_REG

#define BMA250_ACC_X_MSB__POS           0
#define BMA250_ACC_X_MSB__LEN           8
#define BMA250_ACC_X_MSB__MSK           0xFF
#define BMA250_ACC_X_MSB__REG           BMA250_X_AXIS_MSB_REG

#define BMA250_ACC_Y_LSB__POS           6
#define BMA250_ACC_Y_LSB__LEN           2
#define BMA250_ACC_Y_LSB__MSK           0xC0
#define BMA250_ACC_Y_LSB__REG           BMA250_Y_AXIS_LSB_REG

#define BMA250_ACC_Y_MSB__POS           0
#define BMA250_ACC_Y_MSB__LEN           8
#define BMA250_ACC_Y_MSB__MSK           0xFF
#define BMA250_ACC_Y_MSB__REG           BMA250_Y_AXIS_MSB_REG

#define BMA250_ACC_Z_LSB__POS           6
#define BMA250_ACC_Z_LSB__LEN           2
#define BMA250_ACC_Z_LSB__MSK           0xC0
#define BMA250_ACC_Z_LSB__REG           BMA250_Z_AXIS_LSB_REG

#define BMA250_ACC_Z_MSB__POS           0
#define BMA250_ACC_Z_MSB__LEN           8
#define BMA250_ACC_Z_MSB__MSK           0xFF
#define BMA250_ACC_Z_MSB__REG           BMA250_Z_AXIS_MSB_REG


#define BMA250_GET_BITSLICE(regvar, bitname)\
			((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA250_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

//
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("bma250");

struct drv_data {
	struct input_dev *ip_dev;
	struct i2c_client *i2c;
	int irq;
	int bma250_accl_mode;
	char bits_per_transfer;
	struct delayed_work work_data;
	bool config;
	struct list_head next_dd;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend suspend_desc;
#endif
};

struct bma_acc_t {
  short  x,           /**< holds x-axis acceleration data sign extended. Range -512 to 512. */
       y,          /**< holds y-axis acceleration data sign extended. Range -512 to 512. */
       z;          /**< holds z-axis acceleration data sign extended. Range -512 to 512. */
};

static struct mutex bma250_accl_dd_lock;
static struct mutex bma250_accl_wrk_lock;
static struct list_head dd_list;
#ifndef BMA250_ACCL_IRQ_MODE
static struct timer_list bma_wakeup_timer;
#endif
static atomic_t bma_on;
static atomic_t a_flag;
struct bma_acc_t newacc;

static struct i2c_client *bma250_accl_client;

enum bma250_accl_modes {
	SENSOR_DELAY_FASTEST = 0,
	SENSOR_DELAY_GAME = 20,
	SENSOR_DELAY_UI = 60,
	SENSOR_DELAY_NORMAL = 200
};

static inline void bma250_accl_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*      i2c write routine for bma250    */

//modify for  test by jerayli  12-02-21
static inline char bma250_accl_i2c_write(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
#if 0
	s32 dummy;
	if (bma250_accl_client == NULL)	/*      No global client pointer?       */
		return -1;

	while (len--) {
		dummy =
		    i2c_smbus_write_byte_data(bma250_accl_client, reg_addr,
					      *data);
		reg_addr++;
		data++;
		if (dummy < 0)
			return -1;
	}
	return 0;
#endif
}

/*      i2c read routine for bma250     */
static inline char bma250_accl_i2c_read(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;

#if 0
	s32 dummy;
	if (bma250_accl_client == NULL)	/*      No global client pointer?       */
		return -1;

	while (len--) {
		dummy = i2c_smbus_read_byte_data(bma250_accl_client, reg_addr);
		if (dummy < 0)
			return -1;
		*data = dummy & 0x000000ff;
		reg_addr++;
		data++;
	}
	return 0;
#endif
}

static int bma250_accl_power_down(struct drv_data *dd)
{
	u8 data;
	int rc;

	/* For sleep mode set bit 7 of control register (0x11) to 1 */
	data = 0x80;
	rc = i2c_smbus_write_byte_data(dd->i2c, 0x11, data);
	if (rc < 0)
		pr_err("G-Sensor power down failed\n");
	else
		atomic_set(&bma_on, 0);

	return rc;
}

static int bma250_accl_power_up(struct drv_data *dd)
{
	u8 data;
	int rc;

	/* To exit sleep mode set bit 8 of control register (0x11) to 0 */
	data = 0x00;
	rc = i2c_smbus_write_byte_data(dd->i2c, 0x11, data);
	if (rc < 0)
		pr_err("G-Sensor power up failed\n");
	else
		atomic_set(&bma_on, 1);

	return rc;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int bma250_accl_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct drv_data *dd;
	dd = i2c_get_clientdata(client);

#ifdef BMA250_ACCL_IRQ_MODE
	disable_irq(dd->irq);
#else
	del_timer(&bma_wakeup_timer);
	cancel_delayed_work_sync(&dd->work_data);
#endif
	if (atomic_read(&bma_on))
		bma250_accl_power_down(dd);

	return 0;
}

static int bma250_accl_resume(struct i2c_client *client)
{
	struct drv_data *dd;
	dd = i2c_get_clientdata(client);

#ifdef BMA250_ACCL_IRQ_MODE
	enable_irq(dd->irq);
	if (atomic_read(&a_flag))
		bma250_accl_power_up(dd);
#else
	if (atomic_read(&a_flag))
		mod_timer(&bma_wakeup_timer, jiffies + HZ / 1000);
#endif

	return 0;
}

#else
#define bma250_accl_suspend NULL
#define bma250_accl_resume NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_accl_early_suspend(struct early_suspend *desc)
{
	struct drv_data *dd = container_of(desc, struct drv_data, suspend_desc);
	pm_message_t mesg = {
		.event = PM_EVENT_SUSPEND,
		};
	bma250_accl_suspend(dd->i2c, mesg);
}

static void bma250_accl_late_resume(struct early_suspend *desc)
{
	struct drv_data *dd = container_of(desc, struct drv_data, suspend_desc);
	bma250_accl_resume(dd->i2c);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef BMA250_ACCL_IRQ_MODE
static irqreturn_t bma250_accl_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct drv_data *dd;
	long delay;

	dd = dev_get_drvdata(dev);
	delay = dd->bma250_accl_mode * HZ / 1000;
	if (!delay)		/* check for FAST MODE */
		delay = 1;
	schedule_delayed_work(&dd->work_data, delay);

	return IRQ_HANDLED;
}
#else
void bma_wakeup_timer_func(unsigned long data)
{
	struct drv_data *dd;
	long delay = 0;

	dd = (struct drv_data *)data;

	delay = dd->bma250_accl_mode * HZ / 1000;
	/* Set delay >= 2 jiffies to avoid cpu hogging */
	if (delay < 2)
		delay = 2;
	schedule_delayed_work(&dd->work_data, HZ / 1000);
	if (atomic_read(&a_flag))
		mod_timer(&bma_wakeup_timer, jiffies + delay);
}
#endif

static int bma250_accl_open(struct input_dev *dev)
{
	int rc = 0;
#ifdef BMA250_ACCL_IRQ_MODE
	struct drv_data *dd = input_get_drvdata(dev);

	if (!dd->irq)
		return -1;
	/* Timer based implementation */
	/* does not require irq t be enabled */
	rc = request_irq(dd->irq,
			 &bma250_accl_irq, 0, ACCL_NAME, &dd->i2c->dev);
#endif

	return rc;
}

static void bma250_accl_release(struct input_dev *dev)
{
#ifdef BMA250_ACCL_IRQ_MODE
	struct drv_data *dd = input_get_drvdata(dev);

	/* Timer based implementation */
	/* does not require irq t be enabled */
	free_irq(dd->irq, &dd->i2c->dev);
#endif

	return;
}

static int bma250_smbus_read_byte_block(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy;
    dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
    if (dummy < 0)
        return -1;
    return 0;
}

static int bma250_accl_configuration(void)
{
    u8 data;
	int rc;

	/* set register 0x10 '01011b' bandwidth 62.5HZ*/
	data = 0x0b;
	rc = i2c_smbus_write_byte_data(bma250_accl_client, 0x10, data);
	if (rc < 0)
		pr_err("G-Sensor set band width fail\n");

    return rc;

}


//static int bma250_read_accel_xyz(bma_acc_t *acc)
//{
    //int comres;
    //unsigned char data[6];

    //comres = bma250_smbus_read_byte_block(bma250_accl_client, BMA250_X_AXIS_REG, data, 6);
    //acc->x = (signed char)data[0];
   // acc->y = (signed char)data[2];
    //acc->z = (signed char)data[4];

    //printk("get acc xyz [%d][%d][%d] \n",acc->x, acc->y, acc->z);

    //return comres;

//modify  for test by jerayli 12-02-21	
static int bma250_read_accel_xyz(struct i2c_client *client,
							struct bma_acc_t *acc)
{
	int comres;
	unsigned char data[6];
	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte_block(client,
				BMA250_ACC_X_LSB__REG, data, 6);

		acc->x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
			|(BMA250_GET_BITSLICE(data[1],
				BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
		acc->x = acc->x << (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
					+ BMA250_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
					+ BMA250_ACC_X_MSB__LEN));

		acc->y = BMA250_GET_BITSLICE(data[2], BMA250_ACC_Y_LSB)
			| (BMA250_GET_BITSLICE(data[3],
				BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
		acc->y = acc->y << (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
					+ BMA250_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
					+ BMA250_ACC_Y_MSB__LEN));

		acc->z = BMA250_GET_BITSLICE(data[4], BMA250_ACC_Z_LSB)
			| (BMA250_GET_BITSLICE(data[5],
				BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN);
		acc->z = acc->z << (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
					+ BMA250_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
					+ BMA250_ACC_Z_MSB__LEN));
	}

	return comres;
}




static void bma250_accl_getdata(struct drv_data *dd)
{
	struct bma_acc_t acc;
	int X, Y, Z;
	struct bma250_accl_platform_data *pdata = pdata =
	    bma250_accl_client->dev.platform_data;

	mutex_lock(&bma250_accl_wrk_lock);
#ifndef BMA250_ACCL_IRQ_MODE
	if (!atomic_read(&bma_on)) {
		bma250_accl_power_up(dd);
		/* BMA250 need 2 to 3 ms delay */
		/* to give valid data after wakeup */
		msleep(2);
	}
#endif
	bma250_read_accel_xyz(bma250_accl_client,&acc);

	switch (pdata->orientation) {
	case BMA_ROT_90:
		X = -acc.y;
		Y = acc.x;
		Z = acc.z;
		break;
	case BMA_ROT_180:
		X = -acc.x;
		Y = -acc.y;
		Z = acc.z;
		break;
	case BMA_ROT_270:
		X = acc.y;
		Y = -acc.x;
		Z = acc.z;
		break;
	default:
		pr_err("bma250_accl: invalid orientation specified\n");
	case BMA_NO_ROT:
		X = acc.x;
		Y = acc.y;
		Z = acc.z;
		break;
	}
	if (pdata->invert) {
	}
  
//    printk("send to user[%d][%d][%d] \n",X, Y, Z);
	input_report_rel(dd->ip_dev, REL_X, X);
	input_report_rel(dd->ip_dev, REL_Y, Y);
	input_report_rel(dd->ip_dev, REL_Z, Z);
	input_sync(dd->ip_dev);

	newacc.x = X;
	newacc.y = Y;
	newacc.z = Z;

#ifndef BMA250_ACCL_IRQ_MODE
	if (dd->bma250_accl_mode >= SENSOR_DELAY_UI)
		bma250_accl_power_down(dd);
#endif
	mutex_unlock(&bma250_accl_wrk_lock);

	return;

}

static void bma250_accl_work_f(struct work_struct *work)
{
	struct delayed_work *dwork =
	    container_of(work, struct delayed_work, work);
	struct drv_data *dd = container_of(dwork, struct drv_data, work_data);

	    bma250_accl_getdata(dd);

}

static int bma250_accl_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = inode->i_private;

	return 0;
}

static int bma250_accl_misc_ioctl(struct inode *inode, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int delay;
	struct drv_data *dd;
	dd = i2c_get_clientdata(bma250_accl_client);

	switch (cmd) {
	case BMA250_ACCL_IOCTL_GET_DELAY:
		delay = dd->bma250_accl_mode;

		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;

	case BMA250_ACCL_IOCTL_SET_DELAY:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay < 0 || delay > 200)
			return -EINVAL;
		dd->bma250_accl_mode = delay;
		break;
	case BMA250_ACCL_IOCTL_SET_FLAG:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay == 1)
			mod_timer(&bma_wakeup_timer, jiffies + HZ / 1000);
		else if (delay == 0)
			del_timer(&bma_wakeup_timer);
		else
			return -EINVAL;
		atomic_set(&a_flag, delay);
		break;
	case BMA250_ACCL_IOCTL_GET_DATA:
		if (!atomic_read(&a_flag))
			bma250_accl_getdata(dd);
		if (copy_to_user(argp, &newacc, sizeof(newacc)))
			return -EFAULT;
		break;
	}

	return 0;
}

static const struct file_operations bma250_accl_misc_fops = {
	.owner = THIS_MODULE,
	.open = bma250_accl_misc_open,
	.ioctl = bma250_accl_misc_ioctl,
};

static struct miscdevice bma250_accl_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma250_accl",
	.fops = &bma250_accl_misc_fops,
    .mode = 0777,
};

static int __devinit bma250_accl_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct drv_data *dd;
	int rc = 0;
    unsigned char tempvalue;

	struct bma250_accl_platform_data *pdata = pdata =
	    client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality error\n");
		goto probe_exit;
	}
	dd = kzalloc(sizeof(struct drv_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}
	bma250_accl_client = client;

	mutex_lock(&bma250_accl_dd_lock);
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&bma250_accl_dd_lock);
	INIT_DELAYED_WORK(&dd->work_data, bma250_accl_work_f);
	dd->i2c = client;

	if (pdata && pdata->init) {
		rc = pdata->init(&client->dev);
		if (rc)
			goto probe_err_cfg;
	}

    /* check chip id */
    tempvalue = i2c_smbus_read_byte_data(client, BMA250_CHIP_ID_REG);

    if (tempvalue == BMA250_CHIP_ID) {
        printk(KERN_INFO "Bosch Sensortec Device detected!\n"
                "BMA250 registered I2C driver!\n");
    } else{
        printk(KERN_INFO "Bosch Sensortec Device not found"
                "i2c error %d \n", tempvalue);
        goto probe_err_cfg;
    }


	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	dd->irq = client->irq;
	dd->ip_dev->open = bma250_accl_open;
	dd->ip_dev->close = bma250_accl_release;
	dd->ip_dev->name = ACCL_NAME;
	dd->ip_dev->phys = ACCL_NAME;
	dd->ip_dev->id.vendor = ACCL_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
	set_bit(EV_REL, dd->ip_dev->evbit);
	set_bit(REL_X, dd->ip_dev->relbit);
	set_bit(REL_Y, dd->ip_dev->relbit);
	set_bit(REL_Z, dd->ip_dev->relbit);
	rc = input_register_device(dd->ip_dev);
	if (rc) {
		dev_err(&dd->ip_dev->dev,
			"bma250_accl_probe: input_register_device rc=%d\n", rc);
		goto probe_err_reg_dev;
	}
	rc = misc_register(&bma250_accl_misc_device);
	if (rc < 0) {
		dev_err(&client->dev, "bma250 misc_device register failed\n");
		goto probe_err_reg_misc;
	}
	/* bma250 sensor initial */
	if (rc < 0) {
		dev_err(&dd->ip_dev->dev, "bma250_accl_probe: \
				Error configuring device rc=%d\n", rc);
		goto probe_err_smbcfg;
	}

	dd->bma250_accl_mode = 200;	/* NORMAL Mode */
	i2c_set_clientdata(client, dd);

	rc = bma250_accl_configuration();
	if (rc < 0) {
		dev_err(&dd->ip_dev->dev, "bma250_accl_probe: \
				Error configuring device rc=%d\n", rc);
		goto probe_err_setmode;
	}

	setup_timer(&bma_wakeup_timer, bma_wakeup_timer_func, (long)dd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	dd->suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	dd->suspend_desc.suspend = bma250_accl_early_suspend,
	dd->suspend_desc.resume = bma250_accl_late_resume,
	register_early_suspend(&dd->suspend_desc);
#endif

	return rc;

probe_err_setmode:
probe_err_smbcfg:
	misc_deregister(&bma250_accl_misc_device);
probe_err_reg_misc:
	input_unregister_device(dd->ip_dev);
probe_err_reg_dev:
	input_free_device(dd->ip_dev);
	dd->ip_dev = NULL;
probe_err_reg:
	if (pdata && pdata->exit)
		pdata->exit(&client->dev);
probe_err_cfg:
	mutex_lock(&bma250_accl_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma250_accl_dd_lock);
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit bma250_accl_remove(struct i2c_client *client)
{
	struct drv_data *dd;
	struct bma250_accl_platform_data *pdata = pdata =
	    client->dev.platform_data;
	int rc;
	const char *devname;

	dd = i2c_get_clientdata(client);
	devname = dd->ip_dev->phys;

	rc = bma250_accl_power_down(dd);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: power down failed with error %d\n", __func__, rc);
#ifdef BMA250_ACCL_IRQ_MODE
	free_irq(dd->irq, &dd->i2c->dev);
#else
	del_timer(&bma_wakeup_timer);
#endif
	misc_deregister(&bma250_accl_misc_device);
	input_unregister_device(dd->ip_dev);
	i2c_set_clientdata(client, NULL);
	if (pdata && pdata->exit)
		pdata->exit(&client->dev);
	mutex_lock(&bma250_accl_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma250_accl_dd_lock);
	kfree(devname);
	kfree(dd);

	return 0;
}

static struct i2c_device_id bma250_accl_idtable[] = {
	{"bma250_accl", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bma250_accl_idtable);

static struct i2c_driver bma250_accl_driver = {
	.driver = {
		   .name = ACCL_NAME,
		   .owner = THIS_MODULE,
		   },
	.id_table = bma250_accl_idtable,
	.probe = bma250_accl_probe,
	.remove = __devexit_p(bma250_accl_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bma250_accl_suspend,
	.resume = bma250_accl_resume,
#endif
};

static int __init bma250_accl_init(void)
{
	INIT_LIST_HEAD(&dd_list);
	mutex_init(&bma250_accl_dd_lock);
	mutex_init(&bma250_accl_wrk_lock);

	return i2c_add_driver(&bma250_accl_driver);
}

module_init(bma250_accl_init);

static void __exit bma250_accl_exit(void)
{
	i2c_del_driver(&bma250_accl_driver);
}

module_exit(bma250_accl_exit);
