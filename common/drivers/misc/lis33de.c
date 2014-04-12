/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/input/misc/lis33de.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/lis33de.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

#define LIS33DE_DBG_MSG 0 
#define LIS33DE_DBG_FUNC 0 
#define LIS33DE_TEST 0

#if  LIS33DE_DBG_MSG
#define LISDBG(format, ...) printk(KERN_INFO "LIS33DE" format "\n", ## __VA_ARGS__)
#else 
#define LISDBG(format, ...) 
#endif

#if  LIS33DE_DBG_FUNC
#define LISFUNC() printk(KERN_INFO "LIS33DE %s () is called  \n", __FUNCTION__)
#else 
#define LISFUNC() 
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lis33de_early_suspend(struct early_suspend * h);
static void lis33de_late_resume(struct early_suspend * h);
static struct early_suspend gsensor_early_suspend;
#endif

#define LIS33DE_RETRY_COUNT 3

/* uncomment to enable interrupt based event generation */
/* #define LIS33DE_ACCL_IRQ_MODE */
#define ACCL_LIS33DE_NAME		"lis33de_accl"
#define ACCL_VENDORID			0x0001

#define LOW_G_THRES			5
#define LOW_G_DUR			50
#define HIGH_G_THRES			10
#define HIGH_G_DUR			1
#define ANY_MOTION_THRES		1
#define ANY_MOTION_CT			1

// shaojiang@wind-mobi.com 2011.07.18 begin
// fix the bug that compass invalid
//reviewer yuanlan

typedef struct {
	short x,	 /**< holds x-axis acceleration data sign extended. Range -512 to 511. */
	 y,		     /**< holds y-axis acceleration data sign extended. Range -512 to 511. */
	 z;		     /**< holds z-axis acceleration data sign extended. Range -512 to 511. */
} lis33de_acc_t;

static lis33de_acc_t newacc;
// shaojiang@wind-mobi.com 2011.07.18 end

static struct timer_list lis33de_wakeup_timer;

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("lis33de");

struct drv_data {
	struct input_dev *ip_dev;
	struct i2c_client *i2c;
	struct delayed_work work_data;
	int lis33de_accl_mode;
	int irq;
	struct list_head next_dd;
};

static struct mutex lis33de_accl_dd_lock;
static struct mutex lis33de_accl_wrk_lock;
static struct list_head dd_list;

#ifdef CONFIG_HAS_WAKELOCK
extern suspend_state_t get_suspend_state(void);
#endif

static atomic_t lis33de_irq;
static atomic_t a_flag;

/* cannot avoid global pointer as the smb380 read/ write callback does not
 * absract interface level information
 */
static struct i2c_client *lis33de_accl_client;

/*      i2c write routine for lis33de    */

static int lis33de_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
			{
				.addr = lis33de_accl_client ->addr,
				.flags = lis33de_accl_client->flags & I2C_M_TEN,
				.len    =1,
				.buf 	  =buf,
			},
			{
				.addr = lis33de_accl_client ->addr,
				.flags = (lis33de_accl_client ->flags & I2C_M_TEN) | I2C_M_RD,
				.len    =len,
				.buf 	  =buf,
			}
	};

#if 0
	int j;
	char addr = buf[0];
#endif
	for(i = 0; i < LIS33DE_RETRY_COUNT ;i++)
		{
		if(i2c_transfer( lis33de_accl_client->adapter ,  msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if(i >= LIS33DE_RETRY_COUNT){
		pr_err("%s: retry over %d\n", __FUNCTION__, LIS33DE_RETRY_COUNT);
		return -EIO;
	}

#if 0

	pr_info("lis33de_i2c_rx_data: len=%02x, addr=%02x  data=",
		len, addr);
	for (j = 0;j <len ; j++)
		pr_info(" %02x", buf[j ]);
	pr_info("\n");
#endif
	return 0;
}

/*      i2c read routine for lis33de     */

static int lis33de_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
			{
				.addr = lis33de_accl_client ->addr,
				.flags = lis33de_accl_client->flags & I2C_M_TEN,
				.len    =len,
				.buf 	  =buf,
			}
	};
#if 0
	int j;
#endif
	for(i = 0; i < LIS33DE_RETRY_COUNT ; i++)
		{
		if(i2c_transfer( lis33de_accl_client->adapter ,  msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if(i >= LIS33DE_RETRY_COUNT){
		pr_err("%s: retry over %d\n", __FUNCTION__, LIS33DE_RETRY_COUNT);
		return -EIO;
	}
#if 0
	pr_info("lis33de_i2c_tx_data: len=%02x, addr=%02x  data=",
		len, buf[0]);
	for (j = 0; j < (len-1); j++)
		pr_info(" %02x", buf[j + 1]);
	pr_info("\n");
#endif
	return 0;
}


static int lis33de_hw_init()
{
	unsigned char data[4] = {0,0,0,0};

	LISFUNC();

	data[0] = LIS33DE_CTRL_RERG1;
	data[1] = 0x47;
	if(lis33de_i2c_tx_data(data, 2) < 0){
	 	return -EFAULT;
	}

	data[0] = LIS33DE_CTRL_RERG2;
	data[1] = 0;
	if(lis33de_i2c_tx_data(data, 2) < 0){
	 	return -EFAULT;
	}

	data[0] = LIS33DE_CTRL_RERG3;
	data[1] = 0;
	if(lis33de_i2c_tx_data(data, 2) < 0){
	 	return -EFAULT;
	}

	return 0;
}


static int lis33de_readxyz(unsigned char *x , unsigned char *y, unsigned char *z)
{
	unsigned char data[4] = {0,0,0,};

	data[0] = LIS33DE_OUTX;
	if(lis33de_i2c_rx_data(data, 1) < 0){
		return -EFAULT;
	}
	*x = data[0];

	data[0] = LIS33DE_OUTY;
	if(lis33de_i2c_rx_data(data, 1) < 0){
		return -EFAULT;
	}
	*y = data[0];

	data[0] = LIS33DE_OUTZ;
	if(lis33de_i2c_rx_data(data, 1) < 0){
		return -EFAULT;
	}
	*z = data[0];

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void  lis33de_early_suspend(struct early_suspend *h)
{
	unsigned char data[4] = {0,0,0,0};
	unsigned char data2[2] = {0,0};

	struct drv_data *dd;
	dd = i2c_get_clientdata(lis33de_accl_client);

	data[0] = LIS33DE_CTRL_RERG1;
	if(lis33de_i2c_rx_data(data, 1) < 0){
	 	printk("%s, i2c_error 1\n",__func__);
	}

	data2[0] = LIS33DE_CTRL_RERG1;
	data2[1] = data[0] & ~(0x1 << 6); 
	if(lis33de_i2c_tx_data(data2, 2) < 0){
	 	printk("%s, i2c_error 2\n",__func__);
	}

	data[0] = LIS33DE_CTRL_RERG1;
	if(lis33de_i2c_rx_data(data, 1) < 0){
	 	printk("%s, i2c_error 3\n",__func__);
	}

	del_timer(&lis33de_wakeup_timer);
	cancel_delayed_work_sync(&dd->work_data);
	return;
}

static void lis33de_late_resume(struct early_suspend *h)
{
	unsigned char data[4] = {0,0,0,0};
	unsigned char data2[2] = {0,0};

	data[0] = LIS33DE_CTRL_RERG1;
	if(lis33de_i2c_rx_data(data, 1) < 0){
	 	printk("%s, i2c_error 1\n",__func__);
	}

	data2[0] = LIS33DE_CTRL_RERG1;
	data2[1] = data[0] | (0x1 << 6); 
	if(lis33de_i2c_tx_data(data2, 2) < 0){
	 	printk("%s, i2c_error 2\n",__func__);
	}

	data[0] = LIS33DE_CTRL_RERG1;
	if(lis33de_i2c_rx_data(data, 1) < 0){
	 	printk("%s, i2c_error 3\n",__func__);
	}
	
	if (atomic_read(&a_flag))
		mod_timer(&lis33de_wakeup_timer, jiffies + HZ/1000);

	return;
}

#else
static int lis33de_suspend(struct i2c_client *client , pm_message_t mesg)
{
	struct drv_data *dd;
	dd = i2c_get_clientdata(client);

	del_timer(&lis33de_wakeup_timer);
	cancel_delayed_work_sync(&dd->work_data);

	return 0;
}

static int lis33de_resume(struct i2c_client *client)
{
	struct drv_data *dd;
	dd = i2c_get_clientdata(client);

	if (atomic_read(&a_flag))
		mod_timer(&lis33de_wakeup_timer, jiffies + 200);

	return 0;
}
#endif


static int lis33de_accl_open(struct input_dev *dev)
{
	int rc = 0;
	return rc;
}

static void lis33de_accl_release(struct input_dev *dev)
{
	return;
}

static void lis33de_accl_getdata(struct drv_data *dd)
{
/*Modified by lijie@wind-mobi.com on 2005-5-12 for axis support feature*/ 
	int   xx,yy,zz;
	int   X, Y, Z;
	unsigned char  data[16] = {0,0,0,0};

	struct lis33de_accl_platform_data *pdata = lis33de_accl_client->dev.platform_data;
		

	mutex_lock(&lis33de_accl_wrk_lock);

	lis33de_readxyz(data,data+1,data+2);

	xx = (signed char)data[0];
	yy = (signed char)data[1];
	zz = (signed char)data[2];
	
	LISDBG("LIS33DE x=%2x y=%2x  z=%2x \n",data[0],data[1],data[2]);

	switch (pdata->orientation) 
	{
		case ST_ROT_90:
			X = -yy;
			Y = xx;
			Z = zz;
			break;
		case ST_ROT_180:
			X = -xx;
			Y = -yy;
			Z = zz;
			break;
		case ST_ROT_270:
			X = yy;
			Y = -xx;
			Z = zz;
			break;
		case ST_NO_ROT:
			X = xx;
			Y = yy;
			Z = zz;
			break;
		default:
			pr_err("lis33de_accl: invalid orientation specified\n");
	}

	if (pdata->invert) 
	{
		X = -xx;
		Z = -zz;
	}
// xiongbiao@wind-mobi.com 2012.01.04 begin
// modify lis33du g-sensor rotate direction.
	Z = -Z;
	X = -X;
// xiongbiao@wind-mobi.com 2012.01.04 end

/*Modification ends here*/
	input_report_rel(dd->ip_dev, REL_X, X);
	input_report_rel(dd->ip_dev, REL_Y, Y);
		input_report_rel(dd->ip_dev, REL_Z, Z);
		input_sync(dd->ip_dev);
//	}

	LISDBG("LIS33DE x=%d y=%d  z=%d\n",X,Y,Z);

	// shaojiang@wind-mobi.com 2011.07.18 begin
	// fix the bug that compass invalid
	//reviewer yuanlan
	newacc.x = X;
	newacc.y = Y;
	newacc.z = Z;
	// shaojiang@wind-mobi.com 2011.07.18 end
	
	mutex_unlock(&lis33de_accl_wrk_lock);

	return;

}

void lis33de_wakeup_timer_func(unsigned long data)
{
	struct drv_data *dd;
	long delay = 0;

	dd = (struct drv_data *) data;

	delay = dd->lis33de_accl_mode * HZ / 1000;
	/* Set delay >= 2 jiffies to avoid cpu hogging*/
	if (delay < 2)
		delay = 2;
	schedule_delayed_work(&dd->work_data, HZ/1000);
	
	if (atomic_read(&a_flag))
		mod_timer(&lis33de_wakeup_timer, jiffies + delay);
}


static void lis33de_accl_work_f(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct drv_data *dd = container_of(dwork, struct drv_data, work_data);

	lis33de_accl_getdata(dd);

}

static int lis33de_accl_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = inode->i_private;

	return 0;
}

static int lis33de_accl_misc_ioctl(struct inode *inode, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int delay;
	struct drv_data *dd;
	
	dd = i2c_get_clientdata(lis33de_accl_client);
	switch (cmd) {
	case LIS33DE_ACCL_IOCTL_GET_DELAY:
		delay = dd->lis33de_accl_mode;

		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;

	case LIS33DE_ACCL_IOCTL_SET_DELAY:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay < 0 || delay > 200)
			return -EINVAL;
		dd->lis33de_accl_mode = delay;
		
	case LIS33DE_ACCL_IOCTL_SET_FLAG:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay == 1)
			mod_timer(&lis33de_wakeup_timer, jiffies + HZ/1000);
		else if (delay == 0)
			del_timer(&lis33de_wakeup_timer);
		atomic_set(&a_flag, delay);
		break;
	case LIS33DE_ACCL_IOCTL_GET_DATA:
		if (!atomic_read(&a_flag))
			lis33de_accl_getdata(dd);
		// shaojiang@wind-mobi.com 2011.07.18 begin
		// fix the bug that compass invalid
		//reviewer yuanlan
		if (copy_to_user(argp, &newacc, sizeof(newacc)))
			return -EFAULT;
		// shaojiang@wind-mobi.com 2011.07.18 end
		break;
	}

	return 0;
}

static const struct file_operations lis33de_accl_misc_fops = {
	.owner = THIS_MODULE,
	.open = lis33de_accl_misc_open,
	.ioctl = lis33de_accl_misc_ioctl,
};

static struct miscdevice lis33de_accl_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lis33de_accl",
	.fops = &lis33de_accl_misc_fops,
        .mode = 0777,
};

/*This modification is done for dynamically support both on ST and ADI gsensor chips lijie@wind-mobi.com on 2011-5-12*/
static int verify_lis33de()
{

	unsigned char data[4] = {0,0,0,0};

	/*check if i2c with the dedicated adress could return value*/
	data[0] =  LIS33DE_CTRL_RERG1;
	if(lis33de_i2c_rx_data(data,1)<0)
	{
		printk("Could not get lis33de i2c register content\r\n");
		return 0;
	}

	/*check if the register conent is valid*/
	if(0x07 != (data[0]&0x0F))
	{
		printk("This is not a valid lis33de device, data[0] is 0X%02X\r\n", data[0]);
		return 0;
	}

	return 1;
}
/*Modification ends here*/

static int __devinit lis33de_accl_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct drv_data *dd;
	int rc = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality error\n");
		goto probe_exit;
	}

/*This modification is done for dynamically support both on ST and ADI gsensor chips lijie@wind-mobi.com on 2011-5-12*/
	lis33de_accl_client = client;
	

	if(!verify_lis33de()) 
	{
		printk("lis33 g sensor probe bad\r\n");
		rc = -ENOENT;
		goto probe_exit;
	}
/*Modification ends here*/
	
	dd = kzalloc(sizeof(struct drv_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	mutex_lock(&lis33de_accl_dd_lock);
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&lis33de_accl_dd_lock);
	INIT_DELAYED_WORK(&dd->work_data, lis33de_accl_work_f);

	i2c_set_clientdata(client, dd);

	dd->i2c = client;



	if(lis33de_hw_init() < 0) {
		goto probe_err_reg_dev;
	}

	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	
	dd->ip_dev->open = lis33de_accl_open;
	dd->ip_dev->close = lis33de_accl_release;
	dd->ip_dev->name = ACCL_LIS33DE_NAME;
	dd->ip_dev->phys = ACCL_LIS33DE_NAME;
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
			"lis33de_accl_probe: input_register_device rc=%d\n", rc);
		goto probe_err_reg_dev;
	}
	
	rc = misc_register(&lis33de_accl_misc_device);
	if (rc < 0) {
		dev_err(&client->dev, "lis33de misc_device register failed\n");
		goto probe_err_reg_misc;
	}
	/* lis33de sensor initial */

 	#ifdef CONFIG_HAS_WAKELOCK
	#ifdef CONFIG_EARLYSUSPEND
		gsensor_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +2;
		gsensor_early_suspend.suspend = lis33de_early_suspend;
		gsensor_early_suspend.resume = lis33de_late_resume;
		register_early_suspend(& gsensor_early_suspend);

		if(has_wake_lock(WAKE_LOCK_SUSPEND) == 0 && get_suspend_state() == PM_SUSPEND_ON)
		{
			printk("** gsensor early suspend by APM **\n");
		}
	#endif
	#endif
		
	if (rc < 0) {
		dev_err(&dd->ip_dev->dev, "lis33de_accl_probe: \
				Error configuring device rc=%d\n", rc);
		goto probe_err_lis33decfg;
	}
	
	/* NORMAL Mode */
	dd->lis33de_accl_mode = 200;


	setup_timer(&lis33de_wakeup_timer, lis33de_wakeup_timer_func, (long) dd);

	return rc;

probe_err_lis33decfg:
	misc_deregister(&lis33de_accl_misc_device);
probe_err_reg_misc:
	input_unregister_device(dd->ip_dev);
probe_err_reg_dev:
	input_free_device(dd->ip_dev);
	dd->ip_dev = NULL;
probe_err_reg:

	mutex_lock(&lis33de_accl_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&lis33de_accl_dd_lock);
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit lis33de_accl_remove(struct i2c_client *client)
{
	struct drv_data *dd;

	const char *devname;

	#ifdef CONFIG_HAS_WAKELOCK
		unregister_early_suspend(& gsensor_early_suspend);
	#endif

	dd = i2c_get_clientdata(client);
	devname = dd->ip_dev->phys;

	del_timer(&lis33de_wakeup_timer);
	misc_deregister(&lis33de_accl_misc_device);
	input_unregister_device(dd->ip_dev);
	i2c_set_clientdata(client, NULL);

	mutex_lock(&lis33de_accl_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&lis33de_accl_dd_lock);
	kfree(devname);
	kfree(dd);

	return 0;
}

static struct i2c_device_id lis33de_accl_idtable[] = {
	{"lis33de_accl", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lis33de_accl_idtable);

static struct i2c_driver lis33de_accl_driver = {
	.driver = {
		   .name = ACCL_LIS33DE_NAME,
		   .owner = THIS_MODULE,
		   },
	.id_table = lis33de_accl_idtable,
	.probe = lis33de_accl_probe,
	.remove = __devexit_p(lis33de_accl_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = lis33de_suspend,
	.resume = lis33de_resume,
#endif
};

static int __init lis33de_accl_init(void)
{
	INIT_LIST_HEAD(&dd_list);
	mutex_init(&lis33de_accl_dd_lock);
	mutex_init(&lis33de_accl_wrk_lock);

	return i2c_add_driver(&lis33de_accl_driver);
}

module_init(lis33de_accl_init);

static void __exit lis33de_accl_exit(void)
{
	i2c_del_driver(&lis33de_accl_driver);
}

module_exit(lis33de_accl_exit);

