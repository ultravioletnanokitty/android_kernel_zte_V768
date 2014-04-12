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

#include <linux/module.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/kernel.h>

#include <linux/ctype.h>

#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/lis33de.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* uncomment to enable interrupt based event generation */
// #define LIS33DE_IRQ_MODE_SUPPORT 
#define LIS33DE_SENSOR_NAME			"lis33de"
#define LIS33DE_SENSOR_VENDOR_ID			0x0001
#define LIS33DE_SIZE 50
#define AXIS_MAX_VALUE 255



MODULE_LICENSE("GPL v2");
MODULE_ALIAS("lis33de");

struct lis33de_data {
	struct input_dev *input;
	struct i2c_client *client;
	int irq;
	int lis33de_work_mode;
	char bits_per_transfer;
	struct delayed_work work_data;
	bool config;
	struct list_head next_dd;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend suspend_desc;
#endif
    short x;
    short y;
	short z;
};

typedef struct {
    short x,     /**< holds x-axis acceleration data sign extended. Range -128 to 128. */
          y,          /**< holds y-axis acceleration data sign extended. Range -128 to 128. */
          z;          /**< holds z-axis acceleration data sign extended. Range -128 to 128. */
} lis_acc_t;

static struct mutex lis33de_dd_lock;
static struct mutex lis33de_work_lock;
static struct list_head dd_list;
#ifndef LIS33DE_IRQ_MODE_SUPPORT
static struct timer_list lis_wakeup_timer;
#endif
static atomic_t lis_on;
static atomic_t a_flag;
static lis_acc_t newacc;

static struct i2c_client *lis33de_i2c_client;

enum lis33de_work_modes {
	SENSOR_DELAY_FASTEST = 0,
	SENSOR_DELAY_GAME = 20,
	SENSOR_DELAY_UI = 60,
	SENSOR_DELAY_NORMAL = 200
};



static int lis33de_i2c_read_byte(struct i2c_client *client,unsigned char adr)
{
        char buf;
        int ret;

        buf = adr;
        ret = i2c_master_send(client, &buf, 1);
        if(ret < 0) {
                dev_err(&client->dev, "failed to transmit instructions to lis33de.\n");
                return ret;
        }

        ret = i2c_master_recv(client, &buf, 1);
        if (ret<0) {
                dev_err(&client->dev, "failed to receive response from lis33de.\n");
                return ret;
        }

        return ret = buf;
}

static int lis33de_i2c_write_byte(struct i2c_client *client,char adr,char data)
{
        char buf[2];
        int ret;

        buf[0] = adr;
        buf[1] = data;
        ret = i2c_master_send(client, buf, 2);
        if(ret<0)
                dev_err(&client->dev, "failed to transmit instructions to lis33de.\n");

        return ret;
}
/*
 * Generic x,y,z attributes
 */
static ssize_t lis33de_show_xyz(struct device *dev, struct device_attribute *attr,char *buf)
{
//	signed char x, y, z;
	int ret;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct lis33de_data *sensor = i2c_get_clientdata(client);
	
	dev_dbg(dev, "lis33de_show() called on %s; index:0x%x\n", attr->attr.name,sattr->index);
	
	/* Read the x_outregister */	
	if ((sensor->x == 0) && (sensor->y == 0) && (sensor->z == 0))
	{
		ret = lis33de_i2c_read_byte(client, sattr->index);
		if (ret < 0) {
			printk("Read i2c error\n");
			return -EIO;
		}
		sensor->x = ret;
		
		/* Read the y_out register */
		ret = lis33de_i2c_read_byte(client, sattr->index+2);
		if (ret < 0) {
			printk("Read i2c error\n");
			return -EIO;
		}
		sensor->y = ret;
		
		/*Read the z_out register*/
		ret = lis33de_i2c_read_byte(client, sattr->index+4);
		if (ret < 0) {
			printk("Read i2c error\n");
			return -EIO;
		}    
		sensor->z = ret;
	}
	
	return sprintf(buf,"%c%c%c\n", sensor->x, sensor->y, sensor->z);
}

static ssize_t lis33de_show_ctrl(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	dev_dbg(dev, "lis33de_show() called on %s; index:0x%x\n", attr->attr.name,sattr->index);

	/* Read the first register */
	ret = lis33de_i2c_read_byte(client, sattr->index);
	if (ret < 0) {
		printk("Read i2c error\n");
		return -EIO;
	}
	
	return sprintf(buf, "0x%.2x\n", ret);
}

static ssize_t lis33de_set_ctrl(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	char *endp;
	unsigned int val;
	int ret = 0;

	printk("enter lis33de_set.\n");

	dev_dbg(dev, "ds1682_store() called on %s\n", attr->attr.name);

	/* Decode input */
	val = simple_strtoul(buf, &endp, 0);
	if (buf == endp) {
		dev_dbg(dev, "input string not a number\n");
		return -EINVAL;
	}

	ret = lis33de_i2c_write_byte(client,sattr->index,val);

	return (ret>0) ? count:ret;
}



  





/*
 * Simple register attributes
 */
static SENSOR_DEVICE_ATTR_2(xyz, S_IRUGO | S_IWUSR, lis33de_show_xyz,
			    NULL, 3, OUT_X);
static SENSOR_DEVICE_ATTR_2(x, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    NULL, 1, OUT_X);
static SENSOR_DEVICE_ATTR_2(y, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    NULL, 1, OUT_Y);
static SENSOR_DEVICE_ATTR_2(z, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    NULL, 1, OUT_Z);
static SENSOR_DEVICE_ATTR_2(ctrl1, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    lis33de_set_ctrl, 1, CTRL_REG1);
static SENSOR_DEVICE_ATTR_2(ctrl2, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    lis33de_set_ctrl, 1, CTRL_REG2);
static SENSOR_DEVICE_ATTR_2(ctrl3, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    lis33de_set_ctrl, 1, CTRL_REG3);
static SENSOR_DEVICE_ATTR_2(status, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    NULL, 1, STATUS_REG);
static SENSOR_DEVICE_ATTR_2(ff_cfg, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    lis33de_set_ctrl, 1, FF_WU_CFG);
static SENSOR_DEVICE_ATTR_2(ff_src, S_IRUGO, lis33de_show_ctrl,
			    NULL, 1, FF_WU_SRC);
static SENSOR_DEVICE_ATTR_2(ff_ths, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
			    lis33de_set_ctrl, 1, FF_WU_THS);
static SENSOR_DEVICE_ATTR_2(ff_duration, S_IRUGO | S_IWUSR, lis33de_show_ctrl,
                            lis33de_set_ctrl, 1, FF_WU_DURATION);


static const struct attribute_group lis33de_group = {
	.attrs = (struct attribute *[]) {
		&sensor_dev_attr_xyz.dev_attr.attr,
		&sensor_dev_attr_x.dev_attr.attr,
		&sensor_dev_attr_y.dev_attr.attr,
		&sensor_dev_attr_z.dev_attr.attr,
		&sensor_dev_attr_ctrl1.dev_attr.attr,
		&sensor_dev_attr_ctrl2.dev_attr.attr,
		&sensor_dev_attr_ctrl3.dev_attr.attr,
		&sensor_dev_attr_status.dev_attr.attr,
		&sensor_dev_attr_ff_cfg.dev_attr.attr,
		&sensor_dev_attr_ff_src.dev_attr.attr,
		&sensor_dev_attr_ff_ths.dev_attr.attr,
		&sensor_dev_attr_ff_duration.dev_attr.attr,
		NULL
	},
};

/*
 * User data attribute
 */

static ssize_t lis33de_sensor_bin_attr_read(struct file *file, struct kobject *kobj, struct bin_attribute *attr,
				  char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	int ret;
	char buf_tmp;
	
	printk("enter lis33de_sensor_read.\n");

	dev_dbg(&client->dev, "ds1682_eeprom_read(p=%p, off=%lli, c=%zi)\n",
		buf, off, count);

	if (off >= LIS33DE_SIZE)
		return 0;

	if (off + count > LIS33DE_SIZE)
		count = LIS33DE_SIZE - off;

	buf_tmp = off;
	ret =i2c_master_send(client, &buf_tmp, 1);
	if(ret<0){
		dev_err(&client->dev, "failed to transmit instructions to lis33de.\n");
		return ret;
	}
	
	ret = i2c_master_recv(client, buf, count);
	if (ret<0) {
		dev_err(&client->dev, "failed to receive response from lis33de.\n");
		return ret;
	}

	return count;
}

static ssize_t lis33de_sensor_bin_attr_write(struct file *file, struct kobject *kobj, struct bin_attribute *attr,
				   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	int ret;
	char *buf_tmp;
	
	printk("enter lis33de_sensor_write.\n");	

	dev_dbg(&client->dev, "ds1682_eeprom_write(p=%p, off=%lli, c=%zi)\n",
		buf, off, count);

	if (off >= LIS33DE_SIZE)
		return -ENOSPC;

	if (off + count > LIS33DE_SIZE)
		count = LIS33DE_SIZE - off;

	/* Write out to the device */
	buf_tmp = kzalloc(count+1, GFP_TEMPORARY);
	if(buf_tmp == NULL)
		return -ENOMEM;

	buf_tmp[0] = off;
	memcpy(&buf_tmp[1],buf,count);
	
	ret =i2c_master_send(client, buf_tmp, count+1);
	if(ret<0)
		dev_err(&client->dev, "failed to transmit instructions to lis33de.\n");
	
	kfree(buf_tmp);

	return (ret < 0) ? ret:count;
}

static struct bin_attribute lis33de_sensor_attr = {
	.attr = {
		.name = "g_sensor",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = LIS33DE_SIZE,
	.read = lis33de_sensor_bin_attr_read,
	.write = lis33de_sensor_bin_attr_write,
};




static int lis33de_power_down(struct lis33de_data *data)
{
	int rc;

	/* For sleep mode set bit 7 of control register (0x11) to 1 */
	rc = lis33de_i2c_write_byte(lis33de_i2c_client,CTRL_REG1,0x00);
	
	if (rc < 0)
		pr_err("G-Sensor power down failed\n");
	else
		atomic_set(&lis_on, 0);

	return rc;
}

static int lis33de_power_up(struct lis33de_data *data)
{

	int rc = 0;
    lis33de_i2c_write_byte(lis33de_i2c_client, CTRL_REG1, CTRL1_PD | CTRL1_Xen | CTRL1_Yen | CTRL1_Zen);  //100Hz, 2g, xen, yen, zen
    lis33de_i2c_write_byte(lis33de_i2c_client, CTRL_REG2, 0x00);
	//lis33de_i2c_write_byte(client, CTRL_REG3, 0x00);  //disable ff_wu_intterrupt
	lis33de_i2c_write_byte(lis33de_i2c_client, CTRL_REG3, 0x81);  //active low, int1->ff_wu_cfg
	
	lis33de_i2c_write_byte(lis33de_i2c_client, FF_WU_THS, 0x14);  //350mg
	lis33de_i2c_write_byte(lis33de_i2c_client,FF_WU_DURATION,0x00);  //Duration value
	//lis33de_i2c_write_byte(client, FF_WU_CFG, 0x2a);  //or, all high, if i set this, the interrupt is not work ok
	lis33de_i2c_write_byte(lis33de_i2c_client, FF_WU_CFG, 0x6a);  //or, latch, all high
	
	if (rc < 0)
		pr_err("G-Sensor power up failed\n");
	else
		atomic_set(&lis_on, 1);

	return rc;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int lis33del_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis33de_data *data;
	data = i2c_get_clientdata(client);

#ifdef LIS33DE_IRQ_MODE_SUPPORT
	disable_irq(data->irq);
#else
	del_timer(&lis_wakeup_timer);
	cancel_delayed_work_sync(&data->work_data);
#endif
	if (atomic_read(&lis_on))
		lis33de_power_down(data);

	return 0;
}

static int lis33de_resume(struct i2c_client *client)
{
	struct lis33de_data *data;
	data = i2c_get_clientdata(client);

#ifdef LIS33DE_IRQ_MODE_SUPPORT
	enable_irq(data->irq);
	if (atomic_read(&a_flag))
		lis33de_power_up(data);
#else
	if (atomic_read(&a_flag))
		mod_timer(&lis_wakeup_timer, jiffies + HZ / 1000);
#endif

	return 0;
}

#else
#define lis33del_suspend NULL
#define lis33de_resume NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lis33de_early_suspend(struct early_suspend *desc)
{
	struct lis33de_data *data = container_of(desc, struct lis33de_data, suspend_desc);
	pm_message_t mesg = {
		.event = PM_EVENT_SUSPEND,
		};
	lis33del_suspend(data->client, mesg);
}

static void lis33de_late_resume(struct early_suspend *desc)
{
	struct lis33de_data *data = container_of(desc, struct lis33de_data, suspend_desc);
	lis33de_resume(data->client);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef LIS33DE_IRQ_MODE_SUPPORT
static irqreturn_t lis33de_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct lis33de_data *data;
	long delay;

	data = dev_get_drvdata(dev);
	delay = data->lis33de_work_mode * HZ / 1000;
	if (!delay)		/* check for FAST MODE */
		delay = 1;
	schedule_delayed_work(&data->work_data, delay);

	return IRQ_HANDLED;
}
#else
void lis_wakeup_timer_func(unsigned long data)
{
	struct lis33de_data *dd;
	long delay = 0;

	dd = (struct lis33de_data *)data;

	delay = dd->lis33de_work_mode * HZ / 1000;
	/* Set delay >= 2 jiffies to avoid cpu hogging */
	if (delay < 2)
		delay = 2;
	schedule_delayed_work(&dd->work_data, HZ / 1000);
	if (atomic_read(&a_flag))
		mod_timer(&lis_wakeup_timer, jiffies + delay);
}
#endif

static int lis33de_open(struct input_dev *dev)
{
	int rc = 0;
#ifdef LIS33DE_IRQ_MODE_SUPPORT
	struct lis33de_data *data = input_get_drvdata(dev);

	if (!data->irq)
		return -1;
	/* Timer based implementation */
	/* does not require irq t be enabled */
	rc = request_irq(data->irq,
			 &lis33de_irq, 0, LIS33DE_SENSOR_NAME, &data->client->dev);
#endif

	return rc;
}

static void lis33de_release(struct input_dev *dev)
{
#ifdef LIS33DE_IRQ_MODE_SUPPORT
	struct lis33de_data *data = input_get_drvdata(dev);

	/* Timer based implementation */
	/* does not require irq t be enabled */
	free_irq(data->irq, &data->client->dev);
#endif

	return;
}









static void lis33de_get_input(struct lis33de_data *data)
{

	int X, Y, Z;
	int status = 0, ff_src_value;
	struct lis33de_platform_data *pdata = pdata =
	    lis33de_i2c_client->dev.platform_data;

	mutex_lock(&lis33de_work_lock);
#ifndef LIS33DE_IRQ_MODE_SUPPORT
	if (!atomic_read(&lis_on)) {
		lis33de_power_up(data);
		/* lis33de need 2 to 3 ms delay */
		/* to give valid data after wakeup */
		msleep(2);
	}
#endif
	
    status = lis33de_i2c_read_byte(data->client,STATUS_REG);	
	ff_src_value = lis33de_i2c_read_byte(data->client,FF_WU_SRC);	//must read this register		
	data->x = lis33de_i2c_read_byte(data->client,OUT_X);	
	data->y = lis33de_i2c_read_byte(data->client,OUT_Y);	
	data->z = lis33de_i2c_read_byte(data->client,OUT_Z);

	switch (pdata->orientation) {
	case LIS_ROT_90:
		X = -data->y;
		Y = data->x;
		Z = data->z;
		break;
	case LIS_ROT_180:
		X = -data->x;
		Y = -data->y;
		Z = data->z;
		break;
	case LIS_ROT_270:
		X = data->y;
		Y = -data->x;
		Z = data->z;
		break;
	default:
		pr_err("lis33de: invalid orientation specified\n");
	case  LIS_NO_ROT:
		X = data->x;
		Y = data->y;
		Z = data->z;
		break;
	}
	if (pdata->invert) {
		X = -X;
		Z = -Z;
	}
  
//    printk("send to user[%d][%d][%d] \n",X, Y, Z);
	input_report_rel(data->input, REL_X, X);
	input_report_rel(data->input, REL_Y, Y);
	input_report_rel(data->input, REL_Z, Z);
	input_sync(data->input);

	newacc.x = X;
	newacc.y = Y;
	newacc.z = Z;

#ifndef LIS33DE_IRQ_MODE_SUPPORT
	if (data->lis33de_work_mode >= SENSOR_DELAY_UI)
		lis33de_power_down(data);
#endif
	mutex_unlock(&lis33de_work_lock);

	return;

}

static void lis33de_work_callback(struct work_struct *work)
{
	struct delayed_work *dwork =
	    container_of(work, struct delayed_work, work);
	struct lis33de_data *data = container_of(dwork, struct lis33de_data, work_data);

	lis33de_get_input(data);

}

static int lis33de_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = inode->i_private;

	return 0;
}

static int lis33de_misc_ioctl(struct inode *inode, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int delay;
	struct lis33de_data *data;
	data = i2c_get_clientdata(lis33de_i2c_client);

	switch (cmd) {
	case LIS33DE_IOCTL_GET_DELAY:
		delay = data->lis33de_work_mode;

		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;

	case LIS33DE_IOCTL_SET_DELAY:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay < 0 || delay > 200)
			return -EINVAL;
		data->lis33de_work_mode = delay;
		break;
	case LIS33DE_IOCTL_SET_FLAG:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
#ifndef LIS33DE_IRQ_MODE_SUPPORT		
		if (delay == 1)
			mod_timer(&lis_wakeup_timer, jiffies + HZ / 1000);
		else if (delay == 0)
			del_timer(&lis_wakeup_timer);
		else
			return -EINVAL;
#endif		
		atomic_set(&a_flag, delay);
		break;
	case LIS33DE_IOCTL_GET_DATA:
		if (!atomic_read(&a_flag))
			lis33de_get_input(data);
		if (copy_to_user(argp, &newacc, sizeof(newacc)))
			return -EFAULT;
		break;
	}

	return 0;
}

static const struct file_operations lis33de_misc_fops = {
	.owner = THIS_MODULE,
	.open = lis33de_misc_open,
	.ioctl = lis33de_misc_ioctl,
};

static struct miscdevice lis33de_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lis33de",
	.fops = &lis33de_misc_fops,
    .mode = 0644,
};

static int __devinit lis33de_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct lis33de_data *data;
	int rc = 0;
 
	struct lis33de_platform_data *pdata = pdata =
	    client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality error\n");
		goto probe_exit;
	}
	data = kzalloc(sizeof(struct lis33de_data), GFP_KERNEL);
	if (!data) {
		rc = -ENOMEM;
		goto probe_exit;
	}
	lis33de_i2c_client = client;

	mutex_lock(&lis33de_dd_lock);
	list_add_tail(&data->next_dd, &dd_list);
	mutex_unlock(&lis33de_dd_lock);
	INIT_DELAYED_WORK(&data->work_data, lis33de_work_callback);
	data->client = client;

	if (pdata && pdata->init) {
		rc = pdata->init(&client->dev);
		if (rc)
			goto probe_err_cfg;
	}

	data->input = input_allocate_device();
	if (!data->input) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(data->input, data);
	data->irq = client->irq;
	data->input->open = lis33de_open;
	data->input->close = lis33de_release;
	data->input->name = LIS33DE_SENSOR_NAME;
	data->input->phys = LIS33DE_SENSOR_NAME;
	data->input->id.vendor = LIS33DE_SENSOR_VENDOR_ID;
	data->input->id.product = 1;
	data->input->id.version = 1;
	set_bit(EV_REL, data->input->evbit);
	set_bit(REL_X, data->input->relbit);
	set_bit(REL_Y, data->input->relbit);
	set_bit(REL_Z, data->input->relbit);
	rc = input_register_device(data->input);
	if (rc) {
		dev_err(&data->input->dev,
			"lis33de_probe: input_register_device rc=%d\n", rc);
		goto probe_err_reg_dev;
	}
    rc = sysfs_create_group(&client->dev.kobj, &lis33de_group);
	if (rc) return rc;
	
	rc = sysfs_create_bin_file(&client->dev.kobj, &lis33de_sensor_attr);
	if (rc)
		sysfs_remove_group(&client->dev.kobj, &lis33de_group);

	rc = misc_register(&lis33de_misc_device);
	if (rc < 0) {
		dev_err(&client->dev, "lis33de misc_device register failed\n");
		goto probe_err_reg_misc;
	}
	/* lis33de sensor initial */
	if (rc < 0) {
		dev_err(&data->input->dev, "lis33de_probe: \
				Error configuring device rc=%d\n", rc);
		goto probe_err_smbcfg;
	}

	data->lis33de_work_mode = 200;	/* NORMAL Mode */
	i2c_set_clientdata(client, data);

	

#ifndef LIS33DE_IRQ_MODE_SUPPORT
	setup_timer(&lis_wakeup_timer, lis_wakeup_timer_func, (long)data);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	data->suspend_desc.suspend = lis33de_early_suspend,
	data->suspend_desc.resume = lis33de_late_resume,
	register_early_suspend(&data->suspend_desc);
#endif

	return rc;


probe_err_smbcfg:
	misc_deregister(&lis33de_misc_device);
probe_err_reg_misc:
	input_unregister_device(data->input);
probe_err_reg_dev:
	input_free_device(data->input);
	data->input = NULL;
probe_err_reg:
	if (pdata && pdata->exit)
		pdata->exit(&client->dev);
probe_err_cfg:
	mutex_lock(&lis33de_dd_lock);
	list_del(&data->next_dd);
	mutex_unlock(&lis33de_dd_lock);
	kfree(data);
probe_exit:
	return rc;
}

static int __devexit lis33de_remove(struct i2c_client *client)
{
	struct lis33de_data *data;
	struct lis33de_platform_data *pdata = pdata =
	    client->dev.platform_data;
	int rc;
	const char *devname;

	data = i2c_get_clientdata(client);
	devname = data->input->phys;

	rc = lis33de_power_down(data);
	if (rc)
		dev_err(&data->input->dev,
			"%s: power down failed with error %d\n", __func__, rc);
        sysfs_remove_group(&client->dev.kobj, &lis33de_group);
	sysfs_remove_bin_file(&client->dev.kobj, &lis33de_sensor_attr);
#ifdef LIS33DE_IRQ_MODE_SUPPORT
	free_irq(data->irq, &data->client->dev);
#else
	del_timer(&lis_wakeup_timer);
#endif
	misc_deregister(&lis33de_misc_device);
	input_unregister_device(data->input);
	i2c_set_clientdata(client, NULL);
	if (pdata && pdata->exit)
		pdata->exit(&client->dev);
	mutex_lock(&lis33de_dd_lock);
	list_del(&data->next_dd);
	mutex_unlock(&lis33de_dd_lock);
	kfree(devname);
	kfree(data);

	return 0;
}

static struct i2c_device_id lis33de_idtable[] = {
	{"lis33de", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lis33de_idtable);

static struct i2c_driver lis33de_driver = {
	.driver = {
		   .name = LIS33DE_SENSOR_NAME,
		   .owner = THIS_MODULE,
		   },
	.id_table = lis33de_idtable,
	.probe = lis33de_probe,
	.remove = __devexit_p(lis33de_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = lis33del_suspend,
	.resume = lis33de_resume,
#endif
};

static int __init lis33de_init(void)
{
	INIT_LIST_HEAD(&dd_list);
	mutex_init(&lis33de_dd_lock);
	mutex_init(&lis33de_work_lock);

	return i2c_add_driver(&lis33de_driver);
}

module_init(lis33de_init);

static void __exit lis33de_exit(void)
{
	i2c_del_driver(&lis33de_driver);
}

module_exit(lis33de_exit);
