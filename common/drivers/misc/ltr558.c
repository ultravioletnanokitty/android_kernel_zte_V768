/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/input/misc/ltr558.c
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


#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/ltr558.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>

#include <linux/regulator/consumer.h>

#define DEBUG
#ifdef DEBUG
#define D(x...) printk(x)
//#define D(x...) pr_info(x)
#else
#define D(x...)
#endif

#define I2C_RETRY_COUNT 10
#define LTR558_I2C_NAME "ltr558"

#define POLLING_PROXIMITY 1
#define NO_IGNORE_BOOT_MODE 1

//#define NEAR_DELAY_TIME ((30 * HZ) / 1000)

#ifdef POLLING_PROXIMITY
#define POLLING_DELAY		1000 
#define TH_ADD			3
#endif

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

#define YONGQIANG

#ifdef YONGQIANG
static int ps_opened;
static int als_opened;
static struct wake_lock proximity_wake_lock;
static struct work_struct irq_workqueue;
static int ps_data_changed;
static int als_data_changed;
static int ps_active;
static int als_active;

static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

static int ltr558_irq;

static DECLARE_WAIT_QUEUE_HEAD(ps_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(als_waitqueue);


#endif

#ifdef POLLING_PROXIMITY
static void polling_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
#endif

struct ltr558_info {
	struct class *ltr558_class;
	struct device *ls_dev;
	struct device *ps_dev;
	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
	struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;
	int intr_pin;
	int als_enable;
	int ps_enable;
	int ps_irq_flag;
	int led;
	int irq;
	int ls_calibrate;
	int testmode;
	int gainrange;
	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int lightsensor_opened;
	uint16_t check_interrupt_add;
	uint8_t ps_thd_set;
	uint8_t enable_polling_ignore;
	struct regulator *ls_regulator;
};

static struct ltr558_info *lp_info;
static int enable_log = 0;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct ltr558_info *lpi);
static int lightsensor_disable(struct ltr558_info *lpi);
static int initial_ltr558(struct ltr558_info *lpi);
static void psensor_initial_cmd(struct ltr558_info *lpi);


// I2C Read
static int ltr558_i2c_read_reg(u8 regnum)
{
	int readdata;

	/*
	 * i2c_smbus_read_byte_data - SMBus "read byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 *
	 * This executes the SMBus "read byte" protocol, returning negative errno
	 * else a data byte received from the device.
	 */

	readdata = i2c_smbus_read_byte_data(lp_info->i2c_client, regnum);
	return readdata;
}



// I2C Write
static int ltr558_i2c_write_reg(u8 regnum, u8 value)
{
	int writeerror;

	/*
	 * i2c_smbus_write_byte_data - SMBus "write byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 * @value: Byte being written
	 *
	 * This executes the SMBus "write byte" protocol, returning negative errno
	 * else zero on success.
	 */

	writeerror = i2c_smbus_write_byte_data(lp_info->i2c_client, regnum, value);

	if (writeerror < 0)
		return writeerror;
	else
		return 0;
}

static int ltr558_i2c_read(unsigned char reg_addr,unsigned char *data)
{
    int dummy;

    if(!lp_info || !lp_info->i2c_client || (unsigned long)lp_info->i2c_client < 0xc0000000)
            return -1;

    dummy = i2c_smbus_read_byte_data(lp_info->i2c_client, reg_addr);
    if(dummy < 0)
            return -1;

    *data = dummy & 0x000000ff;

    return 0;
}

static int ltr558_i2c_write(unsigned char reg_addr,unsigned char data)
{
    int dummy;

    if(!lp_info || !lp_info->i2c_client || (unsigned long)lp_info->i2c_client < 0xc0000000)
            return -1;

    dummy = i2c_smbus_write_byte_data(lp_info->i2c_client, reg_addr, data);
    if(dummy < 0)
            return dummy;

    return 0;

}
//proximity read
static int get_adc_value(uint8_t *data)
{
	int ret = 0;
	int psval_lo, psval_hi, psdata;
	if (data == NULL)
      return -EFAULT;
	
	ret = ltr558_i2c_read(LTR558_PS_DATA_0,&psval_lo);
		
	ret = ltr558_i2c_read(LTR558_PS_DATA_1,&psval_hi);
		
	psdata = ((psval_hi & 7)* 256) + psval_lo;
	*data = psdata;
	return ret;
}

static void report_psensor_input_event(struct ltr558_info *lpi)
{
	   uint8_t ps_data;
	   int val, ret = 0;

	   ret = get_adc_value(&ps_data);/*check i2c result*/
	   if (ret == 0) {
		     val = (ps_data & 0x80) ? 0 : 1;
		     D("[ltr558] proximity %s, ps_data=%d\n", val ? "FAR" : "NEAR", ps_data);
	   } else {/*i2c err, report far to workaround*/
		     val = 1;
		     ps_data = 0;
		     D("[ltr558] proximity i2c err, report %s, ps_data=%d, record_init_fail %d\n",
			                                             val ? "FAR" : "NEAR", ps_data, record_init_fail);
	   }
		 /* 0 is close, 1 is far */
	  input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
	  input_sync(lpi->ps_input_dev);

    wake_lock_timeout(&(lpi->ps_wake_lock), 2*HZ);
}

static int lux_table[64]={1,1,1,2,2,2,3,4,4,5,6,7,9,11,13,16,19,22,27,32,39,46,56,67,80,96,
                          116,139,167,200,240,289,346,416,499,599,720,864,1037,1245,1495,
                          1795,2154,2586,3105,3728,4475,5372,6449,7743,9295,11159,13396,
                          16082,19307,23178,27826,33405,40103,48144,57797,69386,83298,
                          100000};




static int get_lsensor_value(struct ltr558_info *lpi,uint8_t *data)
{
        int ret = 0;
        int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
        int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
        int luxdata_int;
        int ratio = 0;
        long luxdata_flt = 0;
        if (data == NULL)
            return -EFAULT;

        alsval_ch0_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
        alsval_ch0_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);
        alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

        alsval_ch1_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
        alsval_ch1_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
        alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;


        ratio = alsval_ch1*100-alsval_ch0*69;

        // Compute Lux data from ALS data (ch0 and ch1)
        // For Ratio < 0.69:
        // 1.3618*CH0 a<80>???¡ã.5*CH1
        // For 0.69 <= Ratio < 1:
        // 0.57*CH0 a<80>???¡ã.345*CH1
        // For high gain, divide the calculated lux by 150.

        if (ratio < 0){
                luxdata_flt = (13618 * alsval_ch0) - (15000 * alsval_ch1);
        }
        else if ((ratio >= 0) && (alsval_ch1<alsval_ch0)){
                luxdata_flt = (5700 * alsval_ch0) - (3450 * alsval_ch1);
        }
        else {
                luxdata_flt = 0;
        }

        // For Range1
        if (lpi->gainrange == ALS_RANGE1_320)
                luxdata_flt = luxdata_flt / 1500000;

        // convert float to integer;
        luxdata_int = luxdata_flt;
        if ((luxdata_flt - luxdata_int)*2 > 1){
                luxdata_int = luxdata_int + 1;
        }
        else {
                luxdata_int = luxdata_flt/10000;
        }
        *data = luxdata_int;
        return ret;
}

static void report_lsensor_input_event(struct ltr558_info *lpi)
{

    uint8_t adc_value = 0;
    int level = 0, i, ret = 0;

    mutex_lock(&als_get_adc_mutex);
    ret = get_lsensor_value(lpi,&adc_value);
   
    if (ret == 0) {
        i = adc_value & 0x3f;
        level = i;//lux_table[i];
    }
    else
    {
        printk(KERN_ERR"lsensor read adc fail!\n");
        goto fail_out;
    }
    D("[ltr558] %s: ADC=0x%03X, Level=%d\n",__func__,i, level);
    input_report_abs(lpi->ls_input_dev, ABS_MISC, level);
    input_sync(lpi->ls_input_dev);

fail_out:
    mutex_unlock(&als_get_adc_mutex);

}




static void sensor_irq_do_work(struct work_struct *work)
{
    struct ltr558_info *lpi = lp_info;

    uint8_t status = 0;
    int als_ps_status;
    int interrupt, newdata;

    als_ps_status = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
    interrupt = als_ps_status & 10;
    newdata = als_ps_status & 5;

    switch (interrupt){
                case 2:
                        // PS interrupt
                        if ((newdata == 1) | (newdata == 5)){
                          
                         //wake_up_interruptible(&ps_waitqueue);
                            report_psensor_input_event(lpi);
                        }
                        break;

                case 8:
                        // ALS interrupt
                        if ((newdata == 4) | (newdata == 5)){
                        //wake_up_interruptible(&als_waitqueue);
                            report_lsensor_input_event(lpi);
                        }
                        break;

                case 10:
                        // Both interrupt
                        if ((newdata == 1) | (newdata == 5)){
                            report_psensor_input_event(lpi);
                        }
                        //wake_up_interruptible(&als_waitqueue);
                        if((newdata == 4) | (newdata == 5)){
                            report_lsensor_input_event(lpi);
                        }
                        break;
        }
#if 0
    /*check ALS or PS*/
    ltr558_i2c_read(LTR558_ALS_PS_STATUS,&status);

    D("[ltr558] intr status[0x%x]\n",status);
    if(status & 0x02)/*ps trigger interrupt*/
    {
        report_psensor_input_event(lpi);
    }
#endif

}


#ifdef POLLING_PROXIMITY
static void polling_do_work(struct work_struct *w)
{
	struct ltr558_info *lpi = lp_info;

	/*D("lpi->ps_enable = %d\n", lpi->ps_enable);*/
	if (!lpi->ps_enable && !lpi->als_enable)
		return;

    if(lpi->ps_enable)
    	report_psensor_input_event(lpi);
    if(lpi->als_enable)
    	report_lsensor_input_event(lpi);

    queue_delayed_work(lpi->lp_wq, &polling_work,
		msecs_to_jiffies(POLLING_DELAY));
}
#endif

static irqreturn_t ltr558_irq_handler(int irq, void *data)
{
	struct ltr558_info *lpi = data;
	if (enable_log)
		D("[ltr558] %s\n", __func__);
        printk("[WYQ] LP sensor intr!!!");

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int psensor_enable(struct ltr558_info *lpi)
{
    int ret = 0;
    int error;
	  int setgain;

	  D("[LTR558] %s by pid[%d] thread [%s]\n", __func__,current->pid,current->comm);
	  /* dummy report */
	  input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
	  input_sync(lpi->ps_input_dev);

    switch (lpi->gainrange) {
		    case PS_RANGE1:
			      setgain = MODE_PS_ON_Gain1;
			      break;

		    case PS_RANGE2:
			      setgain = MODE_PS_ON_Gain2;
			      break;

		    case PS_RANGE4:
			      setgain = MODE_PS_ON_Gain4;
			      break;

		    case PS_RANGE8:
			      setgain = MODE_PS_ON_Gain8;
			      break;

		    default:
			      setgain = MODE_PS_ON_Gain1;
			      break;
    }

	  error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain);

  
	  msleep(15);
	  report_psensor_input_event(lpi);
    lpi->ps_enable = 1;

  //if(!lpi->als_enable)
	 // queue_delayed_work(lpi->lp_wq, &polling_work,msecs_to_jiffies(POLLING_DELAY));

	  return ret;
}

static int psensor_disable(struct ltr558_info *lpi)
{
    int ret = -EIO;

    D("[LTR558] %s by pid[%d] thread [%s]\n", __func__,current->pid,current->comm); 
    ret = ltr558_i2c_write_reg(LTR558_PS_CONTR, MODE_PS_StdBy); 
    lpi->ps_enable = 0;

#ifdef POLLING_PROXIMITY
    if(!lpi->als_enable)
	      cancel_delayed_work(&polling_work);
#endif
	  return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct ltr558_info *lpi = lp_info;

	D("[ltr558] %s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct ltr558_info *lpi = lp_info;

	D("[ltr558] %s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
    int val;
	  struct ltr558_info *lpi = lp_info;

	  D("[ltr558] %s cmd %d\n", __func__, _IOC_NR(cmd));

	  switch (cmd) {
	  case LTR558_IOCTL_PS_ENABLE:
		    if (get_user(val, (unsigned long __user *)arg))
			      return -EFAULT;
		    if (val)
			      return psensor_enable(lpi);
		    else
			      return psensor_disable(lpi);
		    break;
	  case LTR558_IOCTL_GET_ENABLED:
		    return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		    break;
		
	  default:
		    pr_err("[ltr558 error]%s: invalid cmd %d\n",__func__, _IOC_NR(cmd));
		    return -EINVAL;
	  }
}

static const struct file_operations psensor_fops = {
    .owner = THIS_MODULE,
	  .open = psensor_open,
	  .release = psensor_release,
	  .unlocked_ioctl = psensor_ioctl
};

struct miscdevice ltr558_psensor_misc = {
    .minor = MISC_DYNAMIC_MINOR,
	  .name = "proximity",
	  .fops = &psensor_fops
};


static int lightsensor_enable(struct ltr558_info *lpi)
{
    int ret = 0;
    
    if(!lpi)
        return -1;
    mutex_lock(&als_enable_mutex);
	  D("[ltr558] %s by pid[%d] thread [%s]\n", __func__,current->pid,current->comm);

    if (lpi->gainrange == 1)
         ret = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
    else if (lpi->gainrange == 2)
        ret = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
    else
		    ret = -1;
    msleep(10);
		/* report an invalid value first to ensure we
		* trigger an event when adc_level is zero.
		*/
    lpi->als_enable = 1;

    input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
    input_sync(lpi->ls_input_dev);
    report_lsensor_input_event(lpi);/*resume, IOCTL and DEVICE_ATTR*/
    mutex_unlock(&als_enable_mutex);

#ifdef POLLING_PROXIMITY
    //if(!lpi->ps_enable)
    queue_delayed_work(lpi->lp_wq, &polling_work,
		msecs_to_jiffies(POLLING_DELAY));
	/* settng command code(0x01) = 0x03*/
#endif

    return ret;
}

static int lightsensor_disable(struct ltr558_info *lpi)
{
    int ret = 0;

    if(!lpi)
            return -1;

	  mutex_lock(&als_disable_mutex);
    ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy);
	  D("[ltr558] %s by pid[%d] thread [%s]\n", __func__,current->pid,current->comm);
	  lpi->als_enable = 0;
	  mutex_unlock(&als_disable_mutex);

#ifdef POLLING_PROXIMITY
    if(!lpi->ps_enable)
	      cancel_delayed_work(&polling_work);
#endif
	  return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
    struct ltr558_info *lpi = lp_info;
	  int rc = 0;

	  D("[ltr558] %s\n", __func__);
	  if (lpi->lightsensor_opened) {
		    pr_err("[ltr558 error]%s: already opened\n", __func__);
		    rc = -EBUSY;
	  }
	  lpi->lightsensor_opened = 1;
	  return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
    struct ltr558_info *lpi = lp_info;

	  D("[ltr558] %s\n", __func__);
	  lpi->lightsensor_opened = 0;
	  return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
    int rc, val;
	  struct ltr558_info *lpi = lp_info;

	  D("[ltr558] %s cmd %d\n", __func__, _IOC_NR(cmd));

	  switch (cmd) {
	      case LTR558_IOCTL_ALS_ENABLE:
		        if (get_user(val, (unsigned long __user *)arg)) {
			           rc = -EFAULT;
			           break;
		        }
		        D("[ltr558] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",__func__, val);
		        rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		        break;
	     case LTR558_IOCTL_GET_ALS_ENABLED:
		        val = lpi->als_enable;
		        D("[ltr558] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			      __func__, val);
		       rc = put_user(val, (unsigned long __user *)arg);
		       break;
	     default:
		       pr_err("[ltr558 error]%s: invalid cmd %d\n",__func__, _IOC_NR(cmd));
		       rc = -EINVAL;
	  }

	  return rc;
}

static const struct file_operations lightsensor_fops = {
    .owner = THIS_MODULE,
	  .open = lightsensor_open,
	  .release = lightsensor_release,
	  .unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	  .minor = MISC_DYNAMIC_MINOR,
	  .name = "lightsensor",
	  .fops = &lightsensor_fops
};

//=========================================
static int lightsensor_setup(struct ltr558_info *lpi)
{
    int ret;

	  lpi->ls_input_dev = input_allocate_device();
	  if (!lpi->ls_input_dev) {
		     pr_err(
			       "[ltr558 error]%s: could not allocate ls input device\n",__func__);
		return -ENOMEM;
          }
	  lpi->ls_input_dev->name = "lightsensor";
	  set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	  input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	  ret = input_register_device(lpi->ls_input_dev);
	  if (ret < 0) {
		    pr_err("[ltr558 error]%s: can not register ls input device\n",__func__);
		    goto err_free_ls_input_device;
	  }

	  ret = misc_register(&lightsensor_misc);
	  if (ret < 0) {
		    pr_err("[ltr558 error]%s: can not register ls misc device\n",__func__);
		    goto err_unregister_ls_input_device;
	  }

	  return ret;

err_unregister_ls_input_device:
	  input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	  input_free_device(lpi->ls_input_dev);
	  return ret;
}

static int psensor_setup(struct ltr558_info *lpi)
{
    int ret;

	  lpi->ps_input_dev = input_allocate_device();
	  if (!lpi->ps_input_dev) {
		    pr_err("[ltr558 error]%s: could not allocate ps input device\n",__func__);
		    return -ENOMEM;
	  }
	  lpi->ps_input_dev->name = "proximity";
	  set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	  input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	  ret = input_register_device(lpi->ps_input_dev);
	  if (ret < 0) {
		    pr_err("[ltr558 error]%s: could not register ps input device\n",__func__);
		    goto err_free_ps_input_device;
	  }

	  ret = misc_register(&ltr558_psensor_misc);
	  if (ret < 0) {
		    pr_err("[ltr558 error]%s: could not register ps misc device\n",__func__);
		    goto err_unregister_ps_input_device;
	  }
    /*Need add psensor interrupt gpio setup*/
    ret = request_irq(lpi->irq, ltr558_irq_handler, IRQF_TRIGGER_FALLING,"ltr558",lpi);
    if(ret < 0) {
        pr_err("[ltr558]%s: req_irq(%d) fail (%d)\n", __func__,lpi->irq, ret);
        goto err_request_irq_ps;
    }
	  return ret;

err_request_irq_ps:
	  misc_deregister(&ltr558_psensor_misc);
err_unregister_ps_input_device:
	  input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	  input_free_device(lpi->ps_input_dev);
	  return ret;
}

static int ltr558_ps_enable(int gainrange)
{
        int error;
        int setgain;

        switch (gainrange) {
                case PS_RANGE1:
                        setgain = MODE_PS_ON_Gain1;
                        break;

                case PS_RANGE2:
                        setgain = MODE_PS_ON_Gain2;
                        break;

                case PS_RANGE4:
                        setgain = MODE_PS_ON_Gain4;
                        break;

                case PS_RANGE8:
                        setgain = MODE_PS_ON_Gain8;
                        break;

                default:
                        setgain = MODE_PS_ON_Gain1;
                        break;
        }

        error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain);
        mdelay(WAKEUP_DELAY);

        /* =============== 
         * ** IMPORTANT **
         * ===============
         * Other settings like timing and threshold to be set here, if required.
         * Not set and kept as device default for now.
         */

        return error;
}

static int ltr558_als_enable(int gainrange)
{
        int error;

        if (gainrange == 1)
                error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
        else if (gainrange == 2)
                error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
        else
                error = -1;

        mdelay(WAKEUP_DELAY);

        /* =============== 
         * ** IMPORTANT **
         * ===============
         * Other settings like timing and threshold to be set here, if required.
         * Not set and kept as device default for now.
         */

        return error;
}
static int ltr558_devinit(void)
{
        int error;
        int init_ps_gain;
        int init_als_gain;

        mdelay(PON_DELAY);

        // Enable PS to Gain1 at startup
        init_ps_gain = PS_RANGE1;
        //ps_gainrange = init_ps_gain;

        error = ltr558_ps_enable(init_ps_gain);
        if (error < 0)
        {
                printk("[ltr558] device init error!!!\n");
                goto out;
        }

        // Enable ALS to Full Range at startup
        init_als_gain = ALS_RANGE2_64K;
        //als_gainrange = init_als_gain;

        error = ltr558_als_enable(init_als_gain);
        if (error < 0)
        {
                printk("[ltr558] device init error!!!\n");
                goto out;
        }
        error = 0;
        printk("[ltr558] device init success!!!\n");

        out:
        return error;
}

static int initial_ltr558(struct ltr558_info *lpi)
{
	  int ret;
	  uint8_t add,val;
    add = 0;    
    //ret = ltr558_i2c_read(add,&val);
    //if((0x03 == val)||(!ret))
    //    printk("ltr558 device found!! id = 0x%x\n",val);
    //else
    //    return -1;
         // ltr558_devinit();
	  return 0;
}


static int ltr558_setup(struct ltr558_info *lpi)
{
    int ret = 0;
	
    ret = initial_ltr558(lpi);
	 // if (ret < 0) {
		//    pr_err("[ltr558 error]%s: fail to initial ltr558 (%d)\n",__func__, ret);
	  //}

	  return ret;
}

static void ltr558_early_suspend(struct early_suspend *h)
{
    struct ltr558_info *lpi = lp_info;
    int ret;

	  D("[ltr558] %s\n", __func__);
#ifdef POLLING_PROXIMITY
    if(lpi->als_enable && !lpi->ps_enable)
    {
	      cancel_delayed_work(&polling_work);
    }
#endif
    ret = psensor_disable(lpi);
	  if (ret == 0)
		    ret = lightsensor_disable(lpi);

	 // return ret;

}

static int ltr558_dev_init(struct ltr558_info *lpi)
{
    int error;
   	int init_ps_gain;
	  int init_als_gain;

	  mdelay(600);

	// Enable PS to Gain1 at startup
	  init_ps_gain = PS_RANGE1;
	 
    lpi->gainrange = init_ps_gain;
	  error = psensor_enable(lpi);
	  if (error < 0)
		    goto out;


	// Enable ALS to Full Range at startup
	  init_als_gain = ALS_RANGE2_64K;
	 
    lpi->gainrange = init_als_gain;
	  error = lightsensor_enable(lpi);
	  if (error < 0)
		    goto out;

	  error = 0;

	out:
	  return error;
}
static void ltr558_late_resume(struct early_suspend *h)
{
	   struct ltr558_info *lpi = lp_info;

	   D("[ltr558] %s\n", __func__);

	   if (lpi->als_enable){
	   	   lpi->gainrange =   ALS_RANGE2_64K;  
	       lightsensor_enable(lpi);
	   }

	   if (lpi->ps_enable){
        D("[ltr558] resume ps enabled\n");
#if 1
        if(lpi->ps_irq_flag)
        {
    	    disable_irq_nosync(lpi->irq);
            lpi->ps_irq_flag = 0;
        }
#endif
        lpi->gainrange =   ALS_RANGE2_64K; 
        lpi->gainrange = PS_RANGE1;
		    psensor_enable(lpi);
    }

}
/**************sys interface start ****************/

static ssize_t ps_adc_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{

    uint8_t value;
    int ret;
    struct ltr558_info *lpi = lp_info;
    int value1;

    ret = get_adc_value(&value);

    ret = sprintf(buf, "ADC[0x%03X]\n ps[%d]\n als[%d]\n irq[%d]\n", \
              value, lpi->ps_enable,lpi->als_enable,lpi->ps_irq_flag);

    return ret;
}

static ssize_t ps_enable_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
    int ps_en;
    struct ltr558_info *lpi = lp_info;

    ps_en = -1;
    sscanf(buf, "%d", &ps_en);

    if (ps_en != 0 && ps_en != 1
        && ps_en != 10 && ps_en != 13 && ps_en != 16)
        return -EINVAL;

    if (ps_en) {
        D("[ltr558] %s: ps_en=%d\n",__func__, ps_en);
        psensor_enable(lpi);
    } else
        psensor_disable(lpi);

    D("[ltr558] %s\n", __func__);

    return count;
}

static ssize_t als_enable_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
    int als_en;
    struct ltr558_info *lpi = lp_info;

    als_en = -1;
    sscanf(buf, "%d", &als_en);

    if (als_en != 0 && als_en != 1
        && als_en != 10 && als_en != 13 && als_en != 16)
        return -EINVAL;

    if (als_en) {
        D("[ltr558] %s: ps_en=%d\n",
            __func__, als_en);
        lightsensor_enable(lpi);
    } else
        lightsensor_disable(lpi);

    D("[ltr558] %s\n", __func__);

    return count;
}
static DEVICE_ATTR(ps_enable, 0664, ps_adc_show, ps_enable_store);
static DEVICE_ATTR(als_enable, 0664, ps_adc_show, als_enable_store);








#ifdef YONGQIANG
static int ltr558_ps_read(void)
{
        int psval_lo, psval_hi, psdata;

        psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
        if (psval_lo < 0){
                psdata = psval_lo;
                goto out;
        }

        psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
        if (psval_hi < 0){
                psdata = psval_hi;
                goto out;
        }

        psdata = ((psval_hi & 7)* 256) + psval_lo;


        out:
        final_prox_val = psdata;
        return psdata;
}

// Put ALS into Standby mode
static int ltr558_als_disable(void)
{
        int error;
        error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy);
        return error;
}




#endif
static int ltr558_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    int ret = 0;
	  struct ltr558_info *lpi;
	  struct ltr558_platform_data *pdata;

	  D("[ltr558] %s\n", __func__);


	  lpi = kzalloc(sizeof(struct ltr558_info), GFP_KERNEL);
	  if (!lpi)
        return -ENOMEM;

	/*D("[ltr558] %s: client->irq = %d\n", __func__, client->irq);*/

	  lpi->i2c_client = client;
	  pdata = client->dev.platform_data;
	  if (!pdata) {
		    pr_err("[ltr558 error]%s: Assign platform_data error!!\n",__func__);
		    ret = -EBUSY;
		    goto err_platform_data_null;
	  }
    /*start to declare regulator and enable power*/
    lpi->ls_regulator = regulator_get(NULL, "hcldo1_3v3");
	  if (!lpi->ls_regulator || IS_ERR(lpi->ls_regulator)) {
		    printk(KERN_ERR "ltr558 No Regulator available\n");
		    ret = -EFAULT;
		    goto err_platform_data_null;
	  }
    
    if(lpi->ls_regulator){
            regulator_set_voltage(lpi->ls_regulator,3300000,3300000);
            regulator_enable(lpi->ls_regulator);/*enable power*/
    }
//xiongbiao@wind-mobi.com 2011.12.21 begin
//check device ID
      uint8_t device_id = 0;
	  ret = ltr558_i2c_read(LTR558_MANUFACTURER_ID,&device_id);
	  if((device_id == 0x05)||(!ret))
	  {
	  	   printk("=====ltr558 device found!!====\n");
	  }
	  else
	  {
	 	   printk("ltr558 device no found error!! id = %x\n",device_id);
		   goto err_platform_data_null;
	  }
	  
//xiongbiao@wind-mobi.com 2011.12.21 end


	  lpi->irq = client->irq;
	  i2c_set_clientdata(client, lpi);
          //ltr558_devinit();
         // if (ret) {
          //      printk("ltr558 device init failed.\n");
          
          //}
          pdata->init();/*init sensor gpio interrupt pin*/

	  lp_info = lpi;

	  mutex_init(&als_enable_mutex);
	  mutex_init(&als_disable_mutex);
	  mutex_init(&als_get_adc_mutex);

	  ret = lightsensor_setup(lpi);
	  if (ret < 0) {
		    pr_err("[ltr558 error]%s: lightsensor_setup error!!\n",__func__);
		    goto err_lightsensor_setup;
	  }

	  ret = psensor_setup(lpi);
	  if (ret < 0) {
		    pr_err("[ltr558 error]%s: psensor_setup error!!\n",__func__);
		    goto err_psensor_setup;
	  }

	  lpi->lp_wq = create_singlethread_workqueue("ltr558_wq");
	  if (!lpi->lp_wq) {
		    pr_err("[ltr558 error]%s: can't create workqueue\n", __func__);
		    ret = -ENOMEM;
		    goto err_create_singlethread_workqueue;
	  }

	  wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");


	 ret = ltr558_setup(lpi);
	 if (ret < 0) {
		   pr_err("[ltr558 error]%s: ltr558_setup error!\n", __func__);
		   goto err_ltr558_setup;
	 }

    lpi->ltr558_class = class_create(THIS_MODULE, "optical_sensors");
    if (IS_ERR(lpi->ltr558_class)) {
        ret = PTR_ERR(lpi->ltr558_class);
        lpi->ltr558_class = NULL;
        goto err_create_class;
    }
    lpi->ls_dev = device_create(lpi->ltr558_class,
                NULL, 0, "%s", "lightsensor");
    if (unlikely(IS_ERR(lpi->ls_dev))) {
        ret = PTR_ERR(lpi->ls_dev);
        lpi->ls_dev = NULL;
        goto err_create_ls_device;
    }

    /* register the attributes */
    ret = device_create_file(lpi->ls_dev, &dev_attr_als_enable);
    if (ret)
        goto err_create_ls_device_file;

    lpi->ps_dev = device_create(lpi->ltr558_class,
                NULL, 0, "%s", "proximity");
    if (unlikely(IS_ERR(lpi->ps_dev))) {
        ret = PTR_ERR(lpi->ps_dev);
        lpi->ps_dev = NULL;
        goto err_create_ps_device;
    }

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_enable);
    if (ret)
        goto err_create_ps_device_file1;

    /* register the attributes */

    lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	  lpi->early_suspend.suspend = ltr558_early_suspend;
	  lpi->early_suspend.resume = ltr558_late_resume;
	  register_early_suspend(&lpi->early_suspend);

    lpi->als_enable = 1;
    lpi->ps_enable = 1;
    lpi->ps_irq_flag = 0;
    ltr558_devinit();

	  D("[ltr558] %s: Probe success!\n", __func__);
	  printk("[ltr558] Probe and setup success!\n");

	 return ret;

err_create_ps_device_file1:
	  device_unregister(lpi->ps_dev);
err_create_ps_device:
    device_remove_file(lpi->ls_dev, &dev_attr_als_enable);
err_create_ls_device_file:
	  device_unregister(lpi->ls_dev);
err_create_ls_device:
	  class_destroy(lpi->ltr558_class);
err_create_class:
err_ltr558_setup:
	  wake_lock_destroy(&(lpi->ps_wake_lock));
	  destroy_workqueue(lpi->lp_wq);
err_create_singlethread_workqueue:
    free_irq(lpi->irq, lpi);
	  input_unregister_device(lpi->ps_input_dev);
	  input_free_device(lpi->ps_input_dev);
	  misc_deregister(&ltr558_psensor_misc);
err_psensor_setup:
    regulator_disable(lpi->ls_regulator);/*disable power*/
	  input_unregister_device(lpi->ls_input_dev);
	  input_free_device(lpi->ls_input_dev);
	  misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
	  mutex_destroy(&als_enable_mutex);
	  mutex_destroy(&als_disable_mutex);
	  mutex_destroy(&als_get_adc_mutex);
    pdata->exit(NULL); /* free gpio request */
    err_platform_data_null:
	  kfree(lpi);
	  return ret;
}

static const struct i2c_device_id ltr558_i2c_id[] = {
    {LTR558_I2C_NAME, 0},
	  {}
};

static struct i2c_driver ltr558_driver = {
	  .id_table = ltr558_i2c_id,
	  .probe = ltr558_probe,
	  .driver = {
		    .name = LTR558_I2C_NAME,
		    .owner = THIS_MODULE,
	  },
};

static int __init ltr558_init(void)
{
    return i2c_add_driver(&ltr558_driver);
}

static void __exit ltr558_exit(void)
{

	i2c_del_driver(&ltr558_driver);
}

module_init(ltr558_init);
module_exit(ltr558_exit);

MODULE_DESCRIPTION("LTR558 Driver");
MODULE_LICENSE("GPL");
