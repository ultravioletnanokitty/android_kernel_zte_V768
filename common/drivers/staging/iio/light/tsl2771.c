/*******************************************************************************
*                                                                              *
*   File Name:    taos.c                                                      *
*   Description:   Linux device driver for Taos ambient light and         *
*   proximity sensors.                                     *
*   Author:         John Koshi                                             *
*   History:   09/16/2009 - Initial creation                          *
*           10/09/2009 - Triton version         *
*           12/21/2009 - Probe/remove mode                *
*           02/07/2010 - Add proximity          *
*                                                                              *
********************************************************************************
*    Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/
// includes
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/delay.h>
//#include "types.h"
#include "taos_common.h"
#include <linux/delay.h>
//iVIZM
#include <linux/irq.h> 
#include <linux/interrupt.h> 
#include <linux/slab.h>
#include <mach/gpio.h> 
#include <linux/poll.h> 
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
//#define TAOS_CALIBRATE 1//WYQ
#define TAOS_CALIBRATE_NEW 1

//#include <linux/platform_device.h>
//#include "../i2c/busses/i2c_gpio_pxa.h"

#define TSL2271ID 						0x72
// device name/id/address/counts
#define TAOS_DEVICE_NAME                "alsps"
#define TAOS_DEVICE_ID                  "tritonFN"
#define TAOS_ID_NAME_SIZE               10
#define TAOS_TRITON_CHIPIDVAL           0x00
#define TAOS_TRITON_MAXREGS             32
#define TAOS_DEVICE_ADDR1               0x29
#define TAOS_DEVICE_ADDR2               0x39
#define TAOS_DEVICE_ADDR3               0x49
#define TAOS_MAX_NUM_DEVICES            3
#define TAOS_MAX_DEVICE_REGS            32
#define I2C_MAX_ADAPTERS                8

// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_TEST_STATUS         0x1F

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG             0X80
#define TAOS_TRITON_CMD_AUTO            0x20 //iVIZM//10--20---tangxin2011.05.10
#define TAOS_TRITON_CMD_BYTE_RW         0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60
#define TAOS_TRITON_CMD_PROX_INTCLR     0X05
#define TAOS_TRITON_CMD_ALS_INTCLR      0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR  0X07
#define TAOS_TRITON_CMD_TST_REG         0X08
#define TAOS_TRITON_CMD_USER_REG        0X09

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL   0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0X04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01
#define TAOS_TRITON_STATUS_PRXVALID     0x02
#define TAOS_TRITON_STATUS_ADCINTR      0x10
#define TAOS_TRITON_STATUS_PRXINTR      0x20

// lux constants
#define TAOS_MAX_LUX                    65536
#define TAOS_SCALE_MILLILUX             3
#define TAOS_FILTER_DEPTH               3
#define CHIP_ID                         0x3d
#define MISC_IOCTL_GET_INIT_VALUE		1

#define TSL2771_DEBUG    0 //wilson
#if TSL2771_DEBUG
#define TSL2771_DBG(fmt, args...)    printk("TAOS......:" fmt "\n", ## args)
#else
#define TSL2771_DBG(fmt, args...)    do{} while(0)
#endif
/* feixiaoping@wind-mobi.com 20120620 begin */
#if 1
static bool debug_mask = 0;//|DEBUG_SUSPEND;

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#endif
/* feixiaoping@wind-mobi.com 20120620 end */

// forward declarations
//static int taos_probe(struct platform_device *);
//static int taos_remove(struct platform_device *);
static int taos_open(struct inode *inode, struct file *file);
static int taos_release(struct inode *inode, struct file *file);
//static int taos_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int taos_ioctl(unsigned int cmd, char *arg);

//static long taos_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int taos_read(struct file *file, char *buf, size_t count, loff_t *ppos);
static int taos_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
static loff_t taos_llseek(struct file *file, loff_t offset, int orig);
static int taos_get_lux(void);
static int taos_lux_filter(int raw_lux);
static int taos_device_name(unsigned char *bufp, char **device_name);

//iVIZM
static int taos_als_threshold_set(void);
static int taos_prox_threshold_set(void);
static int taos_als_get_data(void);
static int taos_interrupts_clear(void);

#define ALS_PROX_DEBUG //iVIZM
static unsigned int ReadEnable = 0;//iVIZM
static struct ReadData  //iVIZM
{
    unsigned int data;
    unsigned int interrupt;
};
static struct ReadData readdata[2];//iVIZM

// first device number
static dev_t taos_dev_number;

static struct class *taos_class;

// client and device
static int num_bad = 0;
static int device_found = 0;
//iVIZM
static char pro_buf[4]; //iVIZM
static int mcount = 0; //iVIZM
static char als_buf[4]; //iVIZM
static u16 status = 0;
static int ALS_ON;
static int timer_on = 0;
static int aaa = 0;
static int prox_adjust = 0;
static int prox_get_complete = 0;
static struct wake_lock proximity_wakelock;
//static int locked = 0;
static struct delayed_work monitor_prox_work;
static struct delayed_work prox_poweron_work;
static struct mutex prox_mutex;
static int prox_status_now = 0;
static int prox_mean = 0;
static int PROX_ON = 0;
// per-device data
static struct taos_data 
{
    struct i2c_client *client;
    struct class *taos_dev;
    struct device *p_dev;
    struct device *l_dev;
    struct cdev cdev;
    unsigned int addr;
    struct input_dev *proximity_input_dev;
    struct input_dev *ambient_input_dev;
    int proximity_enable;					/* attribute value */
    int proximity_delay;					/* attribute value */
    int ambient_enable;						/* attribute value */
    int ambient_delay;						/* attribute value */
    struct work_struct work;				//iVIZM
//	struct wake_lock taos_wake_lock;		//iVIZM
    char taos_id;
    char taos_name[TAOS_ID_NAME_SIZE];
    struct semaphore update_lock;
	struct mutex                    lock;
	
    char valid;
    unsigned long last_updated;
    int irq;
    struct delayed_work d_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend suspend_desc;
#endif
	/* xuhuashan@gmail.com 2011.06.17 begin */
	/* Add PS calibrate function */
#ifdef TAOS_CALIBRATE
	struct kobject                  cali_kobj;
	struct completion               cali_completion;
	u8                              cali_mode;
	u32                             cali_min_val;
	u32                             cali_max_val;
	u32                             cali_avg_val;
	u32                             cali_cur_val;
	/* xuhuashan@gmail.com 2011.06.17 end */
#endif

} 
*taos_datap;


// file operations
static struct file_operations taos_fops = 
{
    .owner = THIS_MODULE,
    .open = taos_open,
    .release = taos_release,
    .read = taos_read,
    .write = taos_write,
    .llseek = taos_llseek,
    .ioctl = taos_ioctl,
};

// device configuration
static struct taos_cfg *taos_cfgp;
static u32 calibrate_target_param = 300000;
static u16 als_time_param = 300;
static u16 scale_factor_param = 1;
static u16 gain_trim_param = 512;
static u8 filter_history_param = 3;
static u8 filter_count_param = 1;
static u8 gain_param = 1;//2;
static u16 prox_threshold_hi_param = 500;
static u16 prox_threshold_lo_param = 400;
static u16 als_threshold_hi_param = 1;//iVIZM
static u16 als_threshold_lo_param = 0;//iVIZM
static u8 prox_int_time_param = 0xEE;//50ms
static u8 prox_adc_time_param = 0xFF;
static u8 prox_wait_time_param = 0xFF;
static u8 prox_intr_filter_param = 0x13;
static u8 prox_config_param = 0x00;
static u8 prox_pulse_cnt_param = 0x02;
static u8 prox_gain_param = 0x20;//0x60;
static u16 standard_prox_threshold_hi_param = 600;
// prox info
static struct taos_prox_info prox_cal_info[20];
static struct taos_prox_info prox_cur_info;
static struct taos_prox_info *prox_cur_infop = &prox_cur_info;
static u8 prox_history_hi = 0;
static u8 prox_history_lo = 0;
static struct timer_list prox_poll_timer;
static int device_released = 0;
static u16 sat_als = 0;
static u16 sat_prox = 0;

//struct delayed_work get_threshold_work;

// device reg init values
static u8 taos_triton_reg_init[16] = {0x00,0xFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00};

// lux time scale
static struct time_scale_factor  
{
    u16 numerator;
    u16 denominator;
    u16 saturation;
};
static struct time_scale_factor TritonTime = {1, 0, 0};
static struct time_scale_factor *lux_timep = &TritonTime;

// gain table
static u8 taos_triton_gain_table[] = {1, 8, 16, 120};

// lux data
static struct lux_data 
{
    u16 ratio;
    u16 clear;
	u16 ir;
};
static struct lux_data TritonFN_lux_data[] = 
{
    { 9830,  8320,  15360 },
    { 12452, 10554, 22797 },
    { 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
    { 0,     0,     0     }
};
static struct lux_data *lux_tablep = TritonFN_lux_data;
static int lux_history[TAOS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};//iVIZM

static int taos_write_byte_data(u8 command, u8 value)
{
    i2c_smbus_write_byte_data(taos_datap->client, command, value);
    return 0;
}

static u8 taos_read_byte_data(u8 command)
{
    u8 data = i2c_smbus_read_byte_data(taos_datap->client, command);
    return data;
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id) //iVIZM
{
    //if(debug_mask)
    	//printk("------fxp----taos_irq_handler--\n");
    schedule_work(&taos_datap->work);
    return IRQ_HANDLED;
}

static void read_all_regs(void)
{
	u8 reg_val = 0, reg_val_high = 0;
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x00);
	TSL2771_DBG("0x00 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x01);
	TSL2771_DBG("0x01 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x02);
	TSL2771_DBG("0x02 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x03);
	TSL2771_DBG("0x03 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x04);
	TSL2771_DBG("0x04 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x05);
	TSL2771_DBG("0x05 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x06);
	TSL2771_DBG("0x06 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x07);
	TSL2771_DBG("0x07 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x08);
	TSL2771_DBG("0x08 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x09);
	TSL2771_DBG("0x09 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0A);
	TSL2771_DBG("0x0A value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0B);
	TSL2771_DBG("0x0B value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0C);
	TSL2771_DBG("0x0C value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0D);
	TSL2771_DBG("0x0D value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0E);
	TSL2771_DBG("0x0E value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0F);
	TSL2771_DBG("0x0F value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x13);
	TSL2771_DBG("0x13 value = 0x%x", reg_val);
	reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x18);
	TSL2771_DBG("0x18 value = 0x%x", reg_val);
	reg_val_high = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x19);
	TSL2771_DBG("0x19 value = 0x%x", reg_val_high);
	TSL2771_DBG("current distence = %d", ((reg_val_high << 8) | reg_val));
}
#if 1
static int taos_get_data(void)//iVIZM
{
    int ret = 0;

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | 0x13)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte(1) failed in taos_work_func()\n");
        return (ret);
    }
    status = i2c_smbus_read_byte(taos_datap->client);
  //  if (down_interruptible(&taos_datap->update_lock))
       // return -ERESTARTSYS;
    if((status & 0x20) == 0x20) {
        ret = taos_prox_threshold_set();
        if(ret >= 0)
            ReadEnable = 1;
    } else if((status & 0x10) == 0x10) {
        ReadEnable = 1;
        taos_als_threshold_set();
        taos_als_get_data();
    }
    //up(&taos_datap->update_lock);
    return ret;
}

#else
static int taos_get_data(void)//iVIZM
{
	TSL2771_DBG("%s", __FUNCTION__);
    int ret = 0;

    status = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x13);

    if((status & 0x20) == 0x20) 
    {
	printk("It is prox interrupt\n");
	mutex_lock(&prox_mutex);
        ret = taos_prox_threshold_set();
        mutex_unlock(&prox_mutex);
        if(ret >= 0)
            ReadEnable = 1;
    } 

    return ret;
}
#endif
static int taos_interrupts_clear(void)//iVIZM
{
	TSL2771_DBG("%s", __FUNCTION__);
        int ret = 0;
	if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07)))) < 0) 
//	if (ret = taos_write_special_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x07) < 0)
	{
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte(2) failed in taos_work_func()\n");
        return (ret);
    }
    return ret;
}

static void taos_work_func(struct work_struct * work) //iVIZM
{
    if(debug_mask)
    	printk("------fxp----taos_irq_handler scheduling taos_work_func-\n");
    taos_get_data();
    taos_interrupts_clear();
}

static int taos_als_get_data(void)//iVIZM
{
//	TSL2771_DBG("%s", __FUNCTION__);
    u8 reg_val;
    int lux_val = 0;

    reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);

    if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
    {
       printk("taos_als_get_data nodata 1\n");
        return -ENODATA;
    }

    reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS);
    if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
     {
             printk("taos_als_get_data nodata 2\n");
	 	 	 return -ENODATA;
    }

    if ((lux_val = taos_get_lux()) < 0)
        printk(KERN_ERR "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n", lux_val);

   // lux_val = taos_lux_filter(lux_val); //  5.4
	TSL2771_DBG("---------wilson------lux_val=%d---\n",lux_val);
	//if((lux_val >= 0 && lux_val < 5) || (lux_val >=100 && lux_val < 3300) || (lux_val >= 4000 && lux_val < 19000) 
			//|| (lux_val >= 22000 && lux_val < 49000) || (lux_val >= 53000 && lux_val < 65537))
	//{
             
		if(aaa == 0)
		{
			input_report_abs(taos_datap->ambient_input_dev, ABS_PRESSURE, lux_val);
			input_sync(taos_datap->ambient_input_dev);
			aaa = 1;
			return 0;
		}
		if(aaa == 1)
		{
			input_report_abs(taos_datap->ambient_input_dev, ABS_PRESSURE/*ABS_MISC*/, (lux_val + 1));
			input_sync(taos_datap->ambient_input_dev);
			aaa = 0;
			return 0;
		}
	//}

    return 0;
}

static int taos_als_threshold_set(void)//iVIZM
{
	TSL2771_DBG("%s", __FUNCTION__);
    int i, ret = 0;
    u8 chdata[2];
    u16 ch0;

    if(i2c_smbus_read_i2c_block_data(taos_datap->client, TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_WORD_BLK_RW|(TAOS_TRITON_ALS_CHAN0LO+i)|0x80, 2, chdata) <= 0)
    {
		TSL2771_DBG("In %s, read fail", __FUNCTION__);
		return -ENODATA;
    }
    ch0 = chdata[0] + chdata[1]*256;
    als_threshold_hi_param = (12*ch0)/10;
    if (als_threshold_hi_param >= 65535)
        als_threshold_hi_param = 65535;	 
    als_threshold_lo_param = (8*ch0)/10;
    als_buf[0] = als_threshold_lo_param & 0x0ff;
    als_buf[1] = als_threshold_lo_param >> 8;
    als_buf[2] = als_threshold_hi_param & 0x0ff;
    als_buf[3] = als_threshold_hi_param >> 8;

    for( mcount=0; mcount<4; mcount++ ) 
    { 
        if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x04) + mcount, als_buf[mcount]))) < 0) 
	{
             printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos als threshold set\n");
             return (ret);
        }
    }
    return ret;
}

static int taos_prox_threshold_set(void)//iVIZM
{
    TSL2771_DBG("%s", __FUNCTION__);
    int ret = 0;
    u8 chdata[6];
    u16 proxdata = 0;
    u16 cleardata = 0;
	int data = 0;

    if(i2c_smbus_read_i2c_block_data(taos_datap->client, TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_WORD_BLK_RW|(TAOS_TRITON_ALS_CHAN0LO+0)|0x80, 6, chdata) <= 0)
    {
		TSL2771_DBG("In %s, read fail", __FUNCTION__);
		return -ENODATA;
    }
    cleardata = chdata[0] + chdata[1]*256;
    proxdata = chdata[4] + chdata[5]*256;
	if(debug_mask)
	{
		printk("000----fxp--- proxdata = %d and cleardata=%d---\n", proxdata,cleardata);
		printk("001----fxp---taos_cfgp->prox_threshold_lo = %d\n", taos_cfgp->prox_threshold_lo);
		printk("002----fxp---taos_cfgp->prox_threshold_hi = %d\n", taos_cfgp->prox_threshold_hi);
	}
    if (proxdata <= taos_cfgp->prox_threshold_lo)
    {
        pro_buf[0] = 0x0;
        pro_buf[1] = 0x0;
        pro_buf[2] = taos_cfgp->prox_threshold_hi & 0x0ff;
        pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;
		TSL2771_DBG("In 100, pro_buf[2] = %d, pro_buf[3] = %d", pro_buf[2], pro_buf[3]);
		data = 1;
		input_report_abs(taos_datap->proximity_input_dev, ABS_DISTANCE, data);
		prox_status_now = 100;
		if(debug_mask)
			printk("003----fxp--abs_distance-power on lcd --\n");
    } 
    else if (proxdata > taos_cfgp->prox_threshold_hi)
    {
        TSL2771_DBG(" prox input branch 2\n");
        if (cleardata > ((sat_als*80)/100)){
           // return -ENODATA;
           data = 1;
		   input_report_abs(taos_datap->proximity_input_dev, ABS_DISTANCE, data);
		   input_sync(taos_datap->proximity_input_dev);
		   return ret;
        }
        pro_buf[0] = taos_cfgp->prox_threshold_lo & 0x0ff;
        pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
        pro_buf[2] = 0xff;
        pro_buf[3] = 0xff;
		TSL2771_DBG("In 3, pro_buf[2] = %d, pro_buf[3] = %d", pro_buf[0], pro_buf[1]);
		data = 0;
		TSL2771_DBG("prox input report 3\n");
		input_report_abs(taos_datap->proximity_input_dev, ABS_DISTANCE, data);
		prox_status_now = 3;
		if(debug_mask)
			printk("004----fxp--abs_distance-power off lcd--\n");
    }

    input_sync(taos_datap->proximity_input_dev);

    for( mcount = 0; mcount < 4; mcount++ ) 
    { 
        if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x08) + mcount, pro_buf[mcount]))) < 0) 
		{
             printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos prox threshold set\n");
             return (ret);
        }
    }
	if(debug_mask)
	{
		printk("------fxp--set_poweron_lcd_interrupt_threshold=%d---\n", taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x08)|taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x09)<<8);
		printk("------fxp--set_poweroff_lcd_interrupt_threshold=%d---\n", taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0a)|taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x0b)<<8);
	}
    return ret;
}


// open
static int taos_open(struct inode *inode, struct file *file) 
{
	TSL2771_DBG("%s", __FUNCTION__);

    struct taos_data *taos_datap;
    int ret = 0;

    device_released = 0;
    taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
    if (strcmp(taos_datap->taos_name, TAOS_DEVICE_ID) != 0) 
    {
        printk(KERN_ERR "TAOS: device name incorrect during taos_open(), get %s\n", taos_datap->taos_name);
        ret = -ENODEV;
    }
    memset(readdata, 0, sizeof(struct ReadData)*2);//iVIZM
    return (ret);
}

// release
static int taos_release(struct inode *inode, struct file *file) 
{
	TSL2771_DBG("%s", __FUNCTION__);
    struct taos_data *taos_datap;
    int ret = 0;

    device_released = 1;
    prox_history_hi = 0;
    prox_history_lo = 0;
    taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
    if (strcmp(taos_datap->taos_name, TAOS_DEVICE_ID) != 0) 
	{
        printk(KERN_ERR "TAOS: device name incorrect during taos_release(), get %s\n", taos_datap->taos_name);
        ret = -ENODEV;
    }
    return (ret);
}

// read
static int taos_read(struct file *file, char *buf, size_t count, loff_t *ppos) 
{
	TSL2771_DBG("%s", __FUNCTION__);
    unsigned long flags;
    int realmax;
    int err;
    if((!ReadEnable) && (file->f_flags & O_NONBLOCK))
        return -EAGAIN;
    local_save_flags(flags);
    local_irq_disable();

    realmax = 0;
    if (down_interruptible(&taos_datap->update_lock))
        return -ERESTARTSYS;
    if (ReadEnable > 0) 
	{
        if (sizeof(struct ReadData)*2 < count)
            realmax = sizeof(struct ReadData)*2;
        else
            realmax = count;
        err = copy_to_user(buf, readdata, realmax);
        if (err) 
            return -EAGAIN;
        ReadEnable = 0;
    }
    up(&taos_datap->update_lock);
    memset(readdata, 0, sizeof(struct ReadData)*2);
    local_irq_restore(flags);
    return realmax;
}

// write
static int taos_write(struct file *file, const char *buf, size_t count, loff_t *ppos) 
{
	TSL2771_DBG("%s", __FUNCTION__);
    struct taos_data *taos_datap;
    u8 i = 0, xfrd = 0, reg = 0;
    u8 my_buf[TAOS_MAX_DEVICE_REGS];
    int ret = 0;

    if ((*ppos < 0) || (*ppos >= TAOS_MAX_DEVICE_REGS) || ((*ppos + count) > TAOS_MAX_DEVICE_REGS)) 
	{
        printk(KERN_ERR "TAOS: reg limit check failed in taos_write()\n");
        return -EINVAL;
    }
    reg = (u8)*ppos;
    if ((ret = copy_from_user(my_buf, buf, count))) 
	{
        printk(KERN_ERR "TAOS: copy_to_user failed in taos_write()\n");
        return -ENODATA;
    }
    taos_datap = container_of(file->f_dentry->d_inode->i_cdev, struct taos_data, cdev);
    while (xfrd < count) 
	{
        if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | reg), my_buf[i++]))) < 0) 
		{
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos_write()\n");
            return (ret);
        }
        reg++;
        xfrd++;
     }

     return ((int)xfrd);
}

// llseek
static loff_t taos_llseek(struct file *file, loff_t offset, int orig) 
{
	TSL2771_DBG("%s", __FUNCTION__);
    int ret = 0;
    loff_t new_pos = 0;

    if ((offset >= TAOS_MAX_DEVICE_REGS) || (orig < 0) || (orig > 1)) 
	{
        printk(KERN_ERR "TAOS: offset param limit or origin limit check failed in taos_llseek()\n");
        return -EINVAL;
    }
    switch (orig) 
	{
    case 0:
        new_pos = offset;
        break;
    case 1:
        new_pos = file->f_pos + offset;
        break;
    default:
        return -EINVAL;
        break;
    }
    if ((new_pos < 0) || (new_pos >= TAOS_MAX_DEVICE_REGS) || (ret < 0)) 
	{
        printk(KERN_ERR "TAOS: new offset limit or origin limit check failed in taos_llseek()\n");
        return -EINVAL;
    }
    file->f_pos = new_pos;

    return new_pos;
}

static int taos_sensors_als_on(void) 
{
	TSL2771_DBG("%s", __FUNCTION__);
    int  ret = 0, i = 0;
    u8 itime = 0, reg_val = 0, reg_cntrl = 0;
   
    for (i = 0; i < TAOS_FILTER_DEPTH; i++)
    	lux_history[i] = -ENODATA;

    itime = (((taos_cfgp->als_time/50) * 18) - 1);
    itime = (~itime);
    TSL2771_DBG("0x01 = 0x%x", itime);
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }

            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | 0x03), 0xF2))) < 0)    //golden
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }

            reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN);
            reg_val = reg_val & 0xFC;
            //reg_val = reg_val | (taos_cfgp->gain & 0x03) & 0xFD;  5.4
            reg_val = reg_val | (taos_cfgp->gain & 0x03);
			TSL2771_DBG("0x0F = 0x%x", reg_val);
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }

	    reg_cntrl = taos_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
	
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), (reg_cntrl |TAOS_TRITON_CNTL_ALS_INT_ENBL | TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)))) < 0) 
	    {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }
		taos_als_threshold_set();
	   return ret;
}	

static void set_threshold(void)
{
	/* feixiaoping@wind-mobi.com 20120620 begin */
	//we have not used this function 
    if(debug_mask)
    	printk("_------set_threshold--fxp----\n");
	/* feixiaoping@wind-mobi.com 20120620 end    */
    taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
    taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;

	//taos_cfgp->prox_threshold_lo_lo = prox_threshold_lo_param & 0xFF;
	//taos_cfgp->prox_threshold_lo_hi = (prox_threshold_lo_param >> 8) & 0xFF;
	//taos_cfgp->prox_threshold_hi_lo = prox_threshold_hi_param & 0xFF;
	//taos_cfgp->prox_threshold_hi_hi = (prox_threshold_hi_param >> 8) & 0xFF;
}
static void prox_power_on(void)
{    
			u8 reg_cntrl = 0;
			int ret = 0;

            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x02), taos_cfgp->prox_adc_time))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x03), taos_cfgp->prox_wait_time))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

            
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0C), taos_cfgp->prox_intr_filter))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0D), taos_cfgp->prox_config))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0E), 0x02))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0F), taos_cfgp->prox_gain))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
           
            reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | 0x20 |
				        	TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_WAIT_TMR_ENBL;
            //if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0)  /*0x02*/
             if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x2F))) < 0) 
            {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

	    	mdelay(300);
            mutex_lock(&prox_mutex);
			/* feixiaoping@wind-mobi.com 20120620 begin */
            //modify L400 can't wake up during calling only for some special phones
			if(debug_mask)
				printk("######fxp-----prox_power_on----\n");
			//set_threshold();//

			/* feixiaoping@wind-mobi.com 20120620 end */
            taos_prox_threshold_set();
            prox_status_now = 0;
            mutex_unlock(&prox_mutex);
#if 0			
	    if((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl | TAOS_TRITON_CNTL_PROX_INT_ENBL))) < 0)
            {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
#endif		
			PROX_ON = 1;
}

// ioctls
static int taos_ioctl(unsigned int cmd, char *arg)
//static int taos_ioctl(struct inode *inode,struct file *file, unsigned int cmd, unsigned long arg)
 
{
	//TSL2771_DBG("%s", __FUNCTION__);
     //struct i2c_client *client = (struct i2c_client *) file->private_data;

    int prox_sum = 0, prox_max = 0;
    int lux_val = 0, ret = 0, i = 0, tmp = 0;
    u16 gain_trim_val = 0;
    u8 reg_val = 0, reg_cntrl = 0;
    int ret_check=0;
    int ret_m=0;
    u8 reg_val_temp=0;
    void __user *argp = (void __user *)arg;
    switch(cmd) 
	{
        case TAOS_IOCTL_ALS_ON:
			if(debug_mask)
            	printk("------fxp------TAOS IOCTL ALS ON\n");
            taos_sensors_als_on();
			ALS_ON = 1;
            break;

        case TAOS_IOCTL_ALS_OFF:
			if(debug_mask)
            	printk("------fxp------TAOS IOCTL ALS OFF\n");
            for (i = 0; i < TAOS_FILTER_DEPTH; i++)
                lux_history[i] = -ENODATA;
            reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
		    if(1 == PROX_ON)
			{   
				printk("it is now phoneing\n");      
				if((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), /*(reg_val & 0xFF)*/ 0x2f))) < 0) 
				{
                   	printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
                   	return (ret);
			   	}
			}
	        if(0 == PROX_ON)
			{    
				printk("it is not phoneing\n");        
				if((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), (reg_val & 0xFC)))) < 0) 
				{
                   	printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
                   	return (ret);
			   	}
			}
			ALS_ON = 0;
            break;

        case TAOS_IOCTL_PROX_ON:
			if(debug_mask)
            	printk("------fxp------TAOS IOCTL PROX ON\n");
                        //set_threshold();//yongqiang add
			schedule_delayed_work(&prox_poweron_work, 0.1*HZ);
            break;

        case TAOS_IOCTL_PROX_OFF:
            TSL2771_DBG("TAOS IOCTL PROX OFF");
			reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			TSL2771_DBG("In TAOS_IOCTL_PROX_OFF, reg_val = 0x%x", reg_val);
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), (reg_val & 0xCB)))) < 0) 
            //if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x13))) < 0)
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
                return (ret);
            }
			
			
#if 1
            if (ALS_ON==1) 
			{
			    printk("####prox off->als_on##\n");
                taos_sensors_als_on();
				//ALS_ON = 1;
				
			} 
#endif
			PROX_ON = 0;
            break;

        default:
            return -EINVAL;
            break;
    }
    return (ret);
}

// read/calculate lux value
static int taos_get_lux(void) 
{
    u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
	u32 lux = 0;
	u32 ratio = 0;
	u8 dev_gain = 0;
	struct lux_data *p;
	u8 chdata[4];
	int tmp = 0, i = 0;

	for (i = 0; i < 4; i++) 
	{
		chdata[i] = taos_read_byte_data(TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i));
	}

//	TSL2771_DBG("chdata[0]= %d chdata[1]= %d chdata[2]= %d chdata[3]= %d \n",chdata[0],chdata[1],chdata[2],chdata[3]);
	tmp = (taos_cfgp->als_time + 25)/50;
    TritonTime.numerator = 1;
    TritonTime.denominator = tmp;

    tmp = 300 * taos_cfgp->als_time;
    if(tmp > 65535)
            tmp = 65535;
    TritonTime.saturation = tmp;
	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir    = chdata[3];
	raw_ir    <<= 8;
	raw_ir    |= chdata[2];
	if(raw_ir > raw_clear) 
	{
		raw_lux = raw_ir;
		raw_ir = raw_clear;
		raw_clear = raw_lux;
	}
	raw_clear *= taos_cfgp->scale_factor;	
	raw_ir *= taos_cfgp->scale_factor;
	dev_gain = taos_triton_gain_table[taos_cfgp->gain & 0x3];
    if(raw_clear >= lux_timep->saturation)
            return(TAOS_MAX_LUX);
    if(raw_ir >= lux_timep->saturation)
            return(TAOS_MAX_LUX);
    if(raw_clear == 0)
            return(0);
    if(dev_gain == 0 || dev_gain > 127) 
	{
		TSL2771_DBG( "TAOS: dev_gain = 0 or > 127 in taos_get_lux()\n");
                return -1;
	}
	
    if(lux_timep->denominator == 0)
	{
		TSL2771_DBG( "TAOS: lux_timep->denominator = 0 in taos_get_lux()\n");
                return -1;
	}
	ratio = (raw_ir<<15)/raw_clear;
	for (p = lux_tablep; p->ratio && p->ratio < ratio; p++);
        if(!p->ratio)
                return 0;
	lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
	lux = ((lux + (lux_timep->denominator >>1))/lux_timep->denominator) * lux_timep->numerator;
	lux = (lux + (dev_gain >> 1))/dev_gain/100;
	lux >>= TAOS_SCALE_MILLILUX;
        if(lux > TAOS_MAX_LUX)
                lux = TAOS_MAX_LUX;

//	TSL2771_DBG("In taos_get_lux, lux = %d", lux);
	return(lux);
}

static int taos_lux_filter(int lux)
{
//	TSL2771_DBG("%s, lux = %d", __FUNCTION__, lux);
    static u8 middle[] = {1,0,2,0,0,2,0,1};
    int index;

    lux_history[2] = lux_history[1];
    lux_history[1] = lux_history[0];
    lux_history[0] = lux;

    if(lux_history[2] < 0)  //iVIZM
	{
        if(lux_history[1] > 0)
            return lux_history[1];       
        else 
            return lux_history[0];
    }
    index = 0;
    if(lux_history[0] > lux_history[1]) 
        index += 4;
    if(lux_history[1] > lux_history[2]) 
        index += 2;
    if(lux_history[0] > lux_history[2])
        index++;
    return(lux_history[middle[index]]);
}

/*
 * sysfs device attributes
 */
static void proximity_irq_monitor(void)
{
    u8 chdata[6];
    u16 proxdata = 0;
    int data = 0;
	if(debug_mask)
			printk("------fxp------monitor---proximity_irq_monitor\n");
    if(i2c_smbus_read_i2c_block_data(taos_datap->client, TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_WORD_BLK_RW|TAOS_TRITON_ALS_CHAN0LO|0x80, 6, chdata) <= 0)
    {
		TSL2771_DBG("In %s, read fail", __FUNCTION__);
		return -ENODATA;
    }
    proxdata = chdata[4] + chdata[5] * 256;
    if(((proxdata <= taos_cfgp->prox_threshold_lo) && (3 == prox_status_now)) || 
    	((proxdata > taos_cfgp->prox_threshold_hi) && (100 == prox_status_now)))
	{
		if(debug_mask)
			printk("------fxp------monitor lost irq\n");
		mutex_lock(&prox_mutex);
    	taos_prox_threshold_set();
    	taos_interrupts_clear();
    	mutex_unlock(&prox_mutex);
    }

    schedule_delayed_work(&monitor_prox_work, 2 * HZ);
}

static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
//	return sprintf(buf, "%d\n", lis33de_get_enable(dev));
	TSL2771_DBG("proximity_enable = %d", taos_datap->proximity_enable);
	return sprintf(buf, "%d\n", taos_datap->proximity_enable);
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
	unsigned long enable = simple_strtoul(buf, NULL, 10);
//	lis33de_set_enable(dev, enable);
    if(debug_mask)
		printk("------fxp-----proximity_enable = %d\n", enable);
	if((0 == taos_datap->proximity_enable) && (1 == enable))
	{
		 taos_ioctl(TAOS_IOCTL_PROX_ON, NULL);
#if 0
		if(0 == locked)
		{
    		wake_lock(&taos_datap->taos_wake_lock);
			locked = 1;
		}
#endif
		schedule_delayed_work(&monitor_prox_work, 0.1 * HZ);
		if(debug_mask)
			printk("------fxp------Start monitor_prox_work\n");
	}
	if((1 == taos_datap->proximity_enable) && (0 == enable))
	{
		cancel_delayed_work_sync(&monitor_prox_work);
		if(debug_mask)
			printk("------fxp------End monitor_prox_work\n");
		taos_ioctl(TAOS_IOCTL_PROX_OFF, NULL);
#if 0
		if(1 == locked)
		{
			wake_unlock(&taos_datap->taos_wake_lock);
			locked = 0;
		}
#endif
	}
	taos_datap->proximity_enable = enable;
	return count;
}

static ssize_t proximity_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
	return sprintf(buf, "%d\n", taos_datap->proximity_delay);
}

static ssize_t proximity_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
	unsigned long delay = simple_strtoul(buf, NULL, 10);
/*
	if(delay > LIS33DE_MAX_DELAY)
	{
		delay = LIS33DE_MAX_DELAY;
	}
*/
//	lis33de_set_delay(dev, delay);
	taos_datap->proximity_delay = delay;
	return count;
}


/*
 * sysfs device attributes
 */
static void asensor_work(unsigned long data)
{
	if(1 == taos_datap->ambient_enable)
	{
		taos_als_get_data();
		schedule_delayed_work(&taos_datap->d_work, 0.2 * HZ);
	}
	if(0 == taos_datap->ambient_enable)
	{
		timer_on = 0;
		TSL2771_DBG("Timer off");
	}
}

static ssize_t ambient_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
	return sprintf(buf, "%d\n", taos_datap->ambient_enable);
}

static ssize_t ambient_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	unsigned long enable = simple_strtoul(buf, NULL, 10);
	TSL2771_DBG("Come into %s ambient_enable=%d enable = %d", __FUNCTION__,taos_datap->ambient_enable,enable);

	if((0 == taos_datap->ambient_enable) && (1 == enable))
	{
		//taos_sensors_als_on();//wilson
		taos_ioctl(TAOS_IOCTL_ALS_ON, NULL);//wilson
		TSL2771_DBG("wilson enter als_get_data %d\n",enable);
		taos_als_get_data();
		TSL2771_DBG("wilson exit als_get_data %d\n",enable);
	}
	if((1 == taos_datap->ambient_enable) && (0 == enable))
	{
		taos_ioctl(TAOS_IOCTL_ALS_OFF, NULL);
	}

	taos_datap->ambient_enable = enable;
	TSL2771_DBG("ambient enable = %d", taos_datap->ambient_enable);
	return count;
}

static ssize_t ambient_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
	return sprintf(buf, "%d\n", taos_datap->ambient_delay);
}

static ssize_t ambient_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	TSL2771_DBG("Come into %s", __FUNCTION__);
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	taos_datap->ambient_delay = delay;
	TSL2771_DBG("delay = %d", taos_datap->ambient_delay);
	if((1 == taos_datap->ambient_enable) && (0 == timer_on))
	{
		TSL2771_DBG("start up ambient timer");
		schedule_delayed_work(&taos_datap->d_work, 0.2 * HZ);
		timer_on = 1;
	}

	return count;
}


/* feixiaoping@wind-mobi.com 20120525 begin */
//modify for CTS fail item:lightsensor and proximity notwritable
static DEVICE_ATTR(proximity_active, 0666, proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(proximity_interval, 0644, proximity_delay_show, proximity_delay_store);

static DEVICE_ATTR(ambient_active, 0666, ambient_enable_show, ambient_enable_store);
static DEVICE_ATTR(ambient_interval, 0644, ambient_delay_show, ambient_delay_store);//0777
/* feixiaoping@wind-mobi.com 20120525 end */


#if 0
static DEVICE_ATTR(position, S_IRUGO | S_IWUGO, lis33de_position_show, lis33de_position_store);
static DEVICE_ATTR(wake, S_IWUGO, NULL, lis33de_wake_store);
static DEVICE_ATTR(data, S_IRUGO, lis33de_data_show, NULL);
#if DEBUG
static DEVICE_ATTR(debug_reg, S_IRUGO, lis33de_debug_reg_show, NULL);
static DEVICE_ATTR(debug_suspend, S_IRUGO | S_IWUGO, lis33de_debug_suspend_show, lis33de_debug_suspend_store);
#endif /* DEBUG */
#endif
static struct attribute *proximity_attributes[] =
{
	&dev_attr_proximity_active.attr,
	&dev_attr_proximity_interval.attr,
#if 0
	&dev_attr_position.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
#endif
	NULL
};
static struct attribute_group proximity_attribute_group =
{
	.attrs = proximity_attributes
};


static struct attribute *ambient_attributes[] =
{
	&dev_attr_ambient_active.attr,
	&dev_attr_ambient_interval.attr,
#if 0
	&dev_attr_position.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
#endif
	NULL
};
static struct attribute_group ambient_attribute_group =
{
	.attrs = ambient_attributes
};

static void poweron_prox(void)
{
	u8 reg_val = 0, ret = 0;
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x01), taos_cfgp->prox_int_time))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x02), taos_cfgp->prox_adc_time))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x03), taos_cfgp->prox_wait_time))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0C), taos_cfgp->prox_intr_filter))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0D), taos_cfgp->prox_config))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0E), taos_cfgp->prox_pulse_cnt))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x0F), taos_cfgp->prox_gain))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

			reg_val = taos_read_byte_data(TAOS_TRITON_CMD_REG | 0x00);
	        if ((ret = (taos_write_byte_data((TAOS_TRITON_CMD_REG|0x00), reg_val | 0x0D))) < 0) 
			{
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
}

static void get_prox_threshold(void)
{
	int prox_max = 0, prox_value = 0, prox_now = 0, prox_mean,i;

	if(0 == prox_adjust)
	{
		poweron_prox();	
		mdelay(100);
	}
	for(i = 0; i < 20; i++)
	{
		prox_now = i2c_smbus_read_word_data(taos_datap->client, 0x18 | 0x80);
		TSL2771_DBG("current distence = %d", prox_now);
		prox_max = (prox_now > prox_max) ? prox_now : prox_max;
		prox_value += prox_now;
		prox_mean=prox_value/20;
		//mdelay(80);
		mdelay(20);
	}
	if(prox_mean < 40) {
		taos_cfgp->prox_threshold_hi = prox_mean * 8 ;
		taos_cfgp->prox_threshold_lo= prox_mean * 22 / 10;
	}
	else if(prox_mean >= 40 && prox_mean < 45) {
		taos_cfgp->prox_threshold_hi= prox_mean * 75 /10;
		taos_cfgp->prox_threshold_lo= prox_mean * 22 / 10;
	}
	else if(prox_mean >= 45 && prox_mean < 50) {
		taos_cfgp->prox_threshold_hi = prox_mean * 7;
		taos_cfgp->prox_threshold_lo = prox_mean * 22 / 10;
	}
	else if(prox_mean >= 50 &&prox_mean < 60) {
		taos_cfgp->prox_threshold_hi = prox_mean * 6;
		taos_cfgp->prox_threshold_lo = prox_mean * 2;
	}
	else if(prox_mean>= 60 && prox_mean< 70) {
		taos_cfgp->prox_threshold_hi = prox_mean * 55/10;
		taos_cfgp->prox_threshold_lo= prox_mean * 19 / 10;
	}
	else if(prox_mean >= 70 && prox_mean <80) {
		taos_cfgp->prox_threshold_hi = prox_mean * 5;
		taos_cfgp->prox_threshold_lo = prox_mean * 18 / 10;
	}
	else if(prox_mean >= 80 && prox_mean <100) {
		taos_cfgp->prox_threshold_hi = prox_mean * 45/10;
		taos_cfgp->prox_threshold_lo = prox_mean * 17 / 10;
	}
	else if(prox_mean >= 100 && prox_mean < 150) {
		taos_cfgp->prox_threshold_hi = prox_mean * 4;
		taos_cfgp->prox_threshold_lo = prox_mean * 16 / 10;
	}
	else if(prox_mean >= 150 && prox_mean < 200) {
		taos_cfgp->prox_threshold_hi = prox_mean * 35/10;
		taos_cfgp->prox_threshold_lo = prox_mean * 16 / 10;
	}
	else if(prox_mean >= 200 && prox_mean < 300) {
		taos_cfgp->prox_threshold_hi = prox_mean * 3;
		taos_cfgp->prox_threshold_lo = prox_mean * 15 / 10;
	}
	
	else {
		taos_cfgp->prox_threshold_hi= prox_mean * 25 / 10;
		taos_cfgp->prox_threshold_lo = prox_mean * 14 / 10;
	}
	if(taos_cfgp->prox_threshold_hi > 900 ){
		taos_cfgp->prox_threshold_hi  = 900;
		taos_cfgp->prox_threshold_lo = 750;
	}
	if(taos_cfgp->prox_threshold_lo < 100 ){
		taos_cfgp->prox_threshold_hi  = 200;
		taos_cfgp->prox_threshold_lo  = 150;
	}
	if(debug_mask)
	{
		printk("----------------fxp---- prox_max = %d \n--------------------", prox_max);
		//prox_mean = prox_value >> 5;
		printk("-----------------fxp--- prox_mean = %d \n--------------------", prox_mean);
	}
#if 0
	if(prox_mean <= 400)
	{
		prox_threshold_hi_param = prox_max + 140;
		prox_threshold_lo_param = prox_threshold_hi_param - 130;
	}
	else if(prox_mean > 400)
	{
		prox_threshold_hi_param = 500;
		prox_threshold_lo_param = prox_threshold_hi_param - 130;
	}
	
	printk("-------------------- prox_threshold_hi_param = %d, prox_threshold_lo_param = %d\n\n", prox_threshold_hi_param, prox_threshold_lo_param);
#endif

	if(0 == prox_adjust)
	{
		taos_ioctl(TAOS_IOCTL_PROX_OFF, NULL);
	}
}






static int taos_misc_open(struct inode *inode, struct file *file)
{
	TSL2771_DBG("%s", __FUNCTION__);
	return nonseekable_open(inode, file);
}
static int taos_misc_release(struct inode *inode, struct file *file)
{
}
static int taos_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	TSL2771_DBG("Come into %s, cmd = %d", __FUNCTION__, cmd);
	void __user *argp = (void __user *)arg;

	int lux_val_init = 0;

	switch(cmd) 
	{
		case MISC_IOCTL_GET_INIT_VALUE:
			if ((lux_val_init = taos_get_lux()) < 0)
        		printk(KERN_ERR "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n", lux_val_init);
    		lux_val_init = taos_lux_filter(lux_val_init);
			TSL2771_DBG("lux_val from taos_get_lux = %d", lux_val_init);
    		if(copy_to_user(argp, &lux_val_init, sizeof(lux_val_init))) 
			{
				return -EFAULT;
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static struct file_operations taos_misc_fops = 
{
	.owner = THIS_MODULE,
	.open = taos_misc_open,
	.release = taos_misc_release,
	.ioctl = taos_misc_ioctl,
};

static struct miscdevice taos_misc_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tsl2771_misc_dev",
	.fops = &taos_misc_fops,
};

static void check_prox_mean()
{
	if(prox_mean <= 100)
	{
		prox_threshold_hi_param = prox_mean * 8 / 5;
		prox_threshold_lo_param = prox_mean * 3 / 2;
	}
	else if(prox_mean > 100 && prox_mean <= 300)
	{
		prox_threshold_hi_param = prox_mean *  7 / 5;
		prox_threshold_lo_param = prox_mean * 13 / 10;
	}
	else if(prox_mean > 300)
	{
		prox_threshold_hi_param = prox_mean * 6 / 5;
		prox_threshold_lo_param = prox_mean * 11 / 10;
		if(prox_threshold_hi_param > 1023)
		{
			prox_threshold_hi_param = 999;
			prox_threshold_lo_param = 950;
		}
	}

	if(prox_threshold_hi_param < 100)
	{
		prox_threshold_hi_param = 200;
		prox_threshold_lo_param = 150;
	}
}



static ssize_t alsps_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{	
	prox_adjust = 1;

	mutex_lock(&prox_mutex);
	get_prox_threshold();
	//check_prox_mean();
	//set_threshold();
	taos_prox_threshold_set();
	prox_status_now = 0;
	mutex_unlock(&prox_mutex);
	sprintf(page, "%d\n", prox_mean);

	prox_adjust = 0;
}

static ssize_t alsps_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
//	do{}
//	while(prox_get_complete != 1);
	int ret = 0;
	int value = 0;
	sscanf(buff, "%d", &value);
	prox_mean = value;
	check_prox_mean();
	set_threshold();
	if(debug_mask)
		printk("-----fxp---prox_threshold_hi_param = %d, prox_threshold_lo_param = %d\n", prox_threshold_hi_param, prox_threshold_lo_param);
	
}

static void create_alsps_proc_file(void)
{
	struct proc_dir_entry *alsps_proc_file = create_proc_entry("driver/tsl2771_threshold", 0644, NULL);

	if (alsps_proc_file) 
	{
		alsps_proc_file->read_proc = alsps_read_proc;
		alsps_proc_file->write_proc = (write_proc_t *)alsps_write_proc;
	} 
	else
		printk(KERN_INFO "alsps proc file create failed!\n");
}

static void get_prox_threshold_work(void)
{
	get_prox_threshold();
	set_threshold();
	prox_get_complete = 1;
	if(debug_mask)
		printk("----fxp-----get_prox_threshold_work--\n");
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void taos_early_suspend(struct early_suspend *handler)
{
	struct taos_data  *tdata = container_of(handler, struct taos_data, suspend_desc);
	//taos_interrupts_clear();
	if(debug_mask)
	{
		printk("------------fxp-----------------taos_early_suspend\n");
	}

}

static void taos_late_resume(struct early_suspend *handler)
{

	struct taos_data *tdata = container_of(handler, struct taos_data, suspend_desc);
	taos_get_data();
	taos_interrupts_clear();
	if(debug_mask)
	{
		printk("------------fxp-----------------taos_late_resume\n");
	}
	//taos_interrupts_clear();
}
#endif




#ifdef TAOS_CALIBRATE

#define TMD2771X_AUTO_INCREMENT_COMMAND                0xa0
#define TMD2771X_PDATAL                                0x18
#define TMD2771X_DEFAULT_COMMAND               0x80
#define TMD2771X_ENABLE                                0x00
#define TMD2771X_PEN_SHIFT                     (2)
#define TMD2771X_PEN                           (0x1 << TMD2771X_PEN_SHIFT)
#define TMD2771X_PON_SHIFT                     (0)
#define TMD2771X_PON                           (0x1 << TMD2771X_PON_SHIFT)



static int tsl_calc_proximity_threshold(uint16_t val, 
				uint16_t *hi, uint16_t *lo)
{

//improve proximity sensor calibrate 

	if(!hi || !lo || !val)
		return -1;

//during calling ,even though press powerkey can't wake up the screen
	if(val < 40) {
		*hi = val * 8 ;
		*lo = val * 22 / 10;
	}
	else if(val >= 40 && val < 45) {
		*hi = val * 75 /10;
		*lo = val * 22 / 10;
	}
	else if(val >= 45 && val < 50) {
		*hi = val * 7;
		*lo = val * 22 / 10;
	}
	else if(val >= 50 && val < 60) {
		*hi = val * 6;
		*lo = val * 2;
	}
	else if(val >= 60 && val < 70) {
		*hi = val * 55/10;
		*lo = val * 19 / 10;
	}
	else if(val >= 70 && val <80) {
		*hi = val * 5;
		*lo = val * 18 / 10;
	}
	else if(val >= 80 && val <100) {
		*hi = val * 45/10;
		*lo = val * 17 / 10;
	}
	else if(val >= 100 && val < 150) {
		*hi = val * 4;
		*lo = val * 16 / 10;
	}
	else if(val >= 150 && val < 200) {
		*hi = val * 35/10;
		*lo = val * 16 / 10;
	}
	else if(val >= 200 && val < 300) {
		*hi = val * 3;
		*lo = val * 15 / 10;
	}
	
	else {
		*hi = val * 25 / 10;
		*lo = val * 14 / 10;
	}
	if(*hi > 900 ){
		  *hi = 900;
		  *lo = 750;
	}

 
    printk("[WYQ] *hi = %d,*lo = %d,cal= %d\n",*hi,*lo,val);
	return 0;

}

static int taos_write_reg(struct i2c_client *client, u8 command, u8 value)
{
       int ret;
       unsigned long timeout;
       int retry = 0;

       timeout = jiffies + msecs_to_jiffies(1000);
       while(time_before(jiffies, timeout)) {
              ret = i2c_smbus_write_byte_data(client, command, value);
              if((ret == 0) || (ret != -EAGAIN))
                     break;

              retry++;
              msleep(10);
       }
       if (ret < 0)
              dev_err(&client->dev, "%s: command 0x%x, err %d\n",
                      __func__, command, ret);

       if(retry > 0 && ret >= 0)
              dev_warn(&client->dev, "%s() retry=%d\n", __func__, retry);

       return ret;
	   
}


static int taos_read_reg(struct i2c_client *client, u8 command,
                            u8 *value, u8 length)
{
       int ret;
       unsigned long timeout;
       int retry = 0;

       timeout = jiffies + msecs_to_jiffies(1000);
       while(time_before(jiffies, timeout)) {
              if (length == 1) {
                      ret = i2c_smbus_read_byte_data(client, command);
                      if (ret >= 0)
                              *value = (u8)ret;
              } else
                      ret = i2c_smbus_read_i2c_block_data(client, command,
                                                   length, value);
              if((ret >= 0) || (ret != -EAGAIN))
                     break;

              retry++;
              msleep(10);
       }
       if (ret < 0)
              dev_err(&client->dev, "%s: command 0x%x, err %d\n",
                       __func__, command, ret);

       if(retry > 0 && ret >= 0)
              dev_warn(&client->dev, "%s() retry=%d\n", __func__, retry);

       return ret;
}	   

static int taos_proximity_calibrate( struct taos_data *tdata,
                int count)
{
	int retval, retcnt = 0;
	u16 min_val, max_val, avg_val, val;
	u32 total;
    
	total = 0;
	while(count-- > 0) {
		u8 regval[2];
		retval = wait_for_completion_interruptible_timeout(
					&tdata->cali_completion, 
					msecs_to_jiffies(10));
		if(retval < 0)
			break;

		taos_read_reg(tdata->client,
				TMD2771X_AUTO_INCREMENT_COMMAND | TMD2771X_PDATAL,
				regval, 2);
		val = (regval[1] << BITS_PER_BYTE) | regval[0];
		TSL2771_DBG("[WYQ] taos_proximity_calibrate val=%d\n",val);
		if(!val)
			continue;


		if(unlikely(!retcnt)) {
			min_val = val;
			max_val = val;
			avg_val = val;
		}
		else {
			if(val > max_val)
				max_val = val;
			if(val < min_val)
				min_val = val;
		}
		total += val;
		retcnt++;
	}

	if(retcnt > 0) {
		tdata->cali_min_val = min_val;
		tdata->cali_max_val = max_val;
		tdata->cali_avg_val = total / retcnt;

	}
	TSL2771_DBG("###tdata->cali_avg_val=%d###",tdata->cali_avg_val);
	return retcnt;
}

static u8 taos_calc_enable_value(struct taos_data *tdata)
{
	   u8 value;
#if 0	   
       value = tdata->pdata->ps_interrupt_enable |
               tdata->pdata->als_interrupt_enable |
               tdata->pdata->wait_enable | tdata->pdata->ps_enable |
               tdata->pdata->als_enable | tdata->pdata->power_on;
#endif
       return 1;
}

static ssize_t taos_cali_trigger_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf,
                size_t count)
{
	struct taos_data *tdata
		= container_of(kobj, struct taos_data, cali_kobj);
	int cali_val;
	int retval = 0;
  	cali_val = simple_strtol(buf, NULL, 10);

	mutex_lock(&tdata->lock);
	if(cali_val > 0) {
		u8 enable = TMD2771X_PEN | TMD2771X_PON;

		tdata->cali_mode = 1;

		taos_write_reg(tdata->client, 
				TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE,
				enable);
        taos_ioctl(TAOS_IOCTL_PROX_ON, NULL);
		retval = taos_proximity_calibrate(tdata, cali_val);

		enable = taos_calc_enable_value(tdata);
		taos_write_reg(tdata->client,
				TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE,
				enable);

		tdata->cali_mode = 0;
	}
	mutex_unlock(&tdata->lock);

	/* Send notifications while calibrate succeed. */
	if(retval > 0) {
		sysfs_notify(&tdata->cali_kobj, NULL, "avg_value");
		sysfs_notify(&tdata->cali_kobj, NULL, "max_value");
		sysfs_notify(&tdata->cali_kobj, NULL, "min_value");
	}

	return count;

}


static ssize_t taos_cali_max_value_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct taos_data *tdata
		= container_of(kobj, struct taos_data, cali_kobj);
 
	return sprintf(buf, "%d\n", tdata->cali_max_val);
}

static ssize_t taos_cali_min_value_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct taos_data *tdata
		= container_of(kobj, struct taos_data, cali_kobj);
 
	return sprintf(buf, "%d\n", tdata->cali_min_val);
}

static ssize_t taos_cali_avg_value_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct taos_data *tdata
		= container_of(kobj, struct taos_data, cali_kobj);	
	return sprintf(buf, "%d\n", tdata->cali_avg_val);
}

static ssize_t taos_cali_cal_value_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct taos_data *tdata
		= container_of(kobj, struct taos_data, cali_kobj);

	return sprintf(buf, "%d\n", tdata->cali_cur_val);
}


static ssize_t taos_cali_cal_value_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf,
                size_t count)
{
	struct taos_data *tdata
		= container_of(kobj, struct taos_data, cali_kobj);

	tdata->cali_cur_val = simple_strtol(buf, NULL, 10);
	tsl_calc_proximity_threshold((uint16_t)tdata->cali_cur_val ,
					taos_cfgp->prox_threshold_hi, taos_cfgp->prox_threshold_lo);  
	sysfs_notify(kobj, NULL, "cal_value");

	return count;
}

#define KOBJ_ATTR(_name, _mode, _show, _store)   \
struct kobj_attribute kobj_attr_##_name = __ATTR(_name, _mode, _show, _store)


//sensor cts permission

KOBJ_ATTR(trigger, 0600, NULL, taos_cali_trigger_store);
KOBJ_ATTR(max_value, 0444, taos_cali_max_value_show, NULL);
KOBJ_ATTR(min_value, 0444, taos_cali_min_value_show, NULL);
KOBJ_ATTR(avg_value, 0444, taos_cali_avg_value_show, NULL);
                       
KOBJ_ATTR(cal_value, 0600, taos_cali_cal_value_show,
                           taos_cali_cal_value_store);


static struct attribute *taos_calibrate_attributes[] = {
        &kobj_attr_trigger.attr,
        &kobj_attr_max_value.attr,
        &kobj_attr_min_value.attr,
        &kobj_attr_avg_value.attr,
        &kobj_attr_cal_value.attr,
        NULL
};

static ssize_t cali_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
                char *buf)
{
        struct kobj_attribute *kattr;
        ssize_t ret = -EIO;

        kattr = container_of(attr, struct kobj_attribute, attr);
        if (kattr->show)
                ret = kattr->show(kobj, kattr, buf);
        return ret;
}

static ssize_t cali_kobj_attr_store(struct kobject *kobj, struct attribute *attr,
                const char *buf, size_t count)
{
        struct kobj_attribute *kattr;
        ssize_t ret = -EIO;

        kattr = container_of(attr, struct kobj_attribute, attr);
        if (kattr->store)
                ret = kattr->store(kobj, kattr, buf, count);
        return ret;
}

struct sysfs_ops cali_sysfs_ops = {
        .show = cali_kobj_attr_show,
        .store = cali_kobj_attr_store,
};

static void cali_kobj_release(struct kobject *kobj)
{
        pr_debug("kobject: (%p): %s\n", kobj, __func__);
}

static struct kobj_type cali_kobj_ktype = {
        .release = cali_kobj_release,
        .sysfs_ops = &cali_sysfs_ops,
        .default_attrs = taos_calibrate_attributes,
};


#endif


#define ALS_PS_GPIO 14
// client probe
static int taos_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("taos probe\n");
    int ret = 0;
    int i = 0; 
    unsigned char buf[TAOS_MAX_DEVICE_REGS];
    char *device_name;
    unsigned char chip_id = 0;
	struct input_dev *input_data_p = NULL;
	struct input_dev *input_data_a = NULL;

	if(client == NULL)
	{
		TSL2771_DBG("Client is NUll!\n");
        return -ENOMEM;
	}

	taos_datap->client = client;

	chip_id = i2c_smbus_read_byte_data(taos_datap->client, TAOS_TRITON_CMD_REG | 0x12);
	TSL2771_DBG("tsl2771 chip id = 0x%x\n", chip_id);
	//if((chip_id != 0x29) && (chip_id != 0x20))
	//	{
		//printk("~~~~~~~~~~~~~~~~~~~ Not taos vendor, return ~~~~~~~~~~~~~~~~~~~~~~\n");
  	//     	return -ENOMEM;
	//	}

	dev_set_drvdata(&client->dev, taos_datap);

    INIT_WORK(&(taos_datap->work), taos_work_func);
    init_MUTEX(&taos_datap->update_lock);
	mutex_init(&taos_datap->lock);

    strlcpy(taos_datap->taos_name, TAOS_DEVICE_ID, TAOS_ID_NAME_SIZE);
    taos_datap->valid = 0;
	taos_datap->irq = client->irq;
    if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL))) 
	{
        printk(KERN_ERR "TAOS: kmalloc for struct taos_cfg failed in taos_probe()\n");
        return -ENOMEM;
    }
    taos_cfgp->calibrate_target = calibrate_target_param;
    taos_cfgp->als_time = als_time_param;
    taos_cfgp->scale_factor = scale_factor_param;
    taos_cfgp->gain_trim = gain_trim_param;
    taos_cfgp->filter_history = filter_history_param;
    taos_cfgp->filter_count = filter_count_param;
    taos_cfgp->gain = gain_param;
    taos_cfgp->prox_int_time = prox_int_time_param;
    taos_cfgp->prox_adc_time = prox_adc_time_param;
    taos_cfgp->prox_wait_time = prox_wait_time_param;
    taos_cfgp->prox_intr_filter = prox_intr_filter_param;
    taos_cfgp->prox_config = prox_config_param;
    taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
    taos_cfgp->prox_gain = prox_gain_param;
	taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
    taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
    sat_als = (256 - taos_cfgp->prox_int_time) << 10;
    sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;

//	INIT_DELAYED_WORK(&get_threshold_work, get_prox_threshold_work);
//	schedule_delayed_work(&get_threshold_work, 1*HZ);

	INIT_DELAYED_WORK(&prox_poweron_work, prox_power_on);

	INIT_DELAYED_WORK(&monitor_prox_work, proximity_irq_monitor);
        taos_datap->taos_dev = class_create(THIS_MODULE, "optical_sensors");
	mutex_init(&prox_mutex);
      ret = gpio_request(ALS_PS_GPIO,"ALS_PS_INT");//iVIZM
    if (ret < 0) {
        printk("failed to request GPIO:%d,ERRNO:%d\n",(int)ALS_PS_GPIO,ret);
       return(ret);
    }
    gpio_direction_input(ALS_PS_GPIO);//iVIZM
//	ret = request_irq(taos_datap->irq, taos_irq_handler, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "proximity_irq", taos_datap);
	ret = request_irq(GPIO_TO_IRQ(14), taos_irq_handler, IRQF_TRIGGER_FALLING, "tsl2771", taos_datap);
    if (ret != 0) 
	{
		free_irq(taos_datap->irq, taos_datap);
        return(ret);
    }

   	input_data_p = input_allocate_device();//iVIZM
   	if(input_data_p == NULL) 
	{
       	return -ENOMEM;
   	}
   	input_data_p->name = "proximity";
	input_data_p->id.bustype = BUS_I2C;

	input_set_capability(input_data_p, EV_ABS, ABS_MISC);
	input_set_abs_params(input_data_p, ABS_DISTANCE, 0, 1, 0, 0);

   	ret = input_register_device(input_data_p);
	if(ret < 0) 
	{
		input_free_device(input_data_p);
		return ret;
	}
	taos_datap->proximity_input_dev = input_data_p;
        taos_datap->p_dev = device_create(taos_datap->taos_dev,
                NULL, 0, "%s", "proximity");
	ret = sysfs_create_group(&taos_datap->p_dev->kobj, &proximity_attribute_group);
	if(ret < 0) 
	{
		return -ENOMEM;
	}


//ambient
   	input_data_a = input_allocate_device();//iVIZM
   	if(input_data_a == NULL) 
	{
       	return -ENOMEM;
   	}
   	input_data_a->name = "lightsensor";
	input_data_a->id.bustype = BUS_I2C;

	input_set_capability(input_data_a, EV_ABS, ABS_MISC);
	input_set_abs_params(input_data_a, ABS_PRESSURE, -100000, 100000000, 0, 0);
        //set_bit(EV_ABS, input_data_a->evbit);
	//input_set_abs_params(input_data_a, ABS_MISC, 0, 9, 0, 0);
   	ret = input_register_device(input_data_a);
	if(ret < 0) 
	{
		input_free_device(input_data_a);
		return ret;
	}
	taos_datap->ambient_input_dev = input_data_a;
        taos_datap->l_dev = device_create(taos_datap->taos_dev,
                NULL, 0, "%s", "lightsensor");
	ret = sysfs_create_group(&taos_datap->l_dev->kobj, &ambient_attribute_group);
	if(ret < 0) 
	{
		return -ENOMEM;
	}

	printk("Now register taos misc\n");
	ret = misc_register(&taos_misc_device);
	if(ret) 
	{
		TSL2771_DBG("misc_device register failed\n");
		return -ENOMEM;
	}
	
	INIT_DELAYED_WORK(&taos_datap->d_work, asensor_work);
	wake_lock_init(&proximity_wakelock, WAKE_LOCK_SUSPEND, "proximity_wakelock");

		
#ifdef TAOS_CALIBRATE_NEW
		get_prox_threshold();
#endif

	create_alsps_proc_file();
#ifdef CONFIG_HAS_EARLYSUSPEND
       //taos_datap->suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1; //EARLY_SUSPEND_LEVEL_DISABLE_FB
        taos_datap->suspend_desc.level = EARLY_SUSPEND_LEVEL_DISABLE_FB+1;
       taos_datap->suspend_desc.suspend = taos_early_suspend;
       taos_datap->suspend_desc.resume =  taos_late_resume;
       register_early_suspend(&taos_datap->suspend_desc);
#endif

	//prox_power_on();
	//taos_sensors_als_on();
#if 0//def TAOS_CALIBRATE	
	 ret = kobject_init_and_add(&taos_datap->cali_kobj, &cali_kobj_ktype,
               &input_data_p->dev.kobj, "calibrate");
       if(ret)
       	       goto error_add_kobject;
       init_completion(&taos_datap->cali_completion);
       taos_datap->cali_mode = 0;
       taos_datap->cali_max_val = 0;
       taos_datap->cali_min_val = 0;
       taos_datap->cali_avg_val = 0;
       taos_datap->cali_cur_val = 0;
#endif	   

	TSL2771_DBG("TSL2771 probe success!");
    return (ret);
#ifdef TAOS_CALIBRATE
error_add_kobject:
       kobject_del(&taos_datap->cali_kobj);
#endif	   
}

static int __devexit taos_remove(struct i2c_client *client) 
{
    TSL2771_DBG("%s", __FUNCTION__);
    misc_deregister(&taos_misc_device);
    int ret = 0;
    return (ret);
}

static int taos_shutdown(struct i2c_client *client)
{
	printk("%s", __FUNCTION__);
	cancel_delayed_work_sync(&taos_datap->d_work);
}




// driver definition
static const struct i2c_device_id taos_idtable[] = 
{
	{"tsl2771", 0},
	{}
};

static struct i2c_driver taos_driver = 
{
    .driver = 
	{
		.owner = THIS_MODULE,
        .name = "tsl2771",
    },
    .id_table = taos_idtable,
    .probe = taos_probe,
    .remove = __devexit_p(taos_remove),
    .shutdown = taos_shutdown,
};

// driver init
static int __init tsl2771_init(void) 
{
//	TSL2771_DBG("%s", __FUNCTION__);
    int ret = 0, k = 0;

    if ((ret = (alloc_chrdev_region(&taos_dev_number, 0, TAOS_MAX_NUM_DEVICES, "taos"))) < 0) 
	{
        printk(KERN_ERR "TAOS: alloc_chrdev_region() failed in taos_init()\n");
        return (ret);
    }
    taos_class = class_create(THIS_MODULE, "taos");

    taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
    if (!taos_datap) 
	{
        printk(KERN_ERR "TAOS: kmalloc for struct taos_data failed in taos_init()\n");
        return -ENOMEM;
    }
    memset(taos_datap, 0, sizeof(struct taos_data));

    cdev_init(&taos_datap->cdev, &taos_fops);
    taos_datap->cdev.owner = THIS_MODULE;
    if ((ret = (cdev_add(&taos_datap->cdev, taos_dev_number, 1))) < 0) 
	{
        printk(KERN_ERR "TAOS: cdev_add() failed in taos_init()\n");
        return (ret);
    }

//    wake_lock_init(&taos_datap->taos_wake_lock, WAKE_LOCK_SUSPEND, "prox_lock");
    device_create(taos_class, NULL, MKDEV(MAJOR(taos_dev_number), 0), &taos_driver , "taos");

//	return platform_driver_register(&taos_driver);
	return i2c_add_driver(&taos_driver);

	
}

// driver exit
static void __exit tsl2771_exit(void) 
{
//	TSL2771_DBG("%s", __FUNCTION__);
//	platform_driver_unregister(&taos_driver);
	i2c_del_driver(&taos_driver);
    unregister_chrdev_region(taos_dev_number, TAOS_MAX_NUM_DEVICES);
    device_destroy(taos_class, MKDEV(MAJOR(taos_dev_number), 0));
    cdev_del(&taos_datap->cdev);
    class_destroy(taos_class);
    kfree(taos_datap);
}

MODULE_AUTHOR("Mingming Qiu - ZTE");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(tsl2771_init);
module_exit(tsl2771_exit);
