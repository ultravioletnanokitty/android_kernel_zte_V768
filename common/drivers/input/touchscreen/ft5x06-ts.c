/*******************************************************************************
  * Copyright 2011 Broadcom Corporation.  All rights reserved.
  *
  *   @file   drivers/input/touchscreen/ft5x06-ts.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <linux/ft5x06-ts.h>
#include <linux/slab.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#if defined(CONFIG_BOARD_ACAR)
#include <linux/gpio.h>
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
#endif
/* xuhuashan@gmail.com 2011.06.02 begin */
/* Reorganize the code for the TP driver. */
struct ft5x06_registers {
	u8 dev_mode;
	u8 gest_id;
	u8 td_status;
	struct {
		u8 touch_xh;
		u8 touch_xl;
		u8 touch_yh;
		u8 touch_yl;
		u8 dummy[2];
	} points[5];
};
/* xuhuashan@gmail.com 2011.06.02. end */
#define BOARD_VIRTUALKEY_FT5X06

struct ft5x06_chip_data {
	struct input_dev *input;
	struct i2c_client *client;
	struct delayed_work work;
	struct ft5x06_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int irq;
	/* xuhuashan@gmail.com 2011.06.01 begin */
	/* Fix the issue that TP cannot wakeup after suspended */
	spinlock_t lock;
	unsigned pen_down:1;
	unsigned irq_disabled:1;
	unsigned suspended:1;
	unsigned resuming:1;
	/* xuhuashan@gmail.com 2011.06.01 end */
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x06_early_suspend(struct early_suspend *h);;
static void ft5x06_late_resume(struct early_suspend *h);
#endif

static int ft5x06_calibrate(struct ft5x06_chip_data *chip)
{
	i2c_smbus_write_byte_data(chip->client, 0xa0, 0x00);
	printk(KERN_INFO "FocalTech 5x06 capcitive touchscreen auto calibration\n");

	return 0;
}

static int ft5x06_read_registers(struct ft5x06_chip_data *chip, 
				struct ft5x06_registers *regs)
{
	int ret = -EIO;

	ret = i2c_smbus_read_i2c_block_data(chip->client, 
			0x00, sizeof(*regs), (void *)regs);
	if(ret >= 0) {
/*kai.zhang59@gmail.com 2011.5.18 begin*/
/*for td_status*/
		regs->td_status = regs->td_status & 0x0F;
/*kai.zhang59@gmail.com 2011.5.18 end*/
	}

	return ret;
}

#define FT5X06_TOUCH_ID(r, i)									\
	(((r)->points[i].touch_xh >> 4) & 0x0F)
#define FT5X06_TOUCH_X(r, i)									\
	((((r)->points[i].touch_xh & 0x0F) << 8) | ((r)->points[i].touch_xl))
#define FT5X06_TOUCH_Y(r, i)									\
	((((r)->points[i].touch_yh & 0x0F) << 8) | ((r)->points[i].touch_yl))
//maxiaohui@wind-mobi.com 20120601 begin
//modify tp virtual upload way
//heweimao@wind-mobi.com 20120910 begin
//modify tp virtual_key coordinate
#if defined(BOARD_VIRTUALKEY_FT5X06)
int virtual_key[][5]=
{
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PASCAL)
{KEY_MENU,       60, 516,30,50},
{KEY_HOME,       130,516,30,50},
{KEY_BACK,       205,516,30,50},
{KEY_SEARCH,     275,516,30,50}
#else
{KEY_MENU,       30,  500,60,60},
{KEY_HOME,       110,500,60,60},
{KEY_BACK,       190,500,60,60},
{KEY_SEARCH,     270,500,60,60}
#endif
};
static int lastkey=0;
#endif
//maxiaohui@wind-mobi.com 20120601 end
//heweimao@wind-mobi.com 20120910 end

static void ft5x06_generate_event(struct ft5x06_chip_data *chip, 
				struct ft5x06_registers *regs)
{
	struct input_dev *input = chip->input;
	u16 rx = 0, ry = 0;
	int index;
	int nr_touch = regs->td_status;

        //printk("%s \n",__func__);
	if (nr_touch) {
		chip->pen_down = 1;

		for (index = 0; index < nr_touch; index++) {
			rx = FT5X06_TOUCH_X(regs, index);
			ry = FT5X06_TOUCH_Y(regs, index);

			//printk("x:%d y:%d state:%d\n", rx, ry, chip->pen_down);
//maxiaohui@wind-mobi.com 20120601 begin
//modify tp virtual upload way
#if defined(BOARD_VIRTUALKEY_FT5X06)
//maxiaohui@wind-mobi.com 20120601 begin
//heweimao@wind-mobi.com 201200910 begin
//modify tp virtual_key way
if(ry > 486 && ry < virtual_key[0][2]+virtual_key[0][3]/2)
{
        int retry;
	for(retry=0; retry<4 ;retry++)
	{
               if(rx > virtual_key[retry][1]-virtual_key[retry][4]/2 && rx < virtual_key[retry][1]+virtual_key[retry][4]/2)
                {
               //printk("valid key  %d \n",virtual_key[retry][0]);
		       lastkey = virtual_key[retry][0];
		       input_report_key(input, virtual_key[retry][0],1);
                }
	}
}
else
#endif
//maxiaohui@wind-mobi.com 20120601 end
//heweimao@wind-mobi.com 20120910 end
{
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, 200);
			input_report_abs(input, ABS_MT_POSITION_X, rx);
			input_report_abs(input, ABS_MT_POSITION_Y, ry);
			input_report_abs(input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(input);
}			
		}

		input_sync(input);
	}
	else {
		if (chip->pen_down) {	/* pen up */
//maxiaohui@wind-mobi.com 20120601 begin
//modify tp virtual upload way
#if defined(BOARD_VIRTUALKEY_FT5X06)			
			if (lastkey)
			{
//			        printk("lastkey %d \n ",lastkey);
				input_report_key(input, lastkey , 0);
			}
#endif
//maxiaohui@wind-mobi.com 20120601 end
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
			input_sync(input);
			chip->pen_down = 0;
		}
	}
}

#define to_delayed_work(_work) container_of(_work, struct delayed_work, work)

/* xuhuashan@gmail.com 2011.06.01 begin */
/* Fix the issue that TP cannot wakeup after suspended */
static void ft5x06_work(struct work_struct *work)
{
	struct ft5x06_chip_data *chip =
	    container_of(to_delayed_work(work), struct ft5x06_chip_data, work);
	struct ft5x06_registers regs;
	unsigned long flags;

	if(chip->suspended)
		return;

	/* delayed resuming is schedule */
	spin_lock_irqsave(&chip->lock, flags);
	if(chip->resuming) {
		if(chip->irq_disabled) {
			enable_irq(chip->irq);
			chip->irq_disabled = 0;
		}
		chip->resuming = 0;
		spin_unlock_irqrestore(&chip->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&chip->lock, flags);

	if(ft5x06_read_registers(chip, &regs) < 0)
		return;

	spin_lock_irqsave(&chip->lock, flags);
	/* analysis the register data and generate input event. */
	ft5x06_generate_event(chip, &regs);
	if (chip->pen_down) {
		schedule_delayed_work(&chip->work,
			      msecs_to_jiffies(20));
	}
	else {
		if(chip->irq_disabled) {
			enable_irq(chip->irq);
			chip->irq_disabled = 0;
		}
	}
	spin_unlock_irqrestore(&chip->lock, flags);
}

static irqreturn_t ft5x06_interrupt(int irq, void *dev_id)
{
	struct ft5x06_chip_data *chip = dev_id;

	pr_debug("%s\n", __func__);

	spin_lock(&chip->lock);
	if (!chip->pen_down && !chip->suspended) {
		disable_irq_nosync(chip->irq);
		chip->irq_disabled = 1;

		schedule_delayed_work(&chip->work,
				      msecs_to_jiffies(1));
	}
	spin_unlock(&chip->lock);

	return IRQ_HANDLED;
}
/* xuhuashan@gmail.com 2011.06.01 end */

static ssize_t ft5x06_calibrate_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct ft5x06_chip_data *chip =
	    (struct ft5x06_chip_data *)dev_get_drvdata(dev);

	ft5x06_calibrate(chip);

	return count;
}

/* xuhuashan@gmail.com 2011.06.13 begin */
/* Add chip identification and version attributes to sysfs */
static ssize_t ft5x06_chip_vendor_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_chip_data *chip = 
		(struct ft5x06_chip_data *)dev_get_drvdata(dev);
	int ret;
	int val;

	val = i2c_smbus_read_byte_data(chip->client, 0xa3);
	if(val > 0)
		ret = sprintf(buf, "%d\n", val);
	else
		ret = val;

	return ret;
}

static ssize_t ft5x06_firmware_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_chip_data *chip = 
		(struct ft5x06_chip_data *)dev_get_drvdata(dev);
	int ret;
	int val;

	val = i2c_smbus_read_byte_data(chip->client, 0xa6);
	if(val > 0)
		ret = sprintf(buf, "%d\n", val);
	else
		ret = val;

	return ret;
}

static ssize_t ft5x06_ctpm_vendor_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_chip_data *chip = 
		(struct ft5x06_chip_data *)dev_get_drvdata(dev);
	int ret;
	int val;

	val = i2c_smbus_read_byte_data(chip->client, 0xa8);
	if(val > 0)
		ret = sprintf(buf, "%d\n", val);
	else
		ret = val;

	return ret;
}

static DEVICE_ATTR(calibrate, 0644, NULL, ft5x06_calibrate_store);
static DEVICE_ATTR(chip_vendor, 0444, ft5x06_chip_vendor_show, NULL);
static DEVICE_ATTR(firmware_ver, 0444, ft5x06_firmware_ver_show, NULL);
static DEVICE_ATTR(ctpm_vendor, 0444, ft5x06_ctpm_vendor_show, NULL);
/* xuhuashan@gmail.com 2011.06.13 end */

/*lijie@wind-mobi.com 2011.5.17 begin add firmware upgrade feature*/

static char *updateBuf = NULL;
static size_t updateLen = 0;
static struct i2c_client *this_client;

#define    FTS_PACKET_LENGTH        128

static int ft5x06_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static void cmd_write(unsigned char btcmd,unsigned char btPara1,unsigned char btPara2,unsigned char btPara3,unsigned char num)
{
    unsigned char write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
	
	i2c_master_send(this_client, write_cmd, num);
}

static void byte_write(unsigned char* pbt_buf, unsigned int dw_len)
{	
	i2c_master_send(this_client, pbt_buf, dw_len);
}


static void byte_read(unsigned char* pbt_buf, unsigned int dw_len)
{
	i2c_master_recv(this_client, pbt_buf, dw_len);
}


static void ft5x06_write_reg(u8 addr, u8 para)
{
	u8 buf[2];

    buf[0] = addr;
    buf[1] = para;
    ft5x06_i2c_txdata(buf, 2);
}

static void ft5x06_fw_upgrade(unsigned char* pbt_buf, unsigned int dw_lenth)
{
	unsigned char reg_val[2] = {0};
	unsigned int i = 0;

	unsigned int  packet_number;
	unsigned int  j;
	unsigned int  temp;
	unsigned int  lenght;
	unsigned char  *packet_buf;
	unsigned char  *auc_i2c_write_buf;
	unsigned char bt_ecc;
	int 	 i_ret;

	packet_buf = kmalloc((FTS_PACKET_LENGTH+6), GFP_KERNEL);
	auc_i2c_write_buf = kmalloc(10, GFP_KERNEL);
	/*********Step 1:Reset	CTPM *****/
	/*write 0xaa to register 0xfc*/
	ft5x06_write_reg(0xfc,0xaa);
	msleep(50);
	 /*write 0x55 to register 0xfc*/
	ft5x06_write_reg(0xfc,0x55);
	printk("[TSP] Step 1: Reset CTPM test\n");
   
	msleep(30);   


	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do
	{
		i ++;
		i_ret = ft5x06_i2c_txdata(auc_i2c_write_buf, 2);
		msleep(5);
	}while(i_ret <= 0 && i < 5 );

	/*********Step 3:check READ-ID***********************/		  
	cmd_write(0x90,0x00,0x00,0x00,4);
	byte_read(reg_val,2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
		kfree(packet_buf);
		kfree(auc_i2c_write_buf);
		return;
	}

	 /*********Step 4:erase app*******************************/
	cmd_write(0x61,0x00,0x00,0x00,1);
   
	msleep(1500);
	printk("[TSP] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (unsigned char)(temp>>8);
		packet_buf[3] = (unsigned char)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (unsigned char)(lenght>>8);
		packet_buf[5] = (unsigned char)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}
		
		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
			  printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (unsigned char)(temp>>8);
		packet_buf[3] = (unsigned char)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (unsigned char)(temp>>8);
		packet_buf[5] = (unsigned char)temp;

		for (i=0;i<temp;i++)
		{
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);	  
		msleep(20);
	}

	//send the last six byte
	for (i = 0; i<6; i++)
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (unsigned char)(temp>>8);
		packet_buf[3] = (unsigned char)temp;
		temp =1;
		packet_buf[4] = (unsigned char)(temp>>8);
		packet_buf[5] = (unsigned char)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);  
		msleep(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
		kfree(packet_buf);
		kfree(auc_i2c_write_buf);
		return;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);

	kfree(packet_buf);
	kfree(auc_i2c_write_buf);

	return;
}

#define _FW_BUF_LEN_ 0x10000
static ssize_t update_getImage(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	if(!updateLen)
	{
		updateBuf = kmalloc(_FW_BUF_LEN_, GFP_KERNEL);
	}
	
	if(updateLen+count>_FW_BUF_LEN_)
	{
		updateLen = 0;
		kfree(updateBuf);
		updateBuf = NULL;
		printk("Invalid touch padder firmware image, update refused\r\n");
		return 0;
	}

	memcpy(updateBuf+updateLen, buf, count);

	updateLen += count;

	return count;
	
}

static ssize_t update_writeImage(struct device *dev, struct device_attribute *attr, char *buf)
{
	if((!updateLen) || (updateLen>_FW_BUF_LEN_))
	{		
		printk("Invalid touch padder firmware image, update refused\r\n");
		updateLen = 0;
		if(updateBuf)
		{
			kfree(updateBuf);
			updateBuf = NULL;
		}
		return 0;
	}

	ft5x06_fw_upgrade(updateBuf, updateLen);

	updateLen = 0;
	kfree(updateBuf);
	updateBuf = NULL;

	return 0;
}

static DEVICE_ATTR(update_fw, 0660, update_writeImage, update_getImage);
/*lijie@wind-mobi.com 2011.5.17 end*/

static struct attribute *ft5x06_attributes[] = {
	&dev_attr_calibrate.attr,

	/*lijie@wind-mobi.com 2011-05-20 begin, add a sys node for upgrade purpose, 
	its store action to get firmware image, its show action to upgrade firmware image*/	
	&dev_attr_update_fw.attr,	
	/*lijie@wind-mobi.com 2011-05-20 end*/

#ifdef DEBUG_FT5X06
	&dev_attr_state_trigger.attr,
#endif
	/* xuhuashan@gmail.com 2011.06.13 begin */
	/* Add chip identification and version attributes to sysfs */
	&dev_attr_chip_vendor.attr,
	&dev_attr_firmware_ver.attr,
	&dev_attr_ctpm_vendor.attr,
	/* xuhuashan@gmail.com 2011.06.13 end */

	NULL,
};

static struct attribute_group ft5x06_attr_group = {
	.attrs = ft5x06_attributes,
};

/* heweimao@wind-mobi.com 2012.09.11 begin */
static int read_tpid(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int ret;

    ret = sprintf(buf, "%s", "ft5x06");

    return ret;
}
/* heweimao@wind-mobi.com 2012.09.11end */

/* heweimao@wind-mobi.com 2012.09.11 begin */
static struct proc_dir_entry *TP_pdir;
static struct proc_dir_entry *TPid_point;
/* heweimao@wind-mobi.com 2012.09.11 end */


//yuanlan@wind-mobi.com 2011-07-22 begin
//check  if the chip is  GT818 
static int verify_FT5X06( struct ft5x06_chip_data *chip)
{
	//uint8_t version_data[5]={0};	//store touchscreen version infomation

	//uint8_t data[4] = {0,0,0,0};
   
	
	/*check if i2c with the dedicated adress could return value*/
	//uint8_t tmp ;
	uint8_t i = 0 ;
	for (i=0;i<2;i++)
	{
     i2c_smbus_read_byte_data(chip->client,0x3a);
	}
	
	if(i2c_smbus_read_byte_data(chip->client,0x3a)<0)
	{
		printk("Could not get FT5X06 i2c register content\r\n");
		return -1;
	}

	
	return 0;
}
//yuanlan@wind-mobi.com 2011-07-22 end

#if 0//defined(BOARD_VIRTUALKEY_FT5X06)
static ssize_t acar_virtual_keys_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
//maxiaohui@wind-mobi.com 2012.05.02 begin
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PASCAL)
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":50:500:60:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	 ":130:500:60:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":210:500:60:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":290:500:60:60"
	   "\n");
#else
//maxiaohui@wind-mobi.com 2012.05.02 end
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":30:500:60:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	 ":110:500:60:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":190:500:60:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":270:500:60:60"
	   "\n");
#endif

}



static struct kobj_attribute acar_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x06-ts",
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


static int ft5x06_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = -EIO;
	struct ft5x06_chip_data *chip = NULL;
	struct ft5x06_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;


	/* xuhuashan@gmail.com 2011.05.27 begin */
	/* Wake up the chip on initialize the driver, 
	 * make sure the TP is avaiable when system up. */
//	if(pdata->wakeup_chip)
//		pdata->wakeup_chip();
	/* xuhuashan@gmail.com 2011.05.27 end */

#if 0
	ret = i2c_smbus_write_byte_data(client, 0, 0);	/* set normal mode */
	if (ret < 0) {
		printk(KERN_ERR "FT5x06 touchscreen set mode failed %d\n", ret);
		goto err;
	}
#endif

	/*lijie@wind-mobi.com 2011-05-20 begin, this variable is needed by tp firmware upgrade feature*/
	this_client = client;
	/*lijie@wind-mobi.com 2011-05-20 end*/

	if(!pdata) {
		dev_err(&client->dev,
			"No platform data for ft5x06!");
		return -EINVAL;
	}
#if defined(CONFIG_BOARD_ACAR)
	 gpio_request(29, "ft5x06-ts");
	 gpio_direction_input(29);
	 bcm_gpio_pull_up_down_enable(29, true);
	 bcm_gpio_pull_up(29, true);
	// set_irq_type(GPIO_TO_IRQ(29), IRQF_TRIGGER_FALLING);
#else
	if(pdata->init_platform_hw) {
		ret = pdata->init_platform_hw();
		if(ret) {
			dev_err(&client->dev, 
				"Failed to init ft5x06 hardware!");
			return ret;
		}
	};
#endif	

      //yuanlan@wind-mobi.com begin 2011.09.01 
        /* xuhuashan@gmail.com 2011.05.27 begin */
        /* Wake up the chip on initialize the driver, 
         * make sure the TP is avaiable when system up. */
/* xiongbiao@gmail.com 2012.02.23 begin */
/* close it,fix ft5206 can't wake up  */     
       // if(pdata->wakeup_chip)
       //         pdata->wakeup_chip();
/* xiongbiao@gmail.com 2012.02.23 end */    
        /* xuhuashan@gmail.com 2011.05.27 end */
      //yuanlan@wind-mobi.com end 2011.09.01
	/* Malloc ft5x06 context */
	chip = kzalloc(sizeof(struct ft5x06_chip_data), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err_kfree;
	}
	chip->pdata = pdata;
	/*yuanlan@wind-mobi.com on 2011-07-22 begin */
	//check if Touch panel is FT5X06	
	chip->client = client;
  //      printk(KERN_INFO "#####################yuanlan_fota_image_update_success\n");	
	if(verify_FT5X06(chip)) 
	{
		printk("FT5X06 TP  probe failed\r\n");
		ret = -ENOENT;
		goto err_kfree;
	}
			
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


	//yuanlan@wind-mobi.com on 2011-07-22 end



	/* register input device */
	input_dev = input_allocate_device();
	input_dev->name = "ft5x06-ts";

	INIT_DELAYED_WORK(&chip->work, ft5x06_work);
	chip->input = input_dev;

	spin_lock_init(&chip->lock);

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
	    BIT_MASK(EV_ABS);
    input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

//maxiaohui@wind-mobi.com 20120601 begin
//modify tp virtual upload way
#if defined(BOARD_VIRTUALKEY_FT5X06)
	{
		int i = 0;
		for (i=0;i<sizeof(virtual_key)/sizeof(virtual_key[0][0]);i++)
		{
			__set_bit(virtual_key[i][0], input_dev->keybit);
		}
	}
#else
	/* register virtualkeys capability if support */
	if(pdata->virtual_keys) {
		int i = 0;
		while(pdata->virtual_keys[i] > 0) {
			__set_bit(pdata->virtual_keys[i++], input_dev->keybit);
		}
	}
#endif
	input_set_abs_params(chip->input, ABS_MT_POSITION_X, 
						pdata->scr_x_min, pdata->scr_x_max, 0, 0);
	input_set_abs_params(chip->input, ABS_MT_POSITION_Y, 
						pdata->scr_y_min, pdata->scr_y_max, 0, 0);
	input_set_abs_params(chip->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(chip->input, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	platform_set_drvdata(client, chip);
	input_set_drvdata(input_dev, chip);
	ret = input_register_device(input_dev);
	if(ret) {
		dev_err(&client->dev,
			"Unable to register input device.\n");
		goto err_kfree;
	}

	chip->client = client;
	chip->pen_down = 0;
	chip->irq_disabled = 0;
	chip->suspended = 0;
	chip->resuming = 0;

	/* setup IRQ */
	if (client->irq < 0) {
		dev_err(&client->dev,
			"No irq allocated in client resources!\n");
		goto err_kfree;
	}

	chip->irq = client->irq;

	ret = request_irq(chip->irq, ft5x06_interrupt,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ft5x06-ts", chip);
	if(ret < 0) {
		dev_err(&client->dev,
			"Failed to request irq %d!\n", chip->irq);
		goto err_unregister_input;
	}

	ret = sysfs_create_group(&client->dev.kobj, &ft5x06_attr_group);
	if (ret < 0) {
		dev_err(&client->dev,
			"Create sysfs file failed %d\n", ret);
		goto err_free_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	chip->early_suspend.suspend = ft5x06_early_suspend;
	chip->early_suspend.resume = ft5x06_late_resume;
	register_early_suspend(&chip->early_suspend);
#endif
#if 0
//kobject begin
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
//kobject end	
#endif
	printk(KERN_INFO "FocalTech 5x06 capcitive touchscreen device detected\n");
	return 0;

err_free_irq:
	free_irq(chip->irq, chip);
err_unregister_input:
	input_unregister_device(chip->input);
err_kfree:
	kfree(chip);
#if defined(CONFIG_BOARD_ACAR)
	gpio_direction_input(29);
	gpio_free(29);
#else
	if (pdata->exit_platform_hw)
	{
		pdata->exit_platform_hw();
	}
#endif
	return ret;
}

/* xuhuashan@gmail.com 2011.05.12 begin */
/* 1. Fix the compiler issue which cannot dereferencing pdata */
/* 2. Fix the sysfs_remove_group parameter mistake. */
static int ft5x06_remove(struct i2c_client *client)
{
	struct ft5x06_chip_data *chip =
	    (struct ft5x06_chip_data *)platform_get_drvdata(client);
	struct ft5x06_platform_data *pdata = chip->pdata;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&chip->early_suspend);
#endif
	sysfs_remove_group(&client->dev.kobj, &ft5x06_attr_group);
	free_irq(chip->irq, chip);
    input_unregister_device(chip->input);
    kfree(chip);

#if defined(CONFIG_BOARD_ACAR)
	gpio_direction_input(29);
	gpio_free(29);
#else
	if (pdata->exit_platform_hw)
	{
		pdata->exit_platform_hw();
	}
#endif


	return 0;
}
/* xuhuashan@gmail.com 2011.05.12 end */

/* xuhuashan@gmail.com 2011.06.01 begin */
/* Fix the issue that TP cannot wakeup after suspended */
static int ft5x06_resume(struct i2c_client *client)
{
	struct ft5x06_chip_data *chip =
	    (struct ft5x06_chip_data *)platform_get_drvdata(client);
	unsigned long flags;

	if(chip->pdata->wakeup_chip)
		chip->pdata->wakeup_chip();

	spin_lock_irqsave(&chip->lock, flags);

	chip->suspended = 0;
	chip->resuming = 1;
	schedule_delayed_work(&chip->work, msecs_to_jiffies(200));

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int ft5x06_suspend(struct i2c_client *client, pm_message_t state)
{
	struct ft5x06_chip_data *chip =
	    (struct ft5x06_chip_data *)platform_get_drvdata(client);
	unsigned long flags;

	cancel_work_sync(&chip->work.work);

	spin_lock_irqsave(&chip->lock, flags);

	chip->suspended = 1;
	if(!chip->irq_disabled) {
		disable_irq_nosync(chip->irq);
		chip->irq_disabled = 1;
	}
	else {
		chip->pen_down = 0;
	}

	spin_unlock_irqrestore(&chip->lock, flags);
// xiongbiao@wind-mobi.com 2011.12.14 begin
// fix ftx06 TP don't  wake up in L300
	//i2c_smbus_write_byte_data(chip->client, 0xa5, 0x03);
// xiongbiao@wind-mobi.com 2011.12.14 end
//songjinguo@wind-mobi.com 2011.01.17 begin
//modify for TP can't suspend issue;
//review by liubing
	i2c_smbus_write_byte_data(chip->client, 0xa5, 0x03);
//songjinguo@wind-mobi.com 2011.01.17 end
	if(chip->pdata->powerdown_chip)
		chip->pdata->powerdown_chip();

	return 0;
}
/* xuhuashan@gmail.com 2011.06.01 end */

#ifdef CONFIG_HAS_EARLYSUSPEND
/* xuhuashan@gmail.com 2011.05.17 begin */
/* Add power management support for FT5x06 driver */
static void ft5x06_early_suspend(struct early_suspend *h)
{
	struct ft5x06_chip_data *chip
		= container_of(h, struct ft5x06_chip_data, early_suspend);

	ft5x06_suspend(chip->client, PMSG_SUSPEND);
}

static void ft5x06_late_resume(struct early_suspend *h)
{
	struct ft5x06_chip_data *chip
		= container_of(h, struct ft5x06_chip_data, early_suspend);

	ft5x06_resume(chip->client);
}
/* xuhuashan@gmail.com 2011.05.17 end */
#endif
//add-s by hanwei fix#550808 20120726
void tp_homekey_control(int press)
{
		//printk(" -------- press %d \n",press );
        struct input_dev *input;
        struct ft5x06_chip_data *chip =
            (struct ft5x06_chip_data *)platform_get_drvdata(this_client);
        input = chip->input;
        input_report_key(input, KEY_HOME,press);
        input_sync(input);
}
//add-e by hanwei fix#550808 20120726
static const struct i2c_device_id ft5x06_id[] = {
	{ "ft5206", 0 },
	{ "ft5306", 1 },
	{ "ft5406", 2 }
};

static struct i2c_driver ft5x06_ts_driver = {
	.driver = {
        .name = "ft5x06-ts",
    },
	.id_table     = ft5x06_id,
	.probe        = ft5x06_probe,
	.remove       = ft5x06_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.resume       = ft5x06_resume,
	.suspend      = ft5x06_suspend,
#endif
};

static int __init ft5x06_init(void)
{
	i2c_add_driver(&ft5x06_ts_driver);
	return 0;
}

static void __exit ft5x06_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}

module_init(ft5x06_init);
module_exit(ft5x06_exit);

MODULE_AUTHOR("Kenny Gong <wuchun@broadcom.com>");
MODULE_DESCRIPTION("FocalTech 5x06 capcitive touch screen driver");
MODULE_LICENSE("GPL");
