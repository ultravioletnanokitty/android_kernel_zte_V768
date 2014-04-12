/* drivers/input/touchscreen/silabs_f760.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
//#include <linux/melfas_ts.h>

#include <linux/firmware.h>

static struct workqueue_struct *silabs_wq;

#define HEX_HW_VER	0x01
#define HEX_SW_VER	0x05	//change the version while Firmware Update 
#define FW_VERSION			    0x03

#define MAX_X	240 
#define MAX_Y	320
#define TSP_INT 30

#define F760_MAX_TOUCH		2
#define ESCAPE_ADDR 	    0xAA
#define TS_READ_START_ADDR 	    0x10
#define TS_READ_VERSION_ADDR	0x1F
#define TS_READ_ESD_ADDR	0x1E
#define TS_READ_REGS_LEN 		13
#define SILABS_MAX_TOUCH		F760_MAX_TOUCH
#define MTSI_VERSION		    0x05

//#define __TOUCH_DEBUG__ 1

#define I2C_RETRY_CNT			10

#define TOUCH_ON 1
#define TOUCH_OFF 0

#define PRESS_KEY				1
#define RELEASE_KEY				0

#define SET_DOWNLOAD_BY_GPIO	1

#define SILABS_TS_NAME "silabs-f760"

#define YTE_MODULE_VER   0x02
#define SMAC_MODULE_VER   0x03
#define FW_VER  0x00

static int prev_wdog_val = -1;
static int check_ic_counter = 3;

static struct workqueue_struct *silabs_wq;
static struct workqueue_struct *check_ic_wq;

static struct regulator *touch_regulator=NULL;
//add by brcm
static DEFINE_SPINLOCK(silabs_spin_lock);

int touch_id[2], posX[2], posY[2], strength[2];

static int firmware_ret_val = -1;
int tsp_irq;
int TSP_MODULE_ID;
EXPORT_SYMBOL(TSP_MODULE_ID);

int firm_update( void );

#if SET_DOWNLOAD_BY_GPIO
#include "bootloader.h"
#endif // SET_DOWNLOAD_BY_GPIO

enum
{
	TOUCH_SCREEN=0,
	TOUCH_KEY
};

struct muti_touch_info
{
	int strength;
	int width;	
	int posX;
	int posY;
};

struct silabs_ts_data {
	uint16_t addr;
	struct i2c_client *client; 
	struct input_dev *input_dev;
    	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
    	struct work_struct  work_timer;
	struct early_suspend early_suspend;
};

struct silabs_ts_data *ts_global;

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(firmware	, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret	, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, firmware_ret_show, firmware_ret_store);
/* sys fs */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void silabs_ts_early_suspend(struct early_suspend *h);
static void silabs_ts_late_resume(struct early_suspend *h);
#endif

extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);

static struct muti_touch_info g_Mtouch_info[SILABS_MAX_TOUCH];


void touch_ctrl_regulator(int on_off)
{
	if(on_off==TOUCH_ON)
	{
			regulator_set_voltage(touch_regulator,2900000,2900000);
			regulator_enable(touch_regulator);
	}
	else
	{
			regulator_disable(touch_regulator);
	}
}
EXPORT_SYMBOL(touch_ctrl_regulator);

int tsp_reset( void )
{
	int ret=1, key = 0;

	printk("[TSP] %s+\n", __func__ );

      
	if (ts_global->use_irq)
	{
		disable_irq(ts_global->use_irq);
	}

	touch_ctrl_regulator(0);

	gpio_direction_output(30, 0);
      	gpio_direction_output(27, 0);
	gpio_direction_output(26, 0);
            
	msleep(200);

#if 0
	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = RELEASE_KEY;
#endif
	if (ts_global->use_irq)
	{
		disable_irq(ts_global->use_irq);
	}

	gpio_direction_output(30, 1);
      	gpio_direction_output(27, 1);
	gpio_direction_output(26, 1);

	gpio_direction_input(30);
      	gpio_direction_input(27);
	gpio_direction_input(26);

	touch_ctrl_regulator(1);
		
	msleep(200);

	enable_irq(ts_global->use_irq);

	return ret;
}


int tsp_i2c_write (unsigned char *rbuf, int num)
{
    int ret;
    ret = i2c_master_send(ts_global->client, rbuf, num);
   // printk("[TSP] tsp_i2c_write : %x\n", ts_global->client->addr);
    
       if(ret<0) {
		printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
	}

    return ret;
}
EXPORT_SYMBOL(tsp_i2c_write);

int tsp_i2c_read(unsigned char *rbuf, int len)
{
    int ret;
    
	ret = i2c_master_recv(ts_global->client, rbuf, len);

       if(ret<0) {
		printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
	}

       return ret;
}
EXPORT_SYMBOL(tsp_i2c_read);


void TSP_forced_release_forkey(struct silabs_ts_data *ts)
{
	int i;
	int temp_value=0;

	for(i=0; i<SILABS_MAX_TOUCH; i++)
		{
			if(g_Mtouch_info[i].strength== -1)
				continue;

			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		      input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		      input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0 );
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);      				
			input_mt_sync(ts->input_dev);   

                   printk("[TSP] force release\n");
 
			if(g_Mtouch_info[i].strength == 0)
				g_Mtouch_info[i].strength = -1;

            		temp_value++;
		}

	if(temp_value>0)
		input_sync(ts->input_dev);

	
}
EXPORT_SYMBOL(TSP_forced_release_forkey);

static void silabs_ts_work_func(struct work_struct *work)
{
	int ret = 0, i;
	uint8_t buf_temp[TS_READ_REGS_LEN];
	uint8_t buf[TS_READ_REGS_LEN];
	int touch_num=0, button_num =0, touchID=0, posX_value=0, posY_value=0, width = 0, reportID = 0, button_status=0, button_check=0;
       int keyID = 0;
       unsigned long    flags;

	struct silabs_ts_data *ts = container_of(work, struct silabs_ts_data, work);

      #ifdef __TOUCH_DEBUG__
	printk("[TSP] %s, %d\n", __func__, __LINE__ );
       #endif

	if(ts ==NULL)
       printk("[TSP] silabs_ts_work_func : TS NULL\n");
    
	buf[0] = ESCAPE_ADDR;
	buf[1] = 0x02;
	
	for(i=0; i<I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, buf, 2);

		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);

			if(ret >=0)
			{
				break; // i2c success
			}
		}
	}

        spin_lock_irqsave(&silabs_spin_lock, flags);
        
	if (ret < 0)
	{
             printk("[TSP] silabs_ts_work_func: i2c failed\n" );
             enable_irq(ts->client->irq);
             spin_unlock_irqrestore(&silabs_spin_lock, flags);
		return ;	
	}
	else 
	{

            touch_num  = buf[0]&0x0F;
	      button_num = ((buf[0]&0xC0)>>6);
            button_status=((buf[1]&0x10)>>4);
            button_check=buf[1]&0x0F;

            #ifdef __TOUCH_DEBUG__
            printk("[TSP] button_num : %d, touch_num : %d, button_check:%d, buf[1] : %d\n", button_num, touch_num, button_check, buf[1]);
            #endif
        
        	if(button_check == 0)
		{
                   if(touch_num >0) 
                   {
                   touch_id[0] = (buf[2]&0xf0)>>4;
        		//posX[0] = (240-(( buf[3]<< (8) ) +  buf[4]));
        		posX[0] = (( buf[3]<< (8) ) +  buf[4]);
        		posY[0] = ( buf[5]<< (8) ) +  buf[6];

        		strength[0] = buf[7]; 

                   touch_id[1] = (buf[2]&0x0f);
        		//posX[1] =  240-(( buf[8]<< (8) ) +  buf[9]);
        		posX[1] =  (( buf[8]<< (8) ) +  buf[9]);
        		posY[1] = ( buf[10]<< (8) ) +  buf[11];

        		strength[1] = buf[12]; 
                   }
                   
                    if(touch_num==0)
                    {
                        touch_id[0]=0;
                        touch_id[1]=0;
                        strength[0]=0;
                        strength[1]=0;
                    }
                    #if 0         
			g_Mtouch_info[touchID].posX= posX;
			g_Mtouch_info[touchID].posY= posY;
			g_Mtouch_info[touchID].width= width;			

			if(touchState)
				g_Mtouch_info[touchID].strength= strength;
			else
				g_Mtouch_info[touchID].strength = 0;
                    #endif
			for(i=0; i<2; i++)
			{
				//if(strength[i]== 0)
				//	continue;

				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_id[i]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, posX[i]);
			      input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  posY[i]);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, touch_id[i] );
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, strength[i]);      				
				input_mt_sync(ts->input_dev);   

            #ifdef __TOUCH_DEBUG__
             printk("[TSP] i : %d, x: %d, y: %d\n, 3:%d, 4:%d", touch_id[i], posX[i], posY[i], strength[i], strength[i]);
		#endif			
			}
		}
            	else
		{
                   
			if (buf[1] & 0x1)
				input_report_key(ts->input_dev, KEY_MENU, button_status ? PRESS_KEY : RELEASE_KEY);		
			if (buf[1] & 0x2)
				input_report_key(ts->input_dev, KEY_BACK, button_status ? PRESS_KEY : RELEASE_KEY);
			if (buf[1] & 0x4)
				input_report_key(ts->input_dev, KEY_BACK, button_status ? PRESS_KEY : RELEASE_KEY);
			if (buf[1] & 0x8)
				input_report_key(ts->input_dev, KEY_SEARCH, button_status ? PRESS_KEY : RELEASE_KEY);			
            #ifdef __TOUCH_DEBUG__
		printk(KERN_ERR "melfas_ts_work_func: buf[1] : %d, button_status: %d\n", buf[1], button_status ? PRESS_KEY : RELEASE_KEY);
		#endif		
		}
   	input_sync(ts->input_dev);              
	}

	enable_irq(ts->client->irq);
	spin_unlock_irqrestore(&silabs_spin_lock, flags);
}

static irqreturn_t silabs_ts_irq_handler(int irq, void *dev_id)
{
	struct silabs_ts_data *ts = dev_id;

       #ifdef __TOUCH_DEBUG__
	printk("[TSP] %s, %d\n", __func__, __LINE__ );
       #endif
       
	disable_irq_nosync(ts->client->irq);
	queue_work(silabs_wq, &ts->work);

	return IRQ_HANDLED;
}

static void check_ic_work_func(struct work_struct *work_timer)
{
	int ret=0;
      uint8_t buf_esd[2];
	uint8_t i2c_addr = 0x1F;
	uint8_t wdog_val[1];

	struct silabs_ts_data *ts = container_of(work_timer, struct silabs_ts_data, work_timer);

	//printk("[TSP] %s, %d\n", __func__, __LINE__ );

    	buf_esd[0] = ESCAPE_ADDR;
	buf_esd[1] = TS_READ_ESD_ADDR;

	wdog_val[0] = 1;
 
    
	if(check_ic_counter == 0)
	{
           ret = i2c_master_send(ts->client, &buf_esd, 2);

		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, wdog_val, 1);
		}
        
	    if(ret < 0)
	    {
                 tsp_reset();
		    printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
	    }

		else if(wdog_val[0] == (uint8_t)prev_wdog_val)
		{
			printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
			tsp_reset();
			prev_wdog_val = -1;
		}
		else
		{
//			printk("[TSP] %s counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
			prev_wdog_val = wdog_val[0];
		}
		
		check_ic_counter = 3;	
	}
	else
	{
		check_ic_counter--;
	}
}

static enum hrtimer_restart silabs_watchdog_timer_func(struct hrtimer *timer)
{
    	//printk("[TSP]\n");
	queue_work(check_ic_wq, &ts_global->work_timer);
	hrtimer_start(&ts_global->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static unsigned int touch_present = 0;

static int silabs_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct silabs_ts_data *ts;
	int ret = 0; 
 	uint8_t buf[TS_READ_REGS_LEN];   
       uint8_t buf_firmware[3];
       int i=0;

	touch_ctrl_regulator(TOUCH_ON);
       mdelay(200);  
	touch_ctrl_regulator(TOUCH_OFF);
       mdelay(200);      
	touch_ctrl_regulator(TOUCH_ON);
       mdelay(200);

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, silabs_ts_work_func);
    	INIT_WORK(&ts->work_timer, check_ic_work_func );
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

      tsp_irq=client->irq;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = silabs_watchdog_timer_func;

	/* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);

	/* sys fs */

#if 1//SET_DOWNLOAD_BY_GPIO
	buf_firmware[0] = ESCAPE_ADDR;
	buf_firmware[1] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buf_firmware, 2);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
	}

	ret = i2c_master_recv(ts->client, &buf_firmware, 3);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);			
	}
	printk("[TSP] silabs_ts_probe %d, %d, %d\n", buf_firmware[0], buf_firmware[1], buf_firmware[2]);
     
    if ((( buf_firmware[2] == YTE_MODULE_VER)||( buf_firmware[2] == SMAC_MODULE_VER))&&(buf_firmware[0] == FW_VER))
    { 
            TSP_MODULE_ID =  buf_firmware[2];
	       local_irq_disable();
		ret = Firmware_Download();	
        	printk("[TSP] enable_irq : %d\n", __LINE__ );
	      local_irq_enable();

		if(ret == 0)
		{
			printk(KERN_ERR "SET Download Fail - error code [%d]\n", ret);			
		}
	}	

 #endif // SET_DOWNLOAD_BY_GPIO

      ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
      		touch_present = 0;
		printk(KERN_ERR "silabs_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "sec_touchscreen ";
    
	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	
	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);		
	ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);	

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS,  ts->input_dev->evbit);
	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	set_bit(EV_SYN, ts->input_dev->evbit); 
	set_bit(EV_KEY, ts->input_dev->evbit);	
     
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
        	touch_present = 0;
		printk(KERN_ERR "silabs_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    	printk("[TSP] %s, irq=%d\n", __func__, client->irq );

    	gpio_request(TSP_INT, "ts_irq");
	gpio_direction_input(TSP_INT);
	bcm_gpio_pull_up(TSP_INT, true);
	bcm_gpio_pull_up_down_enable(TSP_INT, true);
	set_irq_type(GPIO_TO_IRQ(TSP_INT), IRQF_TRIGGER_FALLING);
    

    if (client->irq) {
		ret = request_irq(client->irq, silabs_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);

		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

#if 1
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = silabs_ts_early_suspend;
	ts->early_suspend.resume = silabs_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#endif

	if(ts ==NULL)
       printk("[TSP] silabs_ts_init_read : TS NULL\n");

	touch_present = 1;

       hrtimer_start(&ts->timer, ktime_set(5, 0), HRTIMER_MODE_REL);

	return 0;

err_input_register_device_failed:
	printk(KERN_ERR "silabs-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "silabs-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "silabs-ts: err_alloc_data failed_\n");	
	return ret;
    }

static int silabs_ts_remove(struct i2c_client *client)
{
	struct silabs_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	//else
	//	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int silabs_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    	int ret;
	struct silabs_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	if( touch_present )
	{
	if (ts->use_irq)
	{
		disable_irq(client->irq);
	}
          ret = cancel_work_sync(&ts->work_timer);

	    ret = cancel_work_sync(&ts->work);
        	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	    {
		    enable_irq(client->irq);
	    }

	    hrtimer_cancel(&ts->timer);

	    touch_ctrl_regulator(TOUCH_OFF);

          gpio_direction_output(30, 0);
      	    gpio_direction_output(27, 0);
	    gpio_direction_output(26, 0);

	    msleep(400);    
	}
    	else
		printk("[TSP] TSP isn't present.\n", __func__ );

        TSP_forced_release_forkey(ts);

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

static int silabs_ts_resume(struct i2c_client *client)
{
	int ret, key, retry_count;
	struct silabs_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

      //enable_irq(client->irq); // scl wave

	printk("[TSP] %s+\n", __func__ );
	if( touch_present )
	{

        
	gpio_direction_output(30, 1);
      	gpio_direction_output(27, 1);
	gpio_direction_output(26, 1);

	gpio_direction_input(30);
      	gpio_direction_input(27);
	gpio_direction_input(26);

      touch_ctrl_regulator(TOUCH_ON);
      msleep(40);

    #if 0
	// for TSK
	for(key = 0; key < MAX_KEYS; key++)
		touchkey_status[key] = RELEASE_KEY;

	fingerInfo[0].status = -1;
        fingerInfo[1].status = -1;
        fingerInfo[2].id = 0;
  
	retry_count = 0;
    #endif
    
	prev_wdog_val = -1;

    #if 0
	if(tsp_charger_type_status == 1)
	{
		set_tsp_for_ta_detect(tsp_charger_type_status);
	}
	
	if( tsp_proximity_irq_status == 1)
	{
		set_tsp_for_prox_enable(tsp_proximity_irq_status);
	}
    #endif

	hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	enable_irq(client->irq);
	}
	else
		printk("[TSP] TSP isn't present.\n", __func__ );

	printk("[TSP] %s-\n", __func__ );
 
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void silabs_ts_early_suspend(struct early_suspend *h)
{
	struct silabs_ts_data *ts;
	ts = container_of(h, struct silabs_ts_data, early_suspend);
	silabs_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void silabs_ts_late_resume(struct early_suspend *h)
{
	struct silabs_ts_data *ts;
	ts = container_of(h, struct silabs_ts_data, early_suspend);
	silabs_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id silabs_ts_id[] = {
	{ SILABS_TS_NAME, 0 },
	{ }
};

static struct i2c_driver silabs_ts_driver = {
	.probe		= silabs_ts_probe,
	.remove		= silabs_ts_remove,
#if 1
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= silabs_ts_suspend,
	.resume		= silabs_ts_resume,
#endif
#endif
	.id_table	= silabs_ts_id,
	.driver = {
		.name	= SILABS_TS_NAME,
	},
};

static int __devinit silabs_ts_init(void)
{

	silabs_wq = create_workqueue("silabs_wq");
	if (!silabs_wq)
		return -ENOMEM;
    	check_ic_wq = create_singlethread_workqueue("check_ic_wq");	
	if (!check_ic_wq)
		return -ENOMEM;

	touch_regulator = regulator_get(NULL,"touch_vcc");

	return i2c_add_driver(&silabs_ts_driver);
}

static void __exit silabs_ts_exit(void)
{
	if (touch_regulator) 
	{
       	 regulator_put(touch_regulator);
		 touch_regulator = NULL;
    	}
	
	i2c_del_driver(&silabs_ts_driver);
	if (silabs_wq)
		destroy_workqueue(silabs_wq);

	if (check_ic_wq)
		destroy_workqueue(check_ic_wq);
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
      	uint8_t buf_firmware;
       int ret;
       
	printk("[TSP] %s\n",__func__);

	buf_firmware = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts_global->client, &buf_firmware, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
	}

	ret = i2c_master_recv(ts_global->client, &buf_firmware, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);			
	}
	printk("[TSP] ver firmware=[%d]\n", buf_firmware);

    	sprintf(buf, "%d\n", buf_firmware);

	return sprintf(buf, "%s", buf );
}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	firmware_ret_val = -1;
	printk("[TSP] firmware_store  valuie : %d\n",value);
	if ( value == 0 )
	{
		printk("[TSP] Firmware update start!!\n" );

		firm_update( );
		return size;
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk("[TSP] %s!\n", __func__);

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk("[TSP] %s, operate nothing!\n", __func__);

	return size;
}


int firm_update( void )
{
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	disable_irq(tsp_irq);
	local_irq_disable();

	firmware_ret_val = Firmware_Download();	

	msleep(1000);
	if( firmware_ret_val )
		printk(KERN_INFO "[TSP] %s success, %d\n", __func__, __LINE__);
	else	
		printk(KERN_INFO "[TSP] %s fail, %d\n", __func__, __LINE__);

	local_irq_enable();

	enable_irq(tsp_irq);

	return 0;
} 

module_init(silabs_ts_init);
module_exit(silabs_ts_exit);

MODULE_DESCRIPTION("silabs Touchscreen Driver");
MODULE_LICENSE("GPL");
