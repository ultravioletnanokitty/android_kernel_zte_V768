/*
 *  tmd2771x.c - Texas Advanced Optoelectronic Solutions Inc.
 *              Proximity/Ambient light sensor
 *
 *  Copyright (C) 2010 Samsung Electronics
 *  Donggeun Kim <dg77....@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/hwmon.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <plat/bcm_i2c.h>
#include <linux/tmd2771x.h>
#include "../iio.h"
#include "light.h"
#include <linux/input.h>
struct tmd2771x_chip {
       struct i2c_client               *client;
       struct iio_dev                  *indio_dev;
       struct work_struct              work_thresh;
       s64                             last_timestamp;
       struct mutex                    lock;
/* xuhuashan@gmail.com 2011.06.17 begin */
/* Add PS calibrate function */
       struct kobject                  cali_kobj;
       struct completion               cali_completion;
       u8                              cali_mode;
       u32                             cali_min_val;
       u32                             cali_max_val;
       u32                             cali_avg_val;
       u32                             cali_cur_val;
/* xuhuashan@gmail.com 2011.06.17 end */
       struct device *ls_dev;
	struct device *ps_dev;
	uint16_t  ps_upper_th;
	uint16_t  ps_lower_th;
	unsigned int ps_old_val;
	unsigned int ps_exec_cnt;
	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
       struct tmd2771x_platform_data   *pdata;
};


static int tsl_calc_proximity_threshold(uint16_t val, 
				uint16_t *hi, uint16_t *lo)
{
//liutao@wind-mobi.com 2012-3-15 begin
//improve proximity sensor calibrate 
//yuanlan@wind-mobi.com 2012-3-15 review
	if(!hi || !lo || !val)
		return -1;
//feixiaoping@wind-mobi.com 2012-4-19 begin
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

//feixiaoping@wind-mobi.com 2012-4-19 end

	//liutao@wind-mobi.com 2012-4-18 begin
	//screen disply slowly or can't wake up
	//yuanlan@wind-mobi.com 2012-4-18 review
    
	//liutao@wind-mobi.com 2012-4-18 begin
	//printk("wilson printk *hi=%d,*lo =%d,val = %d\n",*hi,*lo,val);//feixiaoping@wind-mobi.com  delete print message
	//liutao@wind-mobi.com 2012-4-18 end
	return 0;

}
//liutao@wind-mobi.com 2012-3-15 end
static int psensor_setup(struct tmd2771x_chip *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[TMD2771X error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[TMD2771X error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

/*ret = misc_register(&psensor_misc);
     if (ret < 0) {
	  pr_err(
			"[AL3006 error]%s: could not register ps misc device\n",
		__func__);
		goto err_unregister_ps_input_device;
	}
 */
 
	return ret;

//err_unregister_ps_input_device:
//	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}




static int lightsensor_setup(struct tmd2771x_chip *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[TMD2771X error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "lightsensor";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[TMD2771X error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

/*	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[AL3006 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}
*/
	return ret;

//err_unregister_ls_input_device:
//	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}






/* xuhuashan@gmail.com 2011.05.28 begin */
/* Use internal state when set TMD2771X Enable Register */
/* Add timeout for register read and write */
static u8 tmd2771x_calc_enable_value(struct tmd2771x_chip *chip)
{
	   u8 value;
       value = chip->pdata->ps_interrupt_enable |
               chip->pdata->als_interrupt_enable |
               chip->pdata->wait_enable | chip->pdata->ps_enable |
               chip->pdata->als_enable | chip->pdata->power_on;
       return value;
}

static int tmd2771x_write_reg(struct i2c_client *client, u8 command, u8 value)
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

static int tmd2771x_read_reg(struct i2c_client *client, u8 command,
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
/* xuhuashan@gmail.com 2011.05.28 end */

static void tmd2771x_set_wait_en(struct tmd2771x_chip *chip, u8 val)
{
       u8 value, temp;

       val = (val > 0) ? 1 : 0;

       mutex_lock(&chip->lock);
       temp = (val << TMD2771X_WEN_SHIFT) & TMD2771X_WEN;
#if 0
       if (temp == chip->pdata->wait_enable) {
               mutex_unlock(&chip->lock);
               return;
       }
#endif

/* xuhuashan@gmail.com 2011.05.28 begin */
/* Use internal state when set TMD2771X Enable Register */
       value = tmd2771x_calc_enable_value(chip);
       value &= ~TMD2771X_WEN;
       value |= temp;
       if(tmd2771x_write_reg(chip->client,
              TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, 
              value) == 0)
              chip->pdata->wait_enable = temp;
/* xuhuashan@gmail.com 2011.05.28 end */
       mutex_unlock(&chip->lock);
}

static u8 tmd2771x_get_wait_en(struct tmd2771x_chip *chip)
{
       return chip->pdata->wait_enable >> TMD2771X_WEN_SHIFT;
}

static void tmd2771x_set_proximity_en(struct tmd2771x_chip *chip, u8 val)
{
       u8 value, temp;

       val = (val > 0) ? 1 : 0;


       mutex_lock(&chip->lock);
       temp = (val << TMD2771X_PEN_SHIFT) & TMD2771X_PEN;
#if 0
       if (temp == chip->pdata->ps_enable) {
               mutex_unlock(&chip->lock);
               return;
       }
#endif

/* xuhuashan@gmail.com 2011.05.28 begin */
/* Use internal state when set TMD2771X Enable Register */
       value = tmd2771x_calc_enable_value(chip);
       value &= ~TMD2771X_PEN;
       value |= temp;
       if(tmd2771x_write_reg(chip->client,
              TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, 
			  value) == 0)
           chip->pdata->ps_enable = temp;
/* xuhuashan@gmail.com 2011.05.28 end */

       mutex_unlock(&chip->lock);
printk("enable proximitysensor power on enable =%d\n",val);

}

static u8 tmd2771x_get_proximity_en(struct tmd2771x_chip *chip)
{
       return chip->pdata->ps_enable >> TMD2771X_PEN_SHIFT;
}

static void tmd2771x_set_light_en(struct tmd2771x_chip *chip, u8 val)
{
       u8 value, temp;

       val = (val > 0) ? 1 : 0;

       mutex_lock(&chip->lock);
       temp = (val << TMD2771X_AEN_SHIFT) & TMD2771X_AEN;
#if 0
       if (temp == chip->pdata->als_enable) {
               mutex_unlock(&chip->lock);
               return;
       }
#endif

/* xuhuashan@gmail.com 2011.05.28 begin */
/* Use internal state when set TMD2771X Enable Register */
       value = tmd2771x_calc_enable_value(chip);
       value &= ~TMD2771X_AEN;
       value |= temp;
       if(tmd2771x_write_reg(chip->client,
              TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, value) == 0)
       {
              chip->pdata->als_enable = temp;
       }
/* xuhuashan@gmail.com 2011.05.28 end */

       mutex_unlock(&chip->lock);
printk("enable lightsensor power on enable =%d\n",val);
}

static u8 tmd2771x_get_light_en(struct tmd2771x_chip *chip)
{
       
       return chip->pdata->als_enable >> TMD2771X_AEN_SHIFT;
}

static void tmd2771x_set_power_on(struct tmd2771x_chip *chip, u8 val)
{
       u8 value, temp;

       val = (val > 0) ? 1 : 0;

       mutex_lock(&chip->lock);
       temp = (val << TMD2771X_PON_SHIFT) & TMD2771X_PON;
#if 0
       if (temp == chip->pdata->power_on) {
               mutex_unlock(&chip->lock);
               return;
       }
#endif

/* xuhuashan@gmail.com 2011.05.28 begin */
/* Use internal state when set TMD2771X Enable Register */
       value = tmd2771x_calc_enable_value(chip);
       value &= ~TMD2771X_PON;
       value |= temp;
       if(tmd2771x_write_reg(chip->client,
              TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, 
              value) == 0)
       {
               chip->pdata->power_on = temp;
       }
/* xuhuashan@gmail.com 2011.05.28 end */

       mutex_unlock(&chip->lock);
printk("enable bothsensor power on enable =%d\n",val);

}

static u8 tmd2771x_get_power_on(struct tmd2771x_chip *chip)
{
       return chip->pdata->power_on >> TMD2771X_PON_SHIFT;
}

static void tmd2771x_set_light_mag_pos_rising_value(struct tmd2771x_chip *chip,
                                                  u16 val)
{
       u8 temp_high, temp_low;

       mutex_lock(&chip->lock);
       if (val == chip->pdata->als_interrupt_h_thres) {
               mutex_unlock(&chip->lock);
               return;
       }

       temp_high = (val >> BITS_PER_BYTE) & TMD2771X_8BIT_MASK;
       temp_low = val & TMD2771X_8BIT_MASK;

       chip->pdata->als_interrupt_h_thres = (temp_high << BITS_PER_BYTE) |
                                            temp_low;
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_AIHTL, temp_low);
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_AIHTH, temp_high);
       mutex_unlock(&chip->lock);
}

static u16 tmd2771x_get_light_mag_pos_rising_value(struct tmd2771x_chip *chip)
{	
       return chip->pdata->als_interrupt_h_thres;
}

static void tmd2771x_set_light_mag_neg_rising_value(struct tmd2771x_chip *chip,
                                                  u16 val)
{
       u8 temp_high, temp_low;

       mutex_lock(&chip->lock);
       if (val == chip->pdata->als_interrupt_l_thres) {
               mutex_unlock(&chip->lock);
               return;
       }

       temp_high = (val >> BITS_PER_BYTE) & TMD2771X_8BIT_MASK;
       temp_low = val & TMD2771X_8BIT_MASK;

       chip->pdata->als_interrupt_l_thres = (temp_high << BITS_PER_BYTE) |
                                            temp_low;
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_AILTL, temp_low);
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_AILTH, temp_high);
       mutex_unlock(&chip->lock);
}

static u16 tmd2771x_get_light_mag_neg_rising_value(struct tmd2771x_chip *chip)
{
       //liutao@wind-mobi.com 2012-4-18 begin
	//add proximity debug interface
	//yuanlan@wind-mobi.com 2012-4-18 review
	int ret = -1;
	 printk("wilson reset interrupt register \n");
         ret =  tmd2771x_write_reg(chip->client,
               TMD2771X_PS_ALS_INT_CLEAR_COMMAND, 0);
	 if(ret < 0)
	 	printk("wilson i2c write reset interrupt register faild\n");		 	
	 //liutao@wind-mobi.com 2012-4-18 end


	   return chip->pdata->als_interrupt_l_thres;
}

static void tmd2771x_set_proximity_mag_pos_rising_value
                               (struct tmd2771x_chip *chip, u16 val)
{
       u8 temp_high, temp_low;

       mutex_lock(&chip->lock);
       if (val == chip->pdata->ps_interrupt_h_thres) {
               mutex_unlock(&chip->lock);
               return;
       }

       temp_high = (val >> BITS_PER_BYTE) & TMD2771X_8BIT_MASK;
       temp_low = val & TMD2771X_8BIT_MASK;

       chip->pdata->ps_interrupt_h_thres = (temp_high << BITS_PER_BYTE) |
                                           temp_low;
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_PIHTL, temp_low);
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_PIHTH, temp_high);
       mutex_unlock(&chip->lock);
}

static u16 tmd2771x_get_proximity_mag_pos_rising_value
                               (struct tmd2771x_chip *chip)
{
	//liutao@wind-mobi.com 2012-4-18 begin
	//add proximity debug interface
	//yuanlan@wind-mobi.com 2012-4-18 review
	printk("wilson ps_lower_th =%d ps_upper_th=%d\n", chip->ps_lower_th,chip->ps_upper_th);
	 //liutao@wind-mobi.com 2012-4-18 end
	   return chip->pdata->ps_interrupt_h_thres;
}

static void tmd2771x_set_proximity_mag_neg_rising_value
                               (struct tmd2771x_chip *chip, u16 val)
{
       u8 temp_high, temp_low;

       mutex_lock(&chip->lock);
       if (val == chip->pdata->ps_interrupt_l_thres) {
               mutex_unlock(&chip->lock);
               return;
       }

       temp_high = (val >> BITS_PER_BYTE) & TMD2771X_8BIT_MASK;
       temp_low = val & TMD2771X_8BIT_MASK;

       chip->pdata->ps_interrupt_l_thres = (temp_high << BITS_PER_BYTE) |
                                           temp_low;
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_PILTL, temp_low);
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_PILTH, temp_high);
       mutex_unlock(&chip->lock);
}

static u16 tmd2771x_get_proximity_mag_neg_rising_value
                               (struct tmd2771x_chip *chip)
{
       return chip->pdata->ps_interrupt_l_thres;
}

static void tmd2771x_set_proximity_mag_either_rising_period
                               (struct tmd2771x_chip *chip, u8 val)
{
       u8 value, temp;

       mutex_lock(&chip->lock);
       temp = (val << TMD2771X_PPERS_SHIFT) & TMD2771X_PPERS_MASK;
       if (temp == chip->pdata->ps_interrupt_persistence) {
               mutex_unlock(&chip->lock);
               return;
       }

       tmd2771x_read_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PERS, &value, 1);
       value &= ~TMD2771X_PPERS_MASK;
       value |= temp;
       chip->pdata->ps_interrupt_persistence = temp;
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_PERS, value);
       mutex_unlock(&chip->lock);
}

static u8 tmd2771x_get_proximity_mag_either_rising_period
                               (struct tmd2771x_chip *chip)
{
       return chip->pdata->ps_interrupt_persistence >> TMD2771X_PPERS_SHIFT;
}

static void tmd2771x_set_light_mag_either_rising_period
                               (struct tmd2771x_chip *chip, u8 val)
{
       u8 value, temp;

       mutex_lock(&chip->lock);
       temp = (val << TMD2771X_APERS_SHIFT) & TMD2771X_APERS_MASK;
       if (temp == chip->pdata->als_interrupt_persistence) {
               mutex_unlock(&chip->lock);
               return;
       }

       tmd2771x_read_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PERS, &value, 1);
       value &= ~TMD2771X_APERS_MASK;
       value |= temp;
       chip->pdata->als_interrupt_persistence = temp;
       tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_PERS, value);
       mutex_unlock(&chip->lock);
}

static u8 tmd2771x_get_light_mag_either_rising_period
                               (struct tmd2771x_chip *chip)
{
       return chip->pdata->als_interrupt_persistence >> TMD2771X_APERS_SHIFT;
}

static u16 tmd2771x_get_proximity_raw(struct tmd2771x_chip *chip)
{
       u8 values[2];
       u16 proximity_raw;

       mutex_lock(&chip->lock);
       tmd2771x_read_reg(chip->client,
               TMD2771X_AUTO_INCREMENT_COMMAND | TMD2771X_PDATAL, values, 2);
       proximity_raw = (values[1] << BITS_PER_BYTE) | values[0];
       mutex_unlock(&chip->lock);

       return proximity_raw;
}

static u16 tmd2771x_get_adc0(struct tmd2771x_chip *chip)
{
       u8 values[2];
       u16 ch0_data;

       mutex_lock(&chip->lock);
       tmd2771x_read_reg(chip->client,
               TMD2771X_AUTO_INCREMENT_COMMAND | TMD2771X_CH0DATAL, values, 2);
       ch0_data = (values[1] << BITS_PER_BYTE) | values[0];
       mutex_unlock(&chip->lock);

       return ch0_data;
}

static u16 tmd2771x_get_adc1(struct tmd2771x_chip *chip)
{
       u8 values[2];
       u16 ch1_data;

       mutex_lock(&chip->lock);
       tmd2771x_read_reg(chip->client,
               TMD2771X_AUTO_INCREMENT_COMMAND | TMD2771X_CH1DATAL, values, 2);
       ch1_data = (values[1] << BITS_PER_BYTE) | values[0];
       mutex_unlock(&chip->lock);

       return ch1_data;
}

/*
 * Conversions between lux and ADC values.
 *
 * The basic formulas for lux are as follows.
 * lux1 = GA * 24 * (CH0DATA - 2 * CH1DATA) / (integration time * gain)
 * lux2 = GA * 24 * (0.8 * CH0DATA - 1.4 * CH1DATA) / (integration time * gain)
 * lux = MAX(lux1, lux2)
 * GA is Glass Attenuation.
 *
 */
static u32 tmd2771x_get_illuminance0_input(struct tmd2771x_chip *chip)
{
       /* xuhuashan@gmail.com 2011.06.13 begin */
       /* Fix the issue of the calculating overflow */
       int lux, lux1, lux2;
       u32 als_int_time, als_gain,
           ch0, ch1, glass_attenuation = 1;
       /* xuhuashan@gmail.com 2011.06.13 end */

       /* integration time = 2.7ms * (256 - als_time)
          The following equation removes floating point numbers
          Therefore, the result is 10 times greter than the original value */
       als_int_time = 27 * (256 - chip->pdata->als_time);

       ch0 = tmd2771x_get_adc0(chip);
       ch1 = tmd2771x_get_adc1(chip);

       switch (chip->pdata->als_gain) {
       case TMD2771X_AGAIN_1X:
               als_gain = 1;
               break;
       case TMD2771X_AGAIN_8X:
               als_gain = 8;
               break;
       case TMD2771X_AGAIN_16X:
               als_gain = 16;
               break;
       case TMD2771X_AGAIN_120X:
               als_gain = 120;
               break;
       default:
               als_gain = 1;
               break;
       }

       if (chip->pdata->glass_attenuation > 0)
               glass_attenuation = chip->pdata->glass_attenuation;

       /* Because the integration time is 10 times greater than
          the original value,
          numerator is mutiplied by 10 */

       /* xuhuashan@gmail.com 2011.06.13 begin */
       /* Fix the issue of the calculating overflow */
       lux1 = (10 * (int)glass_attenuation * 24 * ((int)ch0 - 2 * (int)ch1)) /
              ((int)als_int_time * (int)als_gain);
       lux2 = ((int)glass_attenuation * 24 * (8 * (int)ch0 - 14 * (int)ch1)) /
              ((int)als_int_time * (int)als_gain);
       lux = lux1 > lux2 ? lux1 : lux2;
       lux = (lux > 0) ? lux : 0;
       /* xuhuashan@gmail.com 2011.06.13 begin */

       return (u32)lux;
}

static int tmd2771x_thresh_handler_th(struct iio_dev *dev_info,
                              int index, s64 timestamp, int no_test)
{
       struct tmd2771x_chip *chip = dev_info->dev_data;
      
       chip->last_timestamp = timestamp;
       schedule_work(&chip->work_thresh);
       //  enable_irq(chip->client->irq);    //modify by wilson
       return 0;
}

/* xuhuashan@gmail.com 2011.05.25 begin */
/* Add poll support for sysfs file 'illuminance0_input' and 'proximity_raw'. */
/* Add PS calibrate function */
static void tmd2771x_thresh_handler_bh(struct work_struct *work_s)
{
  //liutao@wind-mobi.com 2012-3-15 begin
  //improve proximity calibrate
  //yuanlan@wind-mobi.com 2012-3-15 review
       struct tmd2771x_chip *chip =
               container_of(work_s, struct tmd2771x_chip, work_thresh);
	u8 prox_event;
       u8 value;
       u32 new_value;
	unsigned int oldval = chip->ps_old_val;
	unsigned int newval;
	//liutao@wind-mobi.com 2012-02-06 begin
	//proximity sensor is sometimes invalid
	//liubin@wind-mobi.com 2012-02-06 review
	value = chip->pdata->ps_time;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PTIME, value);
	//liutao@wind-mobi.com 2012-02-06 end
       tmd2771x_read_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_STATUS,
               &value, 1);
       if (chip->pdata->ps_interrupt_enable) {
              // if (value & TMD2771X_PINT)   modify by wilson		   
             if(!chip->cali_mode){
		      newval = tmd2771x_get_proximity_raw(chip); 					
          // printk("wilson kernel  ps_upper_th =%d,ps_lower_th=%d,newval =%d,oldval =%d,cal_avg=%d -----\n",chip->ps_upper_th,chip->ps_lower_th,newval,oldval,chip->cali_cur_val);
		      if(( (newval < chip->ps_lower_th))|| ( (newval > chip->ps_upper_th))){				
				//if(((oldval >= chip->ps_lower_th) && (newval < chip->ps_lower_th))
				//|| ((oldval <= chip->ps_upper_th) && (newval > chip->ps_upper_th)))	
			 		/*if(++chip->ps_exec_cnt >= 2) {	//5	 wilson 2
						chip->ps_old_val = newval;*/
						//newval = newval > chip->ps_upper_th ? 0 : 1;
				prox_event= newval > chip->ps_upper_th ? 0 : 1;
				input_report_abs(chip->ps_input_dev, ABS_DISTANCE, prox_event);
	                	input_sync(chip->ps_input_dev);
	                   //printk("wilson***prox_newval=%d***----\n",newval);
		      }
	    }				
            else
                     complete(&chip->cali_completion);
      }      
     if (chip->pdata->als_interrupt_enable) {
               //if (value & TMD2771X_AINT)   modify by wilson
           if(!chip->cali_mode){                        
		      new_value = tmd2771x_get_illuminance0_input(chip);			  
		      input_report_abs(chip->ls_input_dev, ABS_MISC, new_value);
	             input_sync(chip->ls_input_dev);
           }
     }
       /* Acknowledge proximity and ambient light sensor interrupt */
       //liutao@wind-mobi.com 2011/11/17 begin
       //fix light&proximity sensor sometimes can't use
       //liubing@wind-mobi review
      // tmd2771x_write_reg(chip->client,
        //       TMD2771X_PS_ALS_INT_CLEAR_COMMAND, 0);

   enable_irq(chip->client->irq); 
   tmd2771x_write_reg(chip->client,
               TMD2771X_PS_ALS_INT_CLEAR_COMMAND, 0);
       //liutao@wind-mobi.com end
      //liutao@wind-mobi.com 2012-3-15 end
}
/* xuhuashan@gmail.com 2011.06.17 end */
//liutao@wind-mobi.com 2011.12.30 begin
//sensor cts permission
//liubing@wind-mobi.com 2011.12.30 review
#define TMD2771X_OUTPUT(name)                                          \
static ssize_t tmd2771x_show_##name(struct device *dev,                \
               struct device_attribute *attr, char *buf)               \
{                                                                      \
       struct iio_dev *indio_dev = dev_get_drvdata(dev);               \
       struct tmd2771x_chip *chip = indio_dev->dev_data;               \
       u32 value = tmd2771x_get_##name(chip);                          \
       return sprintf(buf, "%d\n", value);                             \
}                                                                      \
static DEVICE_ATTR(name, S_IRUGO, tmd2771x_show_##name, NULL);

#define TMD2771X_INPUT(name)                                           \
static ssize_t tmd2771x_store_##name(struct device *dev,               \
       struct device_attribute *attr, const char *buf, size_t count)   \
{                                                                      \
       struct iio_dev *indio_dev = dev_get_drvdata(dev);               \
       struct tmd2771x_chip *chip = indio_dev->dev_data;               \
       int ret;                                                        \
       unsigned long val;                                              \
                                                                       \
       if (!count)                                                     \
               return -EINVAL;                                         \
                                                                       \
       ret = strict_strtoul(buf, 10, &val);                            \
       if (ret) {                                                      \
               dev_err(dev, "fail: conversion %s to number\n", buf);   \
               return count;                                           \
       }                                                               \
       tmd2771x_set_##name(chip, val);                                 \
       return count;                                                   \
}                                                                      \
static ssize_t tmd2771x_show_##name(struct device *dev,                \
               struct device_attribute *attr, char *buf)               \
{                                                                      \
       struct iio_dev *indio_dev = dev_get_drvdata(dev);               \
       struct tmd2771x_chip *chip = indio_dev->dev_data;               \
       u16 value = tmd2771x_get_##name(chip);                          \
       return sprintf(buf, "%d\n", value);                             \
}                                                                      \
static DEVICE_ATTR(name, 0640,                                         \
               tmd2771x_show_##name, tmd2771x_store_##name);
//liutao@wind-mobi.com 2011.12.30 end
TMD2771X_OUTPUT(proximity_raw);
TMD2771X_OUTPUT(adc0);
TMD2771X_OUTPUT(adc1);
TMD2771X_OUTPUT(illuminance0_input);
TMD2771X_INPUT(power_on);
TMD2771X_INPUT(wait_en);
TMD2771X_INPUT(proximity_en);
TMD2771X_INPUT(light_en);
static IIO_CONST_ATTR(name, "tmd2771x");

static struct attribute *tmd2771x_attributes[] = {
       &dev_attr_proximity_raw.attr,
       &dev_attr_adc0.attr,
       &dev_attr_adc1.attr,
       &dev_attr_illuminance0_input.attr,
       &dev_attr_power_on.attr,
       &dev_attr_wait_en.attr,
       &dev_attr_proximity_en.attr,
       &dev_attr_light_en.attr,
       &iio_const_attr_name.dev_attr.attr,
       NULL
};

static const struct attribute_group tmd2771x_group = {
       .attrs = tmd2771x_attributes,
};

static ssize_t tmd2771x_read_interrupt_config(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
       struct iio_event_attr *this_attr = to_iio_event_attr(attr);
       struct iio_dev *indio_dev = dev_get_drvdata(dev);
       struct tmd2771x_chip *chip = indio_dev->dev_data;
       u8 val;
       int ret;

       ret = tmd2771x_read_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, &val, 1);
       if (ret < 0)
               return ret;

       return sprintf(buf, "%d\n", (val & this_attr->mask) ? 1 : 0);
}

static ssize_t tmd2771x_write_interrupt_config(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t len)
{
       struct iio_event_attr *this_attr = to_iio_event_attr(attr);
       struct iio_dev *indio_dev = dev_get_drvdata(dev);
       struct tmd2771x_chip *chip = indio_dev->dev_data;
       int ret, currentlyset, changed = 0;
       u8 valold;
       bool val;

       val = !(buf[0] == '0');

       mutex_lock(&indio_dev->mlock);

       ret = tmd2771x_read_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, &valold, 1);
       if (ret < 0)
               goto error_mutex_unlock;

       currentlyset = !!(valold & this_attr->mask);
       if (val == false && currentlyset) {
               valold &= ~this_attr->mask;
               changed = 1;
       } else if (val == true && !currentlyset) {
               changed = 1;
               valold |= this_attr->mask;
       }

       if (changed) {
               ret = tmd2771x_write_reg(chip->client,
                       TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, valold);
               if (ret < 0)
                       goto error_mutex_unlock;
       }

error_mutex_unlock:
       mutex_unlock(&indio_dev->mlock);

       return (ret < 0) ? ret : len;
}

IIO_EVENT_SH(threshold, &tmd2771x_thresh_handler_th);

IIO_EVENT_ATTR_SH(proximity_mag_either_rising_en,
                 iio_event_threshold,
                 tmd2771x_read_interrupt_config,
                 tmd2771x_write_interrupt_config,
                 TMD2771X_PIEN);
TMD2771X_INPUT(proximity_mag_pos_rising_value);
TMD2771X_INPUT(proximity_mag_neg_rising_value);
TMD2771X_INPUT(proximity_mag_either_rising_period);

IIO_EVENT_ATTR_SH(light_mag_either_rising_en,
                 iio_event_threshold,
                 tmd2771x_read_interrupt_config,
                 tmd2771x_write_interrupt_config,
                 TMD2771X_AIEN);
TMD2771X_INPUT(light_mag_pos_rising_value);
TMD2771X_INPUT(light_mag_neg_rising_value);
TMD2771X_INPUT(light_mag_either_rising_period);

static struct attribute *tmd2771x_event_attributes[] = {
       &iio_event_attr_proximity_mag_either_rising_en.dev_attr.attr,
       &dev_attr_proximity_mag_pos_rising_value.attr,
       &dev_attr_proximity_mag_neg_rising_value.attr,
       &dev_attr_proximity_mag_either_rising_period.attr,
       &iio_event_attr_light_mag_either_rising_en.dev_attr.attr,
       &dev_attr_light_mag_pos_rising_value.attr,
       &dev_attr_light_mag_neg_rising_value.attr,
       &dev_attr_light_mag_either_rising_period.attr,
       NULL
};

static struct attribute_group tmd2771x_event_attribute_group = {
       .attrs = tmd2771x_event_attributes,
};

/* xuhuashan@gmail.com 2011.06.17 begin */
/* Add PS calibrate function */
#if 1
static int tmd2771x_proximity_calibrate(struct tmd2771x_chip *chip,
                int count)
{
	int retval, retcnt = 0;
	u16 min_val, max_val, avg_val, val;
	u32 total;
// printk("wilson ********** count = %d*************\n",count);
	total = 0;
	while(count-- > 0) {
		u8 regval[2];
		retval = wait_for_completion_interruptible_timeout(
					&chip->cali_completion, 
					msecs_to_jiffies(10));
		if(retval < 0)
			break;

		tmd2771x_read_reg(chip->client,
				TMD2771X_AUTO_INCREMENT_COMMAND | TMD2771X_PDATAL,
				regval, 2);
		val = (regval[1] << BITS_PER_BYTE) | regval[0];
		if(!val)
			continue;
#if 0
		printk(KERN_INFO "wilson %s: get value[%d]= %d\n", __func__,count, (int)val);
#endif
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
		chip->cali_min_val = min_val;
		chip->cali_max_val = max_val;
		chip->cali_avg_val = total / retcnt;
#if 0
		printk("wilson %s: min=%d max=%d avg=%d\n",
					__func__,
					chip->cali_min_val,
					chip->cali_max_val,
					chip->cali_avg_val);
#endif
	}

	return retcnt;
}

static ssize_t tmd2771x_cali_trigger_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf,
                size_t count)
{
	struct tmd2771x_chip *chip
		= container_of(kobj, struct tmd2771x_chip, cali_kobj);
	int cali_val;
	int retval = 0;
     
	cali_val = simple_strtol(buf, NULL, 10);
      // printk("wilson kernel cali_val=%d-----\n",cali_val);
	mutex_lock(&chip->lock);
	if(cali_val > 0) {
		u8 enable = TMD2771X_PEN | TMD2771X_PON;

		chip->cali_mode = 1;

		tmd2771x_write_reg(chip->client, 
				TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE,
				enable);

		retval = tmd2771x_proximity_calibrate(chip, cali_val);

		enable = tmd2771x_calc_enable_value(chip);
		tmd2771x_write_reg(chip->client,
				TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE,
				enable);

		chip->cali_mode = 0;
	}
	mutex_unlock(&chip->lock);

	/* Send notifications while calibrate succeed. */
	if(retval > 0) {
		sysfs_notify(&chip->cali_kobj, NULL, "avg_value");
		sysfs_notify(&chip->cali_kobj, NULL, "max_value");
		sysfs_notify(&chip->cali_kobj, NULL, "min_value");
	}

	return count;
}

static ssize_t tmd2771x_cali_max_value_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct tmd2771x_chip *chip
		= container_of(kobj, struct tmd2771x_chip, cali_kobj);

	return sprintf(buf, "%d\n", chip->cali_max_val);
}

static ssize_t tmd2771x_cali_min_value_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct tmd2771x_chip *chip
		= container_of(kobj, struct tmd2771x_chip, cali_kobj);

	return sprintf(buf, "%d\n", chip->cali_min_val);
}

static ssize_t tmd2771x_cali_avg_value_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct tmd2771x_chip *chip
		= container_of(kobj, struct tmd2771x_chip, cali_kobj);	
	return sprintf(buf, "%d\n", chip->cali_avg_val);
}

static ssize_t tmd2771x_cali_cal_value_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct tmd2771x_chip *chip
		= container_of(kobj, struct tmd2771x_chip, cali_kobj);

	return sprintf(buf, "%d\n", chip->cali_cur_val);
}


static ssize_t tmd2771x_cali_cal_value_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf,
                size_t count)
{
	struct tmd2771x_chip *chip
		= container_of(kobj, struct tmd2771x_chip, cali_kobj);

	chip->cali_cur_val = simple_strtol(buf, NULL, 10);
	tsl_calc_proximity_threshold((uint16_t)chip->cali_cur_val ,
					&chip->ps_upper_th, &chip->ps_lower_th);
 // printk("wilson kernel ps_upper_th =%d,ps_lower_th=%d,cali_cur_val =%d-----\n",chip->ps_upper_th,chip->ps_lower_th,(uint16_t)chip->cali_cur_val );
  
	sysfs_notify(kobj, NULL, "cal_value");

	return count;
}

#define KOBJ_ATTR(_name, _mode, _show, _store)   \
struct kobj_attribute kobj_attr_##_name = __ATTR(_name, _mode, _show, _store)

/*liutao@wind-mobi.com 2011/12/30 start*/
//sensor cts permission
//liubing@wind-mobi.com 2011/12/30 review
KOBJ_ATTR(trigger, 0600, NULL, tmd2771x_cali_trigger_store);
KOBJ_ATTR(max_value, 0444, tmd2771x_cali_max_value_show, NULL);
KOBJ_ATTR(min_value, 0444, tmd2771x_cali_min_value_show, NULL);
KOBJ_ATTR(avg_value, 0444, tmd2771x_cali_avg_value_show, NULL);
//liutao@wind-mobi.com 2011/12/30 end
/*liubing@wind-mobi.com 2011/12/16 start*/
/*sys file must not writeable, for cts require*/
/*KOBJ_ATTR(cal_value, 0666, tmd2771x_cali_cal_value_show,
                           tmd2771x_cali_cal_value_store);*/
   //liutao@wind-mobi.com 2012-3-15 begin
  //improve proximity calibrate
  //yuanlan@wind-mobi.com 2012-3-15 review                          
KOBJ_ATTR(cal_value, 0600, tmd2771x_cali_cal_value_show,
                           tmd2771x_cali_cal_value_store);
   //liutao@wind-mobi.com 2012-3-15 end
/*liubing@wind-mobi.com 2011/12/16 end*/

static struct attribute *tmd2771x_calibrate_attributes[] = {
        &kobj_attr_trigger.attr,
        &kobj_attr_max_value.attr,
        &kobj_attr_min_value.attr,
        &kobj_attr_avg_value.attr,
        &kobj_attr_cal_value.attr,
        NULL
};
/* xuhuashan@gmail.com 2011.06.17 end */
#endif
static void tmd2771x_initialize_chip(struct tmd2771x_chip *chip)
{
       u8 value, temp_low, temp_high;

       /* Disable and powerdown */
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, 0);

       /* ALS timing register */
       value = chip->pdata->als_time;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_ATIME, value);

       /* PS timing register */
       value = chip->pdata->ps_time;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PTIME, value);

       /* Wait time register */
       value = chip->pdata->wait_time;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_WTIME, value);

       /* Proximity pulse count register */
       value = chip->pdata->ps_pulse_count;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PPCOUNT, value);

       /* ALS interrupt threshold register */
       temp_high = (chip->pdata->als_interrupt_l_thres >> BITS_PER_BYTE);
       temp_low = chip->pdata->als_interrupt_l_thres & TMD2771X_8BIT_MASK;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_AILTL, temp_low);
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_AILTH, temp_high);

       temp_high = (chip->pdata->als_interrupt_h_thres >> BITS_PER_BYTE);
       temp_low = chip->pdata->als_interrupt_h_thres & TMD2771X_8BIT_MASK;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_AIHTL, temp_low);
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_AIHTH, temp_high);

       /* PS interrupt threshold register */
       temp_high = (chip->pdata->ps_interrupt_l_thres >> BITS_PER_BYTE);
       temp_low = chip->pdata->ps_interrupt_l_thres & TMD2771X_8BIT_MASK;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PILTL, temp_low);
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PILTH, temp_high);

       temp_high = (chip->pdata->ps_interrupt_h_thres >> BITS_PER_BYTE);
       temp_low = chip->pdata->ps_interrupt_h_thres & TMD2771X_8BIT_MASK;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PIHTL, temp_low);
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PIHTH, temp_high);

       /* Persistence register */
       value = chip->pdata->ps_interrupt_persistence |
               chip->pdata->als_interrupt_persistence;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_PERS, value);

       /* Configuration register */
       value = chip->pdata->wait_long;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_CONFIG, value);

       /* Control register */
       value = chip->pdata->ps_drive_strength | chip->pdata->ps_diode |
               chip->pdata->als_gain;
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_CONTROL, value);

       /* Enable register */
	   value = tmd2771x_calc_enable_value(chip);
       tmd2771x_write_reg(chip->client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_ENABLE, value);

       msleep(TMD2771X_POWERUP_WAIT_TIME);
}

/* xuhuashan@gmail.com 2011.06.17 begin */
/* Add PS calibrate function */
#if 1
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
        .default_attrs = tmd2771x_calibrate_attributes,
};
/* xuhuashan@gmail.com 2011.06.17 end */
#endif
static int __devinit tmd2771x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
       struct tmd2771x_chip *chip;
       int ret;
       u8 device_id;

       chip = kzalloc(sizeof(struct tmd2771x_chip), GFP_KERNEL);
       if (!chip)
               return -ENOMEM;

       chip->pdata = client->dev.platform_data;

       chip->client = client;
       i2c_set_clientdata(client, chip);
       mutex_init(&chip->lock);
       INIT_WORK(&chip->work_thresh, tmd2771x_thresh_handler_bh);

       if (chip->pdata->control_power_source)
               chip->pdata->control_power_source(1);

       /* Detect device id */
       msleep(TMD2771X_POWERUP_WAIT_TIME);
       ret = tmd2771x_read_reg(client,
               TMD2771X_DEFAULT_COMMAND | TMD2771X_ID, &device_id, 1);
       if (ret < 0) {
               dev_err(&client->dev, "failed to detect device id\n");
               goto error_detect_id;
       }
	// printk("wilson read ID register device_id  =%d\n",device_id);
    if(chip->pdata->init)//add by xiongbiao
		chip->pdata->init();/*init sensor gpio interrupt pin*/

	ret = psensor_setup(chip);
	if (ret < 0) {
		pr_err("[TMD2771X  error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}

       ret = lightsensor_setup(chip);
	if (ret < 0) {
		pr_err("[TMD2771X error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}
   
       chip->indio_dev = iio_allocate_device();
       if (!chip->indio_dev)
               goto error_allocate_iio;

       chip->indio_dev->attrs = &tmd2771x_group;
       chip->indio_dev->dev.parent = &client->dev;
       chip->indio_dev->dev_data = (void *)(chip);
       chip->indio_dev->num_interrupt_lines = 1;
       chip->indio_dev->event_attrs = &tmd2771x_event_attribute_group;
       chip->indio_dev->driver_module = THIS_MODULE;
       chip->indio_dev->modes = INDIO_DIRECT_MODE;

       ret = iio_device_register(chip->indio_dev);
       if (ret)
               goto error_register_iio;

/* xuhuashan@gmail.com 2011.06.17 begin */
/* Add PS calibrate function */
       ret = kobject_init_and_add(&chip->cali_kobj, &cali_kobj_ktype,
               &chip->indio_dev->dev.kobj, "calibrate");
       if(ret)
       	       goto error_add_kobject;
       init_completion(&chip->cali_completion);
       chip->cali_mode = 0;
       chip->cali_max_val = 0;
       chip->cali_min_val = 0;
       chip->cali_avg_val = 0;
       chip->cali_cur_val = 0;
	 chip->ps_exec_cnt = 0;
	 chip->ps_old_val = 0;
	 chip->ps_upper_th = 700;
	 chip->ps_lower_th = 600;
/* xuhuashan@gmail.com 2011.06.17 end */

       if (client->irq > 0) {
               /* xuhuashan@gmail.com 2011.07.04 begin */
               /* Add IRQF_NO_SUSPEND to make sure the irq can wakeup
                * the phone while system is suspended. */
               ret = iio_register_interrupt_line(client->irq,
                                 chip->indio_dev,
                                 0,
                                 IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
                                 "tmd2771x");
               /* xuhuashan@gmail.com 2011.07.04 end */
               if (ret)
                       goto error_register_interrupt;

               iio_add_event_to_list(&iio_event_threshold,
                                   &chip->indio_dev->interrupts[0]->ev_list);

       }

       tmd2771x_initialize_chip(chip);

       dev_info(&client->dev, "%s registered\n", id->name);

       return 0;

error_register_interrupt:
       iio_device_unregister(chip->indio_dev);
error_add_kobject:
       kobject_del(&chip->cali_kobj);
error_register_iio:
       iio_free_device(chip->indio_dev);
error_allocate_iio:
err_lightsensor_setup:
	input_unregister_device(chip->ps_input_dev);
	input_free_device(chip->ps_input_dev);
err_psensor_setup:	
	if(chip->pdata->exit)//add by xiongbiao
		chip->pdata->exit(NULL); /* free gpio request */
error_detect_id:
       kfree(chip);
       return ret;
}

static int __devexit tmd2771x_remove(struct i2c_client *client)
{
       struct tmd2771x_chip *chip = i2c_get_clientdata(client);

       if (chip->client->irq > 0)
               iio_unregister_interrupt_line(chip->indio_dev, 0);

       kobject_del(&chip->cali_kobj);
       iio_device_unregister(chip->indio_dev);
       iio_free_device(chip->indio_dev);
	   if(chip->pdata->exit)//add by xiongbiao
	   		chip->pdata->exit(NULL); /* free gpio request */
       kfree(chip);

       return 0;
}

#ifdef CONFIG_PM
static int tmd2771x_suspend(struct i2c_client *client, pm_message_t mesg)
{
       struct tmd2771x_chip *chip = i2c_get_clientdata(client);

       if (chip->pdata->control_power_source)
               chip->pdata->control_power_source(0);

       return 0;
}

static int tmd2771x_resume(struct i2c_client *client)
{
       struct tmd2771x_chip *chip = i2c_get_clientdata(client);

       if (chip->pdata->control_power_source)
               chip->pdata->control_power_source(1);
       tmd2771x_initialize_chip(chip);

       return 0;
}
#else
#define tmd2771x_suspend NULL
#define tmd2771x_resume NULL
#endif

static const struct i2c_device_id tmd2771x_id[] = {
       { "tmd27711", 0 },
       { "tmd27713", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, tmd2771x_id);

static struct i2c_driver tmd2771x_i2c_driver = {
       .driver = {
               .name   = "tmd2771x",
       },
       .probe          = tmd2771x_probe,
       .remove         = __exit_p(tmd2771x_remove),
       .suspend        = tmd2771x_suspend,
       .resume         = tmd2771x_resume,
       .id_table       = tmd2771x_id,
};

static int __init tmd2771x_init(void)
{
       return i2c_add_driver(&tmd2771x_i2c_driver);
}
module_init(tmd2771x_init);

static void __exit tmd2771x_exit(void)
{
       i2c_del_driver(&tmd2771x_i2c_driver);
}
module_exit(tmd2771x_exit);

MODULE_AUTHOR("Donggeun Kim <dg77....@samsung.com>");
MODULE_DESCRIPTION("TMD2771X Proximity/Ambient Light Sensor driver");
MODULE_LICENSE("GPL");
