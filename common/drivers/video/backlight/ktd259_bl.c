/*
 * linux/drivers/video/backlight/ktd259_bl.c
 *company:wind-mobi,shanghai.
 *author:yaogangxiang@wind-mobi.com
 *date:2011.10.20
 * simple ktd259 backlight control.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/ktd259_bl.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/gpio.h>
#include <linux/delay.h>
// S2C timing optimization
#if 0
#include <mach/reg_sysfg.h>
#define SET_S2C_HIGH()  do{__REG32(HW_GPIO_BASE + 0x10) |= (1 << 17);} while(0)
#define SET_S2C_LOW()   do{__REG32(HW_GPIO_BASE + 0x10) &= ~(1 << 17);} while(0)
//#else

//#define SET_S2C_HIGH() gpio_set_value(,1)
//#define SET_S2C_LOW()  gpio_set_value(GPIO_LCD_BACKLIGHT,0)
#endif

struct ktd259_bl_data {
	bool suspend_flag;
	int (*notify) (int brightness);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct backlight_device *bl;
	struct early_suspend early_suspend_desc;
#endif
};


// S2C timing optimization

static int last_brightness_step = 0;
static int gpio_backlight;

static int ktd259_backlight_update_status(struct backlight_device *bl)
{
    struct ktd259_bl_data *pb = dev_get_drvdata(&bl->dev);
	//struct platform_ktd259_backlight_data *bcm_bl_data;
    int brightness = bl->props.brightness;
    int  last_brightness_step_tmp= last_brightness_step;
    static long old_jiffies = 0;
	int sleep_usec = 0;

	unsigned long flag = 0;
       spinlock_t spinlock = SPIN_LOCK_UNLOCKED;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(brightness);

	if (brightness > 31){
		brightness = 31;
	}
	
	if (brightness == 0)
	{
        last_brightness_step = brightness;
		gpio_request(gpio_backlight, "gpio_bl");
		gpio_direction_output(gpio_backlight, 1);
		
		gpio_set_value(gpio_backlight,0);
		
        	//SET_S2C_LOW();
                msleep(5);
		gpio_free(gpio_backlight);
	}
	else{
		if(pb->suspend_flag == 1)
                    return 0;
		last_brightness_step = brightness;
		if( last_brightness_step_tmp < brightness){
      		  brightness=last_brightness_step_tmp + 32 - brightness;
		  }
		else{
              brightness =  last_brightness_step_tmp - brightness;       
         }	

		gpio_request(gpio_backlight, "gpio_bl");
		gpio_direction_output(gpio_backlight, 1);
		

		
		sleep_usec = jiffies_to_usecs(jiffies - old_jiffies);
		if (sleep_usec < 500)
		{
			udelay(500 - sleep_usec);
		}
		
		spin_lock_irqsave(&spinlock, flag);		
		
		while (brightness){ 
/* songjinguo@wind-mobi.com 2011.12.26 start */
/* fix bug 6330; lcd light time too long when wake up */
/* review by liubing */			
			gpio_set_value(gpio_backlight,0);
			udelay(5);
			gpio_set_value(gpio_backlight,1);
			udelay(5);
/* songjinguo@wind-mobi.com 2011.12.26 end */			
			brightness --;	
			}	
	    spin_unlock_irqrestore(&spinlock, flag);
		gpio_free(gpio_backlight);
		old_jiffies = jiffies;
	}

	return 0;
}

static int ktd259_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static struct backlight_ops ktd259_backlight_ops = {
	.update_status = ktd259_backlight_update_status,
	.get_brightness =ktd259_backlight_get_brightness,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ktd259_backlight_earlysuspend(struct early_suspend *desc)
{
	struct ktd259_bl_data *pb = container_of(desc, struct ktd259_bl_data,
							early_suspend_desc);	
/* yaogangxiang@wind-mobi.com 2011.11.24 begin */
//apply gpio before using
    gpio_request(gpio_backlight, "gpio_bl");
/* yaogangxiang@wind-mobi.com 2011.11.24 end */
    gpio_direction_output(gpio_backlight, 0);
    last_brightness_step =0;
    pb->suspend_flag = 1;
}

static void ktd259_backlight_earlyresume(struct early_suspend *desc)
{
	struct ktd259_bl_data *pb = container_of(desc, struct ktd259_bl_data,
							early_suspend_desc);
/* yaogangxiang@wind-mobi.com 2011.11.24 begin */
//free gpio after using
	gpio_free(gpio_backlight);
	msleep(80);
/* yaogangxiang@wind-mobi.com 2011.11.24 end */
	pb->suspend_flag = 0;

	backlight_update_status(pb->bl);
}
#else
#ifdef CONFIG_PM
static int ktd259_backlight_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd259_bl_data *pb = dev_get_drvdata(&bl->dev);

	gpio_direction_output(gpio_backlight, 0);

	return 0;
}

static int ktd259_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define ktd259_backlight_suspend          NULL
#define ktd259_backlight_resume            NULL
#endif
#endif

static int ktd259_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_ktd259_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct ktd259_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->notify = data->notify;
	pb->suspend_flag = 0;

	gpio_backlight = data->gpio_lcd_backlight;

    memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev,
				       pb, &ktd259_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;
    bl->props.brightness = bl->props.brightness;
	
	backlight_update_status(bl);


#ifdef CONFIG_HAS_EARLYSUSPEND	
	pb->early_suspend_desc.level = 0; //EARLY_SUSPEND_LEVEL_BLANK_SCREEN
	pb->early_suspend_desc.suspend = ktd259_backlight_earlysuspend;
	pb->early_suspend_desc.resume = ktd259_backlight_earlyresume;
	register_early_suspend(&pb->early_suspend_desc);
	pb->bl = bl;
#endif

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int ktd259_backlight_remove(struct platform_device *pdev)
{
	struct platform_ktd259_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd259_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&pb->early_suspend_desc);
#endif
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

static struct platform_driver ktd259_backlight_driver = {
	.driver = {
		   .name = "ktd259-backlight",
		   .owner = THIS_MODULE,
		   },
	.probe = ktd259_backlight_probe,
	.remove = ktd259_backlight_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ktd259_backlight_suspend,
	.resume = ktd259_backlight_resume,
#endif
};

static int __init ktd259_backlight_init(void)
{
	return platform_driver_register(&ktd259_backlight_driver);
}

late_initcall(ktd259_backlight_init);

static void __exit ktd259_backlight_exit(void)
{
	platform_driver_unregister(&ktd259_backlight_driver);
}

module_exit(ktd259_backlight_exit);

MODULE_DESCRIPTION("ktd259 Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ktd259-backlight");
