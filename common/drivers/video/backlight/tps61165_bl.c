/*
 * linux/drivers/video/backlight/tps61165_bl.c
 *company:wind-mobi,shanghai.
 *author:songjinguo@wind-mobi.com
 *date:2011.12.13
 * simple tps61165 backlight control.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/tps61165_bl.h>
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

struct tps61165_bl_data {
	bool suspend_flag;
	int (*notify) (int brightness);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct backlight_device *bl;
	struct early_suspend early_suspend_desc;
#endif
};
/* MASK */
#define TPS61165_ADDR 0x72
#define TPS61165_DATA_MASK 0x1F
#define TPS61165_RFA 0x00

/* delay time(us) */
#define TPS61165_RFA_WAIT_TIME 100
#define TEOS_AND_START_TIME 10
#define ES_DETECT_DELAY 120
#define ES_DETECT_TIME 300
#define ES_DETECT_WINDOW 1000

#define ES_DETECT_DELAY_ORIGNAL 120
//static int last_brightness_step = 0;
static int Is_Mode = 0;
static int gpio_backlight;
static unsigned char FirstUpdateBacklight = 0;

//feixiaoping@wind-mobi.com 2012-06-21begin
//this method of using spinlock is wroing ,but I don't modify because now our platform is stable.
//static spinlock_t spinlock = SPIN_LOCK_UNLOCKED;
//feixiaoping@wind-mobi.com 2012-06-21end
/*
 *parameter: mode == 1,set ctrl pin to high,or to low
 *return 0, right mode,or error
 */
/* feixiaoping@wind-mobi.com 20120627 begin */
#if 1
static bool debug_mask = 0;//|DEBUG_SUSPEND;

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#endif
/* feixiaoping@wind-mobi.com 20120627 end */
static int set_ctrlpin(int mode)
{
    if (mode == 0){
        gpio_set_value(gpio_backlight,0);
        udelay(50);
        gpio_set_value(gpio_backlight,1);
        udelay(15);
    } else if (mode == 1){
        gpio_set_value(gpio_backlight,0);
        udelay(15);
        gpio_set_value(gpio_backlight,1);
        udelay(50);
    } else
        return -1;
    return 0;
}


/*
  *function: set LCD backlight
  *param: the new backlight value
  *return: whether recieve ACK,0 is recieved,1 is not
 */
//int g_last=0;
//int g_Count=0;
static int set_backligt_value(int bright_value)
{
    int i;
    u8 addr = TPS61165_ADDR;
    u8 dat = (u8)((TPS61165_DATA_MASK & bright_value) | TPS61165_RFA);
	
    /* send device specific address */
    gpio_set_value(gpio_backlight,1);
    udelay(TEOS_AND_START_TIME);
    
    for (i = 7; i >= 0; i--){
        if ((addr & (0x01 << i)) == 0){
 	    set_ctrlpin(0);
        } else {
	    set_ctrlpin(1);
        }
    }	
    gpio_set_value(gpio_backlight,0);
    udelay(TEOS_AND_START_TIME);

    /* send dimming data */
    gpio_set_value(gpio_backlight,1);
    udelay(TEOS_AND_START_TIME);	
//    printk(KERN_INFO "[%s]: dimming data 0x%x\n", __func__, dat);	
    for (i = 7; i >= 0; i--){
        if ((dat & (0x01 << i)) == 0){
   	    set_ctrlpin(0);
	} else {
	    set_ctrlpin(1);
        }
    }
    gpio_set_value(gpio_backlight,0);
    udelay(TEOS_AND_START_TIME);

    /* wait ACK from device */
//	set_ctrlpin_low(1);
//	udelay(TPS61165_RFA_WAIT_TIME);

	/* data transmit over */
    gpio_set_value(gpio_backlight,1);

    return 0;
}

// S2C timing optimization

static int tps61165_backlight_update_status(struct backlight_device *bl)
{
    struct tps61165_bl_data *pb = dev_get_drvdata(&bl->dev);
    int brightness = bl->props.brightness;
//  static long old_jiffies = 0;
//  int sleep_usec = 0;
    
    unsigned long flag = 0;
    //feixiaoping@wind-mobi.com 2012-06-21begin
	//this method of using spinlock is wroing ,but I don't modify because now our platform is stable.
    spinlock_t spinlock = SPIN_LOCK_UNLOCKED;
    //feixiaoping@wind-mobi.com 2012-06-21end
 
    if (bl->props.power != FB_BLANK_UNBLANK) {
	brightness = 0;
        printk(KERN_ERR "props.power==FB_BLANK_UNBLANK\n");
    }

    if (bl->props.fb_blank != FB_BLANK_UNBLANK) {
	brightness = 0;
        printk(KERN_ERR "props.fb_blank==FB_BLANK_UNBLANK\n");
    }

    if (pb->notify)
	brightness = pb->notify(brightness);

    if (brightness > 31)
 	brightness = 31;
 
    if(pb->suspend_flag == 1){
		if(debug_mask)
    		printk("------tps61165------suspend_flag =%d and framework adjust backlightness =%d\n",brightness,bl->props.brightness);
        return 0;
    }
/* yaogangxiang@wind-mobi.com 2012.03.30 begin */
//  backlight flicker during the power-on
	if (FirstUpdateBacklight == 1){
		gpio_direction_output(gpio_backlight, 0);
		mdelay(4);
		FirstUpdateBacklight = 0;
//		gpio_direction_output(gpio_backlight, 1);
	}
//    printk(KERN_INFO "[%s]: backlight brightness is %d\n", __func__, brightness);	
//    gpio_request(gpio_backlight, "gpio_bl");
//    gpio_direction_output(gpio_backlight, 1);
    
/* sleep_usec = jiffies_to_usecs(jiffies - old_jiffies);
  if ((sleep_usec < 500)){
        udelay(500 - sleep_usec);
    }
*/
	/* feixiaoping@wind-mobi.com 2012.06.6 begin */
	//modify backlight can't light sometimes
    spin_lock_irqsave(&spinlock, flag);	
	/* feixiaoping@wind-mobi.com 2012.06.6 end */
    /* enter EasyScale Mode */
    if ((Is_Mode == 0)){
        gpio_set_value(gpio_backlight,1);
        udelay(ES_DETECT_DELAY);
        gpio_set_value(gpio_backlight,0);
        udelay(ES_DETECT_TIME);  
		//g_last=1;	
        Is_Mode = 1;
    } //else g_last=0;

    gpio_set_value(gpio_backlight, 1);
    udelay(ES_DETECT_WINDOW); 

    /* update LCD backlight brightness */	
	/* feixiaoping@wind-mobi.com 2012.06.6 begin */
		//modify backlight can't light sometimes
    	//spin_lock_irqsave(&spinlock, flag);		
    /* feixiaoping@wind-mobi.com 2012.06.6 end */
    set_backligt_value(brightness);
    spin_unlock_irqrestore(&spinlock, flag);
	if(debug_mask)
    	printk("------tps61165------backlight:brightness =%d and Is_Mode is %d\n",brightness,Is_Mode);	
		
	//  old_jiffies = jiffies;	
	//  gpio_free(gpio_backlight);
	/* yaogangxiang@wind-mobi.com 2012.03.30 end */
    return 0;
}

/* yaogangxiang@wind-mobi.com 2012.05.19 begin */
static int tps61165_backlight_test(struct backlight_device *bl)
{
    struct tps61165_bl_data *pb = dev_get_drvdata(&bl->dev);
    int brightness = bl->props.brightness;
   
    unsigned long flag = 0;
	//feixiaoping@wind-mobi.com 2012-06-21begin
	//this method of using spinlock is wroing ,but I don't modify because now our platform is stable.
    spinlock_t spinlock = SPIN_LOCK_UNLOCKED;
    //feixiaoping@wind-mobi.com 2012-06-21end
 
    if (bl->props.power != FB_BLANK_UNBLANK)
	brightness = 0;

    if (bl->props.fb_blank != FB_BLANK_UNBLANK)
	brightness = 0;

    if (pb->notify)
	brightness = pb->notify(brightness);

    if (brightness > 31)
 	brightness = 31;

    printk(KERN_ERR "test_node:suspend_flag=%d\n", pb->suspend_flag);
    gpio_direction_output(gpio_backlight, 0);
    mdelay(3);

    gpio_set_value(gpio_backlight, 1);
    udelay(ES_DETECT_DELAY_ORIGNAL);
    gpio_set_value(gpio_backlight, 0);
    udelay(ES_DETECT_TIME);     

    gpio_set_value(gpio_backlight, 1);
    udelay(ES_DETECT_WINDOW);
	
    /* update LCD backlight brightness */	
    spin_lock_irqsave(&spinlock, flag);		
    set_backligt_value(brightness);
    spin_unlock_irqrestore(&spinlock, flag);

    return 0;
}
/////////////
static int tps61165_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static struct backlight_ops tps61165_backlight_ops = {
	.update_status = tps61165_backlight_update_status,
	.get_brightness =tps61165_backlight_get_brightness,
	/* yaogangxiang@wind-mobi.com 2012.05.19 begin */
	//just for test
	.update_test = tps61165_backlight_test,
	/* yaogangxiang@wind-mobi.com 2012.05.19 end */
 
	
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tps61165_backlight_earlysuspend(struct early_suspend *desc)
{
	struct tps61165_bl_data *pb = container_of(desc, struct tps61165_bl_data,
							early_suspend_desc);	
	if(debug_mask)
        printk("------tps61165-----enter-tps61165_backlight_earlysuspend\n");
	//gpio_request(gpio_backlight, "gpio_bl");
    gpio_direction_output(gpio_backlight, 0);
    mdelay(3);
    pb->suspend_flag = 1;
    if(debug_mask)
        printk("------tps61165-----exit-tps61165_backlight_earlysuspend\n");

	//gpio_free(gpio_backlight);
}

static void tps61165_backlight_earlyresume(struct early_suspend *desc)
{
	
	struct tps61165_bl_data *pb = container_of(desc, struct tps61165_bl_data,
							early_suspend_desc);
	if(debug_mask)
        printk("------tps61165-----enter-tps61165_backlight_earlyresume\n");
	Is_Mode = 0;
	FirstUpdateBacklight = 0;
	pb->suspend_flag = 0;

	msleep(80);
	if(debug_mask)
        printk("------tps61165-----earlyresume:kernel adjust backlight=%d-\n",pb->bl->props.brightness);

	backlight_update_status(pb->bl);
	if(debug_mask)
        printk("------tps61165-----exit-tps61165_backlight_earlyresume\n");
}
#else
#ifdef CONFIG_PM
static int tps61165_backlight_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tps61165_bl_data *pb = dev_get_drvdata(&bl->dev);

	gpio_direction_output(gpio_backlight, 0);

	return 0;
}

static int tps61165_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define tps61165_backlight_suspend          NULL
#define tps61165_backlight_resume            NULL
#endif
#endif

static int tps61165_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_tps61165_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct tps61165_bl_data *pb;
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
				       pb, &tps61165_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}
	/* yaogangxiang@wind-mobi.com 2012.03.30 begin */
	//	backlight flicker during the power-on
	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;
        bl->props.brightness = bl->props.brightness;
    gpio_request(gpio_backlight,"gpio_bl");
    gpio_direction_output(gpio_backlight, 1);
	FirstUpdateBacklight = 1;
//        gpio_free(gpio_backlight);
//        msleep(2);
//        backlight_update_status(bl);
/* yaogangxiang@wind-mobi.com 2012.03.30 end */


#ifdef CONFIG_HAS_EARLYSUSPEND	
	pb->early_suspend_desc.level = 0; //EARLY_SUSPEND_LEVEL_BLANK_SCREEN
	pb->early_suspend_desc.suspend = tps61165_backlight_earlysuspend;
	pb->early_suspend_desc.resume = tps61165_backlight_earlyresume;
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

static int tps61165_backlight_remove(struct platform_device *pdev)
{
	struct platform_tps61165_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tps61165_bl_data *pb = dev_get_drvdata(&bl->dev);

        gpio_free(gpio_backlight);
	backlight_device_unregister(bl);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&pb->early_suspend_desc);
#endif
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

static struct platform_driver tps61165_backlight_driver = {
	.driver = {
		   .name = "tps61165-backlight",
		   .owner = THIS_MODULE,
		   },
	.probe = tps61165_backlight_probe,
	.remove = tps61165_backlight_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = tps61165_backlight_suspend,
	.resume = tps61165_backlight_resume,
#endif
};

static int __init tps61165_backlight_init(void)
{
	return platform_driver_register(&tps61165_backlight_driver);
}

late_initcall(tps61165_backlight_init);

static void __exit tps61165_backlight_exit(void)
{
	platform_driver_unregister(&tps61165_backlight_driver);
}

module_exit(tps61165_backlight_exit);

MODULE_DESCRIPTION("tps61165 Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps61165-backlight");
