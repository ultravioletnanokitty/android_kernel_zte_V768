/*
 * Generic ZI2848 backlight driver data - see drivers/video/backlight/zi2848_bl.c
 */
#ifndef __LINUX_tps61165_BACKLIGHT_H
#define __LINUX_tps61165_BACKLIGHT_H

struct platform_tps61165_backlight_data {
	unsigned int max_brightness;
	unsigned int dft_brightness;
	unsigned int gpio_lcd_backlight;
	int (*init)(struct device *dev);
	int (*notify)(int brightness);
	void (*exit)(struct device *dev);
};

#endif
