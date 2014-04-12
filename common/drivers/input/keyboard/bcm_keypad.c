/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/input/keyboard/bcm_keypad.c
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <linux/io.h>
#include <plat/bcm_keypad.h>

/*Debug messages */
/* #define       BCMKP_DEBUG */
#ifdef BCMKP_DEBUG
#define BCMKP_DBG(format, args...)	pr_info(__FILE__ ":" format, ## args)
#else
#define	BCMKP_DBG(format, args...)
#endif

#define BCM_KEY_REPEAT_PERIOD     100	/*repeat period (msec) */
#define BCM_KEY_REPEAT_DELAY      400	/*First time press delay (msec) */

#define KEYPAD_MAX_ROWS    8
#define KEYPAD_MAX_COLUMNS 8

#define MATRIX_SIZE               2	/* 32 bits */
#define U32_BITS                  32

#define KPDCR_MSK_COLFLT_S       8
#define KPDCR_MSK_ROWFLT_S       12
#define KPDCR_MSK_COLS_S         16
#define KPDCR_MSK_ROWS_S         20

#define KPD_ROWS(x)              ((x - 1) << KPDCR_MSK_ROWS_S)
#define KPD_COLS(x)              ((x - 1) << KPDCR_MSK_COLS_S)
#define KPD_ROWFLT(x)            ((x) << KPDCR_MSK_ROWFLT_S)
#define KPD_COLFLT(x)            ((x) << KPDCR_MSK_COLFLT_S)

#define KPDCR_FLT_1ms            0x0
#define KPDCR_FLT_2ms            0x1
#define KPDCR_FLT_4ms            0x2
#define KPDCR_FLT_8ms            0x3
#define KPDCR_FLT_16ms           0x4
#define KPDCR_FLT_32ms           0x5
#define KPDCR_FLT_64ms           0x6	/* default */
#define KPDCR_FLT_128ms          0x7

#define KPEMR_NO_TRIGGER           0x0
#define KPEMR_RISING_EDGE          0x1
#define KPEMR_FALLING_EDGE         0x2
#define KPEMR_BOTH_EDGE            0x3

/* ---- Constants and Types ---------------------------------------------- */
static void __iomem *bcm_keypad_base_addr;

#define REG_KEYPAD_KPCR     0x00
#define REG_KEYPAD_KPIOR    0x04

/* Special clone keypad registers, same bases as GPIO */
/* registers, but custom bit assignments for rows/columns. */
#define REG_KEYPAD_KPEMR0  0x10
#define REG_KEYPAD_KPEMR1  0x14
#define REG_KEYPAD_KPEMR2  0x18
#define REG_KEYPAD_KPEMR3  0x1C
#define REG_KEYPAD_KPSSR0  0x20
#define REG_KEYPAD_KPSSR1  0x24
#define REG_KEYPAD_KPIMR0  0x30
#define REG_KEYPAD_KPIMR1  0x34
#define REG_KEYPAD_KPICR0  0x38
#define REG_KEYPAD_KPICR1  0x3C
#define REG_KEYPAD_KPISR0  0x40
#define REG_KEYPAD_KPISR1  0x44

/* REG_KEYPAD_KPCR bits */
#define REG_KEYPAD_KPCR_ENABLE              0x00000001	/* Enable key pad control */
#define REG_KEYPAD_KPCR_PULL_UP             0x00000002
#define REG_KEYPAD_COLFILTER_EN             0x00000800	/* Enable column filtering */
#define REG_KEYPAD_STATFILTER_EN            0x00008000	/* Enable status filtering */

struct bcm_keypad {
	struct input_dev *input_dev;
	struct bcm_keymap *kpmap;
	spinlock_t bcm_kp_spin_Lock;
	unsigned int irq;	/* Device IRQ */
	unsigned int row_num;
	unsigned int col_num;
	unsigned int matrix[MATRIX_SIZE];
	unsigned int oldmatrix[MATRIX_SIZE];
};

static void bcm_keypad_tasklet(unsigned long);
static void bcm_handle_key_state(struct bcm_keypad *bcm_kb);
DECLARE_TASKLET_DISABLED(kp_tasklet, bcm_keypad_tasklet, 0);

//#define KEYPAD_BACKLIGHT_ON
#ifdef KEYPAD_BACKLIGHT_ON
#include <linux/gpio.h>

#define ACAR_KEYPAD_BL_GPIO	(24)
static void keypad_bl_control(struct work_struct *work)
{
	pr_info("keypad_bl_control\n");

	gpio_request(ACAR_KEYPAD_BL_GPIO, "acar_keypad_bl_led");
	gpio_direction_output(ACAR_KEYPAD_BL_GPIO, 1);
	gpio_free(ACAR_KEYPAD_BL_GPIO);
	msleep(2000);
	gpio_request(ACAR_KEYPAD_BL_GPIO, "acar_keypad_bl_led");
	gpio_direction_output(ACAR_KEYPAD_BL_GPIO, 0);
	gpio_free(ACAR_KEYPAD_BL_GPIO);
}

DECLARE_WORK(keypad_bl_work, keypad_bl_control);
static struct workqueue_struct *keypad_bl_workqueue;


#endif

/* ****************************************************************************** */
/* Function Name: bcm_keypad_interrupt */
/* Description: Interrupt handler, called whenever a keypress/release occur. */
/* ****************************************************************************** */

//heweimao@wind-mobi.com 20120212 begin
//L301 difference to external project
//reviewed by liubing@wind-mobi.com
#ifdef CONFIG_BOARD_PASCAL
static irqreturn_t bcm_keypad_interrupt(int irq, void *dev_id)
{
	struct bcm_keypad *bcm_kb = dev_id;
	unsigned int key_matrix[2],i,j, num_of_pressed_bits,bit_position ,key_code = 0;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kb->bcm_kp_spin_Lock, flags);

	disable_irq_nosync(irq);
	key_matrix[0] = ~readl(bcm_keypad_base_addr + REG_KEYPAD_KPSSR0);
	key_matrix[1] = ~readl(bcm_keypad_base_addr + REG_KEYPAD_KPSSR1);

	/* Multiple keys can be detected here.
	 * Count pressed-bits and if the number is over 2, ignore it 
     * Can be used to force phone to upload mode
	 */

    pr_info("old key_matrix[0]:0x08%x,[1]:0x08%x \n",bcm_kb->oldmatrix[0],bcm_kb->oldmatrix[1]);
    pr_info("new key_matrix[0]:0x08%x,[1]:0x08%x \n",key_matrix[0],key_matrix[1]);

    num_of_pressed_bits = 0;
    for(i=0;i<2;i++)
    {
        bit_position = 1;
        for(j=0;j<32;j++)
        {
            if(key_matrix[i] & bit_position) {
			num_of_pressed_bits++;
			/* Check Row & col values for Volume keys. Hit both to force a ramdump*/
				if(j/8 == 2 && j%8 == 6)
					 key_code = 0x1;
				if(j/8 == 3 && j%8 == 6)
					 key_code += 0x2;
				if(key_code == 0x3)
					 panic("Forced Ramdump !!\n");
			}	
            bit_position <<= 1;
        }
    }

	writel(0xFFFFFFFF, bcm_keypad_base_addr + REG_KEYPAD_KPICR0);
	writel(0xFFFFFFFF, bcm_keypad_base_addr + REG_KEYPAD_KPICR1);

	if(num_of_pressed_bits > 2)
    {
        spin_unlock_irqrestore(&bcm_kb->bcm_kp_spin_Lock, flags);
        enable_irq(irq);
        pr_info("the number of keys pressed is %d \n",num_of_pressed_bits);
    }
    else
    {
        bcm_kb->matrix[0] = key_matrix[0];
        bcm_kb->matrix[1] = key_matrix[1];
		
		tasklet_schedule(&kp_tasklet);
	
	#ifdef KEYPAD_BACKLIGHT_ON
	queue_work(keypad_bl_workqueue, &keypad_bl_work);
	#endif
	enable_irq(irq);
	spin_unlock_irqrestore(&bcm_kb->bcm_kp_spin_Lock, flags);
	}

	return IRQ_HANDLED;
}
#else
static irqreturn_t bcm_keypad_interrupt(int irq, void *dev_id)
{
	struct bcm_keypad *bcm_kb = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kb->bcm_kp_spin_Lock, flags);

	disable_irq_nosync(irq);
	bcm_kb->matrix[0] = ~readl(bcm_keypad_base_addr + REG_KEYPAD_KPSSR0);
	bcm_kb->matrix[1] = ~readl(bcm_keypad_base_addr + REG_KEYPAD_KPSSR1);

	writel(0xFFFFFFFF, bcm_keypad_base_addr + REG_KEYPAD_KPICR0);
	writel(0xFFFFFFFF, bcm_keypad_base_addr + REG_KEYPAD_KPICR1);

	tasklet_schedule(&kp_tasklet);

	#ifdef KEYPAD_BACKLIGHT_ON
	queue_work(keypad_bl_workqueue, &keypad_bl_work);
	#endif
	enable_irq(irq);
	spin_unlock_irqrestore(&bcm_kb->bcm_kp_spin_Lock, flags);
	return IRQ_HANDLED;
}
#endif
//heweimao@wind-mobi.com 20120212 end

/* ****************************************************************************** */
/* Function Name: bcm_handle_key_state */
/* Description: Find keys pressed, by comparing old and new state, */
/*              and conert scan key code to virtual keys. */
/* ****************************************************************************** */
static bool check_keypad_1_pressed;
static void bcm_handle_key_state(struct bcm_keypad *bcm_kb)
{

	int index = 0;
	struct bcm_keymap *keymap_p = bcm_kb->kpmap;
	for (index = 0; index < MATRIX_SIZE; index++) {
		u32 change = bcm_kb->matrix[index] ^ bcm_kb->oldmatrix[index];
		if (0 == change) {
			keymap_p += 32;
		} else {
			u32 mask;
			for (mask = 1; 0 != mask; mask <<= 1) {
				if (0 != (change & mask)) {
					unsigned char vk = keymap_p->key_code;
					if (0 != (bcm_kb->matrix[index] & mask)) {
						/* key press */
						input_report_key(bcm_kb->input_dev, vk, 1);	/*0 */
						input_sync(bcm_kb->input_dev);
						pr_debug("key press vk=%d\n",
							 vk);
						if (vk == 2)	/* press 1 */
							check_keypad_1_pressed =
							    1;
					} else {
						/* key release */
						input_report_key(bcm_kb->input_dev, vk, 0);	/*0 */
						input_sync(bcm_kb->input_dev);
						pr_debug("key release vk=%d\n",
							 vk);
					}
				}
				keymap_p++;
			}
		}
	}
}

/* ****************************************************************************** */
/* Function Name: bcm_keypad_check */
/* Description: The function is for the USB gadget to select the USB configuration. */
/* ****************************************************************************** */
bool bcm_keypad_1_check(void)
{
	return check_keypad_1_pressed;
}

/* ****************************************************************************** */
/* Function Name: bcm_set_keypad_register */
/* Description: configure keypad register. */
/*              Row is form row 0 to num_row. */
/*              Col is from col 0 to num_col . */
/* ****************************************************************************** */
static void bcm_set_keypad_register(unsigned int num_row, unsigned int num_col,
				    unsigned int type)
{

	int kpemr0 = 0, kpemr1 = 0, kpemr2 = 0, kpemr3 = 0, kpimr0 = 0, kpimr1 =
	    0, row, col;

	if ((num_row > KEYPAD_MAX_ROWS) || (num_col > KEYPAD_MAX_COLUMNS)) {
		num_row = KEYPAD_MAX_ROWS;
		num_col = KEYPAD_MAX_COLUMNS;
	}

	for (row = 0; row < num_row; row++) {
		if (row < 2) {
			for (col = 0; col < num_col; col++) {
				kpemr0 |= type << (col * 2 + (row % 2) * 16);
				kpimr0 |= 1 << (col + (row % 4) * 8);
			}
		} else if ((2 <= row) && (row < 4)) {
			for (col = 0; col < num_col; col++) {
				kpemr1 |= type << (col * 2 + (row % 2) * 16);
				kpimr0 |= 1 << (col + (row % 4) * 8);
			}
		} else if ((4 <= row) && (row < 6)) {
			for (col = 0; col < num_col; col++) {
				kpemr2 |= type << (col * 2 + (row % 2) * 16);
				kpimr1 |= 1 << (col + (row % 4) * 8);
			}
		} else if ((6 <= row) && (row < 8)) {
			for (col = 0; col < num_col; col++) {
				kpemr3 |= type << (col * 2 + (row % 2) * 16);
				kpimr1 |= 1 << (col + (row % 4) * 8);
			}
		}
	}
	writel(kpemr0 | readl(bcm_keypad_base_addr + REG_KEYPAD_KPEMR0),
	       bcm_keypad_base_addr + REG_KEYPAD_KPEMR0);
	writel(kpemr1 | readl(bcm_keypad_base_addr + REG_KEYPAD_KPEMR1)
	       , bcm_keypad_base_addr + REG_KEYPAD_KPEMR1);
	writel(kpemr2 | readl(bcm_keypad_base_addr + REG_KEYPAD_KPEMR2)
	       , bcm_keypad_base_addr + REG_KEYPAD_KPEMR2);
	writel(kpemr3 | readl(bcm_keypad_base_addr + REG_KEYPAD_KPEMR3)
	       , bcm_keypad_base_addr + REG_KEYPAD_KPEMR3);
	writel(kpimr0 | readl(bcm_keypad_base_addr + REG_KEYPAD_KPIMR0)
	       , bcm_keypad_base_addr + REG_KEYPAD_KPIMR0);
	writel(kpimr1 | readl(bcm_keypad_base_addr + REG_KEYPAD_KPIMR1)
	       , bcm_keypad_base_addr + REG_KEYPAD_KPIMR1);

	return;

}

/* ****************************************************************************** */
/* Function Name: bcm_keypad_probe */
/* Description: Called to perform module initialization when the module is loaded. */
/* ****************************************************************************** */
static int __devinit bcm_keypad_probe(struct platform_device *pdev)
{
	int ret;
	u32 i;
	u32 reg_value;
	struct bcm_keypad *bcm_kb;
	struct bcm_keypad_platform_info *pdata;
	struct bcm_keymap *keymap_p;

	BCMKP_DBG(KERN_NOTICE "bcm_keypad_probe\n");
	if (!pdev) {
		pr_err("%s(%s:%u)::Failed to get platform device\n",
		       __FUNCTION__, __FILE__, __LINE__);
		return -ENOMEM;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err("%s(%s:%u)::Failed to get platform data\n",
		       __FUNCTION__, __FILE__, __LINE__);
		return -ENOMEM;
	}

	keymap_p = pdata->keymap;
	bcm_keypad_base_addr = pdata->bcm_keypad_base;

	bcm_kb = kmalloc(sizeof(*bcm_kb), GFP_KERNEL);
	if (bcm_kb == NULL) {
		pr_err("%s(%s:%u)::Failed to allocate keypad structure...\n",
		       __FUNCTION__, __FILE__, __LINE__);
		return -ENOMEM;
	}
	memset(bcm_kb, 0, sizeof(*bcm_kb));

	bcm_kb->input_dev = input_allocate_device();
	if (bcm_kb->input_dev == NULL) {
		pr_err("%s(%s:%u)::Failed to allocate input device...\n",
		       __FUNCTION__, __FILE__, __LINE__);
		kfree(bcm_kb);
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, bcm_kb);
	bcm_kb->kpmap = pdata->keymap;

	/* Setup input device */
	set_bit(EV_KEY, bcm_kb->input_dev->evbit);
	set_bit(EV_REP, bcm_kb->input_dev->evbit);
	bcm_kb->input_dev->rep[REP_PERIOD] = BCM_KEY_REPEAT_PERIOD;	/* repeat period */
	bcm_kb->input_dev->rep[REP_DELAY] = BCM_KEY_REPEAT_DELAY;	/* fisrt press delay */

	bcm_kb->input_dev->name = "bcm-keypad_v2";
	bcm_kb->input_dev->phys = "keypad/input0";
	bcm_kb->input_dev->id.bustype = BUS_HOST;
	bcm_kb->input_dev->id.vendor = 0x0001;
	bcm_kb->input_dev->id.product = 0x0001;
	bcm_kb->input_dev->id.version = 0x0100;

	memset(bcm_kb->matrix, 0, sizeof(bcm_kb->matrix));

	bcm_kb->row_num = pdata->row_num;	/* KPD_ROW_NUM ; */
	bcm_kb->col_num = pdata->col_num;	/* KPD_COL_NUM ; */
	spin_lock_init(&bcm_kb->bcm_kp_spin_Lock);
	pdata->iocr_cfg(bcm_kb->row_num, bcm_kb->col_num);
	tasklet_enable(&kp_tasklet);
	kp_tasklet.data = (unsigned long)bcm_kb;

	bcm_kb->irq = IRQ_KEYPAD;

	#ifdef KEYPAD_BACKLIGHT_ON
	keypad_bl_workqueue = create_singlethread_workqueue("keypad backlight");
	#endif

	pr_debug("%s::bcm_keypad_probe\n", __FUNCTION__);

	/*KPIOR: set rows as outputs, columns as inputs */
	writel(0xff000000, bcm_keypad_base_addr + REG_KEYPAD_KPIOR);

	reg_value = REG_KEYPAD_KPCR_PULL_UP |
	    REG_KEYPAD_COLFILTER_EN |
	    REG_KEYPAD_STATFILTER_EN | KPD_ROWFLT(KPDCR_FLT_32ms) |
	    KPD_COLFLT(KPDCR_FLT_32ms) | KPD_ROWS(bcm_kb->row_num) |
	    KPD_COLS(bcm_kb->col_num);

	/* First enable the Keypad Control and then configure the remaining values */
	writel(REG_KEYPAD_KPCR_ENABLE, bcm_keypad_base_addr + REG_KEYPAD_KPCR);

	reg_value |= readl(bcm_keypad_base_addr + REG_KEYPAD_KPCR);
	writel(reg_value, bcm_keypad_base_addr + REG_KEYPAD_KPCR);

	/* Clear the interrupt source */
	writel(0xFFFFFFFF, bcm_keypad_base_addr + REG_KEYPAD_KPICR0);
	writel(0xFFFFFFFF, bcm_keypad_base_addr + REG_KEYPAD_KPICR1);

	bcm_set_keypad_register(bcm_kb->row_num, bcm_kb->col_num,
				KPEMR_BOTH_EDGE);

	for (i = 0; i < (KEYPAD_MAX_ROWS * KEYPAD_MAX_COLUMNS); i++) {
		__set_bit(keymap_p->key_code & KEY_MAX,
			  bcm_kb->input_dev->keybit);
		keymap_p++;
	}

	ret =
	    request_irq(bcm_kb->irq, bcm_keypad_interrupt, IRQF_DISABLED |
			IRQF_NO_SUSPEND, "BRCM Keypad", bcm_kb);
	if (ret < 0) {
		pr_err("%s(%s:%u)::request_irq failed IRQ %d\n",
		       __FUNCTION__, __FILE__, __LINE__, bcm_kb->irq);
		goto free_irq;
	}
	ret = input_register_device(bcm_kb->input_dev);
	if (ret < 0) {
		pr_err
		    ("%s(%s:%u)::Unable to register GPIO-keypad input device\n",
		     __FUNCTION__, __FILE__, __LINE__);
		goto free_dev;
	}

	/* Initialization Finished */
	BCMKP_DBG(KERN_DEBUG "BCM keypad initialization completed...\n");
	return ret;

      free_dev:
	input_unregister_device(bcm_kb->input_dev);
	input_free_device(bcm_kb->input_dev);

      free_irq:
	free_irq(bcm_kb->irq, (void *)bcm_kb);

	return -EINVAL;
}

/* ****************************************************************************** */
/* Function Name: bcm_keypad_remove */
/* Description: Called to perform module cleanup when the module is unloaded. */
/* ****************************************************************************** */
static int __devexit bcm_keypad_remove(struct platform_device *pdev)
{
	struct bcm_keypad *bcm_kb = platform_get_drvdata(pdev);
	BCMKP_DBG(KERN_NOTICE "bcm_keypad_remove\n");

	/* disable keypad interrupt handling */
	tasklet_disable(&kp_tasklet);

	/*disable keypad interrupt handling */
	free_irq(bcm_kb->irq, (void *)bcm_kb);

	#ifdef KEYPAD_BACKLIGHT_ON
	flush_workqueue(keypad_bl_workqueue);
	destroy_workqueue(keypad_bl_workqueue);
	#endif

	/* unregister everything */
	input_unregister_device(bcm_kb->input_dev);
	input_free_device(bcm_kb->input_dev);

	return 0;
}

/* ****************************************************************************** */
/* Function Name: bcm_keypad_tasklet */
/* Description: tasklet for a keypress/release occur. . */
/* ****************************************************************************** */

static void bcm_keypad_tasklet(unsigned long data)
{

	/* Use SSR to see which key be pressed */
	struct bcm_keypad *bcm_kb = (struct bcm_keypad *)data;

	pr_debug("ss0=%x;ss1=%x\n", bcm_kb->matrix[0], bcm_kb->matrix[1]);

	bcm_handle_key_state(bcm_kb);

	/* New state become old */
	memcpy(bcm_kb->oldmatrix, bcm_kb->matrix, sizeof(bcm_kb->matrix));
	/* Get new state for scan key table */
	memset(bcm_kb->matrix, 0, sizeof(bcm_kb->matrix));

}

/****************************************************************************/

struct platform_driver bcm_keypad_device_driver = {
	.probe = bcm_keypad_probe,
	.remove = __devexit_p(bcm_keypad_remove),
	.driver = {
		   .name = "bcm_keypad",
		   }
};

static int __init bcm_keypad_init(void)
{
	pr_info("bcm_keypad_device_driver\n");
	return platform_driver_register(&bcm_keypad_device_driver);
}

static void __exit bcm_keypad_exit(void)
{
	platform_driver_unregister(&bcm_keypad_device_driver);
}

module_init(bcm_keypad_init);
module_exit(bcm_keypad_exit);

MODULE_AUTHOR("Broadcom Corportaion");
MODULE_DESCRIPTION("BCM Keypad Driver");
MODULE_LICENSE("GPL");
