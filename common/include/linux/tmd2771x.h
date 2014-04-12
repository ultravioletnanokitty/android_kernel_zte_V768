
/*
 *  tmd2771x.h - Texas Advanced Optoelectronic Solutions Inc.
 *              Proximity/Ambient light sensor
 *
 *  Copyright (c) 2010 Samsung Eletronics
 *  Donggeun Kim <dg77....@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TMD2771X_H_
#define _TMD2771X_H_

#define TMD2771X_ENABLE                                0x00
#define TMD2771X_ATIME                         0x01
#define TMD2771X_PTIME                         0x02
#define TMD2771X_WTIME                         0x03
#define TMD2771X_AILTL                         0x04
#define TMD2771X_AILTH                         0x05
#define TMD2771X_AIHTL                         0x06
#define TMD2771X_AIHTH                         0x07
#define TMD2771X_PILTL                         0x08
#define TMD2771X_PILTH                         0x09
#define TMD2771X_PIHTL                         0x0a
#define TMD2771X_PIHTH                         0x0b
#define TMD2771X_PERS                          0x0c
#define TMD2771X_CONFIG                                0x0d
#define TMD2771X_PPCOUNT                       0x0e
#define TMD2771X_CONTROL                       0x0f
#define TMD2771X_ID                            0x12
#define TMD2771X_STATUS                                0x13
#define TMD2771X_CH0DATAL                      0x14
#define TMD2771X_CH0DATAH                      0x15
#define TMD2771X_CH1DATAL                      0x16
#define TMD2771X_CH1DATAH                      0x17
#define TMD2771X_PDATAL                                0x18
#define TMD2771X_PDATAH                                0x19

#define TMD2771X_PIEN_SHIFT                    (5)
#define TMD2771X_PIEN                          (0x1 << TMD2771X_PIEN_SHIFT)
#define TMD2771X_AIEN_SHIFT                    (4)
#define TMD2771X_AIEN                          (0x1 << TMD2771X_AIEN_SHIFT)
#define TMD2771X_WEN_SHIFT                     (3)
#define TMD2771X_WEN                           (0x1 << TMD2771X_WEN_SHIFT)
#define TMD2771X_PEN_SHIFT                     (2)
#define TMD2771X_PEN                           (0x1 << TMD2771X_PEN_SHIFT)
#define TMD2771X_AEN_SHIFT                     (1)
#define TMD2771X_AEN                           (0x1 << TMD2771X_AEN_SHIFT)
#define TMD2771X_PON_SHIFT                     (0)
#define TMD2771X_PON                           (0x1 << TMD2771X_PON_SHIFT)

#define TMD2771X_PPERS_SHIFT                   (4)
#define TMD2771X_PPERS_MASK                    (0xf << TMD2771X_PPERS_SHIFT)
#define TMD2771X_APERS_SHIFT                   (0)
#define TMD2771X_APERS_MASK                    (0xf << TMD2771X_APERS_SHIFT)

#define TMD2771X_WLONG_SHIFT                   (1)
#define TMD2771X_WLONG                         (0x1 << TMD2771X_WLONG_SHIFT)

#define TMD2771X_PDRIVE_SHIFT                  (6)
#define TMD2771X_PDRIVE_100MA                  (0x0 << TMD2771X_PDRIVE_SHIFT)
#define TMD2771X_PDRIVE_50MA                   (0x1 << TMD2771X_PDRIVE_SHIFT)
#define TMD2771X_PDRIVE_25MA                   (0x2 << TMD2771X_PDRIVE_SHIFT)
#define TMD2771X_PDRIVE_12MA                   (0x3 << TMD2771X_PDRIVE_SHIFT)
#define TMD2771X_PDIODE_SHIFT                  (4)
#define TMD2771X_PDIODE_CH0_DIODE              (0x1 << TMD2771X_PDIODE_SHIFT)
#define TMD2771X_PDIODE_CH1_DIODE              (0x2 << TMD2771X_PDIODE_SHIFT)
#define TMD2771X_PDIODE_BOTH_DIODE             (0x3 << TMD2771X_PDIODE_SHIFT)
#define TMD2771X_AGAIN_SHIFT                   (0)
#define TMD2771X_AGAIN_1X                      (0x0 << TMD2771X_AGAIN_SHIFT)
#define TMD2771X_AGAIN_8X                      (0x1 << TMD2771X_AGAIN_SHIFT)
#define TMD2771X_AGAIN_16X                     (0x2 << TMD2771X_AGAIN_SHIFT)
#define TMD2771X_AGAIN_120X                    (0x3 << TMD2771X_AGAIN_SHIFT)

#define TMD2771X_PINT_SHIFT                    (5)
#define TMD2771X_PINT                          (0x1 << TMD2771X_PINT_SHIFT)
#define TMD2771X_AINT_SHIFT                    (4)
#define TMD2771X_AINT                          (0x1 << TMD2771X_AINT_SHIFT)
#define TMD2771X_AVALID_SHIFT                  (0)
#define TMD2771X_AVALID                        (0x1 << TMD2771X_AVALID_SHIFT)

#define TMD2771X_8BIT_MASK                     (0xff)

#define TMD2771X_DEFAULT_COMMAND               0x80
#define TMD2771X_AUTO_INCREMENT_COMMAND                0xa0
#define TMD2771X_PS_INT_CLEAR_COMMAND          0xe5
#define TMD2771X_ALS_INT_CLEAR_COMMAND         0xe6
#define TMD2771X_PS_ALS_INT_CLEAR_COMMAND      0xe7

#define TMD2771X_POWERUP_WAIT_TIME             12

struct tmd2771x_platform_data {
       /* xuhuashan@gmail.com 2011.06.18 begin */
       /* As bcm1161x i2c bus driver requires, first componment
        * must be i2c_slave_platform_data. */
       struct i2c_slave_platform_data  i2c_pdata;
       /* xuhuashan@gmail.com 2011.06.18 end */

       void (*control_power_source)(int);

       u8 power_on;                    /* TMD2771X_PON   */
       u8 wait_enable;                 /* TMD2771X_WEN  */
       u8 wait_long;                   /* TMD2771X_WLONG */
       u8 wait_time;                   /* 0x00 ~ 0xff  */

       /* Proximity */
       u8 ps_enable;                   /* TMD2771X_PEN */
       u16 ps_interrupt_h_thres;       /* 0x0000 ~ 0xffff */
       u16 ps_interrupt_l_thres;       /* 0x0000 ~ 0xffff */
       u8 ps_interrupt_enable;         /* TMD2771X_PIEN */
       u8 ps_time;                     /* 0x00 ~ 0xff  */
       u8 ps_interrupt_persistence;    /* 0x00 ~ 0xf0 (top four bits) */
       u8 ps_pulse_count;              /* 0x00 ~ 0xff  */
       u8 ps_drive_strength;           /* TMD2771X_PDRIVE_12MA ~
                                          TMD2771X_PDRIVE_100MA */
       u8 ps_diode;                    /* TMD2771X_PDIODE_CH0_DIODE ~
                                          TMD2771X_PDIODE_BOTH_DIODE */

       /* Ambient Light */
       u8 als_enable;                  /* TMD2771X_AEN */
       u16 als_interrupt_h_thres;      /* 0x0000 ~ 0xffff */
       u16 als_interrupt_l_thres;      /* 0x0000 ~ 0xffff */
       u8 als_interrupt_enable;        /* TMD2771X_AIEN */
       u8 als_time;                    /* 0x00 ~ 0xff */
       u8 als_interrupt_persistence;   /* 0x00 ~ 0x0f (bottom four bits) */
       u8 als_gain;                    /* TMD2771X_AGAIN_1X ~
                                          TMD2771X_AGAIN_120X */
       u8 glass_attenuation;
//xiongbiao@wind-mobi.com 2011.12.22 begin
//add gpio init and exit
		int (*init) (void);
		void (*exit) (struct device *);		
//xiongbiao@wind-mobi.com 2011.12.22 begin
};

#endif


