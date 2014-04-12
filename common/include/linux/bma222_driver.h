/*  $Date: 2010/07/29 17:19:00 $
 *  $Revision: 0.2 $ 
 */

/*
* Copyright (C) 2009 Bosch Sensortec GmbH
*
*	BMA222 linux driver ioctl() command define
* 
* Usage:	ioctl() COMMANDS of BMA222 driver by i2c for linux
*
* 
* Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in 
  compliance with the License and the following stipulations. The Apache License , Version 2.0 is applicable unless 
  otherwise stated by the stipulations of the disclaimer below. 
 
* You may obtain a copy of the License at 

    http://www.apache.org/licenses/LICENSE-2.0
  
 

Disclaimer 

 * Common:
 * This Work is developed for the consumer goods industry. It may only be used 
 * within the parameters of the respective valid product data sheet.  The Work 
 * provided with the express understanding that there is no warranty of fitness for a particular purpose. 
 * It is not fit for use in life-sustaining, safety or security sensitive systems or any system or device 
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition, 
 * the Work is not fit for use in products which interact with motor vehicle systems.  
 * The resale and/or use of the Work are at the purchasers own risk and his own responsibility. The 
 * examination of fitness for the intended use is the sole responsibility of the Purchaser. 
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for 
 * incidental, or consequential damages, arising from any Work or Derivative Work use not covered by the parameters of 
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch 
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased Work and Derivative Works, particularly with regard to 
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid 
 * technical specifications of the product series. They are therefore not intended or fit for resale to third 
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an 
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec 
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the 
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering 
 * samples.
 *
 * Special:
 * This Work and any related information (hereinafter called "Information") is provided free of charge 
 * for the sole purpose to support your application work. The Woek and Information is subject to the 
 * following terms and conditions: 
 *
 * The Work is specifically designed for the exclusive use for Bosch Sensortec products by 
 * personnel who have special experience and training. Do not use this Work or Derivative Works if you do not have the 
 * proper experience or training. Do not use this Work or Derivative Works fot other products than Bosch Sensortec products.  
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no 
 * responsibility for the consequences of use of such Information nor for any infringement of patents or 
 * other rights of third parties which may result from its use. No license is granted by implication or 
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are 
 * subject to change without notice.
 *
 */

/*! \file bma222_driver.h
    \brief This file contains all ioctl() commands define for the BMA222 in linux
    
    Details.
*/

#define BMA222_IOC_MAGIC 'B'

#define BMA222_SOFT_RESET				               _IO(BMA222_IOC_MAGIC,0)
#define BMA222_SET_RANGE			                   _IOWR(BMA222_IOC_MAGIC,1, unsigned char)
#define BMA222_GET_RANGE			                   _IOWR(BMA222_IOC_MAGIC,2, unsigned char)
#define BMA222_SET_MODE				                   _IOWR(BMA222_IOC_MAGIC,3, unsigned char)
#define BMA222_GET_MODE				                   _IOWR(BMA222_IOC_MAGIC,4, unsigned char)
#define BMA222_SET_BANDWIDTH		                   _IOWR(BMA222_IOC_MAGIC,5, unsigned char)
#define BMA222_GET_BANDWIDTH		                   _IOWR(BMA222_IOC_MAGIC,6, unsigned char)
#define BMA222_WRITE_REG                               _IOWR(BMA222_IOC_MAGIC,7, unsigned char)
#define BMA222_READ_REG                                _IOWR(BMA222_IOC_MAGIC,8, unsigned char)
#define BMA222_RESET_INTERRUPT		                   _IO(BMA222_IOC_MAGIC,9)
#define BMA222_READ_ACCEL_X			                   _IOWR(BMA222_IOC_MAGIC,10,short)
#define BMA222_READ_ACCEL_Y			                   _IOWR(BMA222_IOC_MAGIC,11,short)
#define BMA222_READ_ACCEL_Z			                   _IOWR(BMA222_IOC_MAGIC,12,short)
#define BMA222_READ_ACCEL_XYZ		                   _IOWR(BMA222_IOC_MAGIC,13,short)
#define BMA222_GET_INTERRUPTSTATUS1                    _IOWR(BMA222_IOC_MAGIC,16, unsigned char)
#define BMA222_GET_INTERRUPTSTATUS2                    _IOWR(BMA222_IOC_MAGIC,17, unsigned char)
#define BMA222_GET_LOW_G_INTERRUPT                     _IOWR(BMA222_IOC_MAGIC,18, unsigned char)
#define BMA222_GET_HIGH_G_INTERRUPT                    _IOWR(BMA222_IOC_MAGIC,19, unsigned char)
#define BMA222_GET_SLOPE_INTERRUPT                     _IOWR(BMA222_IOC_MAGIC,20, unsigned char)
#define BMA222_GET_DOUBLE_TAP_INTERRUPT                _IOWR(BMA222_IOC_MAGIC,21, unsigned char)
#define BMA222_GET_SINGLE_TAP_INTERRUPT                _IOWR(BMA222_IOC_MAGIC,22, unsigned char)
#define BMA222_GET_ORIENT_INTERRUPT                    _IOWR(BMA222_IOC_MAGIC,23, unsigned char)
#define BMA222_GET_FLAT_INTERRUPT                      _IOWR(BMA222_IOC_MAGIC,24, unsigned char)
#define BMA222_GET_DATA_INTERRUPT                      _IOWR(BMA222_IOC_MAGIC,25, unsigned char)
#define BMA222_GET_SLOPE_FIRST                         _IOWR(BMA222_IOC_MAGIC,26, unsigned char)
#define BMA222_GET_SLOPE_SIGN                          _IOWR(BMA222_IOC_MAGIC,27, unsigned char)
#define BMA222_GET_TAP_FIRST                           _IOWR(BMA222_IOC_MAGIC,28, unsigned char)
#define BMA222_GET_TAP_SIGN                            _IOWR(BMA222_IOC_MAGIC,29, unsigned char)
#define BMA222_GET_HIGH_FIRST                          _IOWR(BMA222_IOC_MAGIC,30, unsigned char)
#define BMA222_GET_HIGH_SIGN                           _IOWR(BMA222_IOC_MAGIC,31, unsigned char)
#define BMA222_GET_ORIENT_STATUS                       _IOWR(BMA222_IOC_MAGIC,32, unsigned char)
#define BMA222_GET_ORIENT_FLAT_STATUS                  _IOWR(BMA222_IOC_MAGIC,33, unsigned char)
#define BMA222_GET_SLEEP_DURATION                      _IOWR(BMA222_IOC_MAGIC,34, unsigned char)
#define BMA222_SET_SLEEP_DURATION                      _IOWR(BMA222_IOC_MAGIC,35, unsigned char)
#define BMA222_SET_SUSPEND                             _IOWR(BMA222_IOC_MAGIC,36, unsigned char)
#define BMA222_GET_SUSPEND                             _IOWR(BMA222_IOC_MAGIC,37, unsigned char)
#define BMA222_SET_LOWPOWER                            _IOWR(BMA222_IOC_MAGIC,38, unsigned char)
#define BMA222_GET_LOWPOWER_EN                         _IOWR(BMA222_IOC_MAGIC,39, unsigned char)
#define BMA222_SET_LOW_NOISE_CTRL                      _IOWR(BMA222_IOC_MAGIC,40, unsigned char)
#define BMA222_GET_LOW_NOISE_CTRL                      _IOWR(BMA222_IOC_MAGIC,41, unsigned char)
#define BMA222_SET_SHADOW_DISABLE                      _IOWR(BMA222_IOC_MAGIC,42, unsigned char)
#define BMA222_GET_SHADOW_DISABLE                      _IOWR(BMA222_IOC_MAGIC,43, unsigned char)
#define BMA222_SET_UNFILT_ACC                          _IOWR(BMA222_IOC_MAGIC,44, unsigned char)
#define BMA222_GET_UNFILT_ACC                          _IOWR(BMA222_IOC_MAGIC,45, unsigned char)
#define BMA222_SET_ENABLE_SLOPE_INTERRUPT              _IOWR(BMA222_IOC_MAGIC,46, unsigned char)
#define BMA222_GET_ENABLE_SLOPE_INTERRUPT              _IOWR(BMA222_IOC_MAGIC,47, unsigned char)
#define BMA222_SET_ENABLE_TAP_INTERRUPT                _IOWR(BMA222_IOC_MAGIC,14, unsigned char)
#define BMA222_GET_ENABLE_TAP_INTERRUPT                _IOWR(BMA222_IOC_MAGIC,48, unsigned char)
#define BMA222_SET_ENABLE_HIGH_G_INTERRUPT             _IOWR(BMA222_IOC_MAGIC,49, unsigned char)
#define BMA222_GET_ENABLE_HIGH_G_INTERRUPT             _IOWR(BMA222_IOC_MAGIC,50, unsigned char)
#define BMA222_SET_ENABLE_LOW_G_INTERRUPT              _IO(BMA222_IOC_MAGIC,51)
#define BMA222_GET_ENABLE_LOW_G_INTERRUPT              _IOWR(BMA222_IOC_MAGIC,52, unsigned char)
#define BMA222_SET_ENABLE_DATA_INTERRUPT               _IO(BMA222_IOC_MAGIC,53)
#define BMA222_GET_ENABLE_DATA_INTERRUPT               _IOWR(BMA222_IOC_MAGIC,54, unsigned char)
#define BMA222_SET_INT1_PAD_SEL                        _IOWR(BMA222_IOC_MAGIC,55, unsigned char)
#define BMA222_GET_INT1_PAD_SEL                        _IOWR(BMA222_IOC_MAGIC,56, unsigned char)
#define BMA222_SET_INT_DATA_SEL                        _IOWR(BMA222_IOC_MAGIC,57, unsigned char)
#define BMA222_GET_INT_DATA_SEL                        _IOWR(BMA222_IOC_MAGIC,58, unsigned char)
#define BMA222_SET_INT2_PAD_SEL                        _IOWR(BMA222_IOC_MAGIC,59, unsigned char)
#define BMA222_GET_INT2_PAD_SEL                        _IOWR(BMA222_IOC_MAGIC,60, unsigned char)
#define BMA222_SET_INT_SRC                             _IOWR(BMA222_IOC_MAGIC,61, unsigned char)
#define BMA222_GET_INT_SRC                             _IOWR(BMA222_IOC_MAGIC,62, unsigned char)
#define BMA222_SET_INT_SET                             _IOWR(BMA222_IOC_MAGIC,63, unsigned char)
#define BMA222_GET_INT_SET                             _IOWR(BMA222_IOC_MAGIC,64, unsigned char)
#define BMA222_GET_MODE_CTRL                           _IOWR(BMA222_IOC_MAGIC,65, unsigned char)
#define BMA222_SET_LOW_DURATION                        _IOWR(BMA222_IOC_MAGIC,66, unsigned char)
#define BMA222_GET_LOW_DURATION                        _IOWR(BMA222_IOC_MAGIC,67, unsigned char)
#define BMA222_SET_LOW_G_THRESHOLD                     _IOWR(BMA222_IOC_MAGIC,68, unsigned char)
#define BMA222_GET_LOW_G_THRESHOLD                     _IOWR(BMA222_IOC_MAGIC,69, unsigned char)
#define BMA222_SET_HIGH_G_DURATION                     _IOWR(BMA222_IOC_MAGIC,70, unsigned char)
#define BMA222_GET_HIGH_G_DURATION                     _IOWR(BMA222_IOC_MAGIC,71, unsigned char)
#define BMA222_SET_HIGH_G_THRESHOLD                    _IOWR(BMA222_IOC_MAGIC,72, unsigned char)
#define BMA222_GET_HIGH_G_THRESHOLD                    _IOWR(BMA222_IOC_MAGIC,73, unsigned char)
#define BMA222_SET_SLOPE_DURATION                      _IOWR(BMA222_IOC_MAGIC,74, unsigned char)
#define BMA222_GET_SLOPE_DURATION                      _IOWR(BMA222_IOC_MAGIC,75, unsigned char)
#define BMA222_SET_SLOPE_THRESHOLD                     _IOWR(BMA222_IOC_MAGIC,76, unsigned char)
#define BMA222_GET_SLOPE_THRESHOLD                     _IOWR(BMA222_IOC_MAGIC,77, unsigned char)
#define BMA222_SET_TAP_DURATION                        _IOWR(BMA222_IOC_MAGIC,78, unsigned char)
#define BMA222_GET_TAP_DURATION                        _IOWR(BMA222_IOC_MAGIC,79, unsigned char)
#define BMA222_SET_TAP_SHOCK                           _IOWR(BMA222_IOC_MAGIC,80, unsigned char)
#define BMA222_GET_TAP_SHOCK                           _IOWR(BMA222_IOC_MAGIC,81, unsigned char)
#define BMA222_SET_TAP_QUIET_DURATION                  _IOWR(BMA222_IOC_MAGIC,15, unsigned char)
#define BMA222_GET_TAP_QUIET                           _IOWR(BMA222_IOC_MAGIC,82, unsigned char)
#define BMA222_SET_TAP_THRESHOLD                       _IOWR(BMA222_IOC_MAGIC,83, unsigned char)
#define BMA222_GET_TAP_THRESHOLD                       _IOWR(BMA222_IOC_MAGIC,84, unsigned char)
#define BMA222_GET_TAP_SAMP                            _IOWR(BMA222_IOC_MAGIC,86, unsigned char)
#define BMA222_SET_TAP_SAMP                            _IOWR(BMA222_IOC_MAGIC,147, unsigned char)
#define BMA222_SET_ORIENT_MODE                         _IOWR(BMA222_IOC_MAGIC,85, unsigned char)
#define BMA222_GET_ORIENT_MODE                         _IOWR(BMA222_IOC_MAGIC,87, unsigned char)
#define BMA222_SET_ORIENT_BLOCKING                     _IOWR(BMA222_IOC_MAGIC,88, unsigned char)
#define BMA222_GET_ORIENT_BLOCKING                     _IOWR(BMA222_IOC_MAGIC,89, unsigned char)
#define BMA222_SET_ORIENT_HYST                         _IOWR(BMA222_IOC_MAGIC,90, unsigned char)
#define BMA222_GET_ORIENT_HYST                         _IOWR(BMA222_IOC_MAGIC,91, unsigned char)
#define BMA222_SET_THETA_BLOCKING                      _IOWR(BMA222_IOC_MAGIC,92, unsigned char)
#define BMA222_GET_THETA_BLOCKING                      _IOWR(BMA222_IOC_MAGIC,93, unsigned char)
#define BMA222_SET_ORIENT_EX                           _IOWR(BMA222_IOC_MAGIC,94, unsigned char)
#define BMA222_GET_ORIENT_EX                           _IOWR(BMA222_IOC_MAGIC,95, unsigned char)
#define BMA222_SET_THETA_FLAT                          _IOWR(BMA222_IOC_MAGIC,96, unsigned char)
#define BMA222_GET_THETA_FLAT                          _IOWR(BMA222_IOC_MAGIC,97, unsigned char)
#define BMA222_SET_FLAT_HOLD_TIME                      _IOWR(BMA222_IOC_MAGIC,98, unsigned char)
#define BMA222_GET_FLAT_HOLD_TIME                      _IOWR(BMA222_IOC_MAGIC,99, unsigned char)
#define BMA222_GET_LOW_POWER_STATE                     _IOWR(BMA222_IOC_MAGIC,100, unsigned char)
#define BMA222_SET_SELFTEST_ST                         _IOWR(BMA222_IOC_MAGIC,101, unsigned char)
#define BMA222_GET_SELFTEST_ST                         _IOWR(BMA222_IOC_MAGIC,102, unsigned char)
#define BMA222_SET_SELFTEST_STN                        _IOWR(BMA222_IOC_MAGIC,103, unsigned char)
#define BMA222_GET_SELFTEST_STN                        _IOWR(BMA222_IOC_MAGIC,104, unsigned char)
#define BMA222_SET_SELFTEST_ST_AMP                     _IOWR(BMA222_IOC_MAGIC,105, unsigned char)
#define BMA222_GET_SELFTEST_ST_AMP                     _IOWR(BMA222_IOC_MAGIC,106, unsigned char)
#define BMA222_SET_EE_W                                _IOWR(BMA222_IOC_MAGIC,107, unsigned char)
#define BMA222_GET_EE_W                                _IOWR(BMA222_IOC_MAGIC,108, unsigned char)
#define BMA222_SET_EE_PROG_TRIG                        _IO(BMA222_IOC_MAGIC,109)
#define BMA222_GET_EEPROM_WRITING_STATUS               _IOWR(BMA222_IOC_MAGIC,110, unsigned char)
#define BMA222_SET_UPDATE_IMAGE                        _IO(BMA222_IOC_MAGIC,111)
#define BMA222_SET_I2C_WDT_TIMER                       _IOWR(BMA222_IOC_MAGIC,112, unsigned char)
#define BMA222_GET_I2C_WDT_TIMER                       _IOWR(BMA222_IOC_MAGIC,113, unsigned char)
#define BMA222_SET_UNLOCK_TRIMMING_PART                _IOWR(BMA222_IOC_MAGIC,114, unsigned char)
#define BMA222_SET_HP_EN                               _IOWR(BMA222_IOC_MAGIC,115, unsigned char)
#define BMA222_GET_HP_EN                               _IOWR(BMA222_IOC_MAGIC,116, unsigned char)
#define BMA222_GET_CAL_READY                           _IOWR(BMA222_IOC_MAGIC,117, unsigned char)
#define BMA222_SET_CAL_TRIGGER                         _IOWR(BMA222_IOC_MAGIC,118, unsigned char)
#define BMA222_SET_OFFSET_RESET                        _IO(BMA222_IOC_MAGIC,119)
#define BMA222_SET_OFFSET_CUTOFF                       _IOWR(BMA222_IOC_MAGIC,120, unsigned char)
#define BMA222_GET_OFFSET_CUTOFF                       _IOWR(BMA222_IOC_MAGIC,121, unsigned char)
#define BMA222_SET_OFFSET_TARGET_X                     _IOWR(BMA222_IOC_MAGIC,122, unsigned char)
#define BMA222_GET_OFFSET_TARGET_X                     _IOWR(BMA222_IOC_MAGIC,123, unsigned char)
#define BMA222_SET_OFFSET_TARGET_Y                     _IOWR(BMA222_IOC_MAGIC,124, unsigned char)
#define BMA222_GET_OFFSET_TARGET_Y                     _IOWR(BMA222_IOC_MAGIC,125, unsigned char)
#define BMA222_SET_OFFSET_TARGET_Z                     _IOWR(BMA222_IOC_MAGIC,126, unsigned char)
#define BMA222_GET_OFFSET_TARGET_Z                     _IOWR(BMA222_IOC_MAGIC,127, unsigned char)
#define BMA222_SET_OFFSET_FILT_X                       _IOWR(BMA222_IOC_MAGIC,128, unsigned char)
#define BMA222_GET_OFFSET_FILT_X                       _IOWR(BMA222_IOC_MAGIC,129, unsigned char)
#define BMA222_SET_OFFSET_FILT_Y                       _IOWR(BMA222_IOC_MAGIC,130, unsigned char)
#define BMA222_GET_OFFSET_FILT_Y                       _IOWR(BMA222_IOC_MAGIC,131, unsigned char)
#define BMA222_SET_OFFSET_FILT_Z                       _IOWR(BMA222_IOC_MAGIC,132, unsigned char)
#define BMA222_GET_OFFSET_FILT_Z                       _IOWR(BMA222_IOC_MAGIC,133, unsigned char)
#define BMA222_SET_OFFSET_UNFILT_X                     _IOWR(BMA222_IOC_MAGIC,134, unsigned char)
#define BMA222_GET_OFFSET_UNFILT_X                     _IOWR(BMA222_IOC_MAGIC,135, unsigned char)
#define BMA222_SET_OFFSET_UNFILT_Y                     _IOWR(BMA222_IOC_MAGIC,136, unsigned char)
#define BMA222_GET_OFFSET_UNFILT_Y                     _IOWR(BMA222_IOC_MAGIC,137, unsigned char)
#define BMA222_SET_OFFSET_UNFILT_Z                     _IOWR(BMA222_IOC_MAGIC,138, unsigned char)
#define BMA222_GET_OFFSET_UNFILT_Z                     _IOWR(BMA222_IOC_MAGIC,139, unsigned char)
#define BMA222_SET_INT_MODE                            _IOWR(BMA222_IOC_MAGIC,140, unsigned char)
#define BMA222_GET_INT_MODE                            _IOWR(BMA222_IOC_MAGIC,141, unsigned char)
#define BMA222_SET_INT_ENABLE                          _IOWR(BMA222_IOC_MAGIC,142, unsigned char)
#define BMA222_WRITE_EE                                _IOWR(BMA222_IOC_MAGIC,143, unsigned char)
#define BMA222_SET_LOW_HY                              _IOWR(BMA222_IOC_MAGIC,144, unsigned char)
#define BMA222_SET_HIGH_HY                             _IOWR(BMA222_IOC_MAGIC,145, unsigned char)
#define BMA222_SET_LOW_MODE                            _IOWR(BMA222_IOC_MAGIC,146, unsigned char)
#define BMA222_GET_UPDATE_IMAGE_STATUS                 _IOWR(BMA222_IOC_MAGIC,148, unsigned char)
#define BMA222_CALIBRATION                             _IOWR(BMA222_IOC_MAGIC,149, short)

#define BMA222_IOC_MAXNR				150
