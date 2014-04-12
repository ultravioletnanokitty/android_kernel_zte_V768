/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/dsi/lcd/dsic.h
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

#ifndef __DSIC_H__
#define __DSIC_H__

#include <plat/csl/csl_lcd.h>               // LCD CSL Common Interface 
#include <plat/csl/csl_dsi.h>               // DSI CSL 
#include "display_drv.h"        // Disp Drv Commons
#include "dispdrv_mipi_dcs.h"      // MIPI DCS         
#include "dispdrv_mipi_dsi.h"      // MIPI DSI      

typedef struct
{
    UInt32              left;
    UInt32              right;
    UInt32              top;
    UInt32              bottom;
    UInt32              width;
    UInt32              height;
} lcd_drv_rect_t;

Int32    dsic_init(void);                                                                                                                ///< Routine to initialise the display driver
Int32    dsic_exit(void);                                                                                                                ///< Routine to shutdown the display driver
Int32    dsic_info(UInt32 *screenWidth, UInt32 *screenHeight, UInt32 *bpp, UInt32 *resetGpio);            ///< Routine to return a drivers info (name, version etc..)
#ifndef _BRCM_8BYTE_MSG_CONSTRAINT
Int32    dsic_open(const void* params, DISPDRV_HANDLE_T *handle,void *ptr);                                                                        ///< Routine to open a driver
#else
Int32    dsic_open(const void* params, DISPDRV_HANDLE_T *handle);                                                                        ///< Routine to open a driver
#endif	//_BRCM_8BYTE_MSG_CONSTRAINT
Int32    dsic_close(const DISPDRV_HANDLE_T handle);                                                                                      ///< Routine to close a driver
const DISPDRV_INFO_T* dsic_get_info(DISPDRV_HANDLE_T handle);                                                                            ///< Routine to get the display info
Int32    dsic_start(DISPDRV_HANDLE_T handle);                                                                                            ///< Routine to start a display
Int32    dsic_stop(DISPDRV_HANDLE_T handle);                                                                                             ///< Routine to stop a display
Int32    dsic_power_control(DISPDRV_HANDLE_T handle, DISPLAY_POWER_STATE_T powerState);                                                  ///< Routine to control a displays power
Int32    dsic_update(DISPDRV_HANDLE_T handle, int fb_idx, DISPDRV_CB_T apiCb);                                                                       ///< Routine to update a frame (INT fb)
Int32    dsic_dirty_rect_update (DISPDRV_HANDLE_T drvH, lcd_drv_rect_t* dirtyRect, DISPDRV_CB_T apiCb);					 ///<  Routine to update partial window
Int32    dsic_set_control(DISPDRV_HANDLE_T handle, DISPDRV_CTRL_ID_T controlID, void* controlParams);                                    ///< Routine to set control for the display panel
Int32    dsic_get_control(DISPDRV_HANDLE_T handle, DISPDRV_CTRL_ID_T controlID, void* controlParams);                                    ///< Routine to get control for the display panel
#ifdef CONFIG_BRCM_KPANIC_UI_IND
Int32    dsi_set_dev_border(DISPDRV_HANDLE_T drvH, int img_width, int img_height);          ///< Routime to update the border for the KPANIC Image display.
Int32    dsic_send_data (DISPDRV_HANDLE_T drvH, int fb_idx, DISPDRV_CB_T apiCb, int img_width, int img_height); ///< Routine to update the frame for the KPANIC Image display.
#endif
#endif
