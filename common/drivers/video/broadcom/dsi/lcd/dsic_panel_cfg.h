#ifndef __DSIC_PANEL_CFG_H__
#define __DSIC_PANEL_CFG_H__
/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/broadcom/dsi/lcd/dsic_panel_cfg.h
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
//xiongbiao@wind-mobi.com 2011.12.16 begin
//Add DIS LCD support.
#if defined(CONFIG_BOARD_PASCAL) 
#include "panel/dsi_hx8357c.h"  
//xiongbiao@wind-mobi.com 2011.12.16 end
//xiongbiao@wind-mobi.com 2012.01.12 begin
//Add hx8363b mipi lcd support in L400.
#elif defined(CONFIG_BOARD_L400) 
#include "panel/dsi_hx8363b.h" 
#else
//#include "panel/dsi_hx8369a.h"
#endif
//xiongbiao@wind-mobi.com 2012.01.12 end

#endif

