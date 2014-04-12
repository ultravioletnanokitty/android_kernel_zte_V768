/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
*             @file     drivers/video/broadcom/dsi/dsic.c
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
#include <linux/string.h>
#include <plat/mobcom_types.h>


#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <plat/osabstract/ostask.h>


#include <plat/dma_drv.h>
#include "display_drv.h"           // display driver interface
#include <plat/osdal_os_driver.h>

#include <linux/gpio.h>
#include <plat/syscfg.h>

#include "dsic.h"
#include "dsic_panel_cfg.h"


typedef struct
{
    CSL_LCD_HANDLE      clientH;        // DSI Client Handle
    CSL_LCD_HANDLE      dsiCmVcHandle;  // DSI CM VC Handle
    DISP_DRV_STATE      drvState;
    DISP_PWR_STATE      pwrState;
    UInt32              busId;
    UInt32              teIn;
    UInt32              teOut;
    lcd_drv_rect_t      win;
    UInt32 		row_start;
    UInt32 		row_end;
    UInt32 		col_start;
    UInt32 		col_end;
    void*               pFb;
    void*               pFbA;
} dsic_panel_t;   


// LOCAL FUNCTIONs
static int      dsi_ioctlrd( 
                    DISPDRV_HANDLE_T        drvH, 
                    DISPDRV_CTRL_RW_REG*   acc );
                    
static void     dsi_ioctlwr( 
                    DISPDRV_HANDLE_T        drvH, 
                    DISPDRV_CTRL_RW_REG*   acc );

static void     dsi_wrcmndP0  ( 
                    DISPDRV_HANDLE_T        drvH, 
                    UInt32                  reg );
                    
static void     dsi_wrcmndP1 ( 
                    DISPDRV_HANDLE_T        drvH, 
                    UInt32                  reg,
                    UInt32                  val );

static int dsi_update_column(dsic_panel_t * dev,
                        unsigned int         column,
                        DISPDRV_CB_T        apiCb   );


//#define printk(format, arg...)	do {} while (0)

static  dsic_panel_t  panel[2];
static DISPDRV_CB_T dispdrv_cb = NULL;

//#############################################################################




//*****************************************************************************
//
// Function Name: dsi_teon
// 
// Description:   Configure TE Input Pin & Route it to DSI Controller Input
//
//*****************************************************************************
static int dsi_teon ( dsic_panel_t *pPanel )
{
    Int32       res = 0;
    
	board_sysconfig(SYSCFG_LCD, SYSCFG_ENABLE);
	
    return ( res );
}

//*****************************************************************************
//
// Function Name: dsi_teoff
// 
// Description:   'Release' TE Input Pin Used
//
//*****************************************************************************
static int dsi_teoff ( dsic_panel_t *pPanel )
{
    Int32  res = 0;
    return ( res );
}


//*****************************************************************************
//
// Function Name:  dsi_wrcmndPN
// 
// Parameters:     reg   = 08-bit register address (DCS command)
//                 value = 08-bit register data    (DCS command parm)
//
// Description:    Register Write - DCS command byte, 1 parm
//
//*****************************************************************************
static void dsi_wrcmndPN( 
    DISPDRV_HANDLE_T    drvH, 
    UInt32              reg, 
    UInt32				datasize,
    UInt8*              dataptr    
    )
{
#ifdef _BRCM_8BYTE_MSG_CONSTRAINT
	if(datasize <8)
	{//datasize <=7
	
	    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
	    CSL_DSI_CMND_t      msg;
	    UInt8               msgData [(datasize+1)];
		int index = 0;

	    msg.dsiCmnd    = DSI_DT_LG_DCS_WR;
	    msg.msg        = &msgData[0];
	    msg.msgLen     = datasize+1;
	    msg.vc         = DSI_VC;
	    msg.isLP       = DSI_CMND_IS_LP;
	    msg.isLong     = TRUE;
	    msg.endWithBta = FALSE;

	    msgData[0] = reg;  
		for(index = 0; index < datasize; index++)
		{
			msgData[index+1] = dataptr[index] & 0x000000FF;  
		}
	    
	    CSL_DSI_SendPacket (pPanel->clientH, &msg);
	}
	else
	{//TODO:following codes is just for HX8369A, it need to be replaced with DATA_CMD_FIFO+PIXEL_FIFO method
		dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
	    CSL_DSI_CMND_t      msg;
	    UInt8               msgData [8];
		int index = 0;

		int sendindex[10];
		int sendsize[10];
		int sendcounter = 0;
		int temp = 0;
		
		sendindex[0]=0;
		sendsize[0]=5;

		index += 5;
		sendcounter = 1;
		//calculate size
		while(index < datasize)
		{
			if((datasize-index)<6)
			{
				sendsize[sendcounter]=(datasize-index);
				sendcounter++;
				break;
			}
			else
			{
				sendsize[sendcounter]=6;
				index += 6;
			}

			sendcounter++;			
		}

		//calculate index
		for(index = 1; index < sendcounter; index++)
		{
			temp += sendsize[index-1];
			sendindex[index]=temp-1;
		}

		for(temp = 0; temp < sendcounter; temp++)
		{
		    int size = (temp == 0) ? sendsize[temp]: (sendsize[temp]+1);
		    msg.dsiCmnd    = DSI_DT_LG_DCS_WR;
		    msg.msg        = &msgData[0];
			if(temp == 0)
	    		msg.msgLen     = sendsize[temp]+1;
			else
		    	msg.msgLen     = sendsize[temp]+2;			
	    	msg.vc         = DSI_VC;
	    	msg.isLP       = DSI_CMND_IS_LP;
	    	msg.isLong     = TRUE;
	    	msg.endWithBta = FALSE;

			if(temp == 0)
		    	msgData[0] = reg; 
			else
				msgData[0] = 0xFD; 
			for(index = 0; index < size; index++)
			{
				msgData[index+1] = dataptr[sendindex[temp]+index] & 0x000000FF;  
			}
	    
	    	CSL_DSI_SendPacket (pPanel->clientH, &msg);
		}
	}
#else
	    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
	    CSL_DSI_CMND_t      msg;
	    UInt8               msgData[(datasize + 1)];
	    int index = 0;

	    msg.dsiCmnd    = DSI_DT_LG_DCS_WR;
	    msg.msg        = &msgData[0];
	    msg.msgLen     = datasize+1;
	    msg.vc         = DSI_VC;
	    msg.isLP       = DSI_CMND_IS_LP;
	    msg.isLong     = TRUE;
	    msg.endWithBta = FALSE;

	    //printk("[dsi_wrcmndPN]msgLen:%d\n",msg.msgLen);
	    msgData[0] = reg;
	    for(index = 0; index < datasize; index++)
	    {
		msgData[index+1] = dataptr[index] & 0x000000FF;
	    }

	    CSL_DSI_SendPacket (pPanel->clientH, &msg);
#endif	//_BRCM_8BYTE_MSG_CONSTRAINT
}



static void dsi_set_max_ret_pkt_size( 
    DISPDRV_HANDLE_T    drvH, 
    UInt32              reg 
    )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
    CSL_DSI_CMND_t      msg;
    UInt8               msgData[4];
    
    msg.dsiCmnd    = DSI_DT_SH_MAX_RET_PKT_SIZE;
    msg.msg        = &msgData[0];
    msg.msgLen     = 2;
    msg.vc         = DSI_VC;
    msg.isLP       = DSI_CMND_IS_LP;
    msg.isLong     = FALSE;
    msg.endWithBta = FALSE;

    msgData[0] = 2;                                  
    msgData[1] = 0;   
    
    CSL_DSI_SendPacket (pPanel->clientH, &msg);   
}


//*****************************************************************************
//
// Function Name:  dsi_wrcmndP1
// 
// Parameters:     reg   = 08-bit register address (DCS command)
//                 value = 08-bit register data    (DCS command parm)
//
// Description:    Register Write - DCS command byte, 1 parm
//
//*****************************************************************************
static void dsi_wrcmndP1( 
    DISPDRV_HANDLE_T    drvH, 
    UInt32              reg, 
    UInt32              value 
    )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
    CSL_DSI_CMND_t      msg;
    UInt8               msgData[4];
    
    msg.dsiCmnd    = DSI_DT_SH_DCS_WR_P1;
    msg.msg        = &msgData[0];
    msg.msgLen     = 2;
    msg.vc         = DSI_VC;
    msg.isLP       = DSI_CMND_IS_LP;
    msg.isLong     = FALSE;
    msg.endWithBta = FALSE;

    msgData[0] = reg;                                  
    msgData[1] = value & 0x000000FF;   
    
    CSL_DSI_SendPacket (pPanel->clientH, &msg);   
}

//*****************************************************************************
//
// Function Name:  dsi_wrcmndP0
// 
// Parameters:     reg   = 08-bit register address (DCS command)
//
// Description:    Register Write - DCS command byte, 0 parm 
//
//*****************************************************************************
static void dsi_wrcmndP0( 
    DISPDRV_HANDLE_T    drvH, 
    UInt32              reg 
    )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
    CSL_DSI_CMND_t      msg;
    UInt8               msgData[4];
    
    msg.dsiCmnd    = DSI_DT_SH_DCS_WR_P0;
    msg.msg        = &msgData[0];
    msg.msgLen     = 2;
    msg.vc         = DSI_VC;
    msg.isLP       = DSI_CMND_IS_LP;
    msg.isLong     = FALSE;
    msg.endWithBta = FALSE;

    msgData[0] = reg;                                  
    msgData[1] = 0;   
    
    CSL_DSI_SendPacket (pPanel->clientH, &msg);   
}

//*****************************************************************************
//
// Function Name:  dsi_ioctlwr
// 
// Parameters:     
//
// Description:    IOCTL WR Test Code - DCS Wr With P0(no parm) or P1(1 parm)
//
//*****************************************************************************
static void dsi_ioctlwr( 
    DISPDRV_HANDLE_T       drvH,
    DISPDRV_CTRL_RW_REG*   acc 
    )
{
    if( acc->parmCount == 1 )
    { 
        dsi_wrcmndP0 ( drvH, acc->cmnd );
        LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: DSC+P0 "
            "DCS[0x%08X]\n\r", __FUNCTION__, (unsigned int)acc->cmnd );
    }
    else if( acc->parmCount == 2 )
    {
        dsi_wrcmndP1 ( drvH, acc->cmnd, *((UInt8*)acc->pBuff) );
        LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: DSC+P1 "
            "DCS[0x%08X] P[0x%08X]\n\r", __FUNCTION__, 
            (unsigned int)acc->cmnd, (unsigned int)*((UInt8*)acc->pBuff) );
    }
    else
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] dsi_ioctlwr: "
            "Only DCS with 0|1 Parm Supported\n" );
    }        
} // dsi_ioctlwr   

     
//*****************************************************************************
//
// Function Name:  dsi_ioctlrd
// 
// Parameters:     
//
// Description:    IOCTL RD Test Code - DCS Rd
//
//*****************************************************************************
static int dsi_ioctlrd( 
    DISPDRV_HANDLE_T       drvH,
    DISPDRV_CTRL_RW_REG*   acc 
    )
{
    dsic_panel_t  *pPanel = (dsic_panel_t *)drvH;
    CSL_DSI_CMND_t      msg;         
    CSL_DSI_REPLY_t     rxMsg;
    UInt8               txData[1];  // DCS Rd Command
    UInt32              reg;
    UInt8 *             pRxBuff = (UInt8*)acc->pBuff;
    Int32               res = 0;
    CSL_LCD_RES_T       cslRes;
    
    memset( (void*)&rxMsg, 0, sizeof(CSL_DSI_REPLY_t) );
    
    rxMsg.pReadReply = pRxBuff;
    
    msg.dsiCmnd    = DSI_DT_SH_DCS_RD_P0;
    msg.msg        = &txData[0];
    msg.msgLen     = 1;
    msg.vc         = DSI_VC;
    msg.isLP       = DSI_CMND_IS_LP;
    msg.isLong     = FALSE;
    msg.endWithBta = TRUE;
    msg.reply      = &rxMsg;

    txData[0] = acc->cmnd;                                    
    cslRes = CSL_DSI_SendPacket ( pPanel->clientH, &msg );
    
    if( cslRes != CSL_LCD_OK )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERR"
            "Reading From Reg[0x%08X]\n\r", __FUNCTION__, (unsigned int)acc->cmnd );
        res = -1;    
    }
    else
    {
        reg = pRxBuff[0];
    
        LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: Reg[0x%08X] "
            "Value[0x%08X]\n\r", __FUNCTION__, (unsigned int)acc->cmnd, (unsigned int)reg );
#if 0
        LCD_DBG ( LCD_DBG_INIT_ID, "   TYPE    : %s\n"    , 
            DISPDRV_dsiCslRxT2text (rxMsg.type,dsiE) );     
#endif

        if( rxMsg.type & DSI_RX_TYPE_TRIG )
        {
            LCD_DBG ( LCD_DBG_INIT_ID, "   TRIG    : 0x%08X\n", (unsigned int)rxMsg.trigger );
        }
        
        if( rxMsg.type & DSI_RX_TYPE_READ_REPLY )
        {
#if 0
            LCD_DBG ( LCD_DBG_INIT_ID, "   RD DT   : %s\n"    , 
                DISPDRV_dsiRxDt2text(rxMsg.readReplyDt,dsiE) );     
#endif
            LCD_DBG ( LCD_DBG_INIT_ID, "   RD STAT : 0x%08X\n", (unsigned int)rxMsg.readReplyRxStat );
            LCD_DBG ( LCD_DBG_INIT_ID, "   RD SIZE : %d\n"    , rxMsg.readReplySize );
            LCD_DBG ( LCD_DBG_INIT_ID, "   RD BUFF : 0x%02X 0x%02X 0x%02X 0x%02X "
                                   "0x%02X 0x%02X 0x%02X 0x%02X\n", 
                pRxBuff[0], pRxBuff[1],  pRxBuff[2], pRxBuff[3],
                pRxBuff[4], pRxBuff[5],  pRxBuff[6], pRxBuff[7] );
        }       
                                            
        if( rxMsg.type & DSI_RX_TYPE_ERR_REPLY )
        {
#if 0
            LCD_DBG ( LCD_DBG_INIT_ID, "   ERR DT  : %s\n"    , 
                DISPDRV_dsiRxDt2text (rxMsg.errReportDt,dsiE) );     
#endif
            LCD_DBG ( LCD_DBG_INIT_ID, "   ERR STAT: 0x%08X\n", (unsigned int)rxMsg.errReportRxStat ); 
#if 0
            LCD_DBG ( LCD_DBG_INIT_ID, "   ERR     : %s\n"    , 
                DISPDRV_dsiErr2text (rxMsg.errReport, dsiE) );       
#endif
        }        
    }
    return ( res );
} 

//*****************************************************************************
//
// Function Name: dsic_init
// 
// Description:   
//
//*****************************************************************************
Int32 dsic_init ( void )
{
    Int32 res = 0;

    if(     panel[0].drvState != DRV_STATE_INIT 
         && panel[0].drvState != DRV_STATE_OPEN  
         && panel[1].drvState != DRV_STATE_INIT  
         && panel[1].drvState != DRV_STATE_OPEN  )
    {     
        LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: OK\n\r", __FUNCTION__ );
        panel[0].drvState = DRV_STATE_INIT;
        panel[1].drvState = DRV_STATE_INIT;
    } 
    else
    {
        LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: OK, Already Init\n\r",
            __FUNCTION__ );
    }   
    return ( res );
}

//*****************************************************************************
//
// Function Name: dsic_exit
// 
// Description:   
//
//*****************************************************************************
Int32 dsic_exit ( void )
{
    LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: Not Implemented\n\r",
        __FUNCTION__ );
    return ( -1 );
}

//*****************************************************************************
//
// Function Name: dsic_open
// 
// Description:   Open Drivers
//
//*****************************************************************************
#ifndef _BRCM_8BYTE_MSG_CONSTRAINT
Int32 dsic_open ( 
    const void*         params,
    DISPDRV_HANDLE_T*   drvH,
    void		*ptr
    )
#else
Int32 dsic_open ( 
    const void*         params,
    DISPDRV_HANDLE_T*   drvH 
    )
#endif
{
    Int32                         res = 0;
    UInt32                        busId; 
    const DISPDRV_OPEN_PARM_T*    pOpenParm;
    dsic_panel_t         *pPanel;
    printk("dsic_open() ");

    //busCh - NA to DSI interface
    pOpenParm = (DISPDRV_OPEN_PARM_T*) params;
    busId     = pOpenParm->busCh;

    #define BUS_ID_MAX  0

    if( busId > BUS_ID_MAX )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR Invalid DSI Bus[%d]\n\r",
            __FUNCTION__, (unsigned int)busId );
        return ( -1 );
    }

    pPanel = &panel[busId];

    if( pPanel->drvState == DRV_STATE_OPEN )
    {
        *drvH = (DISPDRV_HANDLE_T) pPanel;
        LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: Returning Handle, "
            "Already Open\n\r", __FUNCTION__ );
        return ( res );
    }

    if ( pPanel->drvState != DRV_STATE_INIT )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR Not Init\n\r",
            __FUNCTION__ );
        return ( -1 );
    }    
    
    //DISPDRV_Reset( FALSE );
    
    dsiCfg.bus = busId;
    
    
    pPanel->pFb = pPanel->pFbA = (void*)pOpenParm->busId;

	if(alexVcCmCfg.teCfg.teInType != DSI_TE_NONE)
	{
    	if( dsi_teon( pPanel ) ==  -1 )
    	{
        	LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: "
            	"Failed To Configure TE Input\n", __FUNCTION__ ); 
        	return ( -1 );
    	}
	}
#ifndef _BRCM_8BYTE_MSG_CONSTRAINT
	if ( CSL_DSI_Init(NULL, &dsiCfg,ptr ) != CSL_LCD_OK )
#else
    if ( CSL_DSI_Init(NULL, &dsiCfg ) != CSL_LCD_OK )
#endif	//_BRCM_8BYTE_MSG_CONSTRAINT
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR, DSI CSL Init "
            "Failed\n\r", __FUNCTION__ );
        return ( -1 );
    }
    
    if ( CSL_DSI_OpenClient ( busId, &pPanel->clientH ) != CSL_LCD_OK )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR, CSL_DSI_OpenClient "
            "Failed\n\r", __FUNCTION__);
        return ( -1 );
    }
    
    if ( CSL_DSI_OpenCmVc ( pPanel->clientH, &alexVcCmCfg, &pPanel->dsiCmVcHandle ) 
            != CSL_LCD_OK )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: CSL_DSI_OpenCmVc Failed\n\r",
            __FUNCTION__);
        return ( -1 );
    }

    pPanel->busId      = busId; 
    
    pPanel->win.left   = 0;  
    pPanel->win.right  = PANEL_WIDTH-1; 
    pPanel->win.top    = 0;  
    pPanel->win.bottom = PANEL_HEIGHT-1;
    pPanel->win.width  = PANEL_WIDTH; 
    pPanel->win.height = PANEL_HEIGHT;
    
    pPanel->drvState   = DRV_STATE_OPEN;
    
    *drvH = (DISPDRV_HANDLE_T) pPanel;
    LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: OK\n\r", __FUNCTION__ );

    return ( res );
}

//*****************************************************************************
//
// Function Name: dsic_close
// 
// Description:   Close The Driver
//
//*****************************************************************************
Int32 dsic_close ( DISPDRV_HANDLE_T drvH ) 
{
    Int32                   res = 0;
    dsic_panel_t   *pPanel = (dsic_panel_t *)drvH;
    
    pPanel->pFb  = NULL;
    pPanel->pFbA = NULL;

    if ( CSL_DSI_CloseCmVc ( pPanel->dsiCmVcHandle ) ) 
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR, "
            "Closing Command Mode Handle\n\r", __FUNCTION__);
        return ( -1 );
    }
    
    if ( CSL_DSI_CloseClient ( pPanel->clientH ) != CSL_LCD_OK )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR, Closing DSI Client\n\r",
            __FUNCTION__);
        return ( -1 );
    }
    
    if ( CSL_DSI_Close( pPanel->busId ) != CSL_LCD_OK )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERR Closing DSI Controller\n\r",
            __FUNCTION__ );
        return ( -1 );
    }

    pPanel->pwrState = DISP_PWR_OFF;
    pPanel->drvState = DRV_STATE_INIT;
    LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: OK\n\r", __FUNCTION__ );

    return ( res );
}

void exec_cmnd_list( 
    DISPDRV_HANDLE_T     dispH,
    Boolean              useOs,  
    pDISPCTRL_REC_T      cmnd_lst 
    )
{
    UInt32  i = 0;

    while (cmnd_lst[i].type != DISPCTRL_LIST_END)
    {

		if (cmnd_lst[i].type == DISPCTRL_WR_CMND_MULTIPLE_DATA)
        {
        	dsi_wrcmndPN (dispH, cmnd_lst[i].cmnd, 
                cmnd_lst[i].datasize, cmnd_lst[i].dataptr);
        }
		else if (cmnd_lst[i].type == DISPCTRL_WR_CMND_DATA)
        {
            dsi_wrcmndP1 (dispH, cmnd_lst[i].cmnd, 
                cmnd_lst[i].data);
        }
        else if (cmnd_lst[i].type == DISPCTRL_WR_CMND)
        {
            dsi_wrcmndP0 (dispH, cmnd_lst[i].cmnd);
        }
        else if (cmnd_lst[i].type == DISPCTRL_SLEEP_MS)
        {
            if ( useOs )
            {
                OSTASK_Sleep ( TICKS_IN_MILLISECONDS(cmnd_lst[i].data) );
            }    
            else
            {
				mdelay( cmnd_lst[i].data );
            }    
        }
        i++;
    }
} // execCmndList

//*****************************************************************************
//
// Function Name:   dsi_setwindow
//
// Description:     
//                   
//*****************************************************************************
static Int32 dsi_setwindow ( DISPDRV_HANDLE_T drvH )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
	UInt8 dataptr[4];
	{//write Set_clumn_address(2A)
		dataptr[0]= (pPanel->col_start) >> 8;
		dataptr[1]= pPanel->col_start & 0xFF;
		dataptr[2]= (pPanel->col_end) >> 8;
		dataptr[3]= pPanel->col_end & 0xFF;
    	dsi_wrcmndPN (drvH, 0x2A, 4, dataptr);
	}

	{//write Set_page_address(2B)
                dataptr[0]= (pPanel->row_start) >> 8;
                dataptr[1]= pPanel->row_start & 0xFF;
                dataptr[2]= (pPanel->row_end) >> 8;
                dataptr[3]= pPanel->row_end & 0xFF;
    	dsi_wrcmndPN (drvH, 0x2B, 4, dataptr);
	}
	//Write_memory_start (2Ch)
//	 dsi_wrcmndP0 ( drvH, 0x2C );
     return 0;
} // dsi_setwindow

//*****************************************************************************
//
// Function Name: dsic_power_control
// 
// Description:   Display Module Control
//
//*****************************************************************************
Int32 dsic_power_control ( 
    DISPDRV_HANDLE_T        drvH, 
    DISPLAY_POWER_STATE_T   state )
{
    Int32  res = 0;
    dsic_panel_t   *pPanel = (dsic_panel_t *)drvH;
    
    switch ( state )
    {
        case DISPLAY_POWER_STATE_ON:
            switch ( pPanel->pwrState )
            {
                case DISP_PWR_OFF:
	                exec_cmnd_list(drvH, TRUE, LCD_Init);
	                //dsi_setwindow(drvH);
	                pPanel->pwrState = DISP_PWR_SLEEP_OFF;
                    LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: INIT-SEQ\n\r",
                        __FUNCTION__ );

                   break; 
                case DISP_PWR_SLEEP_ON:
                    exec_cmnd_list(drvH, TRUE, LCD_ExitSleep);
                    pPanel->pwrState = DISP_PWR_SLEEP_OFF;
                    LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: SLEEP-OUT\n\r",
                        __FUNCTION__ );
                    break;
                    
                default:
                    break;    
            }        
            break;
        case DISPLAY_POWER_STATE_OFF:
            LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: POWER-OFF State "
                "Not Supported\n\r", __FUNCTION__ );
            res = -1;
            break;
            
        case DISPLAY_POWER_STATE_SLEEP:
            if( pPanel->pwrState == DISP_PWR_SLEEP_OFF )
            {
                exec_cmnd_list(drvH, TRUE, LCD_EnterSleep);
                pPanel->pwrState = DISP_PWR_SLEEP_ON;
                LCD_DBG ( LCD_DBG_INIT_ID, "[DISPDRV] %s: SLEEP-IN\n\r",
                    __FUNCTION__ );
            } 
            else
            {
                LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: SLEEP-IN Requested, "
                    "But Not In POWER-ON State\n\r", __FUNCTION__ );
                res = -1;
            }   
            break;
        
        default:
            LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: Invalid Power State[%d] "
                "Requested\n\r", __FUNCTION__, state );
            res = -1;
            break;
    }
    return ( res );
}

//*****************************************************************************
//
// Function Name: dsic_start
// 
// Description:   Configure For Updates
//
//*****************************************************************************
Int32 dsic_start ( DISPDRV_HANDLE_T drvH )
{
    Int32 res = 0;
 
    return ( res );
}

//*****************************************************************************
//
// Function Name: dsic_stop
// 
// Description:   
//
//*****************************************************************************
Int32 dsic_stop ( DISPDRV_HANDLE_T drvH )
{
    Int32 res = 0;
  
    return ( res );
}

//*****************************************************************************
//
// Function Name: dsic_get_info
// 
// Description:   
//
//*****************************************************************************
const DISPDRV_INFO_T* dsic_get_info ( DISPDRV_HANDLE_T drvH )
{
    return ( &DSI_Display_Info );
}

//*****************************************************************************
//
// Function Name: dsic_info
// 
// Description:   
//
//*****************************************************************************
Int32 dsic_info ( UInt32 *screenWidth, UInt32 *screenHeight, UInt32 *bpp, UInt32 *resetGpio)
{
   Int32 res = 0;
   
	*screenWidth = SCREEN_WIDTH;
	*screenHeight = SCREEN_HEIGHT;
	*bpp = INPUT_BPP;
	*resetGpio = RESET_GPIO; 

   return ( res );
}


//*****************************************************************************
//
// Function Name: dsi_cb
// 
// Description:   CSL callback        
//
//*****************************************************************************
static void dsi_cb ( CSL_LCD_RES_T cslRes, CSL_LCD_HANDLE handle, void * pCbRec ) 
{
    //LCD_DBG ( LCD_DBG_ID, "[DISPDRV] +%s\r\n", __FUNCTION__ );

	if(dispdrv_cb)
		((DISPDRV_CB_T)dispdrv_cb)(DISPDRV_CB_RES_OK);
    //LCD_DBG ( LCD_DBG_ID, "[DISPDRV] -%s\r\n", __FUNCTION__ );
}


#ifdef CONFIG_BRCM_KPANIC_UI_IND
Int32 dsi_set_dev_border(DISPDRV_HANDLE_T handle, int img_width, int img_height)
{
	dsic_panel_t *dev = (dsic_panel_t *)handle;

	if((DSI_Display_Info.width < img_width) || (DSI_Display_Info.height < img_height)) {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: img width %d is bigger than panel size %d!\n\r",
                   __FUNCTION__, img_width, DSI_Display_Info.width );
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: img height %d is bigger than panel size %d!\n\r",
                   __FUNCTION__, img_height, DSI_Display_Info.height );
		return -1;
	}

    dev->col_start = (DSI_Display_Info.width - img_width) / 2;
	dev->col_end = DSI_Display_Info.width - (DSI_Display_Info.width - img_width) / 2 - 1;
	dev->row_start = (DSI_Display_Info.height - img_height) / 2;
	dev->row_end = DSI_Display_Info.height - (DSI_Display_Info.height - img_height) / 2 - 1;
	return 0;
}

//*****************************************************************************
//
// Function Name: dsic_send_data
//
// Description:   DMA/OS Update using INT frame buffer
//
//*****************************************************************************
Int32 dsic_send_data (
	DISPDRV_HANDLE_T	drvH,
	int 				fb_idx,
	DISPDRV_CB_T		apiCb,
	int 		img_width,
	int 		img_height
	)
{
	dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
	CSL_LCD_UPD_REQ_T   req;
	Int32   			res  = 0;

	//LCD_DBG ( LCD_DBG_ID, "[DISPDRV] +%s\r\n", __FUNCTION__ );

	if ( pPanel->pwrState != DISP_PWR_SLEEP_OFF )
	{
		LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] +%s: Skip Due To Power State\r\n",
			__FUNCTION__ );
		return ( -1 );
	}

	if (0 == fb_idx)
		req.buff		   = pPanel->pFbA;
	 else
		req.buff		   = (void *)((UInt32)pPanel->pFbA +
                                      DSI_Display_Info.width * DSI_Display_Info.height * INPUT_BPP);

	req.lineLenP	   = img_width;
	req.lineCount      = img_height;
	req.buffBpp 	   = INPUT_BPP;
	req.timeOut_ms     = 100;
	req.multiLLI	   = false;
	//printk(KERN_ERR "SSSSK buf=%08x, linelenp = %d, linecnt =%d\n", (u32)req.buff, req.lineLenP, req.lineCount);
	req.cslLcdCbRef = NULL;
	dispdrv_cb = apiCb;

	if( apiCb != NULL )
	   req.cslLcdCb = dsi_cb;
	else
	   req.cslLcdCb = NULL;

	dsi_setwindow(drvH);

	if ( CSL_DSI_UpdateCmVc ( pPanel->dsiCmVcHandle, &req ) != CSL_LCD_OK )
	{
		LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR ret by "
			"CSL_DSI_UpdateCmVc\n\r", __FUNCTION__ );
		res = -1;
	}

	//LCD_DBG ( LCD_DBG_ID, "[DISPDRV] -%s\r\n", __FUNCTION__ );
	return ( res );
}
#endif


//*****************************************************************************
//
// Function Name: dsic_update
// 
// Description:   DMA/OS Update using INT frame buffer
//                Call this only when the full frame update is required.
//
//*****************************************************************************
Int32 dsic_update ( 
    DISPDRV_HANDLE_T    drvH, 
    int			fb_idx,
    DISPDRV_CB_T        apiCb
    )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
    CSL_LCD_UPD_REQ_T   req;
    Int32               res  = 0;
	bool                multiLLI_Set = true;
	OSDAL_Dma_Buffer_List *buffer_list=NULL, *temp_list=NULL;

    //LCD_DBG ( LCD_DBG_ID, "[DISPDRV] +%s\r\n", __FUNCTION__ );
   
    if ( pPanel->pwrState != DISP_PWR_SLEEP_OFF )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] +%s: Skip Due To Power State\r\n", 
            __FUNCTION__ );
        return ( -1 );
    }
   
	if(multiLLI_Set == true)
	{
		void* src_buff = NULL;
		int i=0;
		buffer_list = kzalloc((DSI_Display_Info.height) * sizeof(OSDAL_Dma_Buffer_List), GFP_KERNEL);
		if (!buffer_list) {
	        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] +%s: Could not allocate memory\r\n",
	            __FUNCTION__ );

			goto done;
		}

		if (0 == fb_idx)
           src_buff = pPanel->pFbA;
	    else
           src_buff = (void *)((UInt32)pPanel->pFbA + DSI_Display_Info.width * DSI_Display_Info.height * INPUT_BPP);

		temp_list = buffer_list;
		for (i = 0; i < DSI_Display_Info.height; i++) {
			temp_list->buffers[0].srcAddr  = (UInt32)src_buff + (i * DSI_Display_Info.width * INPUT_BPP);
			temp_list->buffers[0].destAddr = 0x084a0140;
			temp_list->buffers[0].length   = DSI_Display_Info.width * INPUT_BPP;
			temp_list->buffers[0].bRepeat  = 0;
			temp_list++;
		}
		temp_list--;		/* Go back to the last list item to set interrupt = 1 */
		temp_list->buffers[0].interrupt = 1;

		req.buff = (void *)buffer_list;
		req.multiLLI = true;
	}
	else if( multiLLI_Set == false)
	{
    if (0 == fb_idx)
    	req.buff           = pPanel->pFbA;
     else
		    req.buff = (void *)((UInt32)pPanel->pFbA + DSI_Display_Info.width * DSI_Display_Info.height * INPUT_BPP);

        req.multiLLI = false;
	}

    req.lineLenP       = DSI_Display_Info.width;
    req.lineCount      = DSI_Display_Info.height;
    req.buffBpp        = INPUT_BPP;
    req.timeOut_ms     = 100;
    //printk(KERN_ERR "buf=%08x, linelenp = %d, linecnt =%d\n", (u32)req.buff, req.lineLenP, req.lineCount);
    req.cslLcdCbRef = NULL;
    dispdrv_cb = apiCb;
	
    if( apiCb != NULL )
       req.cslLcdCb = dsi_cb;
    else
       req.cslLcdCb = NULL;
    
	pPanel->col_start = 0;
	pPanel->col_end   = (DSI_Display_Info.width -1);
	pPanel->row_start = 0;
	pPanel->row_end   = (DSI_Display_Info.height-1);
	dsi_setwindow(drvH);

    if ( CSL_DSI_UpdateCmVc ( pPanel->dsiCmVcHandle, &req ) != CSL_LCD_OK )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR ret by "
            "CSL_DSI_UpdateCmVc\n\r", __FUNCTION__ );
        res = -1;    
    }
fail:
   if(buffer_list)
	 kfree(buffer_list);
        
done:
    //LCD_DBG ( LCD_DBG_ID, "[DISPDRV] -%s\r\n", __FUNCTION__ );
     return ( res );
}

static inline bool is_unaligned(dsic_panel_t * dev)
{
        return ((dev->win.left * INPUT_BPP) & 2);
}

static inline bool is_odd_total(dsic_panel_t * dev)
{
        return (((dev->win.right - dev->win.left + 1) * (dev->win.bottom - dev->win.top + 1)) % 2);
}

static inline bool is_odd_stride(dsic_panel_t * dev)
{
        return (((dev->win.right - dev->win.left + 1) * INPUT_BPP) & 2);
}

static inline bool is_out_of_bounds(dsic_panel_t * dev)
{
        return ((dev->win.right >= DSI_Display_Info.width) || (dev->win.left >= DSI_Display_Info.width));
}

static inline bool is_tx_done_16(dsic_panel_t * dev)
{
        return ((dev->win.right <= dev->win.left) || (dev->win.right == 0));
}

static inline bool is_tx_done_32(dsic_panel_t * dev)
{
        return (dev->win.right <= dev->win.left);
}

/****************************************************************************
*
*  dsi_update_column
*
*  Update one column of LCD in non-DMA mode within dirty region.
*  DMA mode is also implemented
*
*
***************************************************************************/
static int dsi_update_column( dsic_panel_t * dev,
                        unsigned int         column,
                        DISPDRV_CB_T        apiCb   )
{
	int i, stride;
	uint32_t  *source;
	Int32     res  = 0;
	CSL_LCD_UPD_REQ_T   req;
	uint32_t  count;
#ifdef LCD_COLUMN_UPDATE_CPU
	unsigned int data_buff[480];
#endif
#ifdef LCD_COLUMN_UPDATE_DMA
	OSDAL_Dma_Buffer_List *buffer_list, *temp_list;
#endif
	stride = DSI_Display_Info.width * INPUT_BPP;
	source = (u32)(phys_to_virt(dev->pFbA)) + stride * dev->win.top +
				column * INPUT_BPP;

	if (4 == INPUT_BPP)  {
		count = (dev->win.bottom - dev->win.top + 1);
		count &= ~1; //Ignore one pixel in case count is odd
#ifdef LCD_COLUMN_UPDATE_DMA
		buffer_list = kzalloc((dev->win.bottom - dev->win.top + 1) * sizeof(OSDAL_Dma_Buffer_List), GFP_KERNEL);
        if (!buffer_list) {
			pr_info("Couldn't allocate memory for dma buffer list\n");
			goto done;
		}
		temp_list = buffer_list;
        for (i = 0; i < count; i++) {
                temp_list->buffers[0].srcAddr = source;
                temp_list->buffers[0].destAddr = 0x084a0140;
                temp_list->buffers[0].length = 1 * INPUT_BPP;
                temp_list->buffers[0].bRepeat = 0;
                temp_list++;
                source += stride;
		}
        temp_list--;            /* Go back to the last list item to set interrupt = 1 */
        temp_list->buffers[0].interrupt = 1;

    req.buff           = (void *)buffer_list;
	req.lineLenP      = 1;
	req.lineCount      = count;
	req.buffBpp        = INPUT_BPP;
	req.timeOut_ms     = 100;
	req.multiLLI       = true;

		req.cslLcdCbRef = NULL;
		dispdrv_cb = apiCb;
		if( apiCb != NULL )
			req.cslLcdCb = dsi_cb;
		else
		req.cslLcdCb = NULL;
	if ( CSL_DSI_UpdateCmVc ( dev->dsiCmVcHandle, &req ) != CSL_LCD_OK )
		{
			LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR ret by "
				"CSL_DSI_UpdateCmVc\n\r", __FUNCTION__ );
			res = -1;
		}
#endif
#ifdef LCD_COLUMN_UPDATE_CPU
        for (i = 0; i < count; i++) {
			data_buff[i] = *source;
			source += stride;
		}
		req.lineLenP    =1;
		req.lineCount   =count;
		req.cslLcdCbRef = NULL;
		dispdrv_cb = apiCb;
		if( apiCb != NULL )
			req.cslLcdCb = dsi_cb;
		else
			req.cslLcdCb    =NULL;
		if( CSL_DSI_CPU_UpdateCmVc(dev->dsiCmVcHandle, &req,data_buff) != CSL_LCD_OK )
		{
			LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR ret by "
				"CSL_DSI_UpdateCmVc\n\r", __FUNCTION__ );
			res = -1;
		}
#endif

	} else {
		pr_info("bpp=%d is not supported\n", INPUT_BPP);
	}
#ifdef LCD_COLUMN_UPDATE_DMA
done :
	kfree(buffer_list);
#endif
	return ( res );
}



//*****************************************************************************
//
// Function Name: dsic_dirty_rect_update
//
// Description:   DMA/OS Update using INT frame buffer
//
//*****************************************************************************
Int32 dsic_dirty_rect_update (
    DISPDRV_HANDLE_T    drvH,
    lcd_drv_rect_t     * dirtyRect,
    DISPDRV_CB_T        apiCb
    )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
    CSL_LCD_UPD_REQ_T   req;
    Int32               res  = 0;
    Int32		 i;
    Int32		 err = -EINVAL;

    OSDAL_Dma_Buffer_List *buffer_list, *temp_list;
    //LCD_DBG ( LCD_DBG_ID, "[DISPDRV] +%s\r\n", __FUNCTION__ );

    if ( pPanel->pwrState != DISP_PWR_SLEEP_OFF )
    {
        LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] +%s: Skip Due To Power State\r\n",
            __FUNCTION__ );
        return ( -1 );
    }

    if(!dirtyRect) {
        pr_info("dirtyRect pointer is NULL\n");
        return -1;
    }

        if ((dirtyRect->top > dirtyRect->bottom)
            || ((dirtyRect->bottom - dirtyRect->top) >= DSI_Display_Info.height)
            || (dirtyRect->left > dirtyRect->right)
            || ((dirtyRect->right - dirtyRect->left) >= DSI_Display_Info.width)) {
                LCD_DBG("invalid dirty-rows params - ignoring\n");
                LCD_DBG("top = %u,  bottom = %u, left = %u, right = %u\n",
                          dirtyRect->top, dirtyRect->bottom,
                          dirtyRect->left, dirtyRect->right);
                return -1;
        }

       pPanel->win = *dirtyRect;
        pPanel->row_start = pPanel->win.top % DSI_Display_Info.height;
        pPanel->row_end = pPanel->win.bottom % DSI_Display_Info.height;
        pPanel->win.width = DSI_Display_Info.width;
        pPanel->win.height = DSI_Display_Info.height;

        /*If start address is aligned to odd boundary */
	if (is_unaligned(pPanel)) {
		pPanel->col_start = pPanel->win.left;
		pPanel->col_end = pPanel->win.left;
		dsi_setwindow(drvH);
		dsi_update_column(pPanel, pPanel->win.left,NULL);
		pPanel->win.left += 1;
	}

        /*If length is odd multiple */
	if (is_odd_stride(pPanel) || (is_odd_total(pPanel))) {
		pPanel->col_start = pPanel->win.right;
		pPanel->col_end = pPanel->win.right;
		dsi_setwindow(drvH);
		dsi_update_column(pPanel, pPanel->win.right,NULL);
		pPanel->win.right -= 1;
	}

	if (is_out_of_bounds(pPanel) || (is_tx_done_16(pPanel))
		|| (is_tx_done_32(pPanel))) {
		/* Dirty columns have been transferred. No further processing required.*/
		goto done;
	}
	buffer_list = kzalloc((pPanel->win.bottom - pPanel->win.top + 1) * sizeof(OSDAL_Dma_Buffer_List), GFP_KERNEL);
	if (!buffer_list) {
		pr_info("Couldn't allocate memory for dma buffer list\n");
		goto done;
    }

	temp_list = buffer_list;
	for (i = pPanel->win.top; i <= pPanel->win.bottom; i++) {
		temp_list->buffers[0].srcAddr = (UInt32)pPanel->pFbA + (i * DSI_Display_Info.width + pPanel->win.left) * INPUT_BPP;
		temp_list->buffers[0].destAddr = 0x084a0140;
		temp_list->buffers[0].length = (pPanel->win.right - pPanel->win.left +1) * INPUT_BPP;
		temp_list->buffers[0].bRepeat = 0;
		temp_list++;
	}

	temp_list--;            /* Go back to the last list item to set interrupt = 1 */
	temp_list->buffers[0].interrupt = 1;
	req.buff = (void *)buffer_list;
	req.buffBpp = INPUT_BPP;
	req.lineLenP = pPanel->win.right - pPanel->win.left + 1;
	req.lineCount = pPanel->win.bottom - pPanel->win.top + 1;
	req.timeOut_ms = 100;
	req.multiLLI = true;
	req.cslLcdCbRef = NULL;
	dispdrv_cb = apiCb;

	if( apiCb != NULL )
		req.cslLcdCb = dsi_cb;
	else
		req.cslLcdCb = NULL;

	pPanel->col_start = pPanel->win.left;
	pPanel->col_end = pPanel->win.right;
	dsi_setwindow(drvH);
	if ( CSL_DSI_UpdateCmVc ( pPanel->dsiCmVcHandle, &req ) != CSL_LCD_OK )
	{
		LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: ERROR ret by "
		"CSL_DSI_UpdateCmVc\n\r", __FUNCTION__ );
		res = -1;
	}

done:
	//LCD_DBG ( LCD_DBG_ID, "[DISPDRV] -%s\r\n", __FUNCTION__ );
	kfree(buffer_list);
	return ( res );
}


//*****************************************************************************
//
// Function Name: dsic_set_control
// 
// Description:   
//
//*****************************************************************************
Int32 dsic_set_control ( 
        DISPDRV_HANDLE_T    drvH, 
        DISPDRV_CTRL_ID_T   ctrlID, 
        void*               ctrlParams 
        )
{
    Int32 res = -1;
   
    switch ( ctrlID )
    {
        case DISPDRV_CTRL_ID_SET_REG:
            dsi_ioctlwr( drvH, (DISPDRV_CTRL_RW_REG*)ctrlParams );
            res = 0;
            break;
    
        default:
            LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: "
                "CtrlId[%d] Not Implemented\n\r", __FUNCTION__, ctrlID );
            break;    
    }
    return ( res );
}
                    
//*****************************************************************************
//
// Function Name: dsic_get_control
// 
// Description:   
//
//*****************************************************************************
Int32 dsic_get_control (
            DISPDRV_HANDLE_T    drvH, 
            DISPDRV_CTRL_ID_T   ctrlID, 
            void*               ctrlParams 
            )
{
    dsic_panel_t *pPanel = (dsic_panel_t *)drvH;
    Int32 res = -1;
  
    switch ( ctrlID )
    {
        case DISPDRV_CTRL_ID_GET_FB_ADDR:
            ((DISPDRV_CTL_GET_FB_ADDR *)ctrlParams)->frame_buffer = 
                pPanel->pFbA;
            res = 0;
            break;
            
        case DISPDRV_CTRL_ID_GET_REG:
			    if(1)
				{//Kevin.Wen:FIXME
					UInt8 dataprt[] = {0xFF, 0x83, 0x69};
        			dsi_wrcmndPN (drvH, 0xB9, 3, dataprt);

					dsi_set_max_ret_pkt_size(drvH, 0x02);
			    }
            res = dsi_ioctlrd( drvH, (DISPDRV_CTRL_RW_REG*)ctrlParams );
            break;
            
        default:
            LCD_DBG ( LCD_DBG_ERR_ID, "[DISPDRV] %s: CtrlId[%d] Not "
                "Implemented\n\r", __FUNCTION__, ctrlID );
            break;
    }
    
    return ( res );
}            
