/*******************************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement
governing use of this software, this software is licensed to you under the
terms of the GNU General Public License version 2, available at
http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software
in any way with any other Broadcom software provided under a license other than
the GPL, without Broadcom's express prior written consent.
*******************************************************************************************/
/**
*
*   @file   capi_emulator_int.h
*
*   @brief  This file defines the types and prototypes for the CAPI emulator internal functions.
*
****************************************************************************/

#ifndef _CAPI_EMULATOR_INT_H_
#define _CAPI_EMULATOR_INT_H_

//******************************************************************************
//	 			include block
//******************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

// enable this flag when implementation of CAPI2 calls available

#ifndef TRUE
#define TRUE (Boolean)1
#endif

#ifndef FALSE
#define FALSE (Boolean)0
#endif


#define CEMU_PrepForCAPI2Call RPC_SyncCreateTID
#define CEMU_GetResponse RPC_SyncWaitForResponse
#define C2GLUE_ACK_Result_t	RPC_ACK_Result_t
#define C2GLUE_ACK_SUCCESS	ACK_SUCCESS
#define CEMU_AddCbkToTid RPC_SyncAddCbkToTid
#define CEMU_GetCbkFromTid	RPC_SyncGetCbkFromTid
#define CEMU_DeleteCbkFromTid	RPC_SyncDeleteCbkFromTid
#define CEMU_LockResponseBuffer	RPC_SyncLockResponseBuffer
#define CEMU_ReleaseResponseBuffer	RPC_SyncReleaseResponseBuffer
#define CEMU_PrepForCAPI2CallEx RPC_SyncSetTID

UInt8 CEMU_GetClientId(void);

Boolean CEMU_HandleNwDateTime(ClientInfo_t *clientInfo,  void *timeZoneDate );

void CEMU_InitClientInfo(ClientInfo_t *clientInfo, UInt8 clientID);

/** @} */
#ifdef __cplusplus
}
#endif

#endif
