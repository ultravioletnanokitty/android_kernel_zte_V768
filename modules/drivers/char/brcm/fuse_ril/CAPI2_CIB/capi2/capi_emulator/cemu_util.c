//*********************************************************************
//
//	Copyright ?2004-2011 Broadcom Corporation
//	
//	This program is the proprietary software of Broadcom Corporation 
//	and/or its licensors, and may only be used, duplicated, modified 
//	or distributed pursuant to the terms and conditions of a separate, 
//	written license agreement executed between you and Broadcom (an 
//	"Authorized License").  Except as set forth in an Authorized 
//	License, Broadcom grants no license (express or implied), right 
//	to use, or waiver of any kind with respect to the Software, and 
//	Broadcom expressly reserves all rights in and to the Software and 
//	all intellectual property rights therein.  IF YOU HAVE NO 
//	AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE 
//	IN ANY WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE 
//	ALL USE OF THE SOFTWARE.  
//	
//	Except as expressly set forth in the Authorized License,
//	
//	1.	This program, including its structure, sequence and 
//		organization, constitutes the valuable trade secrets 
//		of Broadcom, and you shall use all reasonable efforts 
//		to protect the confidentiality thereof, and to use 
//		this information only in connection with your use 
//		of Broadcom integrated circuit products.
//	
//	2.	TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE 
//		IS PROVIDED "AS IS" AND WITH ALL FAULTS AND BROADCOM 
//		MAKES NO PROMISES, REPRESENTATIONS OR WARRANTIES, 
//		EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, 
//		WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY 
//		DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, 
//		MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR A 
//		PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR 
//		COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR 
//		CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE 
//		RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.  
//
//	3.	TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT 
//		SHALL BROADCOM OR ITS LICENSORS BE LIABLE FOR 
//		(i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT, OR 
//		EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY 
//		WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE 
//		SOFTWARE EVEN IF BROADCOM HAS BEEN ADVISED OF THE 
//		POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN 
//		EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE 
//		ITSELF OR U.S. $1, WHICHEVER IS GREATER. THESE 
//		LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE 
//		OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
//
//***************************************************************************
/**
*
*   @file   cemu_util.c
*
*   @brief  This file implements the DATA Account Services functions for CAPI emulator.
*
****************************************************************************/


//******************************************************************************
//	 			include block
//******************************************************************************

#include "mobcom_types.h"
#include "resultcode.h"
#include "taskmsgs.h"
#include "consts.h"

#include "xdr_porting_layer.h"
#include "xdr.h"

#include "common_defs.h"
#include "uelbs_api.h"
#include "ms_database_def.h"
#include "common_sim.h"
#include "sim_def.h"
#ifndef UNDER_LINUX
#include <string.h>
#endif
#include "assert.h"
#include "sysparm.h"
#include "engmode_api.h"
#include "sysparm.h"
///
#include "i2c_drv.h"
#include "ecdc.h"
#include "uelbs_api.h"
#include "common_sim.h"
#include "sim_def.h"
#include "stk_def.h"
#include "mti_trace.h"
#include "logapi.h"
#include "log.h"
#include "tones_def.h"
#include "phonebk_def.h"
#include "phonectrl_def.h"
#include "phonectrl_api.h"
#include "rtc.h"
#include "netreg_def.h"
#include "ms_database_old.h"
#include "ms_database_api.h"
#include "netreg_util_old.h"
#include "netreg_util.h"
#include "netreg_api.h"
#include "common_sim.h"
#include "sim_def.h"
#include "stk_def.h"
#include "ss_def.h"
#include "sim_api.h"
#include "phonectrl_def.h"
#include "isim_def.h"
#include "ostypes.h"
#include "pch_def.h"
#include "pchex_def.h"
#include "hal_em_battmgr.h"
#include "cc_def.h"
#include "rtem_events.h"
#include "rtc.h"
#include "engmode_def.h"
#include "sms_def.h"
#include "simlock_def.h"
#include "isim_def.h"
#include "pch_def.h"
#include "pchex_def.h"

#include "engmode_api.h"
#include "sysparm.h"
#include "ms_database_old.h"
#include "ms_database_api.h"
#include "ss_api.h"
#include "sms_api_atc.h"
#include "sms_api_old.h"
#include "sms_api.h"
#include "cc_api_old.h"
#include "sim_api_old.h"
#include "sim_api.h"
#include "phonebk_api_old.h"
#include "phonebk_api.h"
#include "phonectrl_api.h"
#include "isim_api_old.h"
#include "isim_api.h"

#include "util_api_old.h"
#include "util_api.h"
#include "dialstr_api.h"
#include "stk_api_old.h"
#include "stk_api.h"

#include "pch_api_old.h"
#include "pch_api.h"
#include "pchex_api.h"
#include "ss_api_old.h"
#include "lcs_cplane_rrlp_api.h"
#include "cc_api.h"
#include "netreg_api.h"
#include "lcs_ftt_api.h"
#include "lcs_cplane_rrc_api.h"
#include "lcs_cplane_shared_def.h"

#include "capi2_mstruct.h"
#include "capi2_sim_api.h"
#include "capi2_phonectrl_api.h"
#include "capi2_sms_api.h"
#include "capi2_cc_api.h"
#include "capi2_lcs_cplane_api.h"
#include "capi2_ss_api.h"
#include "capi2_phonebk_api.h"
#include "capi2_cmd_resp.h"
#include "capi2_phonectrl_api.h"
//#include "capi2_gen_api.h"	

#include "ipcproperties.h"
#include "rpc_global.h"
#include "rpc_ipc.h"
#include "xdr_porting_layer.h"
#include "xdr.h"
#include "rpc_api.h"

#include "capi2_global.h"
#include "capi2_mstruct.h"
#include "capi2_cc_ds.h"
#include "capi2_cc_msg.h"
#include "capi2_msnu.h"
#include "ss_api_old.h"
#include "ss_lcs_def.h"
#include "lcs_cplane_api.h"
#include "lcs_cplane_rrc_api.h"
#include "lcs_cplane_shared_def.h"
#include "lcs_ftt_api.h"
#include "capi2_gen_api.h"
#include "capi_emulator_int.h"
#include "rpc_sync_api.h"
#ifndef UNDER_LINUX
#include "sys_string.h"
#endif

#ifdef UNDER_LINUX
#define PCH_LOG_ARRAY
#define MSG_LOGV printk;
#endif

#ifndef UNDER_LINUX
//JW TODO, we do not want to introduce in sys_string.* right now, RIL has 
//implementation on these functions already, so comment out these for now
//need revisit later on to see if we can delete the functions in RIL which 
//are mainly the same as the ones here but with string function differences.
//******************************************************************************
// Function Name: MS_PlmnConvertRawMcc	
//
// Description: This function converts the MCC values: 
// For example: from 0x1300 to 0x310 for Pacific Bell.
//******************************************************************************
UInt16 MS_PlmnConvertRawMcc(UInt16 mcc)
{
	return ( (mcc & 0x0F00) | ((mcc & 0xF000) >> 8) | (mcc & 0x000F) );
}

//******************************************************************************
// Function Name: MS_PlmnConvertRawMnc	
//
// Description: This function converts the MNC values. 
// The third digit of MNC may be stored in the third nibble of the passed MCC.
// For example: the passed MCC is 0x1300 and passed MNC is 0x0071 for Pacific Bell.
// This function returns the converted MNC value as 0x170
//******************************************************************************
UInt16 MS_PlmnConvertRawMnc(UInt16 mcc, UInt16 mnc)
{
	UInt16 converted_mcc;

	if( (mcc & 0x00F0) == 0x00F0 )
	{
		converted_mcc = MS_PlmnConvertRawMcc(mcc);

		if( (converted_mcc == 0x302) ||  ((converted_mcc >= 0x0310) && (converted_mcc <= 0x0316)) )
		{
			/* This is a North America PCS network, the third MNC digit is 0: see Annex A of GSM 03.22.
             * Canadian PCS networks (whose MCC is 0x302) also use 3-digit MNC, even though GSM 03.22 does
             * not specify it.
             */
			return ( (mnc & 0x00F0) | ((mnc & 0x000F) << 8) );
		}
		else
		{
			/* Two digit MNC */
			return ( ((mnc & 0x00F0) >> 4) | ((mnc & 0x000F) << 4) );
		}
	}
	else
	{
		/* Three digit MNC */
		return ( (mnc & 0x00F0) | ((mnc & 0x000F) << 8) | ((mcc & 0x00F0) >> 4) );
	}
}

void MS_ConvertRawPlmnToHer(
	UInt16 mcc,
	UInt16 mnc,
	UInt8 *plmn_str,
	UInt8  plmn_str_sz
	)
{
	UInt16 mcc_code = MS_PlmnConvertRawMcc( mcc );
	UInt16 mnc_code = MS_PlmnConvertRawMnc( mcc, mnc );

	if( (mcc & 0x00F0) != 0x00F0 ) {
		SYS_FormatStr8((UInt8*) plmn_str, (Int32)plmn_str_sz, "%03X%03X", mcc_code, mnc_code);
	}
	else {
		SYS_FormatStr8((UInt8*) plmn_str, (Int32)plmn_str_sz, "%03X%02X", mcc_code, mnc_code);
	}
}

//******************************************************************************
// Function Name:	MS_ConvertStringToRawPlmnId
//
// Description:	This function converts numberic string into plmn id and will 
//				return false if the input is not correct.
//
// Notes:
//******************************************************************************
Boolean MS_ConvertStringToRawPlmnId(UInt8 *oper, PLMNId_t *plmn_id )
{	
	UInt8	i;
	UInt8	len = strlen((char *) oper);
	UInt8	temp[50];

	// convert string to data array
	for(i=0; i<len; i++)
	{
		temp[i] = oper[i] - '0';
	}

	if( len == 5)
	{
		plmn_id->mcc = (temp[0] << 8) | (temp[1] << 12) |  temp[2] | 0xf0;
		plmn_id->mnc = (temp[4] << 4) | temp[3];	
	}
	else if(len == 6)
	{
		plmn_id->mcc = (temp[0] << 8) | (temp[1] << 12) | temp[2] | (temp[5] << 4);
		plmn_id->mnc = (temp[4] << 4) | temp[3];
	}
	else
	{
		// the string is either too long or too short
		return FALSE;
	}

	return TRUE;
}
#endif

//******************************************************************************
//
// Function Name:	PdpApi_GetDecodedPDPAddress
//
// Description: This function is used to get the decoded PDP address information
//				(pdp type, IPV4 and IPV6 address from PDP address IE
//                  
// Notes:
//Decode PDP type and PDP address from PDP Address IE
//From section 10.5.6.4 of 24.008 
//1) If PDP type number indicates IPv4, the Address information in octet 5 to octet 8 contains the IPv4 address
//2) If PDP type number indicates IPv6, the Address information in octet 5 to octet 20 contains the IPv6 address
//3) If PDP type number indicates IPv4v6:
//	The Address information in octet 5 to octet 8 contains the IPv4 address. 
//	The Address information in octet 9 to octet 24 contains the IPv6 address
//******************************************************************************
Result_t		PdpApi_GetDecodedPDPAddress(
							ClientInfo_t* inClientInfoPtr, 
							PCHPDPAddressIE_t inAddressIE, 
							PCHDecodedPDPAddress_T* outAddress)
{
	Result_t result = RESULT_OK;
	PCHPDPAddressType_t	pdpAddressType; //Address type returned from PDP address IE

	//spec defines PDP type number value (octet 4) but stack did not pass the 1st byte (IEI number)
	pdpAddressType = (PCHPDPAddressType_t)inAddressIE[PCH_PDP_TYPE_OCTET-2];
	MSG_LOGV( "PCH: PdpApi_GetDecodedPDPAddress, pdpAddressType = ", pdpAddressType );

	outAddress->pdpAddressType = pdpAddressType;

	if (pdpAddressType == PCH_IPV4_ADDRESS)
	{
		memcpy(outAddress->pdpAddressIPV4, (UInt8 *)&(inAddressIE[PCH_PDP_ADDR_OCTET-2]), PCH_PDP_ADDR_LEN_IPV4);
#ifndef UNDER_LINUX
		PCH_LOG_ARRAY("PCH:PdpApi_GetDecodedPDPAddress4:", outAddress->pdpAddressIPV4, PCH_PDP_ADDR_LEN_IPV4);			
#endif
	}
	else if(pdpAddressType == PCH_IPV6_ADDRESS)
	{
		memcpy(outAddress->pdpAddressIPV6, (UInt8 *)&(inAddressIE[PCH_PDP_ADDR_OCTET-2]), PCH_PDP_ADDR_LEN_IPV6);
		PCH_LOG_ARRAY("PCH:PdpApi_GetDecodedPDPAddress6:", outAddress->pdpAddressIPV6, PCH_PDP_ADDR_LEN_IPV6);	
	}	
	else if(pdpAddressType == PCH_IPV4V6_ADDRESS)
	{
		memcpy(outAddress->pdpAddressIPV4, (UInt8 *)&(inAddressIE[PCH_PDP_ADDR_OCTET-2]), PCH_PDP_ADDR_LEN_IPV4);
		PCH_LOG_ARRAY("PCH:PdpApi_GetDecodedPDPAddress4:", outAddress->pdpAddressIPV4, PCH_PDP_ADDR_LEN_IPV4);	
		memcpy(outAddress->pdpAddressIPV6, (UInt8 *)&(inAddressIE[PCH_PDP_ADDR_OCTET-2+4]), PCH_PDP_ADDR_LEN_IPV6);
		PCH_LOG_ARRAY("PCH:PdpApi_GetDecodedPDPAddress6:", outAddress->pdpAddressIPV6, PCH_PDP_ADDR_LEN_IPV6);	
	}
	else
		result = RESULT_ERROR;

	return result;
}

