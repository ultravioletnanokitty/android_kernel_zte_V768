/****************************************************************************
*
*     Copyright (c) 2009 Broadcom Corporation
*
*   Unless you and Broadcom execute a separate written software license 
*   agreement governing use of this software, this software is licensed to you 
*   under the terms of the GNU General Public License version 2, available 
*    at http://www.gnu.org/licenses/old-licenses/gpl-2.0.html (the "GPL"). 
*
*   Notwithstanding the above, under no circumstances may you combine this 
*   software in any way with any other Broadcom software provided under a license 
*   other than the GPL, without Broadcom's express prior written consent.
*
****************************************************************************/
#include "bcm_kril_common.h"
#include "bcm_kril_capi2_handler.h"
#include "bcm_kril_cmd_handler.h"
#include "bcm_kril_ioctl.h"

#include "capi2_pch_api.h"
#include "capi2_stk_ds.h"
#include "capi2_pch_msg.h"
#include "capi2_gen_msg.h"
#include "capi2_reqrep.h"
#include "capi2_gen_api.h"

#define FILTER_GSM_SIGNAL_LEVEL 16

extern MSRegInfo_t  gRegInfo[DUAL_SIM_SIZE];
extern MSUe3gStatusInd_t  gUE3GInfo[DUAL_SIM_SIZE];
extern MSRadioActivityInd_t  gRadioAcTInfo[DUAL_SIM_SIZE];

// for SIMID
extern SimNumber_t SIMEXCHID[DUAL_SIM_SIZE];

Boolean gNeedToAbortBeforeAutoSelect = FALSE;

UInt8 FindDunPdpCid(SimNumber_t SimId);
UInt8 ReleaseDunPdpContext(SimNumber_t SimId, UInt8 cid);
void Kpdp_pdp_deactivate_ind(int cid);

void ProcessGSMStatus(Kril_CAPI2Info_t* data);

// The cause of register denied
static const UInt8 g_DeniedCause[] =
{
    -1,
    23,
    2,
    3,
    6,
    11,
    12,
    13,
    15,
    17
};
#define NUM_DENIEDCAUSE (sizeof(g_DeniedCause) / sizeof(int))

// Support Band Mode for System
BandSelect_t g_systemband = BAND_NULL;
//WCDMA-IMT-2000 in Android's context means WCDMA 2100Mhz
static UInt32 g_bandMode[] =
{
    BAND_AUTO, // "selected by baseband automatically"
    BAND_NULL, //BAND_GSM900_ONLY|BAND_DCS1800_ONLY|BAND_UMTS2100_ONLY, // "EURO band" (GSM-900 / DCS-1800 / WCDMA-IMT-2000)
    BAND_NULL, //BAND_GSM850_ONLY|BAND_PCS1900_ONLY|BAND_UMTS850_ONLY|BAND_UMTS1900_ONLY, //"US band"
    BAND_NULL, //BAND_UMTS2100_ONLY, // "JPN band" (WCDMA-800 / WCDMA-IMT-2000)
    BAND_NULL, //BAND_GSM900_ONLY|BAND_DCS1800_ONLY|BAND_UMTS850_ONLY|BAND_UMTS2100_ONLY,  // "AUS band" (GSM-900 / DCS-1800 / WCDMA-850 / WCDMA-IMT-2000)
    BAND_NULL, //BAND_GSM900_ONLY|BAND_DCS1800_ONLY|BAND_UMTS850_ONLY, //"AUS band 2"
    BAND_NULL,  // "Cellular (800-MHz Band)"
    BAND_NULL, // "PCS (1900-MHz Band)" // Android MMI does not support
//band as below is CDMA mode
    BAND_NULL, // "Band Class 3 (JTACS Band)"
    BAND_NULL, // "Band Class 4 (Korean PCS Band)"(Tx = 1750-1780MHz)
    BAND_NULL, // "Band Class 5 (450-MHz Band)"
    BAND_NULL, // "Band Class 6 (2-GMHz IMT2000 Band)"
    BAND_NULL, // "Band Class 7 (Upper 700-MHz Band)"
    BAND_NULL, // "Band Class 8 (1800-MHz Band)"
    BAND_NULL, // "Band Class 9 (900-MHz Band)"
    BAND_NULL, // "Band Class 10 (Secondary 800-MHz Band)"
    BAND_NULL, // "Band Class 11 (400-MHz European PAMR Band)"
    BAND_NULL, // "Band Class 15 (AWS Band)"
    BAND_NULL, // "Band Class 16 (US 2.5-GHz Band)"
};
#define NUM_BANDMODE (sizeof(g_bandMode) / sizeof(UInt32))

//******************************************************************************
// Function Name: ConvertNetworkType
//
// Description: Convert RIL network type value to equivalent RATSelect_t
//
//******************************************************************************
RATSelect_t ConvertNetworkType(int type)
{
    if (0 == type || 0xFF == type) // 0xFF is default value, so also use DUAL_MODE_UMTS_PREF for default
    {
        return DUAL_MODE_UMTS_PREF;
    }
    else if (1 == type)
    {
        return GSM_ONLY;
    }
    else if (2 == type)
    {
        return UMTS_ONLY;
    }
    else
    {
        return INVALID_RAT;
    }
}

//******************************************************************************
// Function Name: ConvertRATType
//
// Description: Convert the RAT type(RATSelect_t) value to equivalent set of Android values
//
//******************************************************************************
int ConvertRATType(RATSelect_t type)
{
    if(type == GSM_ONLY)
    {
        return 1;
    }
    else  if(type == UMTS_ONLY)
    {
        return 2;
    }
    else  if(type == DUAL_MODE_GSM_PREF || type == DUAL_MODE_UMTS_PREF)
    {
        return 0;
    }
    else
    {
        return 0xFF;
    }
}

//******************************************************************************
// Function Name: ConvertBandMode
//
// Description: Convert RIL band mode value to equivalent set of BandSelect_t values
//
//******************************************************************************
BandSelect_t ConvertBandMode(int mode)
{
    if (mode >= 0 &&
        mode < NUM_BANDMODE)
    {
        return (g_bandMode[mode] & g_systemband);
    }
    else
    {
        return BAND_NULL;
    }
}

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

//******************************************************************************
// Function Name: MS_ConvertRawPlmnToHer	
//
// Description: 
//******************************************************************************
void MS_ConvertRawPlmnToHer(UInt16 mcc, UInt16 mnc, UInt8 *plmn_str, UInt8 plmn_str_sz)
{
    unsigned short mcc_code = MS_PlmnConvertRawMcc( mcc );
    unsigned short mnc_code = MS_PlmnConvertRawMnc( mcc, mnc );

    if( (mcc & 0x00F0) != 0x00F0 ) {
        sprintf((char *) plmn_str, "%03X%03X", mcc_code, mnc_code);
    }
    else {
        sprintf((char *) plmn_str, "%03X%02X", mcc_code, mnc_code);
    }
}

void BCMSetSupportedRATandBand(KRIL_CmdList_t *pdata)
{
    KrilSetPreferredNetworkType_t *tdata = (KrilSetPreferredNetworkType_t *)pdata->ril_cmd->data;
    KRIL_DEBUG(DBG_INFO, "SimId:%d networktype1:%d networktype2:%d band:%d\n", pdata->ril_cmd->SimId, tdata->networktype, tdata->networktypeEx, tdata->band);
    KRIL_SetPreferredNetworkType(SIM_DUAL_FIRST, tdata->networktype);
    KRIL_SetPreferredNetworkType(SIM_DUAL_SECOND, tdata->networktypeEx);
    CAPI2_NetRegApi_SetSupportedRATandBand(InitClientInfo(pdata->ril_cmd->SimId), ConvertNetworkType(tdata->networktype), ConvertBandMode(tdata->band), ConvertNetworkType(tdata->networktypeEx), ConvertBandMode(tdata->band));
    pdata->handler_state = BCM_RESPCAPI2Cmd;
}

void KRIL_SignalStrengthHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilSignalStrength_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(KrilSignalStrength_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                ((KrilSignalStrength_t*)pdata->bcm_ril_rsp)->RAT = gRegInfo[pdata->ril_cmd->SimId].netInfo.rat;
                KRIL_DEBUG(DBG_INFO, "SignalStrengthHandler::presult:%d\n", ((KrilSignalStrength_t*)pdata->bcm_ril_rsp)->RAT);
                CAPI2_PhoneCtrlApi_GetRxSignalInfo(InitClientInfo(pdata->ril_cmd->SimId));
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }

            if(NULL != capi2_rsp->dataBuf)
            {
                MsRxLevelData_t* presult = (MsRxLevelData_t*) capi2_rsp->dataBuf;
                KrilSignalStrength_t *rdata = (KrilSignalStrength_t *)pdata->bcm_ril_rsp;

                rdata->RxLev = presult->RxLev;
                rdata->RxQual = presult->RxQual;
                KRIL_DEBUG(DBG_INFO, "rdata->RAT:%d RxLev:%d RxQual:%d\n", rdata->RAT, rdata->RxLev, rdata->RxQual);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_RegistationStateHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilRegState_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(KrilRegState_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);    
                KRIL_DEBUG(DBG_TRACE, "handler state:%lu\n", pdata->handler_state);
                CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_NETWORK_ELEM_REGSTATE_INFO);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if(capi2_rsp->result != RESULT_OK)
            {
                KRIL_DEBUG(DBG_ERROR, "Error result:0x%x\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }

            if(NULL != capi2_rsp->dataBuf)
            {
                KrilRegState_t *rdata = (KrilRegState_t *)pdata->bcm_ril_rsp;
                MSRegStateInfo_t* presult = NULL; 
                CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;
                presult = (MSRegStateInfo_t*)(&(rsp->data_u));

                rdata->gsm_reg_state = presult->gsm_reg_state;

                /* GCF TC 26.7.4.3.4 Step 81, MO call attempt blocked by Android UI
                 * Refer to Android framework: GsmServiceStateTracker.handlePollStateResult(.) and GsmServiceStateTracker.regCodeToServiceState(.)
                 */
                if (REG_STATE_LIMITED_SERVICE == rdata->gsm_reg_state  &&
                    GSMCAUSE_REG_NO_ACCESS_MAX_ATTEMPT == gRegInfo[pdata->ril_cmd->SimId].cause)
                {
                    rdata->gsm_reg_state = REG_STATE_NORMAL_SERVICE;
                }

                rdata->gprs_reg_state = presult->gprs_reg_state;
                rdata->mcc = presult->mcc;
                rdata->mnc = presult->mnc;
                rdata->band = presult->band;
                rdata->lac = presult->lac;
                rdata->cell_id = presult->cell_id;
                gRegInfo[pdata->ril_cmd->SimId].netInfo.rat = presult->rat;

                if (presult->gsm_reg_state != REG_STATE_NORMAL_SERVICE
                    && presult->gsm_reg_state != REG_STATE_ROAMING_SERVICE
                    && presult->gsm_reg_state != REG_STATE_LIMITED_SERVICE
                    && presult->gprs_reg_state != REG_STATE_NORMAL_SERVICE
                    && presult->gprs_reg_state != REG_STATE_ROAMING_SERVICE
                    && presult->gprs_reg_state != REG_STATE_LIMITED_SERVICE)
                {
                    rdata->network_type = 0; //Unknown
                }
                else if (presult->rat == RAT_UMTS)
                {
                    if (SUPPORTED == gRegInfo[pdata->ril_cmd->SimId].netInfo.hsdpa_supported ||
                        TRUE == gUE3GInfo[pdata->ril_cmd->SimId].in_uas_conn_info.hsdpa_ch_allocated)
                    {
                        rdata->network_type = 9; //HSDPA
                    }
                    else if (SUPPORTED == gRegInfo[pdata->ril_cmd->SimId].netInfo.hsupa_supported)
                    {
                        rdata->network_type = 10; //HSUPA
                    }
                    else
                    {
                        rdata->network_type = 3; //UMTS
                    }
                }
                else if (presult->rat == RAT_GSM)
                {
                    if (SUPPORTED == gRegInfo[pdata->ril_cmd->SimId].netInfo.egprs_supported)
                    {
                        rdata->network_type = 2; //EDGE
                    }
                    else
                    {
                        rdata->network_type = 1; //GPRS
                    }
                }
                else
                {
                    rdata->network_type = 0; //Unknown
                }

                if(REG_STATE_LIMITED_SERVICE == presult->gsm_reg_state) // set denied cause
                {
                    rdata->cause = 0;
                    KRIL_DEBUG(DBG_INFO, "cause:%d netCause:%d\n",  gRegInfo[pdata->ril_cmd->SimId].cause, gRegInfo[pdata->ril_cmd->SimId].netCause);
                    /* GCF TC 26.7.4.3.4 Step 81 & TC 26.7.4.3.3, MO call attempt blocked by Android UI
                      * Refer to Android framework: GsmServiceStateTracker.handlePollStateResult(.) and GsmServiceStateTracker.regCodeToServiceState(.)
                      */
                    if (GSMCAUSE_REG_NO_ACCESS_MAX_ATTEMPT == gRegInfo[pdata->ril_cmd->SimId].cause || 
                        GSMCAUSE_REG_NO_ACCESS == gRegInfo[pdata->ril_cmd->SimId].cause)
                    {
                        rdata->cause = 10; // Persistent location update reject
                    }
                    else
                    {
                        UInt8 i;
                        for(i = 0 ; i < NUM_DENIEDCAUSE ; i++)
                        {
                            if(g_DeniedCause[i] == gRegInfo[pdata->ril_cmd->SimId].netCause)
                            {
                                rdata->cause = i;
                                break;
                            }
                        }
                    }
                }

                KRIL_DEBUG(DBG_INFO, "regstate:%d gprs_reg_state:%d mcc:ox%x mnc:0x%x rat:%d lac:%d cell_id:%d network_type:%d band:%d cause:%d\n", rdata->gsm_reg_state, rdata->gprs_reg_state, rdata->mcc, rdata->mnc, presult->rat, rdata->lac, rdata->cell_id, rdata->network_type, rdata->band, rdata->cause);

				if (presult->rat == RAT_UMTS)
				{
					CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_NETWORK_ELEM_RNC);
					pdata->handler_state = BCM_MS_GetRNC;
				}
				else
				{
					pdata->handler_state = BCM_FinishCAPI2Cmd;
				}
            }
            else
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

		case BCM_MS_GetRNC:
		{
			if(capi2_rsp->result != RESULT_OK)
            {
                KRIL_DEBUG(DBG_ERROR, "Error result:0x%x\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }

            if(NULL != capi2_rsp->dataBuf)
            {
                KrilRegState_t *rdata = (KrilRegState_t *)pdata->bcm_ril_rsp;
				UInt16* pRncValue = NULL;
                
                CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;
                pRncValue = (UInt16*)(&(rsp->data_u));
                rdata->cell_id += ((*pRncValue) << 16);

				KRIL_DEBUG(DBG_INFO, "rdata->cell_id:0x%x 0d%d RNC=0x%x\n", rdata->cell_id, rdata->cell_id, *pRncValue);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_OperatorHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilOperatorInfo_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(KrilOperatorInfo_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_NETWORK_ELEM_REGSTATE_INFO);
                pdata->handler_state = BCM_MS_GetRegistrationInfo;
            }
        }
        break;

        case BCM_MS_GetRegistrationInfo:
        {
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
            else
            {
                CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;
                MSRegStateInfo_t* presult = NULL; 
                if ( rsp && (rsp->inElemType == MS_NETWORK_ELEM_REGSTATE_INFO) )
                {
                    presult = (MSRegStateInfo_t*)(&(rsp->data_u));
                }
                else
                {
                    KRIL_DEBUG(DBG_ERROR,"unexpected response retrieving CAPI2_MS_GetElement !!\n");
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                    break;
                }
                if (presult->gsm_reg_state != REG_STATE_NORMAL_SERVICE && presult->gsm_reg_state != REG_STATE_ROAMING_SERVICE)
                {
                    pdata->result = BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW;
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
                else
                {
                    KrilOperatorInfo_t *rdata = (KrilOperatorInfo_t *)pdata->bcm_ril_rsp;
                    unsigned char oper_str[7];
                    KRIL_DEBUG(DBG_INFO, "mcc:%d mnc:%d lac:%d\n",presult->mcc, presult->mnc, presult->lac);
                    MS_ConvertRawPlmnToHer(presult->mcc, presult->mnc, oper_str, 7);
                    strcpy(rdata->numeric, oper_str);
                    CAPI2_NetRegApi_GetPLMNNameByCode(InitClientInfo(pdata->ril_cmd->SimId), presult->mcc, presult->mnc, presult->lac, FALSE);
                    pdata->handler_state = BCM_RESPCAPI2Cmd;
                }
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if ( MSG_MS_PLMN_NAME_RSP == capi2_rsp->msgType )
            {
                CAPI2_NetRegApi_GetPLMNNameByCode_Rsp_t* nameResult = (CAPI2_NetRegApi_GetPLMNNameByCode_Rsp_t*)capi2_rsp->dataBuf;
                if ( capi2_rsp->dataLength == sizeof(CAPI2_NetRegApi_GetPLMNNameByCode_Rsp_t) )
                {
                    if ( nameResult->val )
                    {
                        KrilOperatorInfo_t *rdata = (KrilOperatorInfo_t *)pdata->bcm_ril_rsp;
                        KRIL_DEBUG(DBG_INFO, "long coding:%d nameType:%d short_name coding:%d nameType:%d\n", nameResult->long_name.coding, nameResult->long_name.nameType, nameResult->short_name.coding, nameResult->short_name.nameType); //PLMN_NAME_t
                        KRIL_DEBUG(DBG_INFO, "longName:%s size %d shortName:%s size %d\n", nameResult->long_name.name, nameResult->long_name.name_size, nameResult->short_name.name, nameResult->short_name.name_size);
                        rdata->longnamecoding = nameResult->long_name.coding;
                        rdata->shortnamecoding = nameResult->short_name.coding;
                        rdata->longnamelen = (nameResult->long_name.name_size < PLMN_LONG_NAME)?nameResult->long_name.name_size:PLMN_LONG_NAME;
                        rdata->shortnamelen = (nameResult->short_name.name_size < PLMN_SHORT_NAME)?nameResult->short_name.name_size:PLMN_SHORT_NAME;
                        memcpy(rdata->longname, nameResult->long_name.name, (nameResult->long_name.name_size < PLMN_LONG_NAME)?nameResult->long_name.name_size:PLMN_LONG_NAME);
                        memcpy(rdata->shortname, nameResult->short_name.name, (nameResult->short_name.name_size < PLMN_SHORT_NAME)?nameResult->short_name.name_size:PLMN_SHORT_NAME);
                    }
                    else
                    {
                        // lookup failed, but don't return an error; if rdata->longname is empty, URIL
                        // will pass numeric string back to Android RIL as longname
                        KRIL_DEBUG(DBG_INFO, "CAPI2_MS_GetPLMNNameByCode result FALSE, just returning numeric...\n");
                    }
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
                }
                else
                {
                    KRIL_DEBUG(DBG_ERROR, "** MSG_MS_PLMN_NAME_RSP size mismatch got %ld expected %d\n", capi2_rsp->dataLength, sizeof(MsPlmnName_t) );
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "Unexpected response message for CAPI2_MS_GetPLMNNameByCode 0x%x tid %ld\n",capi2_rsp->msgType, capi2_rsp->tid);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

void KRIL_QueryNetworkSelectionModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilSelectionMode_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(KrilSelectionMode_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_PHCTRL_ELEM_PLMN_MODE);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            KrilSelectionMode_t *rdata = (KrilSelectionMode_t *)pdata->bcm_ril_rsp;
            CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*) capi2_rsp->dataBuf;
            PlmnSelectMode_t presult;
            memcpy(&presult, &(rsp->data_u), sizeof(PlmnSelectMode_t));
            // Android only "understands" manual (1) or auto (0) so map PlmnSelectMode_t to this
            if (PLMN_SELECT_AUTO == presult ||
               PLMN_SELECT_MANUAL_FORCE_AUTO == presult)
            {
                //automatic selection
                rdata->selection_mode = 0;
            }
            else if (PLMN_SELECT_MANUAL == presult)
            {
                //manual selection
                rdata->selection_mode = 1;
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "Error PLMN mode:%d...!\n", presult);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
            KRIL_DEBUG(DBG_INFO, "selection_mode:%d PlmnSelectMode:%d\n", rdata->selection_mode, presult);
            pdata->handler_state = BCM_FinishCAPI2Cmd;
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_SetNetworkSelectionAutomaticHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        KRIL_SetNetworkSelectTID(0);
        KRIL_SetInNetworkSelectHandler( FALSE );
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            // we're not re-entrant, so indicate to command thread that we're already handling 
            // a network select request 
            KRIL_SetInNetworkSelectHandler( TRUE );
            if (TRUE == gNeedToAbortBeforeAutoSelect) // Call MSG_MS_PLMN_ABORT_REQ to return RPLMN when automatic selection if previous manual selection fail, 
            {
                KRIL_DEBUG(DBG_INFO, "Calling CAPI2_NetRegApi_AbortPlmnSelect first\n");
                CAPI2_NetRegApi_AbortPlmnSelect(InitClientInfo(pdata->ril_cmd->SimId));
                gNeedToAbortBeforeAutoSelect = FALSE;
                pdata->handler_state = BCM_MS_AbortPlmnSelect;
            }
            else
            {
                KRIL_DEBUG(DBG_INFO, "Calling CAPI2_NetRegApi_PlmnSelect\n");
                CAPI2_NetRegApi_PlmnSelect(InitClientInfo(pdata->ril_cmd->SimId), FALSE, PLMN_SELECT_AUTO, PLMN_FORMAT_LONG, "");
                KRIL_SetNetworkSelectTID(GetTID());
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_MS_AbortPlmnSelect:
        {
            KRIL_DEBUG(DBG_INFO, "Calling CAPI2_NetRegApi_PlmnSelect::capi2 result:%d\n", capi2_rsp->result);
            CAPI2_NetRegApi_PlmnSelect(InitClientInfo(pdata->ril_cmd->SimId), FALSE, PLMN_SELECT_AUTO, PLMN_FORMAT_LONG, "");
            KRIL_SetNetworkSelectTID(GetTID());
            pdata->handler_state = BCM_RESPCAPI2Cmd;
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if ( MSG_PLMN_SELECT_RSP == capi2_rsp->msgType )
            {
                if ( RESULT_OK != capi2_rsp->result )
                {
                    KRIL_DEBUG(DBG_ERROR, "CAPI2_NetRegApi_PlmnSelect error result:%d, exiting handler\n", capi2_rsp->result);
                    pdata->result = RILErrorResult(capi2_rsp->result);
                    KRIL_SetNetworkSelectTID(0);
                    KRIL_SetInNetworkSelectHandler( FALSE );
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
                else
                {
                    // payload is a boolean indicating whether or not request was sent to the network; if not sent, then
                    // we've successfully completed the request; if it was sent, we need to wait for the MSG_PLMN_SELECT_CNF
                    // response...
                    Boolean bSent = *((Boolean*) capi2_rsp->dataBuf);
                    if ( !bSent )
                    {
                        KRIL_DEBUG(DBG_INFO, "CAPI2_NetRegApi_PlmnSelect successfully completed (not sent)\n");
                        KRIL_SetNetworkSelectTID(0);
                        KRIL_SetInNetworkSelectHandler( FALSE );
                        pdata->handler_state = BCM_FinishCAPI2Cmd;
                    }
                    else
                    {
                        // if we get to here, things are OK and we should just have to wait for the MSG_PLMN_SELECT_CNF msg
                        KRIL_DEBUG( DBG_INFO, "CAPI2_NetRegApi_PlmnSelect no error, waiting for MSG_PLMN_SELECT_CNF\n" );
                    }
                }
            }
            else if ( MSG_PLMN_SELECT_CNF ==  capi2_rsp->msgType )
            {
                UInt16 presult = *((UInt16*) capi2_rsp->dataBuf);
                KRIL_DEBUG(DBG_INFO, "MSG_PLMN_SELECT_CNF presult:%d\n", presult);
                if(OPERATION_SUCCEED == presult || NO_REJECTION == presult)
                {
                    KRIL_DEBUG(DBG_INFO, "Completed OK presult: %d\n", presult );
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
                }
                else
                {
                    KRIL_DEBUG(DBG_INFO, "PLMN select fail:%d", presult);
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
                KRIL_SetNetworkSelectTID(0);
                KRIL_SetInNetworkSelectHandler( FALSE );
            }
            else
            {
                KRIL_DEBUG( DBG_ERROR, "CAPI2_NetRegApi_PlmnSelect unexpected message 0x%x\n", capi2_rsp->msgType );
                KRIL_SetNetworkSelectTID(0);
                KRIL_SetInNetworkSelectHandler( FALSE );
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            KRIL_SetNetworkSelectTID(0);
            KRIL_SetInNetworkSelectHandler( FALSE );
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_SetNetworkSelectionManualHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            KrilManualSelectInfo_t *tdata = (KrilManualSelectInfo_t *)pdata->ril_cmd->data;
            // we're not re-entrant, so indicate to command thread that we're already handling 
            // a network select request 
            KRIL_SetInNetworkSelectHandler( TRUE );
            KRIL_DEBUG(DBG_INFO, "network_info:%s\n", tdata->networkInfo);
            CAPI2_NetRegApi_PlmnSelect(InitClientInfo(pdata->ril_cmd->SimId), FALSE, PLMN_SELECT_MANUAL, PLMN_FORMAT_NUMERIC, (char *)tdata->networkInfo);
            KRIL_SetNetworkSelectTID( GetTID() );
            pdata->handler_state = BCM_RESPCAPI2Cmd;
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            // we should first get a MSG_PLMN_SELECT_RSP response indicating that the request has been 
            // sent to the network, followed by a MSG_PLMN_SELECT_CNF message with the result from the network
            if ( MSG_PLMN_SELECT_RSP == capi2_rsp->msgType )
            {
                if ( RESULT_OK != capi2_rsp->result )
                {
                    // request failed before it was issued to network, so we shouldn't need to call abort here...
                    KRIL_DEBUG(DBG_ERROR, "CAPI2_NetRegApi_PlmnSelect capi2 error result:%d\n", capi2_rsp->result);
                    pdata->result = RILErrorResult(capi2_rsp->result);
                    KRIL_SetNetworkSelectTID( 0 );
                    KRIL_SetInNetworkSelectHandler( FALSE );
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
                else
                {
                    // payload is a boolean indicating whether or not request was sent to the network; if not sent, then
                    // we've successfully completed the request; if it was sent, we need to wait for the MSG_PLMN_SELECT_CNF
                    // response...
                    Boolean bSent = *((Boolean*) capi2_rsp->dataBuf);
                    if ( !bSent )
                    {
                        KRIL_DEBUG(DBG_INFO, "CAPI2_NetRegApi_PlmnSelect successfully completed (not sent)\n");
                        pdata->handler_state = BCM_FinishCAPI2Cmd;
                        KRIL_SetNetworkSelectTID( 0 );
                        KRIL_SetInNetworkSelectHandler( FALSE );
                    }
                    else
                    {
                        // if we get to here, things are OK and we should just have to wait for the MSG_PLMN_SELECT_CNF msg
                        KRIL_DEBUG( DBG_INFO, "CAPI2_NetRegApi_PlmnSelect no error, waiting for MSG_PLMN_SELECT_CNF\n" );
                    }
                }
            }
            else if ( MSG_PLMN_SELECT_CNF ==  capi2_rsp->msgType )
            {
                UInt16 presult = *((UInt16*) capi2_rsp->dataBuf);
                KRIL_DEBUG(DBG_INFO, "MSG_PLMN_SELECT_CNF presult:%d\n", presult);
                if(OPERATION_SUCCEED == presult || NO_REJECTION == presult)
                {
                    KRIL_DEBUG(DBG_INFO, "Completed OK presult: %d\n", presult);
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
                }
                else
                {
                    KRIL_DEBUG(DBG_ERROR, "error result:%d, need to call CAPI2_NetRegApi_AbortPlmnSelect before automatic PLMN selection...\n", presult);
                    gNeedToAbortBeforeAutoSelect = TRUE;

                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
                KRIL_SetNetworkSelectTID( 0 );
                KRIL_SetInNetworkSelectHandler( FALSE );
            }
            else
            {
                // unexpected message
                KRIL_DEBUG(DBG_ERROR, "unexpected message 0x%x\n", capi2_rsp->msgType);
                KRIL_SetNetworkSelectTID( 0 );
                KRIL_SetInNetworkSelectHandler( FALSE );
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_QueryAvailableNetworksHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilNetworkList_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(KrilNetworkList_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                if (RAT_UMTS == gRegInfo[pdata->ril_cmd->SimId].netInfo.rat) // check UAS information if RAT is 3G
                {
                    CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_NETWORK_ELEM_UAS_CONN_INFO);
                     pdata->handler_state = BCM_ELEM_UAS_INFO;
                }
                else if (RAT_GSM == gRegInfo[pdata->ril_cmd->SimId].netInfo.rat) // check radio activity information if RAT is 2G
                {
                    KRIL_DEBUG(DBG_INFO, "radio_direction:%d radio_status:%d cid:%d max_dl_data_rate:%ld\n",
                            gRadioAcTInfo[pdata->ril_cmd->SimId].radio_direction, gRadioAcTInfo[pdata->ril_cmd->SimId].radio_status,
                            gRadioAcTInfo[pdata->ril_cmd->SimId].cid, gRadioAcTInfo[pdata->ril_cmd->SimId].max_dl_data_rate);
                    if ((RADIO_BIDIRECTION == gRadioAcTInfo[pdata->ril_cmd->SimId].radio_direction &&
                         RADIO_OFF == gRadioAcTInfo[pdata->ril_cmd->SimId].radio_status) ||
                        (0 == gRadioAcTInfo[pdata->ril_cmd->SimId].radio_direction &&
                         0 == gRadioAcTInfo[pdata->ril_cmd->SimId].radio_status &&
                         0 == gRadioAcTInfo[pdata->ril_cmd->SimId].cid &&
                         0 == gRadioAcTInfo[pdata->ril_cmd->SimId].max_dl_data_rate)) // only check data transfer
                    {
                        CAPI2_NetRegApi_SearchAvailablePLMN(InitClientInfo(pdata->ril_cmd->SimId));
                        pdata->handler_state = BCM_Search_Available_PLMN;
                    }
                    else
                    {
                        KRIL_DEBUG(DBG_INFO, "mcc:%d mnc:%d lac:%d\n", gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, gRegInfo[pdata->ril_cmd->SimId].lac);
                        CAPI2_NetRegApi_GetPLMNNameByCode(InitClientInfo(pdata->ril_cmd->SimId), gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, gRegInfo[pdata->ril_cmd->SimId].lac, FALSE);
                        pdata->handler_state = BCM_RESPCAPI2Cmd;
                    }
                }
                else
                {
                    KRIL_DEBUG(DBG_ERROR, "RAT is not available!\n");
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
            }
        }
        break;

        case BCM_ELEM_UAS_INFO:
        {
            Uas_Conn_Info* presult = NULL;
            CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;
            presult = (Uas_Conn_Info*)(&(rsp->data_u));
            KRIL_DEBUG(DBG_INFO, "in_cell_dch_state:%d\n", presult->in_cell_dch_state);
            if (TRUE == presult->in_cell_dch_state)
            {
                KRIL_DEBUG(DBG_INFO, "mcc:%d mnc:%d lac:%d\n", gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, gRegInfo[pdata->ril_cmd->SimId].lac);
                CAPI2_NetRegApi_GetPLMNNameByCode(InitClientInfo(pdata->ril_cmd->SimId), gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, gRegInfo[pdata->ril_cmd->SimId].lac, FALSE);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            else
            {
                CAPI2_NetRegApi_SearchAvailablePLMN(InitClientInfo(pdata->ril_cmd->SimId));
                pdata->handler_state = BCM_Search_Available_PLMN;
            }
        }
        break;

        case BCM_Search_Available_PLMN:
        {
            SEARCHED_PLMN_LIST_t *rsp = (SEARCHED_PLMN_LIST_t *)capi2_rsp->dataBuf;
            UInt8 i, j;
            Boolean match = FALSE;
            KrilNetworkList_t *rdata = (KrilNetworkList_t *)pdata->bcm_ril_rsp;
            rdata->num_of_plmn = 0;
            KRIL_DEBUG(DBG_INFO, "BCM_Search_Available_PLMN::num_of_plmn:%d plmn_cause:%d\n", rsp->num_of_plmn, rsp->plmn_cause);
            if (PLMNCAUSE_ACTIVE_CALL == rsp->plmn_cause) // show home PLMN if in call
            {
                KRIL_DEBUG(DBG_INFO, "mcc:%d mnc:%d lac:%d\n", gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, gRegInfo[pdata->ril_cmd->SimId].lac);
                CAPI2_NetRegApi_GetPLMNNameByCode(InitClientInfo(pdata->ril_cmd->SimId), gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, gRegInfo[pdata->ril_cmd->SimId].lac, FALSE);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            else if (rsp->plmn_cause != PLMNCAUSE_SUCCESS)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                for(i = 0 ; i < rsp->num_of_plmn ; i++)
                {
                    for (j = 0 ; j < i ; j++)
                    {
                         KRIL_DEBUG(DBG_INFO, "BCM_Search_Available_PLMN::mcc[%d]:%d mcc[%d]:%d\n", i, rsp->searched_plmn[i].mcc, j, rsp->searched_plmn[j].mcc);
                         KRIL_DEBUG(DBG_INFO, "BCM_Search_Available_PLMN::mnc[%d]:%d mnc[%d]:%d\n", i, rsp->searched_plmn[i].mnc, j, rsp->searched_plmn[j].mnc);
                         if ((rsp->searched_plmn[i].mcc == rsp->searched_plmn[j].mcc) &&
                             (rsp->searched_plmn[i].mnc == rsp->searched_plmn[j].mnc))
                         {
                             match = TRUE;
                             break;
                         }
                         match = FALSE;
                    }
                    if (TRUE == match)
                    {
                        match = FALSE;
                        continue;
                    }
                    rdata->available_plmn[rdata->num_of_plmn].mcc = rsp->searched_plmn[i].mcc;
                    rdata->available_plmn[rdata->num_of_plmn].mnc = rsp->searched_plmn[i].mnc;
                    rdata->available_plmn[rdata->num_of_plmn].network_type = rsp->searched_plmn[i].network_type;
                    rdata->available_plmn[rdata->num_of_plmn].rat = rsp->searched_plmn[i].rat;
                    strncpy(rdata->available_plmn[rdata->num_of_plmn].longname, rsp->searched_plmn[i].nonUcs2Name.longName.name, rsp->searched_plmn[i].nonUcs2Name.longName.name_size);
                    strncpy(rdata->available_plmn[rdata->num_of_plmn].shortname, rsp->searched_plmn[i].nonUcs2Name.shortName.name, rsp->searched_plmn[i].nonUcs2Name.shortName.name_size);
                    rdata->num_of_plmn++;
                    KRIL_DEBUG(DBG_INFO, "BCM_Search_Available_PLMN::num_of_plmn:%d network_type:%d longname:%s shortname:%s\n", rdata->num_of_plmn, rdata->available_plmn[rdata->num_of_plmn].network_type, rdata->available_plmn[rdata->num_of_plmn].longname, rdata->available_plmn[rdata->num_of_plmn].shortname);
                }
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            CAPI2_NetRegApi_GetPLMNNameByCode_Rsp_t* nameResult = (CAPI2_NetRegApi_GetPLMNNameByCode_Rsp_t*)capi2_rsp->dataBuf;
            KrilNetworkList_t *rdata = (KrilNetworkList_t *)pdata->bcm_ril_rsp;

            rdata->num_of_plmn = 1;
            rdata->available_plmn[0].mcc = gRegInfo[pdata->ril_cmd->SimId].mcc;
            rdata->available_plmn[0].mnc = gRegInfo[pdata->ril_cmd->SimId].mnc;
            rdata->available_plmn[0].network_type = MSNWTYPE_HOME_PLMN;
            rdata->available_plmn[0].rat = gRegInfo[pdata->ril_cmd->SimId].netInfo.rat;

            if (TRUE == nameResult->val)
            {
                KRIL_DEBUG(DBG_INFO, "longName:%s size %d shortName:%s %d\n", nameResult->long_name.name,nameResult->long_name.name_size, nameResult->short_name.name,nameResult->short_name.name_size);
                memcpy(rdata->available_plmn[0].longname, nameResult->long_name.name, (nameResult->long_name.name_size < PLMN_LONG_NAME)?nameResult->long_name.name_size:PLMN_LONG_NAME );
                memcpy(rdata->available_plmn[0].shortname, nameResult->short_name.name, (nameResult->short_name.name_size < PLMN_SHORT_NAME)?nameResult->short_name.name_size:PLMN_SHORT_NAME);
            }
            else
            {
                unsigned char oper_str[7];
                KRIL_DEBUG(DBG_INFO, "mcc:%d mnc:%d\n", gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc);
                MS_ConvertRawPlmnToHer(gRegInfo[pdata->ril_cmd->SimId].mcc, gRegInfo[pdata->ril_cmd->SimId].mnc, oper_str, 7);
                memcpy(rdata->available_plmn[0].longname, oper_str, (nameResult->long_name.name_size < PLMN_LONG_NAME)?nameResult->long_name.name_size:PLMN_LONG_NAME );
                memcpy(rdata->available_plmn[0].shortname, oper_str, (nameResult->short_name.name_size < PLMN_SHORT_NAME)?nameResult->short_name.name_size:PLMN_SHORT_NAME);
            }
            pdata->handler_state = BCM_FinishCAPI2Cmd;
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_SetBandModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            KrilBandModeInfo_t *tdata = (KrilBandModeInfo_t *)pdata->ril_cmd->data;
            if (tdata->bandmode >= NUM_BANDMODE || BAND_NULL == g_bandMode[tdata->bandmode])
            {
                KRIL_DEBUG(DBG_ERROR, "CP not support the bandmode:%d\n", tdata->bandmode);
                pdata->result = BCM_E_MODE_NOT_SUPPORTED;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                KRIL_DEBUG(DBG_INFO, "g_bandMode:%ld g_systemband:%d combine:%d\n", g_bandMode[tdata->bandmode], g_systemband, ConvertBandMode(tdata->bandmode));
                CAPI2_NetRegApi_SelectBand(InitClientInfo(pdata->ril_cmd->SimId), ConvertBandMode(tdata->bandmode));
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;
        case BCM_RESPCAPI2Cmd:
        {
            if (capi2_rsp->result != RESULT_OK)
            {
                KRIL_DEBUG(DBG_ERROR, "result :%d error...!\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_QueryAvailableBandModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilAvailableBandMode_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(KrilAvailableBandMode_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_NETREG_ELEM_SYSTEM_BAND);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if (capi2_rsp->result != RESULT_OK)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                KrilAvailableBandMode_t *rdata = (KrilAvailableBandMode_t *)pdata->bcm_ril_rsp;
                int index, index1 = 0;
                int tempband[18];
                CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*) capi2_rsp->dataBuf;
                BandSelect_t SystemBand;
                memcpy(&SystemBand, &(rsp->data_u), sizeof(BandSelect_t));

                SystemBand = SystemBand | BAND_AUTO;
                g_systemband = SystemBand;
                for (index = 0 ; index < NUM_BANDMODE ; index++)
                {
                    if(g_bandMode[index] & SystemBand)
                    {
                        tempband[index1] = index;
                        KRIL_DEBUG(DBG_INFO, "g_bandMode[%d]:%ld tempband[%d]:%d\n", index, g_bandMode[index], index1, tempband[index1]);
                        index1++;
                    }
                }
                rdata->band_mode[0] = (index1+1);
                KRIL_DEBUG(DBG_INFO, "band_mode length:%d\n", rdata->band_mode[0]);
                for (index = 0 ; (index < rdata->band_mode[0]-1) ; index++)
                {
                    rdata->band_mode[index+1] = tempband[index];
                    KRIL_DEBUG(DBG_INFO, "rdata->band_mode[%d]:%d\n",index+1, rdata->band_mode[index+1]);
                }
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_SetPreferredNetworkTypeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
    {
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::msgType:%d result:%d\n", pdata->handler_state, capi2_rsp->msgType, capi2_rsp->result);
        if(capi2_rsp->result != RESULT_OK)
        {
            pdata->result = RILErrorResult(capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            KRIL_SetInSetPrferredNetworkTypeHandler(FALSE);
            return;
        }
    }

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            KrilSetPreferredNetworkType_t *tdata = (KrilSetPreferredNetworkType_t *)pdata->ril_cmd->data;
            Boolean isDualSim;
            KRIL_SetInSetPrferredNetworkTypeHandler(TRUE);
            if (SIM_SINGLE == pdata->ril_cmd->SimId)
                isDualSim = FALSE;
            else
                isDualSim = TRUE;

            if (0xFF == KRIL_GetPreferredNetworkType(pdata->ril_cmd->SimId))
            {
                CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_NETREG_ELEM_SUPPORTED_RAT);
                pdata->handler_state = BCM_NET_QueryCurrentRAT;
            }
            else if (((!isDualSim) &&
                      tdata->networktype == KRIL_GetPreferredNetworkType(SIM_SINGLE)) ||
                     (isDualSim &&
                      tdata->networktype == KRIL_GetPreferredNetworkType(SIM_DUAL_FIRST) &&
                      tdata->networktypeEx == KRIL_GetPreferredNetworkType(SIM_DUAL_SECOND)))
            {
                KRIL_SetInSetPrferredNetworkTypeHandler(FALSE);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                if (tdata->networktype != KRIL_GetPreferredNetworkType(SIM_DUAL_FIRST) &&
                    tdata->networktypeEx != KRIL_GetPreferredNetworkType(SIM_DUAL_SECOND))
                {
                    KRIL_DEBUG(DBG_INFO, "SimId:%d networktype1:%d networktype2:%d band:%d\n", pdata->ril_cmd->SimId, tdata->networktype, tdata->networktypeEx, tdata->band);
                    CAPI2_NetRegApi_SetSupportedRATandBand(InitClientInfo(pdata->ril_cmd->SimId), ConvertNetworkType(1), ConvertBandMode(0), ConvertNetworkType(1), ConvertBandMode(0)); // set RAT 2G/2G and band NULL
                    KRIL_SetPreferredNetworkTID(GetTID());
                    pdata->handler_state = BCM_NET_SendChangeRAT;
                }
                else
                {
                    BCMSetSupportedRATandBand(ril_cmd);
                }
            }
        }
        break;

        case BCM_NET_QueryCurrentRAT:
        {
            CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;
            KrilSetPreferredNetworkType_t *tdata = (KrilSetPreferredNetworkType_t *)pdata->ril_cmd->data;
            RATSelect_t data;
            int networktype;

            memcpy(&data, &(rsp->data_u), sizeof(RATSelect_t));
            if (SIM_DUAL_SECOND == pdata->ril_cmd->SimId)
                networktype = tdata->networktypeEx;
            else
                networktype = tdata->networktype;

            if (networktype == ConvertRATType(data)) // RAT is same
            {
                KRIL_SetInSetPrferredNetworkTypeHandler(FALSE);
                KRIL_SetPreferredNetworkType(SIM_DUAL_FIRST, tdata->networktype);
                KRIL_SetPreferredNetworkType(SIM_DUAL_SECOND, tdata->networktypeEx);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                if (tdata->networktype != 1 || tdata->networktypeEx != 1) // if the preferred RAT are not both 2G, set 2G/2G first.
                {
                    KRIL_DEBUG(DBG_INFO, "SimId:%d networktype1:%d networktype2:%d band:%d\n", pdata->ril_cmd->SimId, tdata->networktype, tdata->networktypeEx, tdata->band);
                    CAPI2_NetRegApi_SetSupportedRATandBand(InitClientInfo(pdata->ril_cmd->SimId), ConvertNetworkType(1), ConvertBandMode(0), ConvertNetworkType(1), ConvertBandMode(0)); // set RAT 2G/2G and band NULL
                    KRIL_SetPreferredNetworkTID(GetTID());
                    pdata->handler_state = BCM_NET_SendChangeRAT;
                }
                else
                {
                    BCMSetSupportedRATandBand(ril_cmd);
                }
            }
        }
        break;

        case BCM_NET_SendChangeRAT:
        {
            if (MSG_MS_SET_RAT_BAND_RSP == capi2_rsp->msgType)
            {
                KRIL_DEBUG(DBG_INFO, "handler MSG_MS_SET_RAT_BAND_RSP msgtype:0x%x tid:%ld...!\n", capi2_rsp->msgType, capi2_rsp->tid);
                //luobiao@wind-mobi.com start 2012.05.17
				//patch for csp:524605
				if ( KRIL_GetFlightModeState(SIMEXCHID[pdata->ril_cmd->SimId]) )
                {
                    KRIL_SetPreferredNetworkTID(0);
                    BCMSetSupportedRATandBand(ril_cmd);
                }
				//luobiao@wind-mobi.com end 2012.05.17
            }
            else if (MSG_REG_GSM_IND == capi2_rsp->msgType)
            {
                KRIL_DEBUG(DBG_INFO, "MSG_REG_GSM_IND SimId:%d...!\n", capi2_rsp->SimId);
                if (SIMEXCHID[pdata->ril_cmd->SimId] == capi2_rsp->SimId)
                {
                    MSRegInfo_t *pMSRegInfo = (MSRegInfo_t*)capi2_rsp->dataBuf;
                    KRIL_DEBUG(DBG_INFO, "MSG_REG_GSM_IND rat:%d...!\n", pMSRegInfo->netInfo.rat);
                    if (RAT_GSM == pMSRegInfo->netInfo.rat)
                    {
                        KRIL_SetPreferredNetworkTID(0);
                        BCMSetSupportedRATandBand(ril_cmd);
                    }
                }
                ProcessGSMStatus(capi2_rsp);
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "Receive wrong msgtype:0x%x in KRIL_SetPreferredNetworkTypeHandler()...!\n", capi2_rsp->msgType);
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if (capi2_rsp->result != RESULT_OK)
            {
                KRIL_DEBUG(DBG_ERROR, "handler result:%d error...!\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            KRIL_SetInSetPrferredNetworkTypeHandler(FALSE);
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_GetPreferredNetworkTypeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(krilGetPreferredNetworkType_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;             
            }
            else
            {
                pdata->rsp_len = sizeof(krilGetPreferredNetworkType_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_NETREG_ELEM_SUPPORTED_RAT);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if (capi2_rsp->result != RESULT_OK)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                krilGetPreferredNetworkType_t *rdata = (krilGetPreferredNetworkType_t *)pdata->bcm_ril_rsp;
                CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*) capi2_rsp->dataBuf;
                RATSelect_t RAT;
                memcpy(&RAT, &(rsp->data_u), sizeof(RATSelect_t));

                if (0xFF == ConvertRATType(RAT))
                {
                    KRIL_DEBUG(DBG_ERROR, "RAT:%d\n", RAT);
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                    return;
                }
                rdata->network_type = ConvertRATType(RAT);
                KRIL_DEBUG(DBG_INFO, "BCM_RESPCAPI2Cmd:: network_type:%d RAT:%d\n", rdata->network_type, RAT);
                KRIL_SetPreferredNetworkType(pdata->ril_cmd->SimId, rdata->network_type);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_GetNeighboringCellIDsHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::msgType:%d result:%d\n", pdata->handler_state, capi2_rsp->msgType, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            if ((gRegInfo[pdata->ril_cmd->SimId].regState != REG_STATE_NORMAL_SERVICE) &&
                (gRegInfo[pdata->ril_cmd->SimId].regState != REG_STATE_ROAMING_SERVICE))
            {
                pdata->result = BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
            // can only handle one of these requests at a time...
            KRIL_SetInNeighborCellHandler( TRUE );
            CAPI2_MsDbApi_SYS_EnableCellInfoMsg(InitClientInfo(pdata->ril_cmd->SimId), TRUE);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if (capi2_rsp->result != RESULT_OK)
            {
                CAPI2_MsDbApi_SYS_EnableCellInfoMsg(InitClientInfo(pdata->ril_cmd->SimId), FALSE);
                pdata->handler_state = BCM_SYS_EnableCellInfoMsg;
            }
            else
            {
                if (MSG_SERVING_CELL_INFO_IND == capi2_rsp->msgType)
                {
                    if (NULL == capi2_rsp->dataBuf)
                    {
                        KRIL_DEBUG(DBG_ERROR, "dataBuf is NULL\n");
                        KRIL_ASSERT(1);
                        goto DisableCellInfo;
                    }
                    ServingCellInfo_t *rsp = (ServingCellInfo_t *)capi2_rsp->dataBuf;
                    krilGetNeighborCell_t *rdata;
                    UInt8 loop = 0;
                    UInt32 count = 0, alloc_count=0;
                    UInt8 *temp_rsp = NULL ;
                    KRIL_DEBUG(DBG_ERROR, "mRAT:%d address:0x%p\n", rsp->mRAT, &rsp->mRAT);
                    if (RAT_UMTS == rsp->mRAT)
                    {
                        // 3G Received Signal Code Power Range: -121 to -25
                        const Int8 rscpBottom = -121;
                        const Int8 rscpTop = -25;

                        KRIL_DEBUG(DBG_INFO, "num_umts_freq:%d \n", rsp->mLbsParams.lbs_umts_params.num_umts_freq);
                        if (rsp->mLbsParams.lbs_umts_params.num_umts_freq > LBS_MAX_NUM_UMTS_FREQ)
                        {
                            KRIL_ASSERT(1);
                            goto DisableCellInfo;
                        }
                        for (loop = 0 ; loop < rsp->mLbsParams.lbs_umts_params.num_umts_freq ; loop++)
                        {
                            UInt8 loop1 = 0;
                            KRIL_DEBUG(DBG_INFO, "num_cell:%d\n", rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].num_cell);
                            if (rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].num_cell > LBS_MAX_NUM_CELL_PER_FREQ)
                            {
                                KRIL_DEBUG(DBG_ERROR, "loop:%d num_cell:%d\n", loop, rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].num_cell);
                                KRIL_ASSERT(1);
                                goto DisableCellInfo;
                            }
                            for (loop1 = 0 ; loop1 < rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].num_cell ; loop1++)
                            {
                                if (rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].psc != 0 &&
                                    rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].rscp >= rscpBottom &&
                                    rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].rscp <= rscpTop)
                                {
                                    alloc_count++;
                                }
                            }
                        }
                        KRIL_DEBUG(DBG_INFO, "count:%ld \n", alloc_count);
                        temp_rsp = kmalloc(sizeof(krilGetNeighborCell_t)*alloc_count, GFP_KERNEL);
                        if(!temp_rsp) {
                            KRIL_DEBUG(DBG_ERROR, "unable to allocate temp_rsp buf\n");
                            KRIL_ASSERT(1);
                            goto DisableCellInfo;
                        }
                        rdata = (krilGetNeighborCell_t *)temp_rsp;
                        memset(temp_rsp, 0,sizeof(krilGetNeighborCell_t)*alloc_count);
                        count = 0;
                        for (loop = 0 ; loop < rsp->mLbsParams.lbs_umts_params.num_umts_freq ; loop++)
                        {
                            UInt8 loop1 = 0;
                            KRIL_DEBUG(DBG_INFO, "num_cell:%d\n", rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].num_cell);
                            for (loop1 = 0 ; loop1 < rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].num_cell ; loop1++)
                            {
                                if (rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].psc != 0 &&
                                    rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].rscp >= rscpBottom &&
                                    rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].rscp <= rscpTop)
                                {
                                    KRIL_DEBUG(DBG_INFO, "psc:0x%x rscp:%d\n", rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].psc, rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].rscp);
                                    if(count >= alloc_count || count >= LBS_MAX_NUM_UMTS_FREQ*LBS_MAX_NUM_CELL_PER_FREQ)
                                    {
                                        kfree(temp_rsp);
                                        pdata->rsp_len = 0;
                                        KRIL_DEBUG(DBG_ERROR, "count:%ld alloc_count:%ld\n", count, alloc_count);
                                        KRIL_ASSERT(1);
                                        goto DisableCellInfo;
                                    }
                                    rdata[count].cid = rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].psc;
                                    rdata[count].rssi = rsp->mLbsParams.lbs_umts_params.umts_freqs[loop].cells[loop1].rscp;
                                    KRIL_DEBUG(DBG_INFO, "loop:%d cid:0x%x rssi:%d\n", loop, rdata[count].cid, rdata[count].rssi);
                                    count++;

                                }
                            }
                        }
                        pdata->rsp_len = sizeof(krilGetNeighborCell_t)*count;
                        pdata->bcm_ril_rsp = kmalloc(pdata->rsp_len, GFP_KERNEL);
                        if(!pdata->bcm_ril_rsp) {
                            KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");
                        }
                        else
                        {
                            memcpy(pdata->bcm_ril_rsp, temp_rsp, pdata->rsp_len);
                        }
                        if (temp_rsp != NULL)
                            kfree(temp_rsp);
                    }
                    else if (RAT_GSM == rsp->mRAT)
                    {
                        KRIL_DEBUG(DBG_INFO, "num_gsm_ncells:%d \n", rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells);
                        if(rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells > LBS_MAX_NUM_GSM_NCELL)
                        {
                            KRIL_ASSERT(1);
                            goto DisableCellInfo;
                        }
                        temp_rsp = kmalloc(sizeof(krilGetNeighborCell_t)*rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells, GFP_KERNEL);
                        if(!temp_rsp) {
                            KRIL_DEBUG(DBG_ERROR, "unable to allocate temp_rsp buf\n");
                            KRIL_ASSERT(1);
                            goto DisableCellInfo;
                        }
                        KRIL_DEBUG(DBG_INFO, "num_gsm_ncells:%d \n", rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells);
                        rdata = (krilGetNeighborCell_t *)temp_rsp;
                        memset(temp_rsp, 0,sizeof(krilGetNeighborCell_t)*rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells);
                        count = 0;
                        for (loop = 0 ; loop < rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells ; loop++)
                        {
                            if (rsp->mLbsParams.lbs_mm_params.lac != 0 &&
                                rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].cell_id != 0 &&
                                rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].rxlev >= FILTER_GSM_SIGNAL_LEVEL &&
                                rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].rxlev <= 63) //2G receive power level Valid values: 0 to 63, and also filter lower signal level
                            {
                                KRIL_DEBUG(DBG_INFO, "lac:0x%x cell_id:0x%x rxlev:%d\n", rsp->mLbsParams.lbs_mm_params.lac, rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].cell_id, rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].rxlev);
                                rdata[count].cid = (rsp->mLbsParams.lbs_mm_params.lac << 16) | rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].cell_id;
                                rdata[count].rssi = rsp->mLbsParams.lbs_gsm_params.gsm_ncells[loop].rxlev;
                                KRIL_DEBUG(DBG_INFO, "loop:%d count:%ld cid:0x%x rssi:%d\n", loop, count, rdata[count].cid, rdata[count].rssi);
                                count++;
                                if(count > LBS_MAX_NUM_GSM_NCELL || count > rsp->mLbsParams.lbs_gsm_params.num_gsm_ncells)
                                {
                                    kfree(temp_rsp);
                                    KRIL_ASSERT(1);
                                    goto DisableCellInfo;
                                }
                            }
                        }
                        pdata->rsp_len = sizeof(krilGetNeighborCell_t)*count;
                        pdata->bcm_ril_rsp = kmalloc(pdata->rsp_len, GFP_KERNEL);
                        if(!pdata->bcm_ril_rsp) {
                            KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");
                        }
                        else
                        {    
                            memcpy(pdata->bcm_ril_rsp, temp_rsp, pdata->rsp_len);
                        }
                        if (temp_rsp != NULL)
                            kfree(temp_rsp);
                    }
                    else
                    {
                        KRIL_DEBUG(DBG_ERROR, "KRIL_GetNeighboringCellIDsHandler:: no RAT\n");
                        pdata->result = BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW;
                    }
DisableCellInfo:
                    CAPI2_MsDbApi_SYS_EnableCellInfoMsg(InitClientInfo(pdata->ril_cmd->SimId), FALSE);
                    pdata->handler_state = BCM_SYS_EnableCellInfoMsg;
                }
                else
                {
                    KRIL_SetServingCellTID(capi2_rsp->tid);
                }
            }
        }
        break;

        case BCM_SYS_EnableCellInfoMsg:
        {
            if (BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW == pdata->result)
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            else
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            KRIL_SetInNeighborCellHandler( FALSE );
        }
        break;
 
        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_SetLocationUpdatesHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            KrilLocationUpdates_t *tdate = (KrilLocationUpdates_t *)pdata->ril_cmd->data;
            KRIL_DEBUG(DBG_INFO, "location_updates:%d\n", tdate->location_updates);
            KRIL_SetLocationUpdateStatus(pdata->ril_cmd->SimId, tdate->location_updates);
            pdata->handler_state = BCM_FinishCAPI2Cmd;
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            if (capi2_rsp->result != RESULT_OK)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
        }
        break;

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}


void KRIL_SetBPMModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    if (capi2_rsp && capi2_rsp->result != RESULT_OK)
    {
        KRIL_DEBUG(DBG_ERROR,"CAPI2 response failed:%d\n", capi2_rsp->result);
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            char* rawdata = (char*)pdata->ril_cmd->data;
            KRIL_DEBUG(DBG_INFO, "status:%d\n", rawdata[5]);
            if (0 == rawdata[5] || 1 == rawdata[5])
            {
                CAPI2_PhoneCtrlApi_SetPagingStatus(InitClientInfo(pdata->ril_cmd->SimId), (UInt8)rawdata[5]);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "not support status:%d\n", rawdata[5]);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            UInt8 *resp = NULL;
            pdata->bcm_ril_rsp = kmalloc(sizeof(UInt8)*5, GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;                                         
            }
            else
            {
                pdata->rsp_len = sizeof(UInt8)*5 ;
                resp = (UInt8*)pdata->bcm_ril_rsp;
                resp[0] = (UInt8)'B';
                resp[1] = (UInt8)'R';
                resp[2] = (UInt8)'C';
                resp[3] = (UInt8)'M';
                resp[4] = (UInt8)BRIL_HOOK_SET_BPM_MODE;
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            break;
        }

        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}
