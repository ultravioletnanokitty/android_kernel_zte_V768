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
#include "capi2_lcs_cplane_api.h"
#include "bcm_kril.h"
#include "bcm_cp_cmd_handler.h"

#ifdef BRCM_AGPS_CONTROL_PLANE_ENABLE

void KRIL_AgpsSendUpLinkHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
    
    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            AgpsCp_Data *tdata = (AgpsCp_Data *)pdata->ril_cmd->data;
			//LcsClientInfo_t lcsClientInfo = {0};
			ClientInfo_t* lcsClientInfo;
			lcsClientInfo=InitClientInfo(pdata->ril_cmd->SimId);
            
            if ((NULL != tdata) && (tdata->cPlaneDataLen))
            {
				//KRIL_DEBUG(DBG_ERROR,"AGPS DATA: pro=%d, len=%d, data 0x%lx, 0x%1x, 0x%1x, 0x%1x\n", tdata->protocol, tdata->cPlaneDataLen, tdata->cPlaneData[0], tdata->cPlaneData[1], tdata->cPlaneData[2], tdata->cPlaneData[3]);
				KRIL_DEBUG(DBG_ERROR,"AGPS DATA: pro=%d, len=%d, data 0x%lx\n", tdata->protocol, tdata->cPlaneDataLen, tdata->cPlaneData[0]);
				if(tdata->protocol == AGPS_PROC_RRLP)
				{

					//CAPI2_LCS_SendRrlpDataToNetwork(GetNewTID(), GetClientID(), lcsClientInfo, tdata->cPlaneData, tdata->cPlaneDataLen);
					CAPI2_LcsApi_RrlpSendDataToNetwork(lcsClientInfo, tdata->cPlaneData, tdata->cPlaneDataLen);
				}
				else if(tdata->protocol == AGPS_PROC_RRC)
				{
					CAPI2_LcsApi_RrcSendUlDcch(lcsClientInfo, tdata->cPlaneData, tdata->cPlaneDataLen);
				}
				else
				{
					KRIL_DEBUG(DBG_ERROR,"Unknow UL protocol handler_state:0x%lX\n", pdata->handler_state);
					pdata->handler_state = BCM_ErrorCAPI2Cmd;
					break;
				}
				pdata->handler_state = BCM_RESPCAPI2Cmd;
			}
			else
			{
				KRIL_DEBUG(DBG_ERROR,"Invalid UL data handler_state:0x%lX\n", pdata->handler_state);
				pdata->handler_state = BCM_ErrorCAPI2Cmd;
			}
			
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_ERROR, "BCM_RESPCAPI2Cmd\n");
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

LcsRrcMcFailure_t KRIL_Cp2LcsRrcMcFailure(AgpsCp_RrcMcFailureCode inMcFailure)
{
	LcsRrcMcFailure_t lcsMcFailure = LCS_RRC_unsupportedMeasurement;

	switch(inMcFailure)
	{
	case AGPS_RRC_configurationUnsupported:
		lcsMcFailure = LCS_RRC_configurationUnsupported;
		break;
	case AGPS_RRC_unsupportedMeasurement:
		lcsMcFailure = LCS_RRC_unsupportedMeasurement;
		break;
	case AGPS_RRC_invalidConfiguration:
		lcsMcFailure = LCS_RRC_invalidConfiguration;
		break;
	}
	return lcsMcFailure;
}

void KRIL_AgpsRrcMcFailureHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			AgpsCp_McFailure *tdata = (AgpsCp_McFailure *)pdata->ril_cmd->data;
			//LcsClientInfo_t lcsClientInfo = {0};
			ClientInfo_t* lcsClientInfo;
			lcsClientInfo=InitClientInfo(pdata->ril_cmd->SimId);

			//CAPI2_LCS_RrcMeasurementControlFailure(GetNewTID(), GetClientID(), lcsClientInfo, tdata->transId, KRIL_Cp2LcsRrcMcFailure(tdata->failureCode), 0);				
			CAPI2_LcsApi_RrcMeasCtrlFailure(lcsClientInfo, tdata->transId, KRIL_Cp2LcsRrcMcFailure(tdata->failureCode), 0);

            KRIL_DEBUG(DBG_ERROR,"transId:%d\n", tdata->transId);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_ERROR, "BCM_RESPCAPI2Cmd\n");
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

LcsRrcMcStatus_t KRIL_Cp2LcsRrcStatus(AgpsCp_RrcStatus inRrcStatus)
{
	LcsRrcMcStatus_t lcsMcStatus = LCS_RRC_asn1_ViolationOrEncodingError;

	switch(inRrcStatus)
	{
	case AGPS_RRC_STATUS_asn1_ViolationOrEncodingError:
		lcsMcStatus = LCS_RRC_asn1_ViolationOrEncodingError;
		break;
	case AGPS_RRC_STATUS_messageTypeNonexistent:
		lcsMcStatus = LCS_RRC_messageTypeNonexistent;
		break;
	}
	return lcsMcStatus;
}

void KRIL_AgpsRrcStatusHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			AgpsCp_RrcStatus *tdata = (AgpsCp_RrcStatus *)pdata->ril_cmd->data;
			//LcsClientInfo_t lcsClientInfo = {0};
			ClientInfo_t* lcsClientInfo;
			lcsClientInfo=InitClientInfo(pdata->ril_cmd->SimId);

			CAPI2_LcsApi_RrcStatus(lcsClientInfo, KRIL_Cp2LcsRrcStatus(*tdata));
			//CAPI2_LCS_RrcStatus(GetNewTID(), GetClientID(), lcsClientInfo, KRIL_Cp2LcsRrcStatus(*tdata));
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_ERROR, "BCM_RESPCAPI2Cmd\n");
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

#ifdef BRCM_AGPS_PKCS15_ENABLE

void KRIL_AgpsOpenPkcs15Handler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
	AgpsOpenPkcs15Cxt* pCxtData = (AgpsOpenPkcs15Cxt*)(pdata->cmdContext);

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			//SIM_GetApplicationType() returns SIM_APPL_2G or SIM_APPL_3G
			//AgpsCp_RrcStatus *tdata = (AgpsCp_RrcStatus *)pdata->ril_cmd->data;
			//LcsClientInfo_t lcsClientInfo = {0};
			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);
			pCxtData->mSockID = 0;
			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps calls IsApplicationSupported\n");

			CAPI2_USimApi_IsApplicationSupported(pLcsClientInfo, PKCS15_APPLICATION);
            pdata->handler_state = BCM_AGPS_PKCS15_SUPPORT;
            break;
        }
		case BCM_AGPS_PKCS15_SUPPORT:
		{
			//capi2_rsp->msgType == MSG_USIM_IS_APP_SUPPORTED_RSP
			// payload is CAPI2_USimApi_IsApplicationSupported_Rsp_t
			CAPI2_USimApi_IsApplicationSupported_Rsp_t *rsp = (CAPI2_USimApi_IsApplicationSupported_Rsp_t *) capi2_rsp->dataBuf;

            KRIL_DEBUG(DBG_ERROR, "KRIL_Agps PKCS support=%d\n", rsp->val);
			if (TRUE == rsp->val)
            {
				ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);

				memcpy(&(pCxtData->mAidData), &(rsp->pOutAidData), sizeof(USIM_AID_DATA_t));
				CAPI2_SimApi_SendOpenSocketReq(pLcsClientInfo);
				pdata->handler_state = BCM_AGPS_OPEN_SOCKET;
			}
			else
			{
				KRIL_DEBUG(DBG_INFO, "error result: %d\n", rsp->val);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
		}
		case BCM_AGPS_OPEN_SOCKET:
		{
			//capi2_rsp->msgType == MSG_SIM_OPEN_SOCKET_RSP
			// payload is SIM_OPEN_SOCKET_RES_t
			SIM_OPEN_SOCKET_RES_t *rsp = (SIM_OPEN_SOCKET_RES_t *) capi2_rsp->dataBuf;

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps OpenSocket socket_id=%d\n", rsp->socket_id);
            if (rsp->socket_id)
            {
				ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);

				pCxtData->mSockID = rsp->socket_id;
				CAPI2_SimApi_SendSelectAppiReq(pLcsClientInfo, pCxtData->mSockID, pCxtData->mAidData.aidData, pCxtData->mAidData.aidLen);
				pdata->handler_state = BCM_RESPCAPI2Cmd;
			}
			else
			{
				KRIL_DEBUG(DBG_INFO, "error socket_id: %d\n", rsp->socket_id);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
		}
		case BCM_RESPCAPI2Cmd:
		{
			//capi2_rsp->msgType == MSG_SIM_SELECT_APPLI_RSP
			// payload is SIM_SELECT_APPLI_RES_t
			SIM_SELECT_APPLI_RES_t *rsp = (SIM_SELECT_APPLI_RES_t *) capi2_rsp->dataBuf;
			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps SelectAppi result=%d, pkcsHdl=%d\n", rsp->result, pCxtData->mSockID);
            if (rsp->result == SIMACCESS_SUCCESS)
            {
				unsigned char * pPkcsHdl = NULL;

                pdata->bcm_ril_rsp = kmalloc(sizeof(unsigned char), GFP_KERNEL);
                if(!pdata->bcm_ril_rsp) {
                    KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;             
                    return;
                }
                pdata->rsp_len = sizeof(unsigned char);
                pPkcsHdl = (unsigned char*)pdata->bcm_ril_rsp;
                *pPkcsHdl = pCxtData->mSockID;

                pdata->handler_state = BCM_FinishCAPI2Cmd;
			}
			else
			{
				//2G case
				KRIL_DEBUG(DBG_INFO, "error result: %d\n", rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
		}
		/*
        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_ERROR, "BCM_RESPCAPI2Cmd\n");
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }*/

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

//RIL_REQUEST_AGPS_CLOSE_PKCS15
void KRIL_AgpsClosePkcs15Handler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
	KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
	unsigned char *pSockId = (unsigned char *)pdata->ril_cmd->data;

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);

			KRIL_DEBUG(DBG_ERROR, "KRIL_AgpsClosePkcs15Handler, sockID=%d \n", (unsigned char)(*pSockId));
			CAPI2_SimApi_SendDeactivateAppiReq(pLcsClientInfo, (UInt8) (*pSockId));
			pdata->handler_state = BCM_AGPS_DESELECT_PKCS15;
			break;
		}
		case BCM_AGPS_DESELECT_PKCS15:
		{
			//MsgType_t: ::MSG_SIM_DEACTIVATE_APPLI_RSP
			//ResultData: ::SIM_DEACTIVATE_APPLI_RES_t
			SIM_DEACTIVATE_APPLI_RES_t * rsp = (SIM_DEACTIVATE_APPLI_RES_t *) capi2_rsp->dataBuf;
			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps SendDeactivateAppiReq, result=%d \n", rsp->result);
			CAPI2_SimApi_SendCloseSocketReq(pLcsClientInfo, (UInt8) (*pSockId));
			pdata->handler_state = BCM_RESPCAPI2Cmd;
			break;
		}

		case BCM_RESPCAPI2Cmd:
		{
			//MsgType_t: ::MSG_SIM_CLOSE_SOCKET_RSP
			//ResultData: ::SIM_CLOSE_SOCKET_RES_t

			SIM_CLOSE_SOCKET_RES_t * rsp = (SIM_CLOSE_SOCKET_RES_t *) capi2_rsp->dataBuf;

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps SendCloseSocketReq, result=%d \n", rsp->result);
			pdata->handler_state = BCM_FinishCAPI2Cmd;
			break;
		}
		default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
	}
}

//RIL_REQUEST_AGPS_SIM_EF_INFO
void KRIL_AgpsSimEfInfoHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{

    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
	AgpsEfInfoCxt* pCxtData = (AgpsEfInfoCxt*)(pdata->cmdContext);

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);
			AgpsEfInfoReq *tdata = (AgpsEfInfoReq *)pdata->ril_cmd->data;

			pCxtData->mEfInfoPath =  tdata->path;

			pCxtData->mSimType = AGPS_SIM_APPL_INVALID;

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps, pkcs15Hdl=%d path=0x%x\n", tdata->pkcs15Hdl, tdata->path);

			//EFDIR socket_id should be 0
			CAPI2_SimApi_GetApplicationType(pLcsClientInfo);
            pdata->handler_state = BCM_AGPS_GET_EF_INFO;
            break;
        }
		case BCM_AGPS_GET_EF_INFO:
		{
			//capi2_rsp->msgType == MSG_SIM_APP_TYPE_RSP
			// payload is SIM_APPL_TYPE_t
			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);
			SIM_APPL_TYPE_t *rsp = (SIM_APPL_TYPE_t *) capi2_rsp->dataBuf;

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps BCM_AGPS_GET_EF_INFO SimType=%d\n", *rsp);
			if(*rsp != SIM_APPL_INVALID)
			{
				switch(*rsp)
				{
				case SIM_APPL_2G:
					pCxtData->mSimType = AGPS_SIM_APPL_2G;
					break;
				case SIM_APPL_3G:
					pCxtData->mSimType = AGPS_SIM_APPL_3G;
					break;
				default:
					pCxtData->mSimType = AGPS_SIM_APPL_INVALID;
					break;
				}

				//EFDIR socket_id should be 0
				CAPI2_SimApi_SubmitEFileInfoReq(pLcsClientInfo, 0, (APDUFileID_t) pCxtData->mEfInfoPath, (APDUFileID_t) 0x3F00, 0, NULL);
				pdata->handler_state = BCM_RESPCAPI2Cmd;
			}
			else
			{
				//Invalid SIM
				KRIL_DEBUG(DBG_INFO, "error SIM_APPL_INVALID\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
        }
		case BCM_RESPCAPI2Cmd:
		{
			//capi2_rsp->msgType == MSG_SIM_EFILE_INFO_RSP
			// payload is SIM_EFILE_INFO_t
			SIM_EFILE_INFO_t *rsp = (SIM_EFILE_INFO_t *) capi2_rsp->dataBuf;
			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps EFileInfoRes result=%d, recLen=%d, fsize=%d\n", rsp->result, rsp->record_length, rsp->file_size);
            if (rsp->result == SIMACCESS_SUCCESS)
            {
				AgpsEfInfoRes * pEfInfo = NULL;

                pdata->bcm_ril_rsp = kmalloc(sizeof(AgpsEfInfoRes), GFP_KERNEL);
                if(!pdata->bcm_ril_rsp) {
                    KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;             
                    return;
                }
                pdata->rsp_len = sizeof(AgpsEfInfoRes);
                pEfInfo = (AgpsEfInfoRes*)pdata->bcm_ril_rsp;
                pEfInfo->recLen = rsp->record_length;
				pEfInfo->recNum = (rsp->file_size / rsp->record_length);
				pEfInfo->simType = pCxtData->mSimType;

                pdata->handler_state = BCM_FinishCAPI2Cmd;
			}
			else
			{
				//2G case
				KRIL_DEBUG(DBG_INFO, "error result: %d\n", rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
		}

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

void KRIL_AgpsSimRecHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			AgpsEfRecReq *tdata = (AgpsEfRecReq *)pdata->ril_cmd->data;

			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps calls RecReadReq, path=0x%x, idx=%d, recLen=%d, numOfRec=%d\n", tdata->path, tdata->recIdx, tdata->recLen, tdata->numOfRec);

			//CAPI2_SimApi_SubmitRecordEFileReadReq( pLcsClientInfo, 0, (APDUFileID_t)tdata->path, (APDUFileID_t) 0x3F00, tdata->recIdx, tdata->recLen, 0, NULL);

			CAPI2_SimApi_SubmitMulRecordEFileReq( pLcsClientInfo, 0, (APDUFileID_t)tdata->path, (APDUFileID_t) 0x3F00, 
				tdata->recIdx, tdata->numOfRec, tdata->recLen, 0, NULL, FALSE, FALSE);
            
			pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }
		case BCM_RESPCAPI2Cmd:
		{
			//capi2_rsp->msgType == MSG_SIM_EFILE_DATA_RSP
			// payload is SIM_EFILE_DATA_t
			/*
			SIM_EFILE_DATA_t *rsp = (SIM_EFILE_DATA_t *) capi2_rsp->dataBuf;
			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps EFileInfoRes result=%d, resLen=%d\n", rsp->result, rsp->data_len);
            if ((rsp->result == SIMACCESS_SUCCESS) && (rsp->data_len <= BRCM_AGPS_MAX_MESSAGE_SIZE))
            {
				AgpsEfData * pEfInfo = NULL;

                pdata->bcm_ril_rsp = kmalloc(sizeof(AgpsEfData), GFP_KERNEL);
                if(!pdata->bcm_ril_rsp) {
                    KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;             
                    return;
                }
                pdata->rsp_len = sizeof(AgpsEfData);
                pEfInfo = (AgpsEfData*)pdata->bcm_ril_rsp;
                pEfInfo->dataLen = rsp->data_len;
				memcpy(pEfInfo->data, rsp->ptr,  rsp->data_len);

                pdata->handler_state = BCM_FinishCAPI2Cmd;
			}
			else
			{
				//2G case
				KRIL_DEBUG(DBG_INFO, "error result: %d\n", rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}*/

			// MsgType_t: ::MSG_SIM_MUL_REC_DATA_RSP
			SIM_MUL_REC_DATA_t *rsp = (SIM_MUL_REC_DATA_t *) capi2_rsp->dataBuf;
			int rspDataLen = rsp->rec_len * rsp->num_of_rec;
			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps MultiREC Res result=%d, resLen=%d recLen=%d\n", rsp->result, rspDataLen, rsp->rec_len);
            if ((rsp->result == SIMACCESS_SUCCESS) && (rspDataLen <= BRCM_AGPS_MAX_MESSAGE_SIZE))
            {
				AgpsRecData * pRecData = NULL;

                pdata->bcm_ril_rsp = kmalloc(sizeof(AgpsRecData), GFP_KERNEL);
                if(!pdata->bcm_ril_rsp) {
                    KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;             
                    return;
                }
                pdata->rsp_len = sizeof(AgpsRecData);
                pRecData = (AgpsRecData*)pdata->bcm_ril_rsp;
                pRecData->dataLen = rspDataLen;//rsp->data_len;
				pRecData->recLen = rsp->rec_len;
				memcpy(pRecData->data, rsp->rec_data,  rspDataLen);

                pdata->handler_state = BCM_FinishCAPI2Cmd;
			}
			else
			{
				//2G case
				KRIL_DEBUG(DBG_INFO, "KRIL_Agps MultiREC Res error result: %d\n", rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
		}

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

void KRIL_AgpsSimEfHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
        KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
			const UInt16 mfPath = APDUFILEID_MF;
			AgpsEfReadReq *tdata = (AgpsEfReadReq *)pdata->ril_cmd->data;

			ClientInfo_t* pLcsClientInfo = InitClientInfo(pdata->ril_cmd->SimId);

			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps calls EFileReadReq, appPath=0x%x, subPath=0x%x, idx=%d, len=%d\n", tdata->appPath,tdata->subPath, tdata->idx, tdata->len);

			if(tdata->len == 0)
			{
				CAPI2_SimApi_SubmitWholeBinaryEFileReadReq( pLcsClientInfo, tdata->pkcs15Hdl, 
					(APDUFileID_t) tdata->subPath, (APDUFileID_t) tdata->appPath, 1, &mfPath);
			}
			else
			{
				CAPI2_SimApi_SubmitBinaryEFileReadReq( pLcsClientInfo, tdata->pkcs15Hdl, 
					(APDUFileID_t) tdata->subPath, (APDUFileID_t) tdata->appPath, tdata->idx, tdata->len, 
								  1, &mfPath);
			}            
			pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }
		case BCM_RESPCAPI2Cmd:
		{
			//capi2_rsp->msgType == MSG_SIM_EFILE_DATA_RSP
			// payload is SIM_EFILE_DATA_t
			SIM_EFILE_DATA_t *rsp = (SIM_EFILE_DATA_t *) capi2_rsp->dataBuf;
			KRIL_DEBUG(DBG_ERROR, "KRIL_Agps EFileInfoRes result=%d, resLen=%d\n", rsp->result, rsp->data_len);
            if ((rsp->result == SIMACCESS_SUCCESS) && (rsp->data_len <= BRCM_AGPS_MAX_MESSAGE_SIZE))
            {
				AgpsEfData * pEfInfo = NULL;

                pdata->bcm_ril_rsp = kmalloc(sizeof(AgpsEfData), GFP_KERNEL);
                if(!pdata->bcm_ril_rsp) {
                    KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;             
                    return;
                }
                pdata->rsp_len = sizeof(AgpsEfData);
                pEfInfo = (AgpsEfData*)pdata->bcm_ril_rsp;
                pEfInfo->dataLen = rsp->data_len;
				memcpy(pEfInfo->data, rsp->ptr,  rsp->data_len);

                pdata->handler_state = BCM_FinishCAPI2Cmd;
			}
			else
			{
				//2G case
				KRIL_DEBUG(DBG_INFO, "error result: %d\n", rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;

			}
            break;
		}

        default:
        {
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

#endif //BRCM_AGPS_PKCS15_ENABLE

#endif //BRCM_AGPS_CONTROL_PLANE_ENABLE

