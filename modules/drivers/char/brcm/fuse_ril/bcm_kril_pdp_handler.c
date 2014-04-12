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
#include "capi2_pch_api_old.h"
#include "linux/delay.h"

// for SIMID
SimNumber_t SIMEXCHID[DUAL_SIM_SIZE] = {SIM_SINGLE, SIM_DUAL_SECOND, SIM_DUAL_FIRST};

extern int gdataprefer; // SIM1:0 SIM2:1

KrilDataCallResponse_t pdp_resp[DUAL_SIM_SIZE][BCM_NET_MAX_RIL_PDP_CNTXS];
#define RIL_PDP_CID(index) (BCM_NET_RIL_PDP_CNTXS_BASE + index)

static void SendCHAPOptions();
static void SendPAPOptions();

extern MSRegInfo_t  gRegInfo[DUAL_SIM_SIZE];

static int ParsePdpFailCause(Result_t value)
{
    BRIL_LastDataCallActivateFailCause cause;
    switch (value)
    {
        case PDP_OPERATOR_DETERMINED_BARRING:
            cause = BCM_PDP_FAIL_OPERATOR_BARRED;
            break;

        case PDP_INSUFFICIENT_RESOURCES:
            cause = BCM_PDP_FAIL_INSUFFICIENT_RESOURCES;
            break;

        case PDP_MISSING_OR_UNKNOWN_APN:
            cause = BCM_PDP_FAIL_MISSING_UKNOWN_APN;
            break;

        case PDP_UNKNOWN_PDP_ADDRESS:
            cause = BCM_PDP_FAIL_UNKNOWN_PDP_ADDRESS_TYPE;
            break;

        case PDP_USER_AUTH_FAILED:
            cause = BCM_PDP_FAIL_USER_AUTHENTICATION;
            break;

        case PDP_ACTIVATION_REJECTED_BY_GGSN:
            cause = BCM_PDP_FAIL_ACTIVATION_REJECT_GGSN;
            break;

        case PDP_ACTIVATION_REJECTED_UNSPECIFIED:
            cause = BCM_PDP_FAIL_ACTIVATION_REJECT_UNSPECIFIED;
            break;

        case PDP_SERVICE_OPT_NOT_SUPPORTED:
            cause = BCM_PDP_FAIL_SERVICE_OPTION_NOT_SUPPORTED;
            break;

        case PDP_REQ_SERVICE_NOT_SUBSCRIBED:
            cause = BCM_PDP_FAIL_SERVICE_OPTION_NOT_SUBSCRIBED;
            break;

        case PDP_SERVICE_TEMP_OUT_OF_ORDER:
            cause = BCM_PDP_FAIL_SERVICE_OPTION_OUT_OF_ORDER;
            break;

        case PDP_NSAPI_ALREADY_USED:
            cause = BCM_PDP_FAIL_NSAPI_IN_USE;
            break;

        case PDP_PROTOCOL_ERROR_UNSPECIFIED:
            cause = BCM_PDP_FAIL_PROTOCOL_ERRORS;
            break;

        default:
            cause = BCM_PDP_FAIL_ERROR_UNSPECIFIED;
            break;
    }
    return (int)cause;
}

static UInt8 GetFreePdpContext(SimNumber_t SimId)
{
    UInt8 i;
    for (i=0; i<BCM_NET_MAX_RIL_PDP_CNTXS; i++)
    {
        if (0 == pdp_resp[SimId][i].cid)
        {
            pdp_resp[SimId][i].cid = RIL_PDP_CID(i);
			//lvjinlong@wind-mobi.com start 2012.03.21
			//patch for csp 514724
            //KRIL_DEBUG(DBG_INFO, "SimId:%d GetFreePdpContext[%d]=%d \n", SimId, i, pdp_resp[SimId][i].cid);
            KRIL_DEBUG(DBG_ERROR, "SimId:%d GetFreePdpContext[%d]=%d \n", SimId, i, pdp_resp[SimId][i].cid);
			//lvjinlong@wind-mobi.com end 2012.03.21
            return i;
        }
    }
    return BCM_NET_MAX_RIL_PDP_CNTXS;
}


UInt8 ReleasePdpContext(SimNumber_t SimId, UInt8 cid)
{
    UInt8 i;
    for (i=0; i<BCM_NET_MAX_RIL_PDP_CNTXS; i++)
    {
        if (cid == pdp_resp[SimId][i].cid)
        {            
            pdp_resp[SimId][i].active = 0;
            KRIL_SendNotify(SimId, BRCM_RIL_UNSOL_DATA_CALL_LIST_CHANGED, &pdp_resp[SimId][i], sizeof(KrilDataCallResponse_t));
            memset(&pdp_resp[SimId][i], 0, sizeof(KrilDataCallResponse_t));
            pdp_resp[SimId][i].cid = 0;
            KRIL_DEBUG(DBG_ERROR, "SimId:%d ReleasePdpContext[%d]=%d \n", SimId, i, pdp_resp[SimId][i].cid);
            return i;
        }
    }
    return BCM_NET_MAX_RIL_PDP_CNTXS;
}

UInt8 FindPdpCid(SimNumber_t SimId)
{
    UInt8 i;
    for (i=0; i<BCM_NET_MAX_RIL_PDP_CNTXS; i++)
    {
        if (pdp_resp[SimId][i].cid != 0)
        {
            KRIL_DEBUG(DBG_ERROR, "SimId:%d cid:%d \n", SimId, pdp_resp[SimId][i].cid);
            return pdp_resp[SimId][i].cid;
        }
    }
    return BCM_NET_MAX_RIL_PDP_CNTXS+1;
}


static void FillDataResponseTypeApn(SimNumber_t SimId, UInt8 cid, char* pdpType, char* apn)
{
    UInt8 i;
    for (i=0; i<BCM_NET_MAX_RIL_PDP_CNTXS; i++)
    {
        if (cid == pdp_resp[SimId][i].cid)
        {
            if ((NULL != pdpType)&&(strlen(pdpType) < PDP_TYPE_LEN_MAX))
                strcpy(pdp_resp[SimId][i].pdpType, pdpType);
            if ((NULL != apn)&&(strlen(apn) < PDP_APN_LEN_MAX))
                strcpy(pdp_resp[SimId][i].apn, apn);
            KRIL_DEBUG(DBG_INFO, "SimId:%d FillDataResponseTypeApn[%d]=[%s][%s] \n", SimId, i, pdp_resp[SimId][i].pdpType, pdp_resp[SimId][i].apn);
            return;
        }
    }
}


static void FillDataResponseAddress(SimNumber_t SimId, UInt8 cid, char* addressIPV4, char* addressIPV6)
{
    UInt8 i;
    for (i=0; i<BCM_NET_MAX_RIL_PDP_CNTXS; i++)
    {
        if (cid == pdp_resp[SimId][i].cid)
        {
            pdp_resp[SimId][i].active = 2; // 0=inactive, 1=active/physical link down, 2=active/physical link up
            if (NULL != addressIPV4)
                memcpy(pdp_resp[SimId][i].pdpAddressIPV4, addressIPV4, PDP_ADDRESS_LEN_MAX);
            if (NULL != addressIPV6)
	    {
                memcpy(pdp_resp[SimId][i].pdpAddressIPV6, addressIPV6, PDP_ADDRESS_LEN_MAX);
	    }
            //KRIL_DEBUG(DBG_INFO, "SimId:%d FillDataResponseAddress[%d]=[%s] \n", SimId, i, pdp_resp[SimId][i].address);
            return;
        }
    }
}


void KRIL_SetupPdpHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    static KrilPdpContext_t gContext;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (NULL != capi2_rsp)
        KRIL_DEBUG(DBG_INFO, "BCM_RESPCAPI2Cmd::result:0x%x\n", capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_NETWORK_ELEM_REGSTATE_INFO);
            pdata->handler_state = BCM_MS_GetRegistrationInfo;
        }
        break;
        case BCM_MS_GetRegistrationInfo:
        {
            CAPI2_MS_Element_t* rsp = NULL;
            MSRegStateInfo_t* presult = NULL; 
            UInt8 pindex;

            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "result:0x%x\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }

            rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;

            if ( (rsp!=NULL)&& (rsp->inElemType == MS_NETWORK_ELEM_REGSTATE_INFO) )
            {
                presult = (MSRegStateInfo_t*)(&(rsp->data_u));
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR,"unexpected response retrieving CAPI2_MS_GetElement !!\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }

            KRIL_DEBUG(DBG_INFO, "gsm_state:%d, gprs_state:%d \n",presult->gsm_reg_state, presult->gprs_reg_state);

            if ((presult->gprs_reg_state != REG_STATE_NORMAL_SERVICE) &&
                (presult->gprs_reg_state != REG_STATE_ROAMING_SERVICE))
            {
                pdata->result = BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                KRIL_DEBUG(DBG_ERROR, "Error: gsm & gprs is not in normal or roaming service!\n");
                return;
            }

            if (NULL == pdata->ril_cmd->data)
            {
                KRIL_DEBUG(DBG_ERROR, "PDPActivate Fail with NULL data\n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }

            if (BCM_NET_MAX_RIL_PDP_CNTXS == (pindex = GetFreePdpContext(pdata->ril_cmd->SimId)))
            {
                KRIL_DEBUG(DBG_ERROR, "PDPActivate Fail with over max cid[%d]\n", pindex);
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }

            memcpy(&gContext, (KrilPdpContext_t *)(pdata->ril_cmd->data), sizeof(KrilPdpContext_t));
            if (strlen(gContext.pdpType)==0)
	    {
	 	//set default to beIP
		strcpy(gContext.pdpType, "IP");
	    }

            KRIL_DEBUG(DBG_INFO, "KRIL_SetupPdpHandler - Set PDP Context : apn %s \n", gContext.apn);

            {
	        char cpPdpType[PDP_TYPE_LEN_MAX];
                UInt8 numParms = (strlen(gContext.apn) == 0)?2:3;
                gContext.cid = pdp_resp[pdata->ril_cmd->SimId][pindex].cid;
                FillDataResponseTypeApn(pdata->ril_cmd->SimId, gContext.cid, gContext.pdpType, gContext.apn);
				//lvjinlong@wind-mobi.com start 2012.03.21
				//patch for csp 514724
				*(UInt8*)(pdata->cmdContext) = pindex; /*keep the array index for check later*/
				//lvjinlong@wind-mobi.com end 2012.03.21
                KRIL_DEBUG(DBG_INFO,"**Calling CAPI2_PdpApi_SetPDPContext numParms %d type:%s apn:%s pindex %d cid %d\n",numParms, gContext.pdpType, (gContext.apn==NULL)?"NULL":gContext.apn, pindex,pdp_resp[pdata->ril_cmd->SimId][pindex].cid  );
                KRIL_SetInSetupPDPHandler(TRUE);

		if (strcmp(gContext.pdpType, "IPV4V6") ==0 )
	 	{
		    strcpy(cpPdpType, "IPv4v6");
		}
		else
	 	{
		    strcpy(cpPdpType,  gContext.pdpType);
		}
		
                CAPI2_PdpApi_SetPDPContext(InitClientInfo(pdata->ril_cmd->SimId), 
                                            pdp_resp[pdata->ril_cmd->SimId][pindex].cid, 
                                            numParms, 
                                            cpPdpType,
                                            gContext.apn, 
                                            "", 
                                            0, 
                                            0);
            }
  
            pdata->handler_state = BCM_PDP_SetPdpContext;
        }
        break;

        case BCM_PDP_SetPdpContext:
        {
            PCHProtConfig_t t_PCHP;
            char *username = gContext.username;
            char *password = gContext.password;
            IPConfigAuthType_t t_Authtype = REQUIRE_PAP;/*default setting*/
            //UInt8 ContextID = 1;
            KRIL_DEBUG(DBG_INFO, "KRIL_SetupPdpHandler - Activate PDP context \n");

            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "PDPActivate Fail to SetPDPContext[%d]\n", gContext.cid);
                ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                KRIL_SetInSetupPDPHandler(FALSE);
                break;
            }

	    if (gContext.authtype == AUTH_CHAP)
            {
                SendCHAPOptions(&gContext);
                pdata->handler_state = BCM_PDP_SetPdpOption;
            }
            else
            {
                SendPAPOptions(&gContext);
                pdata->handler_state = BCM_PDP_SetPdpOption;
            }

        }
        break;

        case BCM_PDP_SetPdpOption:
        {
            CAPI2_PchExApi_BuildIpConfigOptions_Rsp_t *rsp = (CAPI2_PchExApi_BuildIpConfigOptions_Rsp_t *)capi2_rsp->dataBuf;

            if( (RESULT_OK != capi2_rsp->result) || (rsp==NULL))
            {
                KRIL_DEBUG(DBG_ERROR, "CAPI2_PchExApi_BuildIpConfigOptions Fail\n");
                ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                pdata->result = RESULT_ERROR;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }

            CAPI2_PchExApi_SendPDPActivateReq(InitClientInfo(pdata->ril_cmd->SimId), gContext.cid, ACTIVATE_MMI_IP_RELAY, &(rsp->cie));

            pdata->handler_state = BCM_ActivatePDP;
        }
        break;


        case BCM_ActivatePDP:
        {
	    PDP_SendPDPActivateReq_Rsp_t *rsp = NULL;
	    KrilPdpData_t *rdata = NULL;
	    PCHDecodedPDPAddress_T decodedPdpAddress;
	    Result_t result;
	    int i;
		//lvjinlong@wind-mobi.com start 2012.03.21
		//patch csp 514724
			UInt8 a_index;
		//lvjinlong@wind-mobi.com end 2012.03.21
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilPdpData_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "Unable to allocate bcm_ril_rsp buf\n");  
                ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
		KRIL_SetInSetupPDPHandler(FALSE);
                return;
            }
            pdata->rsp_len = sizeof(KrilPdpData_t);
            memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);

            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if ((RESULT_OK != capi2_rsp->result) ||
		  (NULL == capi2_rsp->dataBuf) )
            {
                KRIL_DEBUG(DBG_ERROR, "PDPActivate Fail to SendPDPActivateReq[%d] \n", gContext.cid);
                ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
		KRIL_SetInSetupPDPHandler(FALSE);
                break;
            }
                
	    rsp = (PDP_SendPDPActivateReq_Rsp_t *)capi2_rsp->dataBuf;
            rdata = pdata->bcm_ril_rsp;

            KRIL_DEBUG(DBG_INFO, "PDP Activate Resp - cid %d \n", rsp->activatedContext.cid);
            if((rsp->cause != RESULT_OK) || (rsp->response != PCH_REQ_ACCEPTED))
            {
                KRIL_DEBUG(DBG_ERROR, "PDPActivate Fail cause %d, resp(1 accept) %d, cid %d\r\n",
                rsp->cause, rsp->response, rsp->activatedContext.cid);
                ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                rdata->cause = ParsePdpFailCause(rsp->cause);
				//lvjinlong@wind-mobi.com start 2012.04.25
				//csp patch 519965
                //pdata->result = BCM_E_RADIO_NOT_AVAILABLE;
				pdata->result = BCM_E_GENERIC_FAILURE;
				//lvjinlong@wind-mobi.com end 2012.04.25
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
  		KRIL_SetInSetupPDPHandler(FALSE);
                break;
            }

	    //TODO, currently on CP side, only in ipv4v6 case, pdpaddress info would be copied into pdpAddressIE, will be changed later on. For now
            //we will decode first, if get non info, then treat each case differently then.
#if 0  //to be removed after debugging period
		    printk("pdpAddressIE=");
                    for (i=0; i<24;i++)
                    {
                       printk( "%02x ", rsp->activatedContext.pdpAddressIE[i]);
                    }
                    printk("\r\n");
#endif

	    result = PdpApi_GetDecodedPDPAddress(InitClientInfo(pdata->ril_cmd->SimId), rsp->activatedContext.pdpAddressIE,  &decodedPdpAddress);
            if (result == RESULT_OK)
	    {
		KRIL_DEBUG(DBG_INFO, "PdpApi_GetDecodedPDPAddress() return pdpType=%d\n", decodedPdpAddress.pdpAddressType);		

                if (decodedPdpAddress.pdpAddressType == PCH_IPV4_ADDRESS) 
		{
 	            strcpy(rdata->pdpType, "IP");
                    sprintf(rdata->pdpAddressIPV4, "%d.%d.%d.%d", decodedPdpAddress.pdpAddressIPV4[0], decodedPdpAddress.pdpAddressIPV4[1], decodedPdpAddress.pdpAddressIPV4[2], decodedPdpAddress.pdpAddressIPV4[3]);
		}
		else if (decodedPdpAddress.pdpAddressType == PCH_IPV6_ADDRESS)
		{
 	            strcpy(rdata->pdpType, "IPV6");
                    memcpy(rdata->pdpAddressIPV6, decodedPdpAddress.pdpAddressIPV6, PDP_ADDRESS_LEN_MAX);
		}
		else if (decodedPdpAddress.pdpAddressType == PCH_IPV4V6_ADDRESS)
		{	
 	            strcpy(rdata->pdpType, "IPV4V6");
                    //convert binary to string format to be consistent with single ipv4 case
                    sprintf(rdata->pdpAddressIPV4, "%d.%d.%d.%d", decodedPdpAddress.pdpAddressIPV4[0], decodedPdpAddress.pdpAddressIPV4[1], decodedPdpAddress.pdpAddressIPV4[2], decodedPdpAddress.pdpAddressIPV4[3]);
                    memcpy(rdata->pdpAddressIPV6, decodedPdpAddress.pdpAddressIPV6, PDP_ADDRESS_LEN_MAX);
		}
		else
		{
		    KRIL_DEBUG(DBG_ERROR, "PDP Activate, unknown pdp type from decoded pdpAddressID\n");
                    ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                    rdata->cause = ParsePdpFailCause(rsp->cause);
					//lvjinlong@wind-mobi.com start 2012.04.25
					//csp patch 519965
                    //pdata->result = BCM_E_RADIO_NOT_AVAILABLE;
                    pdata->result = BCM_E_GENERIC_FAILURE;
					//lvjinlong@wind-mobi.com end 2012.04.25
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
	
  		    KRIL_SetInSetupPDPHandler(FALSE);
                    break;
		}
	    }
	    else
	    {
		//get ipv4/ipv6 from pdpAddress field
 	        memcpy(rdata->pdpType, gContext.pdpType, PDP_TYPE_LEN_MAX);
	        if (strcmp(gContext.pdpType, "IP") == 0)
                {
		    memcpy(rdata->pdpAddressIPV4, rsp->activatedContext.pdpAddress, PDP_ADDRESS_LEN_MAX);
                    KRIL_DEBUG(DBG_INFO, "PDP Activate: IPV4, PDP Address %s \r\n", rdata->pdpAddressIPV4);

	        }
	        else if (strcmp(gContext.pdpType, "IPV6") == 0)
	        {
                    //ipv6
		    memcpy(rdata->pdpAddressIPV6, rsp->activatedContext.pdpAddress, PDP_ADDRESS_LEN_MAX);
#if 0
	 	    printk("PDP Activate:  IPV6 PDP Address = ");
                    for (i=0; i<PDP_ADDRESS_LEN_MAX;i++)
                    {
                       printk( "%02x ", rdata->pdpAddressIPV6[i]);
                    }
                    printk("\r\n");
#endif
                }
	        else
	        {
		    KRIL_DEBUG(DBG_ERROR, "PDP Activate, no data in pdpAddressID, can't get pdpType or pdpaddress for ipv4v6");
                    ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                    rdata->cause = ParsePdpFailCause(rsp->cause);
                    pdata->result = BCM_E_RADIO_NOT_AVAILABLE;
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;	
  		    KRIL_SetInSetupPDPHandler(FALSE);
                    break;
                }
	    }

            //return apn to URIL for M-PDP purpose
            if (NULL != gContext.apn)
            {
                memcpy(rdata->apn, gContext.apn, PDP_APN_LEN_MAX);
            }

            rdata->cid = rsp->activatedContext.cid;
			 /*check the array index to confirm if cid is still alive*/
            a_index = *(UInt8*)(pdata->cmdContext);
			//lvjinlong@wind-mobi.com 2012.03.21 start
			//patch for csp 514724			
            if (rdata->cid != pdp_resp[pdata->ril_cmd->SimId][a_index].cid)
            {
                KRIL_DEBUG(DBG_ERROR, "The KRIL list of active Cid has been killed or replaced, aCid:%d kCid[%d][%d]:%d!! \n"
                           , rdata->cid, pdata->ril_cmd->SimId, a_index, pdp_resp[pdata->ril_cmd->SimId][a_index].cid);
                if (0 == pdp_resp[pdata->ril_cmd->SimId][a_index].cid)
                {
                    KRIL_DEBUG(DBG_ERROR, "The Cid has been killed, recover it!! \n");
                    pdp_resp[pdata->ril_cmd->SimId][a_index].cid = rdata->cid;
                   memcpy(pdp_resp[pdata->ril_cmd->SimId][a_index].pdpType, rdata->pdpType, PDP_TYPE_LEN_MAX);
                    memcpy(pdp_resp[pdata->ril_cmd->SimId][a_index].apn, gContext.apn, PDP_APN_LEN_MAX);
                }
                else /*the following should be rare case*/
                {
                    UInt8 i;
                    KRIL_DEBUG(DBG_ERROR, "*Rare Case* The Cid has been replaced, reallocate it!! \n");
                    for (i=0; i<BCM_NET_MAX_RIL_PDP_CNTXS; i++)
                    {
                        if (0 == pdp_resp[pdata->ril_cmd->SimId][i].cid)
                        {
                            pdp_resp[pdata->ril_cmd->SimId][i].cid = rdata->cid;
                            memcpy(pdp_resp[pdata->ril_cmd->SimId][i].pdpType, rdata->pdpType, PDP_TYPE_LEN_MAX);
                            memcpy(pdp_resp[pdata->ril_cmd->SimId][i].apn, gContext.apn, PDP_APN_LEN_MAX);
                            KRIL_DEBUG(DBG_ERROR, "Reallocate Cid[%d][%d]:%d \n"
                                       , pdata->ril_cmd->SimId, i, pdp_resp[pdata->ril_cmd->SimId][i].cid);
                        }
                    }
                }
            }
			//lvjinlong@wind-mobi.com 2012.03.21 end
            FillDataResponseAddress(pdata->ril_cmd->SimId, rdata->cid, rdata->pdpAddressIPV4, rdata->pdpAddressIPV6);

	    //read prot config to get DNS info
#if 0  //TODO: for early stage ipv6 debugging, will be removed in the later version.
            printk("PDP Activate:  protConfig length = %d\r\n, options=", rsp->activatedContext.protConfig.length);
            for (i=0; i<PCH_PROT_CONFIG_OPTIONS_SIZE;i++)
            {
                printk("%02x ", rsp->activatedContext.protConfig.options[i]);
            }
            printk("\r\n");
#endif

            CAPI2_PchExApi_GetDecodedProtConfig(InitClientInfo(pdata->ril_cmd->SimId), gContext.cid); 

	    pdata->result = BCM_E_SUCCESS;
            pdata->handler_state = BCM_ReadProtConfig;
        }
        break;


	case BCM_ReadProtConfig:
	{
	    CAPI2_PchExApi_GetDecodedProtConfig_Rsp_t* resp = NULL; 
            KrilPdpData_t *rdata = pdata->bcm_ril_rsp;
	    int i;

            if ( (RESULT_OK != capi2_rsp->result) || (NULL == capi2_rsp->dataBuf) )
            {
                KRIL_DEBUG(DBG_ERROR, "PDPActivate Fail to CAPI2_PchExApi_GetDecodedProtConfig[%d]\n", gContext.cid);
                ReleasePdpContext(pdata->ril_cmd->SimId, gContext.cid);
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
				
		KRIL_SetInSetupPDPHandler(FALSE);
                break;
            }

	    resp = (CAPI2_PchExApi_GetDecodedProtConfig_Rsp_t*)capi2_rsp->dataBuf;
	  
	    if ( (strcmp(gContext.pdpType, "IP") == 0) ||
		 (strcmp(gContext.pdpType, "IPV4V6") == 0) )
            {
	        int hasData=0;
                for (i=0; i<MAX_DNS_ADDR_LEN;i++)
                {
                    if (resp->outParam.dnsPri[i]!=0)
		    {
			hasData = 1;
			break;
		    }
                }
		if (hasData)
		{
	            memcpy(rdata->priDNS, resp->outParam.dnsPri, MAX_DNS_ADDR_LEN);
		}
		else
		{
	            memcpy(rdata->priDNS, resp->outParam.dnsPri1, MAX_DNS_ADDR_LEN);
		}
	
                for (i=0; i<MAX_DNS_ADDR_LEN;i++)
                {
                    if (resp->outParam.dnsSec[i]!=0)
		    {
			hasData = 1;
			break;
		    }
                }
		if (hasData)
		{
	            memcpy(rdata->secDNS, resp->outParam.dnsSec, MAX_DNS_ADDR_LEN);
		}
		else
		{
	            memcpy(rdata->secDNS, resp->outParam.dnsSec1, MAX_DNS_ADDR_LEN);
		}
	    }
#if 0	    //just for testing now
	    //check if any data in dns
            printk("dnsPri=");
            for (i=0; i<MAX_DNS_ADDR_LEN;i++)
            {
                printk("%02x ", rdata->priDNS[i]);
            }
            printk("\r\n");
            printk("dnsSec=");
            for (i=0; i<MAX_DNS_ADDR_LEN;i++)
            {
                printk("%02x ", rdata->secDNS[i]);
            }
            printk("\r\n");
#endif


#if 1  //TODO, once CP integration is ready
	    if ( (strcmp(gContext.pdpType, "IPV6") == 0) ||
		 (strcmp(gContext.pdpType, "IPV4V6") == 0) )
            {
#if 0
                printk("v6dnsPri=");
                for (i=0; i<MAX_DNS_ADDR_LEN;i++)
                {
                    printk("%02x ", resp->outParam.v6dnsPri[i]);
                }
                printk("\r\n");
                printk("v6dnsSec=");
                for (i=0; i<MAX_DNS_ADDR_LEN;i++)
                {
                    printk("%02x ", resp->outParam.v6dnsSec[i]);
                }
                printk("\r\n");
 #endif
 
	        memcpy(rdata->priDNSIPV6, resp->outParam.v6dnsPri, MAX_DNS_ADDR_LEN);
	        memcpy(rdata->secDNSIPV6, resp->outParam.v6dnsSec, MAX_DNS_ADDR_LEN);
	    }
#endif
	    KRIL_SetInSetupPDPHandler(FALSE);

            pdata->result = BCM_E_SUCCESS;
            pdata->handler_state = BCM_FinishCAPI2Cmd;
	}
	break;

        default:
        {
            KRIL_SetInSetupPDPHandler(FALSE);
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}



void KRIL_DeactivatePdpHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (NULL != capi2_rsp)
        KRIL_DEBUG(DBG_INFO, "BCM_RESPCAPI2Cmd::result:0x%x\n", capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            char *cid = (char *)(pdata->ril_cmd->data);
            UInt8 ContextID = (UInt8)(*cid - 0x30);
            UInt8 i;

            KRIL_DEBUG(DBG_INFO, "KRIL_DeactivatePdpHandler - length %d, Cid:%d \n", pdata->ril_cmd->datalen, ContextID);
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilPdpData_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate pdata->bcm_ril_rsp buf\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
            pdata->rsp_len = sizeof(KrilPdpData_t);
            memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
            for (i=0 ; i<BCM_NET_MAX_RIL_PDP_CNTXS ; i++)
            {
                if (ContextID == pdp_resp[pdata->ril_cmd->SimId][i].cid) // To find the contextID in pdp_resp list, need to deactivate the context
                {
                    // found the active context we're looking for...
                    KRIL_DEBUG(DBG_INFO, "ReleasePdpContext[%d]=%d \n", i, pdp_resp[pdata->ril_cmd->SimId][i].cid);
                    break;
                }
                else if ((BCM_NET_MAX_RIL_PDP_CNTXS-1) == i) // Return finish state if can't find the contestID in pdp_resp list.
                {
                    // no match, so we assume context has already been deactivated...
                    KrilPdpData_t* rspData = pdata->bcm_ril_rsp;
                    // pass the cid back down to URIL; will be required for shutting down
                    // network interface
                    rspData->cid = ContextID;
                    KRIL_DEBUG(DBG_INFO, "no active context with cid %d \n", ContextID);
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
                    return;
                }
            }
            CAPI2_PchExApi_SendPDPDeactivateReq(InitClientInfo(pdata->ril_cmd->SimId), ContextID);
            ReleasePdpContext(pdata->ril_cmd->SimId, ContextID);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "PDPDeActivate Fail to SendPDPDeActivateReq \n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }

            if(NULL != capi2_rsp->dataBuf)
            {
                PDP_SendPDPDeactivateReq_Rsp_t *rsp = (PDP_SendPDPDeactivateReq_Rsp_t *)capi2_rsp->dataBuf;
                KrilPdpData_t *rdata = pdata->bcm_ril_rsp;
                if(rsp->response != PCH_REQ_ACCEPTED)
                {
                    KRIL_DEBUG(DBG_ERROR, "PDPDeActivate Fail resp(1 accept) %d, cid %d\r\n", rsp->response, rsp->cid);
                    pdata->result = BCM_E_RADIO_NOT_AVAILABLE;
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                    break;
                }

                rdata->cid = rsp->cid;
                KRIL_DEBUG(DBG_INFO, "PDP Deactivate Resp - cid %d \n", rsp->cid);

                pdata->result = BCM_E_SUCCESS;
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                pdata->result = BCM_E_GENERIC_FAILURE;
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

void KRIL_DataStateHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    static KrilDataState_t gDataState;

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
            if (NULL == pdata->ril_cmd->data)
            {
                KRIL_DEBUG(DBG_ERROR, "Enter Data State Fail with NULL data\n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }
            memcpy(&gDataState, (KrilDataState_t *)(pdata->ril_cmd->data), sizeof(KrilDataState_t));
            CAPI2_PdpApi_GetPCHContextState(InitClientInfo(pdata->ril_cmd->SimId),gDataState.cid);
            pdata->handler_state = BCM_PDP_Verify;
        }
        break;

        case BCM_PDP_Verify:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "Fail to send Enter Data State \n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            } 
            if(NULL != capi2_rsp->dataBuf)
            {
                PCHContextState_t *rsp = (PCHContextState_t *)capi2_rsp->dataBuf;
                KRIL_DEBUG(DBG_INFO, "[BCM_PDP_Verify] - rsp:: %d  *rsp:: %d \n", rsp, *rsp);
                if((gDataState.cid != NULL) && (*rsp== CONTEXT_UNDEFINED))
                {
                    KRIL_DEBUG(DBG_ERROR, "[BCM_PDP_Verify]::CONTEXT_UNDEFINED\n");
                    pdata->result = BCM_E_GENERIC_FAILURE;
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                    break;
                }
                CAPI2_PdpApi_DeactivateSNDCPConnection(InitClientInfo(pdata->ril_cmd->SimId), gDataState.cid );
            }
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "KRIL_DataStateHandler - Fail to send Enter Data State \n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "KRIL_DataStateHandler - RESULT_OK-> result:0x%x\n\n",  capi2_rsp->result);
            }

            if(NULL != capi2_rsp->dataBuf)
            {
                PDP_DataState_t *rsp = (PDP_DataState_t *)capi2_rsp->dataBuf;

                if(rsp->response != PCH_REQ_ACCEPTED)
                {
                    KRIL_DEBUG(DBG_ERROR, "Enter data state Fail resp(1 accept) %d, \r\n", rsp->response);
                    pdata->result = BCM_E_RADIO_NOT_AVAILABLE;
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                }
                else
                {
                    KRIL_DEBUG(DBG_ERROR, "Enter data state Successfull %d, \r\n", rsp->response);
                    pdata->bcm_ril_rsp = kmalloc(sizeof(KrilDataState_t), GFP_KERNEL);
                    if(!pdata->bcm_ril_rsp) {
                        KRIL_DEBUG(DBG_ERROR, "unable to allocate pdata->bcm_ril_rsp buf\n");
                        pdata->handler_state = BCM_ErrorCAPI2Cmd;                
                    }
                    else
                    {
                        pdata->rsp_len = sizeof(KrilDataState_t);
                        memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
    
                        //KrilDataState_t *rdata = pdata->bcm_ril_rsp;
                        //rdata->cid = rsp->cid;
                        //KRIL_DEBUG(DBG_ERROR, "Enter Data State Res- cid %d \n", rsp->cid);
                        pdata->result = BCM_E_SUCCESS;
                        pdata->handler_state = BCM_FinishCAPI2Cmd;
                    }
                }
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

KRIL_DEBUG(DBG_ERROR, "KRIL_DataStateHandler - Command found...!\n");
//KRIL_SendResponse(cmd_list);
}

void KRIL_SendDataHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    static KrilSendData_t gSendData;
    KRIL_DEBUG(DBG_INFO, "KRIL_SendDataHandler Entered \n");

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
            KRIL_DEBUG(DBG_INFO, "KRIL_SendDataHandler Entered::BCM_SendCAPI2Cmd \n");
            if (NULL == pdata->ril_cmd->data)
            {
                KRIL_DEBUG(DBG_ERROR, "Send Data Fail with NULL data\n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }
            memcpy(&gSendData, (KrilSendData_t *)(pdata->ril_cmd->data), sizeof(KrilSendData_t));

            KRIL_DEBUG(DBG_INFO, "KRIL_SendDataHandler - Send Data : CID %d \n", gSendData.cid);
            //KRIL_DEBUG(DBG_ERROR, "KRIL_SendDataHandler - Send Data : NumberofBytes %d \n", gSendData.numberBytes);
            CAPI2_PdpApi_GetPCHContextState(InitClientInfo(pdata->ril_cmd->SimId),gSendData.cid);
            pdata->handler_state = BCM_PDP_Verify;
        }
        break;

        case BCM_PDP_Verify:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "KRIL_DataStateHandler - Fail to send Enter Data State \n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            } 
            if(NULL != capi2_rsp->dataBuf)
            {
                PCHContextState_t *rsp = (PCHContextState_t *)capi2_rsp->dataBuf;		
                KRIL_DEBUG(DBG_INFO, "[BCM_PDP_Verify] - *rsp:: %d \n", *rsp);
                if((gSendData.cid != NULL) && (*rsp == CONTEXT_UNDEFINED))
                {
                    KRIL_DEBUG(DBG_ERROR, "[BCM_PDP_Verify]::CONTEXT_UNDEFINED\n");
                    //KRIL_DEBUG(DBG_ERROR, "%d CID not supported\n", gDataState.cid);
                    pdata->result = BCM_E_GENERIC_FAILURE;
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                    break;
                }
                CAPI2_PdpApi_SendTBFData(InitClientInfo(pdata->ril_cmd->SimId), gSendData.cid, gSendData.numberBytes);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "KRIL_SendDataHandler - Fail to send data \n");
                pdata->result = BCM_E_GENERIC_FAILURE;
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "KRIL_DataStateHandler - RESULT_OK-> result:0x%x\n\n",  capi2_rsp->result);		
            }
            pdata->result = BCM_E_SUCCESS;
            pdata->handler_state = BCM_FinishCAPI2Cmd;
        }
        break;
    }
}


void KRIL_SetPreferredDataHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    char* rawdata = (char*)pdata->ril_cmd->data;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            if (gdataprefer == ((int)rawdata[5] - 0x30))
            {
                KRIL_DEBUG(DBG_INFO, "MS Class to be changed is same as the previous one gdataprefer:%d rawdata[5]:%d\n", gdataprefer, ((int)rawdata[5] - 0x30));
                goto Finish; // return success if MS Class to be changed is same as the previous one
            }
            else
            {
                SimNumber_t simid = ((SimNumber_t)rawdata[5] - 0x30)+1;
                KRIL_DEBUG(DBG_INFO, "simid:%d\n", simid);
                CAPI2_NetRegApi_SetMSClass(InitClientInfo(SIMEXCHID[simid]), PCH_GPRS_CLASS_CC);
                pdata->handler_state = BCM_SET_GPRSClassCC;
            }
            break;
        }

        case BCM_SET_GPRSClassCC:
        {
            if (capi2_rsp && capi2_rsp->result != RESULT_OK)
            {
                KRIL_DEBUG(DBG_ERROR,"Set Class CC response failed:%d\n", capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                SimNumber_t simid = ((SimNumber_t)rawdata[5] - 0x30)+1;
                if (MSG_PDP_SETMSCLASS_RSP == capi2_rsp->msgType)
                {
                    msleep(500);
                    CAPI2_MsDbApi_GetElement(InitClientInfo(SIMEXCHID[simid]), MS_NETWORK_ELEM_REG_INFO);
                    pdata->handler_state = BCM_SET_GPRSClassCC;
                }
                else if (MSG_MS_GET_ELEMENT_RSP == capi2_rsp->msgType)
                {
                    MsState_t* presult = NULL;
                    CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*) capi2_rsp->dataBuf;
                    presult = (MsState_t*)(&(rsp->data_u));
                    KRIL_DEBUG(DBG_INFO,"gprs_status:%d\n", presult->gprs_status);
                    if (REGISTERSTATUS_SERVICE_DISABLED == presult->gprs_status ||
                        REGISTERSTATUS_NO_STATUS == presult->gprs_status)
                    {
                        CAPI2_NetRegApi_SetMSClass(InitClientInfo(simid), PCH_GPRS_CLASS_B);
                        *(UInt32*)(pdata->cmdContext) = (UInt32)simid;
                        pdata->handler_state = BCM_SET_GPRSClassB;
                    }
                    else
                    {
                        msleep(500);
                        CAPI2_MsDbApi_GetElement(InitClientInfo(SIMEXCHID[simid]), MS_NETWORK_ELEM_REG_INFO);
                        pdata->handler_state = BCM_SET_GPRSClassCC;
                    }
                }
                else // receive wrong message, revert the ms class B at previous SIM
                {
                    SimNumber_t simid = ((SimNumber_t)rawdata[5] - 0x30)+1;
                    KRIL_DEBUG(DBG_ERROR, "receive wrong message:0x%x simid:%d\n", capi2_rsp->msgType, simid);
                    CAPI2_NetRegApi_SetMSClass(InitClientInfo(SIMEXCHID[simid]), PCH_GPRS_CLASS_B);
                    *(UInt32*)(pdata->cmdContext) = (UInt32)SIMEXCHID[simid];
                    pdata->handler_state = BCM_REVERT_GPRSClassB;
                }
            }
            break;
        }

        case BCM_SET_GPRSClassB:
        {
            KRIL_DEBUG(DBG_ERROR,"CAPI2 response result:%d\n", capi2_rsp->result);
            if (capi2_rsp->result != RESULT_OK)
            {
                SimNumber_t simid = ((SimNumber_t)rawdata[5] - 0x30)+1;
                KRIL_DEBUG(DBG_ERROR, "Set Class B fail, revert the MS Class B at previous SIM:%d\n", simid);
                CAPI2_NetRegApi_SetMSClass(InitClientInfo(SIMEXCHID[simid]), PCH_GPRS_CLASS_B);
                *(UInt32*)(pdata->cmdContext) = (UInt32)SIMEXCHID[simid];
                pdata->handler_state = BCM_REVERT_GPRSClassB;
            }
            else // receive wrong message, so revert the ms class
            {
                UInt32 simid = *(UInt32*)(pdata->cmdContext);
                KRIL_DEBUG(DBG_ERROR, "Set Class B success to control NWI PDP SIM:%d\n", simid);
                CAPI2_PdpApi_SetPDPNWIControlFlag(InitClientInfo(simid), TRUE);
                goto Finish; // return success if MS Class to be changed is same as the previous one
            }
            break;
        }

        case BCM_REVERT_GPRSClassB:
        {
            UInt32 simid = *(UInt32*)(pdata->cmdContext);
            KRIL_DEBUG(DBG_ERROR, "revert the MS Class to control NWI PDP SIM:%d\n", simid);
            CAPI2_PdpApi_SetPDPNWIControlFlag(InitClientInfo(simid), TRUE);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }

Finish:
        case BCM_RESPCAPI2Cmd:
        {
            UInt8 *resp = NULL;
            pdata->bcm_ril_rsp = kmalloc(sizeof(UInt8)*5, GFP_KERNEL);
            if(capi2_rsp != NULL)
                KRIL_DEBUG(DBG_ERROR,"BCM_RESPCAPI2Cmd::CAPI2 response result:%d\n", capi2_rsp->result);

            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate pdata->bcm_ril_rsp buf\n");
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                pdata->rsp_len = sizeof(UInt8)*5 ;
                resp = (UInt8*)pdata->bcm_ril_rsp;
                resp[0] = (UInt8)'B';
                resp[1] = (UInt8)'R';
                resp[2] = (UInt8)'C';
                resp[3] = (UInt8)'M';
                resp[4] = (UInt8)BRIL_HOOK_SET_PREFDATA;
                KRIL_DEBUG(DBG_INFO, "Set Class success gdataprefer:%d rawdata[5]:%d\n", gdataprefer, ((int)rawdata[5] - 0x30));
                gdataprefer = ((int)rawdata[5] - 0x30);
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



static void SendCHAPOptions(KrilPdpContext_t* context)
{
    ClientInfo_t clientInfo;

#if 0  //RIL doesn't provide this info yet, TODO
    CHAP_ChallengeOptions_t challOptions;
    CHAP_ResponseOptions_t respOptions;

    KRIL_DEBUG(DBG_INFO, "SendCHAPOptions chall_len=%d, resp_len=%d\r\n", context->chall_len, context->resp_len);

    CAPI2_InitClientInfo( &clientInfo, KRILGetNewTID(), KRILGetClientID());

    challOptions.flag=1;
    challOptions.len =context->chall_len;
    memcpy(challOptions.content, context->challenge, PDP_CHALLENGE_LEN_MAX);

    respOptions.flag=1;
    respOptions.len =context->resp_len;
    memcpy(respOptions.content, context->response, PDP_RESPONSE_LEN_MAX);

    CAPI2_PchExApi_BuildIpConfigOptions2(&clientInfo, REQUIRE_CHAP, &challOptions, &respOptions, NULL);
#else
    CAPI2_InitClientInfo( &clientInfo, GetNewTID(), GetClientID());

    KRIL_DEBUG(DBG_INFO, "SendCHAPOptions \r\n");
    CAPI2_PchExApi_BuildIpConfigOptions(&clientInfo, context->username, context->password, REQUIRE_CHAP);
#endif
}

static void SendPAPOptions(KrilPdpContext_t* context)
{
    ClientInfo_t clientInfo;
    CAPI2_InitClientInfo( &clientInfo, GetNewTID(), GetClientID());

    KRIL_DEBUG(DBG_INFO, "SendPAPOptions \r\n");
    CAPI2_PchExApi_BuildIpConfigOptions(&clientInfo, context->username, context->password, REQUIRE_PAP);
}



void KRIL_RejectNWIPdpActivate(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (NULL != capi2_rsp)
        KRIL_DEBUG(DBG_INFO, "BCM_RESPCAPI2Cmd::result:0x%x\n", capi2_rsp->result);

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            PDP_ActivateNWI_Ind_t *pCmd = NULL;;
            if (NULL == pdata->ril_cmd->data)
            {
                KRIL_DEBUG(DBG_ERROR, "Enter Reject NWI PDP Fail with NULL data\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
            }
            pCmd = (PDP_ActivateNWI_Ind_t *)pdata->ril_cmd->data;
            KRIL_DEBUG(DBG_INFO, "Reject NWI PDP Activation\n");
            CAPI2_PdpApi_RejectNWIPDPActivation(InitClientInfo(pdata->ril_cmd->SimId), pCmd->pdpAddress, ACTIVATION_REJECTED_UNSPECIFIED, pCmd->apn);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
        }
        break;

        case BCM_RESPCAPI2Cmd:
        {
            KRIL_DEBUG(DBG_INFO, "result:0x%x\n", capi2_rsp->result);
            if(RESULT_OK != capi2_rsp->result)
            {
                KRIL_DEBUG(DBG_ERROR, "Reject NWI PDP Fail to Send\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                break;
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
