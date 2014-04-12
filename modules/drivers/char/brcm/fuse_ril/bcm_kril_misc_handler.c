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
#include "capi2_stk_ds.h"
#include "capi2_pch_msg.h"
#include "capi2_gen_msg.h"
#include "capi2_reqrep.h"
#include "capi2_gen_cp_api.h"

#include <linux/broadcom/bcm_fuse_sysparm_CIB.h>
#include "capi2_cc_api.h"
#include "netreg_def.h"
#include "capi2_stk_ds.h"
#include "capi2_pch_msg.h"
#include "capi2_gen_msg.h"
#include "capi2_reqrep.h"
#include <linux/reboot.h>	 /* For kernel_power_off() */
//songjinguo@wind-mobi.com 2011.01.07 start
//CSP 491199 patch; for quickly refresh sighal value; 
//review by hebin 
//sunanguo@wind-mobi.com 2012.5.18 start
//fix bug#9458
//review by hebin
#if defined(CONFIG_BOARD_L400)
UInt8 g_RSSIThreshold = 5;
#else
UInt8 g_RSSIThreshold = 1;
#endif
//sunanguo@wind-mobi.com 2012.5.18 end
//songjinguo@mobi.com 2012.01.07 end
int gdataprefer = 0; // SIM1:0 SIM2:1

// Assume flight mode is on unless Android framework requests to set radio power
Boolean gIsFlightModeOnBoot[DUAL_SIM_SIZE] = {TRUE, TRUE, TRUE};

// Store whether Dual SIM boot up
Boolean gDualSIMBoot = FALSE;

// In STK refresh reset case, it need to turn off then turn on SIM card.
Boolean gIsStkRefreshReset[DUAL_SIM_SIZE] = {FALSE, FALSE, FALSE};

// For STK
//lvjinlong@wind-mobi.com
//bcm patch

//UInt8 terminal_profile_data[30] = {0xFF,0xDF,0xFF,0xFF,0x7F,0x81,0x00,0xDF,0xDF,0x00,0x00,

UInt8 terminal_profile_data[30] = {0xFF,0xFF,0xFF,0xFF,0x7F,0x81,0x00,0xDF,0xDF,0x00,0x00,

                                      0x00,0x00,0x10,0x20,0xA6,0x00,0xC0,0x00,0x00,0x00,0x02,
                                      0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//lvjinlong@wind-mobi.com
// IMEI information
#ifdef CONFIG_BRCM_SIM_SECURE_ENABLE
static Boolean ProcessImei(UInt8* imeiStr, UInt8* imeiVal);
#endif // CONFIG_BRCM_SIM_SECURE_ENABLE

UInt8 sImei_Info[DUAL_SIM_SIZE][BCD_IMEI_LEN] = {{0},{0},{0}};

#define QueryMeasureReportLen   800
char QueryMeasureReportBuf[QueryMeasureReportLen];

// for SIMID
extern SimNumber_t SIMEXCHID[DUAL_SIM_SIZE];

static void ParseIMSIData(KRIL_CmdList_t *pdata, Kril_CAPI2Info_t *capi2_rsp);
static void ParseIMEIData(KRIL_CmdList_t *pdata, Kril_CAPI2Info_t *capi2_rsp);

// number of characters in IMEI string (15 digit IMEI plus check digit, plus 1 for null termination)
#define IMEI_STRING_LEN (IMEI_DIGITS+1)

#define isdigit(c) ((c) >= '0' && (c) <= '9')

// Support Band Mode for System
extern BandSelect_t g_systemband;
//luobiao@wind-mobi.com start 2012.05.17
//patch for csp:524605
#ifdef SEND_STOP_CB_SERVICE_IN_FLIGHT_MODE
extern Boolean g_bStopCBServiceAtFlightMode;
#endif
//luobiao@wind-mobi.com end 2012.05.17

// external utility function to convert RIL network type value to equivalent RATSelect_t value
extern RATSelect_t ConvertNetworkType(int type);

// external utility functions to convert RIL band mode to set of BandSelect_t values
extern BandSelect_t ConvertBandMode(int mode);

extern void ProcessGSMStatus(Kril_CAPI2Info_t* data);

void KRIL_InitCmdHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    CAPI2_MS_Element_t data;

    if (capi2_rsp != NULL)
    {
        if(capi2_rsp->result != RESULT_OK)
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
        }
    }

    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            KrilInit_t *pInitData = (KrilInit_t *)(pdata->ril_cmd->data);
            KRIL_SetASSERTFlag(pInitData->assertflag);
            // if there is a valid IMEI, make appropriate CAPI2 call to set
            // IMEI on CP, otherwise fall through to next init command
            if (SIM_DUAL_FIRST == pdata->ril_cmd->SimId)
            {
                if (pInitData->is_valid_imei)
                {
                    KRIL_DEBUG(DBG_INFO, "OTP IMEI[%d]:%s\n", pdata->ril_cmd->SimId, pInitData->imei);
                
#ifdef CONFIG_BRCM_SIM_SECURE_ENABLE
                    // Record IMEI1 infomation  
                    if (FALSE == ProcessImei((UInt8*)pInitData->imei, sImei_Info[pdata->ril_cmd->SimId]))
                    {
                        KRIL_DEBUG(DBG_ERROR,"Process IMEI[%d]:%s Failed!!!",pdata->ril_cmd->SimId, pInitData->imei);
                        pdata->handler_state = BCM_ErrorCAPI2Cmd;
                        kernel_power_off();
                    }
#endif //CONFIG_BRCM_SIM_SECURE_ENABLE
                
                    memset(&data, 0, sizeof(CAPI2_MS_Element_t));
                    memcpy(data.data_u.imeidata, pInitData->imei, IMEI_DIGITS);
                    data.inElemType = MS_LOCAL_PHCTRL_ELEM_IMEI;
                    CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);                    
                    pdata->handler_state = BCM_SET_AMR_TYPE;
                    break;
                }
#ifdef CONFIG_BRCM_SIM_SECURE_ENABLE 
                else
                {
                    // For secure boot, the IMEI is important inform for verifying SIM lock data.
                    KRIL_DEBUG(DBG_ERROR, "IMEI[%d] is invalid. Error!!!\n",pdata->ril_cmd->SimId);
                    kernel_power_off();
                }
#endif //CONFIG_BRCM_SIM_SECURE_ENABLE 
            }
            else if ((SIM_DUAL_SECOND == pdata->ril_cmd->SimId) && pInitData->is_valid_imeiEx)
            {
                KRIL_DEBUG(DBG_INFO, "OTP IMEI[%d]:%s\n", pdata->ril_cmd->SimId, pInitData->imeiEx);
                // Record IMEI2 infomation
                memset(&data, 0, sizeof(CAPI2_MS_Element_t));
                memcpy(data.data_u.imeidata, pInitData->imeiEx, IMEI_DIGITS);
                data.inElemType = MS_LOCAL_PHCTRL_ELEM_IMEI;
                CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
                pdata->handler_state = BCM_SET_AMR_TYPE;
                break;
            }
            // if OTP IMEI passed from URIL is not valid, we skip the
            // CAPI2_MS_SetElement() call and fall through to execute the
            // next CAPI2 init call instead...
        }

        case BCM_SET_AMR_TYPE:
        {
            KrilInit_t *pInitData = (KrilInit_t *)(pdata->ril_cmd->data);
            Boolean flag;
    
            KRIL_DEBUG(DBG_ERROR,"SimID:%d AMR TYPE: %d, %d\n", pdata->ril_cmd->SimId, pInitData->amrtype, pInitData->amrtypeEx);
            if(SIM_DUAL_FIRST == pdata->ril_cmd->SimId) {
                if(pInitData->amrtype != -1)
                {//0 for NB-AMR , 1 for WB-AMR and 2 for Invalid AMR Mode
                    if(pInitData->amrtype == 0){//disable WB AMR
                        flag = FALSE;
                    }
                    else
                    if(pInitData->amrtype == 1){//enable WB AMR
                        flag = TRUE;
                    }
                    KRIL_DEBUG(DBG_ERROR,"SimID:%d flag: %d\n", pdata->ril_cmd->SimId, flag);
                    CAPI2_CcApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), CC_ELEM_UMTS_WB_AMR , NULL , (void *)&flag);
                }
                else
                    KRIL_DEBUG(DBG_ERROR,"SimID:%d, User does not set AMR Type\n", pdata->ril_cmd->SimId);
            }
            else
            if (SIM_DUAL_SECOND == pdata->ril_cmd->SimId) 
            {
                if(pInitData->amrtypeEx != -1)
                {//0 for NB-AMR , 1 for WB-AMR and 2 for Invalid AMR Mode
                    if(pInitData->amrtypeEx == 0){//disable WB AMR
                        flag = FALSE;
                    }
                    else
                    if(pInitData->amrtypeEx == 1){//enable WB AMR
                        flag = TRUE;
                    }
                    KRIL_DEBUG(DBG_ERROR,"SimID:%d flag: %d\n", pdata->ril_cmd->SimId, flag);
                    CAPI2_CcApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), CC_ELEM_UMTS_WB_AMR , NULL, (void *)&flag);
                }
                else
                    KRIL_DEBUG(DBG_ERROR,"SimID:%d, User does not set AMR Type\n", pdata->ril_cmd->SimId);
    
            }
            pdata->handler_state = BCM_SMS_ELEM_CLIENT_HANDLE_MT_SMS;
            break;
        }

        case BCM_SMS_ELEM_CLIENT_HANDLE_MT_SMS:
        {
            memset((UInt8*)&data, 0, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_SMS_ELEM_CLIENT_HANDLE_MT_SMS;
            data.data_u.bData = TRUE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            if (SIM_DUAL_FIRST == pdata->ril_cmd->SimId) // SIM2 does not need to set the MS class, RAT and Band mode
                pdata->handler_state = BCM_Demand_Attach_Always;
            else
                pdata->handler_state = BCM_SMS_SetSmsReadStatusChangeMode;
            break;
        }

        case BCM_Demand_Attach_Always:
        {
            memset((UInt8*)&data, 0, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_PHCTRL_ELEM_ON_DEMAND_ATTACH_ALWAYS;
            data.data_u.bData = FALSE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_Demand_SIM2_Attach_Always;
            break;
        }

        case BCM_Demand_SIM2_Attach_Always:
        {
            memset((UInt8*)&data, 0, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_PHCTRL_ELEM_ON_DEMAND_ATTACH_ALWAYS;
            data.data_u.bData = FALSE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(SIMEXCHID[pdata->ril_cmd->SimId]), &data);
            pdata->handler_state = BCM_SET_GPRSClassCC;
            break;
        }

        case BCM_SET_GPRSClassCC:
        {
            KrilInit_t *pInitData = (KrilInit_t *)(pdata->ril_cmd->data);
            KRIL_DEBUG(DBG_INFO, "dataprefer:%d\n", pInitData->dataprefer);
            gdataprefer = pInitData->dataprefer;
            if (0 == pInitData->dataprefer) // SIM1:0 SIM2:1
            {
                pdata->ril_cmd->SimId = SIM_DUAL_SECOND;
            }
            CAPI2_NetRegApi_SetMSClass(InitClientInfo(pdata->ril_cmd->SimId), PCH_GPRS_CLASS_CC);
            pdata->handler_state = BCM_SET_GPRSClassB;
            break;
        }

        case BCM_SET_GPRSClassB:
        {
            CAPI2_NetRegApi_SetMSClass(InitClientInfo(SIMEXCHID[pdata->ril_cmd->SimId]), PCH_GPRS_CLASS_B);
            pdata->ril_cmd->SimId = SIM_DUAL_FIRST;
            pdata->handler_state = BCM_SET_PDP_NWI_CTRL_FLAG;
            break;
        }
        
        case BCM_SET_PDP_NWI_CTRL_FLAG:
        {
            KrilInit_t *pInitData = (KrilInit_t *)(pdata->ril_cmd->data);
            KRIL_DEBUG(DBG_INFO, "PDP_NWI_CTRL_FLAG dataprefer:%d\n", pInitData->dataprefer);
            if (0 == pInitData->dataprefer) // SIM1:0 SIM2:1
            {
                CAPI2_PdpApi_SetPDPNWIControlFlag(InitClientInfo(SIM_DUAL_FIRST), TRUE);
            }
            else
            {
                CAPI2_PdpApi_SetPDPNWIControlFlag(InitClientInfo(SIM_DUAL_SECOND), TRUE);
            }
            pdata->handler_state = BCM_SET_BPMSetting;
            break;
        }

        case BCM_SET_BPMSetting:
        {
            KrilInit_t *pInitData = (KrilInit_t *)(pdata->ril_cmd->data);
            KRIL_DEBUG(DBG_INFO, "bpmsetting:%d\n", pInitData->bpmsetting);
            CAPI2_PhoneCtrlApi_SetPagingStatus(InitClientInfo(pdata->ril_cmd->SimId), (UInt8)pInitData->bpmsetting);
            pdata->handler_state = BCM_SET_SupportedRATandBand;
            break;
        }

        case BCM_SET_SupportedRATandBand:
        {
            KrilInit_t *pInitData = (KrilInit_t *)(pdata->ril_cmd->data);
            KRIL_SetPreferredNetworkType(SIM_DUAL_FIRST, pInitData->networktype);
            KRIL_SetPreferredNetworkType(SIM_DUAL_SECOND, pInitData->networktypeEx);
            KRIL_DEBUG(DBG_INFO,"networktype:%d networktypeEx:%d band:%d\n", pInitData->networktype, pInitData->networktypeEx, pInitData->band);
            CAPI2_NetRegApi_SetSupportedRATandBand(InitClientInfo(pdata->ril_cmd->SimId), ConvertNetworkType(pInitData->networktype), ConvertBandMode(pInitData->band), ConvertNetworkType(pInitData->networktypeEx), ConvertBandMode(pInitData->band));
            pdata->handler_state = BCM_SMS_SetSmsReadStatusChangeMode;
            break;
        }

        case BCM_SMS_SetSmsReadStatusChangeMode:
        {
            CAPI2_SmsApi_SetSmsReadStatusChangeMode(InitClientInfo(pdata->ril_cmd->SimId), FALSE);
            pdata->handler_state = BCM_SMS_SetVMIndOnOff;
            break;
        }

        case BCM_SMS_SetVMIndOnOff:
        {
            CAPI2_SmsApi_SetVMIndOnOff(InitClientInfo(pdata->ril_cmd->SimId), TRUE);
            pdata->handler_state = BCM_SYS_SetFilteredEventMask;
            break;
        }

        case BCM_SYS_SetFilteredEventMask:
        {
            UInt16 filterList[]={MSG_RSSI_IND, MSG_CELL_INFO_IND, MSG_LCS_RRC_UE_STATE_IND, 
                                 MSG_DATE_TIMEZONE_IND, MSG_DATA_SUSPEND_IND, 
                                 MSG_DATA_RESUME_IND, MSG_CAPI2_AT_RESPONSE_IND, 
                                 MSG_UE_3G_STATUS_IND, MSG_AT_COMMAND_IND, MSG_SIM_INSTANCE_STATUS_IND};
            CAPI2_SYS_SetFilteredEventMask(GetNewTID(), DEFAULT_CLIENT_ID, &filterList[0], sizeof(filterList)/sizeof(UInt16), SYS_AP_DEEP_SLEEP_MSG_FILTER);
            pdata->handler_state = BCM_SYS_SetRssiThreshold;
            break;
        }

        case BCM_SYS_SetRssiThreshold:
        {
            CAPI2_PhoneCtrlApi_SetRssiThreshold(InitClientInfo(pdata->ril_cmd->SimId), g_RSSIThreshold, 20, g_RSSIThreshold, 20);
            pdata->handler_state = BCM_TIMEZONE_SetTZUpdateMode;
            break;
        }

        case BCM_TIMEZONE_SetTZUpdateMode:
        {
            CAPI2_NetRegApi_SetTZUpdateMode(InitClientInfo(pdata->ril_cmd->SimId), TIMEZONE_UPDATEMODE_NO_TZ_UPDATE);
            pdata->handler_state = BCM_SATK_SetTermProfile;
            break;
        }

        case BCM_SATK_SetTermProfile:
        {
            CAPI2_SatkApi_SetTermProfile(InitClientInfo(pdata->ril_cmd->SimId), terminal_profile_data,
                sizeof(terminal_profile_data)/sizeof(UInt8));
            pdata->handler_state = BCM_SATK_SETUP_CALL_CTR;
            break;
        }

        case BCM_SATK_SETUP_CALL_CTR:
        {
            memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_SATK_ELEM_SETUP_CALL_CTR;
            data.data_u.bData = TRUE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_SATK_ICON_DISP_SUPPORTED;
            break;
        }

        case BCM_SATK_ICON_DISP_SUPPORTED:
        {
            memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_SATK_ELEM_ICON_DISP_SUPPORTED;
            data.data_u.bData = TRUE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_SATK_ELEM_ENABLE_TEXT_CONVERSIONS;            
            break;
        }

        case BCM_SATK_ELEM_ENABLE_TEXT_CONVERSIONS:
        {
            // Disable CAPI perform string conversion
            memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_SATK_ELEM_ENABLE_TEXT_CONVERSIONS;
			//lvjinlong@wind-mobi.com start 2012.02.01
			//fix bug 7184
            //data.data_u.bData = FALSE;
			data.data_u.bData = TRUE;
			//lvjinlong@wind-mobi.com end 2012.02.01
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_SET_SIM_LOCK_SUPPORTED;            
            break;
        }

        case BCM_SET_SIM_LOCK_SUPPORTED:
        {
            memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_CFG_ELEM_SIM_LOCK_SUPPORTED;
            data.data_u.u8Data = 1;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_AT_AUDIO_CTRL;
            break;
        }
                
        case BCM_AT_AUDIO_CTRL:
        {
            memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_LOCAL_AT_ELEM_AUDIO_CTRL ;
            data.data_u.bData = FALSE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_SET_STACK_CTRL;
            break;
        }

        case BCM_SET_STACK_CTRL: // 2012.2.2; add stack setting here for SOR, etc
        {
            memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
            data.inElemType = MS_STACK_ELEM_NVRAM_CLASSMARK;
            data.data_u.stackClassmark.nasConfigParams.u2_sor_support_updated = FALSE;
            data.data_u.stackClassmark.nasConfigParams.u2_sor_support= FALSE;
            CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
            pdata->handler_state = BCM_SET_RADIO_OFF;
            break;
        }

        case BCM_SET_RADIO_OFF:
        {
            // For flight mode power up battery ADC & deep sleep issue (MobC00131482), set the initial CP state to RADIO_OFF.
            // If MS is powered up in normal mode, Android framework will send BRCM_RIL_REQUEST_RADIO_POWER to RIL.
            if (SIM_DUAL_SECOND == pdata->ril_cmd->SimId) // SIM2 does not need to set no_RF state
            {
                gDualSIMBoot = TRUE;
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                CAPI2_PhoneCtrlApi_ProcessNoRfReq(InitClientInfo(pdata->ril_cmd->SimId));
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
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

#ifdef CONFIG_BRCM_RADIO_POWER_ONOFF_CARD
/*
     this  is implemented for meeting customer's requirement:
     If SIM card is locked, the PIN is required when the Airplane mode is turned off.
     But the PIN is NOT required  if using CAPI2_SYS_ProcessPowerUpReq to turned off Airplane mode.
     Workaround: Power off and then on the sim card, the CP will do the unlock sim process.
*/
// Don't off/on card if gIsSilencePoweron is TRUE
Boolean gIsSilencePoweron[DUAL_SIM_SIZE] = {TRUE, TRUE, TRUE};
extern void SetSimPinStatusHandle(SimNumber_t SimId, SIM_PIN_Status_t SimPinState);

void KRIL_RadioPowerHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
    {
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
        if(capi2_rsp->result != RESULT_OK)
        {
            pdata->result = RILErrorResult(capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            return;
        }
    }

    //gIsFlightModeOnBoot[pdata->ril_cmd->SimId] = FALSE;

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            int *OnOff = (int *)(pdata->ril_cmd->data);

            KRIL_DEBUG(DBG_INFO, "On-Off:%d gIsStkRefreshReset:%d\n", *OnOff, gIsStkRefreshReset[pdata->ril_cmd->SimId]);
           //luobiao@wind-mobi.com start 2012.05.17
		   //patch for csp:524605
		   pdata->handler_state = BCM_SMS_Start_Stop_CBService;			
		   //luobiao@wind-mobi.com end 2012.05.17

            if (TRUE == gIsStkRefreshReset[pdata->ril_cmd->SimId])
            {
                if (0 == *OnOff)
                {
                    KRIL_DEBUG(DBG_INFO, "Power off Sim card - Refresh\n");
                    CAPI2_SimApi_PowerOnOffCard (InitClientInfo(pdata->ril_cmd->SimId), FALSE, SIM_POWER_OFF_FORCE_MODE);
                }
                else
                {
                    KRIL_DEBUG(DBG_INFO, "Power on Sim card - Refresh\n");
                    CAPI2_SimApi_PowerOnOffCard (InitClientInfo(pdata->ril_cmd->SimId), TRUE, SIM_POWER_ON_NORMAL_MODE);
                    gIsStkRefreshReset[pdata->ril_cmd->SimId] = FALSE;
                }
            }
            else
            {
                if (1 == *OnOff)
                {
                    if (!gIsSilencePoweron[pdata->ril_cmd->SimId] 
						&& KRIL_GetSimAppType(pdata->ril_cmd->SimId) != SIM_APPL_INVALID)
                    {
                        /* workaround only if sim card is valid */
                        KRIL_DEBUG(DBG_INFO, "CAPI2_PhoneCtrlApi_GetSystemState\n");
                        CAPI2_PhoneCtrlApi_GetSystemState(InitClientInfo(pdata->ril_cmd->SimId));
                        pdata->handler_state = BCM_GET_SYSTEM_STATE_RSP;
                    }
                    else
                    {
                        CAPI2_PhoneCtrlApi_ProcessPowerUpReq(InitClientInfo(pdata->ril_cmd->SimId));
                    }
					gIsSilencePoweron[pdata->ril_cmd->SimId] = FALSE;
                }
                else
                {
                    CAPI2_PhoneCtrlApi_ProcessNoRfReq(InitClientInfo(pdata->ril_cmd->SimId));
                }
            }

            break;
        }
        case BCM_GET_SYSTEM_STATE_RSP:
        {
            if(capi2_rsp->result == RESULT_OK)
            {
                SystemState_t state = *(SystemState_t *)capi2_rsp->dataBuf;
                if (state == SYSTEM_STATE_ON_NO_RF)
                {
                    KRIL_DEBUG(DBG_INFO, "Turn off Sim card\n");
                    // Do not allow to send the radio state change notification to android framework when sim card is powered off.
                    gIsFlightModeOnBoot[pdata->ril_cmd->SimId] = TRUE;
                    /*sim card power off*/
                    CAPI2_SimApi_PowerOnOffCard(InitClientInfo(pdata->ril_cmd->SimId), FALSE, SIM_POWER_ON_INVALID_MODE);
                    pdata->handler_state = BCM_SET_SYSTEM_STATE;
                }
                else
                {
                    /* Normally power up if DUT is not in airplane mode */
                    CAPI2_PhoneCtrlApi_ProcessPowerUpReq(InitClientInfo(pdata->ril_cmd->SimId));
                   	//luobiao@wind-mobi.com start 2012.05.17
		   			//patch for csp:524605
		   			pdata->handler_state = BCM_SMS_Start_Stop_CBService;
		   			//luobiao@wind-mobi.com end 2012.05.17
                }
            }
            else
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            break;
        }
        case BCM_SET_SYSTEM_STATE:
        {
            if(capi2_rsp->result == RESULT_OK)
            {
                KRIL_DEBUG(DBG_INFO, "CAPI2_PhoneCtrlApi_SetSystemState\n");
                SetSimPinStatusHandle(pdata->ril_cmd->SimId, NO_SIM_PIN_STATUS); // sim card is off, set SimPinStatus to NO_SIM_PIN_STATUS
                CAPI2_PhoneCtrlApi_SetSystemState(InitClientInfo(pdata->ril_cmd->SimId), SYSTEM_STATE_ON);
                pdata->handler_state = BCM_SIM_POWER_ON_OFF_CARD;
            }
            else
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            break;
        }
        case BCM_SIM_POWER_ON_OFF_CARD:
        {
            if(capi2_rsp->result == RESULT_OK)
            {
                KRIL_DEBUG(DBG_INFO, "Turn on Sim card\n");
                /*sim card power on*/
                CAPI2_SimApi_PowerOnOffCard(InitClientInfo(pdata->ril_cmd->SimId), TRUE, SIM_POWER_ON_NORMAL_MODE);
                //luobiao@wind-mobi.com start 2012.05.17
		   		//patch for csp:524605 
		   		pdata->handler_state = BCM_SMS_Start_Stop_CBService;
		   		//luobiao@wind-mobi.com end 2012.05.17
            }
            else
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            break;
        }
        //luobiao@wind-mobi.com start 2012.05.17
		//patch for csp:524605 
		case BCM_SMS_Start_Stop_CBService:
        {
            pdata->bcm_ril_rsp = kmalloc(pdata->ril_cmd->datalen, GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                int *OnOff = (int *)(pdata->ril_cmd->data);

                memcpy(pdata->bcm_ril_rsp, pdata->ril_cmd->data, pdata->ril_cmd->datalen);
                pdata->rsp_len = pdata->ril_cmd->datalen;
                if (*OnOff == 1)
                {
                    KRIL_SetFlightModeState(pdata->ril_cmd->SimId, FALSE);
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
                }
                else if (*OnOff == 0)
                {
                    KRIL_SetFlightModeState(pdata->ril_cmd->SimId, TRUE);
#ifdef SEND_STOP_CB_SERVICE_IN_FLIGHT_MODE
                    if(g_bStopCBServiceAtFlightMode == TRUE) {
                        KRIL_DEBUG(DBG_ERROR, "SIMID:%d Stop CB Service in Flight Mode\n",pdata->ril_cmd->SimId);
                        CAPI2_SmsApi_StopReceivingCellBroadcastReq(InitClientInfo(pdata->ril_cmd->SimId));
                        pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
                    else
                    {                        
                        pdata->handler_state = BCM_FinishCAPI2Cmd;
                    }
#else                    
            pdata->handler_state = BCM_FinishCAPI2Cmd;
#endif
                }

            }            
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            if(capi2_rsp->result != RESULT_OK )
            {
                KRIL_DEBUG(DBG_ERROR, "SIMID:%d Fail, error result:%d \n",pdata->ril_cmd->SimId,capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }            
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }
		//luobiao@wind-mobi.com end 2012.05.17
        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }

    if (pdata->handler_state == BCM_ErrorCAPI2Cmd || pdata->handler_state == BCM_FinishCAPI2Cmd)
    {
        /* allow to send the radio state change notification to Android framework when request is done. */
        gIsFlightModeOnBoot[pdata->ril_cmd->SimId] = FALSE;
    }
}
#else

void KRIL_RadioPowerHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;

    if (capi2_rsp != NULL)
    {
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
        if(capi2_rsp->result != RESULT_OK)
        {
            pdata->result = RILErrorResult(capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            return;
        }
    }

    gIsFlightModeOnBoot[pdata->ril_cmd->SimId] = FALSE;

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            int *OnOff = (int *)(pdata->ril_cmd->data);

            KRIL_DEBUG(DBG_INFO, "On-Off:%d gIsStkRefreshReset:%d\n", *OnOff, gIsStkRefreshReset[pdata->ril_cmd->SimId]);

            if (TRUE == gIsStkRefreshReset[pdata->ril_cmd->SimId])
            {
                if (0 == *OnOff)
                {
                    KRIL_DEBUG(DBG_INFO, "Power off Sim card - Refresh\n");
                    CAPI2_SimApi_PowerOnOffCard (InitClientInfo(pdata->ril_cmd->SimId), FALSE, SIM_POWER_OFF_FORCE_MODE);
                }
                else
                {
                    KRIL_DEBUG(DBG_INFO, "Power on Sim card - Refresh\n");
                    CAPI2_SimApi_PowerOnOffCard (InitClientInfo(pdata->ril_cmd->SimId), TRUE, SIM_POWER_ON_NORMAL_MODE);
                    gIsStkRefreshReset[pdata->ril_cmd->SimId] = FALSE;
                }                
            }
            else
            {
                if (1 == *OnOff)
                {
                    CAPI2_PhoneCtrlApi_ProcessPowerUpReq(InitClientInfo(pdata->ril_cmd->SimId));
                }
                else
                {
                    CAPI2_PhoneCtrlApi_ProcessNoRfReq(InitClientInfo(pdata->ril_cmd->SimId));
                }                
            }            
			
			//luobiao@wind-mobi.com start 2012.05.17
			//patch for csp:524605 
		   	pdata->handler_state = BCM_SMS_Start_Stop_CBService;		
		   	//luobiao@wind-mobi.com end 2012.05.17
			            
			break;
        }

		//luobiao@wind-mobi.com start 2012.05.17
		//patch for csp:524605 
		case BCM_SMS_Start_Stop_CBService:
        {
            pdata->bcm_ril_rsp = kmalloc(pdata->ril_cmd->datalen, GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                int *OnOff = (int *)(pdata->ril_cmd->data);

                memcpy(pdata->bcm_ril_rsp, pdata->ril_cmd->data, pdata->ril_cmd->datalen);
                pdata->rsp_len = pdata->ril_cmd->datalen;
                if (*OnOff == 1)
                {                    
                    KRIL_SetFlightModeState(pdata->ril_cmd->SimId, FALSE);
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
                else if (*OnOff == 0)
                {
                    KRIL_SetFlightModeState(pdata->ril_cmd->SimId, TRUE);
#ifdef SEND_STOP_CB_SERVICE_IN_FLIGHT_MODE
                    if(g_bStopCBServiceAtFlightMode == TRUE) {
                        KRIL_DEBUG(DBG_ERROR, "SIMID:%d Stop CB Service in Flight Mode\n",pdata->ril_cmd->SimId);
                        CAPI2_SmsApi_StopReceivingCellBroadcastReq(InitClientInfo(pdata->ril_cmd->SimId));
                        pdata->handler_state = BCM_RESPCAPI2Cmd;
                    }
                    else
                    {                        
                        pdata->handler_state = BCM_FinishCAPI2Cmd;
                    }
#else
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
#endif
                }
            }            
            break;
        }
        case BCM_RESPCAPI2Cmd:
        {
            if(capi2_rsp->result != RESULT_OK )
            {
                KRIL_DEBUG(DBG_ERROR, "SIMID:%d Fail, error result:%d \n",pdata->ril_cmd->SimId,capi2_rsp->result);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }    
            
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }
		//luobiao@wind-mobi.com end 2012.05.17
        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;
        }
    }
}

#endif

void KRIL_SetTTYModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
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

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            int *mode = (int *)(pdata->ril_cmd->data);
            KRIL_DEBUG(DBG_INFO, "mode:%d\n", *mode);
            CAPI2_CcApi_SetTTYCall(InitClientInfo(pdata->ril_cmd->SimId), (Boolean) *mode);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            if(capi2_rsp->result != RESULT_OK)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
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


void KRIL_QueryTTYModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
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

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(krilQueryTTYModeType_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                pdata->rsp_len = sizeof(krilQueryTTYModeType_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_CcApi_IsTTYEnable(InitClientInfo(pdata->ril_cmd->SimId));
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {

            if(capi2_rsp->result != RESULT_OK)
            {
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                Boolean mode = *(Boolean *)capi2_rsp->dataBuf;
                krilQueryTTYModeType_t *rdata = (krilQueryTTYModeType_t *)pdata->bcm_ril_rsp;
                rdata->mode = (int)mode;
                KRIL_DEBUG(DBG_TRACE, "BCM_RESPCAPI2Cmd:: rdata->mode:%d mode:%d\n", rdata->mode, mode);
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


void KRIL_BasebandVersionHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
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
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
        if(capi2_rsp->result != RESULT_OK)
        {
            pdata->result = RILErrorResult(capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            return;
        }
    }

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(krilQueryBaseBandVersion_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {
                pdata->rsp_len = sizeof(krilQueryBaseBandVersion_t);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_SYSPARM_GetSWVersion(GetNewTID(), GetClientID());
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            UInt8 *version = (UInt8 *)capi2_rsp->dataBuf;
            krilQueryBaseBandVersion_t *rdata = (krilQueryBaseBandVersion_t *)pdata->bcm_ril_rsp;
            if(strlen(version)> sizeof(rdata->version))
            {
                KRIL_DEBUG(DBG_ERROR, "Version lenght %d is over buffer length %d\n", strlen(version),sizeof(krilQueryBaseBandVersion_t));
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }       
            strcpy(rdata->version, (char *)version);
            KRIL_DEBUG(DBG_INFO, "BCM_RESPCAPI2Cmd:: rdata->version:[%s] version:[%s]\n", (char *)rdata->version, (char *)version);
            pdata->handler_state = BCM_FinishCAPI2Cmd;
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

void KRIL_GetIMSIHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t*)ril_cmd;

    KRIL_DEBUG(DBG_INFO,"pdata->handler_state:0x%lX\n", pdata->handler_state);
    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            CAPI2_SimApi_GetIMSI(InitClientInfo(pdata->ril_cmd->SimId));
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            ParseIMSIData(pdata, capi2_rsp);
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


void KRIL_GetIMEIHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t*)ril_cmd;

    KRIL_DEBUG(DBG_INFO,"pdata->handler_state:0x%lX\n", pdata->handler_state);
    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_PHCTRL_ELEM_IMEI);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            ParseIMEIData(pdata, capi2_rsp);
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


void KRIL_GetIMEISVHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t*)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
    {
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
        if(capi2_rsp->result != RESULT_OK)
        {
            pdata->result = RILErrorResult(capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            return;
        }
    }

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_PHCTRL_ELEM_SW_VERSION);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            CAPI2_MS_Element_t *rsp = (CAPI2_MS_Element_t *) capi2_rsp->dataBuf;
            KrilImeiData_t *imeisv_result;
            pdata->bcm_ril_rsp = kmalloc(sizeof(KrilImeiData_t), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {    
                imeisv_result = (KrilImeiData_t *)pdata->bcm_ril_rsp;
                memset(imeisv_result, 0, sizeof(KrilImeiData_t));
                pdata->rsp_len = sizeof(KrilImeiData_t);
                strcpy(imeisv_result->imeisv, rsp->data_u.u3Bytes);
                KRIL_DEBUG(DBG_INFO, "u3Bytes:[%s] imeisv:[%s]\n", rsp->data_u.u3Bytes, imeisv_result->imeisv);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
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


void KRIL_GetDeviceIdentityHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t*)ril_cmd;

    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

    if (capi2_rsp != NULL)
    {
        KRIL_DEBUG(DBG_INFO, "handler_state:0x%lX::result:%d\n", pdata->handler_state, capi2_rsp->result);
        if(capi2_rsp->result != RESULT_OK)
        {
            pdata->result = RILErrorResult(capi2_rsp->result);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            return;
        }
    }

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
//sysparm only support SIM1 IMEI
            CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_PHCTRL_ELEM_IMEI);
            pdata->handler_state = BCM_GetIMEIInfo;
            break;
        }

        case BCM_GetIMEIInfo:
        {
            ParseIMEIData(pdata, capi2_rsp);
            CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_PHCTRL_ELEM_SW_VERSION);
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            CAPI2_MS_Element_t *rsp = (CAPI2_MS_Element_t *) capi2_rsp->dataBuf;
            KrilImeiData_t *imeisv_result = (KrilImeiData_t *)pdata->bcm_ril_rsp;
            strcpy(imeisv_result->imeisv, rsp->data_u.u3Bytes);
            KRIL_DEBUG(DBG_INFO, "u3Bytes:[%s] imeisv:[%s]\n", rsp->data_u.u3Bytes, imeisv_result->imeisv);
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

void KRIL_QuerySimEmergencyNumberHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t*)ril_cmd;

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
            return;
        }
    }

    switch (pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            pdata->bcm_ril_rsp = kmalloc(sizeof(Kril_SIMEmergency), GFP_KERNEL);
            if(!pdata->bcm_ril_rsp) {
                KRIL_DEBUG(DBG_ERROR, "unable to allocate bcm_ril_rsp buf\n");                
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
            else
            {    
                pdata->rsp_len = sizeof(Kril_SIMEmergency);
                memset(pdata->bcm_ril_rsp, 0, pdata->rsp_len);
                CAPI2_PbkApi_SendInfoReq(InitClientInfo(pdata->ril_cmd->SimId), PB_EN);
                pdata->handler_state = BCM_PBK_SendInfoReq;
            }
            break;
        }

        case BCM_PBK_SendInfoReq:
        {
            PBK_INFO_RSP_t *rsp = (PBK_INFO_RSP_t *) capi2_rsp->dataBuf;
            KRIL_DEBUG(DBG_INFO,"total_entries:[%d] result:[%d]\n", rsp->total_entries, rsp->result);
            if (0 == rsp->total_entries ||FALSE == rsp->result)
            {
                Kril_SIMEmergency *rdata = (Kril_SIMEmergency *)pdata->bcm_ril_rsp;
                rdata->simAppType = KRIL_GetSimAppType(pdata->ril_cmd->SimId);
                KRIL_SendNotify(pdata->ril_cmd->SimId, BRIL_UNSOL_EMERGENCY_NUMBER, pdata->bcm_ril_rsp, pdata->rsp_len);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            else
            {
                CAPI2_PbkApi_SendReadEntryReq(InitClientInfo(pdata->ril_cmd->SimId), PB_EN, 0, (rsp->total_entries-1));
                pdata->handler_state = BCM_PBK_ReadENEnteryReq;
            }
            break;
        }

        case BCM_PBK_ReadENEnteryReq:
        {
            PBK_ENTRY_DATA_RSP_t *rsp = (PBK_ENTRY_DATA_RSP_t *) capi2_rsp->dataBuf;
            Kril_SIMEmergency *rdata = (Kril_SIMEmergency *)pdata->bcm_ril_rsp;
            KRIL_DEBUG(DBG_INFO,"rsp->data_result:%d\n", rsp->data_result);

            if (rsp->data_result == PBK_ENTRY_VALID_IS_LAST || rsp->data_result == PBK_ENTRY_VALID_NOT_LAST)
            {
                KRIL_DEBUG(DBG_INFO,"simecclist:[%s] number:[%s] numlen:%d\n", rdata->simecclist, rsp->pbk_rec.number, strlen(rsp->pbk_rec.number));
                if (strlen(rsp->pbk_rec.number) != 0 && 
                    !(strcmp("112", rsp->pbk_rec.number) == 0 || strcmp("911", rsp->pbk_rec.number) == 0))
                {
                    if (strlen(rdata->simecclist) != 0)
                    {
                        sprintf(&rdata->simecclist[0], "%s%s%s", rdata->simecclist, ",", rsp->pbk_rec.number);
                    } 
                    else
                    {
                        strcpy(rdata->simecclist, rsp->pbk_rec.number);
                    }
                }
                if (rsp->data_result == PBK_ENTRY_VALID_IS_LAST)
                {
                    rdata->simAppType = KRIL_GetSimAppType(pdata->ril_cmd->SimId);
                    KRIL_SendNotify(pdata->ril_cmd->SimId, BRIL_UNSOL_EMERGENCY_NUMBER, pdata->bcm_ril_rsp, pdata->rsp_len);
                    pdata->handler_state = BCM_FinishCAPI2Cmd;
                }
            }
            else
            {
                rdata->simAppType = KRIL_GetSimAppType(pdata->ril_cmd->SimId);
                KRIL_SendNotify(pdata->ril_cmd->SimId, BRIL_UNSOL_EMERGENCY_NUMBER, pdata->bcm_ril_rsp, pdata->rsp_len);
                pdata->handler_state = BCM_FinishCAPI2Cmd;
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

void KRIL_ConfigEmergencyIdleMode(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = NULL;

    if (!ril_cmd)
    {
		    KRIL_DEBUG(DBG_ERROR,"ril_cmd is NULL. Error!!!\n");
        return;
   	}
    pdata = (KRIL_CmdList_t*)ril_cmd;
	
    if (!pdata->ril_cmd)
    {
        KRIL_DEBUG(DBG_ERROR,"pdata->ril_cmd is NULL. Error!!!\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
   	}
	
    KRIL_DEBUG(DBG_INFO,"pdata->handler_state:0x%lX\n", pdata->handler_state);
	
    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }	
	
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
            CAPI2_MsDbApi_GetElement(InitClientInfo(pdata->ril_cmd->SimId), MS_LOCAL_PHCTRL_ELEM_EMERGENCY_IDLE_MODE);
            pdata->handler_state = BCM_Set_Emergency_Idle_Mode;
        }
        break;
        case BCM_Set_Emergency_Idle_Mode:
        {
            //Bytes 0~3="BRCM";Byte 4:Message ID;Byte 5: Flag
            UInt8 *cmd_data = (UInt8*)pdata->ril_cmd->data;
            CAPI2_MS_Element_t *rsp = (CAPI2_MS_Element_t *) capi2_rsp->dataBuf;
            Boolean Flag = FALSE;

            if (!rsp)
            {
                KRIL_DEBUG(DBG_ERROR,"capi2_rsp->dataBuf is NULL, Error!!\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
			
            Flag = (Boolean)cmd_data[5];
			
            KRIL_DEBUG(DBG_INFO,"Emergency Idle mode:%d, Set to:%d \n", rsp->data_u.bData, Flag);

            if(rsp->data_u.bData != Flag)
            {
                CAPI2_MS_Element_t data; 
                memset((UInt8*)&data, 0, sizeof(CAPI2_MS_Element_t));
                data.inElemType = MS_LOCAL_PHCTRL_ELEM_EMERGENCY_IDLE_MODE;
                data.data_u.bData = Flag;
                CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
				
                pdata->handler_state = BCM_RESPCAPI2Cmd;
                if((Flag == FALSE)&&(!KRIL_GetFlightModeState(pdata->ril_cmd->SimId)))
                {
                    pdata->handler_state = BCM_TurnOffRadio;					
                }
            }
            else
                pdata->handler_state = BCM_FinishCAPI2Cmd;
        }
        break;
        case BCM_TurnOffRadio:
        { 
            CAPI2_PhoneCtrlApi_ProcessNoRfReq(InitClientInfo(pdata->ril_cmd->SimId));
            KRIL_SetConfigEmergencyIdleModeTID(GetTID());
            pdata->handler_state = BCM_Wait_GSM_NO_Service;	
        }
        break;
        case BCM_Wait_GSM_NO_Service:
        {
            KRIL_DEBUG(DBG_INFO,"msgtype:0x%x!\n", capi2_rsp->msgType); 
            if (MSG_REG_GSM_IND == capi2_rsp->msgType)
            {
                MSRegInfo_t *pMSRegInfo = (MSRegInfo_t*)capi2_rsp->dataBuf;
                KRIL_DEBUG(DBG_INFO, "regState:%d \n", pMSRegInfo->regState);
                if(REG_STATE_NO_SERVICE == pMSRegInfo->regState)
                {
                    KRIL_SetConfigEmergencyIdleModeTID(0);
                    KRIL_DEBUG(DBG_INFO, "Power on:%d \n");
                    CAPI2_PhoneCtrlApi_ProcessPowerUpReq(InitClientInfo(pdata->ril_cmd->SimId));
                    pdata->handler_state = BCM_RESPCAPI2Cmd;
                }				               	
                ProcessGSMStatus(capi2_rsp);				               	
            }		
        }
        break;		
        case BCM_RESPCAPI2Cmd:
        {
            pdata->handler_state = BCM_FinishCAPI2Cmd;
        }
        break;
        default:
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
        break;		  
    }	 
}

void KRIL_GetCurrentSimVoltageHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t*)ril_cmd;

    KRIL_DEBUG(DBG_INFO,"pdata->handler_state:0x%lX\n", pdata->handler_state);
    
    if((BCM_SendCAPI2Cmd != pdata->handler_state)&&(NULL == capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }

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
            CAPI2_SimApi_GetCurrentSimVoltage(InitClientInfo(pdata->ril_cmd->SimId));
            pdata->handler_state = BCM_RESPCAPI2Cmd;
            break;
        }

        case BCM_RESPCAPI2Cmd:
        {
            UInt8 *resp = NULL;
            SimVoltage_t *voltage = (SimVoltage_t*)capi2_rsp->dataBuf;
            
            if (!voltage)
            {
                KRIL_DEBUG(DBG_ERROR,"capi2_rsp->dataBuf is NULL, Error!!\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
            
            pdata->bcm_ril_rsp = kmalloc(sizeof(UInt8)*6, GFP_KERNEL);
            if (!pdata->bcm_ril_rsp)
            {
                KRIL_DEBUG(DBG_ERROR,"Allocate bcm_ril_rsp memory failed!!\n");
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
                return;
            }
            
            pdata->rsp_len = sizeof(UInt8)*6 ;
            
            resp = (UInt8*)pdata->bcm_ril_rsp;
            resp[0] = (UInt8)'B';
            resp[1] = (UInt8)'R';
            resp[2] = (UInt8)'C';
            resp[3] = (UInt8)'M';
            resp[4] = (UInt8)BRIL_HOOK_GET_SIM_VOLTAGE;
            resp[5] = (UInt8)*voltage;
            
            pdata->handler_state = BCM_FinishCAPI2Cmd;
            break;
        }
        
        default:
            KRIL_DEBUG(DBG_ERROR,"Error handler_state:0x%lX\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            break;                
    }    
}


void KRIL_SetPdpActivationFdnControlHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    KRIL_DEBUG(DBG_INFO,"pdata->handler_state:0x%lX\n", pdata->handler_state);
    
    if((BCM_SendCAPI2Cmd!=pdata->handler_state) && (NULL==capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    } 
	
    if (capi2_rsp && capi2_rsp->result!=RESULT_OK)
    {
        KRIL_DEBUG(DBG_ERROR,"CAPI2 response failed:%d\n", capi2_rsp->result);
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }    
    
    switch(pdata->handler_state)
    {
        case BCM_SendCAPI2Cmd:
        {
            CAPI2_MS_Element_t data;
            UInt8 *tdata = (UInt8 *)pdata->ril_cmd->data;
            KRIL_DEBUG(DBG_INFO, "status:%d\n", tdata[5]);
            if (0 == tdata[5] || 1 == tdata[5])
            {
                memset((UInt8*)&data, 0x00, sizeof(CAPI2_MS_Element_t));
                data.inElemType = MS_LOCAL_PCH_ELEM_PDP_ACTIVATION_FDN_CONTROL;
                data.data_u.u8Data = (UInt8)tdata[5];
				
                CAPI2_MsDbApi_SetElement(InitClientInfo(pdata->ril_cmd->SimId), &data);
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            else
            {
                KRIL_DEBUG(DBG_ERROR, "not support status:%d\n", tdata[5]);
                pdata->handler_state = BCM_ErrorCAPI2Cmd;
            }
        }
        break;

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
                //Bytes 0~3="BRCM";Byte4:Message
                resp = (UInt8*)pdata->bcm_ril_rsp;
                resp[0] = (UInt8)'B';
                resp[1] = (UInt8)'R';
                resp[2] = (UInt8)'C';
                resp[3] = (UInt8)'M';
                resp[4] = (UInt8)BRIL_HOOK_SET_PDP_ACTIVATION_FDN_CONTROL;
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

void MeasureReportPage1(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];
    
    sprintf(tempbuf,"BCCH Information:%d\n",rsp->gsm_param.ci);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Rx Power Level in dBm:%d\n",rsp->gsm_param.rxlev);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"TX Power Level in dBm:%d\n",rsp->gsm_param.txpwr);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Time Slot:%d\n",rsp->gsm_param.timeslot_assigned);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Timing Advance:%d\n",rsp->gsm_param.timing_advance);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    //- 0 DTX was not used
	//- 1 DTX was used.
    if(rsp->gsm_param.dtx_used == 1) {
        sprintf(tempbuf,"RX Quality:%d\n",rsp->gsm_param.rxqualsub);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }
    else
    {
        sprintf(tempbuf,"RX Quality:%d\n",rsp->gsm_param.rxqualfull);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    sprintf(tempbuf,"Radio Link Timeout Value:%d\n",rsp->gsm_param.radio_link_timeout);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Current Channel Type:%d\n",rsp->gsm_param.chan_type);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Paging Information:%d\n",rsp->gsm_param.bs_pa_mfrms);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"IMSI Attach:%d\n",rsp->mm_param.gmm_attach_type);    
    strcat(QueryMeasureReportBuf+5,tempbuf);
}

void MeasureReportPage2(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];
    UInt8 i;

    //UMTS 
    if(1 == rsp->mm_param.rat){        
        sprintf(tempbuf,"MCC:%d\n",rsp->umts_param.plmn_id.mcc);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"MNC:%d\n",rsp->umts_param.plmn_id.mnc);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"LAC:%d\n",rsp->umts_param.lac);
        strcat(QueryMeasureReportBuf+5,tempbuf);
		sprintf(tempbuf,"Downlink UARFCN:%d\n",rsp->umts_param.dl_uarfcn);
        strcat(QueryMeasureReportBuf+5,tempbuf);
		sprintf(tempbuf,"Uplink UARFCN:%d\n",rsp->umts_param.ul_uarfcn);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"Cell Id:%d\n",rsp->umts_param.cell_id);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }
    else//GSM
    if(0 == rsp->mm_param.rat)
    {
        sprintf(tempbuf,"MCC:%d\n",rsp->gsm_param.mcc);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"MNC:%d\n",rsp->gsm_param.mnc);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"LAC:%d\n",rsp->gsm_param.lac);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"ARFCN of Serving Cell:%d\n",rsp->gsm_param.arfcn);
        strcat(QueryMeasureReportBuf+5,tempbuf);
        sprintf(tempbuf,"Cell Id:%d\n",rsp->gsm_param.ci);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }    
    
    sprintf(tempbuf,"Bsic:%d\n",rsp->gsm_param.bsic);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Network Control Order:%d\n",rsp->gsm_param.nco);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Network Operation Mode:%d\n",rsp->mm_param.nom);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    strcpy(tempbuf,"Support for BTS Test:Parmeter Not Found!\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Ciphering Status:%d\n",rsp->gsm_param.cipher_on);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    strcpy(tempbuf,"** Frequency Hopping **\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Assigned or Not:%d\n",rsp->gsm_param.hopping_status);    
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"The Hopping Sequence Number:%d\n",rsp->gsm_param.hsn);    
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Mobile Allocation Index Offset:%d\n",rsp->gsm_param.maio);    
    strcat(QueryMeasureReportBuf+5,tempbuf);

    if(rsp->gsm_param.ma.rf_chan_cnt == 0) {
        strcpy(tempbuf,"Mobile Allocation Is Not Present!\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }
    else
    {
        strcpy(tempbuf,"Mobile Allocation of Radio Frequency Channels:\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
        for(i=0; i<rsp->gsm_param.ma.rf_chan_cnt; i++) {
            sprintf(tempbuf,"%d\n",rsp->gsm_param.ma.rf_chan_array[i]);    
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    }

}

void MeasureReportPage3(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;

    strcpy(QueryMeasureReportBuf+5, "RIL does not handle Battery page\n");    
}

void MeasureReportPage4(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;    
    char tempbuf[100];

    sprintf(tempbuf,"Radio Link Failure:%d\n",rsp->umts_param.rrc_counters.radio_link_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"UMTS GSM Handover:%d\n",rsp->umts_param.rrc_counters.umts_gsm_handover);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"RLC Unrecoverable:%d\n",rsp->umts_param.rrc_counters.rlc_unrecoverable_error);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"NAS Triggered Release:%d\n",rsp->umts_param.rrc_counters.nas_triggered_release);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"Normal Release:%d\n",rsp->umts_param.rrc_counters.normal_release);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"Configuration Failure:%d\n",rsp->umts_param.rrc_counters.configuration_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"N300 Failure:%d\n",rsp->umts_param.rrc_counters.n300_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"T314 T315 Failure:%d\n",rsp->umts_param.rrc_counters.t314_t315_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"N302 Failure:%d\n",rsp->umts_param.rrc_counters.n302_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"T316 T317 T307 Failure:%d\n",rsp->umts_param.rrc_counters.t316_t317_t307_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"Other Failure:%d\n",rsp->umts_param.rrc_counters.other_failure);
    strcat(QueryMeasureReportBuf+5,tempbuf);    
}

void MeasureReportPage5(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;

     //UMTS 
    if(1 == rsp->mm_param.rat){

        if(rsp->umts_param.chn_rel_cause == 0) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Protocol Error -- Unspecified!\n");            
        }
        else
        if(rsp->umts_param.chn_rel_cause == 17) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Normal Release!\n");            
        }
        else
        if(rsp->umts_param.chn_rel_cause == 19) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Pre-Emptive Release!\n");
        }
        else
        if(rsp->umts_param.chn_rel_cause == 20) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Congestion!\n");
        }
        else
        if(rsp->umts_param.chn_rel_cause == 21) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Re-Establishment Reject!\n");
        }
        else
        if(rsp->umts_param.chn_rel_cause == 22) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Directed Signaling Connection Re-Establishment!\n");
        }
        else
        if(rsp->umts_param.chn_rel_cause == 23) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:User Inactivity!\n");
        }
        else
        {
            sprintf(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Mapping is wrong! ID:%d\n",rsp->umts_param.chn_rel_cause);
        }

    }
    else //GSM 
    if(0 == rsp->mm_param.rat){
        if(rsp->gsm_param.chn_rel_cause == 0) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Protocol Error -- Unspecified!\n");
        }
        else
        if(rsp->gsm_param.chn_rel_cause == 13) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Abnormal release!\n");
        }
        else
        if(rsp->gsm_param.chn_rel_cause == 19) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Pre-Emptive Release!\n");
        }
        else
        if(rsp->gsm_param.chn_rel_cause == 38) {
            strcpy(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Normal Release\n");
        }
        else
        {
            sprintf(QueryMeasureReportBuf+5, "Radio Resource Failure Cause Code:Mapping is wrong! ID:%d\n",rsp->gsm_param.chn_rel_cause);
        }
    }    
}

void CheckPsState(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;    

    switch(rsp->mm_param.ps_state) {
            case 0:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Unknow State!\n");            
            break;
            case 1:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:No Network Available!\n");            
            break;
            case 2:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Search For Network!\n");            
            break;
            case 3:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Emergency Calls Only!\n");            
            break;
            case 4:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Limited Service!\n");            
            break;
            case 5:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Full Service!\n");            
            break;
            case 6:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Plmn List Available!\n");            
            break;
            case 7:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Disabled State!\n");            
            break;
            case 8:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Detached State!\n");            
            break;
            case 9:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:No GPRS Cell!\n");            
            break;
            case 10:
            strcpy(QueryMeasureReportBuf+5,"GPRS State:Suspended State!\n");            
            break;
            default:
            strcpy(QueryMeasureReportBuf+5,"Fail! No Supported State!\n");            
            break;
        }
}

void MeasureReportPage6(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[100];
    UInt8 tempValue , i, NumOfSlots;

    CheckPsState(capi2_rsp);

    strcat(QueryMeasureReportBuf+5, "PDP State:\n");
    sprintf(tempbuf,"Ms Initated PDP Context Activation Attempt:%d\n",rsp->ext_param.sm_ext_param.mo_pdp_attempt_cnt);    
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Radio Priority of PDP Context:%d\n",rsp->ext_param.sm_ext_param.pdp_priority);    
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Sec Radio Priority of PDP Context:%d\n",rsp->ext_param.sm_ext_param.sec_pdp_priority);    
    strcat(QueryMeasureReportBuf+5,tempbuf);

    NumOfSlots = 0; 
    tempValue = rsp->gsm_param.gprs_packet_param.dl_timeslot_assigned;    
    
    for(i=0;i<8;i++){
        if((tempValue & 0x1)) {
            NumOfSlots++;
        }
        tempValue = tempValue>>1;
    }

    tempValue = rsp->gsm_param.gprs_packet_param.ul_timeslot_assigned;    
    
    for(i=0;i<8;i++){
        if((tempValue & 0x1)) {
            NumOfSlots++;
        }
        tempValue = tempValue>>1;
    }

    sprintf(tempbuf,"Number of Slots Supported:%d\n",NumOfSlots);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    strcpy(tempbuf,"DL Coding scheme:\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    for(i=0;i<8;i++){
        sprintf(tempbuf,"Slot%d, CS(%d)\n",i,rsp->gsm_param.gprs_packet_param.dl_cs_mode_per_ts[i]);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    strcpy(tempbuf,"UL Coding scheme:\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    for(i=0;i<8;i++){
        sprintf(tempbuf,"Slot%d, CS(%d)\n",i,rsp->gsm_param.gprs_packet_param.ul_cs_mode_per_ts[i]);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    strcpy(tempbuf,"** Timers (RAU) **\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"T3212:%d\n",rsp->gsm_param.t3212);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"T3312:%d\n",rsp->ext_param.mm_ext_param.t3312_to_value);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    strcpy(tempbuf,"Average Throughput: Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    strcpy(tempbuf,"BLER Rate: Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
}

void MeasureReportPage7(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;

    strcpy(QueryMeasureReportBuf+5, "Excel does not have this PageID\n");    
}

void MeasureReportPage8(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;    
    char tempbuf[50];
    UInt8 i;

    sprintf(tempbuf,"Srxlev:%d\n", rsp->umts_param.Srxlev);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    sprintf(tempbuf,"Squal:%d\n", rsp->umts_param.Squal);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    sprintf(tempbuf,"DRX Cycle:%d\n", rsp->umts_param.drx_cycle_length);    
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    sprintf(tempbuf,"Rs:%d\n", rsp->umts_param.ranking_value);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    for(i=0;i<rsp->umts_param.no_umts_ncells;i++) {
        //5 means UMTS_RANKED
        if(rsp->umts_param.umts_ncell.A[i].cell_type == 5) {
            sprintf(tempbuf, "Rn[%d](UMTS):%d\n", i, rsp->umts_param.umts_ncell.A[i].ranking_value);
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    }

    for(i=0;i<rsp->umts_param.no_gsm_ncells;i++) {
        sprintf(tempbuf, "Rn[%d](GSM):%d\n", i, rsp->umts_param.gsm_ncell.A[i].ranking_value);
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    sprintf(tempbuf,"Counter for UMTS to GSM Reselection:%d\n",rsp->umts_param.rrc_counters.umts_gsm_reselect_success);
    strcat(QueryMeasureReportBuf+5,tempbuf);

    sprintf(tempbuf,"Counter for GSM to UMTS Reselection:%d\n",rsp->umts_param.rrc_counters.gsm_umts_reselect_success);
    strcat(QueryMeasureReportBuf+5,tempbuf);

}

void MeasureReportPage9(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[100];
    UInt8 i, j;    
    
    strcpy(tempbuf,"** RAB ID of the Assigned Bearer **\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);    

    if(rsp->umts_param.rab_rb_info.no_rabs != 0) {
        for(i=0;i<rsp->umts_param.rab_rb_info.no_rabs;i++) {
            sprintf(tempbuf,"rab_id:%d, domain:%d\n",rsp->umts_param.rab_rb_info.per_rab_info[i].rab_id, rsp->umts_param.rab_rb_info.per_rab_info[i].domain);
            strcat(QueryMeasureReportBuf+5,tempbuf);    
        }
    }
    else
    {
        strcpy(tempbuf,"No RAB_ID & Domain\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    strcpy(tempbuf,"Data Rate of the RAB depending on its Class:Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    strcpy(tempbuf,"** Signalling Radio Bearer Id **\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    if(rsp->umts_param.rab_rb_info.no_srbs != 0) {
        for(i=0;i<rsp->umts_param.rab_rb_info.no_srbs;i++) {
            sprintf(tempbuf,"** SRB ID:%d **\n",rsp->umts_param.rab_rb_info.srb_id[i]);
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    }
    else
    {
        strcpy(tempbuf,"No SRB_ID\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    strcpy(tempbuf,"Traffic Class:Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    strcpy(tempbuf,"Max Bit Rate:Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);    
    strcpy(tempbuf,"Residual BERatio:Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);    
    strcpy(tempbuf,"Guaranteed Bit Rate:Not Support\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);    
}

void MeasureReportPage10(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];

    sprintf(tempbuf,"Current RRC State:%d\n",rsp->umts_param.rrc_state);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"RRC Connection Rejection Cause:%d\n",rsp->umts_param.chn_rel_cause);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"RRC Connection Release Cause:%d\n",rsp->umts_param.chn_rel_cause);
    strcat(QueryMeasureReportBuf+5,tempbuf);
}

void MeasureReportPage11(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];

    sprintf(tempbuf,"UE TX Power:%d\n",rsp->umts_param.tx_pwr);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"TX Power Control Algorithm:%d\n",rsp->umts_param.l1_info.power_control_algorithm);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"TX Power Control Step Size:%d\n",rsp->umts_param.l1_info.power_control_step_size);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"SIR:%d\n",rsp->umts_param.dch_report.meas_sir);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"BLER:%d\n",rsp->umts_param.dch_report.meas_bler);
    strcat(QueryMeasureReportBuf+5,tempbuf);    
    sprintf(tempbuf,"UTRA Carrier RSSI::%d\n",rsp->umts_param.rssi);
    strcat(QueryMeasureReportBuf+5,tempbuf);
}

void MeasureReportPage12(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    UInt8 i, Cell_in_Active_Set_Count=0, Cell_in_Monitor_Set_Count=0;
    char tempbuf[100];
    
    sprintf(tempbuf,"*** Following Cell Parameters in The Active Set ***\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    for(i=0;i<rsp->umts_param.no_umts_ncells;i++) {
            //0 means ACTIVE_SET
            if(rsp->umts_param.umts_ncell.A[i].cell_type == 0) {
                sprintf(tempbuf,"** Cell Number:%d **\n",i);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"DL UARFCN:%d\n",rsp->umts_param.umts_ncell.A[i].dl_uarfcn);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"Primary Scrambling Code:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_sc);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"RSCP:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_rscp);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"EC2N0:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_ecn0);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                Cell_in_Active_Set_Count++;
            }
    }


    if(Cell_in_Active_Set_Count == 0) 
    {
        strcpy(tempbuf,"No Cell Parameters in The Active Set\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    sprintf(tempbuf,"*** Cell Parameters in The Monitored Set for Best 6 Cells ***\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    
    for(i=0;i<rsp->umts_param.no_umts_ncells;i++) {
        //2 means MONITORED
        //since it's intra frequency, so dl_uarfcn should be the same as serving cell
        if((rsp->umts_param.umts_ncell.A[i].cell_type == 2) &&
           (rsp->umts_param.umts_ncell.A[i].dl_uarfcn == rsp->umts_param.dl_uarfcn)) {
            sprintf(tempbuf,"** Cell Number:%d **\n",i);
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"DL UARFCN:%d\n",rsp->umts_param.umts_ncell.A[i].dl_uarfcn);
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"Primary Scrambling Code:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_sc);
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"RSCP:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_rscp);
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"EC2N0:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_ecn0);
            strcat(QueryMeasureReportBuf+5,tempbuf);
            Cell_in_Monitor_Set_Count++;
        }
    }

    if(Cell_in_Monitor_Set_Count == 0) 
    {
        strcpy(tempbuf,"No Cell Parameters in The Monitored Set\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

}

void MeasureReportPage13(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    UInt8 i, Cell_Count=0;
    char tempbuf[100];

    for(i=0;i<rsp->umts_param.no_umts_ncells;i++) {
            if(rsp->umts_param.dl_uarfcn != rsp->umts_param.umts_ncell.A[i].dl_uarfcn) {
                sprintf(tempbuf,"** Cell Number:%d **\n",i);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"DL UARFCN:%d\n",rsp->umts_param.umts_ncell.A[i].dl_uarfcn);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"Primary Scrambling Code:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_sc);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"RSCP:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_rscp);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                sprintf(tempbuf,"EC2N0:%d\n",rsp->umts_param.umts_ncell.A[i].cpich_ecn0);
                strcat(QueryMeasureReportBuf+5,tempbuf);
                Cell_Count++;
            }
    }

    if(Cell_Count == 0) {
        strcpy(tempbuf,"No Cell Parameters\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

}

#define COMPRESSED_MODE_TDD               0x01
#define COMPRESSED_MODE_FDD               0x02
#define COMPRESSED_MODE_GSM_RSSI_MEAS     0x04
#define COMPRESSED_MODE_GSM_BSIC_IDENT    0x08
#define COMPRESSED_MODE_GSM_BSIC_RECONFIG 0x10
#define COMPRESSED_MODE_EUTRA_MEAS        0x20

void MeasureReportPage14(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];
    UInt8 ranking_val_id;
    UInt8 i,show_gsm_ncells;

    if(rsp->umts_param.no_gsm_ncells > 0) {
        if(rsp->umts_param.no_gsm_ncells > MAX_NUMBER_OF_GSM_NCELLS) {
            show_gsm_ncells = MAX_NUMBER_OF_GSM_NCELLS;
            KRIL_DEBUG(DBG_ERROR,"no_gsm_ncells %d > MAX_NUMBER_OF_GSM_NCELLS\n", rsp->umts_param.no_gsm_ncells);
        }
        else
        {
            show_gsm_ncells = rsp->umts_param.no_gsm_ncells;
            KRIL_DEBUG(DBG_ERROR,"no_gsm_ncells %d <= MAX_NUMBER_OF_GSM_NCELLS\n", rsp->umts_param.no_gsm_ncells);
        }

        for(i=0;i<show_gsm_ncells;i++) {
            sprintf(tempbuf,"Cell[%d]\n", i);
            strcat(QueryMeasureReportBuf+5,tempbuf);
            strcpy(tempbuf,"Cellular System:Not Support!\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"Channel Number::%d\n",rsp->umts_param.gsm_ncell.A[i].arfcn);    
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"BSIC:%d\n",rsp->umts_param.gsm_ncell.A[i].bsic);        
            strcat(QueryMeasureReportBuf+5,tempbuf);
            sprintf(tempbuf,"Cell Id:%d\n",rsp->umts_param.gsm_ncell.A[i].ci);            
            strcat(QueryMeasureReportBuf+5,tempbuf);    
            sprintf(tempbuf,"GSM Carrier RSSI:%d\n",rsp->umts_param.gsm_ncell.A[i].rxlev);                
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    }
    else
    {
        strcpy(tempbuf,"No Cell Parameters\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    strcpy(tempbuf,"Compressed Mode Active or Not:\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);

    if(rsp->umts_param.tgmp_status == 0)
    {
        strcpy(tempbuf,"Not Active\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }
    else
    {    
        if(rsp->umts_param.tgmp_status & COMPRESSED_MODE_TDD) {
            strcpy(tempbuf,"TDD\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    
        if(rsp->umts_param.tgmp_status & COMPRESSED_MODE_FDD) {
            strcpy(tempbuf,"FDD\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    
        if(rsp->umts_param.tgmp_status & COMPRESSED_MODE_GSM_RSSI_MEAS) {
            strcpy(tempbuf,"GSM RSSI MEAS\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    
        if(rsp->umts_param.tgmp_status & COMPRESSED_MODE_GSM_BSIC_IDENT) {
            strcpy(tempbuf,"GSM BSIC IDENT\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    
        if(rsp->umts_param.tgmp_status & COMPRESSED_MODE_GSM_BSIC_RECONFIG) {
            strcpy(tempbuf,"GSM BSIC RECONFIG\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }
    
        if(rsp->umts_param.tgmp_status & COMPRESSED_MODE_EUTRA_MEAS) {
            strcpy(tempbuf,"EUTRA MEAS\n");
            strcat(QueryMeasureReportBuf+5,tempbuf);
        }

    }

}
void MeasureReportPage15(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];
    sprintf(tempbuf,"Number of Assigned HS-SCCH Codes:%d\n",rsp->umts_param.hspa_config.no_hsscch_codes);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Data Rate:%d\n",rsp->umts_param.hsdpa_l1_l2_info.data_rate);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"CQI:%d\n",rsp->umts_param.hsdpa_l1_l2_info.cqi);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"BLER:%d\n",rsp->umts_param.hsdpa_l1_l2_info.bler);
    strcat(QueryMeasureReportBuf+5,tempbuf);
}
void MeasureReportPage16(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];
    sprintf(tempbuf,"Power of First Preamble:%d\n",rsp->umts_param.rach_info.initial_tx_pwr);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Number of Preambles:%d\n",rsp->umts_param.rach_info.preamble_transmission_count);
    strcat(QueryMeasureReportBuf+5,tempbuf);    
    sprintf(tempbuf,"AICH Status:%d\n",rsp->umts_param.rach_info.msg_transmission_result);
    strcat(QueryMeasureReportBuf+5,tempbuf);
    sprintf(tempbuf,"Power Message Was Transmitted at:%d\n",rsp->umts_param.rach_info.msg_power);
    strcat(QueryMeasureReportBuf+5,tempbuf);
}
void MeasureReportPage17(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    strcpy(QueryMeasureReportBuf,"CPC Parameters:Not Support!\n");    
}
void MeasureReportPage18(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    strcpy(QueryMeasureReportBuf+5,"Option to Turn Fast Dormancy On/Off:Not Support!\n");    
}
void MeasureReportPage19(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    strcpy(QueryMeasureReportBuf+5,"Option to Turn Receive Diversity On/Off:Not Support!\n");    
}

void MeasureReportPage20(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[50];
    strcpy(tempbuf,"CTCH schedule period:Not Support!\n");    
    strcat(QueryMeasureReportBuf+5, tempbuf);
    strcpy(tempbuf,"CTCH Length:Not Support!\n");    
    strcat(QueryMeasureReportBuf+5, tempbuf);
}

void MeasureReportPage21(Kril_CAPI2Info_t *capi2_rsp)
{
    MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
    char tempbuf[100];
    //0 for NB-AMR , 1 for WB-AMR and 2 for Invalid AMR Mode
    if(rsp->umts_param.amr_info.umts_amr_codec_mode == 0) {
        strcpy(tempbuf,"Type of AMR Codec: NB-AMR\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    }
    else
    if(rsp->umts_param.amr_info.umts_amr_codec_mode == 1) {
        strcpy(tempbuf,"Type of AMR Codec: WB-AMR\n");
        strcat(QueryMeasureReportBuf+5,tempbuf);
    }
    else
    if(rsp->umts_param.amr_info.umts_amr_codec_mode == 2) {
        strcpy(tempbuf,"Type of AMR Codec: Invalid\n");
    strcat(QueryMeasureReportBuf+5,tempbuf);
    }

    sprintf(tempbuf,"Upgrade and Downgrade of the AMR Codec: %d.%d Kbps\n",rsp->umts_param.amr_info.frame_rate_whole,rsp->umts_param.amr_info.umts_amr_codec_mode);    
    strcat(QueryMeasureReportBuf+5, tempbuf);
    strcpy(tempbuf,"Option to Turn WB-AMR On/Off If Supported:Not Support!\n");    
    strcat(QueryMeasureReportBuf+5, tempbuf);
}

void CombineMeasureReport(UInt32 PageId, Kril_CAPI2Info_t *capi2_rsp)
{    
    QueryMeasureReportBuf[0]='B';
    QueryMeasureReportBuf[1]='R';
    QueryMeasureReportBuf[2]='C';
    QueryMeasureReportBuf[3]='M';    
    QueryMeasureReportBuf[4]=BRIL_HOOK_QUERY_ENG_MODE;    

    switch(PageId) {
        case 1:
        MeasureReportPage1(capi2_rsp);
        break;
        case 2:
        MeasureReportPage2(capi2_rsp);
        break;
        case 3:
        MeasureReportPage3(capi2_rsp);
        break;
        case 4:
        MeasureReportPage4(capi2_rsp);
        break;
        case 5:
        MeasureReportPage5(capi2_rsp);
        break;
        case 6:
        MeasureReportPage6(capi2_rsp);
        break;
        case 7:
        MeasureReportPage7(capi2_rsp);
        break;
        case 8:
        MeasureReportPage8(capi2_rsp);
        break;
        case 9:
        MeasureReportPage9(capi2_rsp);
        break;
        case 10:
        MeasureReportPage10(capi2_rsp);
        break;
        case 11:
        MeasureReportPage11(capi2_rsp);
        break;
        case 12:
        MeasureReportPage12(capi2_rsp);
        break;
        case 13:
        MeasureReportPage13(capi2_rsp);
        break;
        case 14:
        MeasureReportPage14(capi2_rsp);
        break;
        case 15:
        MeasureReportPage15(capi2_rsp);
        break;
        case 16:
        MeasureReportPage16(capi2_rsp);
        break;
        case 17:
        MeasureReportPage17(capi2_rsp);
        break;
        case 18:
        MeasureReportPage18(capi2_rsp);
        break;
        case 19:
        MeasureReportPage19(capi2_rsp);
        break;
        case 20:
        MeasureReportPage20(capi2_rsp);
        break;
        case 21:
        MeasureReportPage21(capi2_rsp);
        break;
        default:
        sprintf(QueryMeasureReportBuf,"This PageID(%d) does not support\n", PageId);
        break;
    }
}

void KRIL_QueryMeasureReportHandler(void *ril_cmd,  Kril_CAPI2Info_t *capi2_rsp)
{
    KRIL_CmdList_t *pdata = (KRIL_CmdList_t *)ril_cmd;
    UInt32 TimeInterval = 0;
    char* rawdata = NULL;
    

    KRIL_DEBUG(DBG_INFO,"pdata->handler_state:0x%lX\n", pdata->handler_state);

    if((BCM_SendCAPI2Cmd!=pdata->handler_state) && (NULL==capi2_rsp))
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp is NULL\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        KRIL_SetInQueryMeasureReportHandler( FALSE );
        return;
    } 

    if (capi2_rsp && capi2_rsp->result!=RESULT_OK)
    {
        KRIL_DEBUG(DBG_ERROR,"CAPI2 response failed:%d\n", capi2_rsp->result);
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        KRIL_SetInQueryMeasureReportHandler( FALSE );
        return;
    }    

    switch(pdata->handler_state)
    {
		case BCM_SendCAPI2Cmd:
		{
            KRIL_SetInQueryMeasureReportHandler( TRUE );
            KRIL_DEBUG(DBG_INFO,"SimID:%d\n", pdata->ril_cmd->SimId);           			
			CAPI2_DiagApi_MeasurmentReportReq ( InitClientInfo(pdata->ril_cmd->SimId),TRUE,TimeInterval);
            KRIL_SetQueryMeasureReportTID(GetTID());
			pdata->handler_state = BCM_QueryMeasureReport;			
		}
		break;
        
		case BCM_QueryMeasureReport:
		{
            
            if (MSG_MEASURE_REPORT_PARAM_IND == capi2_rsp->msgType)
            {   
                MS_RxTestParam_t *rsp = (MS_RxTestParam_t *)capi2_rsp->dataBuf;
                rawdata = (char*)pdata->ril_cmd->data;                
                if ((1 == rsp->mm_param.rat) || (0 == rsp->mm_param.rat))
                {
                    memset(QueryMeasureReportBuf, 0, sizeof(char)*QueryMeasureReportLen);
                    CombineMeasureReport(rawdata[6], capi2_rsp);
                }
                else
                {
                    pdata->result = BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW;
                }

                // get measurement data
                CAPI2_DiagApi_MeasurmentReportReq ( InitClientInfo(capi2_rsp->SimId),FALSE,0);                
                pdata->handler_state = BCM_RESPCAPI2Cmd;
            }
            else
            {
                KRIL_SetQueryMeasureReportTID(capi2_rsp->tid);
            }			
        }
        break;
		case BCM_RESPCAPI2Cmd:
		{
   			if (BCM_E_OP_NOT_ALLOWED_BEFORE_REG_TO_NW == pdata->result)
			{
				pdata->handler_state = BCM_ErrorCAPI2Cmd;
			}
			else
			{   
                pdata->bcm_ril_rsp = kmalloc(sizeof(char)*QueryMeasureReportLen, GFP_KERNEL);
                if (!pdata->bcm_ril_rsp)
                {
                    KRIL_DEBUG(DBG_ERROR,"Allocate bcm_ril_rsp memory failed!!\n");
                    pdata->handler_state = BCM_ErrorCAPI2Cmd;
                    return;
                }
                memset(pdata->bcm_ril_rsp, 0, sizeof(char)*QueryMeasureReportLen);                
                memcpy(pdata->bcm_ril_rsp, QueryMeasureReportBuf, sizeof(char)*QueryMeasureReportLen);            
                pdata->rsp_len = sizeof(char)*QueryMeasureReportLen ;
                pdata->handler_state = BCM_FinishCAPI2Cmd;
            }
            KRIL_SetInQueryMeasureReportHandler( FALSE );
        }
        break;
        default:
        {
            KRIL_DEBUG(DBG_ERROR, "handler_state:%lu error...!\n", pdata->handler_state);
            pdata->handler_state = BCM_ErrorCAPI2Cmd;
            KRIL_SetInQueryMeasureReportHandler( FALSE );
        }
        break;
    }
}

void ParseIMSIData(KRIL_CmdList_t *pdata, Kril_CAPI2Info_t *capi2_rsp)
{
    IMSI_t* rsp = (IMSI_t*)capi2_rsp->dataBuf;

    KrilImsiData_t *imsi_result;

    if (!rsp)
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp->dataBuf is NULL, Error!!\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }
    
    pdata->bcm_ril_rsp = kmalloc(sizeof(KrilImsiData_t), GFP_KERNEL);
    if (!pdata->bcm_ril_rsp)
    {
        KRIL_DEBUG(DBG_ERROR,"Allocate bcm_ril_rsp memory failed!!\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }    

    imsi_result = pdata->bcm_ril_rsp;
    memset(imsi_result, 0, sizeof(KrilImsiData_t));
    pdata->rsp_len = sizeof(KrilImsiData_t);
    
    if ( capi2_rsp->result != RESULT_OK )
    {
        KRIL_DEBUG(DBG_ERROR,"IMSI: SIM access failed!! result:%d\n",capi2_rsp->result);
        imsi_result->result = BCM_E_GENERIC_FAILURE;
    }
    else
    {
        imsi_result->result = BCM_E_SUCCESS;
        KRIL_DEBUG(DBG_INFO,"IMSI:%s\n", (char*)rsp);
        memcpy(imsi_result->imsi, (char*)rsp, (IMSI_DIGITS+1));
    }
    
    pdata->handler_state = BCM_FinishCAPI2Cmd;
}

//******************************************************************************
// Function Name:      ParseIMEIData
//
// Description:        Internal helper function used to parse CAPI response
//                     to retrieval of IMEI from MS database. If retrieval
//                     from MS database fails, or IMEI stored there is all
//                     0's, we instead return the IMEI stored in sysparms
//                     (this mirrors behaviour of SYSPARM_GetImeiStr() on CP)
//
//******************************************************************************
static void ParseIMEIData(KRIL_CmdList_t* pdata, Kril_CAPI2Info_t* capi2_rsp)
{
    CAPI2_MS_Element_t* rsp = (CAPI2_MS_Element_t*)capi2_rsp->dataBuf;
    KrilImeiData_t* imei_result;

    if (!rsp)
    {
        KRIL_DEBUG(DBG_ERROR,"capi2_rsp->dataBuf is NULL, Error!!\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }
    
    pdata->bcm_ril_rsp = kmalloc(sizeof(KrilImeiData_t), GFP_KERNEL);
    if (!pdata->bcm_ril_rsp)
    {
        KRIL_DEBUG(DBG_ERROR,"Allocate bcm_ril_rsp memory failed!!\n");
        pdata->handler_state = BCM_ErrorCAPI2Cmd;
        return;
    }    

    imei_result = pdata->bcm_ril_rsp;
    memset(imei_result, 0, sizeof(KrilImeiData_t));
    pdata->rsp_len = sizeof(KrilImeiData_t);
    
    if (rsp->inElemType != MS_LOCAL_PHCTRL_ELEM_IMEI)
    {
        KRIL_DEBUG(DBG_ERROR,"IMEI: inElemType Error!! inElemType:%d\n",rsp->inElemType);
        imei_result->result = BCM_E_GENERIC_FAILURE;
    }
    else
    {
        // at this point, we have the MS database IMEI string; check if it is non-zero, and if not,
        // use IMEI from sysparms
        UInt8 i=0;
        Boolean bUseMSDBImei = FALSE;
        UInt8* pMSDBImeiStr = (UInt8*)rsp->data_u.imeidata;

        imei_result->result = BCM_E_SUCCESS;
        
        do
        {
            bUseMSDBImei = (pMSDBImeiStr[i] != '0');
        }
        while ( !bUseMSDBImei && (++i < IMEI_DIGITS) );
        
        if ( bUseMSDBImei )
        {
            // MS database IMEI is not all 0's, so we use it
            strncpy(imei_result->imei, pMSDBImeiStr, IMEI_DIGITS);
            imei_result->imei[IMEI_DIGITS] = '\0';
            KRIL_DEBUG(DBG_INFO,"Using MS DB IMEI:%s\n", pMSDBImeiStr );
        }
        else
        {
            // MS database IMEI is all 0's, so retrieve IMEI from sysparms
            UInt8 tmpImeiStr[IMEI_STRING_LEN];
            Boolean bFound;
            
            // retrieve null terminated IMEI string
            bFound = SYSPARM_GetImeiStr( (UInt8) pdata->ril_cmd->SimId, tmpImeiStr );
            if ( bFound )
            {
                // got it from sysparms, so copy to the response struct
                KRIL_DEBUG(DBG_INFO,"Using sysparm IMEI:%s\n", tmpImeiStr );
                strncpy(imei_result->imei, tmpImeiStr, IMEI_STRING_LEN);
            }
            else
            {
                KRIL_DEBUG(DBG_INFO,"** SYSPARM_GetImeiStr() failed\n" );
                imei_result->result = BCM_E_GENERIC_FAILURE;
            }
        }
        
    }
    
    pdata->handler_state = BCM_FinishCAPI2Cmd;
}


//--------------------------------------------------------------------------
// Function Name: ReadIMEI()
//
// Description:  Get IMEI information
//
// Return:  IMEI string
//      
//
//----------------------------------------------------------------------------
UInt8* ReadIMEI(SimNumber_t simid)
{
    return sImei_Info[simid];
}


//--------------------------------------------------------------------------
// Function Name: ProcessImei()
//
// Description:  Convert the IMEI string to the format in our system parameter files
//
// Return:  TRUE - Convert is successful; FALSE - Convert is failed.
//      
//
//----------------------------------------------------------------------------
#ifdef CONFIG_BRCM_SIM_SECURE_ENABLE
static Boolean ProcessImei(UInt8* imeiStr, UInt8* imeiVal)
{
    int i ;
    
    for (i = 0; i < 14; i++)
    {
        if (!isdigit(imeiStr[i]))
        {
            return FALSE ;
        }
    }
    
    /* Use the format in our system parameter files: first nibble and last nibble are not used and
     * set to 0, giving rise to a total of 8 bytes. 
     */
    imeiVal[0] = ( imeiStr[0] - '0')  << 4;
    imeiVal[1] = ((imeiStr[2] - '0')  << 4)  | (imeiStr[1] - '0');
    imeiVal[2] = ((imeiStr[4] - '0')  << 4)  | (imeiStr[3] - '0');
    imeiVal[3] = ((imeiStr[6] - '0')  << 4)  | (imeiStr[5] - '0');
    imeiVal[4] = ((imeiStr[8] - '0')  << 4)  | (imeiStr[7] - '0');
    imeiVal[5] = ((imeiStr[10] - '0') << 4)  | (imeiStr[9] - '0');
    imeiVal[6] = ((imeiStr[12] - '0') << 4)  | (imeiStr[11] - '0');
    imeiVal[7] = imeiStr[13] - '0';
    
    return TRUE;
}
#endif // CONFIG_BRCM_SIM_SECURE_ENABLE
