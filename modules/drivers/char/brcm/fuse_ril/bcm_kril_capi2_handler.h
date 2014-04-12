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

#ifndef _BCM_KRIL_CAPI2_HANDLER_H
#define _BCM_KRIL_CAPI2_HANDLER_H
#include "bcm_kril_cmd_handler.h"

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Query Measure Report handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_QueryMeasureReport 0x0100


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Init Command handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_AT_ProcessCmd                         0x0100
#define BCM_SET_IMEI                              0x0101
#define BCM_SMS_ELEM_CLIENT_HANDLE_MT_SMS         0x0102
#define BCM_SMS_SetSmsReadStatusChangeMode        0x0103
#define BCM_SMS_SetVMIndOnOff                     0x0104
#define BCM_SYS_SetFilteredEventMask              0x0105
#define BCM_SYS_SetRssiThreshold                  0x0106
#define BCM_PMU_Low_Battery_Register              0x0107
#define BCM_PMU_Empty_Battery_Register            0x0108
#define BCM_PMU_LevelChange_Battery_Register      0x0109
#define BCM_SATK_SetTermProfile                   0x010A
#define BCM_SATK_SETUP_CALL_CTR                   0x010B
#define BCM_SET_GPRSClassCC                       0x010C
#define BCM_SET_GPRSClassB                        0x010D
#define BCM_SET_SupportedRATandBand               0x010E
#define BCM_SET_RADIO_OFF                         0x010F
#define BCM_Demand_Attach_Always                  0x0110
#define BCM_TIMEZONE_SetTZUpdateMode              0x0111
#define BCM_SS_SET_ENABLE_OLD_SS_MSG              0x0112
#define BCM_SET_BPMSetting                        0x0113
#define BCM_AT_AUDIO_CTRL                         0x0114
#define BCM_SATK_ICON_DISP_SUPPORTED              0x0115
#define BCM_Demand_SIM2_Attach_Always             0x0116
#define BCM_SET_SIM_LOCK_SUPPORTED                0x0117
#define BCM_SATK_ELEM_ENABLE_TEXT_CONVERSIONS     0x0118
#define BCM_SET_STACK_CTRL                        0x0119
#define BCM_SET_PDP_NWI_CTRL_FLAG                 0x011A
#define BCM_SET_AMR_TYPE                          0x011B

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Device Identity handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_GetIMEIInfo                         0x0100


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Get SIM Emergency Number handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_PBK_SendInfoReq                  0x0100
#define BCM_PBK_ReadENEnteryReq              0x0101


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Dial handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SIM_GetAlsDefaultLine  0x0100
#define BCM_CC_SetCBSTFinished     0x0101
#define BCM_CC_MakeVideoCall       0x0102


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Hangup handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CC_GetCurrentCallIndex  0x0100


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Get Current Call handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CC_AllCallIndex          0x0099 // CSP 381563
#define BCM_CC_GetCallType           0x0101
#define BCM_CC_GetCallNumber         0x0102
#define BCM_CC_IsMultiPartyCall      0x0103
#define BCM_CC_GetCNAPName           0x0104
#define BCM_CC_GetAllCallStates      0x0105
#define BCM_ECHO_SetDigitalTxGain    0x0106
#define BCM_CC_SetVTCallEndCause     0x0107
#define BCM_CC_GetVTCallEndResult    0x0108
#define BCM_CC_GetCallState          0x0109


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Hangup Waiting Or Background handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CC_EndNextHeldCall  0x0105


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Hangup Foreground Resume Background handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CC_GetNextActiveCallIndex      0x0100
#define BCM_CC_GetNextWaitCallIndex        0x0101
#define BCM_CC_AcceptVoiceCall             0x0102
#define BCM_CC_GetNextHeldCallIndex        0x0103
#define BCM_CC_EndMPTYCalls                0x0104


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Switch Waiting Or Holding And Active handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CC_GetMPTYCallIndex        0x0104
#define BCM_CC_SwapCall                0x0105
#define BCM_CC_HoldAnswerWaitCall      0x0106
#define BCM_CC_HoldCallFailEndWaitCall 0x0107


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Explicit Call Transfer handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CC_TransferCall      0x0105


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Send DTMF Request handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_CcApi_SetDtmfTimer     0x0100
#define BCM_CcApi_SendDtmf         0x0101
#define BCM_CcApi_ResetDtmfTimer   0x0102

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set_Emergency Idle Mode for wifi calling
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_Set_Emergency_Idle_Mode     0x001
#define BCM_TurnOffRadio                0x002
#define BCM_Wait_GSM_NO_Service         0x003
#define BCM_TurnOnRadio                 0x004


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Registation State handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_MS_GetRNC   0x001

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Query Available Networks name handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_ELEM_UAS_INFO           0x001
#define BCM_Search_Available_PLMN   0x002

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Operator handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_MS_GetRegistrationInfo   0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Signal Strength handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_MS_GetCurrentRAT         0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set Preferred Network Type handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_NET_QueryCurrentRAT         0x001
#define BCM_NET_SendChangeRAT           0x002
#define BCM_SYS_SelectBand              0x003
#define BCM_REVERT_GPRSClassB           0x004


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set Network Selection Manual handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_MS_AbortPlmnSelect     0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Get Neighboring Cell IDs handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SYS_EnableCellInfoMsg     0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @Enter SIM PIN
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SIM_GetRemainingPinAttempt    0x0100
#define BCM_SS_SendCallBarringReq         0x0101

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @Sim Close Channel
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SIM_CloseSocket 			   0x0100


//EAP-SIM
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @Do Authentication
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SIM_DoGSMAuthentication               0x0100
#define BCM_SIM_SelectMF                          0x0101
#define BCM_SIM_SelectDedicatedFile               0x0102
#define BCM_SIM_GetAdditionDataReq                0x0103
#define BCM_SIM_DoUSimAuthentication              0x0104
#define BCM_SIM_SelectMFUsim                      0x0105
#define BCM_SIM_SelectDedicatedFileOfADF          0x0106


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @Restricted SIM access
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SIM_GetServiceStatus            0x0100
#define BCM_SIM_SIMIO_UpdateFDNRecord       0x0101
#define BCM_SIM_PBK_UpdateFDNRecord         0x0102
#define BCM_SIM_GetFDNInfo                  0x0103
#define BCM_SIM_UpdateFDNRsult              0x0104


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Write SMS in SIM handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_SendAckToNetwork   0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Send SMS Expect More handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_SendSMSPduReq               0x001
#define BCM_SMS_DisableMoreMessageToSend    0x002
#define BCM_SMS_StartMultiSMSTransfer       0x003
#define BCM_SMS_StopMultiSMSTransfer        0x004


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Report ME SMS Status handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_SendMemAvailInd            0x001
#define BCM_SIM_UpdateSMSCapExceededFlag 0x002


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Query SMS in SIM handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SIM_SubmitRecordEFileReadReq   0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set PDP Context handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_PDP_SetPdpContext         0x002
#define BCM_PDP_SetPdpOption          0x003
#define BCM_ActivatePDP               0x004
#define BCM_ReadProtConfig            0x005


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set CLIP/CLIR state handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SS_SetElement         0x001
#define BCM_SS_GetElement         0x002


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set CBSMS Get CBMI handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_GetCBMI 0x001


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Set CBSMS Turn-off handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_SetCBOff 0x001



// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Start CB Normal Service
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_StartCBService 0x001

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Start CB CMAS Service
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_StartCBCMASService 0x002

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Stop CB Service
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_StopCBService 0x003

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Check CBMI Number
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_SMS_CheckCBMINum 0x004


#define BCM_PDP_Verify 0x002

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Query SIM STATUS
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_IS_PIN1_ENABLE  0x100
#define BCM_IS_PUK2_BLOCK   0x101
#define BCM_IS_PIN2_BLOCK   0x102
#define BCM_IS_PIN2_ENABLE  0x103
#define BCM_IS_ISIM_SUPPORTED 0x104

#ifdef CONFIG_BRCM_RADIO_POWER_ONOFF_CARD
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Radio Power
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_GET_SYSTEM_STATE_RSP  	0x100
#define BCM_SET_SYSTEM_STATE   		0x101
#define BCM_SIM_POWER_ON_OFF_CARD   0x102
#endif
//luobiao@wind-mobi.com start 2012.05.17
//patch for csp:524605
#define BCM_SMS_Start_Stop_CBService    0x0103
//luobiao@wind-mobi.com end 2012.05.17

// for Call Control
void KRIL_GetCurrentCallHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_DialHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_HangupHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_HangupWaitingOrBackgroundHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_HangupForegroundResumeBackgroundHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SwitchWaitingOrHoldingAndActiveHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ConferenceHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_UDUBHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AnswerHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_LastCallFailCauseHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SeparateConnectionHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ExplicitCallTransferHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendDTMFRequestHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendDTMFStartHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendDTMFStopHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

// for PDP data
void KRIL_SetupPdpHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_DeactivatePdpHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_DataStateHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendDataHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetPreferredDataHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_RejectNWIPdpActivate(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

// for Network
void KRIL_SignalStrengthHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_RegistationStateHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetBPMModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
//void KRIL_GPRS_RegistationStateHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_OperatorHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryNetworkSelectionModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetNetworkSelectionAutomaticHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetNetworkSelectionManualHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryAvailableNetworksHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetBandModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryAvailableBandModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetPreferredNetworkTypeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetPreferredNetworkTypeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetNeighboringCellIDsHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetLocationUpdatesHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);


// for Supplementary Service
void KRIL_SendUSSDHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_CancelUSSDHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetCLIRHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetCLIRHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryCallForwardStatusHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetCallForwardStatusHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryCallWaitingHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetCallWaitingHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ChangeBarringPasswordHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryCLIPHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetSuppSvcNotificationHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);


// for SMS
void KRIL_SMSAcknowledgeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendSMSHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendSMSExpectMoreHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_WriteSMSToSIMHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_DeleteSMSOnSIMHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetSMSCAddressHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetSMSCAddressHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ReportSMSMemoryStatusHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QuerySMSInSIMHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetSmsSimMaxCapacityHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ReadSmsOnSimHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ListSmsOnSimHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SendStoredSmsHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetElemCscsHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetElemMoreMsgToSendHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetElemMoreMsgToSendHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_StartMultiSmsTxHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_StopMultiSmsTxHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

// for engineer mode
void KRIL_QueryMeasureReportHandler(void *ril_cmd,  Kril_CAPI2Info_t *capi2_rsp);

// for CB
void KRIL_GetBroadcastSmsHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetBroadcastSmsHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SmsBroadcastActivationHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);


// for SIM
void KRIL_GetSimStatusHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_EnterSimPinHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_EnterSimPukHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_EnterNetworkDepersonHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ChangeSimPinHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryFacilityLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetFacilityLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SimIOHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QuerySimPinRemainingHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryNetworkLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryNetworkSubsetLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryServiceProvideLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryCorporateLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryPhoneLockHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GenericSimAccessHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SimOpenChannelHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SimCloseChannelHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_UiccApplicationIOHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void BRIL_IsimAkaAuthenticationHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void BRIL_IsimGbaAuthenticateBootstrapHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void BRIL_IsimGbaAuthenticateNafHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

//EAP-SIM
void KRIL_GsmSimAuthenticationHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_USimAuthenticationHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);


// for STK
void KRIL_StkSendTerminalRspHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_StkSendEnvelopeCmdHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_StkHandleCallSetupRequestedHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_StkGetProfile(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_StkSetProfile(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

// for Misc
void KRIL_InitCmdHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_RadioPowerHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_BasebandVersionHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

void KRIL_GetIMSIHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetIMEIHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetIMEISVHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetDeviceIdentityHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QuerySimEmergencyNumberHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetTTYModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_QueryTTYModeHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_GetCurrentSimVoltageHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_SetPdpActivationFdnControlHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

void KRIL_ATProcessCmdHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

// for OEM Hook RAW 
void KRIL_OemHookRawHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_ConfigEmergencyIdleMode(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

#ifdef BRCM_AGPS_CONTROL_PLANE_ENABLE
void KRIL_AgpsSendUpLinkHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AgpsRrcMcFailureHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AgpsRrcStatusHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);

#ifdef BRCM_AGPS_PKCS15_ENABLE
void KRIL_AgpsOpenPkcs15Handler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AgpsSimEfInfoHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AgpsSimRecHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AgpsSimEfHandler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
void KRIL_AgpsClosePkcs15Handler(void *ril_cmd, Kril_CAPI2Info_t *capi2_rsp);
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Open PKCS#15 handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_AGPS_PKCS15_SUPPORT		0x100
#define BCM_AGPS_OPEN_SOCKET		0x101
#define BCM_AGPS_SELECT_PKCS15		0x102

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Get EF info handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_AGPS_GET_EF_INFO		0x100

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @ Close PKCS#15 handler
//
// @comm None
//
// -----------------------------------------------------------------------------
#define BCM_AGPS_DESELECT_PKCS15		0x100


#endif	//BRCM_AGPS_PKCS15_ENABLE

#endif //BRCM_AGPS_CONTROL_PLANE_ENABLE

#endif //_BCM_KRIL_CAPI2_HANDLER_H
