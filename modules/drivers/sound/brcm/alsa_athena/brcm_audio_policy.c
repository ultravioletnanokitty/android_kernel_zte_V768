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


/*
*
*****************************************************************************
*
*  brcm_audio_policy.c
*
*  PURPOSE:
*
*     Apply policy based on current mode and new mode.
*
*  NOTES:
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */


#include "mobcom_types.h"
#include "resultcode.h"
#include "audio_consts.h"

#include "brcm_audio_policy.h"

// Include BRCM AAUD driver API header files
#include "audio_controller.h"

/* Local typedefs */
#include "log.h"
#include "brcm_alsa.h"
//xiaocui@wind-mobi.com 2012.1.14 begin
//Fix bug 7140,6891.sound of music became poor after the end of video
//Use CSP493759_PATCH
typedef struct 
{
    BRCM_STATE_ENUM     state;
    UInt32     audioMode;
	UInt32     audioApp;
}BRCM_PREVMODEQueue;
//xiaocui@wind-mobi.com 2012.1.14 end
static UInt32 tState = BRCM_STATE_NORMAL;
static UInt32 tPrevState = BRCM_STATE_NORMAL;
static Boolean tActiveStateTbl[AUDIO_STATE_NUM] = {TRUE, FALSE, FALSE, FALSE}; 

/* ---- Functions ecported ---------------------------------------------------- */

static Int32 NextAudioProfile[AUDIO_STATE_NUM][AUDIO_APP_NUMBER] =
{
    {AUDIO_APP_VOICE_CALL, AUDIO_APP_VOICE_CALL_WB, AUDIO_APP_MUSIC,AUDIO_APP_RECORDING,AUDIO_APP_FM, AUDIO_APP_RECORDING_GVS,AUDIO_APP_VOIP,AUDIO_APP_VOIP_INCOMM,AUDIO_APP_VT_CALL,AUDIO_APP_VT_CALL_WB},  //BRCM_STATE_NORMAL
    {AUDIO_APP_VOICE_CALL, AUDIO_APP_VOICE_CALL_WB,-1,-1, -1, -1,-1,-1,AUDIO_APP_VT_CALL,AUDIO_APP_VT_CALL_WB}, //BRCM_STATE_INCALL
    {AUDIO_APP_VOICE_CALL, AUDIO_APP_VOICE_CALL_WB, AUDIO_APP_FM,AUDIO_APP_RECORDING,AUDIO_APP_FM, AUDIO_APP_RECORDING_GVS,AUDIO_APP_VOIP,AUDIO_APP_VOIP_INCOMM,AUDIO_APP_VT_CALL,AUDIO_APP_VT_CALL_WB},  //BRCM_STATE_FM
   // {AUDIO_APP_VOICE_CALL, AUDIO_APP_VOICE_CALL_WB, -1,AUDIO_APP_RECORDING,-1, AUDIO_APP_RECORDING_GVS,AUDIO_APP_VOIP,AUDIO_APP_VOIP_INCOMM,AUDIO_APP_VT_CALL,AUDIO_APP_VT_CALL_WB}  //csp 527296 xiaocui@wind-mobi.com delete BRCM_STATE_RECORD
	{AUDIO_APP_VOICE_CALL, AUDIO_APP_VOICE_CALL_WB, AUDIO_APP_MUSIC,AUDIO_APP_RECORDING,-1, AUDIO_APP_RECORDING_GVS,AUDIO_APP_VOIP,AUDIO_APP_VOIP_INCOMM,AUDIO_APP_VT_CALL,AUDIO_APP_VT_CALL_WB}  //csp 527296 xiaocui@wind-mobi.com add BRCM_STATE_RECORD	
};
//xiaocui@wind-mobi.com 2012.1.14 begin
//Fix bug 7140,6891.sound of music became poor after the end of video
//Use CSP493759_PATCH
static Int32 tTopStatePtr = 0;
static BRCM_PREVMODEQueue tPrevModeQueue[AUDIO_STATE_NUM]={{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
//xiaocui@wind-mobi.com 2012.1.14 end

/* ---- Data structure  ------------------------------------------------- */

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//AUDIO_Policy_SetState
//	Sets the state.
//----------------------------------------------------------------
Result_t AUDIO_Policy_SetState(UInt32 state)
{
    if(state >= BRCM_STATE_END)
    {
        return 0;
    }
	
	tActiveStateTbl[state] =TRUE;
	
    // if we are currently in incall state, don't change it.
    if(tState == BRCM_STATE_INCALL)
    {
    	tPrevState = state;
        return 0;
    }
	
    // back up the current state
    tPrevState  = tState;	
    tState = state;

    DEBUG("AUDIO_Policy_SetState:tPrevState - %d tState-%d state - %d {%d,%d,%d,%d}\n",
		tPrevState,tState,state,tActiveStateTbl[0],tActiveStateTbl[1],tActiveStateTbl[2],tActiveStateTbl[3]);
    return 1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//AUDIO_Policy_RestoreState
//	restore the state.
//----------------------------------------------------------------
Result_t AUDIO_Policy_RestoreState(UInt32 state)
{
    if(state >= BRCM_STATE_END)
    {
        return 0;
    }

	tActiveStateTbl[state] = FALSE;

	if(state = tState)
	{
		tState = tPrevState;
		tPrevState = BRCM_STATE_NORMAL;
	}
	else if(state == tPrevState)
	{
		if(tActiveStateTbl[BRCM_STATE_RECORD]==TRUE)
			tPrevState = BRCM_STATE_RECORD;
		else if(tActiveStateTbl[BRCM_STATE_FM]==TRUE)
			tPrevState = BRCM_STATE_FM;
		else
			tPrevState = BRCM_STATE_NORMAL;
	}

    DEBUG("AUDIO_Policy_RestoreState:tPrevState - %d tState-%d state {%d,%d,%d,%d}\n",
		tPrevState,tState,state,tActiveStateTbl[0],tActiveStateTbl[1],tActiveStateTbl[2],tActiveStateTbl[3]);
    return 1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//AUDIO_Policy_GetState
//	Gets the state.
//----------------------------------------------------------------
UInt32 AUDIO_Policy_GetState(void)
{
     return tState;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//AUDIO_Policy_Get_Profile
//	Identifies the profile to be applied.
//----------------------------------------------------------------
UInt32 AUDIO_Policy_Get_Profile(UInt32 app)
{
    AudioApp_t  new_app, cur_app;

    if(app >= AUDIO_APP_NUMBER)
    {
        app = AUDIO_APP_VOICE_CALL;  // defalult profile
    }
    cur_app = AUDCTRL_GetAudioApp();

    if(cur_app >= AUDIO_APP_NUMBER)
    {
        cur_app = AUDIO_APP_VOICE_CALL;  // defalult profile
    }
    new_app = NextAudioProfile[tState][app];
    if(new_app == -1)
    {
        //Set the profile to existing profile
        new_app = cur_app;
    }
    DEBUG("AUDIO_Policy_Get_Profile:cur_app - %d new_app-%d app - %d\n",cur_app,new_app,app);
    return new_app;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//AUDIO_Policy_Get_Mode
//	Identifies the mode to be applied.
//----------------------------------------------------------------
UInt32 AUDIO_Policy_Get_Mode(UInt32 mode)
{
    AudioMode_t  new_mode, cur_mode;

    cur_mode = AUDCTRL_GetAudioMode();

    if ( cur_mode >= AUDIO_MODE_NUMBER )
        cur_mode = (AudioMode_t) (cur_mode - AUDIO_MODE_NUMBER);

    if((tState == BRCM_STATE_INCALL)||(tState==BRCM_STATE_RECORD))
        new_mode = cur_mode;
    else 
        new_mode = mode;

    return new_mode;
}
//xiaocui@wind-mobi.com 2012.1.14 begin
//Fix bug 7140,6891.sound of music became poor after the end of video
//Use CSP493759_PATCH
Result_t AUDIO_Policy_AddModeToQueue(UInt32 state, UInt32 mode, UInt32 app)
{
	Int32 i;
    if((mode >= AUDIO_MODE_INVALID)||(app >= AUDIO_APP_NUMBER)||(state >= BRCM_STATE_END))
    {
        return 0;
    }
	if(tTopStatePtr>BRCM_STATE_END)
	{
		DEBUG("AUDIO_Policy_AddModeToQueue(): error total %d active states.\n", tTopStatePtr);
		tTopStatePtr = 0;
		for(i=0; i<BRCM_STATE_END; i++)
		{
			tPrevModeQueue[i].state = 0;
			tPrevModeQueue[i].audioMode = 0;
			tPrevModeQueue[i].audioApp = 0;
		}
	}
	for(i=0; i<tTopStatePtr; i++)
	{
		if(tPrevModeQueue[i].state == state)
		{
			tPrevModeQueue[i].audioMode = mode;
			tPrevModeQueue[i].audioApp = app;
			return 1;
		}
	}
	if(tTopStatePtr<BRCM_STATE_END)
	{
		tPrevModeQueue[i].state = state;
		tPrevModeQueue[i].audioMode = mode;
		tPrevModeQueue[i].audioApp = app;
		tTopStatePtr++;
	}
	DEBUG("AUDIO_Policy_AddModeToQueue(): state=%d, tTopStatePtr=%d \n", state, tTopStatePtr);
	DEBUG("Queue: {(%d,%d,%d),(%d,%d,%d),(%d,%d,%d),(%d,%d,%d)}\n", tPrevModeQueue[0].state, tPrevModeQueue[0].audioMode, tPrevModeQueue[0].audioApp,
		tPrevModeQueue[1].state, tPrevModeQueue[1].audioMode, tPrevModeQueue[1].audioApp,
		tPrevModeQueue[2].state, tPrevModeQueue[2].audioMode, tPrevModeQueue[2].audioApp,
		tPrevModeQueue[3].state, tPrevModeQueue[3].audioMode, tPrevModeQueue[3].audioApp);
	return 1;
}
Result_t AUDIO_Policy_RemoveModeFromQueue(UInt32 state,UInt32* pMode, UInt32* pApp)
{
	Int32 i;
	Result_t ret = 0;
	if(tTopStatePtr < 1)
	{
		return 0;
	}
	if(tTopStatePtr>BRCM_STATE_END)
	{
		DEBUG("AUDIO_Policy_RemoveModeFromQueue(): error total %d active states.\n", tTopStatePtr);
		tTopStatePtr = 0;
		for(i=0; i<BRCM_STATE_END; i++)
		{
			tPrevModeQueue[i].state = 0;
			tPrevModeQueue[i].audioMode = 0;
			tPrevModeQueue[i].audioApp = 0;
		}
		return 0;
	}
	for(i=tTopStatePtr-1; i>=0;i--)
	{
		if(tPrevModeQueue[i].state == state)
		{
			break;
		}
	}
	tTopStatePtr--;
	if(i==tTopStatePtr)
	{
		tPrevModeQueue[i].state = 0;
		tPrevModeQueue[i].audioMode = 0;
		tPrevModeQueue[i].audioApp = 0;
	}
	else
	{
		for(; i<tTopStatePtr; i++)
		{
			tPrevModeQueue[i].state = tPrevModeQueue[i+1].state;
			tPrevModeQueue[i].audioMode =tPrevModeQueue[i+1].audioMode; 
			tPrevModeQueue[i].audioApp =tPrevModeQueue[i+1].audioApp; 
		}
		tPrevModeQueue[i].state = 0;
		tPrevModeQueue[i].audioMode = 0;
		tPrevModeQueue[i].audioApp = 0;
	}
	if(tTopStatePtr>0)
	{
		*pMode = tPrevModeQueue[tTopStatePtr-1].audioMode;
		*pApp =  tPrevModeQueue[tTopStatePtr-1].audioApp;
		ret = 1;
	}	
	DEBUG("AUDIO_Policy_RemoveModeFromQueue(): state=%d, tTopStatePtr=%d \n", state, tTopStatePtr);
	DEBUG("Queue: {(%d,%d,%d),(%d,%d,%d),(%d,%d,%d),(%d,%d,%d)}\n", tPrevModeQueue[0].state, tPrevModeQueue[0].audioMode, tPrevModeQueue[0].audioApp,
		tPrevModeQueue[1].state, tPrevModeQueue[1].audioMode, tPrevModeQueue[1].audioApp,
		tPrevModeQueue[2].state, tPrevModeQueue[2].audioMode, tPrevModeQueue[2].audioApp,
		tPrevModeQueue[3].state, tPrevModeQueue[3].audioMode, tPrevModeQueue[3].audioApp);
	return ret;
}
//xiaocui@wind-mobi.com 2012.1.14 end
