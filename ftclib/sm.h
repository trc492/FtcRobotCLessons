#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="sm.h" />
///
/// <summary>
///     This module contains the library functions to handle state machines.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SM_H
#define _SM_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SM

//
// Constants.
//
#define SMO_WAIT_ALL            0x0001
#define SMO_NO_CLEAR_EVENTS     0x0002
#define SMO_SET_TIMEOUT         0x0004

#define SMF_READY               0x0001
#define SMF_TIMEOUT             0x0002
#define SMF_DELAY_WAIT          0x0004

#define SMSTATE_DISABLED        0
#define SMSTATE_STARTED         1

#define EVTTYPE_NONE            0

#ifndef MAX_WAIT_EVENTS
#define MAX_WAIT_EVENTS         4
#endif

//
// Macros.
//

/**
 *  This macro checks if the State Machine is disabled.
 *
 *  @param s Points to the SM structure.
 *
 *  @return Returns true if the state machine is disabled.
 */
#define SMIsDisabled(s)         ((s).currState == SMSTATE_DISABLED)

/**
 *  This macro checks if the State Machine is enabled.
 *
 *  @param s Points to the SM structure.
 *
 *  @return Returns true if the state machine is enabled.
 */
#define SMIsEnabled(s)          ((s).currState != SMSTATE_DISABLED)

/**
 *  This macro checks if the State Machine has timed out.
 *
 *  @param s Points to the SM structure.
 *
 *  @return Returns true if the state machine has timed out.
 */
#define SMIsTimedOut(s)         ((s).flags & SMF_TIMEOUT)

/**
 *  This macro returns the current state.
 *
 *  @param s Points to the SM structure.
 *
 *  @return Returns current state.
 */
#define SMGetState(s)           ((s).currState)

/**
 *  This macro sets the current state.
 *
 *  @param s Points to the SM structure.
 *  @param n Specifies the new state.
 *
 *  @return Returns current state.
 */
#define SMSetState(s,n)         (s).currState = n

//
// Type definitions.
//
typedef struct
{
    int  evtType;
    int  evtID;
    int  evtData;
    int  evtParam1;
    int  evtParam2;
    bool fSignaled;
} WAIT_EVT;

typedef struct
{
    int             currState;
    int             nextState;
    unsigned long   waitTime;
    int             options;
    int             flags;
    int             nWaitEvents;
    WAIT_EVT        WaitEvents[MAX_WAIT_EVENTS];
} SM;

/**
 *  This function clears all wait events in the state machine.
 *
 *  @param sm Points to the SM structure.
 */
void
SMClearAllEvents(
    SM &sm
    )
{
    TFuncName("SMClearAllEvents");
    TLevel(API);
    TEnter();

    for (int i = 0; i < MAX_WAIT_EVENTS; ++i)
    {
        sm.WaitEvents[i].evtType = EVTTYPE_NONE;
        sm.WaitEvents[i].evtID = 0;
        sm.WaitEvents[i].evtData = 0;
        sm.WaitEvents[i].evtParam1 = 0;
        sm.WaitEvents[i].evtParam2 = 0;
        sm.WaitEvents[i].fSignaled = false;
    }
    sm.nWaitEvents = 0;

    TExit();
    return;
}   //SMClearAllEvents

/**
 *  This function initializes the state machine.
 *
 *  @param sm Points to the SM structure.
 */
void
SMInit(
    SM &sm
    )
{
    TFuncName("SMInit");
    TLevel(INIT);
    TEnter();

    sm.currState = SMSTATE_DISABLED;
    sm.nextState = SMSTATE_DISABLED;
    sm.waitTime = 0;
    sm.options = 0;
    sm.flags = 0;
    SMClearAllEvents(sm);

    TExit();
    return;
}   //SMInit

/**
 *  This function starts the state machine.
 *
 *  @param sm Points to the SM structure.
 *  @param startState Optionally specifies the starting state.
 */
void
SMStart(
    SM &sm,
    int startState = SMSTATE_STARTED
    )
{
    TFuncName("SMStart");
    TLevel(API);
    TEnter();

    if (sm.currState == SMSTATE_DISABLED)
    {
        sm.currState = startState;
        sm.nextState = startState;
        sm.waitTime = 0;
        sm.flags = SMF_READY;
        SMClearAllEvents(sm);
    }

    TExit();
    return;
}   //SMStart

/**
 *  This function stops the state machine.
 *
 *  @param sm Points to the SM structure.
 */
void
SMStop(
    SM &sm
    )
{
    TFuncName("SMStop");
    TLevel(API);
    TEnter();

    SMInit(sm);

    TExit();
    return;
}   //SMStop

/**
 *  This function checks if the state machine is ready or timed out.
 *
 *  @param sm Points to the SM structure.
 */
bool
SMIsReady(
    SM &sm
    )
{
    bool fReady;

    TFuncName("SMIsReady");
    TLevel(API);
    TEnter();

    if (SMIsEnabled(sm) &&
        !(sm.flags & SMF_READY) &&
        (sm.waitTime > 0) &&
        ((sm.options & SMO_SET_TIMEOUT) ||
         (sm.flags & SMF_DELAY_WAIT)) &&
        (nPgmTime >= sm.waitTime))
    {
        //
        // Not ready but we have either set a timeout or we have started a
        // delay wait timer and the time has run out.
        //
        sm.waitTime = 0;
        sm.flags &= ~SMF_DELAY_WAIT;
        sm.flags |= SMF_READY;
        if (sm.options & SMO_SET_TIMEOUT)
        {
            sm.flags |= SMF_TIMEOUT;
        }

        if ((sm.options & SMO_NO_CLEAR_EVENTS) == 0)
        {
            SMClearAllEvents(sm);
        }
        sm.currState = sm.nextState;
    }

    fReady = SMIsEnabled(sm) && ((sm.flags & SMF_READY) != 0);

    TExitMsg(("%=d", (byte)fReady));
    return fReady;
}   //SMIsReady

/**
 *  This function adds a wait event to the state machine.
 *
 *  @param sm Points to the SM structure.
 *  @param evtType Specifies the event type to wait for.
 *  @param evtID Optionally specifies the event ID to wait for.
 *  @param evtData Optionally specifies the event data to wait for.
 *
 *  @return Success: Returns true.
 *  @return Failure: Returns false.
 */
bool
SMAddWaitEvent(
    SM &sm,
    int evtType,
    int evtID = -1,
    int evtData = -1
    )
{
    TFuncName("SMAddWaitEvent");
    TLevel(API);
    TEnterMsg(("Type=%x,ID=%x", evtType, evtID));

    bool fAdded = false;

    if (sm.nWaitEvents < MAX_WAIT_EVENTS)
    {
        sm.WaitEvents[sm.nWaitEvents].evtType = evtType;
        sm.WaitEvents[sm.nWaitEvents].evtID = evtID;
        sm.WaitEvents[sm.nWaitEvents].evtData = evtData;
        sm.WaitEvents[sm.nWaitEvents].evtParam1 = 0;
        sm.WaitEvents[sm.nWaitEvents].evtParam2 = 0;
        sm.WaitEvents[sm.nWaitEvents].fSignaled = false;
        sm.nWaitEvents++;
        fAdded = true;
    }
    TInfo(("nEvts=%d", sm.nWaitEvents));

    TExitMsg(("fOK=%d", (byte)fAdded));
    return fAdded;
}   //SMAddWaitEvent

/**
 *  This function sets the wait event mode and the next state to advance to
 *  when the wait is fulfilled.
 *
 *  @param sm Points to the SM structure.
 *  @param nextState Specifies the next state to advance to.
 *  @param waitTime Optionally specifies either the timeout value or
 *         the wait time in msec to delay after the events have been fired.
 *  @param options Optionally specifies the SMF options:
 *         SMO_WAIT_ALL - Wait for all events.
 *         SMO_NO_CLEAR_EVENTS - Do not clear events when ready.
 *         SMO_SET_TIMEOUT - waitTime specifies a timeout value.
 */
void
SMWaitEvents(
    SM &sm,
    int nextState,
    unsigned long waitTime = 0,
    int options = 0
    )
{
    TFuncName("SMWaitEvents");
    TLevel(API);
    TEnterMsg(("Next=%d,waitTime=%d", nextState, waitTime));

    if (sm.nWaitEvents > 0)
    {
        sm.nextState = nextState;
        sm.waitTime = waitTime;
        if ((waitTime > 0) && (options & SMO_SET_TIMEOUT))
        {
            sm.waitTime += nPgmTime;
        }
        sm.options = options;
        sm.flags = 0;
    }

    TExit();
    return;
}   //SMWaitEvents

/**
 *  This function is called when an event has occurred. It will determine if
 *  the state machine is waiting for the event and will advance the state
 *  machine to the next state if necessary.
 *
 *  @param sm Points to the SM structure.
 *  @param evtType Specifies the event type.
 *  @param evtID Optionally specifies the event ID.
 *  @param evtData Optionally specifies the event data.
 *  @param evtParam1 Optionally specifies the event parameter 1.
 *  @param evtParam2 Optionally specifies the event parameter 2.
 */
void
SMSetEvent(
    SM &sm,
    int evtType,
    int evtID = 0,
    int evtData = 0,
    int evtParam1 = 0,
    int evtParam2 = 0
    )
{
    TFuncName("SMSetEvent");
    TLevel(API);
    TEnterMsg(("Type=%x,ID=%x", evtType, evtID));

    if (SMIsEnabled(sm))
    {
        for (int i = 0; i < sm.nWaitEvents; ++i)
        {
            if (!sm.WaitEvents[i].fSignaled &&
                (sm.WaitEvents[i].evtType == evtType) &&
                ((sm.WaitEvents[i].evtID == -1) ||
                 (sm.WaitEvents[i].evtID == evtID)) &&
                ((sm.WaitEvents[i].evtData == -1) ||
                 (sm.WaitEvents[i].evtData == evtData)))
            {
                //
                // If the all event attributes matched or we don't care some of
                // them, we mark the event signaled.
                //
                TInfo(("Type=%x,ID=%x",
                       sm.WaitEvents[i].evtType, sm.WaitEvents[i].evtID));
                sm.WaitEvents[i].fSignaled = true;
                sm.WaitEvents[i].evtID = evtID;
                sm.WaitEvents[i].evtData = evtData;
                sm.WaitEvents[i].evtParam1 = evtParam1;
                sm.WaitEvents[i].evtParam2 = evtParam2;

                bool fAdvanceState = true;
                if (sm.options & SMO_WAIT_ALL)
                {
                    for (int j = 0; j < sm.nWaitEvents; ++j)
                    {
                        if (!sm.WaitEvents[j].fSignaled)
                        {
                            fAdvanceState = false;
                            break;
                        }
                    }
                }

                if (fAdvanceState)
                {
                    //
                    // We have satisfied the wait, get ready to advance to the
                    // next state.
                    //
                    TInfo(("AdvanceState"));
                    if ((sm.waitTime > 0) && !(sm.options & SMO_SET_TIMEOUT))
                    {
                        //
                        // We have set a delay wait time, start the delay wait
                        // timer now.
                        //
                        sm.waitTime += nPgmTime;
                        sm.flags |= SMF_DELAY_WAIT;
                    }
                    else
                    {
                        if ((sm.options & SMO_NO_CLEAR_EVENTS) == 0)
                        {
                            SMClearAllEvents(sm);
                        }
                        sm.currState = sm.nextState;
                        sm.waitTime = 0;
                        sm.flags |= SMF_READY;
                    }
                }
                break;
            }
        }
    }

    TExit();
  return;
}   //SMSetEvent

#endif  //ifndef _SM_H
