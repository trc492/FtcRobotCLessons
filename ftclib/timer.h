#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="timer.h" />
///
/// <summary>
///     This module contains the library functions for the event timer.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TIMER_H
#define _TIMER_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TIMER

//
// Constants.
//
#define TIMERF_ENABLED          0x0001

//
// Type definitions.
//
typedef struct
{
    unsigned long   expiredTime;
    SM             *sm;
    int             evtType;
    int             flags;
} TIMER;

/**
 *  This function initializes the event timer.
 *
 *  @param timer Points to the TIMER structure.
 */
void
TimerInit(
    TIMER &timer
    )
{
    TFuncName("TimerInit");
    TLevel(INIT);
    TEnter();

    timer.expiredTime = 0;
    timer.sm = NULL;
    timer.evtType = 0;
    timer.flags = 0;

    TExit();
    return;
}   //TimerInit

/**
 *  This function sets the event timer.
 *
 *  @param timer Points to the TIMER structure.
 *  @param time Specifies the expire time relative to current time in msec.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signed when
 *         PID operation is completed. Required only if sm is not NULL.
 */
void
TimerSet(
    TIMER &timer,
    unsigned long time,
    SM *sm = NULL,
    int evtType = 0
    )
{
    TFuncName("TimerSet");
    TLevel(API);

    timer.expiredTime = nPgmTime + time;
    timer.sm = sm;
    timer.evtType = evtType;
    timer.flags |= TIMERF_ENABLED;

    TExit();
    return;
}   //TimerSet

/**
 *  This function resets the event timer.
 *
 *  @param timer Points to the TIMER structure.
 */
void
TimerReset(
    TIMER &timer
    )
{
    TFuncName("TimerReset");
    TLevel(API);

    timer.expiredTime = 0;
    timer.sm = NULL;
    timer.evtType = 0;
    timer.flags = 0;

    TExit();
    return;
}   //TimerReset

/**
 *  This function performs the timer task.
 *
 *  @param timer Points to the TIMER structure.
 */
void
TimerTask(
    TIMER &timer
    )
{
    TFuncName("TimerTask");
    TLevel(TASK);
    TEnter();

    if ((timer.flags & TIMERF_ENABLED) &&
        (nPgmTime >= timer.expiredTime))
    {
        timer.flags &= ~TIMERF_ENABLED;
        if (timer.sm != NULL)
        {
            SMSetEvent(*timer.sm, timer.evtType);
        }
    }

    TExit();
    return;
}   //TimerTask

#endif  //ifndef _TIMER_H
