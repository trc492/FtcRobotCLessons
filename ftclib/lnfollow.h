#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="lnfollow.h" />
///
/// <summary>
///     This module contains the library functions for a line follower.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _LNFOLLOW_H
#define _LNFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif

#define MOD_ID                          MOD_LNFOLLOW

//
// Constants.
//
#define LNFOLLOWF_STARTED               0x0001

#define LNFOLLOW_DEF_MAX_DRIVE_POWER    100
#define LNFOLLOW_DEF_FIND_DRIVE_POWER   50
#define LNFOLLOW_DEF_FIND_TURN_POWER    50

#define LNFOLLOW_NOLINE                 0.0
#define LNFOLLOW_INVALID_VALUE          -1.0

//
// Macors.
//

/**
 *  This macro checks if the Line Follower is started.
 *
 *  @param l Points to the LNFOLLOW object.
 *
 *  @return Returns true if line follower is started.
 */
#define LnFollowStarted(l)      ((l).flags & LNFOLLOWF_STARTED)

//
// Type definitions.
//
typedef struct
{
    DRIVE          *drive;
    PIDCTRL        *pidCtrl;
    SM             *sm;
    int             evtType;
    int             flags;
    int             maxDrivePower;
    int             findDrivePower;
    int             findTurnPower;
    unsigned long   expiredTime;
} LNFOLLOW;

/**
 *  This function initializes the line follower object.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param drive Points to the DRIVE object.
 *  @param pidCtrl Points to the PIDCTRL object.
 */
void
LnFollowInit(
    LNFOLLOW &lnfollow,
    DRIVE &drive,
    PIDCTRL &pidCtrl
    )
{
    TFuncName("LnFollowInit");
    TLevel(INIT);
    TEnter();

    lnfollow.drive = &drive;
    lnfollow.pidCtrl = &pidCtrl;
    lnfollow.sm = NULL;
    lnfollow.evtType = 0;
    lnfollow.flags = 0;
    lnfollow.maxDrivePower = LNFOLLOW_DEF_MAX_DRIVE_POWER;
    lnfollow.findDrivePower = LNFOLLOW_DEF_FIND_DRIVE_POWER;
    lnfollow.findTurnPower = LNFOLLOW_DEF_FIND_TURN_POWER;
    lnfollow.expiredTime = 0;

    TExit();
    return;
}   //LnFollowInit

/**
 *  This function starts the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param targetValue Specifies the center sensor value.
 *  @param maxDrivePower Specifies the maximum drive power.
 *  @param findDrivePower Specifies the maximum drive power when finding
 *         the target.
 *  @param findTurnPower Specifies the maximum turn power when finding
 *         the target.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signed when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout value.
 */
void
LnFollowStart(
    LNFOLLOW &lnfollow,
    float targetValue,
    int maxDrivePower,
    int findDrivePower,
    int findTurnPower,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("LnFollowStart");
    TLevel(API);
    TEnterMsg(("target=%5.1f", targetValue));

    lnfollow.maxDrivePower = maxDrivePower;
    lnfollow.findDrivePower = findDrivePower;
    lnfollow.findTurnPower = findTurnPower;
    lnfollow.sm = sm;
    lnfollow.evtType = evtType;
    lnfollow.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;
    PIDCtrlSetTarget(*lnfollow.pidCtrl, targetValue, 0.0);
    lnfollow.flags |= LNFOLLOWF_STARTED;

    TExit();
    return;
}   //LnFollowStart

/**
 *  This function stops the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowStop(
    LNFOLLOW &lnfollow,
    )
{
    TFuncName("LnFollowStop");
    TLevel(API);
    TEnter();

    DriveReset(*lnfollow.drive);
    PIDCtrlReset(*lnfollow.pidCtrl);
    lnfollow.sm = NULL;
    lnfollow.evtType = 0;
    lnfollow.expiredTime = 0;
    lnfollow.flags &= ~LNFOLLOWF_STARTED;

    TExit();
    return;
}   //LnFollowStop

/**
 *  This function sets the find target drive and turn powers.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param drivePower Specifies the drive power for finding the target.
 *  @param turnPower Specifies the turn power for finding the target.
 */
void
LnFollowSetFindPower(
    LNFOLLOW &lnfollow,
    int drivePower,
    int turnPower
    )
{
    TFuncName("SetFindPower");
    TLevel(TASK);
    TEnter();

    lnfollow.findDrivePower = drivePower;
    lnfollow.findTurnPower = turnPower;

    TExit();
    return;
}   //LnFollowSetFindPower

/**
 *  This function processes the line follower task.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowTask(
    LNFOLLOW &lnfollow
    )
{
    TFuncName("LnFollowTask");
    TLevel(TASK);
    TEnter();

    float currValue = PIDCtrlGetInput(*lnfollow.pidCtrl);
    //
    // PIDCtrlGetInput() may determine the stopping condition and
    // call LnFollowStop().
    //
    if (LnFollowStarted(lnfollow))
    {
        if ((lnfollow.expiredTime != 0) && (nPgmTime >= lnfollow.expiredTime))
        {
            //
            // We run out of time, so stop.
            //
            if (lnfollow.sm != NULL)
            {
                SMSetEvent(*lnfollow.sm, lnfollow.evtType);
            }
            LnFollowStop(lnfollow);
        }
        else if (currValue != LNFOLLOW_NOLINE)
        {
            int turnPower = (int)PIDCtrlOutput(*lnfollow.pidCtrl, currValue);
            int drivePower = lnfollow.maxDrivePower*
                             (MOTOR_MAX_VALUE - abs(turnPower))/
                             MOTOR_MAX_VALUE;
            DriveArcade(*lnfollow.drive, drivePower, turnPower);
        }
        else
        {
            //
            // We lost the line. Find it again.
            //
            DriveArcade(*lnfollow.drive,
                        lnfollow.findDrivePower,
                        lnfollow.findTurnPower);
        }
    }

    TExit();
    return;
}   //LnFollowTask

#endif  //ifndef _LNFOLLOW_H
