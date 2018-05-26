#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="piddrive.h" />
///
/// <summary>
///     This module contains the library functions for the PID drive
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDDRIVE_H
#define _PIDDRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                          MOD_PIDDRIVE

//
// Constants.
//
#define PIDDRIVEF_PIDDRIVE_ON           0x0001
#define PIDDRIVEF_HOLD_TARGET           0x0002
#define PIDDRIVEF_TURN_ONLY             0x0004

//
// Macros.
//

/**
 *  This macro checks if PID drive is enabled.
 *
 *  @param m Points to the PIDDRIVE structure.
 *
 *  @return Returns true if PID drive is enabled.
 */
#define PIDDriveIsEnabled(p)            ((p).flags & PIDDRIVEF_PIDDRIVE_ON)

//
// Type definitions.
//
typedef struct
{
    DRIVE          *drive;
    PIDCTRL        *pidCtrlX;
    PIDCTRL        *pidCtrlY;
    PIDCTRL        *pidCtrlTurn;
    SM             *sm;
    int             evtType;
    int             flags;
    unsigned long   expiredTime;
#if defined(_TUNE_PID_X) || defined(_TUNE_PID_Y) || defined(_TUNE_PID_TURN)
    int            prevPower;
    unsigned long  oscStartTime;
#endif
} PIDDRIVE;

/**
 *  This function resets the PID Drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure to be reset.
 */
void
PIDDriveReset(
    PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveReset");
    TLevel(API);
    TEnter();

    DriveReset(*pidDrive.drive);
    if (pidDrive.pidCtrlX != NULL)
    {
        PIDCtrlReset(*pidDrive.pidCtrlX);
    }
    PIDCtrlReset(*pidDrive.pidCtrlY);
    PIDCtrlReset(*pidDrive.pidCtrlTurn);
    DriveStallProtect(*pidDrive.drive, false);
    pidDrive.sm = NULL;
    pidDrive.evtType = 0;
    pidDrive.expiredTime = 0;
    pidDrive.flags = 0;

    TExit();
    return;
}   //PIDDriveReset

/**
 *  This function initializes the drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param drive Specifies the DRIVE object.
 *  @param pidCtrlY Specifies the PID controller for the Y direction.
 *  @param pidCtrlTurn Specifies the PID controller for turn.
 */
void
PIDDriveInit(
    PIDDRIVE &pidDrive,
    DRIVE &drive,
    PIDCTRL &pidCtrlY,
    PIDCTRL &pidCtrlTurn
    )
{
    TFuncName("PIDDriveInit");
    TLevel(INIT);
    TEnter();

    pidDrive.drive = &drive;
    pidDrive.pidCtrlX = NULL;
    pidDrive.pidCtrlY = &pidCtrlY;
    pidDrive.pidCtrlTurn = &pidCtrlTurn;
    pidDrive.sm = NULL;
    pidDrive.evtType = 0;
    pidDrive.flags = 0;
#if defined(_TUNE_PID_X) || defined(_TUNE_PID_Y) || defined(_TUNE_PID_TURN)
    pidDrive.prevPower = 0;
    pidDrive.oscStartTime = 0;
#endif

    TExit();
    return;
}   //PIDDriveInit

/**
 *  This function initializes the drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param drive Specifies the DRIVE object.
 *  @param pidCtrlX Specifies the PID controller for the X direction.
 *  @param pidCtrlY Specifies the PID controller for the Y direction.
 *  @param pidCtrlTurn Specifies the PID controller for turn.
 */
void
PIDDriveInit(
    PIDDRIVE &pidDrive,
    DRIVE &drive,
    PIDCTRL &pidCtrlX,
    PIDCTRL &pidCtrlY,
    PIDCTRL &pidCtrlTurn
    )
{
    TFuncName("PIDDriveInit");
    TLevel(INIT);
    TEnter();

    PIDDriveInit(pidDrive, drive, pidCtrlY, pidCtrlTurn);
    pidDrive.pidCtrlX = &pidCtrlX;

    TExit();
    return;
}   //PIDDriveInit

/**
 *  This function sets PID drive target with the given drive distance and
 *  turn angle setpoints.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param distXSetPoint Specifies the target X distance to travel.
 *  @param distYSetPoint Specifies the target Y distance to travel.
 *  @param angleSetPoint Specifies the target angle to turn.
 *  @param fHoldTarget Optionally specifies if PIDDrive will maintain target
 *         or will stop when target is reached.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signaled when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout period.
 */
void
PIDDriveSetTarget(
    PIDDRIVE &pidDrive,
    float distXSetPoint,
    float distYSetPoint,
    float angleSetPoint,
    bool fHoldTarget = false,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("PIDDriveSetTarget");
    TLevel(API);
    TEnterMsg(("DY=%5.1f,A=%5.1f", distYSetPoint, angleSetPoint));

    pidDrive.sm = sm;
    pidDrive.evtType = evtType;
    if (pidDrive.pidCtrlX != NULL)
    {
        PIDCtrlSetTarget(*pidDrive.pidCtrlX,
                         distXSetPoint,
                         PIDCtrlGetInput(*pidDrive.pidCtrlX));
    }
    PIDCtrlSetTarget(*pidDrive.pidCtrlY,
                     distYSetPoint,
                     PIDCtrlGetInput(*pidDrive.pidCtrlY));
    PIDCtrlSetTarget(*pidDrive.pidCtrlTurn,
                     angleSetPoint,
                     PIDCtrlGetInput(*pidDrive.pidCtrlTurn));
    pidDrive.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;

    if (fHoldTarget)
    {
        pidDrive.flags |= PIDDRIVEF_HOLD_TARGET;
    }
    else
    {
        pidDrive.flags &= ~PIDDRIVEF_HOLD_TARGET;
    }

    if ((distXSetPoint == 0.0) &&
        (distYSetPoint == 0.0) &&
        (angleSetPoint != 0.0))
    {
        pidDrive.flags |= PIDDRIVEF_TURN_ONLY;
    }
    else
    {
        pidDrive.flags &= ~PIDDRIVEF_TURN_ONLY;
    }

#if defined(_TUNE_PID_X) || defined(_TUNE_PID_Y) || defined(_TUNE_PID_TURN)
    pidDrive.prevPower = 0;
    pidDrive.oscStartTime = nPgmTime;
#endif

    DriveStallProtect(*pidDrive.drive, true);
    pidDrive.flags |= PIDDRIVEF_PIDDRIVE_ON;

    TExit();
    return;
}   //PIDDriveSetTarget

/**
 *  This function sets PID drive target with the given drive distance and
 *  turn angle setpoints.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param distSetPoint Specifies the target Y distance to travel.
 *  @param angleSetPoint Specifies the target angle to turn.
 *  @param fHoldTarget Optionally specifies if PIDDrive will maintain target
 *         or will stop when target is reached.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signaled when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout period.
 */
void
PIDDriveSetTarget(
    PIDDRIVE &pidDrive,
    float distSetPoint,
    float angleSetPoint,
    bool fHoldTarget = false,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("PIDDriveSetTarget");
    TLevel(API);
    TEnterMsg(("D=%5.1f,A=%5.1f", distSetPoint, angleSetPoint));

    PIDDriveSetTarget(pidDrive,
                      0.0,
                      distSetPoint,
                      angleSetPoint,
                      fHoldTarget,
                      sm,
                      evtType,
                      timeout);

    TExit();
    return;
}   //PIDDriveSetTarget

/**
 *  This function performs the PID drive.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 */
void
PIDDriveTask(
    PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveTask");
    TLevel(TASK);
    TEnter();

    if (pidDrive.flags & PIDDRIVEF_PIDDRIVE_ON)
    {
        int driveXPower = ((pidDrive.flags & PIDDRIVEF_TURN_ONLY) ||
                           (pidDrive.pidCtrlX == NULL))?
                                0:
                                (int)PIDCtrlOutput(
                                        *pidDrive.pidCtrlX,
                                        PIDCtrlGetInput(*pidDrive.pidCtrlX));
        int driveYPower = (pidDrive.flags & PIDDRIVEF_TURN_ONLY)?
                                0:
                                (int)PIDCtrlOutput(
                                        *pidDrive.pidCtrlY,
                                        PIDCtrlGetInput(*pidDrive.pidCtrlY));
        int turnPower = (int)PIDCtrlOutput(
                                *pidDrive.pidCtrlTurn,
                                PIDCtrlGetInput(*pidDrive.pidCtrlTurn));

        if ((pidDrive.expiredTime != 0) &&
            (nPgmTime >= pidDrive.expiredTime) ||
            PIDCtrlIsOnTarget(*pidDrive.pidCtrlTurn) &&
            ((pidDrive.flags & PIDDRIVEF_TURN_ONLY) ||
             PIDCtrlIsOnTarget(*pidDrive.pidCtrlY) &&
             ((pidDrive.pidCtrlX == NULL) ||
              PIDCtrlIsOnTarget(*pidDrive.pidCtrlX))))
        {
            if (!(pidDrive.flags & PIDDRIVEF_HOLD_TARGET))
            {
                if (pidDrive.sm != NULL)
                {
                    SMSetEvent(*pidDrive.sm, pidDrive.evtType);
                }
                PIDDriveReset(pidDrive);
            }
        }

        if (pidDrive.flags & PIDDRIVEF_PIDDRIVE_ON)
        {
            //
            // PIDDrive is still ON. Either we haven't reached target yet
            // or we are holding target.
            //
            if (pidDrive.pidCtrlX != NULL)
            {
                DriveMecanumCartesian(*pidDrive.drive,
                                      driveXPower, driveYPower, turnPower);
            }
            else
            {
                DriveArcade(*pidDrive.drive, driveYPower, turnPower);
            }
        }
#if defined(_TUNE_PID_X) || defined(_TUNE_PID_Y) || defined(_TUNE_PID_TURN)
  #ifdef _TUNE_PID_X
        if ((pidDrive.prevPower > 0) && (driveXPower < 0))
  #endif
  #ifdef _TUNE_PID_Y
        if ((pidDrive.prevPower > 0) && (driveYPower < 0))
  #endif
  #ifdef _TUNE_PID_TURN
        if ((pidDrive.prevPower > 0) && (turnPower < 0))
  #endif
        {
            nxtDisplayTextLine(7, "Tu=%d", nPgmTime - pidDrive.oscStartTime);
            pidDrive.oscStartTime = nPgmTime;
        }
  #ifdef _TUNE_PID_X
        pidDrive.prevPower = driveXPower;
  #endif
  #ifdef _TUNE_PID_Y
        pidDrive.prevPower = driveYPower;
  #endif
  #ifdef _TUNE_PID_TURN
        pidDrive.prevPower = turnPower;
  #endif
#endif
    }

    TExit();
    return;
}   //PIDDriveTask

#endif  //ifndef _PIDDRIVE_H
