#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="wallfollow.h" />
///
/// <summary>
///     This module contains the library functions for a wall follower.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _WALLFOLLOW_H
#define _WALLFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif

#define MOD_ID                          MOD_WALLFOLLOW

//
// Constants.
//
#define WALLFOLLOWF_STARTED             0x0001
#define WALLFOLLOWF_LEFT                0x0002

#define WALLFOLLOW_DEF_DRIVE_POWER      50
#define WALLFOLLOW_DEF_FIND_DRIVE_POWER 50
#define WALLFOLLOW_DEF_FIND_TURN_POWER  30

#define WFMODE_FIND_WALL                0
#define WFMODE_AVOID_OBSTACLE           1
#define WFMODE_FOLLOW_WALL              2

//
// Macros.
//

/**
 *  This macro checks if the Wall Follower is started.
 *
 *  @param w Points to the WALLFOLLOW structure.
 *
 *  @return Returns true if wall following is started.
 */
#define WallFollowIsStarted(w)          ((w).flags & WALLFOLLOWF_STARTED)

//
// Type definitions.
//
typedef struct
{
    DRIVE      *drive;
    PIDCTRL    *pidCtrlFront;
    PIDCTRL    *pidCtrlSide;
    int         flags;
    int         mode;
    float       targetDist;
    int         drivePower;
    int         findDrivePower;
    int         findTurnPower;
} WALLFOLLOW;

/**
 *  This function initializes the wall follower object.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 *  @param drive Specifies the drive object.
 *  @param pidCtrlFront Specifies the PID controller for the front sonar.
 *  @param pidCtrlSide Specifies the PID controller for the side sonar.
 */
void
WallFollowInit(
    WALLFOLLOW &wallfollow,
    DRIVE &drive,
    PIDCTRL &pidCtrlFront,
    PIDCTRL &pidCtrlSide
    )
{
    int i;

    TFuncName("WallFollowInit");
    TLevel(INIT);
    TEnter();

    wallfollow.drive = &drive;
    wallfollow.pidCtrlFront = &pidCtrlFront;
    wallfollow.pidCtrlSide = &pidCtrlSide;
    wallfollow.flags = 0;
    wallfollow.mode = WFMODE_FIND_WALL;
    wallfollow.targetDist = 0.0;
    wallfollow.drivePower = WALLFOLLOW_DEF_DRIVE_POWER;
    wallfollow.findDrivePower = WALLFOLLOW_DEF_FIND_DRIVE_POWER;
    wallfollow.findTurnPower = WALLFOLLOW_DEF_FIND_TURN_POWER;

    TExit();
    return;
}   //WallFollowInit

/**
 *  This function starts the wall follower.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 *  @param targetDist Specifies the target distance from the wall.
 *  @param drivePower Optionally specifies the drive power for wall following.
 *  @param findDrivePower Optionally specifies the maximum drive power when
 *         finding the wall.
 *  @param findTurnPower Optionally specifies the maximum turn power when
 *         finding the wall.
 */
void
WallFollowStart(
    WALLFOLLOW &wallfollow,
    float targetDist,
    int drivePower = WALLFOLLOW_DEF_DRIVE_POWER,
    int findDrivePower = WALLFOLLOW_DEF_FIND_DRIVE_POWER,
    int findTurnPower = WALLFOLLOW_DEF_FIND_TURN_POWER,
    )
{
    TFuncName("WallFollowStart");
    TLevel(API);
    TEnterMsg(("target=%5.1f", targetDist));

    wallfollow.targetDist = abs(targetDist);
    if (targetDist < 0.0)
    {
        wallfollow.flags |= WALLFOLLOWF_LEFT;
    }
    else
    {
        wallfollow.flags &= ~WALLFOLLOWF_LEFT;
    }
    PIDCtrlSetTarget(*wallfollow.pidCtrlSide, wallfollow.targetDist, 0.0);
    wallfollow.drivePower = drivePower;
    wallfollow.findDrivePower = findDrivePower;
    wallfollow.findTurnPower = findTurnPower;
    wallfollow.flags |= WALLFOLLOWF_STARTED;

    TExit();
    return;
}   //WallFollowStart

/**
 *  This function stops the wall follower.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 */
void
WallFollowStop(
    WALLFOLLOW &wallfollow,
    )
{
    TFuncName("WallFollowStop");
    TLevel(API);
    TEnter();

    DriveReset(*wallfollow.drive);
    PIDCtrlReset(*wallfollow.pidCtrlFront);
    PIDCtrlReset(*wallfollow.pidCtrlSide);
    wallfollow.flags &= ~WALLFOLLOWF_STARTED;

    TExit();
    return;
}   //FollowStop

/**
 *  This function sets the find target drive and turn powers.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 *  @param drivePower Specifies the drive power for finding the target.
 *  @param turnPower Specifies the turn power for finding the target.
 */
void
WallFollowSetFindPower(
    WALLFOLLOW &wallfollow,
    int drivePower,
    int turnPower
    )
{
    TFuncName("SetFindPower");
    TLevel(TASK);
    TEnter();

    wallfollow.findDrivePower = drivePower;
    wallfollow.findTurnPower = turnPower;

    TExit();
    return;
}   //WallFollowSetFindPower

/**
 *  This function processes the wall follow task.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 */
void
WallFollowTask(
    WALLFOLLOW &wallfollow
    )
{
    TFuncName("WallFollowTask");
    TLevel(TASK);
    TEnter();

    if (WallFollowIsStarted(wallfollow))
    {
        float frontDist = PIDCtrlGetInput(*wallfollow.pidCtrlFront);
        float sideDist = PIDCtrlGetInput(*wallfollow.pidCtrlSide);

        switch (wallfollow.mode)
        {
            case WFMODE_FIND_WALL:
                if (frontDist <= wallfollow.targetDist)
                {
                    wallfollow.mode = WFMODE_AVOID_OBSTACLE;
                }
                else
                {
                    nxtDisplayTextLine(3, "FindWall");
                    DriveArcade(*wallfollow.drive,
                                wallfollow.findDrivePower,
                                0);
                }
                break;

            case WFMODE_AVOID_OBSTACLE:
                if (frontDist > wallfollow.targetDist*8.0)
                {
                    wallfollow.mode = WFMODE_FOLLOW_WALL;
                }
                else
                {
                    nxtDisplayTextLine(3, "AvoidObstacle");
                    DriveArcade(
                        *wallfollow.drive,
                        0,
                        (wallfollow.flags & WALLFOLLOWF_LEFT)?
                            wallfollow.findTurnPower:
                            -wallfollow.findTurnPower);
                }
                break;

            case WFMODE_FOLLOW_WALL:
                if (frontDist <= wallfollow.targetDist)
                {
                    wallfollow.mode = WFMODE_AVOID_OBSTACLE;
                }
                else
                {
                    nxtDisplayTextLine(3, "FollowWall");
                    int turnPower = (int)PIDCtrlOutput(
                                        *wallfollow.pidCtrlSide,
                                        sideDist);
                    if (wallfollow.flags & WALLFOLLOWF_LEFT)
                    {
                        DriveTank(*wallfollow.drive,
                                  wallfollow.drivePower,
                                  wallfollow.drivePower + turnPower);
                     }
                    else
                    {
                        DriveTank(*wallfollow.drive,
                                  wallfollow.drivePower + turnPower,
                                  wallfollow.drivePower);
                    }
                }
                break;
        }
    }

    TExit();
    return;
}   //WallFollowTask

#endif  //ifndef _WALLFOLLOW_H
