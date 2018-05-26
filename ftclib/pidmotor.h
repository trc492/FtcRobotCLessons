#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidmotor.h" />
///
/// <summary>
///     This module contains the library functions for the PID motor
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDMOTOR_H
#define _PIDMOTOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDMOTOR

//
// Constants.
//
#define PIDMOTORF_PIDMODE_ON    0x0001
#define PIDMOTORF_HOLD_TARGET   0x0002
#define PIDMOTORF_USE_2PIDCTRL  0x0004
#define PIDMOTORF_STALLED       0x0008

//
// Type definitions.
//
typedef struct
{
    tMotor          motorID;
    PIDCTRL        *pidCtrl1;
    PIDCTRL        *pidCtrl2;
    SM             *sm;
    int             evtType;
    int             flags;
    int             motorPower;
    unsigned long   expiredTime;
    int             prevEncoder;
    unsigned long   prevTime;
} PIDMOTOR;

/**
 *  This function resets the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure to be reset.
 */
void
PIDMotorReset(
    PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorReset");
    TLevel(API);
    TEnter();

    motor[pidMotor.motorID] = 0;
    if (pidMotor.pidCtrl1 != NULL)
    {
        PIDCtrlReset(*pidMotor.pidCtrl1);
    }

    if (pidMotor.pidCtrl2 != NULL)
    {
        PIDCtrlReset(*pidMotor.pidCtrl2);
    }

    pidMotor.sm = NULL;
    pidMotor.evtType = 0;
    pidMotor.motorPower = 0;
    pidMotor.expiredTime = 0;
    pidMotor.prevEncoder = 0;
    pidMotor.prevTime = 0;
    pidMotor.flags = 0;

    TExit();
    return;
}   //PIDMotorReset

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param motorID Specifies the motor ID.
 *  @param pidCtrl1 Specifies the PID controller 1.
 */
void
PIDMotorInit(
    PIDMOTOR &pidMotor,
    tMotor motorID,
    PIDCTRL &pidCtrl1
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    pidMotor.motorID = motorID;
    pidMotor.pidCtrl1 = &pidCtrl1;
    pidMotor.pidCtrl2 = NULL;
    pidMotor.sm = NULL;
    pidMotor.evtType = 0;
    pidMotor.motorPower = 0;
    pidMotor.expiredTime = 0;
    pidMotor.prevEncoder = 0;
    pidMotor.prevTime = 0;
    pidMotor.flags = 0;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param motorID Specifies the motor ID.
 *  @param pidCtrl1 Specifies the PID controller 1.
 *  @param pidCtrl2 Specifies the PID controller 2.
 */
void
PIDMotorInit(
    PIDMOTOR &pidMotor,
    tMotor motorID,
    PIDCTRL &pidCtrl1,
    PIDCTRL &pidCtrl2
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    PIDMotorInit(pidMotor, motorID, pidCtrl1);
    pidMotor.pidCtrl2 = &pidCtrl2;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function sets PID motor power.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param power Specifies the power level to set the motor.
 *  @param stallMinPower Optionally specifies stall detection minimum power.
 *  @param stallTimeout Optionally specifies stall detection timeout.
 *  @param resetTimeout Optionally specifies stall reset timeout.
 */
void
PIDMotorSetPower(
    PIDMOTOR &pidMotor,
    int power,
    int stallMinPower = 0,
    unsigned long stallTimeout = 0,
    unsigned long resetTimeout = 0
    )
{
    TFuncName("PIDMotorSetPower");
    TLevel(API);
    TEnterMsg(("power=%d", power));

    if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        //
        // There was a previous unfinished PID operation, cancel it.
        //
        PIDMotorReset(pidMotor);
    }

    power = BOUND(power, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (pidMotor.flags & PIDMOTORF_STALLED)
    {
        if (power == 0)
        {
            //
            // Clear the stall condition if power is zero.
            //
            if ((resetTimeout == 0) ||
                (nPgmTime - pidMotor.prevTime > resetTimeout))
            {
                pidMotor.prevEncoder = nMotorEncoder[pidMotor.motorID];
                pidMotor.prevTime = nPgmTime;
                pidMotor.flags &= ~PIDMOTORF_STALLED;
            }
        }
        else
        {
            //
            // Beep as long as power is still applied while stalled.
            //
            pidMotor.prevTime = nPgmTime;
            PlayImmediateTone(1000, 10);
        }
    }
    else
    {
        pidMotor.motorPower = power;
        motor[pidMotor.motorID] = power;
        if ((stallMinPower > 0) && (stallTimeout > 0))
        {
            //
            // Stall protection is ON, check for stall condition.
            // - power is above stallMinPower
            // - motor has not moved for at least stallTimeout
            //
            if ((abs(power) < abs(stallMinPower)) ||
                (nMotorEncoder[pidMotor.motorID] != pidMotor.prevEncoder))
            {
                pidMotor.prevEncoder = nMotorEncoder[pidMotor.motorID];
                pidMotor.prevTime = nPgmTime;
            }

            if (nPgmTime - pidMotor.prevTime > stallTimeout)
            {
                //
                // We have detected a stalled condition for at least
                // stallMinTime. Kill the power to protect the motor.
                //
                motor[pidMotor.motorID] = 0;
                pidMotor.flags |= PIDMOTORF_STALLED;
            }
        }
    }

    TExit()
    return;
}   //PIDMotorSetPower

/**
 *  This function performs zero point calibration.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param zeroLimitSW Specifies the touch sensor used as the limit switch.
 *  @param calPower Specifies the motor power for the calibration.
 *  @param stallTimeout Specifies the stall timeout value.
 */
void
PIDMotorZeroCalibrate(
    PIDMOTOR &pidMotor,
    TOUCH &zeroLimitSW,
    int calPower,
    unsigned long stallTimeout
    )
{
    TFuncName("PIDMotorZeroCalibrate");
    TLevel(API);
    TEnterMsg(("power=%d,timeout=%d", calPower, stallTimeout));

    if (TouchGetState(zeroLimitSW) == false)
    {
        unsigned long timeout = nPgmTime + stallTimeout;
        int prevEncoder = nMotorEncoder[pidMotor.motorID];
        int currEncoder;

        PIDMotorSetPower(pidMotor, calPower);
        while (TouchGetState(zeroLimitSW) == false)
        {
            currEncoder = nMotorEncoder[pidMotor.motorID];
            if (currEncoder != prevEncoder)
            {
                prevEncoder = currEncoder;
                timeout = nPgmTime + stallTimeout;
            }

            if (nPgmTime > timeout)
            {
                break;
            }

            wait1Msec(50);
        }
        PIDMotorReset(pidMotor);
    }
    nMotorEncoder[pidMotor.motorID] = 0;
    PlayImmediateTone(440, 15);

    TExit();
    return;
}   //PIDMotorZeroCalibrate

/**
 *  This function sets PID motor target.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param setPoint Specifies the set motor target.
 *  @param fHoldTarget Optionally specifies if PIDMotor will maintain target
 *         or will stop when target is reached.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signed when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout period.
 */
void
PIDMotorSetTarget(
    PIDMOTOR &pidMotor,
    float setPoint,
    bool fHoldTarget = false,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("PIDMotorSetTarget");
    TLevel(API);
    TEnterMsg(("setPt=%5.1f", setPoint));

    pidMotor.sm = sm;
    pidMotor.evtType = evtType;
    if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        //
        // Previous SetTarget has not been completed, cancel it.
        //
        PIDMotorReset(pidMotor);
    }

    PIDCtrlSetTarget(*pidMotor.pidCtrl1,
                     setPoint,
                     PIDCtrlGetInput(*pidMotor.pidCtrl1));
    pidMotor.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;

    if (fHoldTarget)
    {
        pidMotor.flags |= PIDMOTORF_HOLD_TARGET;
    }
    else
    {
        pidMotor.flags &= ~PIDMOTORF_HOLD_TARGET;
    }

    pidMotor.flags &= ~PIDMOTORF_USE_2PIDCTRL;
    pidMotor.flags |= PIDMOTORF_PIDMODE_ON;

    TExit();
    return;
}   //PIDMotorSetTarget

/**
 *  This function sets PID motor target.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param setPoint1 Specifies the set motor target 1.
 *  @param setPoint2 Specifies the set motor target 2.
 *  @param fHoldTarget Optionally specifies if PIDMotor will maintain target
 *         or will stop when target is reached.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signed when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout period.
 */
void
PIDMotorSetTarget(
    PIDMOTOR &pidMotor,
    float setPoint1,
    float setPoint2,
    bool fHoldTarget = false,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("PIDMotorSetTarget");
    TLevel(API);
    TEnterMsg(("SP1=%5.1f,SP2=%5.1f", setPoint1, setPoint2));

    pidMotor.sm = sm;
    pidMotor.evtType = evtType;
    if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        //
        // Previous SetTarget has not been completed, cancel it.
        //
        PIDMotorReset(pidMotor);
    }

    PIDCtrlSetTarget(*pidMotor.pidCtrl1,
                     setPoint1,
                     PIDCtrlGetInput(*pidMotor.pidCtrl1));
    if (setPoint2 != 0.0)
    {
        PIDCtrlSetTarget(*pidMotor.pidCtrl2,
                         setPoint2,
                         PIDCtrlGetInput(*pidMotor.pidCtrl2));
    }

    pidMotor.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;

    if (fHoldTarget)
    {
        pidMotor.flags |= PIDMOTORF_HOLD_TARGET;
    }
    else
    {
        pidMotor.flags &= ~PIDMOTORF_HOLD_TARGET;
    }

    if (setPoint2 != 0.0)
    {
        pidMotor.flags |= PIDMOTORF_USE_2PIDCTRL;
    }
    else
    {
        pidMotor.flags &= ~PIDMOTORF_USE_2PIDCTRL;
    }
    pidMotor.flags |= PIDMOTORF_PIDMODE_ON;

    TExit();
    return;
}   //PIDMotorSetTarget

/**
 *  This function performs the PID motor task.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 */
void
PIDMotorTask(
    PIDMOTOR &pidMotor
    )
{
    int output;

    TFuncName("PIDMotorTask");
    TLevel(TASK);
    TEnter();

    if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        int power1 = (int)PIDCtrlOutput(*pidMotor.pidCtrl1,
                                        PIDCtrlGetInput(*pidMotor.pidCtrl1));
        int power2 = 0;

        if (pidMotor.flags & PIDMOTORF_USE_2PIDCTRL)
        {
            power2 = (int)PIDCtrlOutput(*pidMotor.pidCtrl2,
                                        PIDCtrlGetInput(*pidMotor.pidCtrl2));
            pidMotor.motorPower = min(power1, power2);
        }
        else
        {
            pidMotor.motorPower = power1;
        }

        if ((pidMotor.expiredTime != 0) && (nPgmTime >= pidMotor.expiredTime) ||
            PIDCtrlIsOnTarget(*pidMotor.pidCtrl1))
        {
            if (!(pidMotor.flags & PIDMOTORF_HOLD_TARGET))
            {
                if (pidMotor.sm != NULL)
                {
                    SMSetEvent(*pidMotor.sm, pidMotor.evtType);
                }
                PIDMotorReset(pidMotor);
            }
        }

        if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
        {
            //
            // PID mode is still ON. Either we haven't rearched target yet
            // or we are holding target.
            //
            motor[pidMotor.motorID] = pidMotor.motorPower;
        }
    }

    TExit();
    return;
}   //PIDMotorTask

#endif  //ifndef _PIDMOTOR_H
