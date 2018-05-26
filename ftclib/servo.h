#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="servo.h" />
///
/// <summary>
///     This module contains the library functions for the servo subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SERVO_H
#define _SERVO_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SERVO

//
// Constants.
//
#define SERVOO_INVERSE_SERVO2           0x0001

#define SERVOF_CONTINUOUS_ENABLED       0x0001
#define SERVOF_CONTINUOUS               0x0002
#define SERVOF_STEPPING                 0x0004

#define SERVO_MIN_VALUE                 0
#define SERVO_MAX_VALUE                 255

#define SERVO_CONTINUOUS_STOP           127
#define SERVO_CONTINUOUS_FWD_MAX        255
#define SERVO_CONTINUOUS_REV_MAX        0
#define SERVO_CONTINUOUS_LOW            -127
#define SERVO_CONTINUOUS_HIGH           128

#ifndef SERVO_ANGLE_RANGE
    #define SERVO_ANGLE_RANGE           180.0
#endif
#define SERVO_POS_PER_DEGREE    ((SERVO_MAX_VALUE - SERVO_MIN_VALUE)/   \
                                  SERVO_ANGLE_RANGE)

//
// Macros.
//

/**
 *  This macro returns the current servo angle if it is in stepping mode.
 *  Otherwise, it returns zero.
 *
 *  @param s Points to the SERVO structure.
 *
 *  @return Returns the current servo angle if in stepping mode.
 */
#define ServoGetAngle(s)        (((s).flags & SERVOF_CONTINUOUS)?       \
                                 0.0: (s).currAngle)

/**
 *  This macro returns the current power if it is a continuous servo.
 *
 *  @param s Points to the SERVO structure.
 *
 *  @return Returns the current power if it is a continuous servo.
 */
#define ServoGetPower(s)        (((s).flags & SERVOF_CONTINUOUS)?       \
                                 (s).power: 0)

//
// Type definitions.
//
typedef struct
{
    TServoIndex     servoMotor1;
    TServoIndex     servoMotor2;
    int             options;
    int             flags;
    tSensors        lowerTouchID;
    tSensors        upperTouchID;
    int             power;
    float           posPerDegree;
    float           targetAngle;
    float           stepRate;
    float           currAngle;
    unsigned long   prevTime;
} SERVO;

/**
 *  This function initializes a standard servo motor.
 *
 *  @param serv Points to the SERVO structure.
 *  @param servoMotor Specifies the servo motor ID.
 *  @param initDegrees Specifies the initial position in degrees.
 *  @param options Optionally specifies the servo options:
 *         SERVOO_INVERSE_SERVO2 - Servo 2 is inverse of servo 1.
 *  @param posPerDegree Optionally specifies the position value per degree.
 */
void
ServoInit(
    SERVO &serv,
    TServoIndex servoMotor,
    float initDegrees,
    int options = 0,
    float posPerDegree = SERVO_POS_PER_DEGREE
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    serv.servoMotor1 = servoMotor;
    serv.servoMotor2 = (TServoIndex)-1;
    serv.options = options;
    serv.flags = 0;
    serv.lowerTouchID = (tSensors)-1;
    serv.upperTouchID = (tSensors)-1;
    serv.power = SERVO_CONTINUOUS_STOP;
    serv.posPerDegree = posPerDegree;
    serv.targetAngle = 0.0;
    serv.stepRate = 0.0;
    serv.currAngle = initDegrees;
    serv.prevTime = 0;
    servo[servoMotor] = (int)(initDegrees*posPerDegree);

    TExit();
    return;
}   //ServoInit

/**
 *  This function initializes a standard servo motor.
 *
 *  @param serv Points to the SERVO structure.
 *  @param servoMotor1 Specifies the motor ID of the first servo.
 *  @param servoMotor2 Specifies the motor ID of the second servo.
 *  @param initDegrees Specifies the initial position in degrees.
 *  @param options Optionally specifies the servo options:
 *         SERVOO_INVERSE_SERVO2 - Servo 2 is inverse of servo 1.
 *  @param posPerDegree Optionally specifies the position value per degree.
 */
void
ServoInit(
    SERVO &serv,
    TServoIndex servoMotor1,
    TServoIndex servoMotor2,
    float initDegrees,
    int options = 0,
    float posPerDegree = SERVO_POS_PER_DEGREE
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    ServoInit(serv, servoMotor1, initDegrees, options, posPerDegree);
    serv.servoMotor2 = servoMotor2;

    if (servoMotor2 != (TServoIndex)-1)
    {
        int servoPos = (int)(initDegrees*posPerDegree);

        servo[servoMotor2] = (options & SERVOO_INVERSE_SERVO2)?
                                SERVO_MAX_VALUE - servoPos: servoPos;
    }

    TExit();
    return;
}   //ServoInit

/**
 *  This function initializes a continuous servo motor.
 *
 *  @param serv Points to the SERVO structure.
 *  @param servoMotor Specifies the servo motor ID.
 *  @param lowerTouchID Optionally specifies the sensor ID for the lower
 *         limit switch.
 *  @param upperTouchID Optionally specifies the sensor ID for the upper
 *         limit switch.
 */
void
ServoInit(
    SERVO &serv,
    TServoIndex servoMotor,
    tSensors lowerTouchID = -1,
    tSensors upperTouchID = -1
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    serv.servoMotor1 = servoMotor;
    serv.servoMotor2 = (TServoIndex)(-1);
    serv.options = 0;
    serv.flags = SERVOF_CONTINUOUS;
    serv.lowerTouchID = lowerTouchID;
    serv.upperTouchID = upperTouchID;
    serv.power = SERVO_CONTINUOUS_STOP;
    serv.posPerDegree = 0.0;
    serv.targetAngle = 0.0;
    serv.stepRate = 0.0;
    serv.currAngle = 0.0;
    serv.prevTime = 0;
    servo[servoMotor] = serv.power;

    TExit();
    return;
}   //ServoInit

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param angle Specifies the servo position in degrees.
 *  @param stepRate Specifies the step rate in degrees/sec.
 */
void
ServoSetAngle(
    SERVO &serv,
    float angle,
    float stepRate = 0.0
    )
{
    TFuncName("ServoSetPos");
    TLevel(API);
    TEnterMsg(("A=%5.1f,R=%5.1f", angle, stepRate));

    if (!(serv.flags & SERVOF_CONTINUOUS))
    {
        serv.stepRate = abs(stepRate);
        if (stepRate == 0.0)
        {
            serv.currAngle = angle;
            serv.flags &= ~SERVOF_STEPPING;
        }
        else
        {
            serv.prevTime = nPgmTime;
            serv.targetAngle = angle;
            serv.flags |= SERVOF_STEPPING;
        }
    }

    TExit();
    return;
}   //ServoSetAngle

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param power Specifies the continuous servo power.
 */
void
ServoContinuousSetPower(
    SERVO &serv,
    int power
    )
{
    TFuncName("ServoContSetPower");
    TLevel(API);
    TEnterMsg(("power=%5.1f", power));

    if (serv.flags & SERVOF_CONTINUOUS)
    {
        serv.power = NORMALIZE(BOUND(power, -100, 100),
                               -100, 100,
                               SERVO_MIN_VALUE, SERVO_MAX_VALUE);

        servo[serv.servoMotor1] = serv.power;
        serv.flags |= SERVOF_CONTINUOUS_ENABLED;
    }

    TExit();
    return;
}   //ServoContinuousSetPower

/**
 *  This function stops the servo and hold its position.
 *
 *  @param serv Points to the SERVO structure.
 */
void
ServoStop(
    SERVO &serv
    )
{
    TFuncName("ServoStop");
    TLevel(API);
    TEnter();

    if (serv.flags & SERVOF_CONTINUOUS)
    {
        serv.power = SERVO_CONTINUOUS_STOP;
        servo[serv.servoMotor1] = serv.power;
        serv.flags &= ~SERVOF_CONTINUOUS_ENABLED;
    }
    else if (serv.flags & SERVOF_STEPPING)
    {
        serv.flags &= ~SERVOF_STEPPING;
    }

    TExit();
    return;
}   //ServoStop

/**
 *  This function performs the servo task.
 *
 *  @param serv Points to the SERVO structure.
 */
void
ServoTask(
    SERVO &serv
    )
{
    TFuncName("ServoTask");
    TLevel(TASK);
    TEnter();

    if (serv.flags & SERVOF_CONTINUOUS_ENABLED)
    {
        if ((serv.lowerTouchID != -1) &&
            (SensorValue[serv.lowerTouchID] == 1) ||
            (serv.upperTouchID != -1) &&
            (SensorValue[serv.upperTouchID] == 1))
        {
            servo[serv.servoMotor1] = SERVO_CONTINUOUS_STOP;
            serv.flags &= ~SERVOF_CONTINUOUS_ENABLED;
        }
    }
    else
    {
        if (serv.flags & SERVOF_STEPPING)
        {
            unsigned long currTime = nPgmTime;
            float stepAngle = serv.stepRate*(currTime - serv.prevTime)/1000.0;

            if (serv.currAngle < serv.targetAngle)
            {
                serv.currAngle += stepAngle;
                if (serv.currAngle > serv.targetAngle)
                {
                    serv.currAngle = serv.targetAngle;
                }
            }
            else if (serv.currAngle > serv.targetAngle)
            {
                serv.currAngle -= stepAngle;
                if (serv.currAngle < serv.targetAngle)
                {
                    serv.currAngle = serv.targetAngle;
                }
            }
            else
            {
                //
                // We have reached the target angle, stop stepping.
                //
                serv.flags &= ~SERVOF_STEPPING;
            }

            serv.prevTime = currTime;
        }

        int servoPos = (int)(serv.currAngle*serv.posPerDegree);

        if (servoPos > SERVO_MAX_VALUE)
        {
            servoPos = SERVO_MAX_VALUE;
        }
        else if (servoPos < SERVO_MIN_VALUE)
        {
            servoPos = SERVO_MIN_VALUE;
        }

        servo[serv.servoMotor1] = servoPos;

        if (serv.servoMotor2 != (TServoIndex)(-1))
        {
            servo[serv.servoMotor2] =
                (serv.options & SERVOO_INVERSE_SERVO2)?
                    SERVO_MAX_VALUE - servoPos: servoPos;
        }
    }

    TExit();
    return;
}   //ServoTask

#endif  //ifndef _SERVO_H
