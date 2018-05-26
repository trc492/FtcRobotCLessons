#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="drive.h" />
///
/// <summary>
///     This module contains the library functions for the drive subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DRIVE_H
#define _DRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVE

//
// Constants.
//
#define DRIVEF_STATE_MASK       0x00ff
#define DRIVEF_ON               0x0001
#define DRIVEF_STALLED          0x0002
#define DRIVEF_STALL_PROTECT_ON 0x0100
#define DRIVEF_FOUR_MOTORS      0x0200
#define DRIVEF_SERVO_MOTORS     0x0400

#define DRIVEO_SERVO_REVERSED   0x0001

#define DRIVE_MIN_STALL_POWER   20
#define DRIVE_STALL_TIME        2000    //2 seconds

#define MAX_NUM_MOTORS          4
#define IDX_FRONT_LEFT          0
#define IDX_FRONT_RIGHT         1
#define IDX_REAR_LEFT           2
#define IDX_REAR_RIGHT          3

#define DEF_POS_PER_DEGREE      (255.0/180.0)
#define DEF_LEFT_ANGLELIMIT     -90.0
#define DEF_RIGHT_ANGLELIMIT    90.0

//
// Macros.
//

/**
 *  This macro checks if the drive subsystem is stalled.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns true if detected stall condition.
 */
#define DriveIsStalled(d)       ((d).flags & DRIVEF_STALLED)

//
// Type definitions.
//
typedef struct
{
    int             options;
    int             flags;
    tMotor          motorIDs[MAX_NUM_MOTORS];
    TServoIndex     servoIDs[MAX_NUM_MOTORS];
    float           posPerDegree;
    float           leftAngleLimit;
    float           rightAngleLimit;
    int             motorPowers[MAX_NUM_MOTORS];
    int             motorEncoders[MAX_NUM_MOTORS];
    unsigned long   stallTimer;
} DRIVE;

/**
 *  This function sets the servo to the given angle.
 *
 *  @param drive Points to the DRIVE structure to be reset.
 *  @param servoID Specifies the servo ID.
 *  @param angle Specifies the angle to set the servo to.
 */
void
SetServoAngle(
    DRIVE      &drive,
    TServoIndex servoID,
    float       angle
    )
{
    TFuncName("SetServoAngle");
    TLevel(FUNC);
    TEnterMsg(("ID=%d,angle=%f", servoID, angle));

    int servoPos;

    angle = BOUND(angle, drive.leftAngleLimit, drive.rightAngleLimit);
    angle += 90.0;
    servoPos = (int)(angle*drive.posPerDegree);
    if (drive.options & DRIVEO_SERVO_REVERSED)
    {
        servoPos = 255 - servoPos;
    }
    servo[servoID] = servoPos;

    TExit();
    return;
}   //SetServoAngle

/**
 *  This function resets the drive system.
 *
 *  @param drive Points to the DRIVE structure to be reset.
 */
void
DriveReset(
    DRIVE &drive
    )
{
    TFuncName("DriveReset");
    TLevel(API);
    TEnter();

    int numMotors = (drive.flags & DRIVEF_FOUR_MOTORS)? 4: 2;

    //
    // Stop the motors.
    //
    for (int i = 0; i < numMotors; i++)
    {
        drive.motorPowers[i] = 0;
        drive.motorEncoders[i] = 0;
        motor[drive.motorIDs[i]] = 0;

        if (drive.flags & DRIVEF_SERVO_MOTORS)
        {
            //
            // Set servos to forward angle.
            //
            SetServoAngle(drive, drive.servoIDs[i], 0.0);
        }
    }
    drive.stallTimer = 0;
    drive.flags &= ~DRIVEF_STATE_MASK;

    TExit();
    return;
}   //DriveReset

/**
 *  This function initializes the drive system for 2-motor drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param leftMotor Specifies the left motor.
 *  @param rightMotor Specifies the right motor.
 */
void
DriveInit(
    DRIVE &drive,
    tMotor leftMotor,
    tMotor rightMotor
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    for (int i = 0; i < MAX_NUM_MOTORS; i++)
    {
        drive.motorIDs[i] = (tMotor)0;
        drive.servoIDs[i] = (TServoIndex)0;
        drive.motorPowers[i] = 0;
        drive.motorEncoders[i] = 0;
    }
    drive.motorIDs[IDX_FRONT_LEFT] = leftMotor;
    drive.motorIDs[IDX_FRONT_RIGHT] = rightMotor;
    drive.posPerDegree = DEF_POS_PER_DEGREE;
    drive.leftAngleLimit = DEF_LEFT_ANGLELIMIT;
    drive.rightAngleLimit = DEF_RIGHT_ANGLELIMIT;
    drive.stallTimer = 0;
    drive.flags = 0;
    drive.options = 0;

    TExit();
    return;
}   //DriveInit

/**
 *  This function initializes the drive system for 4-motor drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param frontLeftMotor Specifies the front left motor.
 *  @param frontRightMotor Specifies the front right motor.
 *  @param rearLeftMotor Specifies the rear left motor.
 *  @param rearRightMotor Specifies the rear right motor.
 */
void
DriveInit(
    DRIVE &drive,
    tMotor frontLeftMotor,
    tMotor frontRightMotor,
    tMotor rearLeftMotor,
    tMotor rearRightMotor
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    DriveInit(drive, frontLeftMotor, frontRightMotor);
    drive.motorIDs[IDX_REAR_LEFT] = rearLeftMotor;
    drive.motorIDs[IDX_REAR_RIGHT] = rearRightMotor;
    drive.flags |= DRIVEF_FOUR_MOTORS;

    TExit();
    return;
}   //DriveInit

/**
 *  This function initializes the drive system for 4-motor and 4-servo drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param frontLeftMotor Specifies the front left motor.
 *  @param frontRightMotor Specifies the front right motor.
 *  @param rearLeftMotor Specifies the rear left motor.
 *  @param rearRightMotor Specifies the rear right motor.
 *  @param frontLeftServo Specifies the front left servo motor.
 *  @param frontRightServo Specifies the front right servo motor.
 *  @param rearLeftServo Specifies the rear left servo motor.
 *  @param rearRightServo Specifies the rear right servo motor.
 *  @param posPerDegree Optionally specifies the position value per degree.
 *  @param leftAngleLimit Optionally specifies the left servo angle limit.
 *  @param rightAngleLimit Optionally specifies the right servo angle limit.
 *  @param options Optionally specifies the drive options:
 *         DRIVEO_SERVO_REVERSED - Specifes the servo direction is reversed.
 */
void
DriveInit(
    DRIVE      &drive,
    tMotor      frontLeftMotor,
    tMotor      frontRightMotor,
    tMotor      rearLeftMotor,
    tMotor      rearRightMotor,
    TServoIndex frontLeftServo,
    TServoIndex frontRightServo,
    TServoIndex rearLeftServo,
    TServoIndex rearRightServo,
    float       posPerDegree = DEF_POS_PER_DEGREE,
    float       leftAngleLimit = DEF_LEFT_ANGLELIMIT,
    float       rightAngleLimit = DEF_RIGHT_ANGLELIMIT,
    int         options = 0
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    DriveInit(drive, frontLeftMotor, frontRightMotor,
              rearLeftMotor, rearRightMotor);
    drive.servoIDs[IDX_FRONT_LEFT] = frontLeftServo;
    drive.servoIDs[IDX_FRONT_RIGHT] = frontRightServo;
    drive.servoIDs[IDX_REAR_LEFT] = rearLeftServo;
    drive.servoIDs[IDX_REAR_RIGHT] = rearRightServo;
    drive.posPerDegree = posPerDegree;
    drive.leftAngleLimit = leftAngleLimit;
    drive.rightAngleLimit = rightAngleLimit;
    drive.flags |= DRIVEF_SERVO_MOTORS;
    drive.options = options;

    for (int i = 0; i < MAX_NUM_MOTORS; i++)
    {
        SetServoAngle(drive, drive.servoIDs[i], 0.0);
    }

    TExit();
    return;
}   //DriveInit

/**
 *  This function enables or disables stall protection.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param fOn If true, enables stall protection.
 */
void
DriveStallProtect(
    DRIVE &drive,
    bool fOn
    )
{
    TFuncName("DriveStallProtect");
    TLevel(API);
    TEnterMsg(("fOn=%d", (byte)fOn));

    if (fOn)
    {
        drive.flags |= DRIVEF_STALL_PROTECT_ON;
    }
    else
    {
        drive.flags &= ~DRIVEF_STALL_PROTECT_ON;
    }

    TExit();
    return;
}   //DriveStallProtect

/**
 *  This function sets power of the motors for tank drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param leftPower Specifies the left motor power.
 *  @param rightPower Specifies the right motor power.
 */
void
DriveTank(
    DRIVE &drive,
    int leftPower,
    int rightPower
    )
{
    TFuncName("DriveTank");
    TLevel(API);
    TEnterMsg(("Left=%d,Right=%d", leftPower, rightPower));

    drive.motorPowers[IDX_FRONT_LEFT] =
        BOUND(leftPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    drive.motorPowers[IDX_FRONT_RIGHT] =
        BOUND(rightPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (drive.flags & DRIVEF_FOUR_MOTORS)
    {
        drive.motorPowers[IDX_REAR_LEFT] =
            drive.motorPowers[IDX_FRONT_LEFT];
        drive.motorPowers[IDX_REAR_RIGHT] =
            drive.motorPowers[IDX_FRONT_RIGHT];
    }
    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveTank

/**
 *  This function sets power of the motors for arcade drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param drivePower Specifies the drive power.
 *  @param turnPower Specifies the turn power.
 */
void
DriveArcade(
    DRIVE &drive,
    int drivePower,
    int turnPower
    )
{
    TFuncName("DriveArcade");
    TLevel(API);
    TEnterMsg(("Drive=%d,Turn=%d", drivePower, turnPower));

    drivePower = BOUND(drivePower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    turnPower = BOUND(turnPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (drivePower + turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward right:
        //  left = drive + turn - (drive + turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MAX_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = MOTOR_MAX_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = -2*turnPower + MOTOR_MAX_VALUE;
    }
    else if (drivePower - turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward left:
        //  left = drive + turn - (drive - turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MAX_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = 2*turnPower + MOTOR_MAX_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = MOTOR_MAX_VALUE;
    }
    else if (drivePower + turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward left:
        //  left = drive + turn - (drive + turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MIN_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = MOTOR_MIN_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = -2*turnPower + MOTOR_MIN_VALUE;
    }
    else if (drivePower - turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward right:
        //  left = drive + turn - (drive - turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MIN_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = 2*turnPower + MOTOR_MIN_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = MOTOR_MIN_VALUE;
    }
    else
    {
        drive.motorPowers[IDX_FRONT_LEFT] = drivePower + turnPower;
        drive.motorPowers[IDX_FRONT_RIGHT] = drivePower - turnPower;
    }

    if (drive.flags & DRIVEF_FOUR_MOTORS)
    {
        drive.motorPowers[IDX_REAR_LEFT] =
            drive.motorPowers[IDX_FRONT_LEFT];
        drive.motorPowers[IDX_REAR_RIGHT] =
            drive.motorPowers[IDX_FRONT_RIGHT];
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveArcade

/**
 *  This function sets power of the motors for mecanum drive in cartesian
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param x Specifies the x speed.
 *  @param y Specifies the y speed.
 *  @param rot Specifies the rotaton speed.
 */
void
DriveMecanumCartesian(
    DRIVE &drive,
    int x,
    int y,
    int rot
    )
{
    TFuncName("MecanumCartesian");
    TLevel(API);
    TEnterMsg(("x=%d,y=%d,rot=%d", x, y, rot));

    if (drive.flags & DRIVEF_FOUR_MOTORS)
    {
        int mag, maxMag, i;

        drive.motorPowers[IDX_FRONT_LEFT] = x + y + rot;
        drive.motorPowers[IDX_FRONT_RIGHT] = -x + y - rot;
        drive.motorPowers[IDX_REAR_LEFT] = -x + y + rot;
        drive.motorPowers[IDX_REAR_RIGHT] = x + y - rot;
        //
        // Normalize
        //
        maxMag = abs(drive.motorPowers[0]);
        for (i = 1; i < MAX_NUM_MOTORS; i++)
        {
            mag = abs(drive.motorPowers[i]);
            if (mag > maxMag)
            {
                maxMag = mag;
            }
        }

        if (maxMag > MOTOR_MAX_VALUE)
        {
            for (i = 0; i < MAX_NUM_MOTORS; i++)
            {
                drive.motorPowers[i] =
                    (drive.motorPowers[i]*MOTOR_MAX_VALUE)/maxMag;
            }
        }
    }
    else
    {
        //
        // Mecanum drive is only possible with 4 motors. For 2 motors, we
        // do arcade drive instead.
        //
        DriveArcade(drive, y, rot);
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveMecanumCartesian

/**
 *  This function sets power of the motors for mecanum drive in polar
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param mag Specifies the magnitude.
 *  @param dir Specifies the direction.
 *  @param rot Specifies the rotaton.
 */
void
DriveMecanumPolar(
    DRIVE &drive,
    int mag,
    int dir,
    int rot
    )
{
    TFuncName("MecanumPolar");
    TLevel(API);
    TEnterMsg(("m=%d,d=%d,r=%d", mag, dir, rot));

    if (drive.flags & DRIVEF_FOUR_MOTORS)
    {
        float magnitude = (BOUND(mag, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)*
                           sqrt(2.0))/MOTOR_MAX_VALUE;
        float rotation = (float)BOUND(rot, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)/
                         MOTOR_MAX_VALUE;
        float dirInRad = (dir + 45.0)*PI/180.0;
        float sinD = sin(dirInRad);
        float cosD = cos(dirInRad);
        float powers[MAX_NUM_MOTORS];

        powers[IDX_FRONT_LEFT] = sinD*magnitude + rotation;
        powers[IDX_FRONT_RIGHT] = cosD*magnitude - rotation;
        powers[IDX_REAR_LEFT] = cosD*magnitude + rotation;
        powers[IDX_REAR_RIGHT] = sinD*magnitude - rotation;
        //
        // Normalize
        //
        float maxMag = abs(powers[0]);
        float mag;
        int i;

        for (i = 1; i < MAX_NUM_MOTORS; i++)
        {
            mag = abs(powers[i]);
            if (mag > maxMag)
            {
                maxMag = mag;
            }
        }

        for (i = 0; i < MAX_NUM_MOTORS; i++)
        {
            if (maxMag > 1.0)
            {
                drive.motorPowers[i] = (int)
                    (powers[i]*MOTOR_MAX_VALUE/maxMag);
            }
            else
            {
                drive.motorPowers[i] = (int)(powers[i]*MOTOR_MAX_VALUE);
            }
        }
    }
    else
    {
        //
        // Mecanum drive is only possible with 4 motors. For 2 motors, we
        // do arcade drive instead.
        //
        DriveArcade(drive, mag, rot);
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveMecanumPolar

/**
 *  This function sets power of the motors for swerve drive in cartesian
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param x Specifies the x speed.
 *  @param y Specifies the y speed.
 *  @param rot Specifies the rotaton speed.
 */
void
DriveSwerve(
    DRIVE &drive,
    int x,
    int y,
    int rot
    )
{
    TFuncName("DriveSwerve");
    TLevel(API);
    TEnterMsg(("x=%d,y=%d,rot=%d", x, y, rot));

    int mag;
    float angle;
    int yDir;

    x = BOUND(x, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    y = BOUND(y, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    rot = BOUND(rot, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    //
    // Calculate the magnitude and scale to a maximum of MAX_MOTOR_VALUE.
    //
    yDir = (y >= 0)? 1: -1;
    mag = yDir*BOUND((int)sqrt(x*x + y*y), MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (mag == 0)
    {
        angle = 0.0;
    }
    else
    {
        angle = 90.0 - atan2(y, x)*180.0/PI;
        if (angle > 90.0)
        {
            angle -= 180.0;
        }
    }

    if ((mag == 0) && (rot != 0))
    {
        //
        // We are doing rotate only. We will limit the rotate angle to a fixed
        // 45-degree and driving wheel power proportional to the rotation.
        // We must apply some power to the wheel motors because, first of all,
        // the user is expecting the robot to turn. Without wheel power,
        // the robot won't turn. Secondly, with the robot not moving,
        // the friction on the wheels may be too big to even turn the servos.
        //
        SetServoAngle(drive, drive.servoIDs[IDX_FRONT_LEFT], 45.0);
        SetServoAngle(drive, drive.servoIDs[IDX_FRONT_RIGHT], -45.0);
        SetServoAngle(drive, drive.servoIDs[IDX_REAR_LEFT], -45.0);
        SetServoAngle(drive, drive.servoIDs[IDX_REAR_RIGHT], 45.0);

        drive.motorPowers[IDX_FRONT_LEFT] = drive.motorPowers[IDX_REAR_LEFT] = rot;
        drive.motorPowers[IDX_FRONT_RIGHT] = drive.motorPowers[IDX_REAR_RIGHT] = -rot;
    }
    else
    {
	    rot = NORMALIZE(rot, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
	                    (int)drive.leftAngleLimit, (int)drive.rightAngleLimit);

	    for (int i = 0; i < MAX_NUM_MOTORS; i++)
	    {
	        drive.motorPowers[i] = mag;
	    }

	    float frontAngle = BOUND(angle + rot,
	                             drive.leftAngleLimit,
	                             drive.rightAngleLimit);
	    float rearAngle = BOUND(angle - rot,
	                            drive.leftAngleLimit,
	                            drive.rightAngleLimit);
	    SetServoAngle(drive, drive.servoIDs[IDX_FRONT_LEFT], frontAngle);
	    SetServoAngle(drive, drive.servoIDs[IDX_FRONT_RIGHT], frontAngle);
	    SetServoAngle(drive, drive.servoIDs[IDX_REAR_LEFT], rearAngle);
	    SetServoAngle(drive, drive.servoIDs[IDX_REAR_RIGHT], rearAngle);
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveSwerve

/**
 *  This function performs the driving task according to the drive state.
 *
 *  @param drive Points to the DRIVE structure.
 */
void
DriveTask(
    DRIVE &drive
    )
{
    TFuncName("DriveTask");
    TLevel(TASK);
    TEnter();

    if (drive.flags & DRIVEF_ON)
    {
        if ((drive.flags & DRIVEF_STALLED) == 0)
        {
            motor[drive.motorIDs[IDX_FRONT_LEFT]] =
                drive.motorPowers[IDX_FRONT_LEFT];
            motor[drive.motorIDs[IDX_FRONT_RIGHT]] =
                drive.motorPowers[IDX_FRONT_RIGHT];
            if (drive.flags & DRIVEF_FOUR_MOTORS)
            {
                motor[drive.motorIDs[IDX_REAR_LEFT]] =
                    drive.motorPowers[IDX_REAR_LEFT];
                motor[drive.motorIDs[IDX_REAR_RIGHT]] =
                    drive.motorPowers[IDX_REAR_RIGHT];
            }

            if (drive.flags & DRIVEF_STALL_PROTECT_ON)
            {
                unsigned long currTime = nPgmTime;
                //
                // Check for motor stall conditions:
                // - Stall timer is set AND
                // - Any motors are powered above MIN_STALL_POWER AND
                // - All motors have not moved
                //
                if ((drive.stallTimer == 0) ||
                    (abs(drive.motorPowers[IDX_FRONT_LEFT]) <=
                     DRIVE_MIN_STALL_POWER) &&
                    (abs(drive.motorPowers[IDX_FRONT_RIGHT]) <=
                     DRIVE_MIN_STALL_POWER) &&
                    (!(drive.flags & DRIVEF_FOUR_MOTORS) ||
                     (abs(drive.motorPowers[IDX_REAR_LEFT]) <=
                      DRIVE_MIN_STALL_POWER) &&
                     (abs(drive.motorPowers[IDX_REAR_RIGHT]) <=
                      DRIVE_MIN_STALL_POWER)) ||
                    (nMotorEncoder[drive.motorIDs[IDX_FRONT_LEFT]] !=
                     drive.motorEncoders[IDX_FRONT_LEFT]) ||
                    (nMotorEncoder[drive.motorIDs[IDX_FRONT_RIGHT]] !=
                     drive.motorEncoders[IDX_FRONT_RIGHT]) ||
                    (drive.flags & DRIVEF_FOUR_MOTORS) &&
                     ((nMotorEncoder[drive.motorIDs[IDX_REAR_LEFT]] !=
                       drive.motorEncoders[IDX_REAR_LEFT]) ||
                      (nMotorEncoder[drive.motorIDs[IDX_REAR_RIGHT]] !=
                       drive.motorEncoders[IDX_REAR_RIGHT])))
                {
                    //
                    // We are not in a stalled situation if any of the
                    // following are true:
                    // - motor powers are below min stall power
                    // - motor encoders showed that they have moved
                    //
                    drive.motorEncoders[IDX_FRONT_LEFT] =
                        nMotorEncoder[drive.motorIDs[IDX_FRONT_LEFT]];
                    drive.motorEncoders[IDX_FRONT_RIGHT] =
                        nMotorEncoder[drive.motorIDs[IDX_FRONT_RIGHT]];
                    if (drive.flags & DRIVEF_FOUR_MOTORS)
                    {
                        drive.motorEncoders[IDX_REAR_LEFT] =
                            nMotorEncoder[drive.motorIDs[IDX_REAR_LEFT]];
                        drive.motorEncoders[IDX_REAR_RIGHT] =
                            nMotorEncoder[drive.motorIDs[IDX_REAR_RIGHT]];
                    }
                    drive.stallTimer = currTime;
                }

                if (currTime - drive.stallTimer >= DRIVE_STALL_TIME)
                {
                    //
                    // We have detected a stalled condition for at
                    // DRIVE_STALL_TIME.
                    // Let's kill the power of all the motors.
                    //
                    motor[drive.motorIDs[IDX_FRONT_LEFT]] = 0;
                    motor[drive.motorIDs[IDX_FRONT_RIGHT]] = 0;
                    if (drive.flags & DRIVEF_FOUR_MOTORS)
                    {
                        motor[drive.motorIDs[IDX_REAR_LEFT]] = 0;
                        motor[drive.motorIDs[IDX_REAR_RIGHT]] = 0;
                    }
                    drive.flags |= DRIVEF_STALLED;
                    PlayImmediateTone(1000, 100);
                }
            }
        }
    }
    else
    {
        //
        // The motors should be OFF.
        // Let's make sure they are.
        //
        motor[drive.motorIDs[IDX_FRONT_LEFT]] = 0;
        motor[drive.motorIDs[IDX_FRONT_RIGHT]] = 0;
        if (drive.flags & DRIVEF_FOUR_MOTORS)
        {
            motor[drive.motorIDs[IDX_REAR_LEFT]] = 0;
            motor[drive.motorIDs[IDX_REAR_RIGHT]] = 0;
        }
    }

    TExit();
    return;
}   //DriveTask

#endif  //ifndef _DRIVE_H
