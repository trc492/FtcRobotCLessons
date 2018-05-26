#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="accel.h" />
///
/// <summary>
///     This module contains the library functions for the accelerometer
///     sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ACCEL_H
#define _ACCEL_H

#include "..\RobotCDrivers\drivers\hitechnic-accelerometer.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ACCEL

//
// Constants.
//
#ifdef _KALMAN_H
#define ACCELO_FILTER           0x0001
#endif

#define ACCELF_ENABLED          0x0001
#define ACCELF_HTSMUX           0x0002

#define ACCEL_COUNT_PER_G       200
#define ACCEL_NUM_CAL_SAMPLES   50
#define ACCEL_CAL_INTERVAL      10

//
// Macros.
//

/**
 *  This macro gets the acceleration value of the X-axis in in/sec/sec.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the acceleration value of the X-axis.
 */
#define AccelGetXAccel(a)       ((a).xAccel*INCHES_PER_METER)

/**
 *  This macro gets the acceleration value of the Y-axis in in/sec/sec.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the acceleration value of the Y-axis.
 */
#define AccelGetYAccel(a)       ((a).yAccel*INCHES_PER_METER)

/**
 *  This macro gets the acceleration value of the Z-axis in in/sec/sec.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the acceleration value of the Z-axis.
 */
#define AccelGetZAccel(a)       ((a).zAccel*INCHES_PER_METER)

/**
 *  This macro gets the velocity value of the X-axis in unit in/sec.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the velocity value of the X-axis.
 */
#define AccelGetXVel(a)         ((a).xVel*INCHES_PER_METER)

/**
 *  This macro gets the velocity value of the Y-axis in unit in/sec.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the velocity value of the Y-axis.
 */
#define AccelGetYVel(a)         ((a).yVel*INCHES_PER_METER)

/**
 *  This macro gets the velocity value of the Z-axis in unit in/sec.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the velocity value of the Z-axis.
 */
#define AccelGetZVel(a)         ((a).zVel*INCHES_PER_METER)

/**
 *  This macro gets the distance value of the X-axis in inches.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the distance value of the X-axis.
 */
#define AccelGetXDist(a)        ((a).xDist*INCHES_PER_METER)

/**
 *  This macro gets the distance value of the Y-axis in inches.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the distance value of the Y-axis.
 */
#define AccelGetYDist(a)        ((a).yDist*INCHES_PER_METER)

/**
 *  This macro gets the distance value of the Z-axis in inches.
 *
 *  @param a Points to the ACCEL structure.
 *
 *  @return Returns the distance value of the Z-axis.
 */
#define AccelGetZDist(a)        ((a).zDist*INCHES_PER_METER)

//
// Type definitions.
//
typedef struct
{
    tSensors        sensorID;
    int             options;
    int             flags;
    int             xZeroOffset;
    int             yZeroOffset;
    int             zZeroOffset;
    int             xDeadBand;
    int             yDeadBand;
    int             zDeadBand;
    float           xAccel;
    float           yAccel;
    float           zAccel;
    float           xVel;
    float           yVel;
    float           zVel;
    float           xDist;
    float           yDist;
    float           zDist;
    unsinged long   prevTime;
#ifdef _KALMAN_H
    KALMAN          kalmanX;
    KALMAN          kalmanY;
    KALMAN          kalmanZ;
#endif
} ACCEL;

/**
 *  This function calibrates the accelerometer for zero offsets and deadband
 *  on all axes.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
AccelCal(
    ACCEL &accel,
    int numSamples,
    int calInterval
    )
{
    int i;
    int xRaw, yRaw, zRaw;
    int xMin, yMin, zMin;
    int xMax, yMax, zMax;
    bool fSMux;

    TFuncName("AccelCal");
    TLevel(API);

    accel.xZeroOffset = 0;
    accel.yZeroOffset = 0;
    accel.zZeroOffset = 0;
    accel.xDeadBand = 0;
    accel.yDeadBand = 0;
    accel.zDeadBand = 0;
    xMin = yMin = zMin = 1023;
    xMax = yMax = zMax = 0;
    fSMux = (accel.flags & ACCELF_HTSMUX) != 0;
    for (i = 0; i < numSamples; i++)
    {
#ifdef __HTSMUX_SUPPORT__
        if (fSMux &&
            HTACreadAllAxes((tMUXSensor)accel.sensorID, xRaw, yRaw, zRaw) ||
            !fSMux &&
            HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw))
#else
        if (HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw))
#endif
        {
            accel.xZeroOffset += xRaw;
            accel.yZeroOffset += yRaw;
            accel.zZeroOffset += zRaw;

            if (xRaw < xMin)
            {
                xMin = xRaw;
            }
            else if (xRaw > xMax)
            {
                xMax = xRaw;
            }

            if (yRaw < yMin)
            {
                yMin = yRaw;
            }
            else if (yRaw > yMax)
            {
                yMax = yRaw;
            }

            if (zRaw < zMin)
            {
                zMin = zRaw;
            }
            else if (zRaw > zMax)
            {
                zMax = zRaw;
            }
        }
        wait1Msec(calInterval);
    }

    accel.xZeroOffset /= numSamples;
    accel.yZeroOffset /= numSamples;
    accel.zZeroOffset /= numSamples;

    accel.xDeadBand = xMax - xMin;
    accel.yDeadBand = yMax - yMin;
    accel.zDeadBand = zMax - zMin;

    TExit();
    return;
}   //AccelCal

/**
 *  This function resets the accelerometer values.
 *
 *  @param accel Points to the ACCEL structure.
 */
void
AccelReset(
    ACCEL &accel
    )
{
    TFuncName("AccelReset");
    TLevel(API);
    TEnter();

    accel.xAccel = 0.0;
    accel.yAccel = 0.0;
    accel.zAccel = 0.0;
    accel.xVel = 0.0;
    accel.yVel = 0.0;
    accel.zVel = 0.0;
    accel.xDist = 0.0;
    accel.yDist = 0.0;
    accel.zDist = 0.0;

    TExit();
    return;
}   //AccelReset

/**
 *  This function initializes the accelerometer sensor.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param sensorID Specifies the ID of the accelerometer sensor.
 *  @param options Optionally specifies the accelerometer options:
 *         ACCELO_FILTER - Apply filter to the accelerometer data.
 */
void
AccelInit(
    ACCEL &accel,
    tSensors sensorID,
    int options = 0
    )
{
    TFuncName("AccelInit");
    TLevel(INIT);
    TEnter();

    accel.sensorID = sensorID;
    accel.options = options;
    accel.flags = 0;
    AccelCal(accel, ACCEL_NUM_CAL_SAMPLES, ACCEL_CAL_INTERVAL);
    AccelReset(accel);
    accel.prevTime = 0;
#ifdef _KALMAN_H
    KalmanInit(accel.kalmanX);
    KalmanInit(accel.kalmanY);
    KalmanInit(accel.kalmanZ);
#endif

    TExit();
    return;
}   //AccelInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the accelerometer sensor.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param sensorID Specifies the sensor MUX ID of the accelerometer sensor.
 *  @param options Optionally specifies the accelerometer options:
 *         ACCELO_FILTER - Apply filter to the accelerometer data.
 */
void
AccelInit(
    ACCEL &accel,
    tMUXSensor sensorID,
    int options = 0
    )
{
    TFuncName("AccelInit");
    TLevel(INIT);
    TEnter();

    accel.sensorID = (tSensors)sensorID;
    accel.options = options;
    accel.flags = ACCELF_HTSMUX;
    AccelCal(accel, ACCEL_NUM_CAL_SAMPLES, ACCEL_CAL_INTERVAL);
    AccelReset(accel);
    accel.prevTime = 0;
#ifdef _KALMAN_H
    KalmanInit(accel.kalmanX);
    KalmanInit(accel.kalmanY);
    KalmanInit(accel.kalmanZ);
#endif

    TExit();
    return;
}   //AccelInit
#endif

/**
 *  This function is called to enable or disable the accelerometer integrator.
 *
 *  @param accel Points to the ACCEL structure.
 */
void
AccelSetEnable(
    ACCEL &accel,
    bool fEnable
    )
{
    TFuncName("AccelSetEnable");
    TLevel(API);
    TEnterMsg(("fEnable=%d", fEnable));

    if (fEnable)
    {
        accel.flags |= ACCELF_ENABLED;
    }
    else
    {
        accel.flags &= ~ACCELF_ENABLED;
    }

    TExit();
    return;
}   //AccelSetEnable

/**
 *  This function is called periodically to get the accelerometer axis values
 *  and integrate them to velocity and distance values.
 *
 *  @param accel Points to the ACCEL structure.
 */
void
AccelTask(
    ACCEL &accel
    )
{
    TFuncName("AccelTask");
    TLevel(TASK);
    TEnter();

    if (accel.flags & ACCELF_ENABLED)
    {
        unsigned long currTime;
        float period;
        bool rc;
        int xRaw, yRaw, zRaw;

        currTime = nPgmTime;
        period = (accel.prevTime == 0)?
                    0.0: (currTime - accel.prevTime)/1000.0;
#ifdef __HTSMUX_SUPPORT__
        rc = (accel.flags & ACCELF_HTSMUX)?
             HTACreadAllAxes((tMUXSensor)accel.sensorID, xRaw, yRaw, zRaw):
             HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw);
#else
        rc = HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw);
#endif
        if (rc)
        {
            accel.xAccel = (float)(DEADBAND(xRaw - accel.xZeroOffset,
                                            accel.xDeadBand))/
                           ACCEL_COUNT_PER_G;
            accel.yAccel = (float)(DEADBAND(yRaw - accel.yZeroOffset,
                                            accel.yDeadBand))/
                           ACCEL_COUNT_PER_G;
            accel.zAccel = (float)(DEADBAND(zRaw - accel.zZeroOffset,
                                            accel.zDeadBand))/
                           ACCEL_COUNT_PER_G;
#ifdef _KALMAN_H
            if (accel.options & ACCELO_FILTER)
            {
                accel.xAccel = KalmanFilter(accel.kalmanX, accel.xAccel);
                accel.yAccel = KalmanFilter(accel.kalmanY, accel.yAccel);
                accel.zAccel = KalmanFilter(accel.kalmanZ, accel.zAccel);
            }
#endif
            accel.xVel += accel.xAccel*period;
            accel.yVel += accel.yAccel*period;
            accel.zVel += accel.zAccel*period;
            accel.xDist += accel.xVel*period;
            accel.yDist += accel.yVel*period;
            accel.zDist += accel.zVel*period;
        }
        accel.prevTime = currTime;
    }

    TExit();
    return;
}   //AccelTask

#endif  //ifndef _ACCEL_H
