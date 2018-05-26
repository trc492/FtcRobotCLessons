#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="gyro.h" />
///
/// <summary>
///     This module contains the library functions for the gyro sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _GYRO_H
#define _GYRO_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_GYRO

//
// Constants.
//
#define GYROO_INVERSE           0x0001
#ifdef _KALMAN_H
#define GYROO_FILTER            0x0002
#endif

#define GYROF_HTSMUX            0x0001

#ifndef GYRO_SAMPLING_PERIOD
#define GYRO_SAMPLING_PERIOD    5
#endif

#define GYRO_NUM_CAL_SAMPLES    100
#define GYRO_CAL_INTERVAL       10

//
// Macros
//

/**
 *  This macro returns the raw value of the gyro sensor.
 *
 *  @param g Points to the GYRO structure.
 *
 *  @return Returns the raw sensor value.
 */
#ifdef __HTSMUX_SUPPORT__
    #define GyroGetRawValue(g)  (((g).flags & GYROF_HTSMUX)? \
                                 HTSMUXreadAnalogue((tMUXSensor)(g).sensorID): \
                                 SensorValue[(g).sensorID])
#else
    #define GyroGetRawValue(g)  SensorValue[(g).sensorID]
#endif

/**
 *  This macro returns the turn rate reading of the gyro sensor.
 *
 *  @param g Points to the GYRO structure.
 *
 *  @return Returns the turn rate.
 */
#define GyroGetTurnRate(g)      ((g).turnRate)

/**
 *  This macro returns the integrated heading of the gyro sensor.
 *
 *  @param g Points to the GYRO structure.
 *
 *  @return Returns the integrated heading.
 */
#define GyroGetHeading(g)       ((g).heading)

//
// Type definitions.
//
typedef struct
{
    tSensors        sensorID;
    int             options;
    int             flags;
    int             zeroOffset;
    int             deadBand;
    unsigned long   prevTime;
    unsigned long   nextPeriod;
    int             turnRate;
    float           heading;
#ifdef _KALMAN_H
    KALMAN      kalman;
#endif
} GYRO;

/**
 *  This function calibrates the gyro for zero offset and deadband.
 *
 *  @param gyro Points to the GYRO structure.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
GyroCal(
    GYRO &gyro,
    int numSamples,
    int calInterval
    )
{
    int i;
    int turnRate;
    int minValue, maxValue;
    float sum = 0.0;

    TFuncName("GyroCal");
    TLevel(API);
    TEnter();

    gyro.zeroOffset = 0;
    gyro.deadBand = 0;
    minValue = 1023;
    maxValue = 0;

    for (i = 0; i < numSamples; i++)
    {
        turnRate = GyroGetRawValue(gyro);
        sum += turnRate;

        if (turnRate < minValue)
        {
            minValue = turnRate;
        }
        else if (turnRate > maxValue)
        {
            maxValue = turnRate;
        }

        wait1Msec(calInterval);
    }

    gyro.zeroOffset = round(sum/numSamples);
    gyro.deadBand = round((float)(maxValue - minValue)/2.0);
#ifdef _DEBUG_GYRO
    nxtDisplayTextLine(5, "min=%d,max=%d", minValue, maxValue);
    nxtDisplayTextLine(6, "zeroOffset=%d", gyro.zeroOffset);
    nxtDisplayTextLine(7, "deadBand=%d", gyro.deadBand);
#endif

    TExit();
    return;
}   //GyroCal

/**
 *  This function performs the gyro task where it integrates the turn rate
 *  into a heading value.
 *
 *  @param gyro Points to the GYRO structure.
 */
void
GyroTask(
    GYRO &gyro
    )
{
    TFuncName("GyroTask");
    TLevel(TASK);
    TEnter();

    unsigned long currTime = nPgmTime;
    if (currTime >= gyro.nextPeriod)
    {
        gyro.nextPeriod += GYRO_SAMPLING_PERIOD;
        gyro.turnRate = GyroGetRawValue(gyro);
        gyro.turnRate -= gyro.zeroOffset;
        gyro.turnRate = DEADBAND(gyro.turnRate, gyro.deadBand);
        if (gyro.options & GYROO_INVERSE)
        {
            gyro.turnRate = -gyro.turnRate;
        }
#ifdef _KALMAN_H
        if (gyro.options & GYROO_FILTER)
        {
            gyro.turnRate = KalmanFilter(gyro.kalman, gyro.turnRate);
        }
#endif
        gyro.heading += (float)gyro.turnRate*(currTime - gyro.prevTime)/
                         1000;
        gyro.prevTime = currTime;
    }

    TExit();
    return;
}   //GyroTask

/**
 *  This function resets the gyro heading.
 *
 *  @param gyro Points to the GYRO structure to be reset.
 */
void
GyroReset(
    GYRO &gyro
    )
{
    TFuncName("GyroReset");
    TLevel(API);
    TEnter();

    GyroTask(gyro);
    gyro.heading = 0;

    TExit();
    return;
}   //GyroReset

/**
 *  This function initializes the gyro sensor.
 *
 *  @param gyro Points to the GYRO structure.
 *  @param sensorID Specifies the ID of the gyro sensor.
 *  @param options Optionally specifies the gyro options:
 *         GYROO_INVERSE - Specifies sensor reading is inverse.
 *         GYROO_FILTER - Apply filter to the gyro reading.
 */
void
GyroInit(
    GYRO &gyro,
    tSensors sensorID,
    int options = 0
    )
{
    TFuncName("GyroInit");
    TLevel(INIT);
    TEnter();

    gyro.sensorID = sensorID;
    gyro.options = options;
    gyro.flags = 0;
#ifdef _KALMAN_H
    KalmanInit(gyro.kalman);
#endif

    if (SensorType[sensorID] != sensorAnalogInactive)
    {
        SetSensorType(sensorID, sensorAnalogInactive);
        wait1Msec(100);
    }

    GyroCal(gyro, GYRO_NUM_CAL_SAMPLES, GYRO_CAL_INTERVAL);
    gyro.prevTime = nPgmTime;
    gyro.nextPeriod = gyro.prevTime;
    GyroReset(gyro);

    TExit();
    return;
}   //GyroInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the gyro sensor.
 *
 *  @param gyro Points to the GYRO structure.
 *  @param sensorID Specifies the sensor MUX ID of the gyro sensor.
 *  @param options Optionally specifies the gyro options:
 *         GYROO_INVERSE - Specifies sensor reading is inverse.
 *         GYROO_FILTER - Apply filter to the gyro reading.
 */
void
GyroInit(
    GYRO &gyro,
    tMUXSensor sensorID,
    int options = 0
    )
{
    TFuncName("GyroInit");
    TLevel(INIT);
    TEnter();

    gyro.sensorID = (tSensors)sensorID;
    gyro.options = options;
    gyro.flags = GYROF_HTSMUX;
#ifdef _KALMAN_H
    KalmanInit(gyro.kalman);
#endif

    GyroCal(gyro, GYRO_NUM_CAL_SAMPLES, GYRO_CAL_INTERVAL);
    gyro.prevTime = nPgmTime;
    gyro.nextPeriod = gyro.prevTime;
    GyroReset(gyro);

    TExit();
    return;
}   //GyroInit
#endif

#endif  //ifndef _GYRO_H
