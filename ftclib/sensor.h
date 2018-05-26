#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="sensor.h" />
///
/// <summary>
///     This module contains the library functions to handle various sensors.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SENSOR_H
#define _SENSOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SENSOR

//
// Constants.
//
#define SENSORZONE_LOW          0
#define SENSORZONE_MID          1
#define SENSORZONE_HIGH         2
#define NUM_SENSOR_ZONES        3

#define SENSORO_INVERSE         0x0001
#ifdef _KALMAN_H
#define SENSORO_FILTER          0x0002
#endif

#define SENSORF_HTSMUX          0x0001
#define SENSORF_CALIBRATING     0x0002

//
// Macros.
//

/**
 *  This macro checks if it is in calibration mode.
 *
 *  @param s Points to the SENSOR structure.
 *
 *  @return Returns true if sensor calibration is in progress.
 */
#define SensorCalibrating(s)    ((s).flags & SENSORF_CALIBRATING)

/**
 * This macro returns the current zone detected.
 *
 *  @param s Points to the SENSOR structure.
 *
 *  @return Returns the current sensor zone.
 */
#define SensorGetZone(s)        ((s).zone)

/**
 *  This macro returns the value of the sensor.
 *
 *  @return Returns the sensor value.
 */
#ifdef __HTSMUX_SUPPORT__
    #define SensorGetValue(s)   (((s).flags & SENSORF_HTSMUX)?  \
                                 (1023 -                        \
                                  HTSMUXreadAnalogue((tMUXSensor)(s).sensorID)): \
                                 SensorRaw((s).sensorID))
#else
    #define SensorGetValue(s)   SensorRaw[(s).sensorID]
#endif

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         lowThreshold;
    int         highThreshold;
    int         options;
    int         flags;
    int         value;
    int         zone;
    int         rawMin;
    int         rawMax;
#ifdef _KALMAN_H
    KALMAN      kalman;
#endif
} SENSOR;

//
// Callback function prototypes.
//

/**
 *  This callback function is called when the sensor value crosses to a
 *  different zone.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param zone Specifying the current sensor zone.
 */
void
SensorEvent(
    SENSOR &sensor,
    int zone
    );

/**
 *  This function updates the sensor reading and determines the corresponding
 *  zone.
 *
 *  @param sensor Points to the SENSOR structure.
 */
void
SensorUpdate(
    SENSOR &sensor
    )
{
    TFuncName("SensorTask");
    TLevel(FUNC);
    TEnter();

#ifdef __HTSMUX_SUPPORT__
    if (sensor.flags & SENSORF_HTSMUX)
    {
        sensor.value = 1023 - HTSMUXreadAnalogue((tMUXSensor)sensor.sensorID);
    }
    else
    {
        sensor.value = SensorRaw[sensor.sensorID];
    }
#else
    sensor.value = SensorRaw[sensor.sensorID];
#endif
#ifdef _KALMAN_H
    if (sensor.options & SENSORO_FILTER)
    {
        sensor.value = KalmanFilter(sensor.kalman, sensor.value);
    }
#endif
    if (sensor.flags & SENSORF_CALIBRATING)
    {
        //
        // We are in calibration mode.
        //
        if (sensor.value < sensor.rawMin)
        {
            sensor.rawMin = sensor.value;
        }
        else if (sensor.value > sensor.rawMax)
        {
            sensor.rawMax = sensor.value;
        }
    }
    else
    {
        if (sensor.value <= sensor.lowThreshold)
        {
            sensor.zone = (sensor.options & SENSORO_INVERSE)?
                            SENSORZONE_HIGH: SENSORZONE_LOW;
        }
        else if (sensor.value <= sensor.highThreshold)
        {
            sensor.zone = SENSORZONE_MID;
        }
        else
        {
            sensor.zone = (sensor.options & SENSORO_INVERSE)?
                            SENSORZONE_LOW: SENSORZONE_HIGH;
        }
    }

    TExit();
    return;
}   //SensorUpdate

/**
 *  This function initializes the sensor system.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param sensorID Specifies the sensor ID.
 *  @param lowThreshold Specifies the low threshold value.
 *  @param highThreshold Specifies the high threshold value.
 *  @param options Optionally specifies the sensor options:
 *         SENSORO_INVERSE - Specifies sensor reading is inverse.
 *         SENSORO_FILTER - Apply filter to the sensor reading.
 */
void
SensorInit(
    SENSOR &sensor,
    tSensors sensorID,
    int lowThreshold,
    int highThreshold,
    int options = 0
    )
{
    TFuncName("SensorInit");
    TLevel(INIT);
    TEnter();

    sensor.sensorID = sensorID;
    sensor.lowThreshold = lowThreshold;
    sensor.highThreshold = highThreshold;
    sensor.options = options;
    sensor.flags = 0;
#ifdef _KALMAN_H
    KalmanInit(sensor.kalman);
#endif
    SensorUpdate(sensor);

    TExit();
    return;
}   //SensorInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the sensor system.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param sensorID Specifies the sensor MUX ID.
 *  @param lowThreshold Specifies the low threshold value.
 *  @param highThreshold Specifies the high threshold value.
 *  @param options Optionally specifies the sensor options:
 *         SENSORO_INVERSE - Specifies sensor reading is inverse.
 *         SENSORO_FILTER - Apply filter to the sensor reading.
 */
void
SensorInit(
    SENSOR &sensor,
    tMUXSensor sensorID,
    int lowThreshold,
    int highThreshold,
    int options = 0
    )
{
    TFuncName("SensorInit");
    TLevel(INIT);
    TEnter();

    sensor.sensorID = (tSensors)sensorID;
    sensor.lowThreshold = lowThreshold;
    sensor.highThreshold = highThreshold;
    sensor.options = options;
    sensor.flags = SENSORF_HTSMUX;
#ifdef _KALMAN_H
    KalmanInit(sensor.kalman);
#endif
    SensorUpdate(sensor);

    TExit();
    return;
}   //SensorInit
#endif

/**
 *  This function starts or stops the sensor calibration process.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param fStart Specifies whether to start or stop the calibration.
 */
void
SensorCal(
    SENSOR &sensor,
    bool fStart
    )
{
    TFuncName("SensorCal");
    TLevel(API);
    TEnterMsg(("fStart=%d", (byte)fStart));

    if (fStart)
    {
        sensor.flags |= SENSORF_CALIBRATING;
        sensor.rawMin = 1023;
        sensor.rawMax = 0;
    }
    else
    {
        int zoneRange = (sensor.rawMax - sensor.rawMin)/3;

        sensor.flags &= ~SENSORF_CALIBRATING;
        sensor.lowThreshold = sensor.rawMin + zoneRange;
        sensor.highThreshold = sensor.rawMax - zoneRange;
        TInfo(("LoTh=%d,HiTh=%d",
               sensor.lowThreshold, sensor.highThreshold));
    }

    TExit();
    return;
}   //SensorCal

/**
 *  This function processes the sensor reading and sends a trigger event if
 *  necessary.
 *
 *  @param sensor Points to the SENSOR structure.
 */
void
SensorTask(
    SENSOR &sensor
    )
{
    TFuncName("SensorTask");
    TLevel(TASK);
    TEnter();

    int prevZone = sensor.zone;
    SensorUpdate(sensor);
    if (!(sensor.flags & SENSORF_CALIBRATING) && (sensor.zone != prevZone))
    {
        //
        // We have crossed to another zone, let's send a sensor event.
        //
        SensorEvent(sensor, sensor.zone);
    }

    TExit();
    return;
}   //SensorTask

#endif  //ifndef _SENSOR_H
