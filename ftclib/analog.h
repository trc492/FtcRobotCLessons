#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="analog.h" />
///
/// <summary>
///     This module contains the library functions for the analog sensors.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ANALOG_H
#define _ANALOG_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ANALOG

//
// Constants.
//
#define ANALOGO_INVERSE         0x0001

#define ANALOGF_HTSMUX          0x0001

#ifndef ANALOG_NUM_CAL_SAMPLES
    #define ANALOG_NUM_CAL_SAMPLES  50
#endif
#ifndef ANALOG_CAL_INTERVAL
    #define ANALOG_CAL_INTERVAL     10
#endif

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         options;
    int         flags;
    int         zeroOffset;
    int         deadBand;
} ANALOG;

/**
 *  This function calibrates the analog sensor for zero offset and deadband.
 *
 *  @param analog Points to the ANALOG structure to be calibrated.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
AnalogCalibrate(
    ANALOG &analog,
    int numSamples,
    int calInterval
    )
{
    TFuncName("AnalogCal");
    TLevel(API);
    TEnter();

    analog.zeroOffset = 0;
    analog.deadBand = 0;

    if ((numSamples != 0) && (calInterval != 0))
    {
        int i;
        int data;
        int minValue, maxValue;

        minValue = 1023;
        maxValue = 0;
        for (i = 0; i < numSamples; i++)
        {
#ifdef __HTSMUX_SUPPORT__
            data = (analog.flags & ANALOGF_HTSMUX)?
                        HTSMUXreadAnalogue((tMUXSensor)analog.sensorID):
                        SensorRaw[analog.sensorID];
#else
            data = SensorRaw[analog.sensorID];
#endif
            analog.zeroOffset += data;

            if (data < minValue)
            {
                minValue = data;
            }
            else if (data > maxValue)
            {
                maxValue = data;
            }

            wait1Msec(calInterval);
        }

        analog.zeroOffset /= numSamples;
        analog.deadBand = maxValue - minValue;
    }

    TExit();
    return;
}   //AnalogCalibrate

/**
 *  This function sets the zero offset and deadband for the sensor.
 *
 *  @param analog Points to the ANALOG structure to be calibrated.
 *  @param zeroOffset Specifies the zero offset.
 *  @param deadBand Specifies the deadband value.
 */
void
AnalogSetCalibration(
    ANALOG &analog,
    int zeroOffset,
    int deadBand
    )
{
    TFuncName("AnalogSetCal");
    TLevel(API);
    TEnter();

    analog.zeroOffset = zeroOffset;
    analog.deadBand = deadBand;

    TExit();
    return;
}   //AnalogSetCalibration

/**
 *  This function reads the data from the analog sensor and adjust it for
 *  offset and deadband.
 *
 *  @param analog Points to the ANALOG structure.
 */
int
AnalogReadValue(
    ANALOG &analog,
    )
{
    int data;

    TFuncName("AnalogReadValue");
    TLevel(API);
    TEnter();

#ifdef __HTSMUX_SUPPORT__
    data = (analog.flags & ANALOGF_HTSMUX)?
                HTSMUXreadAnalogue((tMUXSensor)analog.sensorID):
                SensorRaw[analog.sensorID];
#else
    data = SensorRaw[analog.sensorID];
#endif
    data -= analog.zeroOffset;
    data = DEADBAND(data, analog.deadBand);
    if (analog.options & ANALOGO_INVERSE)
    {
        data = -data;
    }

    TExitMsg(("=%d", data));
    return data;
}   //AnalogReadValue

/**
 *  This function initializes the analog sensor.
 *
 *  @param analog Points to the ANALOG structure.
 *  @param sensorID Specifies the ID of the analog sensor.
 *  @param options Optionally specifies the analog sensor options:
 *         ANALOGO_INVERSE - Specifies sensor reading is inverse.
 *  @param numSamples Optionally specifies the number of calibration samples.
 *  @param calInterval Optionally specifies the calibration interval in msec.
 */
void
AnalogInit(
    ANALOG &analog,
    tSensors sensorID,
    int options = 0,
    int numSamples = 0,
    int calInterval = 0
    )
{
    TFuncName("AnalogInit");
    TLevel(INIT);
    TEnter();

    analog.sensorID = sensorID;
    analog.options = options;
    analog.flags = 0;
    AnalogCalibrate(analog, numSamples, calInterval);

    TExit();
    return;
}   //AnalogInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the analog sensor.
 *
 *  @param analog Points to the ANALOG structure.
 *  @param sensorID Specifies the sensor MUX ID of the analog sensor.
 *  @param options Optionally specifies the analog sensor options:
 *         ANALOGO_INVERSE - Specifies sensor reading is inverse.
 *  @param numSamples Optionally specifies the number of calibration samples.
 *  @param calInterval Optionally specifies the calibration interval in msec.
 */
void
AnalogInit(
    ANALOG &analog,
    tMUXSensor sensorID,
    int options = 0,
    int numSamples = 0,
    int calInterval = 0
    )
{
    TFuncName("AnalogInit");
    TLevel(INIT);
    TEnter();

    analog.sensorID = (tSensors)sensorID;
    analog.options = options;
    analog.flags = ANALOGF_HTSMUX;
    AnalogCalibrate(analog, numSamples, calInterval);

    TExit();
    return;
}   //AnalogInit
#endif

#endif  //ifndef _ANALOG_H
