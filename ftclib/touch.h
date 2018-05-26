#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="touch.h" />
///
/// <summary>
///     This module contains the library functions for the touch sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TOUCH_H
#define _TOUCH_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TOUCH

//
// Constants.
//
#define TOUCHF_HTSMUX           0x0001

//
// Macros.
//

/**
 *  This macro returns the state of the touch sensor.
 *
 *  @param t Points to the TOUCH structure.
 *
 *  @return Returns the state of the touch sensor.
 */
#ifdef __HTSMUX_SUPPORT__
    #define TouchGetState(t)    (((t).flags & TOUCHF_HTSMUX)? \
                                    HTSMUXreadAnalogue( \
                                        (tMUXSensor)(t).sensorID) < 500: \
                                    SensorRaw[(t).sensorID] < 500)
#else
    #define TouchGetState(t)    (SensorRaw[(t).sensorID] < 500)
#endif

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         flags;
    bool        fActive;
} TOUCH;

//
// Callback function prototypes.
//

/**
 *  This callback function is called when the touch sensor has changed state.
 *
 *  @param touch Points to the TOUCH structure.
 *  @param fActive Specifies the touch sensor is triggered.
 */
void
TouchEvent(
    TOUCH &touch,
    bool fActive
    );

/**
 *  This function initializes the touch sensor.
 *
 *  @param touch Points to the TOUCH structure.
 *  @param sensorID Specifies the ID of the gyro sensor.
 */
void
TouchInit(
    TOUCH &touch,
    tSensors sensorID
    )
{
    TFuncName("TouchInit");
    TLevel(INIT);
    TEnter();

    touch.sensorID = sensorID;
    touch.flags = 0;
    touch.fActive = false;
    if ((SensorType[sensorID] != sensorTouch) &&
        (SensorMode[sensorID] != modeBoolean))
    {
        SetSensorType(sensorID, sensorTouch);
        SetSensorMode(sensorID, modeBoolean);
        wait1Msec(10);
    }

    TExit();
    return;
}   //TouchInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the touch sensor.
 *
 *  @param touch Points to the TOUCH structure.
 *  @param sensorID Specifies the sensor MUX ID of the gyro sensor.
 */
void
TouchInit(
    TOUCH &touch,
    tMUXSensor sensorID
    )
{
    TFuncName("TouchInit");
    TLevel(INIT);
    TEnter();

    touch.sensorID = (tSensors)sensorID;
    touch.flags = TOUCHF_HTSMUX;
    touch.fActive = false;

    TExit();
    return;
}   //TouchInit
#endif

/**
 *  This function performs the touch task where it monitors the touch sensor
 *  state and send a notification if necessary.
 *
 *  @param touch Points to the TOUCH structure.
 */
void
TouchTask(
    TOUCH &touch
    )
{
    bool fActive = TouchGetState(touch);

    TFuncName("TouchTask");
    TLevel(TASK);
    TEnter();

    if (fActive != touch.fActive)
    {
        //
        // Touch sensor has changed state.
        //
        TouchEvent(touch, fActive);
        touch.fActive = fActive;
    }

    TExit();
    return;
}   //TouchTask

#endif  //ifndef _TOUCH_H
