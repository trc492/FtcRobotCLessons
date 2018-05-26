#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="compass.h" />
///
/// <summary>
///     This module contains the library functions for the compass sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _COMPASS_H
#define _COMPASS_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_COMPASS

//
// Constants.
//
#define COMPASSF_HTSMUX         0x0001

#ifndef COMPASS_SAMPLING_PERIOD
#define COMPASS_SAMPLING_PERIOD 10      //msec
#endif

//
// Macros
//

/**
 *  This macro returns the value of the compass sensor.
 *
 *  @param c Points to the COMPASS structure.
 *
 *  @return Returns the current reading of the sensor.
 */
#ifdef __HTSMUX_SUPPORT__
    #define CompassGetValue(c)  (((c).flags & COMPASSF_HTSMUX)? \
                                 HTSMUXreadAnalogue((tMUXSensor)(c).sensorID): \
                                 SensorValue[(c).sensorID])
#else
    #define CompassGetValue(c)  SensorValue[(c).sensorID]
#endif

/**
 *  This macro returns the heading of the compass sensor (0-359 degrees).
 *
 *  @param c Points to the COMPASS structure.
 *
 *  @return Returns the current heading.
 */
#define CompassGetHeading(c)    ((c).currHeading)

/**
 *  This macro returns the accumulated heading of the compass sensor.
 *
 *  @param c Points to the COMPASS structure.
 *
 *  @return Returns the accumulated heading.
 */
#define CompassGetAccumHeading(c) ((c).accRev*360 + (c).currHeading)

//
// Type definitions.
//
typedef struct
{
    tSensors        sensorID;
    int             flags;
    int             revThreshold;
    unsigned long   nextPeriod;
    int             currHeading;
    int             accRev;
} COMPASS;

/**
 *  This function performs the compass task monitoring the compass value for
 *  wraparounds.
 *
 *  @param compass Points to the COMPASS structure.
 */
void
CompassTask(
    COMPASS &compass
    )
{
    int currHeading;

    TFuncName("CompassTask");
    TLevel(TASK);
    TEnter();

    if (nPgmTime >= compass.nextPeriod)
    {
        compass.nextPeriod += COMPASS_SAMPLING_PERIOD;
        currHeading = CompassGetValue(compass);
        if (abs(currHeading - compass.currHeading) > compass.revThreshold)
        {
            if (currHeading > compass.currHeading)
            {
                compass.accRev--;
            }
            else
            {
                compass.accRev++;
            }
        }
        compass.currHeading = currHeading;
    }

    TExit();
    return;
}   //CompassTask

/**
 *  This function resets the compass accumulated revolution count.
 *
 *  @param compass Points to the COMPASS structure.
 */
void
CompassReset(
    COMPASS &compass
    )
{
    TFuncName("CompassReset");
    TLevel(API);
    TEnter();

    compass.accRev = 0;

    TExit();
    return;
}   //CompassReset

/**
 *  This function initializes the compass sensor.
 *
 *  @param compass Points to the COMPASS structure.
 *  @param sensorID Specifies the ID of the compass sensor.
 *  @param revThreshold Optionally specifies the heading threshold to be
 *         considered wrapped around.
 */
void
CompassInit(
    COMPASS &compass,
    tSensors sensorID,
    int revThreshold = 270
    )
{
    TFuncName("CompassInit");
    TLevel(INIT);
    TEnter();

    compass.sensorID = sensorID;
    compass.flags = 0;
    compass.revThreshold = revThreshold;
    compass.nextPeriod = nPgmTime;
    compass.currHeading = CompassGetValue(compass);
    compass.accRev = 0;

    TExit();
    return;
}   //CompassInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the compass sensor.
 *
 *  @param compass Points to the COMPASS structure.
 *  @param sensorID Specifies the Sensor MUX ID of the compass sensor.
 *  @param revThreshold Optionally specifies the heading threshold to be
 *         considered wrapped around.
 */
void
CompassInit(
    COMPASS &compass,
    tMUXSensor sensorID,
    int revThreshold = 270
    )
{
    TFuncName("CompassInit");
    TLevel(INIT);
    TEnter();

    compass.sensorID = (tSensors)sensorID;
    compass.flags = SENSORF_HTSMUX;
    compass.revThreshold = revThreshold;
    compass.nextPeriod = nPgmTime;
    compass.currHeading = CompassGetValue(compass);
    compass.accRev = 0;

    TExit();
    return;
}   //CompassInit
#endif

#endif  //ifndef _COMPASS_H
