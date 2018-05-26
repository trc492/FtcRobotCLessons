#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="irseeker.h" />
///
/// <summary>
///     This module contains the library functions for the IR Seeker sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _IRSEEKER_H
#define _IRSEEKER_H

#include "..\RobotCDrivers\drivers\hitechnic-irseeker-v2.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_IRSEEKER

//
// Constants.
//
#ifdef _KALMAN_H
#define IRSEEKERO_FILTER        0x0001
#endif

#define IRSEEKERF_HTSMUX        0x0001

//
// Macros.
//

/**
 *  This macro returns the raw AC Direction value of the IR Seeker sensor.
 *
 *  @param i Points to the IRSEEKER structure.
 *
 *  @return Returns the raw AC direction value.
 */
#ifdef __HTSMUX_SUPPORT__
    #define IRSeekerGetRawACDir(i) (((i).flags & IRSEEKERF_HTSMUX)? \
                                    HTIRS2readACDir((tMUXSensor)((i).sensorID)): \
                                    HTIRS2readACDir((i).sensorID))
#else
    #define IRSeekerGetRawACDir(i)  HTIRS2readACDir((i).sensorID)
#endif

/**
 *  This macro returns the raw AC Data of the IR Seeker sensor.
 *
 *  @param i Points to the IRSEEKER structure.
 *
 *  @return Returns true if successful.
 */
#ifdef __HTSMUX_SUPPORT__
    #define IRSeekerGetRawACData(i) (((i).flags & IRSEEKERF_HTSMUX)? \
                                      HTIRS2readAllACStrength( \
                                        (tMUXSensor)((i).sensorID), \
                                        i.rawACStrength[0], \
                                        i.rawACStrength[1], \
                                        i.rawACStrength[2], \
                                        i.rawACStrength[3], \
                                        i.rawACStrength[4]): \
                                      HTIRS2readAllACStrength( \
                                        i.sensorID, \
                                        i.rawACStrength[0], \
                                        i.rawACStrength[1], \
                                        i.rawACStrength[2], \
                                        i.rawACStrength[3], \
                                        i.rawACStrength[4]))

#else
    #define IRSeekerGetRawACData(i) HTIRS2readAllACStrength( \
                                        (i).sensorID, \
                                        i.rawACStrength[0], \
                                        i.rawACStrength[1], \
                                        i.rawACStrength[2], \
                                        i.rawACStrength[3], \
                                        i.rawACStrength[4])
#endif

/**
 *  This macro returns the AC Direction value of the IR Seeker sensor.
 *
 *  @param i Points to the IRSEEKER structure.
 *
 *  @return Returns the AC direction value.
 */
#define IRSeekerGetACDir(i)         ((i).acDir)

/**
 *  This macro returns the AC Strength value of the IR Seeker sensor.
 *
 *  @param i Points to the IRSEEKER structure.
 *
 *  @return Returns the AC strength value.
 */
#define IRSeekerGetACStrength(i)    ((i).acStrength)

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         options;
    int         flags;
    int         rawACStrength[5];
    int         rawACDir;
    float       acDir;
    int         acStrength;
#ifdef _KALMAN_H
    KALMAN      kalman;
#endif
} IRSEEKER;

/**
 *  This function is called periodically to read the IR seeker and update
 *  the various values.
 *
 *  @param irSeeker Points to the IRSEEKER structure.
 */
void
IRSeekerTask(
    IRSEEKER &irSeeker
    )
{
    TFuncName("IRSeekerUpdate");
    TLevel(TASK);
    TEnter();

    int rawACDir = IRSeekerGetRawACDir(irSeeker);
    //
    // When dir is zero, it means we lost the IR beacon signal.
    // In this case, we return the previous dir data.
    //
    if ((rawACDir > 0) && IRSeekerGetRawACData(irSeeker))
    {
        irSeeker.rawACDir = rawACDir;
        irSeeker.acDir = (float)rawACDir;
        //
        // Let's make the value more precise by interpolating the adjacent
        // sensor reading.
        //
        int idx = (rawACDir - 1)/2;
        if ((idx < 4) &&
            (irSeeker.rawACStrength[idx] != 0) &&
            (irSeeker.rawACStrength[idx + 1] != 0))
        {
            irSeeker.acDir += (float)(irSeeker.rawACStrength[idx + 1] -
                                      irSeeker.rawACStrength[idx])/
                              max(irSeeker.rawACStrength[idx],
                                  irSeeker.rawACStrength[idx + 1]);
            if (irSeeker.options & IRSEEKERO_FILTER)
            {
                irSeeker.acDir = (float)KalmanFilter(irSeeker.kalman,
                                                     irSeeker.acDir);
            }
            irSeeker.acStrength = (rawACDir%2 == 1)?
                                    irSeeker.rawACStrength[idx]:
                                    (irSeeker.rawACStrength[idx] +
                                     irSeeker.rawACStrength[idx + 1])/2;
        }
    }

    TExit();
    return;
}   //IRSeekerTask

/**
 *  This function initializes the IR Seeker sensor.
 *
 *  @param irSeeker Points to the IRSEEKER structure.
 *  @param sensorID Specifies the ID of the IR Seeker sensor.
 *  @param options Optionally specifies the IRSeeker options:
 *         IRSEEKERO_FILTER - Apply filter to the IRSeeker acDir reading.
 */
void
IRSeekerInit(
    IRSEEKER &irSeeker,
    tSensors sensorID,
    int options = 0
    )
{
    TFuncName("IRSeekerInit");
    TLevel(INIT);
    TEnter();

    irSeeker.sensorID = sensorID;
    irSeeker.options = options;
    irSeeker.flags = 0;
    irSeeker.rawACDir = 0;
    irSeeker.acDir = 0.0;
    irSeeker.acStrength = 0;
    IRSeekerTask(irSeeker);
#ifdef _KALMAN_H
    KalmanInit(irSeeker.kalman);
#endif

    TExit();
    return;
}   //IRSeekerInit

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the IR Seeker sensor.
 *
 *  @param irSeeker Points to the IRSEEKER structure.
 *  @param sensorID Specifies the sensor MUX ID of the IR Seeker sensor.
 *  @param options Optionally specifies the IRSeeker options:
 *         IRSEEKERO_FILTER - Apply filter to the IRSeeker acDir reading.
 */
void
IRSeekerInit(
    IRSEEKER &irSeeker,
    tMUXSensor sensorID,
    int options = 0
    )
{
    TFuncName("IRSeekerInit");
    TLevel(INIT);
    TEnter();

    irSeeker.sensorID = (tSensors)sensorID;
    irSeeker.options = options;
    irSeeker.flags = IRSEEKERF_HTSMUX;
    irSeeker.rawACDir = 0;
    irSeeker.acDir = 0.0;
    irSeeker.acStrength = 0;
    IRSeekerTask(irSeeker);
#ifdef _KALMAN_H
    KalmanInit(irSeeker.kalman);
#endif

    TExit();
    return;
}   //IRSeekerInit
#endif

#endif  //ifndef _IRSEEKER_H
