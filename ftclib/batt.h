#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="batt.h" />
///
/// <summary>
///     This module contains the library functions for dealing with the NXT
///     battery and the external battery.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _BATT_H
#define _BATT_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_BATT

//
// Constants.
//
#define BATTO_SHOW_DETAILS      0x0001

#define BATTF_ENABLED           0x0001

//
// Type definitions.
//
typedef struct
{
    int summaryLineNum;
    int options;
    int flags;
    int minIntBatt;
    int maxIntBatt;
    int minExtBatt;
    int maxExtBatt;
    int intBatt;
    int extBatt;
} BATT;

/**
 *  This function display the battery info on the NXT LCD screen.
 *
 *  @param batt Points to the BATT structure.
 */
void
BattShowInfo(
    BATT &batt
    )
{
    TFuncName("BattShowInfo");
    TLevel(FUNC);
    TEnter();

    nxtDisplayTextLine(batt.summaryLineNum, "Int:%3.1f Ext:%4.1f",
                       (float)batt.intBatt/1000.0,
                       (float)batt.extBatt/1000.0);

    if (batt.options & BATTO_SHOW_DETAILS)
    {
        nxtDisplayTextLine(batt.summaryLineNum + 1, "%4.1f<=Int<=%4.1f",
                           (float)batt.minIntBatt/1000.0,
                           (float)batt.maxIntBatt/1000.0);
        nxtDisplayTextLine(batt.summaryLineNum + 2, "%4.1f<=Ext<=%4.1f",
                           (float)batt.minExtBatt/1000.0,
                           (float)batt.maxExtBatt/1000.0);
    }

    TExit();
    return;
}   //BattShowInfo

/**
 *  This function initializes the BATT info.
 *
 *  @param batt Specifies the BATT structure.
 *  @param summaryLineNum If positive, a battery summary line will be displayed
 *         at the given line number. If -1, no battery summary is displayed.
 *  @param options Optionally specifies the battery options:
 *         BATTO_SHOW_DETAILS - Specifies showing the battery voltage ranges.
 */
void
BattInit(
    BATT &batt,
    int summaryLineNum,
    int options = 0
    )
{
    TFuncName("BattInit");
    TLevel(INIT);
    TEnter();

    batt.summaryLineNum = summaryLineNum;
    batt.options = options;
    batt.flags = BATTF_ENABLED;
    batt.minIntBatt = nAvgBatteryLevel;
    batt.maxIntBatt = batt.minIntBatt;
    batt.minExtBatt = (externalBatteryAvg < 0)? 0: externalBatteryAvg;
    batt.maxExtBatt = batt.minExtBatt;
#if defined(_Target_Robot_)
    StopTask(displayDiagnostics);
#endif
    eraseDisplay();

    TExit();
    return;
}   //BattInit

/**
 *  This function enables or disables the display of battery info.
 *
 *  @param batt Points to the BATT structure.
 *  @param fEnable If true, enables the battery info display, false otherwise.
 */
void
BattSetState(
    BATT &batt,
    bool fEnable
    )
{
    TFuncName("BattSetState");
    TLevel(API);
    TEnter();

    if (fEnable)
    {
        batt.flags |= BATTF_ENABLED;
    }
    else
    {
        batt.flags &= ~BATTF_ENABLED;
    }

    TExit();
    return;
}   //BattSetState

/**
 *  This function checks the battery voltages against the min and max values
 *  for determining the operating voltage range. This is helpful to determine
 *  if the batteries are running low under load.
 *
 *  @param batt Points to the BATT structure.
 */
void
BattTask(
    BATT &batt
    )
{
    int currIntBatt;
    int currExtBatt;

    TFuncName("BattTask");
    TLevel(TASK);
    TEnter();

    batt.intBatt = nAvgBatteryLevel;
    batt.extBatt = (externalBatteryAvg < 0)? 0: externalBatteryAvg;

    if (batt.intBatt < batt.minIntBatt)
    {
        batt.minIntBatt = batt.intBatt;
    }
    else if (batt.intBatt > batt.maxIntBatt)
    {
        batt.maxIntBatt = batt.intBatt;
    }

    if (batt.extBatt < batt.minExtBatt)
    {
        if (batt.extBatt > 0)
        {
            //
            // If the external battery reads 0, it may mean the power switch
            // is off. We don't want to wipe out the min voltage just because
            // we turn off the power switch.
            //
            batt.minExtBatt = batt.extBatt;
        }
    }
    else if (batt.extBatt > batt.maxExtBatt)
    {
        batt.maxExtBatt = batt.extBatt;
    }

    if (batt.flags & BATTF_ENABLED)
    {
        BattShowInfo(batt);
    }

    TExit();
    return;
}   //BattTask

#endif  //ifndef _BATT_H
