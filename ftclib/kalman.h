#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="kalman.h" />
///
/// <summary>
///     This module contains the library functions to implement the Kalman
///     filter.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _KALMAN_H
#define _KALMAN_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_KALMAN

//
// Type definitions.
//
typedef struct
{
    double Q;
    double R;
    double prevP;
    double prevXEst;
    bool   fInitialized;
} KALMAN;

/**
 *  This function initializes the Kalman filter.
 *
 *  @param kalman Points to the KALMAN structure.
 *  @param Q Optionally specifies the Q constant (default is 0.022).
 *  @param R Optionally specifies the R constant (default is 0.617).
 */
void
KalmanInit(
    KALMAN &kalman,
    double Q = 0.022,
    double R = 0.617
    )
{
    TFuncName("KalmanInit");
    TLevel(INIT);
    TEnter();

    kalman.Q = Q;
    kalman.R = R;
    kalman.prevP = 0.0;
    kalman.prevXEst = 0.0;
    kalman.fInitialized = false;

    TExit();
    return;
}   //KalmanInit

/**
 *  This function applies the Kalman filter to the data.
 *
 *  @param kalman Points to the KALMAN structure.
 *  @param data Specifies the data to apply the filter.
 */
double
KalmanFilter(
    KALMAN &kalman,
    double data
    )
{
    TFuncName("KalmanFilter");
    TLevel(API);
    TEnterMsg(("data=%f", data));

    if (!kalman.fInitialized)
    {
        kalman.prevXEst = data;
        kalman.fInitialized = true;
    }

    double tempP = kalman.prevP + kalman.Q;
    double K = tempP/(tempP + kalman.R);
    double xEst = kalman.prevXEst + K*(data - kalman.prevXEst);
    double P = (1 - K)*tempP;

    kalman.prevP = P;
    kalman.prevXEst = xEst;

    TExitMsg(("=%f", = kalman.prevXEst));
    return kalman.prevXEst;
}   //KalmanFilter

#endif  //ifndef _KALMAN_H
