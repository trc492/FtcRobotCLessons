#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="nxtbtn.h" />
///
/// <summary>
///     This module contains the library functions for handling the NXT button
///     events.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _NXTBTN_H
#define _NXTBTN_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_NXTBTN

//
// Constants.
//

//
// Type definitions.
//
typedef struct
{
    TButtons        prevBtn;
    unsigned long   debounceTimeout;
} NXTBTN;

//
// Callback function prototypes.
//

/**
 *  This callback function is called when a NXT button is pressed or
 *  released.
 *
 *  @param nxtButton Specifies the NXT button ID.
 *  @param fPressed TRUE if a pressed event, otherwise it is a released event.
 */
void
NxtBtnEvent(
    TButtons nxtButton,
    bool     fPressed
    );

/**
 *  This function initializes the NXT button system.
 *
 *  @param nxtbtn Points to the NXTBTN structure.
 */
void
NxtBtnInit(
    NXTBTN &nxtbtn
    )
{
    TFuncName("NxtBtnInit");
    TLevel(INIT);
    TEnter();

    nxtbtn.prevBtn = nNxtButtonPressed;
    nxtbtn.debounceTimeout = 0;

    TExit();
    return;
}   //NxtBtnInit

/**
 *  This function processes the changed buttons and sends button event
 *  notifications.
 *
 *  @param nxtbtn Points to the NXTBTN structure.
 */
void
NxtBtnTask(
    NXTBTN &nxtbtn
    )
{
    unsigned long currTime = nPgmTime;
    TButtons currBtn = nNxtButtonPressed;

    TFuncName("NxtBtnTask");
    TLevel(TASK);
    TEnterMsg(("Prev=%d,Curr=%d", nxtbtn.prevBtn, currBtn));

    if ((nxtbtn.debounceTimeout == 0) ||
        (currTime >= nxtbtn.debounceTimeout))
    {
        TButtons btnID;
        bool fPressed;

        nxtbtn.debounceTimeout = 0;
        if (currBtn != nxtbtn.prevBtn)
        {
            if (currBtn == kNoButton)
            {
                btnID = nxtbtn.prevBtn;
                fPressed = false;
            }
            else
            {
                btnID = currBtn;
                fPressed = true;
            }

            NxtBtnEvent(btnID, fPressed);

            nxtbtn.prevBtn = currBtn;
            nxtbtn.debounceTimeout = currTime + NXTBTN_DEBOUNCE_TIME;
        }
    }

    TExit();
    return;
}   //NxtBtnTask

#endif  //ifndef _NXTBTN_H
