#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="joystick.h" />
///
/// <summary>
///     This module contains the library functions for the joysticks.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _JOYSTICK_H
#define _JOYSTICK_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_JOYSTICK

//
// Constants.
//
#ifndef NUM_BTNS
  #define NUM_BTNS              12
#endif

#define JOYEVENT_BUTTON         0
#define JOYEVENT_TOPHAT         1

#define Logitech_Btn1           Bit(0)
#define Logitech_Btn2           Bit(1)
#define Logitech_Btn3           Bit(2)
#define Logitech_Btn4           Bit(3)
#define Logitech_LB5            Bit(4)
#define Logitech_RB6            Bit(5)
#define Logitech_LB7            Bit(6)
#define Logitech_RB8            Bit(7)
#define Logitech_Btn9           Bit(8)
#define Logitech_Btn10          Bit(9)
#define Logitech_LStick         Bit(10)
#define Logitech_RStick         Bit(11)

#define Xbox_A                  Bit(0)
#define Xbox_B                  Bit(1)
#define Xbox_X                  Bit(2)
#define Xbox_Y                  Bit(3)
#define Xbox_LB                 Bit(4)
#define Xbox_RB                 Bit(5)
#define Xbox_Back               Bit(6)
#define Xbox_Start              Bit(7)
#define Xbox_LStick             Bit(8)
#define Xbox_RStick             Bit(9)

#define TopHat_Released         -1
#define TopHat_Up               0
#define TopHat_UpRight          1
#define TopHat_Right            2
#define TopHat_DownRight        3
#define TopHat_Down             4
#define TopHat_DownLeft         5
#define TopHat_Left             6
#define TopHat_UpLeft           7

//
// Type definitions.
//
typedef struct
{
    int  joystickID;
    int  prevButtons;
    int  prevTopHat;
} JOYSTICK;

//
// Callback function prototypes.
//

/**
 *  This callback function is called when a joystick button is pressed or
 *  released.
 *
 *  @param joystickID Specifies the joystick ID.
 *  @param eventType Specifies the joystick event type:
 *         JOYEVENT_BUTTON - Specifies a button event
 *         JOYEVENT_TOPHAT - Specifies a tophat event
 *  @param eventID Specifies the button or tophat event ID.
 *  @param fPressed TRUE if a pressed event, otherwise it is a released event.
 */
void
JoyBtnEvent(
    int  joystickID,
    int  eventType,
    int  eventID,
    bool fPressed
    );

/**
 *  This function initializes the joystick device.
 *
 *  @param js Points to the JOYSTICK structure.
 *  @param joystickID Specifies the joystick ID.
 */
void
JoystickInit(
    JOYSTICK &js,
    int joystickID
    )
{
    TFuncName("JoystickInit");
    TLevel(INIT);
    TEnter();

    js.joystickID = joystickID;
#if defined(_Target_Robot_)
    js.prevButtons = (joystickID == 1)? joystick.joy1_Buttons:
                                        joystick.joy2_Buttons;
    js.prevTopHat = (joystickID == 1)? joystick.joy1_TopHat:
                                       joystick.joy2_TopHat;
#else
    js.prevButtons = joystick.joy1_Buttons;
    js.prevTopHat = joystick.joy1_TopHat;
#endif

    TExit();
    return;
}   //JoystickInit

/**
 *  This function processes the changed buttons and sends button event
 *  notifications.
 *
 *  @param js Points to the JOYSTICK structure.
 */
void
JoystickTask(
    JOYSTICK &js
    )
{
#if defined(_Target_Robot_)
    int currButtons = (js.joystickID == 1)? joystick.joy1_Buttons:
                                            joystick.joy2_Buttons;
    int currTopHat = (js.joystickID == 1)? joystick.joy1_TopHat:
                                           joystick.joy2_TopHat;
#else
    int currButtons = joystick.joy1_Buttons;
    int currTopHat = joystick.joy1_TopHat;
#endif

    TFuncName("JoystickTask");
    TLevel(TASK);
    TEnterMsg(("Prev=%x,Curr=%x", js.prevButtons, currButtons));

    //
    // If callback is not enabled, there is nothing to do except for
    // updating the button values.
    //
#if defined(_Target_Robot_)
    if (!bDisconnected)
#endif
    {
        int changedButtons = currButtons^js.prevButtons;
        int buttonMask;

        while (changedButtons != 0)
        {
            //
            // maskButton contains the least significant set bit.
            //
            buttonMask = changedButtons & ~(changedButtons^-changedButtons);
            if ((currButtons & buttonMask) != 0)
            {
                //
                // Button is pressed.
                //
                JoyBtnEvent(js.joystickID,
                            JOYEVENT_BUTTON,
                            buttonMask,
                            true);
            }
            else
            {
                //
                // Button is released.
                //
                JoyBtnEvent(js.joystickID,
                            JOYEVENT_BUTTON,
                            buttonMask,
                            false);
            }
            changedButtons &= ~buttonMask;
        }

        if (currTopHat != js.prevTopHat)
        {
            JoyBtnEvent(js.joystickID,
                        JOYEVENT_TOPHAT,
                        currTopHat,
                        currTopHat != -1);
        }
    }

    js.prevButtons = currButtons;
    js.prevTopHat = currTopHat;

    TExit();
    return;
}   //JoyBtnTask

/**
 *  This function gets the X1 value of the joystick with deadband.
 *
 *  @param js Points to the JOYSTICK structure.
 *  @param fSquared Optionally specifies if the joystick value should be
 *         squared.
 *  @param fNormalize Optionally specifies if the joystick value should be
 *         normalize to the motor value.
 *  @param threshold Optionally specifies the deadband threshold.
 */
int
JoystickGetX1WithDeadband(
    JOYSTICK &js,
    bool fSquared = false,
    bool fNormalize = true,
    int threshold = DEADBAND_INPUT_THRESHOLD
    )
{
    TFuncName("JoystickGetX1WithDeadband");
    TLevel(API);
    TEnter();

#if defined(_Target_Robot_)
    int value = DEADBAND((js.joystickID == 1)? joystick.joy1_x1:
                                               joystick.joy2_x1,
                         threshold);
#else
    int value = DEADBAND(joystick.joy1_x1, threshold);
#endif

    if (fSquared)
    {
        int sign = (value >= 0)? 1: -1;
        float adjValue = (float)value/128;
        value = (int)(sign*adjValue*adjValue*128);
    }

    if (fNormalize)
    {
        value = NORMALIZE_DRIVE(value, -128, 127);
    }

    TExitMsg(("=%d", value));
    return value;
}   //JoystickGetX1WithDeadband

/**
 *  This function gets the Y1 value of the joystick with deadband.
 *
 *  @param js Points to the JOYSTICK structure.
 *  @param fSquared Optionally specifies if the joystick value should be
 *         squared.
 *  @param fNormalize Optionally specifies if the joystick value should be
 *         normalize to the motor value.
 *  @param threshold Optionally specifies the deadband threshold.
 */
int
JoystickGetY1WithDeadband(
    JOYSTICK &js,
    bool fSquared = false,
    bool fNormalize = true,
    int threshold = DEADBAND_INPUT_THRESHOLD
    )
{
    TFuncName("JoystickGetY1WithDeadband");
    TLevel(API);
    TEnter();

#if defined(_Target_Robot_)
    int value = DEADBAND((js.joystickID == 1)? joystick.joy1_y1:
                                               joystick.joy2_y1,
                         threshold);
#else
    int value = DEADBAND(joystick.joy1_y1, threshold);
#endif

    if (fSquared)
    {
        int sign = (value >= 0)? 1: -1;
        float adjValue = (float)value/128;
        value = (int)(sign*adjValue*adjValue*128);
    }

    if (fNormalize)
    {
        value = NORMALIZE_DRIVE(value, -128, 127);
    }

    TExitMsg(("=%d", value));
    return value;
}   //JoystickGetY1WithDeadband

/**
 *  This function gets the X2 value of the joystick with deadband.
 *
 *  @param js Points to the JOYSTICK structure.
 *  @param fSquared Optionally specifies if the joystick value should be
 *         squared.
 *  @param fNormalize Optionally specifies if the joystick value should be
 *         normalize to the motor value.
 *  @param threshold Optionally specifies the deadband threshold.
 */
int
JoystickGetX2WithDeadband(
    JOYSTICK &js,
    bool fSquared = false,
    bool fNormalize = true,
    int threshold = DEADBAND_INPUT_THRESHOLD
    )
{
    TFuncName("JoystickGetX2WithDeadband");
    TLevel(API);
    TEnter();

#if defined(_Target_Robot_)
    int value = DEADBAND((js.joystickID == 1)? joystick.joy1_x2:
                                               joystick.joy2_x2,
                         threshold);
#else
    int value = DEADBAND(joystick.joy1_x2, threshold);
#endif

    if (fSquared)
    {
        int sign = (value >= 0)? 1: -1;
        float adjValue = (float)value/128;
        value = (int)(sign*adjValue*adjValue*128);
    }

    if (fNormalize)
    {
        value = NORMALIZE_DRIVE(value, -128, 127);
    }

    TExitMsg(("=%d", value));
    return value;
}   //JoystickGetX2WithDeadband

/**
 *  This function gets the Y2 value of the joystick with deadband.
 *
 *  @param js Points to the JOYSTICK structure.
 *  @param fSquared Optionally specifies if the joystick value should be
 *         squared.
 *  @param fNormalize Optionally specifies if the joystick value should be
 *         normalize to the motor value.
 *  @param threshold Optionally specifies the deadband threshold.
 */
int
JoystickGetY2WithDeadband(
    JOYSTICK &js,
    bool fSquared = false,
    bool fNormalize = true,
    int threshold = DEADBAND_INPUT_THRESHOLD
    )
{
    TFuncName("JoystickGetY2WithDeadband");
    TLevel(API);
    TEnter();

#if defined(_Target_Robot_)
    int value = DEADBAND((js.joystickID == 1)? joystick.joy1_y2:
                                               joystick.joy2_y2,
                         threshold);
#else
    int value = DEADBAND(joystick.joy1_y2, threshold);
#endif

    if (fSquared)
    {
        int sign = (value >= 0)? 1: -1;
        float adjValue = (float)value/128;
        value = (int)(sign*adjValue*adjValue*128);
    }

    if (fNormalize)
    {
        value = NORMALIZE_DRIVE(value, -128, 127);
    }

    TExitMsg(("=%d", value));
    return value;
}   //JoystickGetY2WithDeadband

#endif  //ifndef _JOYSTICK_H
