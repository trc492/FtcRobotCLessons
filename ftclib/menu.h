#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="menu.h" />
///
/// <summary>
///     This module contains the library functions for displaying a choice
///     menu on the NXT LCD screen and allowing the user to scroll through
///     the menu and pick a choice.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _MENU_H
#define _MENU_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MENU

//
// Constants
//
#ifndef MAX_NUM_CHOICES
    #define MAX_NUM_CHOICES     16
#endif
#define NUM_DISPLAY_LINES       8
#define NUM_PIXELS_PER_LINE     8
#define NUM_LINE_PIXELS         100
#define NUM_CHAR_PER_LINE       17

#define MENUF_ENABLED           0x0001

//
// Macros.
//

/**
 *  This macro translates the LCD line number to pixel row.
 *
 *  @param n Specifies the LCD line number.
 *
 *  @return Returns the pixel row of a given LCD text line.
 */
#define LinePos(n)              (NUM_DISPLAY_LINES*NUM_PIXELS_PER_LINE - 1 - \
                                 (n)*NUM_PIXELS_PER_LINE)

/**
 *  This macro checks if the Menu is enabled.
 *
 *  @param m Points to the MENU structure.
 *
 *  @return Returns true if Menu is enabled.
 */
#define MenuIsEnabled(m)        (((m).flags & MENUF_ENABLED) != 0)

/**
 *  This macro gets the menu index of the user choice.
 *
 *  @param m Points to the MENU structure.
 *
 *  @return Returns the user choice index.
 */
#define MenuGetChoiceIndex(m)   ((m).userChoice)

/**
 *  This macro gets the user choice value of the menu.
 *
 *  @param m Points to the MENU structure.
 *
 *  @return Returns the user choice value.
 */
#define MenuGetChoiceValue(m)   ((m).choiceValues[(m).userChoice])

/**
 *  This macro returns the text associated with the user choice of the menu.
 *
 *  @param m Points to the MENU structure.
 *
 *  @return Returns the text associated with the user choice.
 */
#define MenuGetChoiceText(m)    (&(m).choiceTexts[(m).userChoice][0])

//
// Type definitions.
//
typedef struct
{
    char     titleText[NUM_CHAR_PER_LINE + 1];
    int      flags;
    int      numChoices;
    char     choiceTexts[MAX_NUM_CHOICES][NUM_CHAR_PER_LINE + 1];
    float    choiceValues[MAX_NUM_CHOICES];
    int      currChoice;
    int      firstChoice;
    int      userChoice;
} MENU;

/**
 *  This function initializes the choice menu.
 *
 *  @param menu Points to the MENU structure.
 *  @param titleText Specifies the title text of the menu.
 */
void
MenuInit(
    MENU &menu,
    const char *titleText
    )
{
    TFuncName("MenuInit");
    TLevel(INIT);
    TEnter();

    strcpy(menu.titleText, titleText);
    menu.flags = 0;
    menu.numChoices = 0;
    menu.currChoice = 0;
    menu.firstChoice = 0;
    menu.userChoice = -1;

    TExit();
    return;
}   //MenuInit

/**
 *  This function adds a choice to the menu.
 *
 *  @param menu Points to the MENU structure.
 *  @param choiceText Specifies the choice text.
 *  @param choiceValue Specifies the value to return if chosen.
 *
 *  @return Returns true if the choice is added successful, otherwise the
 *          menu is full.
 */
bool
MenuAddChoice(
    MENU &menu,
    const char *choiceText,
    float choiceValue
    )
{
    bool fSuccess = false;

    TFuncName("MenuAddChoice");
    TLevel(API);
    TEnter();

    if (menu.numChoices < MAX_NUM_CHOICES)
    {
        strcpy(&menu.choiceTexts[menu.numChoices][0], choiceText);
        menu.choiceValues[menu.numChoices] = choiceValue;
        menu.numChoices++;
        fSuccess = true;
    }

    TExit();
    return fSuccess;
}   //MenuAddChoice

/**
 *  This function waits for the start signal from the FCS or the enter button
 *  is pressed.
 */
void
MenuWaitStart()
{
    TFuncName("MenuWaitStart");
    TLevel(API);
    TEnter();

#if defined(_Target_Robot_)
    TButtons prevNxtBtn = nNxtButtonPressed;
    TButtons currNxtBtn;

    while (true)
    {
        getJoystickSettings(joystick);
        currNxtBtn = nNxtButtonPressed;
        if (!joystick.StopPgm ||
            (currNxtBtn != prevNxtBtn) && (currNxtBtn == kEnterButton))
        {
            break;
        }
        prevNxtBtn = currNxtBtn;
    }
#endif

    TExit();
    return;
}   //MenuWaitStart

/**
 *  This function invert the menu line.
 *
 *  @param lineNum
 */
void
MenuInvertLine(
    int lineNum
    )
{
    TFuncName("MenuInvertLine");
    TLevel(FUNC);
    TEnter();

    for (int y = LinePos(lineNum + 1) + 1; y <= LinePos(lineNum); y++)
    {
        nxtInvertLine(0, y, NUM_LINE_PIXELS -1, y);
    }

    TExit();
    return;
}   //MenuInvertLine

/**
 *  This function is called to refresh the menu display.
 *
 *  @param menu Points to the MENU structure.
 */
void
MenuDisplay(
    MENU &menu
    )
{
    TFuncName("MenuDisplay");
    TLevel(API);
    TEnter();

    int lastChoice = min(menu.firstChoice + NUM_DISPLAY_LINES - 2,
                         menu.numChoices - 1);
    eraseDisplay();
    nxtDisplayTextLine(0, menu.titleText);
    for (int choice = menu.firstChoice; choice <= lastChoice; choice++)
    {
        nxtDisplayTextLine(choice - menu.firstChoice + 1,
                           "%d:%s", choice, &menu.choiceTexts[choice][0]);
    }
    MenuInvertLine(menu.currChoice - menu.firstChoice + 1);

    TExit();
    return;
}   //MenuDisplay

/**
 *  This function is called to enable or disable the menu.
 *
 *  @param menu Points to the MENU structure.
 *  @param fEnable If true, display and enable the menu, otherwise erase and
 *         disable the menu.
 */
void
MenuSetState(
    MENU &menu,
    bool fEnable
    )
{
    TFuncName("MenuSetState");
    TLevel(API);
    TEnterMsg(("fEnable=%d", fEnable));

    if (fEnable)
    {
        menu.userChoice = -1;
        MenuDisplay(menu);
        menu.flags |= MENUF_ENABLED;
    }
    else
    {
        menu.userChoice = menu.currChoice;
        menu.flags &= ~MENUF_ENABLED;
        eraseDisplay();
    }

    TExit();
    return;
}   //MenuSetState

/**
 *  This function set the current choice to next or previous.
 *
 *  @param menu Points to the MENU structure.
 *  @param inc Specifies the increment.
 */
void
MenuIncChoice(
    MENU &menu,
    int inc
    )
{
    TFuncName("MenuIncChoice");
    TLevel(API);
    TEnter();

    menu.currChoice += inc;
    if (menu.currChoice < 0)
    {
        menu.currChoice += menu.numChoices;
    }
    else if (menu.currChoice >= menu.numChoices)
    {
        menu.currChoice -= menu.numChoices;
    }

    int lineNum = menu.currChoice - menu.firstChoice + 1;
    if (lineNum >= NUM_DISPLAY_LINES)
    {
        menu.firstChoice = menu.currChoice - (NUM_DISPLAY_LINES - 2);
    }
    else if (lineNum < 1)
    {
        menu.firstChoice = menu.currChoice;
    }
    MenuDisplay(menu);

    TExit();
    return;
}   //MenuIncChoice

/**
 *  This function displays the menu and returns the user choice.
 *  Note that this is a blocking function. It should only be called outside
 *  of the robot loop (e.g. in RobotInit).
 *
 *  @param menu Points to the MENU structure.
 *
 *  @return Returns the user choice.
 */
float
MenuGetChoice(
    MENU &menu
    )
{
    TButtons prevNxtBtn = nNxtButtonPressed;
    TButtons currNxtBtn;

    TFuncName("MenuGetChoice");
    TLevel(API);
    TEnter();

    MenuSetState(menu, true);
    while (true)
    {
        currNxtBtn = nNxtButtonPressed;
        if (currNxtBtn != prevNxtBtn)
        {
            if (currNxtBtn == kLeftButton)
            {
                MenuIncChoice(menu, -1);
            }
            else if (currNxtBtn == kRightButton)
            {
                MenuIncChoice(menu, 1);
            }
            else if (currNxtBtn == kEnterButton)
            {
                break;
            }
            prevNxtBtn = currNxtBtn;
            wait1Msec(NXTBTN_DEBOUNCE_TIME);
        }
    }
    MenuSetState(menu, false);

    TExit();
    return menu.choiceValues[menu.userChoice];
}   //MenuGetChoice

#endif  //ifndef _MENU_H
