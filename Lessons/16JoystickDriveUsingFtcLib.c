#pragma config(Sensor, S1,     soundSensor,    sensorSoundDBA)
#pragma config(Sensor, S2,     lightSensor,    sensorLightInactive)
#pragma config(Sensor, S3,     sonarSensor,    sensorSONAR)
#pragma config(Sensor, S4,     touchSensor,    sensorTouch)
#pragma config(Motor,  motorA,          rightMotor,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "..\ftclib\trcdefs.h"
#include "..\ftclib\dbgtrace.h"
#include "..\ftclib\joystick.h"
#include "..\ftclib\drive.h"

JOYSTICK    g_Joystick;
DRIVE       g_Drive;

void JoyBtnEvent(int joystickID, int eventType, int eventID, bool fPressed)
{
    if (joystickID == 1)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            nxtDisplayTextLine(2, "Btn=%d,Press=%d", eventID, fPressed);
            switch (eventID)
            {
                case Xbox_A:
                    break;
            }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
            {
                case TopHat_Up:
                    break;
            }
        }
    }
}   //JoyBtnEvent

void RobotInit()
{
    JoystickInit(g_Joystick, 1);
    DriveInit(g_Drive, leftMotor, rightMotor);
}   //RobotInit

void HiFreqTasks()
{
}   //HiFreqTasks

void InputTasks()
{
    getJoystickSettings(joystick);
    JoystickTask(g_Joystick);
}   //InputTasks

void MainTasks()
{
    int drivePower = JoystickGetY1WithDeadband(g_Joystick, true);
    int turnPower = JoystickGetX1WithDeadband(g_Joystick, true);
    nxtDisplayTextLine(0, "Arcade: %d,%d", drivePower, turnPower);
    DriveArcade(g_Drive, drivePower, turnPower);
}   //MainTasks

void OutputTasks()
{
    DriveTask(g_Drive);
}   //OutputTasks

task main()
{
    unsigned long nextPeriod;

    RobotInit();
    nextPeriod = nPgmTime;
    while (true)
    {
        HiFreqTasks();
        if (nPgmTime >= nextPeriod)
        {
            nextPeriod += LOOP_PERIOD;
            InputTasks();
            MainTasks();
            OutputTasks();
        }
        EndTimeSlice();
    }
}   //main
