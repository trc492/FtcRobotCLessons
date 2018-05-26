#pragma config(Sensor, S1,     soundSensor,    sensorSoundDBA)
#pragma config(Sensor, S2,     lightSensor,    sensorLightInactive)
#pragma config(Sensor, S3,     sonarSensor,    sensorSONAR)
#pragma config(Sensor, S4,     touchSensor,    sensorTouch)
#pragma config(Motor,  motorA,          rightMotor,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "..\ftclib\trcdefs.h"
#include "..\ftclib\dbgtrace.h"

#include "..\ftclib\sm.h"
#include "..\ftclib\drive.h"
#include "..\ftclib\pidctrl.h"
#include "..\ftclib\piddrive.h"
#include "..\ftclib\song.h"

#define CLICKS_PER_INCH                 53.0
#define CLICKS_PER_DEGREE               3.52

#define ENCODER_DRIVE_KP                15.0
#define ENCODER_DRIVE_KI                0.0
#define ENCODER_DRIVE_KD                0.0
#define ENCODER_DRIVE_TOLERANCE         1.0
#define ENCODER_DRIVE_SETTLING          200

#define ENCODER_TURN_KP                 1.0
#define ENCODER_TURN_KI                 0.0
#define ENCODER_TURN_KD                 0.0
#define ENCODER_TURN_TOLERANCE          1.0
#define ENCODER_TURN_SETTLING           200

#define EVTTYPE_PIDDRIVE                (EVTTYPE_NONE + 1)
#define EVTTYPE_SOUND                   (EVTTYPE_NONE + 2)

#define BAR_DURATION                    1920

//
// Global data.
//
DRIVE       g_drive;
PIDCTRL     g_pidCtrlEncoderDrive;
PIDCTRL     g_pidCtrlEncoderTurn;
PIDDRIVE    g_encoderDrive;
SM          g_sm;

char g_Segment1[] =
    "G4.12,G4.12,G4.12,"
    "C5.2,G5.2,"
    "F5.12,E5.12,D5.12,C6.2,G5.4,"
    "F5.12,E5.12,D5.12,C6.2,G5.4,"
    "F5.12,E5.12,F5.12,D5.2,G4.8,G4.8,"
    "C5.2,G5.2";
char g_Segment2[] =
    "F5.12,E5.12,D5.12,C6.2,G5.4,"
    "F5.12,E5.12,D5.12,C6.2,G5.4,"
    "F5.12,E5.12,F5.12,D5.2,G4.8,G4.8,"
    "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
    "C5.12,D5.12,E5.12,D5.4,B4.4,G4.8,G4.8,"
    "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
    "G5.4,D5.2,G4.8,G4.8";
char g_Segment3[] =
    "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
    "C5.12,D5.12,E5.12,D5.4,B4.4,G5.8,G5.8,"
    "C6.8,Bb5.8,Ab5.8,G5.8,F5.8,Eb5.8,D5.8,C5.8,"
    "G5.2+";

char *g_StarWarsSegments[] =
{
    g_Segment1,
    g_Segment2,
    g_Segment3,
    NULL
};

SONG g_StarWars;

float PIDCtrlGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;

    if (&pidCtrl == &g_pidCtrlEncoderDrive)
    {
        inputValue = (nMotorEncoder[leftMotor] + nMotorEncoder[rightMotor])/
                     (2.0*CLICKS_PER_INCH);
    }
    else if (&pidCtrl == &g_pidCtrlEncoderTurn)
    {
        inputValue = (nMotorEncoder[leftMotor] - nMotorEncoder[rightMotor])/
                     CLICKS_PER_DEGREE;
    }

    return inputValue;
}   //PIDCtrlGetInput

void RobotInit()
{
    nMotorEncoder[leftMotor] = 0;
    nMotorEncoder[rightMotor] = 0;
    DriveInit(g_drive, leftMotor, rightMotor);
    PIDCtrlInit(g_pidCtrlEncoderDrive,
                ENCODER_DRIVE_KP, ENCODER_DRIVE_KI, ENCODER_DRIVE_KD,
                ENCODER_DRIVE_TOLERANCE, ENCODER_DRIVE_SETTLING);
    PIDCtrlInit(g_pidCtrlEncoderTurn,
                ENCODER_TURN_KP, ENCODER_TURN_KI, ENCODER_TURN_KD,
                ENCODER_TURN_TOLERANCE, ENCODER_TURN_SETTLING);
    PIDDriveInit(g_encoderDrive,
                 g_drive,
                 g_pidCtrlEncoderDrive,
                 g_pidCtrlEncoderTurn);

    SongInit(g_StarWars, &g_StarWarsSegments[0]);

    SMInit(g_sm);
    SMStart(g_sm);
}   //RobotInit

void HiFreqTasks()
{
    SongTask(g_StarWars);
}   //HiFreqTasks

void InputTasks()
{
}   //InputTasks

void MainTasks()
{
    if (SMIsReady(g_sm))
    {
        int currState = SMGetState(g_sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                SongStart(g_StarWars, BAR_DURATION, SONGO_REPEAT);
                PIDDriveSetTarget(g_encoderDrive,
                                  24.0,     //distance in inches
                                  0.0,      //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                PIDDriveSetTarget(g_encoderDrive,
                                  0.0,      //distance in inches
                                  90.0,     //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                PIDDriveSetTarget(g_encoderDrive,
                                  12.0,     //distance in inches
                                  0.0,      //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 3:
                PIDDriveSetTarget(g_encoderDrive,
                                  0.0,      //distance in inches
                                  90.0,     //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                PIDDriveSetTarget(g_encoderDrive,
                                  24.0,     //distance in inches
                                  0.0,      //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                PIDDriveSetTarget(g_encoderDrive,
                                  0.0,      //distance in inches
                                  90.0,     //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 6:
                PIDDriveSetTarget(g_encoderDrive,
                                  12.0,     //distance in inches
                                  0.0,      //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            case SMSTATE_STARTED + 7:
                PIDDriveSetTarget(g_encoderDrive,
                                  0.0,      //distance in inches
                                  90.0,     //turn in degrees
                                  false,
                                  &g_sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(g_sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(g_sm, currState + 1);
                break;

            default:
                SongStop(g_StarWars);
                SMStop(g_sm);
                break;
        }
    }
}   //MainTasks

void OutputTasks()
{
    PIDDriveTask(g_encoderDrive);
    DriveTask(g_drive);
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
        nxtDisplayTextLine(0, "D=%6.1f", PIDCtrlGetInput(g_pidCtrlEncoderDrive));
        nxtDisplayTextLine(1, "A=%6.1f", PIDCtrlGetInput(g_pidCtrlEncoderTurn));
        EndTimeSlice();
    }
}   //main