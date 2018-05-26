#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="FtcMain.h" />
///
/// <summary>
///     This module contains the main entry point for the FTC competition.
///     This file is included by the competition main template.
///     It supports three different modes:
///     - Autonomous mode only
///     - TeleOp mode only
///     - Both modes
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

/**
 *  This function determines the current program mode.
 *
 *  @return Returns current program mode.
 */
int GetPgmMode()
{
#if defined(_Target_Robot_)
    getJoystickSettings(joystick);
    return joystick.StopPgm? PGMMODE_DISABLED:
           joystick.UserMode? PGMMODE_TELEOP: PGMMODE_AUTONOMOUS;
#else
    getJoystickSettings(joystick);
  #if defined(_AUTO_ONLY)
    return PGMMODE_AUTONOMOUS;
  #else
    #if defined(_TELEOP_ONLY)
    return PGMMODE_TELEOP;
    #endif
  #endif
#endif
}   //GetPgmMode

/**
 *  This task is the program entry point.
 */
task main()
{
    int prevMode = PGMMODE_DISABLED;
    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    //
    // RobotInit contains code to initialize the robot.
    //
    RobotInit();

    unsigned long startTime = nPgmTime;
    unsigned long nextPeriod = startTime;

    while (true)
    {
        int currMode = GetPgmMode();

        if (currMode != prevMode)
        {
            //
            // The program mode has changed.
            //
            if (prevMode != PGMMODE_DISABLED)
            {
                //
                // StopMode contains code to stop the previous mode and do
                // mode specific cleanup.
                //
                StopMode(prevMode);
            }

            if (currMode != PGMMODE_DISABLED)
            {
                //
                // StartMode contains code to start the current mode and do
                // mode specific setup.
                //
                eraseDisplay();
                StartMode(currMode);
            }

            prevMode = currMode;
            startTime = nPgmTime;
        }

        //
        // HiFreqTasks contains tasks that require high resolution processing.
        //
        HiFreqTasks(currMode);

        if (nPgmTime >= nextPeriod)
        {
            nextPeriod += LOOP_PERIOD;
            //
            // InputTasks contains tasks that process sensors and inputs.
            //
            InputTasks(currMode);

            //
            // MainTasks contains main tasks for the competition.
            //
            MainTasks(currMode,
                      (float)(nPgmTime - startTime)/1000.0);

            if (currMode != PGMMODE_DISABLED)
            {
                //
                // OutputTasks contains tasks that process motors and actuators.
                //
                OutputTasks(currMode);
            }
        }

        EndTimeSlice();
    }
}   //main
