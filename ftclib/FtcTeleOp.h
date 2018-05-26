 #if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="FtcTeleOp.h" />
///
/// <summary>
///     This module contains the main entry point for FTC TeleOp mode.
///     This file is included by the competition main template.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

/**
 *  This task is the program entry point.
 */
task main()
{
    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    //
    // RobotInit contains code to initialize the robot.
    //
    RobotInit();

    //
    // Wait for the beginning of autonomous mode.
    //
#if defined(_Target_Robot_)
    waitForStart();
#endif

    unsigned long startTime = nPgmTime;
    unsigned long nextPeriod = startTime;

    //
    // StartMode contains code to start TeleOp and do TeleOp specific
    // setup.
    //
    eraseDisplay();
    StartMode(PGMMODE_TELEOP);
    while (true)
    {
        //
        // HiFreqTasks contains tasks that require high resolution processing.
        //
        HiFreqTasks(PGMMODE_TELEOP);

        if (nPgmTime >= nextPeriod)
        {
            nextPeriod += LOOP_PERIOD;
            getJoystickSettings(joystick);
            //
            // InputTasks contains tasks that process sensors and inputs.
            //
            InputTasks(PGMMODE_TELEOP);

            //
            // MainTasks contains tasks for the competition.
            //
            MainTasks(PGMMODE_TELEOP,
                      (float)(nPgmTime - startTime)/1000.0);

            //
            // OutputTasks contains tasks that process motors and actuators.
            //
            OutputTasks(PGMMODE_TELEOP);
        }
        EndTimeSlice();
    }

    //
    // StopMode contains code to stop Autonomous and do Autonomous specific
    // cleanup.
    //
    StopMode(PGMMODE_TELEOP);
}   //main
