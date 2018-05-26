#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="encoder.h" />
///
/// <summary>
///     This module contains the library functions for the encoder.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ENCODER_H
#define _ENCODER_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ENCODER

//
// Constants.
//
#ifndef ENCODER_SAMPLING_PERIOD
#define ENCODER_SAMPLING_PERIOD 5
#endif

//
// Macros
//
/**
 *  This macro returns the distance.
 *
 *  @param e Points to the ENCODER structure.
 *
 *  @return Returns the distance.
 */
#define EncoderGetDistance(e)   ((e).distance)

/**
 *  This macro returns the speed.
 *
 *  @param e Points to the ENCODER structure.
 *
 *  @return Returns the speed.
 */
#define EncoderGetSpeed(e)      ((e).speed)


//
// Type definitions.
//
typedef struct
{
    tMotor          motorID;
    float           distPerClick;
    int             prevValue;
    unsigned long   prevTime;
    unsigned long   nextPeriod;
    float           distance;
    float           speed;
} ENCODER;

/**
 *  This function performs the encoder task where it takes a derivative of
 *  the reading against time to calculate speed.
 *
 *  @param encoder Points to the ENCODER structure.
 */
void
EncoderTask(
    ENCODER &encoder
    )
{
    TFuncName("EnocderTask");
    TLevel(TASK);
    TEnter();

    unsigned long currTime = nPgmTime;
    if (currTime >= encoder.nextPeriod)
    {
        int currValue = nMotorEncoder[encoder.motorID];

        encoder.nextPeriod += ENCODER_SAMPLING_PERIOD;
        encoder.distance = (float)(currValue*encoder.distPerClick);
        encoder.speed = (float)(currValue - encoder.prevValue)*
                        encoder.distPerClick*1000.0/
                        (currTime - encoder.prevTime);
        encoder.prevValue = currValue;
        encoder.prevTime = currTime;
    }

    TExit();
    return;
}   //EnocderTask

/**
 *  This function resets the encoder.
 *
 *  @param encoder Points to the ENCODER structure to be reset.
 */
void
EncoderReset(
    ENCODER &encoder
    )
{
    TFuncName("EncoderReset");
    TLevel(API);
    TEnter();

    nMotorEncoder[encoder.motorID] = 0;
    encoder.prevValue = 0;
    encoder.prevTime = nPgmTime;
    encoder.nextPeriod = encoder.prevTime;
    EncoderTask(encoder);

    TExit();
    return;
}   //EncoderReset

/**
 *  This function initializes the encoder.
 *
 *  @param encoder Points to the ENCODER structure.
 *  @param motorID Specifies the motor ID associated with the encoder.
 *  @param distPerClick Specifies the distance per encoder click.
 */
void
EncoderInit(
    ENCODER &encoder,
    tMotor motorID,
    float distPerClick
    )
{
    TFuncName("EncoderInit");
    TLevel(INIT);
    TEnter();

    encoder.motorID = motorID;
    encoder.distPerClick = distPerClick;
    encoder.prevValue = nMotorEncoder[motorID];
    encoder.prevTime = nPgmTime;
    encoder.nextPeriod = encoder.prevTime;
    EncoderTask(encoder);

    TExit();
    return;
}   //EncoderInit

#endif  //ifndef _ENCODER_H
