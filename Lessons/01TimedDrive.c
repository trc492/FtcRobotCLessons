#pragma config(Sensor, S1,     soundSensor,    sensorSoundDBA)
#pragma config(Sensor, S2,     lightSensor,    sensorLightInactive)
#pragma config(Sensor, S3,     sonarSensor,    sensorSONAR)
#pragma config(Sensor, S4,     touchSensor,    sensorTouch)
#pragma config(Motor,  motorA,          rightMotor,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
    motor[leftMotor] = 30;
    motor[rightMotor] = 30;
    wait1Msec(1000);

    motor[leftMotor] = -30;
    motor[rightMotor] = -30;
    wait1Msec(1000);

    motor[leftMotor] = 30;
    motor[rightMotor] = -30;
    wait1Msec(500);

    motor[leftMotor] = -30;
    motor[rightMotor] = 30;
    wait1Msec(500);
}