#pragma config(Sensor, S1,     soundSensor,    sensorSoundDBA)
#pragma config(Sensor, S2,     lightSensor,    sensorLightInactive)
#pragma config(Sensor, S3,     sonarSensor,    sensorSONAR)
#pragma config(Sensor, S4,     touchSensor,    sensorTouch)
#pragma config(Motor,  motorA,          rightMotor,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
    while (true)
    {
        if (SensorValue[sonarSensor] >= 25)
        {
            motor[leftMotor] = 30;
            motor[rightMotor] = 30;
        }
        else if (SensorValue[sonarSensor] < 20)
        {
            motor[leftMotor] = -30;
            motor[rightMotor] = -30;
        }
        else
        {
            motor[leftMotor] = 0;
            motor[rightMotor] = 0;
        }

        wait1Msec(100);
    }
}