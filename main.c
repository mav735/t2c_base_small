#pragma config(Sensor, S1,     leftSensor,             sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Sensor, S2,     rightSensor,            sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Motor,  motorA,          leftMotor,     tmotorEV3_Medium, openLoop, encoder)
#pragma config(Motor,  motorB,          rightMotor,    tmotorEV3_Medium, openLoop, encoder)

#define READ
#include "includes/includes.h"


task main (){
    setMotorBrakeMode(motorA, motorCoast);
    setMotorBrakeMode(motorB, motorCoast);
    setMotorBrakeMode(motorD, motorCoast);
    setMotorBrakeMode(motorC, motorCoast);

    colorDetectInit();
	#ifdef READ
    	readMinMax();
	#endif
	setDefaultLine();
	lineFollowEncoder(20, 500);
    fileClose(fileHandle);
}
