#include "Robot/servo.h"
#include <opencv2/core/core.hpp>

Servo::Servo(int servoNumber1, int middleSignal1)
{
	servoNumber = servoNumber1;
	middleSignal = middleSignal1;
	rate = 1000/(CV_PI/2 - 1.18);
}

void Servo::setServo(float angle)
{
	int signal = angle*rate + middleSignal;
	//device->setTarget(servoNumber, signal);
}