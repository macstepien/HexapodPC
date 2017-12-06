#ifndef SERVO_H
#define SERVO_H

class Servo
{
	private:
		int servoNumber;
		int middleSignal; //signal in default position
		float rate; //rate used to calculate angles to signals
		
	public:
		Servo(int servoNumber1 = 0, int middleSignal1 = 5000);
		void setServo(float angle);
};

#endif