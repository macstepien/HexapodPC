#include "robotcontroler.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

RobotControler::RobotControler(float walkStep1, float rotStep1, float sMoveStep1, float sRotStep1, cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths)
 : rob(pos, ang, width1, length1, leglengths)
{
	walkStep = walkStep1;
	rotStep = rotStep1;
	sMoveStep = sMoveStep1;
	sRotStep = sRotStep1;
	defaultRobotPosition = pos;
	defaultRobotAngles = ang;
}

void RobotControler::walk(int mode, int direction, View& view1)
{
	Point3f step;
	switch(direction)
	{
		case 0:
			step = Point3f(0,0,walkStep);
			break;
		case 1:
			step = Point3f(0,0,-walkStep);
			break;
		case 2:
			step = Point3f(-walkStep,0,0);
			break;
		case 3:
			step = Point3f(walkStep,0,0);
			break;
		default:
			cout << "Nieprawidłowy kierunek" << endl;
			return;
	}

	switch(mode)
	{
		case 0:
			rob.walk(step);
			break;
		case 1:
			rob.walkC(step, view1);
			break;
		case 2:
			rob.walk2C(step, view1);
			break;
		case 3:
			rob.walk3C(step, view1);
			break;
		case 4:
			rob.walkAsym(step, view1);
			break;
	}
	
}

void RobotControler::rotate(int mode, int direction, View& view1)
{
	float angle;
	switch(direction)
	{
		case 0:
			angle = rotStep;
			break;
		case 1:
			angle = -rotStep;
			break;
		default:
			cout << "Nieprawidłowy kierunek";
			return;
	}

	switch(mode)
	{
		case 0:
			rob.walkRot(angle);
			break;
		case 1:
			rob.walkRotC(angle, view1);
			break;
		case 2:
			rob.walkRot3C(angle, view1);
			break;
	}
}

void RobotControler::moveBase(int direction)
{
	Point3f step;
	switch(direction)
	{
		case 0:
			step = Point3f(sMoveStep,0,0);
			break;
		case 1:
			step = Point3f(-sMoveStep,0,0);
			break;
		case 2:
			step = Point3f(0,0,sMoveStep);
			break;
		case 3:
			step = Point3f(0,0,-sMoveStep);
			break;
		case 4:
			step = Point3f(0,sMoveStep,0);
			break;
		case 5:
			step = Point3f(0,-sMoveStep,0);
			break;
		default:
			cout << "Nieprawidłowy kierunek" << endl;
			return;
	}
	rob.move(step);
}

void RobotControler::rotateBase(int direction)
{
	Point3f rotation;
	switch(direction)
	{
		case 0:
			rotation = Point3f(sRotStep,0,0);
			break;
		case 1:
			rotation = Point3f(-sRotStep,0,0);
			break;
		case 2:
			rotation = Point3f(0,0,sRotStep);
			break;
		case 3:
			rotation = Point3f(0,0,-sRotStep);
			break;
		case 4:
			rotation = Point3f(0,sRotStep,0);
			break;
		case 5:
			rotation = Point3f(0,-sRotStep,0);
			break;
		default:
			cout << "Nieprawidłowy kierunek" << endl;
			return;
	}
	rob.rotate(rotation);
}

void RobotControler::walkToPoint(cv::Point2f point, View& view1)
{
    float destAngle = atan2(point.y, point.x);
    float differenceAngle = destAngle - rob.getAngles().y;

    int rotateTimes = differenceAngle/rotStep;

    for(int i = 0; i < rotateTimes; ++i)
    {
        rob.walkRot3C(rotStep, view1);
    }
    rob.walkRot3C(differenceAngle-rotateTimes*rotStep, view1);

    float distance = sqrt(point.x*point.x + point.y*point.y);

    int walkTimes = distance/walkStep;

    for(int i = 0; i < walkTimes; ++i)
    {
        rob.walkAsym(Point3f(0,0,walkStep), view1);
    }
    rob.walkAsym(Point3f(0,0,distance-walkTimes*walkStep), view1);
}

void RobotControler::showoff(View& view1)
{
    int wait = 500;

    rob.restart(Point3f(0, 17 ,100), Point3f(0,0,0));

    rob.move(Point3f(2,0,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(-4,0,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(2,0,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(0,0,2));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(0,0,-4));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(0,0,2));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(0,3,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(0,-6,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.move(Point3f(0,3,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0,0.1,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0,-0.2,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0,0.1,0));
    view1.update('b', rob);
    waitKey(wait);
    rob.rotate(Point3f(0.1,0,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(-0.2,0,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0.1,0,0));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0,0,0.2));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0,0,-0.4));
    view1.update('b', rob);
    waitKey(wait);

    rob.rotate(Point3f(0,0,0.2));
    view1.update('b', rob);
    waitKey(wait);
}