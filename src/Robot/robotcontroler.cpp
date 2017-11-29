#include "Robot/robotcontroler.h"
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
	mode = 7;
}

void RobotControler::control(char key, GUI& view1)
{
	switch(key)
    {
        case '1':
            mode = 1;
            break;
        case '2':
            mode = 2;
            break;
        case '3':
            mode = 3;
            break;
        case '4':
            mode = 4;
            break;
        case '5':
            mode = 5;
            break;
        case '6':
            mode = 6;
            break;
        case '7':
        	mode = 7;
        	break;
        case '8':
        	mode = 8;
        	break;
        case '9':
        	mode = 9;
        	break;
        case 'R':
        	restart();
        	break;
    }

    Point2f pkt;
    int xMode = 0;
    int yMode = 0;
    int rotMode = 0;

    switch(mode)
    {
        case 1:
            moveBase(key);
            break;
        case 2:
            rotateBase(key);
            break;
        case 3:
            xMode = 0;
            yMode = 0;
            rotMode = 0;
            break;
        case 4:
            xMode = 1;
            yMode = 1;
            rotMode = 1;
            break;
        case 5:
            xMode = 1;
            yMode = 2;
            rotMode = 1;
            break;
        case 6:
            xMode = 3;
            yMode = 3;
            rotMode = 2;
            break;
        case 7:
            xMode = 3;
            yMode = 4;
            rotMode = 2;
            break;
        case 8:
        	cout << "Podaj wspolrzedne: ";
        	cin >> pkt.x >> pkt.y;
        	walkToPoint(pkt, view1);
        	mode = 7;
        	break;
        case 9:
        	showoff(view1);
    		mode = 7;
    		break;
    }

    if(mode >= 3 && mode <= 7)
    {
        switch(key)
        {
            case 'W':
            case 'S':
                walk(yMode,key,view1);
                break;

            case 'A':
            case 'D':
                walk(xMode,key,view1);
                break;
            
            case 'Q':
            case 'E':
                rotate(rotMode,key,view1);
                break;
        }
    }
}

void RobotControler::walk(int mode, char direction, GUI& view1)
{
	Point3f step;
	switch(direction)
	{
		case 'W':
			step = Point3f(0,0,walkStep);
			break;
		case 'S':
			step = Point3f(0,0,-walkStep);
			break;
		case 'A':
			step = Point3f(-walkStep,0,0);
			break;
		case 'D':
			step = Point3f(walkStep,0,0);
			break;
		default:
			//cout << "Nieprawidłowy kierunek" << endl;
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

void RobotControler::rotate(int mode, char direction, GUI& view1)
{
	float angle;
	switch(direction)
	{
		case 'Q':
			angle = -rotStep;
			break;
		case 'E':
			angle = rotStep;
			break;
		default:
			//cout << "Nieprawidłowy kierunek";
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

void RobotControler::moveBase(char direction)
{
	Point3f step;
	switch(direction)
	{
		case 'W':
			step = Point3f(sMoveStep,0,0);
			break;
		case 'S':
			step = Point3f(-sMoveStep,0,0);
			break;
		case 'A':
			step = Point3f(0,0,sMoveStep);
			break;
		case 'D':
			step = Point3f(0,0,-sMoveStep);
			break;
		case 'Q':
			step = Point3f(0,sMoveStep,0);
			break;
		case 'E':
			step = Point3f(0,-sMoveStep,0);
			break;
		default:
			//cout << "Nieprawidłowy kierunek" << endl;
			return;
	}
	rob.move(step);
}

void RobotControler::rotateBase(char direction)
{
	Point3f rotation;
	switch(direction)
	{
		case 'W':
			rotation = Point3f(sRotStep,0,0);
			break;
		case 'S':
			rotation = Point3f(-sRotStep,0,0);
			break;
		case 'A':
			rotation = Point3f(0,0,sRotStep);
			break;
		case 'D':
			rotation = Point3f(0,0,-sRotStep);
			break;
		case 'Q':
			rotation = Point3f(0,sRotStep,0);
			break;
		case 'E':
			rotation = Point3f(0,-sRotStep,0);
			break;
		default:
			//cout << "Nieprawidłowy kierunek" << endl;
			return;
	}
	rob.rotate(rotation);
}

void RobotControler::walkToPoint(cv::Point2f point, GUI& view1)
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

void RobotControler::showoff(GUI& view1)
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