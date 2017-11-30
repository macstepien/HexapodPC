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
	mode = 6;
    started = false;
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

    switch(mode)
    {
        case 1:
            moveBase(key);
            break;
        case 2:
            rotateBase(key);
            break;
        case 3:
            mode3(key);
            break;
        case 4:
            mode4(key, view1);
            break;
        case 5:
            mode5(key, view1);
            break;
        case 6:
            if(!started)
            {
                mode6(key, 0, view1);
            }
            else
                mode6(key, 1, view1);

            break;
        case 7:

            break;
        case 8:
        	cout << "Podaj wspolrzedne: ";
        	cin >> pkt.x >> pkt.y;
        	walkToPoint(pkt, view1);
        	mode = 5;
        	break;
        case 9:
        	showoff(view1);
    		mode = 5;
    		break;
    }

    if(started && (mode != 6))
    {
        mode6(key, 2, view1);
        started = false;
    }
}

void RobotControler::mode6(char direction, int stage, GUI& view1)
{
    switch(direction)
    {
        case 'D':
            walker.walk(Point3f(walkStep, 0, 0), rob, view1);
            break;
        case 'A':
            walker.walk(Point3f(-walkStep, 0, 0), rob, view1);
            break;
        case 'W':
            walker.walkStraightAlt2(walkStep, stage, rob, view1);
            started = true;
            break;
        case 'S':
            walker.walkStraightAlt2(-walkStep, stage, rob, view1);
            started = true;
            break;
        case 'E':
            walker.rotation(rotStep, rob, view1);
            break;
        case 'Q':
            walker.rotation(-rotStep, rob, view1);
            break;
        default:
            return;
    }
}

void RobotControler::mode3(char direction)
{
    switch(direction)
    {
        case 'D':
            walker.simpleWalk(Point3f(walkStep, 0, 0), rob);
            break;
        case 'A':
            walker.simpleWalk(Point3f(-walkStep, 0, 0), rob);
            break;
        case 'W':
            walker.simpleWalk(Point3f(0, 0, walkStep), rob);
            break;
        case 'S':
            walker.simpleWalk(Point3f(0, 0, -walkStep), rob);
            break;
        case 'E':
            walker.simpleRotation(rotStep, rob);
            break;
        case 'Q':
            walker.simpleRotation(-rotStep, rob);
            break;
        default:
            return;
    }
}

void RobotControler::mode4(char direction, GUI& view1)
{
    switch(direction)
    {
        case 'D':
            walker.simpleAutomaticWalk(Point3f(walkStep, 0, 0), rob, view1);
            break;
        case 'A':
            walker.simpleAutomaticWalk(Point3f(-walkStep, 0, 0), rob, view1);
            break;
        case 'W':
            walker.simpleAutomaticWalk(Point3f(0, 0, walkStep), rob, view1);
            break;
        case 'S':
            walker.simpleAutomaticWalk(Point3f(0, 0, -walkStep), rob, view1);
            break;
        case 'E':
            walker.simpleAutomaticRotation(rotStep, rob, view1);
            break;
        case 'Q':
            walker.simpleAutomaticRotation(-rotStep, rob, view1);
            break;
        default:
            return;
    }
}

void RobotControler::mode5(char direction, GUI& view1)
{
    switch(direction)
    {
        case 'D':
            walker.walk(Point3f(walkStep, 0, 0), rob, view1);
            break;
        case 'A':
            walker.walk(Point3f(-walkStep, 0, 0), rob, view1);
            break;
        case 'W':
            walker.walk(Point3f(0, 0, walkStep), rob, view1);
            break;
        case 'S':
            walker.walk(Point3f(0, 0, -walkStep), rob, view1);
            break;
        case 'E':
            walker.rotation(rotStep, rob, view1);
            break;
        case 'Q':
            walker.rotation(-rotStep, rob, view1);
            break;
        default:
            return;
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
        walker.rotation(rotStep, rob, view1);
    }
    walker.rotation(differenceAngle-rotateTimes*rotStep, rob, view1);

    float distance = sqrt(point.x*point.x + point.y*point.y);

    int walkTimes = distance/walkStep;

    for(int i = 0; i < walkTimes; ++i)
    {
        walker.walkStraightAlt(walkStep, rob, view1);
    }
    walker.walkStraightAlt(distance-walkTimes*walkStep, rob, view1);
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