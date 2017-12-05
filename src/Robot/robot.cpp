#include "Robot/robot.h"
#include "GUI/gui.h"
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>

using namespace cv;

Robot::Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths)
{
    width = width1;
    length = length1;

    for(int i = 0; i < 6; ++i)
    {
        legs[i].setLengths(leglengths);
    }

    restart(pos, ang);
}

void Robot::restart(cv::Point3f pos, cv::Point3f ang)
{
    position = pos;
    angles = ang;
    position.y = -position.y;
    initPosition = position;
    initAngles = angles;

    lFrame.dl = Point3f(-width/2,0,-length/2);
    lFrame.dr = Point3f(width/2,0,-length/2);
    lFrame.ul = Point3f(-width/2,0,length/2);
    lFrame.ur = Point3f(width/2,0,length/2);

    for(int i = 3; i < 6; ++i)
        legs[i].setR((Mat_<float>(3,3) << -1, 0, 0, 0, 1, 0, 0, 0, -1));
    for(int i = 0; i < 3; ++i)
        legs[i].setR((Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1));

    Point3f ang1(0.0 ,0.0 ,CV_PI/2);

    legs[0].setJointA(lFrame.ur);
    legs[0].setSignals(Point3f(5900,5140,4960));
    legs[0].setServos(Point3i(3,4,5));

    legs[1].setJointA((lFrame.ur+lFrame.dr)/2);
    legs[1].setSignals(Point3f(5900,5200,5020));
    legs[1].setServos(Point3i(9,10,11));

    legs[2].setJointA(lFrame.dr);
    legs[2].setSignals(Point3f(6160,5080,5160));
    legs[2].setServos(Point3i(15,16,17));

    legs[3].setJointA(lFrame.ul);
    legs[3].setSignals(Point3f(6080,5320,4700));
    legs[3].setServos(Point3i(0,1,2));

    legs[4].setJointA((lFrame.ul+lFrame.dl)/2);
    legs[4].setSignals(Point3f(6100,4820,5020));
    legs[4].setServos(Point3i(6,7,8));

    legs[5].setJointA(lFrame.dl);
    legs[5].setSignals(Point3f(6100,5080,4920));
    legs[5].setServos(Point3i(12,13,14));

    for(int i = 0; i < 6; ++i)
    {
        legs[i].setAngles(ang1);
        legs[i].initJointPoints();
    }
}

joints Robot::getLegJoints(int n)
{
    if(!(n>=0 && n<=5))
        return joints();

    joints x = legs[n].getJoints();

    ///Przekształcanie zgięć przedstawianych domyślnie w lokalnych układach współrzędnych na globalne układy współrzędnych

    x.A = rotate3D(x.A, angles) + position;
    x.B = rotate3D(x.B, angles) + position;
    x.C = rotate3D(x.C, angles) + position;
    x.D = rotate3D(x.D, angles) + position;

    return x;
}

rect Robot::getFrame()
{
    ///Przekształcanie rogów podstawy robota przedstawianych domyślnie w lokalnym układadzie współrzędnych na globalny układ współrzędnych

    gFrame.dl = rotate3D(Point3f(-width/2, 0, -length/2), angles) + position;
    gFrame.dr = rotate3D(Point3f(width/2, 0, -length/2), angles) + position;
    gFrame.ul = rotate3D(Point3f(-width/2, 0, length/2), angles) + position;
    gFrame.ur = rotate3D(Point3f(width/2, 0, length/2), angles) + position;

    return gFrame;
}

void Robot::moveCoordinates(Point3f p, Point3f ang)
{
    angles += ang;

    ang = -ang;

    joints x;

    Mat P1, P11;

    for(int i = 0; i < 6; ++i)
    {
        x = legs[i].getJoints();
        x.D = rotate3D(x.D, ang) - p;
        legs[i].setLegEnd(x.D);
        legs[i].calculateAngles();
    }

    p = rotate3D(p, angles);

    position += p;
}

void Robot::move(Point3f p)
{
    moveCoordinates(p, Point3f(0,0,0));

    for(int i = 0; i < 6; ++i)
    {
        if(legs[i].calculateAngles() == -1)
            this->move(-p);
    }
}

void Robot::rotate(Point3f ang)
{
    moveCoordinates(Point3f(0,0,0), ang);

    for(int i = 0; i < 6; ++i)
    {
        if(legs[i].calculateAngles() == -1)
            this->rotate(-ang);
    }
}

void Robot::moveLeg(int n, cv::Point3f p)
{
    legs[n].setLegEnd(legs[n].getJoints().D+p);
    legs[n].calculateAngles();
}