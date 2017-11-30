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

    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    Mat P1 = (Mat_<float>(3,1) << x.A.x, x.A.y, x.A.z);
    Mat P2 = (Mat_<float>(3,1) << x.B.x, x.B.y, x.B.z);
    Mat P3 = (Mat_<float>(3,1) << x.C.x, x.C.y, x.C.z);
    Mat P4 = (Mat_<float>(3,1) << x.D.x, x.D.y, x.D.z);

    Mat P11 = R*P1;
    Mat P22 = R*P2;
    Mat P33 = R*P3;
    Mat P44 = R*P4;

    x.A = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + position;
    x.B = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + position;
    x.C = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + position;
    x.D = Point3f(P44.at<float>(0,0), P44.at<float>(0,1), P44.at<float>(0,2)) + position;

    return x;
}

rect Robot::getFrame()
{
    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    Mat P1 = (Mat_<float>(3,1) << - width/2, 0, - length/2);
    Mat P2 = (Mat_<float>(3,1) << width/2, 0, - length/2);
    Mat P3 = (Mat_<float>(3,1) << - width/2, 0, length/2);
    Mat P4 = (Mat_<float>(3,1) << width/2, 0, length/2);

    Mat P11 = R*P1;
    Mat P22 = R*P2;
    Mat P33 = R*P3;
    Mat P44 = R*P4;

    gFrame.dl = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + position;
    gFrame.dr = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + position;
    gFrame.ul = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + position;
    gFrame.ur = Point3f(P44.at<float>(0,0), P44.at<float>(0,1), P44.at<float>(0,2)) + position;

    return gFrame;
}

void Robot::moveCoordinates(Point3f p, Point3f ang)
{
    angles += ang;

    ang = -ang;


    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(ang.x), -sin(ang.x), 0, sin(ang.x), cos(ang.x));
    Ry = (Mat_<float>(3,3) << cos(ang.y), 0, sin(ang.y), 0, 1, 0, -sin(ang.y), 0, cos(ang.y));
    Rz = (Mat_<float>(3,3) << cos(ang.z), -sin(ang.z), 0, sin(ang.z), cos(ang.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    joints x;

    Mat P1, P11;

    for(int i = 0; i < 6; ++i)
    {
        x = legs[i].getJoints();
        P1 = (Mat_<float>(3,1) << x.D.x, x.D.y, x.D.z);
        P11 = R*P1;
        x.D = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) - p;
        legs[i].setLegEnd(x.D);
        legs[i].calculateAngles();
    }
    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;
    P1 = (Mat_<float>(3,1) << p.x, p.y, p.z);
    P11 = R*P1;
    p = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
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