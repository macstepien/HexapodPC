#include "robot.h"
#include "view.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <strstream>
#include <unistd.h>

using namespace cv;
using namespace std;

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
    walkingStep = 0;
    delayLong = 500;
    delayShort = 2;

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
        legs[i].setAgnles(ang1);
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

void Robot::walk(Point3f steps)
{
    float h = 4;
    if(walkingStep == 0)
    {
        steps.x /= 2;
        steps.y = -h;
        steps.z /= 2;
        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 1)
    {
        steps.x /= 2;
        steps.y = h;
        steps.z /= 2;

        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if (walkingStep == 2)
    {
        move(steps);
        ++walkingStep;
    }
    else if(walkingStep == 3)
    {

        steps.x/=2;
        steps.y = -h;
        steps.z /=2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 4)
    {
        steps.x /= 2;
        steps.y = h;
        steps.z /= 2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        walkingStep = 0;
    }
}

void Robot::walkRot(float angle)
{
    Mat Rx1 = (Mat_<float>(3,3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));

    if(walkingStep == 0)
    {
        for(int i = 0; i < 6; ++i)
        {
            Point3f g11 = (legs[i].getJoints().D);
            Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
            Mat P11 = Rx1*P1;
            Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
            stepsl[i] = g12 - g11;
            stepsl[i].x /= 2;
            stepsl[i].y = -2;
            stepsl[i].z /= 2;
        }

        legs[0].setLegEnd(legs[0].getJoints().D+stepsl[0]);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+stepsl[4]);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+stepsl[2]);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 1)
    {
        stepsl[0].y = 2;
        stepsl[4].y = 2;
        stepsl[2].y = 2;

        legs[0].setLegEnd(legs[0].getJoints().D+stepsl[0]);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+stepsl[4]);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+stepsl[2]);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 2)
    {
        rotate(Point3f(0,angle,0));
        ++walkingStep;
    }
    else if(walkingStep == 3)
    {
        for(int i = 0; i < 6; ++i)
        {
            Point3f g11 = (legs[i].getJoints().D);
            Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
            Mat P11 = Rx1*P1;
            Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
            stepsl[i] = g12 - g11;
            stepsl[i].x /= 2;
            stepsl[i].y = -2;
            stepsl[i].z /= 2;
        }
        legs[3].setLegEnd(legs[3].getJoints().D+stepsl[3]);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+stepsl[1]);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+stepsl[5]);
        legs[5].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 4)
    {
        stepsl[3].y = 2;
        stepsl[1].y = 2;
        stepsl[5].y = 2;

        legs[3].setLegEnd(legs[3].getJoints().D+stepsl[3]);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+stepsl[1]);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+stepsl[5]);
        legs[5].calculateAngles();

        walkingStep = 0;

    }

}

void Robot::walkC(Point3f steps, View& view1)
{
    walk(steps);
    view1.update('b', *this);
    waitKey(delayLong);

    walk(steps);
    view1.update('b', *this);
    waitKey(delayLong);

    walk(steps);
    view1.update('b', *this);
    waitKey(delayLong);

    walk(steps);
    view1.update('b', *this);
    waitKey(delayLong);

    walk(steps);
    view1.update('b', *this);
    waitKey(delayLong);
}

void Robot::walkRotC(float angle, View& view1)
{
    walkRot(angle);
    view1.update('b', *this);
    waitKey(delayLong);

    walkRot(angle);
    view1.update('b', *this);
    waitKey(delayLong);

    walkRot(angle);
    view1.update('b', *this);
    waitKey(delayLong);

    walkRot(angle);
    view1.update('b', *this);
    waitKey(delayLong);

    walkRot(angle);
    view1.update('b', *this);
    waitKey(delayLong);
}

void Robot::walk2C(Point3f steps, View& view1)
{
    ///najpierw chodzenie tylko do przodu
    /// parabola -h*x(x-z)
    Point3f steps1 = steps;
    float z = steps1.z;
    float a = (8)/(z*z);
    float di = 0.08;
    float sdi = (z<0)?-di:di;
    z = abs(z);
    for (double i = 0; i < z; i += di)
    {
        steps = Point3f(0, 2*a*i*di-a*z*di, sdi);
        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        move(Point3f(0,0,sdi/2));

        view1.update('b', *this);

        waitKey(delayShort);
    }

    for (double i = 0; i < z; i += di)
    {
        steps = Point3f(0, 2*a*i*di-a*z*di, sdi);
        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        move(Point3f(0,0,sdi/2));

        view1.update('b', *this);

        waitKey(delayShort);
    }
}

void Robot::walk3C(Point3f steps, View& view1)
{
    ///najpierw chodzenie tylko do przodu
    /// parabola -h*x(x-z)
    Point3f steps1 = steps;
    float x = steps1.x;
    float y = steps1.y;
    float z = steps1.z;

    float x2 = sqrt(x*x + z*z);

    float stepHeight = x!=0 ? 3 : 2; // wyzsze kroki przy chodzeniu na bok

    float a = (4*stepHeight)/(x2*x2);
    float di = 0.3;//0.08
    float dz = 0, dx = 0;

    if(z == 0)
    {
        dx = (x<0)?(-di):di;
    }
    else
    {
        dz = di*sqrt((x/z)*(x/z)+1);
        dx = (x/z)*dz;

        dz = (z<0)?(-abs(dz)):(abs(dz));
        dx = (x<0)?(-abs(dx)):(abs(dx));
    }

    for (double i = 0; i < x2; i += di)
    {
        //steps = Point3f(dx, 2*a*i*di-a*x2*di, dz);
        steps = Point3f(dx, -(legs[0].getJoints().D.y - 16.3) + a*i*(i-x2), dz);
        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        move(Point3f(dx/2,0,dz/2));

        view1.update('b', *this);

        waitKey(delayShort);
    }

    for (double i = 0; i < x2; i += di)
    {
        //steps = Point3f(dx, 2*a*i*di-a*x2*di, dz);
        steps = Point3f(dx, -(legs[3].getJoints().D.y - 16.3) + a*i*(i-x2), dz);
        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        move(Point3f(dx/2,0,dz/2));

        view1.update('b', *this);

        waitKey(delayShort);
    }
}

void Robot::walkRot3C(float angle, View& view1)
{
    Mat Rx1;
    double da;
    Point3d steps1, steps2;
    Point3d g11, g12;
    Mat P1, P11;
    double x,z;
    Point3f step1;
    double j=0;

    Rx1 = (Mat_<double>(3,3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
    da = 0.01;
    da = (angle<0)?(-da):da;

    double x2[6];
    double a[6];
    double di[6];
    double dx[6], dz[6];
    double phi[6];
    double i1[6];

    for (int i = 0; i < 6; ++i)
    {
        g11 = (legs[i].getJoints().D);
        P1 = (Mat_<double>(3,1) << g11.x, g11.y, g11.z);
        P11 = Rx1*P1;
        g12 = Point3f(P11.at<double>(0,0), P11.at<double>(0,1), P11.at<double>(0,2));
        steps1 = g12 - g11;

        x = steps1.x;
        z = steps1.z;

        x2[i] = sqrt(x*x + z*z);
        a[i] = (-4*2)/(x2[i]*x2[i]);
        di[i] = x2[i]*(da/(angle))*2;
        phi[i] = atan2(z, x);

        dx[i] = di[i]*cos(phi[i]);
        dz[i] = di[i]*sin(phi[i]);

        i1[i] = 0;
    }

    j = 0;

    int N = (angle)/(2*da);

    for (int n = 0; n < N; ++n)
    {
        j+=da;

        for (int k = 0; k < 6; k += 2)
        {
            i1[k] += di[k];

            step1 = Point3f(dx[k], -(legs[k].getJoints().D.y - 16.3) - a[k]*i1[k]*(i1[k]-x2[k]), dz[k]);
            legs[k].setLegEnd(legs[k].getJoints().D+step1);
            legs[k].calculateAngles();

            dx[k] = di[k]*cos(phi[k]+j);
            dz[k] = di[k]*sin(phi[k]+j);
        }

        rotate(Point3f(0,da,0));

        view1.update('b', *this);

        waitKey(delayShort);
    }

    for (int k = 1; k < 6; k += 2)
    {
        dx[k] = di[k]*cos(phi[k]+j);
        dz[k] = di[k]*sin(phi[k]+j);
    }

    for (int n = 0; n < N; ++n)
    {
        j+=da;

        for (int k = 1; k < 6; k += 2)
        {
            i1[k] += di[k];

            step1 = Point3f(dx[k], -(legs[k].getJoints().D.y - 16.3) - a[k]*i1[k]*(i1[k]-x2[k]) , dz[k]);
            legs[k].setLegEnd(legs[k].getJoints().D+step1);
            legs[k].calculateAngles();

            dx[k] = di[k]*cos(phi[k]+j);
            dz[k] = di[k]*sin(phi[k]+j);
        }
        rotate(Point3f(0,da,0));

        view1.update('b', *this);

        waitKey(delayShort);
    }
}

void Robot::walkAsym(cv::Point3f steps, View& view1)
{
    

    Point3f steps1 = steps;
    float x = steps1.x;
    float y = steps1.y;
    float z = steps1.z;

    float x2 = sqrt(x*x + z*z);

    float stepHeight = x!=0 ? 3 : 2; // wyzsze kroki przy chodzeniu na bok

    float a = (4*stepHeight)/(x2*x2);
    float di = 0.3;//0.08
    float dz = 0, dx = 0;

    if(z == 0)
    {
        dx = (x<0)?(-di):di;
    }
    else
    {
        dz = di*sqrt((x/z)*(x/z)+1);
        dx = (x/z)*dz;

        dz = (z<0)?(-abs(dz)):(abs(dz));
        dx = (x<0)?(-abs(dx)):(abs(dx));
    }

    for (double i = 0; i < x2; i += di)
    {
        //steps = Point3f(dx, 2*a*i*di-a*x2*di, dz);
        steps = Point3f(dx, -(legs[0].getJoints().D.y - 16.3) + a*i*(i-x2), dz);
        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        move(Point3f(dx/2,0,dz/2));

        view1.update('b', *this);

        waitKey(delayShort);
    }

    for (double i = 0; i < x2; i += di)
    {
        //steps = Point3f(dx, 2*a*i*di-a*x2*di, dz);
        steps = Point3f(dx, -(legs[3].getJoints().D.y - 16.3) + a*i*(i-x2), dz);
        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        move(Point3f(dx/2,0,dz/2));

        view1.update('b', *this);

        waitKey(delayShort);
    }
}
