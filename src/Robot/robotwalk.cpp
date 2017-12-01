#include "Robot/robotwalk.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

using namespace cv;

RobotWalk::RobotWalk()
{
    walkingSequnceNo = 0;
    rotatingSequenceNo = 0;

    delayLong = 500;
    delayShort = 10;

    firstStep = true;
    stepHeight = 4;

    smallMotion = 0.3;
}

void RobotWalk::walkStraight(float step, Robot& rob, GUI& view1)
{
    ///najpierw chodzenie tylko do przodu
    /// parabola -h*x(x-z)

    Point3f steps;

    float z = step;
    float a = (4*stepHeight)/(z*z);

    float di = smallMotion;

    float sdi = (z<0)?-di:di;
    z = abs(z);

    for (double i = 0; i < z; i += di)
    {
        steps = Point3f(0, -(rob.getLeg(0).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

        rob.moveLeg(0, steps);
        rob.moveLeg(4, steps);
        rob.moveLeg(2, steps);

        rob.move(Point3f(0,0,sdi/2));

        view1.update('b', rob);

        waitKey(delayShort);
    }

    for (double i = 0; i < z; i += di)
    {
        steps = Point3f(0, -(rob.getLeg(3).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

        rob.moveLeg(3, steps);
        rob.moveLeg(1, steps);
        rob.moveLeg(5, steps);

        rob.move(Point3f(0,0,sdi/2));

        view1.update('b', rob);

        waitKey(delayShort);
    }
}

void RobotWalk::walkStraightAlt(float step, Robot& rob, GUI& view1)
{
    int legOrder[6];
    if(firstStep)
    {
        legOrder[0] = 0;
        legOrder[1] = 4;
        legOrder[2] = 2;
        legOrder[3] = 3;
        legOrder[4] = 1;
        legOrder[5] = 5;
        firstStep = false;
    }
    else
    {
        legOrder[0] = 3;
        legOrder[1] = 1;
        legOrder[2] = 5;
        legOrder[3] = 0;
        legOrder[4] = 4;
        legOrder[5] = 2;
        firstStep = true;
    }

    Point3f steps;

    float z = step;
    float a = (4*stepHeight)/(z*z);

    float di = smallMotion;

    float sdi = (z<0)?-di:di;
    z = abs(z);

    for (double i = 0; i < z; i += di)
    {
        steps = Point3f(0, -(rob.getLeg(legOrder[0]).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

        for(int j = 0; j < 3; ++j)
            rob.moveLeg(legOrder[j], steps);

        rob.move(Point3f(0,0,sdi/2));

        view1.update('b', rob);

        waitKey(delayShort);
    }

    for (double i = 0; i < z; i += di)
    {
        steps = Point3f(0, -(rob.getLeg(legOrder[3]).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

        for(int j = 3; j < 6; ++j)
            rob.moveLeg(legOrder[j], steps);

        rob.move(Point3f(0,0,sdi/2));

        view1.update('b', rob);

        waitKey(delayShort);
    }
}

void RobotWalk::walkStepAhead(float step, int mode, Robot& rob, GUI& view1)
{
    int legOrder[6];
    
    if(step > 0)
    {
        legOrder[0] = 0;
        legOrder[1] = 4;
        legOrder[2] = 2;
        legOrder[3] = 3;
        legOrder[4] = 1;
        legOrder[5] = 5;
    }
    else
    {
        legOrder[0] = 3;
        legOrder[1] = 1;
        legOrder[2] = 5;
        legOrder[3] = 0;
        legOrder[4] = 4;
        legOrder[5] = 2;
    }
    
    if(mode == 0)
    {
        Point3f steps;
        float z = step;
        float a = (4*stepHeight)/(z*z);

        float di = smallMotion;

        float sdi = (z<0)?-di:di;
        z = abs(z);

        for (double i = 0; i < z; i += di)
        {
            steps = Point3f(0, -(rob.getLeg(legOrder[0]).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

            for(int j = 0; j < 3; ++j)
                rob.moveLeg(legOrder[j], steps);

            rob.move(Point3f(0,0,sdi/2));

            view1.update('b', rob);

            waitKey(delayShort);
        }
    }
    else if(mode == 1)
    {
        Point3f steps;

        float z = 2*step;
        float a = (4*stepHeight)/(z*z);

        float di = smallMotion;

        float sdi = (z<0)?-di:di;
        z = abs(z);

        for (double i = 0; i < z; i += di)
        {
            steps = Point3f(0, -(rob.getLeg(legOrder[3]).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

            for(int j = 3; j < 6; ++j)
                rob.moveLeg(legOrder[j], steps);

            rob.move(Point3f(0,0,sdi/2));

            view1.update('b', rob);

            waitKey(delayShort);
        }

        for (double i = 0; i < z; i += di)
        {
            steps = Point3f(0, -(rob.getLeg(legOrder[0]).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

            for(int j = 0; j < 3; ++j)
                rob.moveLeg(legOrder[j], steps);

            rob.move(Point3f(0,0,sdi/2));

            view1.update('b', rob);

            waitKey(delayShort);
        }
    }
    else if(mode == 2)
    {
        Point3f steps;
        float z = step;
        float a = (4*stepHeight)/(z*z);

        float di = smallMotion;

        float sdi = (z<0)?-di:di;
        z = abs(z);

        for (double i = 0; i < z; i += di)
        {
            steps = Point3f(0, -(rob.getLeg(legOrder[3]).getJoints().D.y - 16.3) + a*i*(i-z), sdi);

            for(int j = 3; j < 6; ++j)
                rob.moveLeg(legOrder[j], steps);

            rob.move(Point3f(0,0,sdi/2));

            view1.update('b', rob);

            waitKey(delayShort);
        }
    }
}

void RobotWalk::walk(Point3f steps, Robot& rob, GUI& view1)
{
    ///najpierw chodzenie tylko do przodu
    /// parabola -h*x(x-z)
    Point3f steps1 = steps;

    float x = steps1.x;
    float z = steps1.z;

    float x2 = sqrt(x*x + z*z);

    float a = (4*stepHeight)/(x2*x2);

    float di = smallMotion;

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
        steps = Point3f(dx, -(rob.getLeg(0).getJoints().D.y - 16.3) + a*i*(i-x2), dz);

        rob.moveLeg(0, steps);
        rob.moveLeg(4, steps);
        rob.moveLeg(2, steps);

        rob.move(Point3f(dx/2,0,dz/2));

        view1.update('b', rob);

        waitKey(delayShort);
    }

    for (double i = 0; i < x2; i += di)
    {
        steps = Point3f(dx, -(rob.getLeg(3).getJoints().D.y - 16.3) + a*i*(i-x2), dz);

        rob.moveLeg(3, steps);
        rob.moveLeg(1, steps);
        rob.moveLeg(5, steps);

        rob.move(Point3f(dx/2,0,dz/2));

        view1.update('b', rob);

        waitKey(delayShort);
    }
}

void RobotWalk::rotation(float angle, Robot& rob, GUI& view1)
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
        g11 = (rob.getLeg(i).getJoints().D);
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

            step1 = Point3f(dx[k], -(rob.getLeg(k).getJoints().D.y - 16.3) - a[k]*i1[k]*(i1[k]-x2[k]), dz[k]);

            rob.moveLeg(k, step1);

            dx[k] = di[k]*cos(phi[k]+j);
            dz[k] = di[k]*sin(phi[k]+j);
        }

        rob.rotate(Point3f(0,da,0));

        view1.update('b', rob);

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

            step1 = Point3f(dx[k], -(rob.getLeg(k).getJoints().D.y - 16.3) - a[k]*i1[k]*(i1[k]-x2[k]) , dz[k]);

            rob.moveLeg(k, step1);

            dx[k] = di[k]*cos(phi[k]+j);
            dz[k] = di[k]*sin(phi[k]+j);
        }

        rob.rotate(Point3f(0,da,0));

        view1.update('b', rob);

        waitKey(delayShort);
    }
}

void RobotWalk::simpleAutomaticWalk(Point3f steps, Robot& rob, GUI& view1)
{
    simpleWalk(steps, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleWalk(steps, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleWalk(steps, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleWalk(steps, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleWalk(steps, rob);
    view1.update('b', rob);
    waitKey(delayLong);
}

void RobotWalk::simpleAutomaticRotation(float angle, Robot& rob, GUI& view1)
{
    simpleRotation(angle, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleRotation(angle, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleRotation(angle, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleRotation(angle, rob);
    view1.update('b', rob);
    waitKey(delayLong);

    simpleRotation(angle, rob);
    view1.update('b', rob);
    waitKey(delayLong);
}

void RobotWalk::simpleWalk(Point3f step, Robot& rob)
{
    if(walkingSequnceNo == 0)
    {
        step.x /= 2;
        step.y = -stepHeight;
        step.z /= 2;

        rob.moveLeg(0, step);
        rob.moveLeg(4, step);
        rob.moveLeg(2, step);

        ++walkingSequnceNo;
    }
    else if(walkingSequnceNo == 1)
    {
        step.x /= 2;
        step.y = stepHeight;
        step.z /= 2;

        rob.moveLeg(0, step);
        rob.moveLeg(4, step);
        rob.moveLeg(2, step);

        ++walkingSequnceNo;
    }
    else if (walkingSequnceNo == 2)
    {
        rob.move(step);
        ++walkingSequnceNo;
    }
    else if(walkingSequnceNo == 3)
    {

        step.x/=2;
        step.y = -stepHeight;
        step.z /=2;

        rob.moveLeg(3, step);
        rob.moveLeg(1, step);
        rob.moveLeg(5, step);

        ++walkingSequnceNo;
    }
    else if(walkingSequnceNo == 4)
    {
        step.x /= 2;
        step.y = stepHeight;
        step.z /= 2;

        rob.moveLeg(3, step);
        rob.moveLeg(1, step);
        rob.moveLeg(5, step);

        walkingSequnceNo = 0;
    }
}

void RobotWalk::simpleRotation(float angle, Robot& rob)
{
    Mat Rx1 = (Mat_<float>(3,3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));

    if(rotatingSequenceNo == 0)
    {
        for(int i = 0; i < 6; ++i)
        {
            Point3f g11 = (rob.getLeg(i).getJoints().D);
            Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
            Mat P11 = Rx1*P1;
            Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
            stepsl[i] = g12 - g11;
            stepsl[i].x /= 2;
            stepsl[i].y = -stepHeight;
            stepsl[i].z /= 2;
        }

        rob.moveLeg(0, stepsl[0]);
        rob.moveLeg(4, stepsl[4]);
        rob.moveLeg(2, stepsl[2]);

        ++rotatingSequenceNo;
    }
    else if(rotatingSequenceNo == 1)
    {
        stepsl[0].y = stepHeight;
        stepsl[4].y = stepHeight;
        stepsl[2].y = stepHeight;

        rob.moveLeg(0, stepsl[0]);
        rob.moveLeg(4, stepsl[4]);
        rob.moveLeg(2, stepsl[2]);

        ++rotatingSequenceNo;
    }
    else if(rotatingSequenceNo == 2)
    {
        rob.rotate(Point3f(0,angle,0));
        ++rotatingSequenceNo;
    }
    else if(rotatingSequenceNo == 3)
    {
        for(int i = 0; i < 6; ++i)
        {
            Point3f g11 = (rob.getLeg(i).getJoints().D);
            Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
            Mat P11 = Rx1*P1;
            Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
            stepsl[i] = g12 - g11;
            stepsl[i].x /= 2;
            stepsl[i].y = -stepHeight;
            stepsl[i].z /= 2;
        }

        rob.moveLeg(3, stepsl[3]);
        rob.moveLeg(1, stepsl[1]);
        rob.moveLeg(5, stepsl[5]);

        ++rotatingSequenceNo;
    }
    else if(rotatingSequenceNo == 4)
    {
        rob.moveLeg(3, stepsl[3]);
        rob.moveLeg(1, stepsl[1]);
        rob.moveLeg(5, stepsl[5]);

        rotatingSequenceNo = 0;
    }

}