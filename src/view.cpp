#include "view.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgproc.hpp"

#define SCALE 0.017453

using namespace std;
using namespace cv;

View::View(double f1, cv::Point3f angles, cv::Point3f translation)
{
    f = f1;
    alfa = angles.x;
    beta = angles.y;
    gamma = angles.z;
    T = translation;
    /*Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(alfa), -sin(alfa), 0, sin(alfa), cos(alfa));
    Ry = (Mat_<float>(3,3) << cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta));
    Rz = (Mat_<float>(3,3) << cos(gamma), -sin(gamma), 0, sin(gamma), cos(gamma), 0,0, 0, 1);
    R = Rz*Ry*Rx;*/
    translationStep = 10;
    rotationStep = 0.1;

    screen = Mat(480, 640, CV_8UC3, Scalar(255,255,255));

    namedWindow("img");
    alfaS = 105;
    betaS = 180;
    gammaS = 180;
    createTrackbar("alfa", "img", &alfaS, 360);
    createTrackbar("beta", "img", &betaS, 360);
    createTrackbar("gamma", "img", &gammaS, 360);
}

void View::update(char key, Robot& rob)
{
    screen = Mat(480, 640, CV_8UC3, Scalar(255,255,255));
    change(key);
    setAngles(Point3f((alfaS-180)*SCALE, (betaS-180)*SCALE, (gammaS-180)*SCALE));
    drawFloor();
    drawAxis(Point3f(0,0,0));
    drawAxis(rob.getPosition());
    drawRobot(rob);

    if(voltage != "")
    {
        putText(screen, voltage, Point(20,30), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 0, 0), 2);
    }

    imshow("img", screen);
}

void View::rotateView(cv::Point3f angles)
{
    alfa += angles.x;
    beta += angles.y;
    gamma += angles.z;
}

void View::setAngles(cv::Point3f angles)
{
    alfa = angles.x;
    beta = angles.y;
    gamma = angles.z;
}

void View::translateView(cv::Point3f translation)
{
    //odpowiednie przeksztalcanie przesuniecia aby poruszac sie relatywnie do aktualnych katow, a nie po globalnych osiach(tak lepiej sie to uzytkuje)
    Mat Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(alfa), -sin(alfa), 0, sin(alfa), cos(alfa));
    Mat Ry = (Mat_<float>(3,3) << cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta));
    Mat Rz = (Mat_<float>(3,3) << cos(gamma), -sin(gamma), 0, sin(gamma), cos(gamma), 0,0, 0, 1);
    Mat R = Rz*Ry*Rx;
    Mat P = (Mat_<float>(3,1) << translation.x, translation.y, translation.z);
    P = R*P;
    T += Point3f(P.at<float>(0,0), P.at<float>(0,1), P.at<float>(0,2));
    /*T.x += translation.x;
    T.y += translation.y;
    T.z += translation.z;*/
}

void View::drawPoint(cv::Point3f p1, Scalar color, int size1)
{
    float x = p1.x-T.x;
    float y = p1.y-T.y;
    float z = p1.z-T.z;

    //rzutowanie punktu tak jak na wikipedii
    Point3f p11;
    p11.x = cos(beta)*(sin(gamma)*y+cos(gamma)*x)-sin(beta)*z;
    p11.y = sin(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))+cos(alfa)*(cos(gamma)*y-sin(gamma)*x);
    p11.z = cos(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))-sin(alfa)*(cos(gamma)*y-sin(gamma)*x);

    Point point1;
    point1.x = f*(p11.x/p11.z)+320; // dodawanie aby srodek byl na srodku ekranu
    point1.y = f*(p11.y/p11.z)+240;

    circle(screen, Point(point1.x, point1.y), size1, Scalar(0, 0, 255), -1);
}

void View::drawLine(cv::Point3f p1, cv::Point3f p2, Scalar color, int thickness)
{
    float x = p1.x-T.x;
    float y = p1.y-T.y;
    float z=p1.z-T.z;

    Point3f p11;
    p11.x = cos(beta)*(sin(gamma)*y+cos(gamma)*x)-sin(beta)*z;
    p11.y = sin(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))+cos(alfa)*(cos(gamma)*y-sin(gamma)*x);
    p11.z = cos(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))-sin(alfa)*(cos(gamma)*y-sin(gamma)*x);

    Point point1;
    point1.x = f*(p11.x/p11.z)+320;
    point1.y = f*(p11.y/p11.z)+240;

    x = p2.x-T.x;
    y = p2.y-T.y;
    z = p2.z-T.z;

    Point3f p22;
    p22.x = cos(beta)*(sin(gamma)*y+cos(gamma)*x)-sin(beta)*z;
    p22.y = sin(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))+cos(alfa)*(cos(gamma)*y-sin(gamma)*x);
    p22.z = cos(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))-sin(alfa)*(cos(gamma)*y-sin(gamma)*x);

    Point point2;
    point2.x = f*(p22.x/p22.z)+320;
    point2.y = f*(p22.y/p22.z)+240;

    line(screen, point1, point2, color, thickness);
}

void View::drawRectangle(rect rec, Scalar color, int thickness)
{
    drawLine(rec.dl, rec.dr, color, thickness);
    drawLine(rec.dr, rec.ur, color, thickness);
    drawLine(rec.ur, rec.ul, color, thickness);
    drawLine(rec.ul, rec.dl, color, thickness);
}

void View::drawRobot(Robot& rob)
{
    int thickness = 2;
    int size1 = 4;
    drawRectangle(rob.getFrame(), Scalar(255,0,0), thickness);
    joints joint;
    for(int i=0; i<6; ++i)
    {
        joint = rob.getLegJoints(i);
        drawLine(joint.A, joint.B, Scalar(255,0,0), thickness);
        drawLine(joint.B, joint.C, Scalar(255,0,0), thickness);
        drawLine(joint.C, joint.D, Scalar(255,0,0), thickness);
        drawPoint(joint.A, Scalar(0,0,255), size1);
        drawPoint(joint.B, Scalar(0,0,255), size1);
        drawPoint(joint.C, Scalar(0,0,255), size1);
        drawPoint(joint.D, Scalar(0,0,255), size1);
    }
}

void View::change(char key)
{
    switch(key)
    {
        case 'w':
            translateView(Point3f(0,-translationStep,0));
            break;
        case 's':
            translateView(Point3f(0,translationStep,0));
            break;
        case 'a':
            translateView(Point3f(-translationStep,0,0));
            break;
        case 'd':
            translateView(Point3f(translationStep,0,0));
            break;
        case 'q':
            translateView(Point3f(0,0,translationStep));
            break;
        case 'e':
            translateView(Point3f(0,0,-translationStep));
            break;
        case 'r':
            rotateView(Point3f(rotationStep,0,0));
            break;
        case 'f':
            rotateView(Point3f(-rotationStep,0,0));
            break;
        case 't':
            rotateView(Point3f(0,rotationStep,0));
            break;
        case 'g':
            rotateView(Point3f(0,-rotationStep,0));
            break;
        case 'y':
            rotateView(Point3f(0,0,rotationStep));
            break;
        case 'h':
            rotateView(Point3f(0,0,-rotationStep));
            break;
    }
}

void View::drawFloor()
{
    int i;
    for(i = -10; i <= 10; ++i)
    {
        drawLine(Point3f(i*10,0,0), Point3f(i*10,0,200), Scalar(0,0,0));
    }
    for(i = 0; i <= 20; ++i)
    {
        drawLine(Point3f(-100,0,i*10), Point3f(100,0,i*10), Scalar(0,0,0));
    }
}

void View::drawAxis(cv::Point3f pt)
{
    drawPoint(pt);
    drawLine(pt, pt+Point3f(10,0,0), Scalar(255,0,0), 2);//x niebieski
    drawLine(pt, pt+Point3f(0,10,0), Scalar(0,255,0), 2);//y zielony
    drawLine(pt, pt+Point3f(0,0,10), Scalar(0,0,255), 2);//z czerwony
}
