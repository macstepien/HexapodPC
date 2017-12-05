#include "Robot/leg.h"
#include "Robot/inversekinematics.h"

using namespace cv;

Leg::Leg(Point3f joint1, Point3f angles1, Point3f lengths1, cv::Point3i servos1, cv::Point3f signals1)
{
    legJoints.A = joint1;
    angles = angles1;
    lengths = lengths1;
    servos = servos1;
    signals = signals1;
    initJointPoints();
}

void Leg::initJointPoints()
{
    initAngles = angles;

    calculateJointPoints();
    calculateServoSignals();
}

void Leg::restart()
{
    angles = initAngles;

    calculateJointPoints();
    calculateServoSignals();
}

void Leg::calculateJointPoints()
{
    Mat P1 = (Mat_<float>(3,1) << lengths.x, 0, 0);
    Mat R1 = (Mat_<float>(3,3) << cos(angles.x), 0, sin(angles.x), 0, 1, 0, -sin(angles.x), 0, cos(angles.x));
    Mat P11 = R1*P1;
    P11 = R*P11;
    legJoints.B = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + legJoints.A;

    Mat P2 = (Mat_<float>(3,1) << lengths.y, 0, 0);
    Mat R2 = (Mat_<float>(3,3) << cos(angles.y), -sin(angles.y), 0, sin(angles.y), cos(angles.y), 0, 0, 0, 1);
    Mat P22 = R1*(R2*P2);
    P22 = R*P22;
    legJoints.C = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + legJoints.B;

    Mat P3 = (Mat_<float>(3,1) << lengths.z, 0, 0);
    Mat R3 = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);
    Mat P33 = R1*(R3*P3);
    P33 = R*P33;
    legJoints.D = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + legJoints.C;
}

void Leg::moveLeg(cv::Point3f t)
{
    legJoints.D += t;
    Point3f desiredPosition = legJoints.D - legJoints.A;
    angles = InverseKinematics::calculate(desiredPosition, lengths);

    calculateJointPoints();

    calculateServoSignals();

    /*if(angles.x != angles.x || angles.y != angles.y || angles.z!=angles.z)//nan detect
    {
        calculateJointPoints();
        return -1;
    }*/
}

void Leg::calculateServoSignals()
{
    Point3f relativeAngles = angles;
    relativeAngles.z = (CV_PI - angles.z + angles.y);
  
    float wspolczynnik = 1000/(CV_PI/2 - 1.18);

    int sygnalA, sygnalB, sygnalC;
    sygnalA = relativeAngles.x*wspolczynnik + signals.x;
    sygnalB = -relativeAngles.y*wspolczynnik + signals.y;
    sygnalC = (CV_PI/2 -relativeAngles.z)*wspolczynnik + signals.z;

    /*device->setTarget(servos.x, sygnalA);
    device->setTarget(servos.y, sygnalB);
    device->setTarget(servos.z, sygnalC);*/
}