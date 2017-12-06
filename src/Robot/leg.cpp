#include "Robot/leg.h"
#include "Robot/inversekinematics.h"

using namespace cv;

Leg::Leg(Point3f joint1, Point3f angles1, Point3f lengths1, cv::Point3i servos1, cv::Point3f signals1)
{
    legJoints.A = joint1;
    angles = angles1;
    lengths = lengths1;

    setServos(servos1, signals1);

    initialize();
}

void Leg::initialize()
{
    initAngles = angles;

    calculateJointPoints();
    sendServoSignals();
}

void Leg::restart()
{
    angles = initAngles;

    calculateJointPoints();
    sendServoSignals();
}

void Leg::setServos(cv::Point3i servos1, cv::Point3f signals1)
{
    servos[0] = Servo(servos1.x, signals1.x);
    servos[1] = Servo(servos1.y, signals1.y);
    servos[2] = Servo(servos1.z, signals1.z);
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

    sendServoSignals();

    /*if(angles.x != angles.x || angles.y != angles.y || angles.z!=angles.z)//nan detect
    {
        calculateJointPoints();
        return -1;
    }*/
}

void Leg::sendServoSignals()
{
    Point3f relativeAngles;
    relativeAngles.x = angles.x;
    relativeAngles.y = -angles.y;
    relativeAngles.z = CV_PI/2 - (CV_PI - angles.z + angles.y);
  
    servos[0].moveServo(relativeAngles.x);
    servos[1].moveServo(relativeAngles.y);
    servos[2].moveServo(relativeAngles.z);
}