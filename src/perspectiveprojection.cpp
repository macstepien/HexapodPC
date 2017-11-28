#include "view.h"
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

View::View(double f1, cv::Point3f angles, cv::Point3f translation)
{
    f = f1;
    alfa = angles.x;
    beta = angles.y;
    gamma = angles.z;
    T = position;
}

void View::setAngles(cv::Point3f angles)
{
    alfa = angles.x;
    beta = angles.y;
    gamma = angles.z;
}

cv::Point2f View::drawPoint(cv::Point3f p1)
{
    float x = p1.x-T.x;
    float y = p1.y-T.y;
    float z = p1.z-T.z;

    //rzutowanie punktu tak jak na wikipedii
    Point3f p11;
    p11.x = cos(beta)*(sin(gamma)*y+cos(gamma)*x)-sin(beta)*z;
    p11.y = sin(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))+cos(alfa)*(cos(gamma)*y-sin(gamma)*x);
    p11.z = cos(alfa)*(cos(beta)*z + sin(beta)*(sin(gamma)*y + cos(gamma)*x))-sin(alfa)*(cos(gamma)*y-sin(gamma)*x);

    Point2f point1;
    point1.x = f*(p11.x/p11.z);
    point1.y = f*(p11.y/p11.z);

    return point1;
}