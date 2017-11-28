#include "perspectiveprojection.h"

using namespace cv;

cv::Point2f PerspectiveProjection::projectPoint(cv::Point3f p1) const
{
    float x = p1.x-T.x;
    float y = p1.y-T.y;
    float z = p1.z-T.z;

    //rzutowanie punktu tak jak na wikipedii
    Point3f p11;
    p11.x = cos(angles.y)*(sin(angles.z)*y+cos(angles.z)*x)-sin(angles.y)*z;
    p11.y = sin(angles.x)*(cos(angles.y)*z + sin(angles.y)*(sin(angles.z)*y + cos(angles.z)*x))+cos(angles.x)*(cos(angles.z)*y-sin(angles.z)*x);
    p11.z = cos(angles.x)*(cos(angles.y)*z + sin(angles.y)*(sin(angles.z)*y + cos(angles.z)*x))-sin(angles.x)*(cos(angles.z)*y-sin(angles.z)*x);

    Point2f point1;
    point1.x = f*(p11.x/p11.z);
    point1.y = f*(p11.y/p11.z);

    return point1;
}