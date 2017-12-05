#include "Robot/inversekinematics.h"

using namespace cv;

Point3f InverseKinematics::calculate(Point3f desiredPosition, Point3f legLengths)
{
    float L = sqrt( pow(desiredPosition.x,2) + pow(desiredPosition.z,2) );
    float iksw = sqrt( pow((L-legLengths.x),2) + pow(desiredPosition.y,2) );

    float a1 = atan((L-legLengths.x)/desiredPosition.y);
    float a2 = acos((pow(legLengths.z,2)-pow(legLengths.y,2)-pow(iksw,2))/((-2.)*iksw*legLengths.y));
    float b = acos((pow(iksw,2)-pow(legLengths.y,2)-pow(legLengths.z,2))/((-2.)*legLengths.y*legLengths.z));

    Point3f angles;

    if(desiredPosition.x && desiredPosition.z)
        angles.x = -atan(desiredPosition.z/desiredPosition.x);
    else
        angles.x = 0;

    angles.y = -(a1+a2-0.5*CV_PI);
    angles.z = (CV_PI - b + angles.y);

    return angles;
}