#ifndef UTIL_H
#define UTIL_H

#include <opencv2/core/core.hpp>

struct rect
{
    cv::Point3f ul, ur, dl, dr; //up-left up-right down-left down-right
};

struct joints
{
    cv::Point3f A,B,C,D;
};

cv::Point3f rotate3D(cv::Point3f pt, cv::Point3f angles); ///rotates given point by angles in 3D

#endif // UTIL_H
