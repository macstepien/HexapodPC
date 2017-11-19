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

#endif // UTIL_H
