#ifndef UTIL_H
#define UTIL_H

#include <opencv2/core/core.hpp>
#include <mutex>

struct rect
{
    cv::Point3f ul, ur, dl, dr; //up-left up-right down-left down-right
};

struct joints
{
    cv::Point3f A,B,C,D;
};

struct voltage
{
    std::string voltageStr;
    std::mutex own;
};

cv::Point3f rotate3D(cv::Point3f pt, cv::Point3f angles); ///rotates given point by angles in 3D

void battery(bool communication, char* adres, bool* end, voltage* v);
char getChar();


#endif // UTIL_H
