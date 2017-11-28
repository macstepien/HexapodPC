#ifndef VIEWER_H
#define VIEWER_H

///Klasa do wyświetlania obrazów w OpenCV

#include "util.h"
#include "robot.h"
#include <opencv2/core/core.hpp>

class Viewer
{
    private:
        const PerspectiveProjection& projector;

    public:
        Viewer(const PerspectiveProjection& projector1) : projector{projector1};

        void drawPoint(cv::Point3f point, cv::Mat& img, cv::Scalar color = cv::Scalar(0,0,255), int size1 = 2);
        void drawLine(cv::Point3f p1, cv::Point3f p2, cv::Mat& img, cv::Scalar color = cv::Scalar(0,0,255), int thickness=1);
        void drawRectangle(rect rec, cv::Mat& img, cv::Scalar color = cv::Scalar(0,0,255), int thickness=1);

        void drawRobot(Robot& rob, cv::Mat& img);
        void drawFloor(cv::Mat& img, int size = 20); // floor is grid line with (size lines, size lines)
        void drawAxis(cv::Point3f pt, cv::Mat& img, cv::Point3f angles = cv::Point3f(0,0,0));

        void drawString(const std::string& str, cv::Mat& img, cv::Point upleft = cv::Point(20,30), float scale = 1., cv::Scalar color = cv::Scalar(125, 255, 0), int thickness = 2);
};

#endif // VIEWER_H
