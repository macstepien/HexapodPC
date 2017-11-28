#include "viewer.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;


void Viewer::drawPoint(cv::Point3f p1, cv::Mat& img, Scalar color, int size1)
{
    Point p = projector.projectPoint(p1);
    p.x += 320;
    p.y += 240;
    circle(img, p, size1, Scalar(0, 0, 255), -1);
}

void Viewer::drawLine(cv::Point3f p1, cv::Point3f p2, cv::Mat& img, Scalar color, int thickness)
{
    Point lp1 = projector.projectPoint(p1);
    lp1.x += 320;
    lp1.y += 240;

    Point lp2 = projector.projectPoint(p2);
    lp2.x += 320;
    lp2.y += 240;

    line(img, lp1, lp2, color, thickness);
}

void Viewer::drawRectangle(rect rec, cv::Mat& img, Scalar color, int thickness)
{
    drawLine(rec.dl, rec.dr, img, color, thickness);
    drawLine(rec.dr, rec.ur, img, color, thickness);
    drawLine(rec.ur, rec.ul, img, color, thickness);
    drawLine(rec.ul, rec.dl, img, color, thickness);
}

void Viewer::drawRobot(Robot& rob, cv::Mat& img)
{
    int thickness = 2;
    int size1 = 4;
    drawRectangle(rob.getFrame(), img, Scalar(255,0,0), thickness);
    joints joint;
    for(int i=0; i<6; ++i)
    {
        joint = rob.getLegJoints(i);
        drawLine(joint.A, joint.B, img, Scalar(255,0,0), thickness);
        drawLine(joint.B, joint.C, img, Scalar(255,0,0), thickness);
        drawLine(joint.C, joint.D, img, Scalar(255,0,0), thickness);

        drawPoint(joint.A, img, Scalar(0,0,255), size1);
        drawPoint(joint.B, img, Scalar(0,0,255), size1);
        drawPoint(joint.C, img, Scalar(0,0,255), size1);
        drawPoint(joint.D, img, Scalar(0,0,255), size1);
    }
}

void Viewer::drawFloor(cv::Mat& img, int size)
{
    int i;
    for(i = -size/2; i <= size/2; ++i)
    {
        drawLine(Point3f(i*size/2.,0,0), Point3f(i*size/2.,0,200), img, Scalar(0,0,0));
    }
    for(i = 0; i <= size; ++i)
    {
        drawLine(Point3f(-100,0,i*size/2.), Point3f(100,0,i*size/2.), img, Scalar(0,0,0));
    }
}

void Viewer::drawAxis(cv::Point3f pt, cv::Mat& img, cv::Point3f angles)
{
    drawPoint(pt, img);

    drawLine(pt, rotate3D(Point3f(10,0,0),angles)+pt, img, Scalar(255,0,0), 2);//x niebieski
    drawLine(pt, rotate3D(Point3f(0,10,0),angles)+pt, img, Scalar(0,255,0), 2);//y zielony
    drawLine(pt, rotate3D(Point3f(0,0,10),angles)+pt, img, Scalar(0,0,255), 2);//z czerwony
}

void Viewer::drawString(const std::string& str, cv::Mat& img, cv::Point upleft, float scale, cv::Scalar color, int thickness)
{
    if(str != "")
    {
        putText(img, str, upleft, FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    }
}