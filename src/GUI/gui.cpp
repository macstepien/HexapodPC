#include "gui.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgproc.hpp"

#define SCALE 0.017453

using namespace std;
using namespace cv;

GUI::GUI(double f1, cv::Point3f angles, cv::Point3f position) : projector(f1, angles, position), viewer1(projector)
{
    translationStep = 10;
    rotationStep = 0.1;

    screen = Mat(480, 640, CV_8UC3, Scalar(255,255,255));

    namedWindow("img");

    alfaS = 105;
    betaS = 180;
    gammaS = 180;

    createTrackbar("alfa", "img", &alfaS, 360);
    createTrackbar("beta", "img", &betaS, 360);
    createTrackbar("gamma", "img", &gammaS, 360);
}

void GUI::update(char key, Robot& rob)
{
    screen = Mat(480, 640, CV_8UC3, Scalar(255,255,255));

    change(key);

    projector.setAngles(Point3f((alfaS-180)*SCALE, (betaS-180)*SCALE, (gammaS-180)*SCALE));

    viewer1.drawFloor(screen);
    viewer1.drawAxis(Point3f(0,0,0), screen);
    viewer1.drawAxis(rob.getPosition(), screen, rob.getAngles());
    viewer1.drawRobot(rob, screen);

    if(voltage != "")
    {
        viewer1.drawString(voltage, screen);
    }

    imshow("img", screen);
}

void GUI::rotateView(cv::Point3f angles)
{
    projector.rotate(angles);
}


void GUI::translateView(cv::Point3f translation)
{
    //odpowiednie przeksztalcanie przesuniecia aby poruszac sie relatywnie do aktualnych katow, a nie po globalnych osiach(tak lepiej sie to uzytkuje)
    projector.translate(rotate3D(translation, projector.getAngles()));
}

void GUI::change(char key)
{
    switch(key)
    {
        case 'w':
            translateView(Point3f(0,-translationStep,0));
            break;
        case 's':
            translateView(Point3f(0,translationStep,0));
            break;
        case 'a':
            translateView(Point3f(-translationStep,0,0));
            break;
        case 'd':
            translateView(Point3f(translationStep,0,0));
            break;
        case 'q':
            translateView(Point3f(0,0,translationStep));
            break;
        case 'e':
            translateView(Point3f(0,0,-translationStep));
            break;
        case 'r':
            rotateView(Point3f(rotationStep,0,0));
            break;
        case 'f':
            rotateView(Point3f(-rotationStep,0,0));
            break;
        case 't':
            rotateView(Point3f(0,rotationStep,0));
            break;
        case 'g':
            rotateView(Point3f(0,-rotationStep,0));
            break;
        case 'y':
            rotateView(Point3f(0,0,rotationStep));
            break;
        case 'h':
            rotateView(Point3f(0,0,-rotationStep));
            break;
    }
}
