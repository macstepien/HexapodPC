#ifndef VIEW_H
#define VIEW_H

///Obsluga wyswietlania obrazu
///Rzutowanie perspektywiczne na plaszczyzne ekranu
///Tu jest to opisane i sa wykorzystywane rownania https://en.wikipedia.org/wiki/3D_projection

#include "util.h"
#include "robot.h"
#include <opencv2/core/core.hpp>

class View
{
    private:
        double alfa, beta, gamma;//katy kamery (alfa - os x, beta - os y, gamma - os z)
        int alfaS, betaS, gammaS;
        double f; //
        cv::Point3f T; //T - pozycja
        float translationStep, rotationStep; //o ile sie przesuwa kamera; o ile sie obraca
        cv::Mat screen;
        std::string voltage;

    public:
        View(double f1, cv::Point3f angles, cv::Point3f translation);

        void setVoltage(std::string v) {voltage = v;};

        void setAngles(cv::Point3f angles);
        void rotateView(cv::Point3f angles);
        void translateView(cv::Point3f translation);

        void drawPoint(cv::Point3f point, cv::Scalar color = cv::Scalar(0,0,255), int size1 = 2);
        void drawLine(cv::Point3f p1, cv::Point3f p2, cv::Scalar color = cv::Scalar(0,0,255), int thickness=1);
        void drawRectangle(rect rec, cv::Scalar color = cv::Scalar(0,0,255), int thickness=1);

        void drawRobot(Robot& rob);
        void drawFloor();
        void drawAxis(cv::Point3f pt);

        void change(char key);
        void update(char key, Robot& rob);
        void updateStr(char key, Robot& rob, std::string voltage);
};

#endif // VIEW_H
