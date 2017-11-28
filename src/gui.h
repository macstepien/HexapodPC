#ifndef GUI_H
#define GUI_H

///Klasa odpowiedzialna za graficzny interfejs aplikacji

#include "util.h"
#include "robot.h"
#include "viewer.h"
#include "perspectiveprojection.h"
#include <opencv2/core/core.hpp>

class GUI
{
    private:
        PerspectiveProjection projector;
        Viewer viewer1;

        int alfaS, betaS, gammaS;//katy kamery (alfa - os x, beta - os y, gamma - os z)

        float translationStep, rotationStep; //o ile sie przesuwa kamera; o ile sie obraca
        cv::Mat screen;
        std::string voltage;

    public:
        GUI(double f1, cv::Point3f angles, cv::Point3f position);

        void setVoltage(std::string v) {voltage = v;};

        void setAngles(cv::Point3f angles);
        void rotateView(cv::Point3f angles);
        void translateView(cv::Point3f translation);

        void change(char key);
        void update(char key, Robot& rob);
};

#endif // GUI_H
