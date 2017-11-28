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

        double alfa, beta, gamma;//katy kamery (alfa - os x, beta - os y, gamma - os z)
        int alfaS, betaS, gammaS;
        double f; //
        cv::Point3f T; //T - pozycja
        float translationStep, rotationStep; //o ile sie przesuwa kamera; o ile sie obraca
        cv::Mat screen;
        std::string voltage;

    public:
        View(double f1, cv::Point3f angles, cv::Point3f position) : projector(f1, angles, position), viewer1(projector);

        void setVoltage(std::string v) {voltage = v;};

        void setAngles(cv::Point3f angles);
        void rotateView(cv::Point3f angles);
        void translateView(cv::Point3f translation);

        void change(char key);
        void update(char key, Robot& rob);
        void updateStr(char key, Robot& rob, std::string voltage);
};

#endif // GUI_H
