#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/core/core.hpp>
#include "util.h"
#include "leg.h"

class View;

class Robot
{
    private:
        rect lFrame, gFrame;// l - lokalne g - globalne
        cv::Point3f position;
        cv::Point3f initPosition;
        cv::Point3f initAngles;
        cv::Point3f angles;
        float width, length;
        cv::Mat Rx, Ry, Rz, R;
        Leg legs[6];
        void moveCoordinates(cv::Point3f p, cv::Point3f ang);
        int walkingStep;
        cv::Point3f stepsl[6];
        int delayLong; //delay between each step in function walk
        int delayShort; //delay used between each iteration in walk2C walk3C walkRot3C
        bool firstStep;

    public:
        Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths);
        cv::Point3f getPosition(){return position;};
        cv::Point3f getAngles(){return angles;};
        rect getFrame();
        joints getLegJoints(int n);
        void restart(cv::Point3f pos, cv::Point3f ang);

        void move(cv::Point3f p);
        void rotate(cv::Point3f ang);

        void walk(cv::Point3f steps);
        void walkC(cv::Point3f steps, View& view1);
        void walk2C(cv::Point3f steps, View& view1);
        void walk3C(cv::Point3f steps, View& view1);
        void walkAsym(cv::Point3f steps, View& view1);

        void walkRot(float angle);
        void walkRotC(float angle, View& view1);
        void walkRot3C(float angle, View& view1);

        void walkToPoint(cv::Point2f point, View& view1);
        

};

#endif // ROBOT_H
