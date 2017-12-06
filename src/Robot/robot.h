#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/core/core.hpp>
#include "util.h"
#include "Robot/leg.h"

class Robot
{
    private:
        rect frame;// robots base

        //position and angles of middles point in robots base
        cv::Point3f position; 
        cv::Point3f angles;

        //initial values ( will be set in case of restart )
        cv::Point3f initPosition;
        cv::Point3f initAngles;
        
        //width and length of robots base
        float width, length;

        cv::Mat Rx, Ry, Rz, R;
        Leg legs[6];
        
        void moveCoordinates(cv::Point3f p, cv::Point3f ang);

    public:
        Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths);

        cv::Point3f getPosition() const {return position;};
        cv::Point3f getAngles() const {return angles;};
        Leg getLeg(int n) const {return legs[n];}
        joints getLegJoints(int n) const;
        rect getFrame() const;

        void restart(cv::Point3f pos, cv::Point3f ang); // restart robot to its base position

        void move(cv::Point3f p); // moves robot base ( only base moves - leg ends stays the same )
        void rotate(cv::Point3f ang); // rotates robot base ( only base rotates - leg ends stays the same )

        void moveLeg(int n, cv::Point3f p);
};

#endif // ROBOT_H
