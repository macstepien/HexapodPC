#ifndef ROBOTCONTROLER_H
#define ROBOTCONTROLER_H

#include <opencv2/core/core.hpp>
#include "Robot/robot.h"
#include "Robot/robotwalk.h"
#include "GUI/gui.h"

class RobotControler
{
	private:
		Robot rob;
		RobotWalk walker;

		float walkStep; // length of step which robot will take
		float rotStep; // angle of rotation of robot

		float sMoveStep; // how much will robot move its base
		float sRotStep; // how much will robot rotate its base

		cv::Point3f defaultRobotPosition; //coordinates to which robot will be restarted
		cv::Point3f defaultRobotAngles; //angles to which robot will be restarted

		int mode;

		bool started;
		int walkingDirection;


	public:
		RobotControler(float walkStep1, float rotStep1, float sMoveStep1, float sRotStep1, cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths);

		Robot& getRobot() {return rob;};

		//simple walking
		void mode3(char direction);

		//automatic walking
		void mode4(char direction, GUI& view1);

		//smooth walking
		void mode5(char direction, GUI& view1);

		void mode6(char direction, int stage, GUI& view1);

		///directions
		/// 0 - forward
		/// 1 - backward
		/// 2 - left
		/// 3 - right
		/// 4 - up
		/// 5 - down
		void moveBase(char direction); // move only robots base without moving legends
		void rotateBase(char direction); // rotate only robots base without moving legends

		void restart() {rob.restart(defaultRobotPosition, defaultRobotAngles);}; //resets robot position

		void walkToPoint(cv::Point2f point, GUI& view1);
        void showoff(GUI& view1);

        void control(char key, GUI& view1);
};


#endif