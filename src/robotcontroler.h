#ifndef ROBOTCONTROLER_H
#define ROBOTCONTROLER_H

#include "robot.h"

class RobotControler
{
	private:
		Robot rob;

		float walkStep; // length of step which robot will take
		float rotStep; // angle of rotation of robot

		float sMoveStep; // how much will robot move its base
		float sRotStep; // how much will robot rotate its base

		cv::Point3f defaultRobotPosition; //coordinates to which robot will be restarted
		cv::Point3f defaultRobotAngles; //angles to which robot will be restarted


	public:
		RobotControler(float walkStep1, float rotStep1, float sMoveStep1, float sRotStep1, cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths);

		///direction:
		/// 0 - forward
		/// 1 - backward
		/// 2 - left
		/// 3 - right
		void walk(int mode, int direction, View& view1);

		///direction:
		/// 0 - clockwise
		///	1 - counterclockwise
		void rotate(int mode, int direction, View& view1);

		///directions
		/// 0 - forward
		/// 1 - backward
		/// 2 - left
		/// 3 - right
		/// 4 - up
		/// 5 - down
		void moveBase(int direction); // move only robots base without moving legends
		void rotateBase(int direction); // rotate only robots base without moving legends

		void reset() {rob.reset(defaultRobotPosition, defaultRobotAngles);}; //resets robot position

		void walkToPoint(cv::Point2f point, View& view1);
        void showoff(View& view1);
};


#endif