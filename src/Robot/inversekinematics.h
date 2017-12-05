#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include <opencv2/core/core.hpp>

class InverseKinematics
{
	private:
		
	public:
		static cv::Point3f calculate(cv::Point3f desiredPosition, cv::Point3f legLengths);
};

#endif