 #ifndef _VEHICLE_
 #define _VEHICLE_
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <iostream>

#include "../../main/defines.h"
#include "../Kalman/MyKalman.h"
//using namespace cv;

class Vehicle {
public:
	Vehicle();
	std::vector<cv::Point3f> image2Dpositions;
	std::vector<cv::Point3f> world3Dpositions;
	std::vector<cv::Point3f> widthHeightDepth;
	//std::vector<cv::Point3f> directionVectors;
	cv::Point3f vehicleKalman2DPoint;
	cv::Point3f vehicleKalman3DPoint;

	cv::Point3f vehicleKalmanMotionPoint2D, vehicleKalmanMotionPoint3D;

	cv::Point3f upperLeftCorner, lowerRightCorner, nearestPoint, rightPoint, leftPoint;
	float movementType[9]={0};
	float result[numberOfEvents-1]={0};
	int bestMatchIndex=0;
	float avgDist;
	float minDist;
	float finalDirectionX = 0;
	float finalDirectionZ = 0;
	int minDistFoundAtFrame;
	//int foundInFront=0;
	int leftCount=0;
	int rightCount=0;
	int centerCount=0;
	float maxDist;
	int foundAtFrame;
	int lifeTime = 0;
	MyKalman vKalman;
	void initKalman(cv::Point3f world3Dpoint);
	void predictVehicle();
	void getVehiclePoint();
	void getVehicleEgomotionCompensatedDirection();
	void kalmanCorrect(cv::Point3f world3Dpoint);
	bool detectedBefore=false;
	void calcAvgDist();

private:
	float focalLenth = 1612.77;
};
 #endif