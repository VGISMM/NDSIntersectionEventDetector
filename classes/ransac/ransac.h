
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>

//using namespace cv;

class ransac {
public:
	ransac();
	float slope = 1.8;
	float intersection = 180;
	double d;
	int allPointsLength;
	void runRansac(cv::Mat image);
	std::vector<CvPoint2D32f> allPoints;
	cv::Point linePoint1, linePoint2, testPoint, ransacPoint1, ransacPoint2;
private:
	const double maxDistance = 1.5;
	//const int minInliers = 200;
	int bestInliers = 0;

	void extractPoints(cv::Mat& img);
	void createLine();
	int count=0;
};
