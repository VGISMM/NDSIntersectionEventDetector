#ifndef DISPARITY_H
#define DISPARITY_H
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "../../main/defines.h"
//using namespace cv;

class Disparity {
public:
	cv::Mat dispLR, dispRL;
	cv::Mat combinedDisps=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::Mat temporalCombinedDisps=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::Mat obstacleImage=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::Mat postDarkImage=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::StereoSGBM sgbm;

	Disparity();
	void disparityImages(cv::Mat imgL, cv::Mat imgR);
	void combinedRLLRDisparity(bool first);
	void temporalDisparity();
	void vDispThresholdedImage(float slope, float intersection, float thresholdOffset);
	void removeDarkRegions(cv::Mat orgImgLoi, cv::Mat modifiedDispImage);

	//void reduceNumberOfBins(cv::Mat dispImg);
	void generateVdisp();
	void generateUdisp(cv::Mat dispImg);
	//void generateVdispPlanes(cv::Mat dispImg);
	//void generateUdispPlanes(cv::Mat dispImg);
	cv::Mat Vdisp = cv::Mat::zeros(imageHeight, 255, CV_8UC1);
	cv::Mat Udisp = cv::Mat::zeros(255, imageWidth, CV_8UC1);
	//cv::Mat reducedImg = cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
	cv::Mat VdispPlanes = cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
	cv::Mat UdispPlanes = cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
private:
	cv::Mat temporalDisp;
	cv::Vec3b pixelValueBGR;
    int pixelValueBlue;
    int pixelValueGreen;
    int pixelValueRed;
};
#endif