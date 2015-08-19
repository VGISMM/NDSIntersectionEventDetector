#include "../classes/Disparity/Disparity.h"
#include "../classes/Ransac/ransac.h"
#include "../classes/Kalman/Kalman.h"
#include "../classes/Pressentation/Pressent.h"
#include "../classes/PointCloud/PointCloud.h"
#include "../classes/Counter/Counter.h"
#include "../classes/odometry/odometry.h"

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <viso_stereo.h>

//using namespace cv::;
using namespace std;

float thresholdOffset = 13.1;
cv::VideoWriter output;

int main(int argc,char** argv) {
	cv::Mat img, imgLOI, imgROI, combinedOutput, vdispout, dispout, ovOutLeft, ovOutRight;
	//cv::Mat dispOutGPU;
	Disparity Disparity;
	ransac myransac;
	Pressent Pressent;
	Kalman Kalman;
	PointCloud pointCloud;
	Counter Counter;
	Odometry Odometry;
	
	VisualOdometryStereo::parameters param;
	param.calib.f  = 721.24; // focal length in pixels
	param.calib.cu = 644.93824; // principal point (u-coordinate) in pixels
	param.calib.cv  = 247.6805; // principal point (v-coordinate) in pixels
	param.base  = 0.239813;

	VisualOdometryStereo viso(param);

	cv::Point correctedKalman, predictedKalman;

	bool first=true;
	bool second=false;

	output.open( "../dispTest.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (imageWidth+256,imageHeight*2), true );
	//output.open( "../out/dispTest.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (imageWidth/2+128,imageHeight), true );
    cv::VideoCapture capture(argv[1]);
	
	while(1) {
		/** Get left and right stereo image pair **/
		capture >> img;

		if (img.data==NULL){
			Counter.createVehicleReport();
			output.release();
			break;
			//capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
			//capture >> img;
		}
		imgLOI = img(cv::Rect(cv::Point(0,imageHeightOffset), cv::Size(imageWidth, imageHeight)));
		imgROI = img(cv::Rect(cv::Point(imageWidth,imageHeightOffset), cv::Size(imageWidth, imageHeight)));
		//imwrite("../out/imgLOI.png", imgLOI);
		//imwrite("../out/imgROI.png", imgROI);  
   		cvtColor(imgLOI, ovOutLeft, CV_RGB2GRAY);
   		cvtColor(imgROI, ovOutRight, CV_RGB2GRAY);

   		Counter.overviewMat = cv::Mat::zeros(imageHeight, 512, CV_8UC3);
   		//Disparity.imgLOIGPU.upload(ovOutLeft);
		//Disparity.imgROIGPU.upload(ovOutRight);
		//Disparity.dispFinished.download(dispOutGPU);
		/** Calculate disparity from both LR an RL direction **/
		Disparity.disparityImages(imgLOI.clone(), imgROI.clone(), first);

		Odometry.egoOdometry(ovOutLeft, ovOutRight, &viso);
		//imwrite("../out/dispLR.png", Pressent.representGrayInColor(Disparity.dispLR, 0, 255));
		if (!first)
        {
        	/** Calcluate ego-motion between current and previous frame **/
        	//Odometry.egoOdometry(ovOutLeft, ovOutLeftOld, ovOutRight, ovOutRightOld);
        	/** Remove noise by looking for large varyations between current and previous frame **/
        	Disparity.temporalDisparity();

        	/** Remove darken difficult regions in the disparity map **/
        	Disparity.removeDarkRegions(imgLOI, Disparity.temporalCombinedDisps);
        	//imwrite("../out/postNoise.png", Pressent.representGrayInColor(Disparity.postDarkImage, 0, 255));

        	/** Generate vdisp and run ransac for locationg road surface **/
        	Disparity.generateVdisp(Disparity.postDarkImage);
        	
			myransac.runRansac(Disparity.Vdisp);

			/** Initilize Kalman at second frame **/
			if (second)
        	{
				Kalman.initKalman((float)myransac.slope,(float)myransac.intersection);
				second = false;
			}
			Kalman.kalmanCorrect((float)myransac.slope, (float)myransac.intersection);
		    Kalman.kalmanPredict();
		  	//cout << "measPt;" << (float)myransac.slope << ";" << (float)myransac.intersection << ";statePt;" << Kalman.estimated.at<float>(0) << ";" << Kalman.estimated.at<float>(1) << endl;

		    /** Present the found road surface on the vdisp **/
		    circle(Disparity.Vdisp, myransac.ransacPoint1, 3, cvScalar(150,150,150), 2, 8, 0);
		    circle(Disparity.Vdisp, myransac.ransacPoint2, 3, cvScalar(220,220,220), 2, 8, 0);
            line(Disparity.Vdisp, cv::Point((double)50,(double)(myransac.slope*50+myransac.intersection)-thresholdOffset),  cv::Point((double)200,(double)(myransac.slope*200+myransac.intersection)-thresholdOffset), cvScalar(155,155,155), 1, CV_AA);
            line(Disparity.Vdisp, cv::Point((double)50,(double)(Kalman.estimated.at<float>(0)*50+Kalman.estimated.at<float>(1))-thresholdOffset),  cv::Point((double)200,(double)(Kalman.estimated.at<float>(0)*200+Kalman.estimated.at<float>(1))-thresholdOffset), cvScalar(255,255,255), 1, CV_AA);
            vdispout = Pressent.representGrayInColor(Disparity.Vdisp, 0, 255).clone();
            //imwrite("../out/vdispout.png", vdispout);
            // Remove the road surface based on the found line
            //imshow("disp postDarkImage ",Pressent.representGrayInColor(Disparity.postDarkImage, 0, 255));
    		Disparity.vDispThresholdedImage(Disparity.postDarkImage, Kalman.estimated.at<float>(0), Kalman.estimated.at<float>(1), thresholdOffset);
			//imshow("obstacleImage", Pressent.representGrayInColor(Disparity.obstacleImage, 0, 255)); 
    		dispout = Pressent.representGrayInColor(Disparity.obstacleImage, 0, 255).clone();
    		//imwrite("../out/dispout.png", dispout);
			Counter.vehicles.clear();
			//counter.myvehicles.clear();

    		pointCloud.dispToXYZRGB(Disparity.obstacleImage, imgLOI, Counter, false); // v = day, true = night
    				
    		Counter.frame = imgLOI.clone();
    		circle(Counter.overviewMat, cv::Point(256, imageHeight-50), 10, cv::Scalar( 255, 0, 0 ), 3, 8 );
    		line(Counter.overviewMat, cv::Point(256-Counter.egoMotionX*topViewScaleing, imageHeight-Counter.egoMotionZ*topViewScaleing-50),cv::Point(256, imageHeight-50), cv::Scalar( 250, 250, 0 ),  2, 8 );
    		cout << "X: " << Counter.egoMotionX << "Z: " << Counter.egoMotionZ << endl;
    		Counter.updateCounter(Odometry.pose); 
    	    
            //imwrite("../out/detectedFrame.png", Counter.frame); 
		    combinedOutput=cv::Mat::zeros(imageHeight*2,imageWidth+256,imgLOI.type());
			Counter.frame.copyTo(combinedOutput(cv::Rect(0,0,imageWidth,imageHeight)));
			dispout.copyTo(combinedOutput(cv::Rect(0,imageHeight,imageWidth,imageHeight)));
			//Counter.overviewMat.copyTo(combinedOutput(cv::Rect(imageWidth,0,512,imageHeight)));
			vdispout.copyTo(combinedOutput(cv::Rect(imageWidth,imageHeight,255,imageHeight)));
			cv::putText(combinedOutput, "Detections", cv::Point(40,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
		    cv::putText(combinedOutput, "Fixed disp", cv::Point(40,imageHeight+40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
		    cv::putText(combinedOutput, "V disp", cv::Point(imageWidth+40,imageHeight+40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
		    output.write(combinedOutput);
			cv::resize(combinedOutput,combinedOutput,cv::Size(imageWidth/2+128,imageHeight),CV_INTER_LINEAR);
			imshow("combinedOutput", combinedOutput); 
			//cv::waitKey();
        }
		int k = cv::waitKey(10);
		if (k=='q') {
			Counter.createVehicleReport();
			output.release();
			break;
		}
		if(k=='p') {
			cv::waitKey();
		}
		if (first){
			first = false;
			second = true;
		}
	}
	return 1;
}

