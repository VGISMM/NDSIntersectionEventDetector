#include "../classes/Disparity/Disparity.h"
#include "../classes/Ransac/ransac.h"
#include "../classes/Kalman/Kalman.h"
#include "../classes/Pressentation/Pressent.h"
#include "../classes/PointCloud/PointCloud.h"
#include "../classes/Counter/Counter.h"
#include "../classes/odometry/odometry.h"
#include <png++/png.hpp>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>

//using namespace cv::;
using namespace std;

float thresholdOffset = 20.1;
cv::VideoWriter output;

int main(int argc,char** argv) {
	cv::Mat img, imgLOI, imgROI, myvdispImg, RGBBLOBimg, combinedOutput, vdispout, dispout, ovOutLeft, ovOutLeftOld, ovOutRight, ovOutRightOld;

	Disparity Disparity;
	ransac myransac;
	Pressent Pressent;
	Kalman Kalman;
	PointCloud pointCloud;
	Counter counter;
	Odometry Odometry;
	cv::vector<cv::Rect> returnedBlobs;
	cv::Point correctedKalman, predictedKalman;
	cv::vector<cv::Point> blobCenterCurrent;
	cv::vector<cv::Point> blobCenterPrevious;
	cv::vector<cv::Point> blobCenterFound;

	Matrix donepose;

	string testString;
	bool first=true;
	bool second=false;
	bool blobFirst=true;

	output.open( "../out/dispTest.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (imageWidth+256,imageHeight*2), true );
	//output.open( "../out/dispTest.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (imageWidth/2+128,imageHeight), true );
    cv::VideoCapture capture(argv[1]);
	
	while(1) {
		
		/** Get left and right stereo image pair **/
		capture >> img;

		if (img.data==NULL){
			counter.createVehicleReport();
			output.release();
			break;
			//capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
			//capture >> img;
		}
		imgLOI = img(cv::Rect(cv::Point(0,imageHeightOffset), cv::Size(imageWidth, imageHeight)));
		imgROI = img(cv::Rect(cv::Point(imageWidth,imageHeightOffset), cv::Size(imageWidth, imageHeight)));

		RGBBLOBimg = imgLOI.clone();

   		cvtColor(imgLOI, ovOutLeft, CV_RGB2GRAY);
   		cvtColor(imgROI, ovOutRight, CV_RGB2GRAY);

		/** Calculate disparity from both LR an RL direction **/
		Disparity.disparityImages(imgLOI, imgROI);

	    //imshow("dispLR", Pressent.representGrayInColor(Disparity.dispLR, 0, 255));
		/** Combine the two disperity maps in to one **/
		Disparity.combinedRLLRDisparity(first);
		if (!first)
        {
        	/** Calcluate ego-motion between current and previous frame **/
        	//Matrix currentEgoMotion = Matrix::eye(4);
        	donepose = Odometry.egoOdometry(ovOutLeft, ovOutLeftOld, ovOutRight, ovOutRightOld);
        	//std::cout << donepose << std::endl;

        	/** Remove noise by looking for large varyations between current and previous frame **/
        	Disparity.temporalDisparity();

        	/** Remove darken difficult regions in the disparity map **/
        	Disparity.removeDarkRegions(RGBBLOBimg, Disparity.temporalCombinedDisps);
        	//imshow("temporalCombinedDisps", Disparity.temporalCombinedDisps);
        	/** Generate vdisp and run ransac for locationg road surface **/
        	Disparity.generateVdisp();

        	//dispout = Pressent.representGrayInColor(Disparity.postDarkImage, 0, 255).clone();
        	//imshow("dispout ",dispout);

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
    		//imshow("dispV", Pressent.representGrayInColor(Disparity.Vdisp, 0, 255));
            vdispout = Pressent.representGrayInColor(Disparity.Vdisp, 0, 255).clone();
            /** Remove the road surface based on the found line **/
            //imshow("disp postDarkImage ",Pressent.representGrayInColor(Disparity.postDarkImage, 0, 255));
    		Disparity.vDispThresholdedImage(Kalman.estimated.at<float>(0), Kalman.estimated.at<float>(1), thresholdOffset);
			//imshow("obstacleImage", Pressent.representGrayInColor(Disparity.obstacleImage, 0, 255)); 
    		dispout = Pressent.representGrayInColor(Disparity.obstacleImage, 0, 255).clone();

			//imshow("bitwise res ",Pressent.representGrayInColor(Disparity.obstacleImage, 0, 255));

			counter.vehicles.clear();
			//counter.myvehicles.clear();
    		pointCloud.dispToXYZRGB(Disparity.obstacleImage, RGBBLOBimg, counter);
    				
    		counter.frame=RGBBLOBimg;

    		counter.updateCounter(donepose); 

    		//line( counter.frame, cv::Point(centerLeft, 0), cv::Point(centerLeft, imageHeight), cv::Scalar( 110, 220, 0 ),  2, 8 );
    		//line( counter.frame, cv::Point(centerRight, 0), cv::Point(centerRight, imageHeight), cv::Scalar( 110, 220, 0 ),  2, 8 );
    		
    		for (int vit=0; vit<counter.vehicles.size();vit++)
    		{
    			line( counter.frame, cv::Point(counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].x-10, counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].y), cv::Point(counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].x+10, counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].y), cv::Scalar( 250, 250, 0 ),  2, 8 );
				line( counter.frame, cv::Point(counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].x, counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].y-10), cv::Point(counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].x, counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].y+10), cv::Scalar( 250, 250, 0 ),  2, 8 );
    		
    			// Draw a circle
  				//circle(counter.frame, cv::Point(counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].x, counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].y), 3, cv::Scalar( 0, 0, 255 ), 3, 8 );
  				//putText(RGBBLOBimg, testString, cv::Point(counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].x-20, counter.vehicles[vit].image2Dpositions[counter.vehicles[vit].image2Dpositions.size()-1].y), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 0, 0, 255 ), 3,3);
    		}
    		//imshow("RGBBLOBimg", RGBBLOBimg); 
    		//imshow("imgROI", imgROI); 
    	    //imwrite("RGBClusterimg.png", RGBBLOBimg); 

		    combinedOutput=cv::Mat::zeros(imageHeight*2,imageWidth+256,RGBBLOBimg.type());
			counter.frame.copyTo(combinedOutput(cv::Rect(0,0,imageWidth,imageHeight)));
			dispout.copyTo(combinedOutput(cv::Rect(0,imageHeight,imageWidth,imageHeight)));
			vdispout.copyTo(combinedOutput(cv::Rect(imageWidth,imageHeight,255,imageHeight)));

			cv::putText(combinedOutput, "Detections", cv::Point(40,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
		    cv::putText(combinedOutput, "Fixed disp", cv::Point(40,imageHeight+40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
		    cv::putText(combinedOutput, "V disp", cv::Point(imageWidth+40,imageHeight+40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
		    output.write(combinedOutput);
			//cv::resize(combinedOutput,combinedOutput,cv::Size(imageWidth/2+128,imageHeight),CV_INTER_LINEAR);
			imshow("combinedOutput", combinedOutput); 
			//imshow("2D movement",counter.movementFrame);
			//imshow("Ego Motion 2D",egoMotionFrame);
			//cv::waitKey();
        }
        ovOutLeftOld = ovOutLeft.clone();
        ovOutRightOld = ovOutRight.clone();
		int k = cv::waitKey(100);
		if (k=='q') {
			counter.createVehicleReport();
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

