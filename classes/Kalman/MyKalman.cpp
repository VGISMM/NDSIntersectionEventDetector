#include "MyKalman.h"

MyKalman::MyKalman(){

}

void MyKalman::setKalman(float x, float y, float z)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(6, 3, 0);

    measurement.at<float>(0) = x;
    measurement.at<float>(1) = y;
	measurement.at<float>(2) = z;

    KF.statePre.setTo(0);
    KF.statePre.at<float>(0) = x;
    KF.statePre.at<float>(1) = y;
    KF.statePre.at<float>(2) = z;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0) = x;
    KF.statePost.at<float>(1) = y; 
    KF.statePost.at<float>(2) = z; 

    KF.transitionMatrix = *(cv::Mat_<float>(6, 6) << 1,0,0,1.5,0,0,   0,1,0,0,0.2,0,  0,0,1,0,0,0.8,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(0.008)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
}

void MyKalman::kalmanPredict() 
{
    prediction = KF.predict();
   // Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    //cout << "prediction x: " << prediction.at<float>(0) << endl;
    //cout << "prediction y: " << prediction.at<float>(1) << endl;
    //cout << "prediction z: " << prediction.at<float>(2) << endl;

}

void MyKalman::Correct(float x, float y, float z)
{	
    measurement(0) = x;
    measurement(1) = y;
    measurement(2) = z;
    
    estimated = KF.correct(measurement);
   // cout << "estimated y: " << estimated.at<float>(1) << endl;
    //Point statePt(estimated.at<float>(0),estimated.at<float>(1));
}
