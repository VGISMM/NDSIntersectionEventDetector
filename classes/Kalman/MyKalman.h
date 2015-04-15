/*--------- INCLUDES------------*/
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

//using namespace cv;
using namespace std;

/*--------- Class with attributes and methods declarations------------*/
class MyKalman{
public:
	MyKalman();
	void setKalman(float x, float y, float z);
	void kalmanPredict();
	void Correct(float x, float y, float z);
	cv::Mat prediction, estimated;
	//cv::Mat_<float>  = cv::Mat_<float>::zeros(3,1);
	cv::Mat_<float> measurement = cv::Mat_<float>::zeros(3,1);
private:
	cv::KalmanFilter KF;
	
	//Mat_<float> measurement(2,1); 
	//Mat_<float> state(4, 1); // (x, y, Vx, Vy)

};