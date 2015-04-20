
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#import <viso_stereo.h>
#include "../../main/defines.h"

//using namespace cv;
using namespace std;

class Odometry {
public:
	Odometry();
	Matrix egoOdometry(cv::Mat imageLeft, cv::Mat imageLeftOld, cv::Mat imageRight, cv::Mat imageRightOld);
	Matrix pose;
private:
    VisualOdometryStereo::parameters param;
    
};
