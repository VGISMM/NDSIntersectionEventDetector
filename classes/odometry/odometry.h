
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>
#include <viso_stereo.h>
#include "../../main/defines.h"

//using namespace cv;
using namespace std;

class Odometry {
public:
	Odometry();
	Matrix egoOdometry(cv::Mat imageLeft, cv::Mat imageLeftOld, cv::Mat imageRight, cv::Mat imageRightOld);
private:
	VisualOdometryStereo::parameters param;
    Matrix pose;
    

};
