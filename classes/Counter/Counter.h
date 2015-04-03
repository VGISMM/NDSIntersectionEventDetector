#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <png++/png.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#import "../Vehicle/Vehicle.h"
#include "../Kalman/Kalman.h"
#include "../../main/defines.h"

#include "matrix.h"
//#include <viso_stereo.h>

using namespace std;

class Counter {
public:
	Counter();
	int count;
	int frameCount;
	int bestPointIndex;
	bool newVehicle;
	float minVehicleRadius;
	cv::Mat frame;
	int minTrackerCount;
	//cv::Mat movementFrame;
	cv::vector<Vehicle> vehicles;
	cv::vector<Vehicle> myvehicles;
	cv::vector<Vehicle> candidateVehicles;
	cv::vector<Vehicle> oldvehicles;
	Matrix egoMotionMove = Matrix::eye(4);
	Matrix previousPose = Matrix::eye(4);
	

	bool firstRun = true;

	Kalman motionKalman;
	/*
	#define SameDirection 0
	#define OppositeDirection 1
	#define SameRight 2
	#define SameLeft 3
	#define OppositeRight 4
	#define OpporsiteLeft 5
	#define LeftDirection 6
	#define RightDirection 7
	#define NoMovement 8
	*/
	float movementTypes[6][9] = {
								{0,0,0.2,0,0.2,0,0,0.6,0},   // 0 right - intersection Other vehicle entering intersection - straight across path
								{0,0,0,0.2,0,0.2,0.6,0,0},   // 1 left - intersection Other vehicle entering intersection - straight across path
								{0,0.1,0.1,0,0.4,0,0,0.3,0.1}, // 2 leftturnacross - Other vehicle entering intersection - left turn across path
								{0,0.2,0,0,0,0.4,0.3,0,0.1}, // 3 oppositeturn - Other vehicle entering intersection - turning onto opposite direction
								{0.3,0,0.3,0,0,0,0.3,0,0.1}, // 4 sameDirectionturnshort
								{0.3,0,0,0.3,0,0,0,0.4,0}	 // 5 sameDirectionturnlong
							   };
	int eventScores[7] = {0};
	cv::vector<int> foundVehiclesInframe;
	
	void updateCounter(Matrix vOPose);
	void findNDSEvents(Matrix vOPose);
	void createVehicleReport();
	void classifyMovement(int k);
private:
	string distanceString;
};

