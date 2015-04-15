#include "Counter.h"
Counter::Counter() {
	frameCount=0;
	minTrackerCount=2;
	newVehicle=true;
	minVehicleRadius=0.9;

}

void Counter::updateCounter(Matrix vOPose) {
	//cout << "vehicles: " << vehicles.size() << endl; 
	for(int i=0;i<vehicles.size();i++) 
	{
		// looking for matches
		float bestMatchDist = 99.9;
		for(int j=0;j<myvehicles.size();j++) 
		{
			float dist = cv::norm(vehicles[i].world3Dpositions[vehicles[i].world3Dpositions.size()-1] - myvehicles[j].vehicleKalman3DPoint);			
			if (dist<bestMatchDist)
			{
				bestMatchDist = dist;
				bestPointIndex = j;
				//std::cout << "HERE " << std::endl;
			}
		}
		if(minVehicleRadius > bestMatchDist)
		{	
			// match found   tmpVehicle.image2Dpositions.push_back(cv::Point3f(image_x, image_y, range));
			myvehicles[bestPointIndex].world3Dpositions.push_back(vehicles[i].world3Dpositions[0]);
			myvehicles[bestPointIndex].widthHeightDepth.push_back(vehicles[i].widthHeightDepth[0]);
			myvehicles[bestPointIndex].image2Dpositions.push_back(vehicles[i].image2Dpositions[0]);
			/*if (image2Dpositions[0].x > imageWidth/2 && image2Dpositions[0].x < (imageWidth/2+imageWidth/4))
			{
				myvehicles[bestPointIndex].foundInFront++;
			}*/
			myvehicles[bestPointIndex].upperLeftCorner = vehicles[i].upperLeftCorner;
			myvehicles[bestPointIndex].lowerRightCorner = vehicles[i].lowerRightCorner;
			myvehicles[bestPointIndex].nearestPoint = vehicles[i].nearestPoint;

			myvehicles[bestPointIndex].rightPoint = vehicles[i].rightPoint;
			myvehicles[bestPointIndex].leftPoint = vehicles[i].leftPoint;

			myvehicles[bestPointIndex].foundAtFrame=frameCount;
			myvehicles[bestPointIndex].kalmanCorrect(vehicles[i].world3Dpositions[0]);
			myvehicles[bestPointIndex].lifeTime++;

		}
		else
		{
			// new vehicle
			vehicles[i].foundAtFrame=frameCount;
			vehicles[i].initKalman(vehicles[i].world3Dpositions[0]);
			vehicles[i].kalmanCorrect(vehicles[i].world3Dpositions[0]);
			myvehicles.push_back(vehicles[i]);
			//cout << "new vehicle: " << vehicles[i].world3Dpositions[0] << endl; 
		}
	}

	// remove vehicles that have not been detected in the two previous frames
	for(int k=0;k<myvehicles.size();k++) {

		if((frameCount-myvehicles[k].foundAtFrame)>0)
		{
			if(myvehicles[k].lifeTime > minLifeTime)
			{
				circle(frame, cv::Point(imageWidth-20, 20), 6, cv::Scalar(0, 0, 255), 8, 8);
				stringstream distanceText;
				distanceText << "Car: " << oldvehicles.size()+1 << " stored";
				distanceString = distanceText.str();
				putText(frame, distanceString, cv::Point(imageWidth-280, 30), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 255), 3,3);	
		
				myvehicles[k].calcAvgDist();
				oldvehicles.push_back(myvehicles[k]);
				classifyMovement((oldvehicles.size()-1));
			}
			//myvehicles.insert(myvehicles.begin() + j, vehicles[i]);
			myvehicles.erase(myvehicles.begin() + (k));
		}
	}

	foundVehiclesInframe.push_back(myvehicles.size());
	//cout << "myvehicles: " << myvehicles.size() << endl; 
	for (int vit=0; vit<myvehicles.size();vit++)
	{
		//Predict remaing vehicles's positions in the next frame and 
		myvehicles[vit].predictVehicle();
  		myvehicles[vit].getVehiclePoint();

		// Draw a circle and write distance
		if((frameCount-myvehicles[vit].foundAtFrame) == 0)
		{	
			stringstream distanceText;
			distanceText << std::fixed << std::setprecision(1) << myvehicles[vit].nearestPoint.z*distanceMultiplier-egoCarOfset << " m";
			distanceString = distanceText.str();
			//line( frame, cv::Point(myvehicles[vit].nearestPoint.x, myvehicles[vit].nearestPoint.y), cv::Point(myvehicles[vit].rightPoint.x, myvehicles[vit].rightPoint.y), cv::Scalar( 110, 220, 0 ),  2, 8 );
			//line( frame, cv::Point(myvehicles[vit].nearestPoint.x, myvehicles[vit].nearestPoint.y), cv::Point(myvehicles[vit].leftPoint.x, myvehicles[vit].leftPoint.y), cv::Scalar( 110, 220, 0 ),  2, 8 );

			rectangle(frame, cv::Point(myvehicles[vit].upperLeftCorner.x, myvehicles[vit].upperLeftCorner.y), cv::Point(myvehicles[vit].lowerRightCorner.x, myvehicles[vit].lowerRightCorner.y), cv::Scalar( 0, 55, 255 ), 2, 4 );
			circle(frame, cv::Point(myvehicles[vit].vehicleKalman2DPoint.x, myvehicles[vit].vehicleKalman2DPoint.y), 16, cv::Scalar(0, 255, 0 ), 3, 8);
			circle(frame, cv::Point(myvehicles[vit].nearestPoint.x, myvehicles[vit].nearestPoint.y), 6.0, cv::Scalar(255, 0, 0 ), 4, 8);
			putText(frame, distanceString, cv::Point(myvehicles[vit].nearestPoint.x, myvehicles[vit].nearestPoint.y-20), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 0, 255, 0 ), 3,3);	
		}
	}
	frameCount++;
	findNDSEvents(vOPose);
}

		
void Counter::findNDSEvents(Matrix vOPose){

	float motionScaleFactor = 1.0;

	if(vOPose.val[0][3] < 10 && vOPose.val[0][3] > -10 && vOPose.val[2][3] < 10 && vOPose.val[2][3] > -10)
	{
		egoMotionX = vOPose.val[0][3]*motionScaleFactor;
        egoMotionZ = vOPose.val[2][3]*motionScaleFactor;
	}
	else
	{
		egoMotionX = 0;
		egoMotionZ = 0;
	}

	if (firstRun)
	{
		motionKalman.initKalman(egoMotionX,egoMotionZ);
		firstRun = false;
		egoMotionX = 0;
		egoMotionZ = 0;
	}
	else
	{
		motionKalman.kalmanCorrect(egoMotionX, egoMotionZ);
	    motionKalman.kalmanPredict();
	    egoMotionX = motionKalman.estimated.at<float>(0);
	    egoMotionZ = motionKalman.estimated.at<float>(1);
	    //cout << "egoMotionX: " << egoMotionX << "egoMotionZ: " << egoMotionX << endl;
		//std::cout << "ego x: " << egoMotionX << " Kalman x: " << motionKalman.estimated.at<float>(0) << " ego z: " << egoMotionZ << " Kalman z: " << motionKalman.estimated.at<float>(1) << std::endl;
	}

	for(int vit=0; vit<myvehicles.size();vit++)
	{
		// As we want to use prior knowledge, we wait a few frames.
		if (myvehicles[vit].world3Dpositions.size()>1)
		{
			// Find average for x and z directions baesd on last two data entries. 
			float foundVechicleDirectionX=(myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-1].x - myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-2].x);
			float foundVechicleDirectionY=(myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-1].y - myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-2].y);
			float foundVechicleDirectionZ=(myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-1].z - myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-2].z);
           
            Matrix centerOfMassMatrixPrev(1,4);
			Matrix centerOfMassMatrixCur(1,4);
			Matrix centerOfMassMatrixEgoMot(4,4);

			centerOfMassMatrixPrev.val[0][0] = myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-2].x;
			centerOfMassMatrixPrev.val[0][1] = myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-2].y;
			centerOfMassMatrixPrev.val[0][2] = myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-2].z;
			centerOfMassMatrixPrev.val[0][3] = 0;

			centerOfMassMatrixCur.val[0][0] = myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-1].x;
			centerOfMassMatrixCur.val[0][1] = myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-1].y;
			centerOfMassMatrixCur.val[0][2] = myvehicles[vit].world3Dpositions[myvehicles[vit].world3Dpositions.size()-1].z;
			centerOfMassMatrixCur.val[0][3] = 0;

			float finalDirectionX, finalDirectionZ;
	
			if(abs(foundVechicleDirectionX) > minMoveX)
			{
				if(foundVechicleDirectionX > egoMotionX)
				{
					finalDirectionX = foundVechicleDirectionX - egoMotionX;
				}
				else
				{
					finalDirectionX = foundVechicleDirectionX + egoMotionX;
				}
			}
			else
			{
				finalDirectionX = foundVechicleDirectionX;
			}
			if(abs(foundVechicleDirectionZ) > minMoveZ)
			{
				if(foundVechicleDirectionZ > egoMotionZ)
				{
					finalDirectionZ = foundVechicleDirectionZ - egoMotionZ;
				}
				else
				{
					finalDirectionZ = foundVechicleDirectionZ + egoMotionZ;
				}
			}
			else
			{
				finalDirectionZ = foundVechicleDirectionZ;
			}
			
			myvehicles[vit].finalDirectionX = finalDirectionX;
			myvehicles[vit].finalDirectionZ = finalDirectionZ;
			
			myvehicles[vit].getVehicleEgomotionCompensatedDirection();
			
			line( frame, cv::Point(myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].x-10, myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].y), cv::Point(myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].x+10, myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].y), cv::Scalar( 250, 250, 0 ),  2, 8 );
			line( frame, cv::Point(myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].x, myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].y-10), cv::Point(myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].x, myvehicles[vit].image2Dpositions[myvehicles[vit].image2Dpositions.size()-1].y+10), cv::Scalar( 250, 250, 0 ),  2, 8 );
			line(frame, cv::Point(myvehicles[vit].vehicleKalman2DPoint.x, myvehicles[vit].vehicleKalman2DPoint.y),  cv::Point(myvehicles[vit].vehicleKalmanMotionPoint2D.x, myvehicles[vit].vehicleKalmanMotionPoint2D.y), cvScalar(0,255,0), 1, CV_AA);			

			//cout << "Point: " << myvehicles[vit].vehicleKalman3DPoint << "motion point: " << myvehicles[vit].vehicleKalmanMotionPoint3D << endl;

			circle(overviewMat, cv::Point(myvehicles[vit].vehicleKalman3DPoint.x*50+100, imageHeight-20-myvehicles[vit].vehicleKalman3DPoint.z*30), 8, cv::Scalar( 0, 0, 180 ), 3, 8 );
    		line(overviewMat, cv::Point(myvehicles[vit].vehicleKalman3DPoint.x*50+100, imageHeight-20-myvehicles[vit].vehicleKalman3DPoint.z*30), cv::Point(myvehicles[vit].vehicleKalmanMotionPoint3D.x*50+100, imageHeight-20-myvehicles[vit].vehicleKalmanMotionPoint3D.z*30), cv::Scalar( 250, 250, 0 ),  2, 8 );
    		imwrite("../out/overviewMat.png", overviewMat);
			// disable ego motion:
			//finalDirectionX = foundVechicleDirectionX;
			//finalDirectionZ = foundVechicleDirectionZ;
			//std::cout << "vehicle-" << vit <<" x: " << foundVechicleDirectionX<< " final x: "<< finalDirectionX << std::endl;
            //std::cout << "vehicle-" << vit <<" z: " << foundVechicleDirectionZ<< " final z: "<< finalDirectionZ << std::endl;

			if (abs(finalDirectionX) < minMoveX && finalDirectionZ > minMoveZ) // S
			{	
   				myvehicles[vit].movementType[SameDirection]++;
				putText(frame, "S", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
			else if (abs(finalDirectionX) < minMoveX && finalDirectionZ < -minMoveZ) // O
   			{	
 				myvehicles[vit].movementType[OppositeDirection]++;
				putText(frame, "O", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
		    else if (finalDirectionX > minMoveX && finalDirectionZ > minMoveZ) // SR
			{	
   				myvehicles[vit].movementType[SameRight]++;
				putText(frame, "SR", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
			else if (finalDirectionX < -minMoveX && finalDirectionZ > minMoveZ) // SL
   			{	
				myvehicles[vit].movementType[SameLeft]++;
				putText(frame, "SL", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
			else if (finalDirectionX > minMoveX && finalDirectionZ < -minMoveZ) // OR
			{	
   				myvehicles[vit].movementType[OppositeRight]++;
				putText(frame, "OR", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
			else if (finalDirectionX < -minMoveX && finalDirectionZ < -minMoveZ) // OL
   			{	
 				myvehicles[vit].movementType[OpporsiteLeft]++;
				putText(frame, "OL", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
			else if (finalDirectionX > minMoveX && abs(finalDirectionZ) < minMoveZ) // R
			{	
   				myvehicles[vit].movementType[RightDirection]++;
				putText(frame, "R", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
			else if (finalDirectionX < -minMoveX && abs(finalDirectionZ) < minMoveZ) // L
   			{	
 				myvehicles[vit].movementType[LeftDirection]++;
				putText(frame, "L", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
    		else  // other
   			{
   				myvehicles[vit].movementType[NoMovement]++;
				putText(frame, "OO", cv::Point(myvehicles[vit].vehicleKalman2DPoint.x-10, myvehicles[vit].vehicleKalman2DPoint.y-40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar( 255, 0, 0 ), 3,3);
			}
		}
		else
		{
			myvehicles[vit].finalDirectionX = 0;
			myvehicles[vit].finalDirectionZ = 0;
		}
	}
}

void Counter::createVehicleReport() 
{	
	for(int k=0;k<myvehicles.size();k++) 
	{
		if(myvehicles[k].lifeTime > minLifeTime)
		{
			myvehicles[k].calcAvgDist();
			oldvehicles.push_back(myvehicles[k]);
			classifyMovement((oldvehicles.size()-1));
		}
	}

	int sumOfVehicles = 0;
	for(int k=0;k<foundVehiclesInframe.size();k++) 
	{
		sumOfVehicles += foundVehiclesInframe[k];
	}

	cout << "Avg number of cars in video clip: " << (float)sumOfVehicles/foundVehiclesInframe.size() << endl;
	cout << "oldvehicles.size(): " << oldvehicles.size() << endl;

	for(int k=0;k<oldvehicles.size();k++) 
	{
		cout << "Car " << k << " did event: " << oldvehicles[k].bestMatchIndex << endl;
		eventScores[oldvehicles[k].bestMatchIndex]++;
	}

	for(int k=0; k<numberOfEvents; k++)
	{
		if (k==6)
		{
			cout << "Event 'could not be matched " << k << "' was registrated " << eventScores[k] << " times" << endl;
		}
		else
		{
			cout << "Event " << k << " was registrated " << eventScores[k] << " times" << endl;
		}
		
	}

	for(int k=0;k<oldvehicles.size();k++) 
	{
		if (oldvehicles[k].centerCount > minCenterDetections)
		{
			cout << "Car " << k << " with avg distance of : " << oldvehicles[k].avgDist*distanceMultiplier-egoCarOfset << ", min distance of: " << 
			oldvehicles[k].minDist*distanceMultiplier-egoCarOfset << "m at frame: " << oldvehicles[k].minDistFoundAtFrame << ", was found to be in front" << endl;
		}
	}
}

void Counter::classifyMovement(int k) 
{
	int sum=0;
	for(int i=0; i<numberOfMovementTypes; i++)
	{
		//cout << "bin " << i << " size: " << oldvehicles[k].movementType[i] << endl;
		sum += oldvehicles[k].movementType[i];
	}

	for(int i=0; i<numberOfMovementTypes; i++)
	{
		oldvehicles[k].movementType[i]=oldvehicles[k].movementType[i]/sum;
		cout << "bin " << i << " size: " << oldvehicles[k].movementType[i] << endl;
	}

	for(int i=0; i<numberOfEvents-1; i++)
	{
		oldvehicles[k].result[i] = sqrt(
		 pow(oldvehicles[k].movementType[0]-movementTypes[i][0],2)+pow(oldvehicles[k].movementType[0]-movementTypes[i][0],2)
		+pow(oldvehicles[k].movementType[1]-movementTypes[i][1],2)+pow(oldvehicles[k].movementType[2]-movementTypes[i][2],2)
		+pow(oldvehicles[k].movementType[3]-movementTypes[i][3],2)+pow(oldvehicles[k].movementType[4]-movementTypes[i][4],2)
		+pow(oldvehicles[k].movementType[5]-movementTypes[i][5],2)+pow(oldvehicles[k].movementType[6]-movementTypes[i][6],2)
		+pow(oldvehicles[k].movementType[7]-movementTypes[i][7],2)+pow(oldvehicles[k].movementType[8]-movementTypes[i][8],2));
	}

	int index = 0;
	for(int i = 0; i < numberOfEvents-1; i++)
	{
    	if(oldvehicles[k].result[i] < oldvehicles[k].result[index])
    	{
    		index = i; 
    	}
    	cout << "This was car: " << k << " match " << i << " was: " << oldvehicles[k].result[i] << endl;              
	}
	if (oldvehicles[k].result[index] < 0.7)
	{
		oldvehicles[k].bestMatchIndex = index;
		cout << "This was car: " << k << " Best match was: " << index << endl;
	}
	else
	{
		oldvehicles[k].bestMatchIndex = 6;
		cout << "It was not possilbe to match an event for this car: " << k << " Best match was: " << index << endl;
	}
	
}
