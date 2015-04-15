#include "ransac.h"
using namespace std;
ransac::ransac(){}

void ransac::runRansac(cv::Mat image) {
    ransac::extractPoints(image);
    allPointsLength = allPoints.size();
    if(allPointsLength>10)
    {
        ransac::createLine();
    }
}

void ransac::extractPoints(cv::Mat& img) {
    allPoints.clear();
    for (int j = 80; j < img.cols-40; j++ ) {
        for (int i = 280; i < img.rows-20; i++) {
            if(img.at<uchar>(i, j) > 20){
                allPoints.push_back(cvPoint2D32f((float)j,(float)i)); 
            } 
        }
    }
}

void ransac::createLine() {
    while(count < 20000) {
        int inLiers = 0;
        int point1 = rand()%allPointsLength;
        int point2 = rand()%allPointsLength;   

        linePoint1.x = allPoints[point1].x;
        linePoint1.y = allPoints[point1].y;
        linePoint2.x = allPoints[point2].x;
        linePoint2.y = allPoints[point2].y;

        if(abs(allPoints[point2].x-allPoints[point1].x)>10.1) 
        {

            for(int i=0;i<allPointsLength/3;i++) 
            {
                int point0 = rand()%allPointsLength;
                testPoint.x = allPoints[point0].x;
                testPoint.y = allPoints[point0].y;
                
                //d = ((linePoint2.x-linePoint1.x)*(linePoint1.y-testPoint.y)-(linePoint1.x-testPoint.x)*(linePoint2.y-linePoint1.y))/sqrt(((linePoint2.x-linePoint1.x)*(linePoint2.x-linePoint1.x))+((linePoint2.y-linePoint1.y)*(linePoint2.y-linePoint1.y)));
                double normalLength = hypot(linePoint2.x - linePoint1.x, linePoint2.y - linePoint1.y);
                double distance = (double)((testPoint.x - linePoint1.x) * (linePoint2.y - linePoint1.y) - (testPoint.y - linePoint1.y) * (linePoint2.x - linePoint1.x)) / normalLength;
                if(abs(distance) < maxDistance) 
                {
                    inLiers++;
                }
            }
            
            if(inLiers>=bestInliers) 
            {
                bestInliers=inLiers;
                ransacPoint1 = linePoint1;
                ransacPoint2 = linePoint2;
            }
        }
        count++;
    }
    //cout << "slope;" << (float)(ransacPoint1.y-ransacPoint2.y)/(float)(ransacPoint1.x-ransacPoint2.x) << "inters;" << (float)((ransacPoint2.y-slope*ransacPoint2.x)-0) << endl;
    if (0!=(ransacPoint1.y-ransacPoint2.y)&&0!=(ransacPoint1.x-ransacPoint2.x))
    {
        if ((float)(ransacPoint1.y-ransacPoint2.y)/(float)(ransacPoint1.x-ransacPoint2.x)>0.8 && (float)(ransacPoint1.y-ransacPoint2.y)/(float)(ransacPoint1.x-ransacPoint2.x)<2.8)
        {
            slope = (float)(ransacPoint1.y-ransacPoint2.y)/(float)(ransacPoint1.x-ransacPoint2.x);
        }
        if ((float)((ransacPoint2.y-slope*ransacPoint2.x)-0)>0.1 && (float)((ransacPoint2.y-slope*ransacPoint2.x)-0)<300.1)
        {
            intersection = (float)((ransacPoint2.y-slope*ransacPoint2.x)-0);
        }
    }
    count = 0;
    bestInliers = 0;
}





