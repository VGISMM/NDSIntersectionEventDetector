#include "Disparity.h"
Disparity::Disparity() 
{
  sgbm = cv::StereoSGBM::create(0,64,3);
  sgbm->setPreFilterCap(1);
  int sgbmWinSize = 9;
  sgbm->setBlockSize(sgbmWinSize);
  sgbm->setP1(8*3*sgbmWinSize*sgbmWinSize);
  sgbm->setP2(32*3*sgbmWinSize*sgbmWinSize);
  sgbm->setMinDisparity(0);
  //sgbm->setNumDisparities(numberOfDisparities);
  sgbm->setUniquenessRatio(1); //10 for daytime
  sgbm->setSpeckleWindowSize(3); //2
  sgbm->setSpeckleRange(1); //
  sgbm->setDisp12MaxDiff(10);
  sgbm->setMode(cv::StereoSGBM::MODE_SGBM); //StereoSGBM::MODE_SGBM || cv::StereoSGBM::MODE_HH
}

void Disparity::disparityImages(cv::Mat imgL, cv::Mat imgR, bool first) 
{
  if(!first)
  {
      temporalDisp = dispRLLR.clone();
  }
  sgbm->compute(imgL, imgR, dispLR);
  dispLR.convertTo(dispLR, CV_8U, 255/(96*16.));
  flip(imgL, imgL, 1);
  flip(imgR, imgR, 1);
  sgbm->compute(imgR, imgL, dispRL);
  dispRL.convertTo(dispRL, CV_8U, 255/(96*16.));
  flip(dispRL, dispRL, 1);
  cv::min(dispLR,dispRL,dispRLLR);
}

void Disparity::temporalDisparity() 
{
    temporalCombinedDisps.setTo(cv::Scalar(0));
    for (int j = 0; j < dispRLLR.cols; j++ ) {
        for (int i = 0; i < dispRLLR.rows; i++) {
            if(abs(dispRLLR.at<uchar>(i, j)-temporalDisp.at<uchar>(i, j)) <= 30) 
            {
                temporalCombinedDisps.at<uchar>(i, j) = dispRLLR.at<uchar>(i, j);
            }
        }
    }
}

void Disparity::vDispThresholdedImage(cv::Mat dispImg, float slope, float intersection, float thresholdOffset) 
{
// Create V disparity thresholded obstacleImage 
    obstacleImage.setTo(cv::Scalar(0));
    for (int j = 0; j < dispImg.cols; j++ ) {
        for (int i = 0; i < dispImg.rows; i++) {
            if (dispImg.at<uchar>(i, j) > (((i-(intersection))/(slope))+thresholdOffset))
            {  
                obstacleImage.at<uchar>(i, j) = dispImg.at<uchar>(i, j);
            } 
            if (dispImg.at<uchar>(i, j) > 220)
            {
                obstacleImage.at<uchar>(i, j) = 0;
            }
        }
    }
    //cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    //morphologyEx( obstacleImage, obstacleImage, cv::MORPH_CLOSE, sel, cv::Point(-1,-1), 1 );
}

void Disparity::removeDarkRegions(cv::Mat orgImgLoi, cv::Mat modifiedDispImage) 
{   
    postDarkImage=modifiedDispImage.clone();
    for (int j = 0; j < orgImgLoi.cols; j++ ) {
        for (int i = 0; i < orgImgLoi.rows; i++) {
            pixelValueBGR = orgImgLoi.at<cv::Vec3b>(i,j);    
            pixelValueBlue = pixelValueBGR.val[0];
            pixelValueGreen = pixelValueBGR.val[1];
            pixelValueRed = pixelValueBGR.val[2];

            if ( ((pixelValueBlue < 1) && (pixelValueGreen < 1) && (pixelValueRed < 1)) || ((pixelValueBlue > 254) && (pixelValueGreen > 254) && (pixelValueRed > 254)) )
            {   
               postDarkImage.at<uchar>(i, j) = 0;
            }
        }
    }
}

void Disparity::generateVdisp(cv::Mat dispImg) 
{
    //cv::Mat Vdisp = cv::Mat::zeros(dispImg.rows, 255, CV_8UC1);
    Vdisp.setTo(cv::Scalar(0));
    for (int j = 0; j < dispImg.cols; j++ ) 
    {
        for (int i = 0; i < dispImg.rows; i++) 
        {
            if(dispImg.at<uchar>(i, j) > 40 && dispImg.at<uchar>(i, j) < 225)
            {
                if(Vdisp.at<uchar>(i, dispImg.at<uchar>(i, j)) < 255 ){
                    Vdisp.at<uchar>(i, dispImg.at<uchar>(i, j))++;
                }
            } 
        }
    }
    //return Vdisp;
}

