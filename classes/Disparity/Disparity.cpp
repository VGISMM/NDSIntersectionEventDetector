#include "Disparity.h"
Disparity::Disparity() 
{
    sgbm.preFilterCap = 1;
    sgbm.SADWindowSize = 9;
    sgbm.P1 = 8*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = 64;
    //sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 2;
    sgbm.speckleRange = 3;
    //sgbm.disp12MaxDiff = 1;
    //sgbm.fullDP = 1;
}

void Disparity::disparityImages(cv::Mat imgL, cv::Mat imgR) 
{
    sgbm(imgL, imgR, dispLR);
    dispLR.convertTo(dispLR, CV_8U, 255/(sgbm.numberOfDisparities*16.));
    flip(imgL, imgL, 1);
    flip(imgR, imgR, 1);
    sgbm(imgR, imgL, dispRL);
    dispRL.convertTo(dispRL, CV_8U, 255/(sgbm.numberOfDisparities*16.));
    flip(dispRL, dispRL, 1);
}

void Disparity::combinedRLLRDisparity(bool first) 
{
    if(!first)
    {
        temporalDisp = combinedDisps.clone();
    }
    combinedDisps.setTo(cv::Scalar(0));
   
    //cv::cv::Mat combinedDisps=cv::cv::Mat::zeros(disp.rows,disp.cols,disp.type());
    for (int j = 0; j < dispLR.cols; j++ ) {
        for (int i = 0; i < dispLR.rows; i++) {
            if(dispLR.at<uchar>(i, j) <= dispRL.at<uchar>(i, j)) 
            {
                combinedDisps.at<uchar>(i, j) = dispLR.at<uchar>(i, j);
            }
            if(dispLR.at<uchar>(i, j) > dispRL.at<uchar>(i, j)) 
            {
                combinedDisps.at<uchar>(i, j) = dispRL.at<uchar>(i, j);
            }
        }
    }
}

void Disparity::temporalDisparity() 
{
    temporalCombinedDisps.setTo(cv::Scalar(0));
    //cv::Mat temporalCombinedDisps=cv::Mat::zeros(combinedDisps.rows,combinedDisps.cols,combinedDisps.type());
    for (int j = 0; j < combinedDisps.cols; j++ ) {
        for (int i = 0; i < combinedDisps.rows; i++) {
            if(abs(combinedDisps.at<uchar>(i, j)-temporalDisp.at<uchar>(i, j)) <= 20) 
            {
                temporalCombinedDisps.at<uchar>(i, j) = combinedDisps.at<uchar>(i, j);
            }
        }
    }
}

void Disparity::vDispThresholdedImage(float slope, float intersection, float thresholdOffset) 
{
// Create V disparity thresholded obstacleImage 
    obstacleImage.setTo(cv::Scalar(0));
    for (int j = 0; j < postDarkImage.cols; j++ ) {
        for (int i = 0; i < postDarkImage.rows; i++) {
            if (postDarkImage.at<uchar>(i, j) > (((i-(intersection))/(slope))+thresholdOffset))
            {  
                obstacleImage.at<uchar>(i, j) = postDarkImage.at<uchar>(i, j);
            } 
            if (postDarkImage.at<uchar>(i, j) > 220)
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
                //postDarkImage.at<uchar>(i, j) = 0;
            }
        }
    }
}
/*
void Disparity::reduceNumberOfBins(cv::Mat dispImg) 
{   
    reducedImg.setTo(cv::Scalar(0));
    //cv::Mat reducedImg = cv::Mat::zeros(dispImg.rows, dispImg.cols, CV_8UC1);
    for (int j = 0; j < dispImg.cols; j++ ) 
    {
        for (int i = 0; i < dispImg.rows; i++) 
        {
            reducedImg.at<uchar>(i, j) = dispImg.at<uchar>(i, j)/8;
        }
    }
    //return reducedImg;
}
*/

void Disparity::generateVdisp() 
{
    //cv::Mat Vdisp = cv::Mat::zeros(dispImg.rows, 255, CV_8UC1);
    Vdisp.setTo(cv::Scalar(0));
    for (int j = 0; j < postDarkImage.cols; j++ ) 
    {
        for (int i = 0; i < postDarkImage.rows; i++) 
        {
            if(postDarkImage.at<uchar>(i, j) > 40 && postDarkImage.at<uchar>(i, j) < 225)
            {
                if(Vdisp.at<uchar>(i, postDarkImage.at<uchar>(i, j)) < 255 ){
                    Vdisp.at<uchar>(i, postDarkImage.at<uchar>(i, j))++;
                }
            } 
        }
    }
    //return Vdisp;
}

void Disparity::generateUdisp(cv::Mat dispImg) 
{
    Udisp.setTo(cv::Scalar(0));
    //cv::Mat Udisp = cv::Mat::zeros(255, dispImg.cols, CV_8UC1);
    for (int j = 0; j < dispImg.cols; j++ ) 
    {
        for (int i = 0; i < dispImg.rows; i++) 
        {
            if(dispImg.at<uchar>(i, j) > 10 && dispImg.at<uchar>(i, j) < 235)
            {
                if(Udisp.at<uchar>(dispImg.at<uchar>(i, j), j) < 255 ){
                    Udisp.at<uchar>(dispImg.at<uchar>(i, j), j)++;
                }
            } 
        }
    }
}

/*
void Disparity::generateVdispPlanes(cv::Mat dispImg) 
{   
    VdispPlanes.setTo(cv::Scalar(0));
    //cv::Mat VdispPlanes = cv::Mat::zeros(dispImg.rows, dispImg.cols, CV_8UC1);
    for (int VdispRow = 0; VdispRow < Vdisp.rows; VdispRow++) 
    {
        for (int VdispCol = 0; VdispCol < Vdisp.cols; VdispCol++ ) 
        {
            if(Vdisp.at<uchar>(VdispRow, VdispCol) > (4*((VdispCol+1)/2)))
            //if(Vdisp.at<uchar>(VdispRow-1, VdispCol) + Vdisp.at<uchar>(VdispRow, VdispCol) + Vdisp.at<uchar>(VdispRow+1, VdispCol) > 60)
            {
                for (int dispImgCol = 0; dispImgCol < dispImg.cols; dispImgCol++ ) 
                {
                    if(dispImg.at<uchar>(VdispRow, dispImgCol) == VdispCol)
                    {
                        VdispPlanes.at<uchar>(VdispRow, dispImgCol) = VdispCol;
                    }
                }
            } 
        }
    }
    //return VdispPlanes;
}

void Disparity::generateUdispPlanes(cv::Mat dispImg) 
{
    UdispPlanes.setTo(cv::Scalar(0));
    //cv::Mat UdispPlanes = cv::Mat::zeros(dispImg.rows, dispImg.cols, CV_8UC1);
    for (int UdispRow = 0; UdispRow < Udisp.rows; UdispRow++) 
    {
        for (int UdispCol = 0; UdispCol < Udisp.cols; UdispCol++ ) 
        {
            if(Udisp.at<uchar>(UdispRow, UdispCol) > (1*((UdispRow+1)/2)))
            {
                for (int dispImgRow = 0; dispImgRow < dispImg.rows; dispImgRow++ ) 
                {
                    if(dispImg.at<uchar>(dispImgRow, UdispCol) == UdispRow)
                    {
                        UdispPlanes.at<uchar>(dispImgRow, UdispCol) = UdispRow;
                    }
                }
            } 
        }
    }
    //return UdispPlanes;
}
*/

