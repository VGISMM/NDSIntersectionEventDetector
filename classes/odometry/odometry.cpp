#include "odometry.h"

Odometry::Odometry(){
  pose = Matrix::eye(4);
  cumulativePose = Matrix::eye(4);

}


void Odometry::egoOdometry(cv::Mat imageLeft, cv::Mat imageRight, VisualOdometryStereo *viso){
    
    //pose = Matrix::eye(4);
    int channels = imageLeft.channels();
    int nRows = imageLeft.rows;
    int nCols = imageLeft.cols * channels;
    int32_t dims[] = {nCols,nRows,nCols};

    int32_t k=0;
  
    uint8_t* left_img_data = (uint8_t*)malloc(nCols*nRows*sizeof(uint8_t));
    uint8_t* right_img_data = (uint8_t*)malloc(nCols*nRows*sizeof(uint8_t));

    if (imageLeft.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    uchar* L;
    uchar* R;
    for( i = 0; i < nRows; ++i)
    {
        L = imageLeft.ptr<uchar>(i);
        R = imageRight.ptr<uchar>(i);
        for ( j = 0; j < nCols; ++j)
        {
            left_img_data[k] = (uint8_t)L[j];
            right_img_data[k] = (uint8_t)R[j];
            k++;
        }
    }

    // compute visual odometry
    if (viso->process(left_img_data,right_img_data,dims)) {
      pose = Matrix::inv(viso->getMotion());
      cumulativePose = cumulativePose * pose;
      
      // output some statistics
      double num_matches = viso->getNumberOfMatches();
      double num_inliers = viso->getNumberOfInliers();
      cout << ", Matches: " << num_matches;
      cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
      cout << cumulativePose << endl << endl;

    } else {
      //pose = Matrix::eye(4);
      cout << " ... failed! on iter: 1 " << endl;
    }
}