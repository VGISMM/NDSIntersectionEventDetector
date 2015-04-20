#include "odometry.h"

Odometry::Odometry(){
  param.calib.f  = 721.24; // focal length in pixels
  param.calib.cu = 644.93824; // principal point (u-coordinate) in pixels
  param.calib.cv  = 247.6805; // principal point (v-coordinate) in pixels
  param.base  = 0.239813;
  pose = Matrix::eye(4);
}


Matrix Odometry::egoOdometry(cv::Mat imageLeft, cv::Mat imageLeftOld, cv::Mat imageRight, cv::Mat imageRightOld){
    VisualOdometryStereo viso(param);

    int width = imageLeft.cols;
    int height = imageLeft.rows;
    int32_t dims[] = {width,height,width};

     // copy input images to uint8_t buffer
    uint8_t* left_img_dataOld = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    uint8_t* right_img_dataOld = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    int32_t k=0;
    for (int32_t v=0; v<width; v++) {
      for (int32_t u=0; u<height; u++) {
        left_img_dataOld[k]  = imageLeft.at<uchar>(u,v);
        right_img_dataOld[k] = imageRight.at<uchar>(u,v);
        k++;
      }
    }
    // compute visual odometry
    if (viso.process(left_img_dataOld,right_img_dataOld,dims)) {
      pose = pose * Matrix::inv(viso.getMotion());
      /*
      // output some statistics
      double num_matches = viso.getNumberOfMatches();
      double num_inliers = viso.getNumberOfInliers();
      cout << ", Matches: " << num_matches;
      cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
      cout << pose << endl << endl;
      */

    } else {
      //pose = Matrix::eye(4);
      //cout << " ... failed! on iter: 1 " << endl;
    }

    free(left_img_dataOld);
    free(right_img_dataOld);

    // convert input images to uint8_t buffer
    uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    k=0;
    for (int32_t v=0; v<width; v++) {
      for (int32_t u=0; u<height; u++) {
        left_img_data[k]  = imageLeft.at<uchar>(u,v);
        right_img_data[k] = imageRight.at<uchar>(u,v);
        k++;
      }
    }
    // compute visual odometry 
    if (viso.process(left_img_data,right_img_data,dims)) {
    
      // on success, update current pose
     // cout << " ... success! on iter: " << i << endl;
      pose = pose * Matrix::inv(viso.getMotion());
      /*
      // output some statistics
      double num_matches = viso.getNumberOfMatches();
      double num_inliers = viso.getNumberOfInliers();
      cout << ", Matches: " << num_matches;
      cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
      cout << pose << endl << endl;
      */
    } else {
      //pose = Matrix::eye(4);
      cout << " ... failed! on iter: 2 " << endl;
    }
    //cout << "her" <<endl;
    // release uint8_t buffers
    free(left_img_data);
    free(right_img_data);

  return pose;
}