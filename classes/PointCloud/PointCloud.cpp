#include "PointCloud.h"
PointCloud::PointCloud() 
{
  focalLenth = 1612.77; //1.25998*1280; // focal length in pixels
  baseLine = 0.239813; // baseline length in m
}

void PointCloud::dispToXYZRGB( cv::Mat disp, cv::Mat colorImage, Counter &counter )
{
  float            x, y, z; 
  int              r, g, b;
  //int              nPoints = 0;
  int              i, j, k;
  unsigned short   disparity;

  // The format for the output file is:
  // <x> <y> <z> <red> <grn> <blu>
  // <x> <y> <z> <red> <grn> <blu>
  // ...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_xyzRGBcloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyzCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);



  // Determine the number of pixels spacing per row
  for ( i = 0; i < disp.rows; i++ )
  {
      for ( j = 0; j < disp.cols; j++ )
      {
          unsigned short disparity = (unsigned short)disp.at<uchar>(i, j);

          // do not save invalid points
          if ( disparity>10 && disparity < 255 )
          {
              // convert the 16 bit disparity value to floating point x,y,z
              //triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

              float z = (focalLenth*baseLine)/(float)disparity;
              float x = (j*z)/focalLenth; //i = row
              float y = (i*z)/focalLenth; //j = col
              // look at points within a range
              b = (int)colorImage.at<cv::Vec3b>(i,j)[0];
              g = (int)colorImage.at<cv::Vec3b>(i,j)[1];
              r = (int)colorImage.at<cv::Vec3b>(i,j)[2];

              //nPoints++;

              pcl::PointXYZRGB pointRGB;
              pcl::PointXYZ point;
              point.x = x;
              point.y = y;
              point.z = z;
              pointRGB.x = x;
              pointRGB.y = y;
              pointRGB.z = z;
              uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
              pointRGB.rgb = *reinterpret_cast<float*>(&rgb);
              point_xyzRGBcloud_ptr->points.push_back(pointRGB);
              point_xyzCloudPtr->points.push_back(point);
          
          }
      }
  }

  point_xyzRGBcloud_ptr->width = (int) point_xyzRGBcloud_ptr->points.size();
  point_xyzRGBcloud_ptr->height = 1;
  point_xyzRGBcloud_ptr->is_dense = false;

  point_xyzCloudPtr->width = (int) point_xyzRGBcloud_ptr->points.size();
  point_xyzCloudPtr->height = 1;
  point_xyzCloudPtr->is_dense = false;

  //pcl::io::savePCDFileASCII("out/xyz_point_cloud.pcd",*point_xyzCloudPtr);
  //pcl::io::savePLYFileASCII("out/point_xyzRGBcloud_ptr.ply",*point_xyzRGBcloud_ptr);
 // std::cout << "PointCloud before filtering has: " << point_xyzCloudPtr->points.size () << " data points." << std::endl; //*

  int vIt = 0;

  // Filter objects futher than 10 meters away
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyzCloudPtr2to8 (new pcl::PointCloud<pcl::PointXYZ>);
  pass.setInputCloud (point_xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.0,9.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*point_xyzCloudPtr2to8);
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (point_xyzCloudPtr2to8);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*point_xyzCloudPtr2to8);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2to8 (new pcl::PointCloud<pcl::PointXYZ>);
  // Filter outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (point_xyzCloudPtr2to8);
  sor.setMeanK (200); //night
  //sor.setMeanK (300); //day
  sor.setStddevMulThresh (0.9); 
  sor.filter (*cloud_filtered2to8);

  //std::cout << "PointCloud after filtering has: " << cloud_filtered2to8->points.size ()  << " data points." << std::endl; //*

  //pcl::io::savePLYFileASCII("out/cloud_filtered2to8.ply",*cloud_filtered2to8);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered2to8);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.09); // 2cm
  ec.setMinClusterSize (500); //night
  //ec.setMinClusterSize (800); //day
  ec.setMaxClusterSize (20000); // Night
  //ec.setMaxClusterSize (100000); // day
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered2to8);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered2to8->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = false;
    // Find center points of clusters and remap to 2D image coord
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    // Object for retrieving the convex hull.
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_cluster);
    hull.reconstruct(*convexHull);
 
    int indexMinX=0, indexMaxX=0;
    int indexMinY=0;
    int indexMaxY=0;
    int indexMinZ=0;
    int indexMaxZ=0;

    // Find extreeme points in the convex hull point cloud
    for(int k=0;k<convexHull->width;k++) 
    { 
      if(convexHull->points[indexMaxX].x < convexHull->points[k].x)
      {
        indexMaxX = k; 
      }
      if(convexHull->points[indexMinX].x > convexHull->points[k].x)
      {
        indexMinX = k; 
      }
      if(convexHull->points[indexMaxY].y < convexHull->points[k].y)
      {
        indexMaxY = k; 
      }
      if(convexHull->points[indexMinY].y > convexHull->points[k].y)
      {
        indexMinY = k; 
      }
      if(convexHull->points[indexMaxZ].z < convexHull->points[k].z)
      {
        indexMaxZ = k; 
      }
      if(convexHull->points[indexMinZ].z > convexHull->points[k].z)
      {
        indexMinZ = k; 
      }
    }
   
    // Find center point of point cloud
    Eigen::Vector4f centroid4;
    compute3DCentroid(*cloud_cluster, centroid4);
    cv::Point3f clusterCenter3Dpoint;
    clusterCenter3Dpoint.x = centroid4(0);
    clusterCenter3Dpoint.y = centroid4(1);
    clusterCenter3Dpoint.z = centroid4(2);

   // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points. " << std::endl;

    /*std::stringstream ss;
    ss << "out/segmentedClusters/cloud_cluster_" << vIt << ".ply";
    pcl::io::savePLYFileASCII(ss.str (),*cloud_cluster); */
    
    Vehicle tmpVehicle;

    cv::Point3f cluster3DWidthHeightDepth;
    cluster3DWidthHeightDepth.x = (convexHull->points[indexMaxX].x - convexHull->points[indexMinX].x);
    cluster3DWidthHeightDepth.y = (convexHull->points[indexMaxY].y - convexHull->points[indexMinY].y);
    cluster3DWidthHeightDepth.z = (convexHull->points[indexMaxZ].z - convexHull->points[indexMinZ].z);

    cv::Point3f clusterFront3Dpoint;
    clusterFront3Dpoint.x = convexHull->points[indexMinZ].x;
    clusterFront3Dpoint.y = convexHull->points[indexMinY].y + cluster3DWidthHeightDepth.y/2;
    clusterFront3Dpoint.z = convexHull->points[indexMinZ].z;

    cv::Point3f clusterRight3Dpoint;
    clusterRight3Dpoint.x = convexHull->points[indexMaxX].x;
    clusterRight3Dpoint.y = convexHull->points[indexMinY].y + cluster3DWidthHeightDepth.y/2;
    clusterRight3Dpoint.z = convexHull->points[indexMaxX].z;

    cv::Point3f clusterLeft3Dpoint;
    clusterLeft3Dpoint.x = convexHull->points[indexMinX].x;
    clusterLeft3Dpoint.y = convexHull->points[indexMinY].y + cluster3DWidthHeightDepth.y/2;
    clusterLeft3Dpoint.z = convexHull->points[indexMinX].z;

    tmpVehicle.nearestPoint = projectFrom3Dto2D(clusterFront3Dpoint);
    tmpVehicle.rightPoint = projectFrom3Dto2D(clusterRight3Dpoint);
    tmpVehicle.leftPoint = projectFrom3Dto2D(clusterLeft3Dpoint);

    tmpVehicle.world3Dpositions.push_back(clusterCenter3Dpoint);
    tmpVehicle.widthHeightDepth.push_back(cluster3DWidthHeightDepth);
    tmpVehicle.image2Dpositions.push_back(projectFrom3Dto2D(clusterCenter3Dpoint));

    cv::Point3f upperLeftCorner3Dpoint;
    upperLeftCorner3Dpoint.x = convexHull->points[indexMinX].x;
    upperLeftCorner3Dpoint.y = convexHull->points[indexMaxY].y;
    upperLeftCorner3Dpoint.z = convexHull->points[indexMinZ].z+cluster3DWidthHeightDepth.z/2;

    cv::Point3f lowerRightCorner3Dpoint;
    lowerRightCorner3Dpoint.x = convexHull->points[indexMaxX].x;
    lowerRightCorner3Dpoint.y = convexHull->points[indexMinY].y;
    lowerRightCorner3Dpoint.z = convexHull->points[indexMinZ].z+cluster3DWidthHeightDepth.z/2;

    tmpVehicle.upperLeftCorner = projectFrom3Dto2D(upperLeftCorner3Dpoint);
    tmpVehicle.lowerRightCorner = projectFrom3Dto2D(lowerRightCorner3Dpoint);

    counter.vehicles.push_back(tmpVehicle);

    vIt++;
  }
  std::cout << "Number of clusters found: " << vIt  << std::endl;
}

cv::Point3f PointCloud::projectFrom3Dto2D(cv::Point3f world3Dcoordinate)
{
    cv::Point3f world2Dcoordiante;
    world2Dcoordiante.x = (world3Dcoordinate.x*focalLenth)/world3Dcoordinate.z;
    world2Dcoordiante.y = (world3Dcoordinate.y*focalLenth)/world3Dcoordinate.z;
    world2Dcoordiante.z = (world3Dcoordinate.x*focalLenth)/world2Dcoordiante.x;
    return world2Dcoordiante;
}



