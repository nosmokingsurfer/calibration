#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pose/pose.h>

#include <random>
#include <iostream>

using namespace pcl;




PointCloud<PointXYZ>::Ptr getRawLidarData(const ModelCoefficients& plane, const Pose& lidarPose)
{
  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());

  std::default_random_engine generator;

  std::normal_distribution<float> scale_error(1.0, 0.00000000001);
  std::normal_distribution<float> range_eror(0, 0.1);
  
  //ground plane parameters
  Vector3f n;
  n << plane.values[0], plane.values[1], plane.values[2];

  float d = plane.values[3];
  
  Vector3f r_0 = lidarPose.T.inverse().translation();


  //vlp16 like lidar model
  float maxRange = 100.0;
  float azimuth_delta = 0.25*EIGEN_PI/180.0;
  float elevation_delta = 2.0*EIGEN_PI/180.0;

  for(auto i = 0; i < 1440; i++)
  {
    float azimuth = i*azimuth_delta;
    for(auto j = 0; j < 16; j++)
    {
      float elevation = -15*EIGEN_PI/180.0 + j*elevation_delta;

      Vector3f tau;
      tau << cos(azimuth)*cos(elevation), cos(elevation)*sin(azimuth), sin(elevation);

      float t = -(r_0.dot(n) + d)/(tau.dot(n));

      t = t*scale_error(generator) + range_eror(generator);

      if((tau.dot(n) < 0) && t <= maxRange)
      {
        Vector3f r;
        r = r_0 + t*tau; // point in global frame

        r = lidarPose.T*r; // the same point in lidar frame

        result->points.push_back(PointXYZ((float)r.x(), (float)r.y(), (float)r.z()));
      }
    }
  }
  
 
  return result;
}





int main(int argc, char** argv) 
{
  ModelCoefficients plane;
  plane.values.push_back(0);
  plane.values.push_back(0);
  plane.values.push_back(1);
  plane.values.push_back(0);
  
  Vector3f gt_angles;
  gt_angles << 0,0,0;

  Vector3f gt_translation;
  gt_translation << 0,0,1;

  Pose lidarPose(gt_angles, gt_translation);

  PointCloud<PointXYZ>::Ptr raw_data = getRawLidarData(plane, lidarPose);

  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("lidar viewer"));

  visualization::PointCloudColorHandlerCustom<PointXYZ> raw_rgb(raw_data, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(raw_data, raw_rgb, "raw_data");
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 4, "raw_data");

  viewer->addCoordinateSystem(2, lidarPose.T.inverse(), "lidar");
  

  //estimate plane model
  ModelCoefficients::Ptr plane_coeffs(new ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);
  SACSegmentation<PointXYZ> seg;

  seg.setInputCloud(raw_data);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(0.01);
  seg.setOptimizeCoefficients(true);
  seg.segment(*inliers, *plane_coeffs);

  //extract points
  PointCloud<PointXYZ>::Ptr plane_points(new PointCloud<PointXYZ>);
  ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(raw_data);
  extract.setIndices(inliers);
  extract.filter(*plane_points);

  visualization::PointCloudColorHandlerCustom<PointXYZ> plane_rgb(plane_points, 255,0,0);
  viewer->addPointCloud<PointXYZ>(plane_points,"plane_points");

  viewer->addPlane(*plane_coeffs, "plane");

  std::cout << "Estimated plane coeffs:" << std::endl;
  for(auto p : plane_coeffs->values)
  {
    std::cout << p << std::endl;
  }

  float lidarHeight = plane_coeffs->values[3];

  std::cout << "Lidar height = " << lidarHeight << std::endl;




  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

  return -1;
}