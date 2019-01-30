#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pose.h>
#include <camera.h>
#include <lidar.h>

#include <iostream>

using namespace pcl;


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

  
  Lidar lidar;

  lidar.pose = Pose(gt_angles, gt_translation);

  PointCloud<PointXYZ> raw_data = lidar.getRawLidarData(plane);

  Camera cam(Pose(gt_angles, gt_translation));


  //raw_data = cam.getRawDepthData(plane);

  PointCloud<PointXYZ>::Ptr raw_data_ptr = boost::make_shared<PointCloud<PointXYZ>>(raw_data);

  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("lidar viewer"));

  visualization::PointCloudColorHandlerCustom<PointXYZ> raw_rgb(raw_data_ptr, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(raw_data_ptr, raw_rgb, "raw_data");
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 4, "raw_data");

  viewer->addCoordinateSystem(2, lidar.pose.inverse().getTransformation(), "lidar");
  

  //estimate plane model
  ModelCoefficients::Ptr plane_coeffs(new ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);
  SACSegmentation<PointXYZ> seg;

  seg.setInputCloud(raw_data_ptr);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(0.01);
  seg.setOptimizeCoefficients(true);
  seg.segment(*inliers, *plane_coeffs);

  //extract points
  PointCloud<PointXYZ>::Ptr plane_points(new PointCloud<PointXYZ>);
  ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(raw_data_ptr);
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