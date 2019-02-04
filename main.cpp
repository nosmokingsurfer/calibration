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
  ModelCoefficients plane; // plane coeffs in global frame
  plane.values.push_back(0);
  plane.values.push_back(0);
  plane.values.push_back(1);
  plane.values.push_back(0);
  
  Vector3f gt_angles; // lidar pose angles in global frame
  gt_angles << 0, -EIGEN_PI/6., EIGEN_PI/10;
  Vector3f gt_translation; // lidar pose translation in global frame
  gt_translation << 0, 0, 3;

  
  Lidar lidar;

  lidar.pose = Pose(gt_angles, gt_translation);

  PointCloud<PointXYZ>::Ptr raw_data = lidar.getRawLidarData(plane); // generating lidar data in global frame

  Camera cam(lidar.pose);
  
  
  //raw_data = cam.getRawDepthData(plane);

  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("lidar viewer"));
  viewer->addCoordinateSystem(0.5, 0, 0, 0, "GF_origin");

  visualization::PointCloudColorHandlerCustom<PointXYZ> raw_rgb(raw_data, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(raw_data, raw_rgb, "raw_data");
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 4, "raw_data");

  viewer->addCoordinateSystem(2, cam.pose.inv().getTransformation(), "lidar");
  

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

  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

  return -1;
}