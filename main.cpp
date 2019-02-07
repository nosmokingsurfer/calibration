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

//returns oriented distance from point to given plane
float getDist(const Pose& pose, const ModelCoefficients& plane)
{
  float result = 0;

  Vector3f n(plane.values.data());
  float d = plane.values[3];

  Vector3f point = pose.inv().getTranslation();

  result = (point.dot(n) + d)/n.norm();

  return result;
}


int main(int argc, char** argv) 
{
  ModelCoefficients gt_plane; // plane coeffs in global frame
  gt_plane.values.push_back(0);
  gt_plane.values.push_back(0);
  gt_plane.values.push_back(1);
  gt_plane.values.push_back(0);

  Vector3f gt_angles; //device pose angles in global frame
  gt_angles << 0, 0, 0;
  Vector3f gt_translation; //device pose translation in global frame
  gt_translation << 0, 0, 0;
  
  Pose gt_pose;

  Camera cam;
  Lidar lidar;

  PointCloud<PointXYZ>::Ptr raw_data; //pointer for raw data

  if (argc != 2)
  {
    std::cout << "Enter flags as follows:" << std::endl <<
      "\t-c for depth camera test" << std::endl <<
      "\t-l for lidar test" << std::endl;
      return 0;
  }
  else if (argc == 2)
  {
  
    std::cout << "Enter device z coordianete above horizontal plane:" << std::endl;
    std::cin >> gt_translation[2];

    gt_pose = Pose(gt_angles, gt_translation);

    if(std::string(argv[1]) == "-c")
    {
      cam = Camera(gt_pose);
      raw_data = cam.getRawDepthData(gt_plane);
    }
    else if (std::string(argv[1]) == "-l")
    {
      lidar = Lidar(gt_pose);
      raw_data = lidar.getRawLidarData(gt_plane);
    }
  }  

  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("PointCloud viewer"));
  viewer->addCoordinateSystem(0.5, 0, 0, 0, "GF_origin");

  visualization::PointCloudColorHandlerCustom<PointXYZ> raw_rgb(raw_data, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(raw_data, raw_rgb, "raw_data");

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

  if(std::string(argv[1]) == "-c")
  {
    if(plane_coeffs->values.size() == 4)
    { 
      float dist = getDist(Vector3f(), *plane_coeffs);
      viewer->addText("Estimated sensor height:" + boost::lexical_cast<std::string>(dist), 50, 50,20, 1, 1, 1);
    }
    else
    {
      viewer->addText("Estimated sensor height: FAILED", 50, 50);
    }

    pcl::PointCloud<PointXYZ>::Ptr projectionPoints = cam.getProjectionModel();

    visualization::PointCloudColorHandlerCustom<PointXYZ> projection_rgb(projectionPoints, 255, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(projectionPoints, projection_rgb, "projection_data");
  }
  else if(std::string(argv[1]) == "-l")
  {
    if(plane_coeffs->values.size() == 4)
    { 
      float dist = getDist(Vector3f(), *plane_coeffs);
      viewer->addText("Estimated sensor height:" + boost::lexical_cast<std::string>(dist),50, 50, 20, 1,1, 1);
    }
    else
    {
      viewer->addText("Estimated sensor height: FAILED", 50, 50);
    }


    pcl::PointCloud<PointXYZ>::Ptr projectionPoints = lidar.getProjectionModel();
    visualization::PointCloudColorHandlerCustom<PointXYZ> projection_rgb(projectionPoints, 255, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(projectionPoints, projection_rgb, "projection_data");
  }


  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

  return -1;
}