#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>

#include <pose/pose.h>

using namespace pcl;


PointCloud<PointXYZ>::Ptr getRawLidarData(const ModelCoefficients& plane, const Pose& lidarPose)
{
  Vector3f n;
  n << plane.values[0], plane.values[1], plane.values[2];

  float d = plane.values[3];
  
  Vector3f r_0 = lidarPose.T.translation();

  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());
  
  float maxRange = 100.0;
  float azimuth_delta = 0.1*EIGEN_PI/180.0;
  float elevation_delta = 2.0*EIGEN_PI/180.0;

  for(auto i = 0; i < 3600; i++)
  {
    float azimuth = i*azimuth_delta;
    for(auto j = 0; j < 16; j++)
    {
      float elevation = -15*EIGEN_PI/180.0 + j*elevation_delta;

      Vector3f tau;
      tau << cos(azimuth)*cos(elevation), cos(elevation)*sin(azimuth), sin(elevation);

      float t = -(r_0.dot(n) + d)/(tau.dot(n));

      if((tau.dot(n) < 0) && t <= maxRange)
      {
        Vector3f r;
        r = r_0 + t*tau;

        result->points.push_back(PointXYZ((float)r.x(), (float)r.y(), (float)r.z()));
      }

    }
  }
  
  
  result->header.frame_id="raw_data";



  return result;
}





int main(int argc, char** argv) 
{
  ModelCoefficients plane;
  plane.values.push_back(0);
  plane.values.push_back(0);
  plane.values.push_back(1);
  plane.values.push_back(0);
  
  Vector3f angles;
  angles << 0,0,0;

  Vector3f translation;
  translation << 0,0,1;

  Pose lidarPose(angles, translation);

  PointCloud<PointXYZ>::Ptr raw_data = getRawLidarData(plane, lidarPose);

  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("lidar viewer"));

  viewer->addPointCloud<pcl::PointXYZ>(raw_data, "raw_data");
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_data");

  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

  return -1;
}