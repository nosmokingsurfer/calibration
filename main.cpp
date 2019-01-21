#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pose/pose.h>

using namespace pcl;


PointCloud<PointXYZ>::Ptr getRawLidarData(const Pose& lidarPose)
{
  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());

  return result;
}





int main(int argc, char** argv) 
{
  PointCloud<PointXYZ>::Ptr raw_data = getRawLidarData(Pose());

  return -1;
}