#include <lidar.h>

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>


using namespace Eigen;
using namespace std;
using namespace pcl;


Lidar::Lidar():
scaleErrorMean(1.0),
scaleErrorSigma(std::numeric_limits<float>::min()),
rangeErrorMean(0.0),
rangeErrorSigma(0.03)
{
  this->scale_error_ = std::normal_distribution<float>(scaleErrorMean, scaleErrorSigma);
  this->range_error_ = std::normal_distribution<float>(rangeErrorMean, rangeErrorSigma);
}

Lidar::~Lidar()
{}

pcl::PointCloud<pcl::PointXYZ> Lidar::getRawLidarData(const pcl::ModelCoefficients& plane)
{
  PointCloud<PointXYZ> result;

  std::default_random_engine generator;

  //ground plane parameters
  Vector3f n;
  n << plane.values[0], plane.values[1], plane.values[2];

  //TODO transform plane to lidar frame

  float d = plane.values[3];

  Vector3f r_0 = this->pose.inverse().getTranslation();

  for (auto i = 0; i < 900; i++)
  {
    float azimuth = i*azimuth_delta_;
    for (auto j = 0; j < 16; j++)
    {
      float elevation = -15 * EIGEN_PI / 180.0 + j*elevation_delta_;

      Vector3f tau;
      tau << cos(azimuth)*cos(elevation), cos(elevation)*sin(azimuth), sin(elevation);

      float t = -(r_0.dot(n) + d) / (tau.dot(n));

      t = t*scale_error_(generator) + range_error_(generator);

      if ((tau.dot(n) < 0) && t <= maxRange_)
      {
        Vector3f r;
        r = r_0 + t*tau; // point in global frame

        r = this->pose(r); // the same point in lidar frame

        result.points.push_back(PointXYZ((float)r.x(), (float)r.y(), (float)r.z()));
      }
    }
  }


  return result;
}


