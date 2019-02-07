#include <lidar.h>

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>


using namespace Eigen;
using namespace std;
using namespace pcl;


Lidar::Lidar()
: rangeErrorMean_(0.0)
, rangeErrorSigma_(0.03)
, minRange_(0.5)
, maxRange_(100.0)
{
  this->range_error_ = std::normal_distribution<float>(rangeErrorMean_, rangeErrorSigma_);
}


Lidar::Lidar(const Pose& pose)
{
  (*this) = Lidar();

  this->pose_ = pose;
}

Lidar::~Lidar()
{}

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::getRawLidarData(const pcl::ModelCoefficients& plane)
{
  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());

  std::default_random_engine generator;

  Vector4f plane_coeffs(plane.values.data());

  //plane model in sensor related frame
  plane_coeffs = this->pose_.inv().getTransformation().matrix().transpose()*plane_coeffs; 

  Vector3f n(plane_coeffs.head(3));

  float d = plane_coeffs[3];

  for (auto i = 0; i < 900; i++)
  {
    float azimuth = i*azimuth_delta_;
    for (auto j = 0; j < 16; j++)
    {
      float elevation = -15 * EIGEN_PI / 180.0 + j*elevation_delta_;

      Vector3f tau;
      tau << cos(azimuth)*cos(elevation), cos(elevation)*sin(azimuth), sin(elevation); //laser beam vector


      float t = - d / (tau.dot(n)); //intersection with plane where t is parameter in equation r = t*tau

      t += this->range_error_(generator); //adding range error

      if ((tau.dot(n) < 0) && (t <= maxRange_) && (t >= minRange_))
      {
        Vector3f r;
        r = t*tau; // point on plane in lidar frame

        result->points.push_back(PointXYZ(r.x(), r.y(), r.z()));
      }
    }
  }

  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::getProjectionModel() const
{
  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());

  for (auto i = 0; i < 900; i++)
  {
    float azimuth = i*azimuth_delta_;
    for (auto j = 0; j < 16; j++)
    {
      float elevation = -15 * EIGEN_PI / 180.0 + j*elevation_delta_;

      Vector3f tau;
      tau << cos(azimuth)*cos(elevation), cos(elevation)*sin(azimuth), sin(elevation); //laser beam vector

      result->points.push_back(PointXYZ(tau.x(),
                                        tau.y(),
                                        tau.z()));
    }
  }

  return result;
}


