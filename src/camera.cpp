#include <camera.h>

#include <pcl/common/common_headers.h>
#include <pcl/Modelcoefficients.h>

using namespace Eigen;
using namespace pcl;

Vector3f Camera::reprojectPtWithDist(Vector2i pixel, float meterDist) const
{
  float ax = (pixel[0] - projection(0, 2)) / projection(0, 0);
  float ay = (pixel[1] - projection(1, 2)) / projection(1, 1);
  Vector3f dirVec;
  dirVec << ax, ay, 1;

  float z = meterDist / dirVec.norm();
  Vector3f preres = dirVec * z;
  return preres;
}

Camera::Camera()
  : pose(Pose())
  , rangeErrorMean(0.0)
  , rangeErrorSigma(0.03)
  , maxRange_(40.0)
  , minRange_(1.0)
{
  this->range_error_ = std::normal_distribution<float>(rangeErrorMean, rangeErrorSigma);

  this->distortion << -1.1983283309789111e-01, 2.7076763925130121e-01, 0., 0., -7.3458604303021896e-02;

  this->projection << 140, 0., 70,
                       0., 140, 70,
                       0., 0., 1.;

  this->imageSize << 140, 140;
}



Camera::Camera(const Pose & pose_)
{
  (*this) = Camera();
  this->pose = pose_;
}

Camera::Camera(const Matrix3f &K, const Matrix<float,5,1> &D, const Pose& pose, const Vector2i &sz)
  : projection(K)
  , distortion(D)
  , imageSize(sz)
  , pose(pose)
  , maxRange_(40.0)
{
}

Vector2i Camera::projectPt(Vector3f pt) const
{
  Vector3f result = projection * pt;
  if (result[2])
    return Vector2i(result[0] / result[2], result[1] / result[2]);
  return Vector2i();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Camera::getRawDepthData(const pcl::ModelCoefficients& plane)
{
  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());

  std::default_random_engine generator;

  Vector4f plane_coeffs(plane.values.data());

  plane_coeffs = this->pose.inv().getTransformation().matrix().transpose()*(plane_coeffs);

  Vector3f n(plane_coeffs.head(3));

  float d = plane_coeffs[3];

  for(auto i = 0; i < this->imageSize[0]; i++)
  {
    for(auto j = 0; j < this->imageSize[1]; j++)
    {
      Vector3f tau;
      tau = this->reprojectPtWithDist(Vector2i(i, j), 1); //direction in image frame - Z axis is along optical axis of camera
      tau.normalize();
      tau = this->axisRemapping.inv().getRotation()*tau; // direction in sensor related frame
      
      float t = -d/(tau.dot(n));

      t += this->range_error_(generator); //adding range error

      if ((t <= maxRange_) && (t >= 0))
      {
        Vector3f r;

        r = t*tau;

        result->points.push_back(PointXYZ(r.x(), r.y(), r.z()));
      }
    }
  }

  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Camera::getProjectionModel() const
{
  PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>());

  for (auto i = 0; i < this->imageSize[0]; i++)
  {
    for (auto j = 0; j < this->imageSize[1]; j++)
    {
      Vector3f tau;
      tau = this->reprojectPtWithDist(Vector2i(i, j), 1); //direction in image frame - Z axis is along optical axis of camera
      tau = this->axisRemapping.inv().getRotation()*tau; // direction in sensor related frame

      result->points.push_back(PointXYZ(tau.x(), 
                                        tau.y(), 
                                        tau.z()));
    }
  }

  return result;
}