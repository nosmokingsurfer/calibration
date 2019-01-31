#include <camera.h>

#include <pcl/common/common_headers.h>
#include <pcl/Modelcoefficients.h>

using namespace Eigen;
using namespace pcl;

Vector3f Camera::reprojectPtWithHeight(Vector2i pixel, float meterHeight) const
{
  float h = meterHeight;
  Matrix3f K = projection;
  Matrix<float, 3, 4> P = K * this->pose.getTransformation().matrix().topRows(3);
  Matrix3f A;
  for (int k = 0; k < 3; ++k)
  {
    A(k, 0) = P(k, 0);
    A(k, 1) = P(k, 1);
    A(k, 2) = h * P(k, 2) + P(k, 3);
  }

  Vector3f xyw = A.inverse() * Vector3f((float)pixel[0], (float)pixel[1], 1);
  if (xyw[2] == 0)
    return Vector3f();

  return Vector3f(xyw[0] / xyw[2], xyw[1] / xyw[2], h);
}

Vector3f Camera::reprojectPtWithDepth(Vector2i pixel, float meterDepth) const
{
  return this->pose.inverse()(Vector3f(
    (pixel[0] - projection(0, 2)) * meterDepth / projection(0, 0),
    (pixel[1] - projection(1, 2)) * meterDepth / projection(1, 1),
    meterDepth));
}

Vector3f Camera::reprojectPtWithDist(Vector2i pixel, float meterDist) const
{
  float ax = (pixel[0] - projection(0, 2)) / projection(0, 0);
  float ay = (pixel[1] - projection(1, 2)) / projection(1, 1);
  Vector3f dirVec;
  dirVec << ax, ay, 1;

  float z = meterDist / dirVec.norm();
  Vector3f preres = this->pose.inverse()(dirVec * z);
  return preres;
}


Camera::Camera(const Pose & pose_)
: pose(pose_)
, maxRange_(40.0)
{
  this->distortion << -1.1983283309789111e-01, 2.7076763925130121e-01, 0., 0., -7.3458604303021896e-02;

  this->projection << 140, 0., 70,
                      0., 140, 70, 
                      0.,  0., 1.;
  this->imageSize <<  140, 140;
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

  Vector4f plane_coeffs(plane.values.data());
  plane_coeffs = this->pose.inverse().getTransformation().matrix().transpose()*plane_coeffs;

  Vector3f n(plane_coeffs.head(3));

  float d = plane_coeffs[3];

  for(auto i = 0; i < this->imageSize[0]; i++)
  {
    for(auto j = 0; j < this->imageSize[1]; j++)
    {
      Vector3f tau;
      tau = this->reprojectPtWithDist(Vector2i(i, j), 1); //direction in camera pose
      tau = this->pose.inverse()(tau); //direction in global frame

      float t = -d/(tau.dot(n));

      if ((t <= maxRange_) && (t >= 0))
      {
        Vector3f r;

        r = t*tau;

        r = this->pose.inverse()(r);

        result->points.push_back(PointXYZ((float)r.x(), (float)r.y(), (float)r.z()));
      }
    }
  }


  return result;
}