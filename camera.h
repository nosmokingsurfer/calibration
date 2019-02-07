///
/// @file lidar.h
///@authors Panchenko A.V.
///@brief Camera class - OOP class to work with camera mathematical model
///@brief Contains device pose and generative method for depthcam-plane system.
///

#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include <pose.h>
#include <random>

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>

using namespace Eigen;


class Camera
{
private:
  Pose axisRemapping{Vector3f(0,-EIGEN_PI/2,0),Vector3f(0,0,0)};


public:
  Camera::Camera();

  Camera::Camera(const Pose& pose);

  //! @brief camera model similar to Opencv https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  //! @param [in] K - projection matrix
  //! @param [in] D - distorsion coefficients (OpenCV)
  //! @param [in] pose - camera pose in global frame
  //! @param [in] sz - image size in pixels
  Camera::Camera(const Matrix3f &K, const Matrix<float, 5, 1> &D, const Pose& pose, const Vector2i &sz);

  // Projects a 3d point onto undistorted camera image Point
  //! @param[in] pt - 3d point in global frame
  //! @retval Vector2i - resulting pixel coordinates
  Eigen::Vector2i projectPt(Eigen::Vector3f pt) const;

  // reprojects a point pixel on image to 3d relative to this->pose CS, with an
  // assumption of the point eukledian distance from camera origin to 3D point is meterDist. 
  //! @brief Reconstructs 3d point for give (i,j) pixel and given meter dist
  //! @param[in] pixel - pixel to reproject
  //! @param[in] meterDist range in meters
  //! @retval Vector3f 3d point in global frame. 
  Eigen::Vector3f reprojectPtWithDist(Eigen::Vector2i pixel, float meterDist) const;

public:
  //camera projection parameters
  Eigen::Matrix3f projection;
  Eigen::Matrix<float, 5, 1> distortion;
  Eigen::Vector2i imageSize;
  Pose pose;

  float maxRange_;
  float minRange_;


  float rangeErrorMean;
  float rangeErrorSigma;

  std::normal_distribution<float> range_error_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr getRawDepthData(const pcl::ModelCoefficients& plane);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getProjectionModel() const;
};

#endif //