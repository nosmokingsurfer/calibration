///
/// @file lidar.h
///@authors Panchenko A.V.
///@brief Lidar class - OOP class to work with lidar mathematical model of Velodyne like lidars
///@brief Contains device pose and generative method for lidar-plane system.
///@brief Scale and range errors are similar to VLP16 lidar
///

#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include <pose.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>

using namespace Eigen;


class Camera
{
public:

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

  // reprojects a point on image to 3d relative to this->pose CS, with an
  // assumption of the point height is meterheight
  //! @brief ������������ ����� � �������� �������
  //! @param[in] pixel ���������� �������, ���������������� ������ �����
  //! @param[in] meterHeight ������ ������� � ������ (������������ pose)
  //! @param[in] usePrecalc ������������ ��� ��� ������������������ ���������� Ainv ��� �������� �������������� ����� �����
  //! @retval cv::Point3d ���������� ����� �� ������ meterHeight, ���������� � pixel ��� �������� ������ �������
  //! ������� ��������������� �����, ��������������� ������� �������, ��� �������������, ��� ������ 3d ����� ����
  //! meterHeight. ��������������� ����� ����� ���������� � �.�. exPose.inv()
  Eigen::Vector3f reprojectPtWithHeight(Eigen::Vector2i pixel, float meterHeight = 0) const;

  // reprojects a point pixel on image to 3d relative to this->pose CS, with an
  // assumption of the point DEPTH is meterDepth. 
  // Depth here means a value of Z coordinate in camera CS (exPose)
  //! @brief ������������ ����� � �������� ��������
  //! @param[in] pixel ���������� �������, ���������������� ������ �����
  //! @param[in] meterDepth ������� ������� � ������
  //! @retval cv::Point3d ���������� ����� �� ������� meterDepth, ���������� � pixel ��� �������� ������ �������.
  //! ������� ��������������� �����, ��������������� ������� �������, ��� �������������, ���
  //! ������� ����� - meterDepth. ��� �������� ����� ���������� z-���������� ����� 
  //! � �.�. exPose. ��������������� ����� ����� ���������� � �.�. exPose.inv()
  Eigen::Vector3f reprojectPtWithDepth(Eigen::Vector2i pixel, float meterDepth) const;

  // reprojects a point pixel on image to 3d relative to this->pose CS, with an
  // assumption of the point eukledian distance from camera origin to 3D point is meterDist. 
  //! @brief ������������ ����� �� �������� ���������� �� ������.
  //! @param[in] pixel ���������� �������, ���������������� ������ �����
  //! @param[in] meterDist ���������� �� ����� � ������
  //! @retval cv::Point3d ���������� ����� �� ���������� meterDist, ���������� � pixel ��� �������� ������ �������.
  //! ������� ��������������� �����, ��������������� ������� �������, ��� �������������, ���
  //! ���������� �� ����� - meterDist. ��� ����������� ����� ���������� ��������� ���������� �� �����
  //! �� ����������� ������. ��������������� ����� ����� ���������� � �.�. exPose.inv()
  Eigen::Vector3f reprojectPtWithDist(Eigen::Vector2i pixel, float meterDist) const;

public:
  //camera projection parameters
  Eigen::Matrix3f projection;
  Eigen::Matrix<float, 5, 1> distortion;
  Eigen::Vector2i imageSize;
  Pose pose;

  float maxRange_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr getRawDepthData(const pcl::ModelCoefficients& plane);
};

#endif //