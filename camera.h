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
  //! @brief восстановить точку с заданной высотой
  //! @param[in] pixel координаты пикселя, соответствующего данной точке
  //! @param[in] meterHeight высота пикселя в метрах (относительно pose)
  //! @param[in] usePrecalc использовать или нет прекалькуляционное вычисление Ainv для быстрого восстановления пачки точек
  //! @retval cv::Point3d трехмерная точка на высоте meterHeight, попадающая в pixel при проекции данной камерой
  //! Функция восстанавливает точку, соответсвтующую данному пикселю, при предположении, что высота 3d точки была
  //! meterHeight. Восстановленная точка имеет координаты в с.к. exPose.inv()
  Eigen::Vector3f reprojectPtWithHeight(Eigen::Vector2i pixel, float meterHeight = 0) const;

  // reprojects a point pixel on image to 3d relative to this->pose CS, with an
  // assumption of the point DEPTH is meterDepth. 
  // Depth here means a value of Z coordinate in camera CS (exPose)
  //! @brief восстановить точку с заданной глубиной
  //! @param[in] pixel координаты пикселя, соответствующего данной точке
  //! @param[in] meterDepth глубина пикселя в метрах
  //! @retval cv::Point3d трехмерная точка на глубине meterDepth, попадающая в pixel при проекции данной камерой.
  //! Функция восстанавливает точку, соответсвтующую данному пикселю, при предположении, что
  //! глубина точки - meterDepth. Под глубиной здесь понимается z-координата точки 
  //! в с.к. exPose. Восстановленная точка имеет координаты в с.к. exPose.inv()
  Eigen::Vector3f reprojectPtWithDepth(Eigen::Vector2i pixel, float meterDepth) const;

  // reprojects a point pixel on image to 3d relative to this->pose CS, with an
  // assumption of the point eukledian distance from camera origin to 3D point is meterDist. 
  //! @brief восстановить точку на заданном расстоянии от камеры.
  //! @param[in] pixel координаты пикселя, соответствующего данной точке
  //! @param[in] meterDist расстояние до точки в метрах
  //! @retval cv::Point3d трехмерная точка на расстоянии meterDist, попадающая в pixel при проекции данной камерой.
  //! Функция восстанавливает точку, соответсвтующую данному пикселю, при предположении, что
  //! расстояние до точки - meterDist. Под расстоянием здесь понимается эвклидово расстояние до точки
  //! от оптического центра. Восстановленная точка имеет координаты в с.к. exPose.inv()
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