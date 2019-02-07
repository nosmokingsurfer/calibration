///
/// @file lidar.h
///@authors Panchenko A.V.
///@brief Lidar class - OOP class to work with lidar mathematical model of Velodyne like lidars
///@brief Contains device pose and generative method for lidar-plane system.
///@brief Range errors are similar to VLP16 lidar
///


#pragma once
#ifndef LIDAR_H
#define LIDAR_H

#include <Eigen/Dense>
#include <pose.h>
#include <random>

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>

using namespace Eigen;

class Lidar
{
  public:
    Lidar();
    Lidar(const Pose& pose);
    ~Lidar();

  public:
    Pose pose_; //sensor pose in global frame

  private:
    //Velodyne VLP16 like lidar parameters
    float maxRange_; // max lidar measurement range
    float minRange_; //min lidar measurement range
    float azimuth_delta_{static_cast<float>(0.4*EIGEN_PI/180.0)}; //angular step for azimuth 
    float elevation_delta_{static_cast<float>(2.0*EIGEN_PI/180.0)}; //step for elevation angle
    int lasersNumber{16}; //number of laser beams

    float rangeErrorMean_; // mean range error
    float rangeErrorSigma_; // covariance of range error

    std::normal_distribution<float> range_error_; // range error random generator

  public:

   //! @brief Returns point cloud in sensor frame for give plane model
   //! @param [in] plane - array of coefficients of plane equation a*x + b*y + c*z + d = 0
   //! @retval point cloud - points on a plane with some random range errors
   pcl::PointCloud<pcl::PointXYZ>::Ptr getRawLidarData(const pcl::ModelCoefficients& plane);

   //! @brief Returns point cloud in sensor frame which represents sensor perception model.
   //! @brief Good for debug purposes
   pcl::PointCloud<pcl::PointXYZ>::Ptr getProjectionModel() const;
};

#endif //LIDAR_H