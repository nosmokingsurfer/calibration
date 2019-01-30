///
/// @file lidar.h
///@authors Panchenko A.V.
///@brief Lidar class - OOP class to work with lidar mathematical model of Velodyne like lidars
///@brief Contains device pose and generative method for lidar-plane system.
///@brief Scale and range errors are similar to VLP16 lidar
///


#pragma once
#ifndef LIDAR_H
#define LIDAR_H

#include <Eigen/Dense>
#include <pose.h>

#include <random>

using namespace Eigen;

namespace pcl
{
  struct PointXYZ;

  template<typename T>
  class PointCloud;

  struct ModelCoefficients;
}

class Lidar
{
  public:
    Lidar();
    ~Lidar();

  public:
    Pose pose;

  private:
    //Velodyne VLP16 like lidar parameters
    float maxRange_{100.0};
    float azimuth_delta_{static_cast<float>(0.4*EIGEN_PI/180.0)};
    float elevation_delta_{static_cast<float>(2.0*EIGEN_PI/180.0)};
    int lasersNUmber{16};

    float scaleErrorMean;
    float scaleErrorSigma;

    float rangeErrorMean;
    float rangeErrorSigma;

    std::normal_distribution<float> scale_error_;
    std::normal_distribution<float> range_error_;

  public:
   pcl::PointCloud<pcl::PointXYZ> getRawLidarData(const pcl::ModelCoefficients& plane);
};

#endif //LIDAR_H