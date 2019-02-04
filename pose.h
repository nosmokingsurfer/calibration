///
///@file pose.h
///@authors Panchenko A.V.
///@brief Pose class - for working with coordinate transformations in 3D space
///


#pragma once
#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>


using namespace Eigen;

class Pose{
public: 
  Pose();
  Pose(const Vector3f& angles);
  Pose(const Vector3f& angles, const  Vector3f& t);
  Pose(const Transform<float, 3, Affine>& T);
  ~Pose();  

private:
  Transform<float, 3, Affine> T;

public:
  //! @brief get corresponding rotation matrix from pose
  //! @retval 3 by 3 rotation matrix
  MatrixXf getRotation() const;

  //! @brief get correspondin translation vector from pose
  //! @retval 3d translation vector
  Vector3f getTranslation() const;

  //! @brief get Euler corresponding angles
  //! @retval 3d vector with [yaw, pitch, roll] angles
  Vector3f getAngles() const;

  //! @brief get inverse Pose: pose*pose.inverse() = E
  //! @retval inverse pose
  Pose inv() const;


  //! @brief get homogenous transformation corresponding to Pose
  //! @retval Affine3f - affine homogenous transformation
  Affine3f getTransformation() const;

  //! @brief get superposition of several Pose instances
  //! @retval resulting Pose
  Pose operator*(Pose);

  //! @brief applies Pose to given 3d vector
  //! @retval transformed vector
  Vector3f operator()(const Vector3f&) const;
};

#endif