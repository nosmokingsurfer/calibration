///
///@file pose/pose.h
///@authors Panchenko A.V.
///@brief Pose class - for working with coordinate transformations
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

public:
  Transform<float, 3, Affine> T;

public:
  MatrixXf getRotation();
  Vector3f getTranslation();

  bool setPosition(Eigen::Vector3f position);
  void getAngles();

  static Pose getRotationAroundAxis(Vector3f axis, float angle);
  Pose operator*(Pose);
  Pose inverse();

};

#endif