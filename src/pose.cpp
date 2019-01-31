#include <pose.h>


using namespace Eigen;
using namespace std;



Pose::Pose()
{
  (*this) = Pose(Vector3f(0,0,0), Vector3f(0,0,0));  
}

Pose::Pose(const Vector3f& angles)
{
  (*this) = Pose(angles, Vector3f(0,0,0));
}


Pose::Pose(const Vector3f& angles, const Vector3f& t)
{
  
  this->T = Translation<float,3>(t)
            *AngleAxis<float>(angles[0], Vector3f::UnitZ()) //yaw
            *AngleAxis<float>(angles[1], Vector3f::UnitX()) //pitch
            *AngleAxis<float>(angles[2], Vector3f::UnitY()); //roll

  this->T = this->T.inverse();  
}

Pose::Pose(const Transform<float, 3, Affine>& _T)
{
  this->T = _T;
}

Pose::~Pose()
{}

Eigen::MatrixXf Pose::getRotation()
{
  return this->T.rotation();
}

Eigen::Vector3f Pose::getTranslation()
{
  return this->T.translation();
}


Eigen::Vector3f Pose::getAngles()
{
  return this->T.rotation().eulerAngles(2,0,1);
}


Pose Pose::operator*(Pose pose)
{
  return Pose(this->T*pose.T);
}

Vector3f Pose::operator()(const Vector3f& point) const
{
  return this->T*point;
}

Pose Pose::inverse() const
{
  return Pose(this->T.inverse());
}

Affine3f Pose::getTransformation() const
{
  return this->T;
}
