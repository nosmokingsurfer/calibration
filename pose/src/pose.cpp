#include <pose/pose.h>


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
            *AngleAxis<float>(angles[0], Vector3f::UnitZ())
            *AngleAxis<float>(angles[1], Vector3f::UnitX())
            *AngleAxis<float>(angles[2], Vector3f::UnitY());

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

bool Pose::setPosition(Eigen::Vector3f position)
{
  this->T.translation() = position;
  return true;
}


Pose Pose::getRotationAroundAxis(Vector3f axis, float angle)
{
  Pose result;

  result.T = AngleAxis<float>(angle, axis);
  result.T = result.T.inverse();

  return result;
}

Pose Pose::operator*(Pose pose)
{
  return Pose(this->T*pose.T);
}

Pose Pose::inverse()
{
  return Pose(this->T.inverse());
}
