#include "Obstacle.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Configuration/FieldDimensions.h"

Obstacle::Obstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right, const Vector2f& velocity, Obstacle::Type type) :
  covariance(covariance), center(center), left(left), right(right), velocity(velocity), type(type)
{}

Obstacle::Obstacle(const Matrix2f& covariance, const Vector2f& center, Type type) :
covariance(covariance), center(center), velocity(Vector2f::Zero()), type(type)
{
  float radius = 55.f;
  if(type != goalpost)
    radius = getRobotDepth();
  else if(Blackboard::getInstance().exists("FieldDimensions"))
    radius = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]).goalPostRadius;

  left = right = center.normalized(radius);
  left.rotateLeft();
  right.rotateRight();
  left += center;
  right += center;
}

void Obstacle::fusion2D(Obstacle& one, const Obstacle& other)
{
  //multidimensional square equation (german: "Multidimensionale quadratische Ausgleichsrechnung" aus dem Skript
  //"Theorie der Sensorfusion")
  const Eigen::Matrix<float, 4, 2> A = (Eigen::Matrix<float, 4, 2>() << Matrix2f::Identity(), Matrix2f::Identity()).finished();
  const Eigen::Matrix<float, 2, 4> AT = A.transpose();
  const Eigen::Matrix4f Sigma = ((Eigen::Matrix4f() << one.covariance, Matrix2f::Zero(), Matrix2f::Zero(), other.covariance).finished());
  if(Sigma.determinant() == 0)
  {
    OUTPUT_ERROR("Call Florian: one's type " << one.type << " cov: " << one.covariance(0, 0) << " " << one.covariance(1, 1)
                 << " other type " << other.type << " cov: " << other.covariance(0, 0) << " " << other.covariance(1, 1));
    return;
  }
  const Eigen::Matrix<float, 4, 4> SigmaInv = Sigma.inverse();
  const Eigen::Vector4f z = (Eigen::Vector4f() << one.center, other.center).finished();
  const Eigen::Matrix2f ATSigmaInvAINV = ((AT * SigmaInv) * A).inverse();
  const Vector2f X = ATSigmaInvAINV * AT * SigmaInv * z;
  ASSERT(X.allFinite());
  ASSERT(ATSigmaInvAINV.allFinite());
  one.center << X(0), X(1);
  one.covariance = ATSigmaInvAINV;
}

void Obstacle::setLeftRight(const float radius)
{
  left = right = center.normalized(radius);
  left.rotateLeft();
  right.rotateRight();
  left += center;
  right += center;
}
