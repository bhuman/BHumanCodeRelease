#include "Obstacle.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Configuration/FieldDimensions.h"

Obstacle::Obstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right, const Vector2f& velocity,
                   unsigned int lastSeen, Obstacle::Type type) :
  covariance(covariance), center(center), left(left), right(right), velocity(velocity), lastSeen(lastSeen), type(type)
{}

Obstacle::Obstacle(const Matrix2f& covariance, const Vector2f& center, unsigned int lastSeen, Type type) :
  covariance(covariance), center(center), velocity(Vector2f::Zero()), lastSeen(lastSeen), type(type)
{
  float radius = 55.f;
  if(type != Obstacle::goalpost)
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
  if(!twoDimSquareEquation(one.center, one.covariance, other.center, other.covariance))
  {
    OUTPUT_ERROR("Call Florian: one's type " << one.type << " cov: " << one.covariance(0, 0) << " " << one.covariance(1, 1)
                 << " other type " << other.type << " cov: " << other.covariance(0, 0) << " " << other.covariance(1, 1));
  }
}

void Obstacle::setLeftRight(const float radius)
{
  left = right = center.normalized(radius);
  left.rotateLeft();
  right.rotateRight();
  left += center;
  right += center;
}

bool Obstacle::isTeammate() const
{
  return type == Obstacle::teammate || type == Obstacle::fallenTeammate;
}

bool Obstacle::isOpponent() const
{
  return type == Obstacle::opponent || type == Obstacle::fallenOpponent;
}
