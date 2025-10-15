/**
 * @file RobotTrackerUtils.cpp
 *
 *
 *
 * @author
 */

#include "RobotTrackerUtils.h"
#include "Debugging/DebugDrawings.h"

Vector4f GlobalRobotTracker::Estimate::measurementModelPercept(const Vector4f& z)
{
  return Vector4f(z.x(), z.y(), 0, 0);
}

Vector4f GlobalRobotTracker::Estimate::measurementModel(const Vector4f&)
{
  return Vector4f();
}

void GlobalRobotTracker::Estimate::measurementUpdate(const Estimate& z)
{
  if(z.isPercept)
  {
    kf.update<4>(z.kf.mean, [this](auto& p) {return measurementModelPercept(p); }, z.kf.cov);
  }

  isPercept = false;
  wasSeen = true;
}

void GlobalRobotTracker::Estimate::predict(const std::function<void(Vector4f&)>& dynamicModel, const Matrix4f& noise)
{
  kf.predict(dynamicModel, noise);
  wasSeen = false;
}

bool GlobalRobotTracker::Robot::cleanup()
{
  //if there is only one or none estimate then return true
  if(getEstimations().size() <= 1)
    return true;

  std::vector<Estimate> cleanedEstimates;
  for(size_t n = 0; n < getEstimations().size(); n++)
  {
    //if there are "wasSeen true" estimates
    if(getEstimations().at(n).hasNewMeasurement())
      cleanedEstimates.push_back(getEstimations().at(n));
  }

  //if there are no "wasSeen true" estimates
  if(cleanedEstimates.empty() && !getEstimations().empty())
    cleanedEstimates = getEstimations();

  //if there are more than one estimate, dispose all estimates that have too high cov
  while(cleanedEstimates.size() > 1)
  {
    if((cleanedEstimates.at(1).getCovariance().isMuchSmallerThan(cleanedEstimates.at(0).getCovariance())))
    {
      cleanedEstimates.erase(cleanedEstimates.begin());
    }
    else
    {
      cleanedEstimates.erase(cleanedEstimates.begin() +1);
    }
  }

  estimates = cleanedEstimates;
  return true;
}

//TODO:
// - call the predict function
// - update penalizedProbability: change number and include game controller information
void GlobalRobotTracker::Robot::update()
{
  //TODO
  for(size_t i = 0; i < getEstimations().size(); i++)
  {
    if(!this->estimates.at(i).hasNewMeasurement())
      penalizedProbability += 1;
  }
}

Vector2f GlobalRobotTracker::Robot::getFuturePosition(unsigned time) const
{
  ASSERT(this->estimates.size() < 1);
  if(this->estimates.size() == 1)
  {
    return this->estimates.at(0).getPosition() + this->estimates.at(0).getVelocity() * time;
  }

  std::vector<Estimate> e = this->estimates;
  while(e.size() > 1)
  {
    if((e.at(1).getCovariance().isMuchSmallerThan(e.at(0).getCovariance())))
    {
      e.erase(e.begin());
    }
    else
    {
      e.erase(e.begin() + 1);
    }
  }
  return e.at(0).getPosition() + e.at(0).getVelocity() * time;
}

void GlobalRobotTracker::Teammate::includeCommunication(const Teammate& other)
{
  for(size_t n = 0; n < other.estimates.size(); n++)
  {
    this->getEstimations().push_back(other.estimates.at(n));
  }

  this->goalPosition = other.getFuturePosition(other.lastCommunication);

  if(number == 0)
    this->number = other.getNumber();
}

void GlobalRobotTracker::Teammate::update() {}

void GlobalRobotTracker::Teammate::switchNumber(Teammate& other)
{
  int temp = this->number;
  this->number = other.number;
  other.number = temp;
}

Vector2f GlobalRobotTracker::Teammate::getFuturePosition(unsigned time) const
{
  ASSERT(this->estimates.size() < 1);

  if(this->estimates.size() == 1)
  {
    return this->estimates.at(0).getPosition() + this->estimates.at(0).getVelocity() * time;
  }

  std::vector<Estimate> e = this->estimates;
  while(e.size() > 1)
  {
    if((e.at(1).getCovariance().isMuchSmallerThan(e.at(0).getCovariance())))
    {
      e.erase(e.begin());
    }
    else
    {
      e.erase(e.begin() + 1);
    }
  }
  return e.at(0).getPosition() + e.at(0).getVelocity() * time;
}

void GlobalRobotTracker::Robot::draw(const ColorRGBA& /*color*/) const {}
