/**
 * @file FallDownStateDetector.cpp
 *
 * This file implements a module that provides information about the current state of the robot's body.
 *
 * @author <a href="mailto:maring@informatik.uni-bremen.de">Martin Ring</a>
 */

#include "FallDownStateDetector.h"
#include "Representations/Infrastructure/JointData.h"
#include "Tools/Debugging/DebugDrawings.h"

using namespace std;

FallDownStateDetector::FallDownStateDetector() : lastFallDetected(0)
{
  fallDownAngleX *= pi_180;
  fallDownAngleY *= pi_180;
  onGroundAngle  *= pi_180;
  staggeringAngleX *= pi_180;
  staggeringAngleY *= pi_180;
}

void FallDownStateDetector::update(FallDownState& fallDownState)
{
  DECLARE_PLOT("module:FallDownStateDetector:accelerationAngleXZ");
  DECLARE_PLOT("module:FallDownStateDetector:accelerationAngleYZ");

  // Buffer data:
  buffers[accX].add(theFilteredSensorData.data[SensorData::accX]);
  buffers[accY].add(theFilteredSensorData.data[SensorData::accY]);
  buffers[accZ].add(theFilteredSensorData.data[SensorData::accZ]);

  // Compute average acceleration values and angles:
  float accXaverage(buffers[accX].getAverage());
  float accYaverage(buffers[accY].getAverage());
  float accZaverage(buffers[accZ].getAverage());
  float accelerationAngleXZ(atan2(accZaverage, accXaverage));
  float accelerationAngleYZ(atan2(accZaverage, accYaverage));
  MODIFY("module:FallDownStateDetector:accX",  accXaverage);
  MODIFY("module:FallDownStateDetector:accY",  accYaverage);
  MODIFY("module:FallDownStateDetector:accZ",  accZaverage);
  MODIFY("module:FallDownStateDetector:accAngleXZ", accelerationAngleXZ);
  MODIFY("module:FallDownStateDetector:accAngleYZ", accelerationAngleYZ);
  PLOT("module:FallDownStateDetector:accelerationAngleXZ", accelerationAngleXZ);
  PLOT("module:FallDownStateDetector:accelerationAngleYZ", accelerationAngleYZ);

  fallDownState.odometryRotationOffset = 0;

  if(isCalibrated() && !specialSpecialAction())
  {
    if(theFrameInfo.getTimeSince(lastFallDetected) <= fallTime)
    {
      fallDownState.state = FallDownState::falling;
    }
    else if((abs(theFilteredSensorData.data[SensorData::angleX]) <= staggeringAngleX - pi_180
             && abs(theFilteredSensorData.data[SensorData::angleY]) <= staggeringAngleY - pi_180)
            || (fallDownState.state == FallDownState::upright && !isStaggering()))
    {
      fallDownState.state = FallDownState::upright;
      fallDownState.direction = FallDownState::none;
      fallDownState.sidewards = FallDownState::noot;
    }
    else if(fallDownState.state == FallDownState::staggering && isFalling())
    {
      lastFallDetected = theFrameInfo.time;
      fallDownState.state = FallDownState::falling;
      fallDownState.direction = directionOf(theFilteredSensorData.data[SensorData::angleX], theFilteredSensorData.data[SensorData::angleY]);
      if(fallDownState.sidewards != FallDownState::fallen)
      {
        fallDownState.sidewards = sidewardsOf(fallDownState.direction);
      }
    }
    else if((isUprightOrStaggering(fallDownState)
             && isStaggering())
            || (fallDownState.state == FallDownState::staggering
                && abs(theFilteredSensorData.data[SensorData::angleX]) <= staggeringAngleX - pi_180
                && abs(theFilteredSensorData.data[SensorData::angleY]) <= staggeringAngleY - pi_180))
    {
      fallDownState.state = FallDownState::staggering;
      fallDownState.direction = directionOf(theFilteredSensorData.data[SensorData::angleX], theFilteredSensorData.data[SensorData::angleY]);
      if(fallDownState.sidewards != FallDownState::fallen)
      {
        fallDownState.sidewards = sidewardsOf(fallDownState.direction);
      }
    }
    else
    {
      fallDownState.state = FallDownState::undefined;

      if(abs(accelerationAngleXZ) < 0.5f)
      {
        fallDownState.direction = FallDownState::front;
        if(theMotionInfo.motion != MotionRequest::getUp)
        {
          fallDownState.state = FallDownState::onGround;
          if(fallDownState.sidewards == FallDownState::leftwards)
          {
            fallDownState.odometryRotationOffset = pi_2;
            fallDownState.sidewards = FallDownState::fallen;
          }
          else if(fallDownState.sidewards == FallDownState::rightwards)
          {
            fallDownState.odometryRotationOffset = -pi_2;
            fallDownState.sidewards = FallDownState::fallen;
          }
        }
      }
      else if(abs(accelerationAngleXZ) > 2.5f)
      {

        fallDownState.direction = FallDownState::back;
        if(theMotionInfo.motion != MotionRequest::getUp)
        {
          fallDownState.state = FallDownState::onGround;
          if(fallDownState.sidewards == FallDownState::leftwards)
          {
            fallDownState.odometryRotationOffset = -pi_2;
            fallDownState.sidewards = FallDownState::fallen;
          }
          else if(fallDownState.sidewards == FallDownState::rightwards)
          {
            fallDownState.odometryRotationOffset = pi_2;
            fallDownState.sidewards = FallDownState::fallen;
          }
        }
      }
      else if(abs(accelerationAngleYZ) < 0.5f)
      {
        fallDownState.direction = FallDownState::left;
        if(theMotionInfo.motion != MotionRequest::getUp)
        {
          fallDownState.state = FallDownState::onGround;

          if(fallDownState.sidewards != FallDownState::fallen)
          {
            fallDownState.sidewards = FallDownState::leftwards;
          }
        }
      }
      else if(abs(accelerationAngleYZ) > 2.5f)
      {
        fallDownState.direction = FallDownState::right;
        if(theMotionInfo.motion != MotionRequest::getUp)
        {
          fallDownState.state = FallDownState::onGround;

          if(fallDownState.sidewards != FallDownState::fallen)
          {
            fallDownState.sidewards = FallDownState::rightwards;
          }
        }
      }

    }
  }
  else
  {
    fallDownState.state = FallDownState::undefined;
  }
}


bool FallDownStateDetector::isUprightOrStaggering(FallDownState& fallDownState)
{
  return fallDownState.state == FallDownState::upright
         || fallDownState.state == FallDownState::staggering;
}

bool FallDownStateDetector::specialSpecialAction()
{
  return (theMotionInfo.motion == MotionRequest::specialAction
          && (theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead));
}

bool FallDownStateDetector::isStaggering()
{
  return abs(theFilteredSensorData.data[SensorData::angleX]) >= staggeringAngleX + pi_180
         || abs(theFilteredSensorData.data[SensorData::angleY]) >= staggeringAngleY + pi_180;
}

bool FallDownStateDetector::isFalling()
{
  return abs(theFilteredSensorData.data[SensorData::angleX]) >= fallDownAngleX
         || abs(theFilteredSensorData.data[SensorData::angleY]) >= fallDownAngleY;
}

bool FallDownStateDetector::isCalibrated()
{
  return theInertiaSensorData.calibrated;
}

FallDownState::Direction FallDownStateDetector::directionOf(float angleX, float angleY)
{
  if(abs(angleX) > abs(angleY) + 0.2f)
  {
    if(angleX < 0.f) return FallDownState::left;
    else return FallDownState::right;
  }
  else
  {
    if(angleY > 0.f) return FallDownState::front;
    else return FallDownState::back;
  }
}

FallDownState::Sidestate FallDownStateDetector::sidewardsOf(FallDownState::Direction dir)
{
  switch(dir)
  {
  case FallDownState::left:
    return FallDownState::leftwards;
  case FallDownState::right:
    return FallDownState::rightwards;
  default:
    return FallDownState::noot;
  }
}

MAKE_MODULE(FallDownStateDetector, Sensing)
