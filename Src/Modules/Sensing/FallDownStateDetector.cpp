/**
 * @file FallDownStateDetector.cpp
 *
 * This file implements a module that provides information about the current state of the robot's body.
 *
 * @author <a href="mailto:maring@informatik.uni-bremen.de">Martin Ring</a>
 */

#include "FallDownStateDetector.h"
#include "Tools/Debugging/DebugDrawings.h"

using namespace std;

MAKE_MODULE(FallDownStateDetector, sensing)

void FallDownStateDetector::update(FallDownState& fallDownState)
{
  // Buffer data:
  accXbuffer.push_front(theInertialData.acc.x());
  accYbuffer.push_front(theInertialData.acc.y());
  accZbuffer.push_front(theInertialData.acc.z());

  // Compute average acceleration values and angles:
  float accXaverage(accXbuffer.average());
  float accYaverage(accYbuffer.average());
  float accZaverage(accZbuffer.average());
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

  //use different measurement if it kicks
  staggeringAngleX = (theMotionInfo.motion == MotionRequest::kick) ? staggeringKickThresX : staggeringThresX;
  staggeringAngleY = (theMotionInfo.motion == MotionRequest::kick) ? staggeringKickThresY : staggeringThresY;
  fallDownAngleX = (theMotionInfo.motion == MotionRequest::kick) ? fallingKickThresX: fallingThresX;
  fallDownAngleY = (theMotionInfo.motion == MotionRequest::kick) ? fallingKickThresY : fallingThresY;

  if(!specialSpecialAction())
  {
    if(theFrameInfo.getTimeSince(lastFallDetected) <= fallTime)
    {
      fallDownState.state = FallDownState::falling;
    }
    else if((abs(theInertialData.angle.x()) <= staggeringAngleX - 1_deg
             && abs(theInertialData.angle.y()) <= staggeringAngleY - 1_deg)
            || (fallDownState.state == FallDownState::upright && !isStaggering()))
    {
      fallDownState.state = FallDownState::upright;
      fallDownState.direction = FallDownState::none;
      fallDownState.sidewards = FallDownState::noot;
    }
    else if(fallDownState.state == FallDownState::staggering && isFalling())
    {
      //SystemCall::playSound("doh.wav");
      lastFallDetected = theFrameInfo.time;
      fallDownState.state = FallDownState::falling;
      fallDownState.direction = directionOf(theInertialData.angle);
      if(fallDownState.sidewards != FallDownState::fallen)
      {
        fallDownState.sidewards = sidewardsOf(fallDownState.direction);
      }
    }
    else if((isUprightOrStaggering(fallDownState)
             && isStaggering())
            || (fallDownState.state == FallDownState::staggering
                && abs(theInertialData.angle.x()) <= staggeringAngleX - 1_deg
                && abs(theInertialData.angle.y()) <= staggeringAngleY - 1_deg))
    {
      fallDownState.state = FallDownState::staggering;
      fallDownState.direction = directionOf(theInertialData.angle);
      if(fallDownState.sidewards != FallDownState::fallen)
      {
        fallDownState.sidewards = sidewardsOf(fallDownState.direction);
      }
    }
    else
    {
      fallDownState.state = FallDownState::undefined;

      if(abs(accelerationAngleXZ) > 2.5f)
      {
        fallDownState.direction = FallDownState::front;
        fallDownState.state = FallDownState::onGround;
        if(theMotionInfo.motion != MotionRequest::getUp)
        {
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
      else if(abs(accelerationAngleXZ) < 0.5f)
      {
        fallDownState.direction = FallDownState::back;
        fallDownState.state = FallDownState::onGround;
        if(theMotionInfo.motion != MotionRequest::getUp)
        {
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
      else if(abs(accelerationAngleYZ) > 2.5f)
      {
        fallDownState.direction = FallDownState::left;
        fallDownState.state = FallDownState::onGround;

        if(theMotionInfo.motion != MotionRequest::getUp)
          if(fallDownState.sidewards != FallDownState::fallen)
            fallDownState.sidewards = FallDownState::leftwards;
      }
      else if(abs(accelerationAngleYZ) < 0.5f)
      {
        fallDownState.direction = FallDownState::right;
        fallDownState.state = FallDownState::onGround;

        if(theMotionInfo.motion != MotionRequest::getUp)
          if(fallDownState.sidewards != FallDownState::fallen)
            fallDownState.sidewards = FallDownState::rightwards;
      }
    }
  }
  else
    fallDownState.state = FallDownState::undefined;

  /*
  if(theMotionInfo.motion == MotionRequest::specialAction
     && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::keeperJumpLeftBack)
  {
    if(theMotionInfo.specialActionRequest.mirror)
    {
      keeperJumped = KeeperJumpedRight;
    }
    else
    {
      keeperJumped = KeeperJumpedLeft;
    }
  }*/
  //these odometry changes really need to be proven if they are still correct
  if(keeperJumped != None)
  {
    if(theMotionInfo.motion == MotionRequest::getUp)
    {
      if(fallDownState.direction == FallDownState::front)
      {
        if(keeperJumped == KeeperJumpedLeft)
        {
          fallDownState.odometryRotationOffset = pi_2;
        }
        else
        {
          fallDownState.odometryRotationOffset = -pi_2;
        }
      }
      else
      {
        // standUpBack
        if(keeperJumped == KeeperJumpedLeft)
        {
          fallDownState.odometryRotationOffset = -pi_2;
        }
        else
        {
          fallDownState.odometryRotationOffset = pi_2;
        }
      }
      keeperJumped = None;
    }
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
          && (theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead
              ));
}

bool FallDownStateDetector::isStaggering()
{
  return abs(theInertialData.angle.x()) >= staggeringAngleX + 1_deg
         || abs(theInertialData.angle.y()) >= staggeringAngleY + 1_deg;
}

bool FallDownStateDetector::isFalling()
{
  return abs(theInertialData.angle.x()) >= fallDownAngleX
         || abs(theInertialData.angle.y()) >= fallDownAngleY;
}

FallDownState::Direction FallDownStateDetector::directionOf(Vector2a angle)
{
  if(abs(angle.x()) > abs(angle.y()) + 0.2f)
  {
    if(angle.x() < 0.f)
      return FallDownState::left;
    else
      return FallDownState::right;
  }
  else
  {
    if(angle.y() > 0.f)
      return FallDownState::front;
    else
      return FallDownState::back;
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
