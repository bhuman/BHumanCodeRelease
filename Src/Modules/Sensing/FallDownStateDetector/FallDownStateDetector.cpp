/**
 * @file FallDownStateDetector.cpp
 *
 * This file implements a module that determines whether the robot is upright,
 * staggering, falling, or on the ground. Its core idea is to use the
 * orientation of the support foot relative to the ground rather than the
 * orientation of the torso.
 *
 * @author Thomas Röfer
 */

#include "FallDownStateDetector.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Rotation.h"

MAKE_MODULE(FallDownStateDetector, sensing)

void FallDownStateDetector::update(FallDownState& fallDownState)
{
  theFallDownState = &fallDownState;

  // Determine orientation of the support foot
  Pose3f rotation(RotationMatrix(Rotation::Euler::fromAngles(theInertialData.angle.x(), theInertialData.angle.y(), 0.f)));
  Pose3f leftFoot = rotation * theRobotModel.soleLeft;
  Pose3f rightFoot = rotation * theRobotModel.soleRight;
  const Pose3f& foot = leftFoot.translation.z() < rightFoot.translation.z() ? leftFoot : rightFoot;
  orientation.x() = std::atan2(foot.rotation(2, 1), foot.rotation(1, 1));
  orientation.y() = -std::atan2(foot.rotation(2, 0), foot.rotation(0, 0));
  MODIFY("module:FallDownStateDetector:orientation", orientation);

  // Determine fall direction
  if(std::abs(theInertialData.angle.x()) > std::abs(theInertialData.angle.y()))
    direction = theInertialData.angle.x() > 0.f ? FallDownState::right : FallDownState::left;
  else
    direction = theInertialData.angle.y() > 0.f ? FallDownState::front : FallDownState::back;

  // Determine how far the torso is above the ground
  torsoAboveGround = -foot.translation.z();

  // Determine conditions
  footUpright = std::abs(orientation.x()) <= maxFootOrientationToKeepUpright.x()
                && std::abs(orientation.y()) <= maxFootOrientationToKeepUpright.y();
  footStaggering = std::abs(orientation.x()) <= maxFootOrientationToKeepStaggering.x()
                   && std::abs(orientation.y()) <= maxFootOrientationToKeepStaggering.y();
  bool torsoUpright = std::abs(theInertialData.angle.x()) <= maxFootOrientationToKeepUpright.x()
                      && std::abs(theInertialData.angle.y()) <= maxFootOrientationToKeepUpright.y();
  bool torsoStaggering = std::abs(theInertialData.angle.x()) <= maxFootOrientationToKeepStaggering.x()
                         && std::abs(theInertialData.angle.y()) <= maxFootOrientationToKeepStaggering.y();

  stable = std::abs(theInertialData.gyro.x()) <= maxGyroToRegainStableState
           && std::abs(theInertialData.gyro.y()) <= maxGyroToRegainStableState;
  toUpright = footUpright
              && torsoAboveGround >= minTorsoAboveGroundToRegainUpright;
  toSquatting = torsoAboveGround < maxTorsoAboveGroundToKeepUpright;
  toStableUpright = toUpright && torsoUpright && theGroundContactState.contact;
  toAlmostStableUpright = toUpright && torsoStaggering && theGroundContactState.contact;
  toStableSquatting = footUpright && torsoUpright && theGroundContactState.contact;
  useTorsoOrientation = std::abs(theInertialData.angle.x()) >= minTorsoOrientationToDetermineDirection.x()
                        || std::abs(theInertialData.angle.y()) >= minTorsoOrientationToDetermineDirection.y();

  // Execute behavior
  beginFrame(theFrameInfo.time);
  execute(OptionInfos::getOption("Root"));
  endFrame();
}

void FallDownStateDetector::setState(FallDownState::State state, FallDownState::Direction direction,
                                     Angle odometryRotationOffset)
{
  theFallDownState->state = state;
  theFallDownState->direction = direction;
  theFallDownState->odometryRotationOffset = odometryRotationOffset;
}

void FallDownStateDetector::setStateWithPossibleDirectionChange(FallDownState::State state)
{
  if(theFallDownState->direction == FallDownState::left || theFallDownState->direction == FallDownState::right)
  {
    if(theFallDownState->direction % 4 + 1 == direction)
      setState(state, direction, -pi_2);
    else if((theFallDownState->direction + 2) % 4 + 1 == direction)
      setState(state, direction, pi_2);
    else
      setState(state, theFallDownState->direction);
  }
  else
    setState(state, theFallDownState->direction);
}

void FallDownStateDetector::playSound(const char* file) const
{
  ANNOTATION("FallDownStateDetector", file);
  if(playSounds)
    SystemCall::playSound((std::string(file) + ".wav").c_str());
}
