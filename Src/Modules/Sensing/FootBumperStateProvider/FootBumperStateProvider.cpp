/**
 * FootBumperStateProvider.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: arne
 *              simont@tzi.de
 */

#include "FootBumperStateProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(FootBumperStateProvider, sensing);

void FootBumperStateProvider::update(FootBumperState& footBumperState)
{
  bool ignoreInnerBumper = ignoreContact();
  bool ignoreByState = !((theMotionInfo.executedPhase == MotionPhase::stand || theMotionInfo.executedPhase == MotionPhase::walk) &&
                         (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING) && //The bumper is used for configuration in initial
                         (theFallDownState.state == FallDownState::upright));
  // Check, if any bumper is pressed
  const bool leftFootLeft = !theDamageConfigurationBody.sides[Legs::left].footBumperDefect && checkContact(KeyStates::lFootLeft, leftFootLeftDuration);
  const bool leftFootRight = !theDamageConfigurationBody.sides[Legs::left].footBumperDefect && checkContact(KeyStates::lFootRight, leftFootRightDuration) && !ignoreInnerBumper;
  const bool rightFootLeft = !theDamageConfigurationBody.sides[Legs::right].footBumperDefect &&  checkContact(KeyStates::rFootLeft, rightFootLeftDuration) && !ignoreInnerBumper;
  const bool rightFootRight = !theDamageConfigurationBody.sides[Legs::right].footBumperDefect && checkContact(KeyStates::rFootRight, rightFootRightDuration);
  const bool contactLeftFoot = leftFootLeft || leftFootRight;
  const bool contactRightFoot = rightFootLeft || rightFootRight;
  // Update statistics
  if(!ignoreByState)
  {
    if(contactLeftFoot)
    {
      contactBufferLeftLeft.push_front(leftFootLeft ? 1 : 0);
      contactBufferLeftRight.push_front(leftFootRight ? 1 : 0);
      contactBufferLeft.push_front(1);
      contactDurationLeft++;
    }
    else
    {
      contactBufferLeftLeft.push_front(0);
      contactBufferLeftRight.push_front(0);
      contactBufferLeft.push_front(0);
      contactDurationLeft = 0;
    }
    if(contactRightFoot)
    {
      contactBufferRightLeft.push_front(rightFootLeft ? 1 : 0);
      contactBufferRightRight.push_front(rightFootRight ? 1 : 0);
      contactBufferRight.push_front(1);
      contactDurationRight++;
    }
    else
    {
      contactBufferRightLeft.push_front(0);
      contactBufferRightRight.push_front(0);
      contactBufferRight.push_front(0);
      contactDurationRight = 0;
    }
  }
  else
  {
    //In the current robot state, we ignore all bumper signals
    contactBufferLeftLeft.push_front(0);
    contactBufferLeftRight.push_front(0);
    contactBufferLeft.push_front(0);
    contactBufferRightLeft.push_front(0);
    contactBufferRightRight.push_front(0);
    contactBufferRight.push_front(0);
    contactDurationLeft = 0;
    contactDurationRight = 0;
  }
  // Generate model
  int thresholdContacts = static_cast<int>(1.f / Constants::motionCycleTime / contactThreshold);
  if((theMotionInfo.executedPhase == MotionPhase::stand || theMotionInfo.executedPhase == MotionPhase::walk) &&
     (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING) && //The bumper is used for configuration in initial
     (theFallDownState.state == FallDownState::upright))
  {
    // One contact buffer must exceed the threshold and both bumper sensors of one foot must detected at least 1 contact. Otherwise no contact is detected
    if(contactBufferLeft.sum() > thresholdContacts && contactBufferLeftLeft.sum() > 0 && contactBufferLeftRight.sum() > 0)
    {
      footBumperState.status[Legs::left].contact = true;
      footBumperState.status[Legs::left].contactDuration = contactDurationLeft;
      if(contactLeftFoot)
        footBumperState.status[Legs::left].lastContact = theFrameInfo.time;
    }
    else
    {
      footBumperState.status[Legs::left].contact = false;
      footBumperState.status[Legs::left].contactDuration = 0;
    }
    if(contactBufferRight.sum() > thresholdContacts && contactBufferRightLeft.sum() > 0 && contactBufferRightRight.sum() > 0)
    {
      footBumperState.status[Legs::right].contact = true;
      footBumperState.status[Legs::right].contactDuration = contactDurationRight;
      if(contactRightFoot)
        footBumperState.status[Legs::right].lastContact = theFrameInfo.time;
    }
    else
    {
      footBumperState.status[Legs::right].contact = false;
      footBumperState.status[Legs::right].contactDuration = 0;
    }
  }
  else
  {
    footBumperState.status[Legs::left].contact = false;
    footBumperState.status[Legs::right].contact = false;
    footBumperState.status[Legs::left].contactDuration = 0;
    footBumperState.status[Legs::right].contactDuration = 0;
  }

  // Debugging stuff:
  PLOT("module:FootBumperStateProvider:sumLeft", contactBufferLeft.sum());
  PLOT("module:FootBumperStateProvider:durationLeft", contactDurationLeft);
  PLOT("module:FootBumperStateProvider:sumRight", contactBufferRight.sum());
  PLOT("module:FootBumperStateProvider:durationRight", contactDurationRight);
  PLOT("module:FootBumperStateProvider:contactLeft", footBumperState.status[Legs::left].contact ? 10 : 0);
  PLOT("module:FootBumperStateProvider:contactRight", footBumperState.status[Legs::right].contact ? 10 : 0);
  PLOT("module:FootBumperStateProvider:leftFootLeft", leftFootLeft ? 10 : 0);
  PLOT("module:FootBumperStateProvider:leftFootRight", leftFootRight ? 10 : 0);
  PLOT("module:FootBumperStateProvider:rightFootLeft", rightFootLeft ? 10 : 0);
  PLOT("module:FootBumperStateProvider:rightFootRight", rightFootRight ? 10 : 0);
}

bool FootBumperStateProvider::checkContact(KeyStates::Key key, int& duration)
{
  bool pressed = theKeyStates.pressed[key];
  duration = pressed ? duration + 1 : 0;
  // if key is pressed longer than the malfunction threshold, it is ignored
  return pressed && duration < malfunctionThreshold;
}

bool FootBumperStateProvider::ignoreContact()
{
  const Vector2f footLeftLowerPoint = (theRobotModel.soleLeft * Vector3f(theRobotDimensions.bumperInnerEdge.x() + lowerShift.x(), -theRobotDimensions.bumperInnerEdge.y() - lowerShift.y(), 0.f)).head<2>();
  const Vector2f footLeftUpperPoint = (theRobotModel.soleLeft * Vector3f(theRobotDimensions.bumperInnerEdge.x() + upperShift.x(), -theRobotDimensions.bumperInnerEdge.y() - upperShift.y(), 0.f)).head<2>();
  const Vector2f footRightLowerPoint = (theRobotModel.soleRight * Vector3f(theRobotDimensions.bumperInnerEdge.x() + lowerShift.x(), theRobotDimensions.bumperInnerEdge.y() + lowerShift.y(), 0.f)).head<2>();
  const Vector2f footRightUpperPoint = (theRobotModel.soleRight * Vector3f(theRobotDimensions.bumperInnerEdge.x() + upperShift.x(), theRobotDimensions.bumperInnerEdge.y() + upperShift.y(), 0.f)).head<2>();
  const Vector2f footRightLowerExtraPoint = (theRobotModel.soleRight * Vector3f(theRobotDimensions.bumperInnerEdge.x() + lowerShift.x() + additionalLowerShift, theRobotDimensions.bumperInnerEdge.y() + lowerShift.y(), 0.f)).head<2>();
  const Vector2f footLeftLowerExtraPoint = (theRobotModel.soleLeft * Vector3f(theRobotDimensions.bumperInnerEdge.x() + lowerShift.x() + additionalLowerShift, -theRobotDimensions.bumperInnerEdge.y() - lowerShift.y(), 0.f)).head<2>();

  return std::abs(Geometry::getDistanceToEdge(Geometry::Line(footLeftLowerPoint, footLeftUpperPoint - footLeftLowerPoint), footRightLowerPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footLeftLowerPoint, footLeftUpperPoint - footLeftLowerPoint), footRightUpperPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footRightLowerPoint, footRightUpperPoint - footRightLowerPoint), footLeftLowerPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footRightLowerPoint, footRightUpperPoint - footRightLowerPoint), footLeftUpperPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footLeftLowerExtraPoint, footLeftLowerPoint - footLeftLowerExtraPoint), footRightLowerPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footLeftLowerExtraPoint, footLeftLowerPoint - footLeftLowerExtraPoint), footRightUpperPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footRightLowerExtraPoint, footRightLowerPoint - footRightLowerExtraPoint), footLeftLowerPoint)) < distanceBetweenFootBumpers ||
         std::abs(Geometry::getDistanceToEdge(Geometry::Line(footRightLowerExtraPoint, footRightLowerPoint - footRightLowerExtraPoint), footLeftUpperPoint)) < distanceBetweenFootBumpers;
}
