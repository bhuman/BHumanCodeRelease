/**
 * @file RefereeGestureDetection.cpp
 *
 * This file implements a module that detects referee gestures.
 *
 * @author Thomas Röfer
 */

#include "RefereeGestureDetection.h"
#include "Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>

MAKE_MODULE(RefereeGestureDetection);

#if defined TARGET_ROBOT && defined NDEBUG
#define DEBUG(code)
#else
#define DEBUG(code) code
#endif

RefereeGestureDetection::RefereeGestureDetection()
{
  history.reserve(bufferSize);
}

void RefereeGestureDetection::update(RefereePercept& theRefereePercept)
{
  DEBUG(DECLARE_DEBUG_RESPONSE("debug data:module:RefereeGestureDetection:checks");
        DECLARE_DEBUG_RESPONSE("debug data:module:RefereeGestureDetection:history"));

  if(!theOptionalCameraImage.image.has_value())
  {
    theRefereePercept.gesture = RefereePercept::none;
    history.clear();
    std::fill(histogram.begin(), histogram.end(), 0);
    return;
  }

  // Only update history in SimRobot at a similar rate as on the real robot.
  if(SystemCall::getMode() == SystemCall::physicalRobot
     || Time::getTimeSince(lastHistoryUpdate) > 285
     || Time::getTimeSince(lastHistoryUpdate) < 0)
  {
    lastHistoryUpdate = Time::getCurrentSystemTime();

    // Detect gesture and filter it over time.
    updateHistogram((!mustCrossMiddle || crossesMiddle()) && pointsLowEnough() ? detectGesture() : RefereePercept::none);
    theRefereePercept.gesture = getPredominantGesture();
  }

  // Publish history for debugging purposes.
  DEBUG(STREAMABLE(Helper,
        {,
          (std::vector<RefereePercept::Gesture>) history,
        }) helper;
        helper.history.resize(history.size());
        std::copy(history.begin(), history.end(), helper.history.begin());
        MODIFY("module:RefereeGestureDetection:history", helper));
}

bool RefereeGestureDetection::crossesMiddle() const
{
  bool isLeft = false;
  bool isRight = false;
  FOREACH_ENUM(Keypoints::Keypoint, keypoint)
    if(theKeypoints.points[keypoint].valid)
    {
      isLeft |= theKeypoints.points[keypoint].position.x() < theOptionalCameraImage.image.value().width;
      isRight |= theKeypoints.points[keypoint].position.x() >= theOptionalCameraImage.image.value().width;
    }
  return isLeft && isRight;
}

bool RefereeGestureDetection::pointsLowEnough() const
{
  Vector2f dummy;
  FOREACH_ENUM(Keypoints::Keypoint, keypoint)
  {
    const Keypoints::Point& point = theKeypoints.points[keypoint];
    if(point.valid && point.position.y() > yThreshold)
      return true;
  }
  return false;
}

const Keypoints::Point& RefereeGestureDetection::getOrdered(const Keypoints::Keypoint keypoint) const
{
  if(keypoint == Keypoints::nose)
    return theKeypoints.points[Keypoints::nose];
  else
  {
    const Keypoints::Keypoint mirror = static_cast<Keypoints::Keypoint>((keypoint - 1 ^ 1) + 1);
    return theKeypoints.points[(theKeypoints.points[keypoint].position.x() >= theKeypoints.points[mirror].position.x())
                               == (keypoint < mirror) ? keypoint : mirror];
  }
}

RefereePercept::Gesture RefereeGestureDetection::detectGesture() const
{
  // Publish failed checks for debugging purposes
  DEBUG(STREAMABLE(Checks,
        {
          ~Checks() {MODIFY("module:RefereeGestureDetection:checks", *this);},
          (std::vector<std::string>) failed,
        }) checks;
        std::string text);

  const Vector2f refereeOnField(theFieldDimensions.xPosHalfwayLine,
                                (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                * (theGameState.leftHandTeam ? 1 : -1));
  const Vector2f refereeOffset = refereeOnField - theRobotPose.translation;
  const float yScale = refereeOffset.norm() / 3000.f;
  const float xScale = yScale / std::cos(std::abs(refereeOffset.angle()) - 90_deg);

  // Go through all rules.
  for(const Rule& rule : rules)
  {
    // Go through all constraints. If one is not satisfied, the gesture is rejected.
    for(const Constraint& constraint : rule.constraints)
    {
      DEBUG(text = TypeRegistry::getEnumName(rule.gesture);
            text += " - ";
            text += TypeRegistry::getEnumName(constraint.from);
            text += " -> ";
            text += TypeRegistry::getEnumName(constraint.to);
            text += ": valid?");

      const Keypoints::Point& from = getOrdered(constraint.from);
      const Keypoints::Point& to = getOrdered(constraint.to);
      if(!from.valid || !to.valid)
        goto gestureRejected;

      Vector2f diff(to.position - from.position);
      diff.x() *= xScale;
      diff.y() *= -yScale;

      DEBUG(text = text.substr(0, text.find(':') + 2) + "distance " + std::to_string(static_cast<int>(diff.norm())));
      if(!constraint.distance.isInside(diff.norm()))
        goto gestureRejected;

      const Angle angle = Angle::normalize(diff.angle() + pi_2);
      DEBUG(text = text.substr(0, text.find(':') + 2) + "direction " + std::to_string(static_cast<int>(toDegrees(angle))) + "°");
      if(!constraint.direction.isInside(angle))
        goto gestureRejected;
    }

    // A gesture was found.
    return rule.gesture;

  gestureRejected:
    DEBUG(checks.failed.push_back(text));
  }

  // No rule matched.
  return RefereePercept::none;
}

void RefereeGestureDetection::updateHistogram(const RefereePercept::Gesture gesture)
{
  if(history.size() == history.capacity())
    --histogram[history.back()];
  history.push_front(gesture);
  ++histogram[gesture];
}

RefereePercept::Gesture RefereeGestureDetection::getPredominantGesture() const
{
  std::array<int, RefereePercept::numOfGestures> histogram = this->histogram;

  // If the wide fullTime gesture was detected, the substitution gesture is also fullTime.
  if(histogram[RefereePercept::fullTime])
    histogram[RefereePercept::fullTime] += histogram[RefereePercept::substitution];

  // Determine most frequent gesture, ignoring "none".
  const auto predominantGesture = std::max_element(histogram.begin() + 1, histogram.end());

  // Return most frequent gesture if predominant enough, otherwise "none".
  return *predominantGesture < minDetectionRatio * history.capacity() ? RefereePercept::none
         : static_cast<RefereePercept::Gesture>(predominantGesture - histogram.begin());
}
