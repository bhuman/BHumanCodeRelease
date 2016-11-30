#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"

STREAMABLE(GoaliePose,
{
  /**
   * Checks whether robot is near its target position using the provided thresholds
   * @param distTolerance Tolerance in mm to the target pose to consider it 'reached'
   * @param rotationTolerance Tolerance in degree to the target pose to consider it 'reached'
   * @return Whether the pose is reached
   */
  bool positionReached(float distTolerance = 100.f, Angle rotationTolerance = 10_deg) const
  {
    return goaliePoseRel.translation.norm() < distTolerance &&
           std::abs(goaliePoseRel.rotation) < rotationTolerance;
  }
  void draw() const;
  void verify() const;
  ,
  (Pose2f) goaliePoseField,        /**< Current walk target in absolute coordinates */
  (Pose2f) goaliePoseRel,          /**< Current walk target in relative coordinates */
  (bool)(false) isNearLeftPost,    /**< Whether the keeper is tanding near to the left goal post and should thus not jump */
  (bool)(false) isNearRightPost,   /**< Whether the keeper is tanding near to the right goal post and should thus not jump */
  (bool)(false) align,             /**< Whether keeper should align its rotation towards its walk target */
  (bool)(false) isBallApproaching, /**< Whether the ball is moving from the opponent half towards our own */
});
