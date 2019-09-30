/**
 * @file AlternativeRobotPoseHypothesis.h
 *
 * A new robot pose, based on recently observed field features.
 * This pose is meant to be used for sensor resetting inside the SelfLocalization
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct AlternativeRobotPoseHypothesis
 * An alternative pose of the robot
 */
STREAMABLE(AlternativeRobotPoseHypothesis,
{
  /** Draws this representation. */
  void draw() const;

  /** Verifies vality of the pose */
  void verify() const,

  (Pose2f) pose,                             /** the pose in 2D (position + rotation) */
  (bool)(false) isValid,                     /** true, if the content of pose is valid */
  (unsigned)(0) timeOfLastPerceptionUpdate,  /** point of time, when the last perception that contributed to the pose was seen*/
  (bool)(false) isInOwnHalf,                 /** true, if the pose is probably in the own half */
  (int)(0) numOfContributingObservations,    /** the number of observations that have been averaged to compute the pose */
});
