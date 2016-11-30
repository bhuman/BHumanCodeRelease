/**
 * @file RobotPose.h
 *
 * The file contains the definition of the struct RobotPose.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct RobotPose
 * The pose of the robot with additional information
 */
STREAMABLE_WITH_BASE(RobotPose, Pose2f,
{
  enum { unknownDeviation = 100000 };
  Pose2f inversePose;

  /**
   * Assignment operator for Pose2f objects
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
   */
  const RobotPose& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    inversePose = other.inverse();
    // validity and co are not set
    return *this;
  }

  void onRead();
  Pose2f inverse() const;
  /** Verifies that the robot pose contains valid values. */
  void verify() const;
  /** Draws the robot pose in the color of the team to the field view. */
  void draw() const,

  (float)(0) validity,                            /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
  (unsigned)(0) timeOfLastConsideredFieldFeature, /**< Additional information about how good this pose might be */
  (float)(unknownDeviation) deviation,            /**< The deviation of the robot pose. */
  (Matrix3f)(Matrix3f::Identity()) covariance,    /**< The covariance matrix of the estimated robot pose. */
});

/**
 * @struct GroundTruthRobotPose
 * The same as the RobotPose, but - in general - provided by an external
 * source that has ground truth quality
 */
STREAMABLE_WITH_BASE(GroundTruthRobotPose, RobotPose,
{
  /** Draws the robot pose to the field view*/
  void draw() const,

  (unsigned)(0) timestamp,
});

/**
 * @struct RobotPoseCompressed
 * A compressed version of RobotPose used in team communication
 */
STREAMABLE(RobotPoseCompressed,
{
  RobotPoseCompressed() = default;
  RobotPoseCompressed(const RobotPose& robotPose);
  operator RobotPose() const,

  (Vector2f) translation,
  (float) rotation,
  (unsigned char) validity,
  (unsigned) timeOfLastConsideredFieldFeature,
  (float) deviation,
  (std::array<float, 6>) covariance,
});
