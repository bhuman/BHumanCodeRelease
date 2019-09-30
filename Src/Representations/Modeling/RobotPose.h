/**
 * @file RobotPose.h
 *
 * The file contains the definition of the struct RobotPose.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"

/**
 * @struct RobotPose
 * The pose of the robot with additional information
 */
STREAMABLE_WITH_BASE(RobotPose, Pose2f, COMMA public BHumanMessageParticle<idRobotPose>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;

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
  void draw() const;

  enum { unknownDeviation = 100000 };
  Pose2f inversePose,

  (float)(0) validity,                            /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
  (unsigned)(0) timeOfLastConsideredFieldFeature, /**< Additional information about how good this pose might be */
  (float)(unknownDeviation) deviation,            /**< The deviation of the robot pose. */
  (Matrix3f)(Matrix3f::Identity()) covariance,    /**< The covariance matrix of the estimated robot pose. */
  (unsigned)(0) timestampLastJump,                /**< Timestamp of last "big change" (jump) notificaion */
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
