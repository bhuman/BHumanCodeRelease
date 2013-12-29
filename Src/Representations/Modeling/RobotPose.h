/**
 * @file RobotPose.h
 *
 * The file contains the definition of the class RobotPose.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Math/Matrix3x3.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class RobotPose
* The pose of the robot with additional information
*/
STREAMABLE_WITH_BASE(RobotPose, Pose2D,
{
public:
  enum {unknownDeviation = 100000};

  /** Assignment operator for Pose2D objects
  * @param other A Pose2D object
  * @return A reference to the object after the assignment
  */
  const RobotPose& operator=(const Pose2D& other)
  {
    (Pose2D&) *this = other;
    // validity and co are not set
    return *this;
  }

  /** Draws the robot pose in the color of the team to the field view*/
  void draw(bool teamRed),

  (float)(0) validity,                 /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
  (float)(unknownDeviation) deviation, /**< The deviation of the robot pose. */
  (Matrix3x3<>) covariance,            /**< The covariance matrix of the estimated robot pose. */
});

/**
* @class GroundTruthRobotPose
* The same as the RobotPose, but - in general - provided by an external
* source that has ground truth quality
*/
STREAMABLE_WITH_BASE(GroundTruthRobotPose, RobotPose,
{
public:
  /** Draws the robot pose to the field view*/
  void draw() const,

  (unsigned)(0) timestamp,
});

/**
* @class RobotPoseCompressed
* A compressed version of RobotPose used in team communication
*/
STREAMABLE(RobotPoseCompressed,
{
public:
  RobotPoseCompressed(const RobotPose& robotPose);
  operator RobotPose() const,

  (Vector2<short>) translation,
  (char) rotation,
  (unsigned char) validity,
  (float) deviation,
});
