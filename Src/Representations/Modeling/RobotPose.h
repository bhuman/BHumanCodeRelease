/**
 * @file RobotPose.h
 *
 * The file contains the definition of the struct RobotPose.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct RobotPose
 * The pose of the robot with additional information
 */
STREAMABLE_WITH_BASE(RobotPose, Pose2f, COMMA public BHumanMessageParticle<idRobotPose>
{
  /** Different states of robot pose estimate quality */
  ENUM(LocalizationQuality,
  {,
    superb,   /**< Everything is cool! The internal model ist unimodal and all deviations are low. */
    okay,     /**< Could be better. The model is still somewhat unimodal but at least one deviation is quite high or the validity is low. Rough estimate might be correct but not precise. */
    poor,     /**< Nope. Do not do any serious stuff, if this is the current state. There seem to be multiple quite different hypotheses about the current pose or some deviations are REALLY high. We do not have any clue about the current pose, actually. */
  });

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

  /**
   * Computes the standard deviation given the covariance matrix.
   * The unit is the same as for the pose: mm
   * Please note that both dimensions of the translation are considered and only
   * the higher deviation is returned. An alternative would be a combination like
   * return sqrt(c(0,0) + c(1,1));
   * @return A standard deviation as described above.
   */
  float getTranslationalStandardDeviation() const
  {
    return std::sqrt(std::max(covariance(0, 0), covariance(1, 1)));;
  }

  /**
   * Computes the standard deviation on the field's x-axis given the covariance matrix.
   * The unit is the same as for the pose: mm
   * @return A standard deviation as described above.
   */
  float getXAxisStandardDeviation() const
  {
    return std::sqrt(covariance(0, 0));
  }

  /**
   * Computes the standard deviation on the field's y-axis given the covariance matrix.
   * The unit is the same as for the pose: mm
   * @return A standard deviation as described above.
   */
  float getYAxisStandardDeviation() const
  {
    return std::sqrt(covariance(1, 1));
  }

  Pose2f inversePose,                             /**< The inverted robot pose. Precomputed as it is needed often. */
  (LocalizationQuality)(superb) quality,          /**< Indicates how good the pose estimate seems to be (must not be true). */
  (Matrix3f)(Matrix3f::Identity()) covariance,    /**< The covariance matrix of the estimated robot pose. */
  (unsigned)(0) timestampLastJump,                /**< Timestamp of last "big change" (jump) notification */
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
