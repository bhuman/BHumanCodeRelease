/**
 * @file FieldFeature.h
 * Declaration of a struct that represents a field feature.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeatureCheck.h"
#include "Streaming/AutoStreamable.h"

/**
 * The pose of a field feature.
 */
STREAMABLE_WITH_BASE(FieldFeature, Pose2f,
{
  STREAMABLE(RobotPoseToFF,
  {
    RobotPoseToFF(const Pose2f& pose1, const Pose2f& pose2);
    RobotPoseToFF(const Pose2f& pose),

    (Pose2f) pos1,
    (Pose2f) pos2,
  });

  FieldFeature() = default;
  FieldFeature(const Pose2f& pose);

  /**
   * Assignment operator for Pose2f objects.
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
   */
  const FieldFeature& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
   * Returns the position of the robot on the field according to this feature.
   * Because all features on the field are dot mirrored, this function will
   * return a struct holding both.
   * !!! Just use in case of isValid (assert(isValid)) !!!
   * @return RobotPoseToFF the two Poses
   */
  const RobotPoseToFF getGlobalRobotPosition() const;

  /**
   * Given a robot pose, the better fitting pose computed by the field feature is returned in the pickedPose parameter.
   * @param robotPose The currently assumed pose of the robot
   * @param pickedPose A possible robot pose measurement based on the field feature
   * @return true, if pickedPose has been filled. false, otherwise.
   */
  bool pickMorePlausiblePose(const Pose2f& robotPose, Pose2f& pickedPose) const;

  /**
   * Returns 1 of the 2 global positions of this feature (in case of isValid == true).
   */
  virtual const Pose2f getGlobalFeaturePosition() const = 0;

  /** Draws the representation */
  void draw() const,

  (Matrix3f)((Matrix3f() << 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f).finished()) covOfAbsoluteRobotPose, /**< The robot pose's overall covariance in field coordinates */
  (bool)(false) isValid,             /**< Set to "true", if the representation contains valid data */
});

inline FieldFeature::RobotPoseToFF::RobotPoseToFF(const Pose2f& pose1, const Pose2f& pose2) : pos1(pose1), pos2(pose2) {}
inline FieldFeature::RobotPoseToFF::RobotPoseToFF(const Pose2f& pose) : pos1(pose), pos2(pose.dotMirror()) {}
inline FieldFeature::FieldFeature(const Pose2f& pose) : Pose2f(pose), isValid(true) {}
