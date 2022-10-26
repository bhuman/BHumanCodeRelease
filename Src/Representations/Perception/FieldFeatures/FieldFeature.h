/**
 * @file FieldFeature.h
 * Declaration of a struct that represents a fieldFeature.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "FieldFeatureCheck.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"

#include <vector>

#define CHECK_FIELD_FEATURE_POSE_OF(name) CHECK_FIELD_FEATURE_POSE(name, 1000.f, 40_deg)

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
   * Returns 1 of the 2 global positions of this feature (in case of isValid == true).
   */
  virtual const Pose2f getGlobalFeaturePosition() const = 0;
  void draw() const,

  (bool)(false) isValid, // <-- the percept pose is valid
});

inline FieldFeature::RobotPoseToFF::RobotPoseToFF(const Pose2f& pose1, const Pose2f& pose2) : pos1(pose1), pos2(pose2) {}
inline FieldFeature::RobotPoseToFF::RobotPoseToFF(const Pose2f& pose) : pos1(pose), pos2(pose.dotMirror()) {}
inline FieldFeature::FieldFeature(const Pose2f& pose) : Pose2f(pose), isValid(true) {}
