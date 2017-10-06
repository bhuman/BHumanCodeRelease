/**
 * @file OuterCorner.h
 * Declaration of a struct that represents an outer corner.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct OuterCorner
 * It defines a the pose of an outer corner in relative field coordinates to the robot.
 * The outer corner pose: position => position of the corner; rotation => looking in direction of goal
 */
STREAMABLE_WITH_BASE(OuterCorner, FieldFeature,
{
  void draw() const;
  CHECK_FIELD_FEATURE_POSE_OF("OuterCorner");

  OuterCorner() = default;
  OuterCorner(const Pose2f& pose) : FieldFeature(pose) {};

  /**
   * Assignment operator for Pose2f objects.
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
   */
  const OuterCorner& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
   * Returns 1 of the 2 global position of this feature (in case of isValid == true).
   */
  const Pose2f getGlobalFeaturePosition() const override,

  (bool) isRightCorner,
});
