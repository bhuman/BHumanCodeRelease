/**
 * @file MidCircle.h
 * Declaration of a struct that represents a mid circle.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct MidCircle
 * It defines a the pose of a mid circle in relative field coordinates to the robot.
 * The mid circle pose: position => position of the center point; rotation => looking in direction of the opposite goal (vertical to the mid line)
 */
STREAMABLE_WITH_BASE(MidCircle, FieldFeature,
{
  void draw() const;
  CHECK_FIELD_FEATURE_POSE_OF("MidCircle");

  MidCircle() = default;
  MidCircle(const Pose2f& pose) : FieldFeature(pose) {};

  /**
   * Assignment operator for Pose2f objects.
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
     */
  const MidCircle& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
   * Returns 1 of the 2 global position of this feature (in case of isValid == true).
   */
  const Pose2f getGlobalFeaturePosition() const override,
});
