/**
 * @file GoalFeature.h
 * Declaration of a struct that represents a goal frame
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct GoalFeature
 * it defines the pose of the goal in relativ field coords to the robot
 * the GoalFeature pose: position => position in the middle of the goal; rotation => looking in direction of goal frame (out of the field)
 */
STREAMABLE_WITH_BASE(GoalFeature, FieldFeature,
{
  void draw() const;
  CHECK_FIELD_FEATURE_POSE_OF("GoalFeature");

  GoalFeature() = default;
  GoalFeature(const Pose2f& pose) : FieldFeature(pose) {};

  /**
   * Assignment operator for Pose2f objects
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
   */
  const GoalFeature& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
   * returns 1 of the 2 global position of this feature (in case of isValid == true)
   */
  const Pose2f getGlobalFeaturePosition() const,
});
