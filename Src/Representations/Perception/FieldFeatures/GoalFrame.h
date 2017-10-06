/**
 * @file GoalFrame.h
 * Declaration of a struct that represents a goal frame.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct GoalFrame
 * It defines a the pose of goal frame in relative field coordinates to the robot.
 * The goal frame pose: position => position in the middle of the goal; rotation => looking in direction of goal frame (out of the field)
 */
STREAMABLE_WITH_BASE(GoalFrame, FieldFeature,
{
  void draw() const;
  CHECK_FIELD_FEATURE_POSE_OF("GoalFrame");

  GoalFrame() = default;
  GoalFrame(const Pose2f& pose);

  /**
   * Assignment operator for Pose2f objects.
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
   */
  const GoalFrame& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
   * Returns 1 of the 2 global position of this feature (in case of isValid == true).
   */
  const Pose2f getGlobalFeaturePosition() const override,

  (bool)(false) isGroundLineValid,
});

inline GoalFrame::GoalFrame(const Pose2f& pose) : FieldFeature(pose), isGroundLineValid(true) {}
