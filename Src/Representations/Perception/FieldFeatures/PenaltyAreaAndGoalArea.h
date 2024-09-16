/**
 * @file PenaltyAreaAndGoalArea.h
 *
 * Declaration of a struct that represents the penalty area and goal area as a field feature.
 *
 * @author Nico Wellbrock
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct PenaltyAreaAndGoalArea
 *
 * This struct defines the pose of the penalty area in field coordinates relative to the robot.
 * The pose is defined as:
 * - translation: position of the center of the penalty area
 * - rotation: the angle of the vector pointing towards the goal
 *
 */
STREAMABLE_WITH_BASE(PenaltyAreaAndGoalArea, FieldFeature,
{
  void draw() const;
  VERIFY_FIELD_FEATURE;

  const Pose2f getGlobalFeaturePosition() const override;

  /**
   * Assignment operator for Pose2f objects.
   * Validity and other members are not set
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
     */
  const PenaltyAreaAndGoalArea& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    return *this;
  };

  PenaltyAreaAndGoalArea() = default,
});
