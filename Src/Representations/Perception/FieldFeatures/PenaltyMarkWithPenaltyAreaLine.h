/**
 * @file PenaltyMarkWithPenaltyAreaLine.h
 *
 * Declaration of a struct that represents the combination of
 * a penalty mark and the line of the penalty area that is
 * closest to it (the line between penalty mark and the field's
 * halfway line):
 *
 *                  (goal)
 *-----------------------------------------
 *          |                   |
 *          |         + (mark)  |
 *          ===================== (<- this line)
 *
 * @author Tim Laue
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct PenaltyMarkWithPenaltyAreaLine
 *
 * The struct defines the pose of the penalty mark in field coordinates relative to the robot.
 * The pose is defined as:
 *  - translation => position of the penalty mark's projection on the penalty area line
 *  - rotation    => the direction of a vector from the projected mark to the penalty mark
 */
STREAMABLE_WITH_BASE(PenaltyMarkWithPenaltyAreaLine, FieldFeature,
{
  void draw() const;
  VERIFY_FIELD_FEATURE;

  PenaltyMarkWithPenaltyAreaLine() = default;
  PenaltyMarkWithPenaltyAreaLine(const Pose2f& pose) : FieldFeature(pose) {};

  /**
   * Assignment operator for Pose2f objects.
   * Validity and other members are not set
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
     */
  const PenaltyMarkWithPenaltyAreaLine& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    return *this;
  };

  /**
   * Returns 1 of the 2 global positions of this feature (in case of isValid == true).
   */
  const Pose2f getGlobalFeaturePosition() const override,
});
