/**
 * @file PenaltyAreaAndGoalAreaPerceptor.h
 *
 * Declaration of module that tries to find the penalty area in the current field percepts.
 * The module searches two different combinations of field lines and intersections and computes a field feature if
 * one of those is found.
 *
 * The first approach is looking for three parallel lines, corresponding to the goal-, goal area- and penalty area lines, and either corner of the penalty area (A or D).
 * The second approach is used, if the first is not successful and looks for two intersections, corresponding to the penalty area and goal area (A and B or C and D).
 *
 * ========================(goal line)=======================
 *    |         |                             |         |
 *    |         |                             |         |
 *    |         B-----(goal area line)--------C         |
 *    |                                                 |
 *    |                         X                       |
 *    A------------------(penalty area line)------------D
 *
 * @author Nico Wellbrock
 * @author Tim Gebers
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldFeatures/PenaltyAreaAndGoalArea.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"

MODULE(PenaltyAreaAndGoalAreaPerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLines),
  REQUIRES(FieldLineIntersections),
  REQUIRES(WorldModelPrediction),
  PROVIDES(PenaltyAreaAndGoalArea),
  DEFINES_PARAMETERS(
  {,
    (float)(80.f) maxLineDistanceDeviation,             /**< Maximum deviation of the measured distance from the actual distance between parallel lines on the field */
    (Angle)(10_deg) maxAngleDeviationParallelLines,     /**< Maximum angle deviation for two lines to be considered parallel */
    (float)(50.f) maxPointDeviation,                    /**< Maximum distance between two points on the field to be considered as the same point */
    (Angle)(30_deg) maxAngleDeviationParallelVectors,   /**< Maximum angle deviation between vectors */
    (float)(100.f) maxIntersectionDistanceDeviation,
    (Angle)(45_deg) maxAngleDeviationTwoCorners,
  }),
});

/**
 * This module tries to find the pose of the penalty area based on detected field lines and intersections.
 */
class PenaltyAreaAndGoalAreaPerceptor : public PenaltyAreaAndGoalAreaPerceptorBase
{
  FieldLines::Line penaltyAreaLine;
  FieldLines::Line goalAreaLine;
  FieldLines::Line goalLine;

  FieldLineIntersections::Intersection intersectionOnGoalAreaLine;
  FieldLineIntersections::Intersection intersectionOnPenaltyAreaLine;

  // Constant distances on the field
  const float penaltyAreaToGoalAreaDistance = std::abs(theFieldDimensions.xPosOwnPenaltyArea - theFieldDimensions.xPosOwnGoalArea);
  const float penaltyAreaToGoalDistance = std::abs(theFieldDimensions.xPosOwnPenaltyArea - theFieldDimensions.xPosOwnGoalLine);
  const float goalAreaToGoalDistance = std::abs(penaltyAreaToGoalDistance - penaltyAreaToGoalAreaDistance);
  const float penaltyAreaLineLength = std::abs(theFieldDimensions.yPosLeftPenaltyArea - theFieldDimensions.yPosRightPenaltyArea);
  const float penaltyAreaToGoalAreaDistancey = std::abs(theFieldDimensions.yPosRightPenaltyArea - theFieldDimensions.yPosRightGoalArea);
  const float penaltyAndGoalAreaIntersectionDistance = Geometry::distance(Vector2f(0, 0), Vector2f(penaltyAreaToGoalAreaDistance, penaltyAreaToGoalAreaDistancey));

  /**
   * The main function of this module.
   * @param penaltyAreaAndGoalArea The representation that is updated by this module
   */
  void update(PenaltyAreaAndGoalArea& penaltyAreaAndGoalArea) override;

  /**
   * Looks for three parallel lines which correspond to the penalty area line, the goal area line and the goal line.
   * The lines are stored in the respective member variables.
   * @return true if the lines were found
   */
  bool searchParallelLines();

  /**
   * Looks for an intersection either end of the penalty area line. The member variable penaltyAreaLine must be set before this function is called.
   * The intersection gets stored in the member variable intersectionOnPenaltyAreaLine.
   * @return true if the intersection was found
   */
  bool searchIntersectionOnPenaltyAreaLine();

  /**
   * Looks for an intersection of the penalty area line and an intersection of the goal area line. The member variables intersectionOnPenaltyAreaLine and intersectionOnGoalAreaLine must be set before this function is called.
   * @return true if both intersections were found
   */
  bool searchTwoIntersectionsOfPenaltyAndGoalArea();

  /**
   * The Jacobian of the function pose.inverse() * globalLandmark for pose.
   * @param pose The pose at which to evaluate the Jacobian.
   * @param globalLandmark The global landmark which is transformed into relative coordinates.
   */
  Matrix2x3f relativeLandmarkJacobian(const Pose2f& pose, const Vector2f& globalLandmark);

  /**
   * Computes the field feature from the found lines and intersection and updates the representation. penaltyAreaLine and intersectionOnPenaltyAreaLine must be set before this function is called.
   * @param penaltyAreaAndGoalArea The representation that is updated by this module
   */
  void computeFeatureFrom3Lines(PenaltyAreaAndGoalArea& penaltyAreaAndGoalArea);

  /**
   * Computes the field feature from the found intersections and updates the representation. The member variables intersectionOnPenaltyAreaLine and intersectionOnGoalAreaLine must be set before this function is called.
   * @param penaltyAreaAndGoalArea The representation that is updated by the module
   */
  void computeFeatureFrom2Corners(PenaltyAreaAndGoalArea& penaltyAreaAndGoalArea);
};
