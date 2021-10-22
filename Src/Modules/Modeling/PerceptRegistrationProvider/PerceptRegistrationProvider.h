/**
 * @file PerceptRegistrationProvider.h
 *
 * This file declares a module,
 * which provides functions to assign perceived field elements
 * to their counterparts in the model of the field.
 * These pairs are later used in the self-localization
 * process to update the robot's pose estimates.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/PerceptRegistration.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/FieldFeatures/MidCircle.h"
#include "Representations/Perception/FieldFeatures/PenaltyMarkWithPenaltyAreaLine.h"
#include "Representations/Perception/GoalPercepts/GoalPostsPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Module/Module.h"

MODULE(PerceptRegistrationProvider,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(CirclePercept),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLineIntersections),
  REQUIRES(FieldLines),
  REQUIRES(FrameInfo),
  REQUIRES(GoalPostsPercept),
  REQUIRES(MidCircle),
  REQUIRES(MotionInfo),
  REQUIRES(PenaltyMarkPercept),
  REQUIRES(PenaltyMarkWithPenaltyAreaLine),
  PROVIDES(PerceptRegistration),
  LOADS_PARAMETERS(
  {,
    (bool) goalFrameIsPerceivedAsLines,               /**< Set to true, if the field has goals with a white frame, which can easily be perceived as a line, on the floor */
    (bool) registerLinesOnCenterCircle,               /**< Set to true, if small lines on an unperceived center circle should be used for localization. Set to false to ignore these short lines. */
    (float) maxPenaltyMarkDeviation,                  /**< The maximum distance (in mm) between model and perception for registering a penalty mark percept */
    (float) maxIntersectionDeviation,                 /**< The maximum distance (in mm) between model and perception for registering a field line intersection */
    (float) lineAssociationCorridor,                  /**< Maximum distance between points on a perceived line and a line in the world model */
    (float) globalPoseAssociationMaxDistanceDeviation,/**< Distance threshold (metric) for associating a computed pose (by field feature) and the currently estimated pose */
    (Angle) globalPoseAssociationMaxAngularDeviation, /**< Angular threshold for associating a computed pose (by field feature) and the currently estimated pose */
    (Vector2f) robotRotationDeviation,                /**< Deviation of the rotation of the robot's torso */
    (Vector2f) robotRotationDeviationInStand,         /**< Deviation of the rotation of the robot's torso when it is standing. */
  }),
});


/**
 * @class PerceptRegistrationProvider
 *
 * A module contributing to self-localization. It provides functions to assign perceived field elements
 * to their counterparts in the model of the field. These pairs are later used in the self-localization
 * process to update the robot's pose estimates.
 */
class PerceptRegistrationProvider : public PerceptRegistrationProviderBase
{
public:
  /** Constructor */
  PerceptRegistrationProvider();

private:
  /**
   * An internal representation for the lines
   * in the field model, containing additional information.
   */
  class WorldModelFieldLine
  {
  public:
    /**
     * Constructor, computes some attributes.
     * @param s The starting point of the line
     * @param e The ending point of the line
     */
    WorldModelFieldLine(const Vector2f& s, const Vector2f& e)
    {
      start = s;
      end = e;
      dir = end - start;
      length = dir.norm();
      dir.normalize();
      isCenterLine = start.x() == 0.f && end.x() == 0.f;
    }

    Vector2f start;    /**< The starting point of the line. */
    Vector2f end;      /**< The ending point of the line. */
    Vector2f dir;      /**< The normalized direction of the line (from starting point). */
    float length;      /**< The length of the line. */
    bool isCenterLine; /**< True, if the line is the center line. False otherwise. */
  };


  Pose3f inverseCameraMatrix;                                 /**< Precomputed matrix that is needed multiple times */
  Vector2f currentRotationDeviation;                          /**< Set to either robotRotationDeviation or robotRotationDeviationInStand */

  Vector2f ownPenaltyMarkWorldModel;                          /**< Original position of own penalty mark (in field coordinates) */
  Vector2f opponentPenaltyMarkWorldModel;                     /**< Original position of opponent penalty mark (in field coordinates) */
  Vector2f ownGoalPostsWorldModel[2];                         /**< The positions of the two posts of our goal. */
  Vector2f opponentGoalPostsWorldModel[2];                    /**< The positions of the two posts of the opponent goal. */
  std::vector<WorldModelFieldLine> verticalLinesWorldModel;   /**< Field lines to match against, lines in this list are parallel to the field's x axis */
  std::vector<WorldModelFieldLine> horizontalLinesWorldModel; /**< Field lines to match against, lines in this list are parallel to the field's y axis */

  Matrix2f penaltyMarkCovariance;                             /**< Covariance of last penalty mark perception (saved, as it is needed multiple times in one frame)*/
  unsigned int timeOfLastPenaltyMarkCovarianceUpdate;         /**< Timestamp of frame in which the penalty mark covariance was computed the last time*/
  Matrix2f centerCircleCovariance;                            /**< Covariance of last center circle perception (saved, as it is needed multiple times in one frame)*/
  unsigned int timeOfLastCenterCircleCovarianceUpdate;        /**< Timestamp of frame in which the center circle covariance was computed the last time*/
  std::vector<Matrix2f> intersectionCovariances;              /**< Covariances of last intersection perceptions (saved, as they are needed multiple times in one frame)*/
  std::vector<unsigned int> timesOfLastIntersectionCovarianceUpdates; /**< Timestamps of frames in which the intersection perceptions covariances were computed the last time*/
  std::vector<Matrix2f> goalPostsCovariances;                     /**< Covariances of last goal post perceptions (saved, as they are needed multiple times in one frame)*/
  std::vector<unsigned int> timesOfLastGoalPostCovarianceUpdates; /**< Timestamps of frames in which the goal post perception covariances were computed the last time*/
  std::vector<Matrix2f> lineCovariances;                      /**< Covariance of last line perceptions (saved, as they are needed multiple times in one frame)*/
  std::vector<unsigned int> timesOfLastLineCovarianceUpdates; /**< Timestamps of frames in which the line perceptions covariances were computed the last time*/

  float maxCenterCircleDeviation;                             /**< The maximum distance (in mm) between model and perception for registering a center circle percept */
  float maxGoalPostDeviation;                                 /**< The maximum distance (in mm) between model and perception for registering a goal post percept */

  /**
   * The module's main function, precomputes whatever can be precomputed
   * and sets the registration function in the given representation.
   * @param perceptRegistration The representation provided by this module.
   */
  void update(PerceptRegistration& perceptRegistration);

  /**
   * Makes some checks and precomputes values that are the same for
   * all later calls (within one frame) of the registration functions.
   * @param perceptRegistration The representation provided by this module. This function sets some of the members.
   */
  void preprocessMeasurements(PerceptRegistration& perceptRegistration);

  /**
   * Determine which absolute pose measurements (center circle with line, penalty area ...)
   * are compatible (given some thresholds) to the assumed robot pose
   * @param pose The assumed robot pose
   * @param absolutePoseMeasurements A list to which all compatible measurements are added
   */
  void registerAbsolutePoseMeasurements(const Pose2f& pose, std::vector<RegisteredAbsolutePoseMeasurement>& absolutePoseMeasurements);

  /**
   * Determine, if an absolute pose measurements (center circle with line, penalty area ...)
   * is compatible (given some thresholds) to the assumed robot pose
   * @param pose The assumed robot pose
   * @param measurement The absolute pose measurement
   * @param absolutePoseMeasurements A list to which all compatible measurements are added. The measurement argument will be added, if it fits.
   */
  void registerSingleAbsolutePoseMeasurement(const Pose2f& pose, const FieldFeature& measurement, std::vector<RegisteredAbsolutePoseMeasurement>& absolutePoseMeasurements);

  /**
  * A FieldFeature is - in most cases - ambiguous. This functions tries to find the
  * most likely of the two possibilities.
  * @param robotPose The assumed current robot pose
  * @param fieldFeature The absolute pose measurement / field feature representation
  * @param pickedPose A reference to a post. The more likely of the two hypotheses will be assigned to it.
  * @return true, if one the two hypotheses appears to be more likely than the other one. false, if not.
  */
  bool pickPoseFromFieldFeature(const Pose2f& robotPose, const FieldFeature& fieldFeature, Pose2f& pickedPose) const;

  /**
   * Determine which landmarks (penalty mark, center circle, ...)
   * are compatible (given some thresholds) to the assumed robot pose
   * @param pose The assumed robot pose
   * @param landmarks A list to which all compatible measurements are added
   */
  void registerLandmarks(const Pose2f& pose, std::vector<RegisteredLandmark>& landmarks);

  /**
   * Determine which field lines
   * are compatible (given some thresholds) to the assumed robot pose
   * @param pose The assumed robot pose
   * @param lines A list to which all compatible measurements are added
   * @param numberOfIgnoredLines The number of lines that appear to be on the center circle and should be ignored completely
   */
  void registerLines(const Pose2f& pose, std::vector<RegisteredLine>& lines, int& numberOfIgnoredLines);

  /**
   * Determine, if the perceived penalty mark matches one of the two marks on the
   * field. If this is the case, the matching mark is returned.
   * @param pose The assumed pose of the robot
   * @param penaltyMarkPercept The position of the perceived penalty mark (in robot coordinates)
   * @param penaltyMarkWorldModel The position of the real penalty mark (in field coordinates)
   */
  bool getCorrespondingPenaltyMark(const Pose2f& pose, const Vector2f& penaltyMarkPercept, Vector2f& penaltyMarkWorldModel) const;

  /**
   * Determine, if the perceived goal post matches one of the four goal posts on the
   * field. If this is the case, the matching post is returned.
   * @param pose The assumed pose of the robot
   * @param goalPostPercept The position of the perceived goal post (in robot coordinates)
   * @param goalPostWorldModel The position of the real goal post (in field coordinates)
   */
  bool getCorrespondingGoalPost(const Pose2f& pose, const Vector2f& goalPostPercept, Vector2f& goalPostWorldModel) const;

  /**
   * Determine, if the perceived field line intersection matches one of the intersections on the
   * field. Position as well as orientation (in steps of 90 degrees) are checked.
   * If this is the case, the matching intersection is returned.
   * @param pose The assumed pose of the robot
   * @param intersectionPercept The position of the perceived intersection (in robot coordinates)
   * @param intersectionWorldModel The position of the real intersection (in field coordinates)
  */
  bool getCorrespondingIntersection(const Pose2f& pose, const FieldLineIntersections::Intersection& intersectionPercept, Vector2f& intersectionWorldModel) const;

  /**
   * L and T intersections are stored in lists depending on their direction (0,90,180,270) on the field.
   * This function maps the perceived continuous direction to one of these sections.
   * @param pose The assumed pose of the robot
   * @param dir The direction of the intersection (in robot coordinates)
   * @return The section (0,90,180,270) to which the intersection direction belongs.
   */
  int intersectionDirectionTo90DegreeSection(const Pose2f& pose, const Vector2f& dir) const;

  /**
   * Determine, if the perceived field line  matches one of the lines on the field.
   * If this is the case, a pointer to the line entry in the world model is returned.
   * @param pose The assumed pose of the robot
   * @param start The start of the perceived line (in robot coordinates)
   * @param end The end of the perceived line (in robot coordinates)
   * @param lineLength The length of the line
   * @param isPartOfCenterCircle Set to true by this function, if the given line appears to be on the center circle.
   * @return A pointer to the line entry in the world model (if a normal line was found) or a nullptr (otherwise). Special case: If the line is on the center circle, nullptr is returned, too, but isPartOfCenterCircle is set to true
  */
  const WorldModelFieldLine* getPointerToCorrespondingLineInWorldModel(const Pose2f& pose, const Vector2f& start, const Vector2f& end, float lineLength, bool& isPartOfCenterCircle) const;

  /**
   * Checks, if a given line is probably on the center circle
   * @param start The start point of the line (in global field coordinates)
   * @param end The end point of the line (in global field coordinates)
   * @param dir The direction of the line (redundant information, but the direction is computed anyway before calling this function)
   * @param lineLength The length of the line
   * @return true, if the line seems to be a part of the center circle. false, if not.
   */
  bool lineShouldBeMatchedWithCenterCircle(const Vector2f& start, const Vector2f& end, const Vector2f& dir, float lineLength) const;

  /**
   * Determine the distance of a point to a line segment.
   * @param base The base of the line
   * @param dir The direction of the line
   * @param length The length of the line
   * @param point The point from which the distance to the line is computed
   * @return The distance from the point to the line
  */
  float getSqrDistanceToLineSegment(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const;
};
